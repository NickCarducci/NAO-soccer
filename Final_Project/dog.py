#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
"""
NAO 1v1 Soccer
Two robots, one red ball, two neon-yellow goals, one field.

Vision detection:
  - White blob detection (opponent NAO body + joints)
  - Neon yellow blob detection (goal rims)
  - Red ball via ALRedBallDetection

Strategy:
  - CHARGE: we're closer to ball than opponent -> pursue ball
  - STALK: opponent is closer -> follow opponent, prepare to intercept
  - TRAP: ball is moving -> steer to predicted ball path
  - DEFEND: ball stopped, opponent has control -> position opposite to goal
  - ATTACK: ball stopped, we have initiative -> charge goal

Startup:
  - Calibrate goal positions (scan field for neon yellow)
  - Line up 1v1 center-field
  - Press to start
"""
import os
import sys
import time
import threading
import math

sdk_folder = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib/python2.7/site-packages"
sys.path.append(sdk_folder)
os.environ["DYLD_LIBRARY_PATH"] = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib"

from naoqi import ALProxy
import cv2
import numpy as np

# ── Connection ────────────────────────────────────────────────────────────────
ROBOT_IP   = "172.16.0.29"
ROBOT_PORT = 9559

# ── Sonar thresholds (metres) ─────────────────────────────────────────────────
WARN_DIST    = 0.80
OBS_DIST     = 0.55
DANGER_DIST  = 0.32

# ── Walking velocities (normalised 0-1) ───────────────────────────────────────
WALK_VX    = 0.55
ARC_THETA  = 0.12
TURN_THETA = 0.75

# ── Ball seeking ──────────────────────────────────────────────────────────────
KICK_DIST = 0.25

# ── Voice recognition ─────────────────────────────────────────────────────────
VOCABULARY       = ["go", "stop", "fetch"]
VOICE_CONFIDENCE = 0.38
VOICE_POLL_SEC   = 0.15

# ── Control loop ──────────────────────────────────────────────────────────────
POLL_SEC = 0.10
VISION_POLL_SEC = 0.20

# ── NAO 6 dimensions (metres) ─────────────────────────────────────────────────
NAO_HEIGHT = 0.573   # metres
NAO_WIDTH  = 0.305   # shoulder-to-shoulder

# ── Camera intrinsics (top camera) ────────────────────────────────────────────
CAMERA_RES_W = 640.0
CAMERA_RES_H = 480.0
CAMERA_FOV_H = math.radians(60.97)  # horizontal FOV in radians
CAMERA_FOCAL_LENGTH = (CAMERA_RES_W / 2.0) / math.tan(CAMERA_FOV_H / 2.0)
# White NAO: high V, low S
WHITE_HSV_LOWER = np.array([0, 0, 180])
WHITE_HSV_UPPER = np.array([180, 50, 255])

# Neon yellow goal: H ~50-65, high S, high V
YELLOW_HSV_LOWER = np.array([40, 100, 100])
YELLOW_HSV_UPPER = np.array([70, 255, 255])

# Red ball: two ranges to catch red wrap-around in HSV
RED_HSV_LOWER1 = np.array([0, 100, 100])
RED_HSV_UPPER1 = np.array([10, 255, 255])
RED_HSV_LOWER2 = np.array([170, 100, 100])
RED_HSV_UPPER2 = np.array([180, 255, 255])

# ── States ────────────────────────────────────────────────────────────────────
S_STOPPED = "stopped"
S_WALKING = "walking"

# ── Game states ───────────────────────────────────────────────────────────────
GAME_INIT     = "init"      # calibrating goals
GAME_LINEUP   = "lineup"    # waiting at center
GAME_PLAYING  = "playing"


def _open_side(left_m, right_m):
    """Return +1.0 (turn left) or -1.0 (turn right) toward the clearer side."""
    return 1.0 if left_m > right_m else -1.0


def _blob_centroid(mask, estimate_distance=False):
    """Find largest blob in mask, return (x, y, area) or (x, y, distance_m).

    If estimate_distance=True, estimates distance from blob height using NAO dimensions.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < 50:
        return None
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    if estimate_distance:
        # Get bounding rect
        x, y, w, h = cv2.boundingRect(largest)
        if h < 10:
            return None
        # distance = (known_height * focal_length) / bbox_height_pixels
        distance_m = (NAO_HEIGHT * CAMERA_FOCAL_LENGTH) / float(h)
        return cx, cy, distance_m
    else:
        return cx, cy, area


class Soccer1v1(object):

    def __init__(self):
        self._motion_state  = S_STOPPED
        self._game_state    = GAME_INIT
        self._lock          = threading.Lock()
        self._running       = True
        self._arc_sign      = 1
        self._last_word     = None
        self._search_theta  = 0.5  # sweep direction for head-scan

        # Vision
        self._camera        = None
        self._opponent_pos  = None   # (x, y, area)
        self._goal_left     = None   # (x, y) field coords
        self._goal_right    = None

        # Ball tracking
        self._ball_pos_prev = None
        self._ball_vel      = None   # (vx, vy) estimated velocity

        print("Connecting to NAO at {}:{}...".format(ROBOT_IP, ROBOT_PORT))
        self.motion  = ALProxy("ALMotion",            ROBOT_IP, ROBOT_PORT)
        self.posture = ALProxy("ALRobotPosture",      ROBOT_IP, ROBOT_PORT)
        self.tts     = ALProxy("ALTextToSpeech",      ROBOT_IP, ROBOT_PORT)
        self.memory  = ALProxy("ALMemory",            ROBOT_IP, ROBOT_PORT)
        self.sonar   = ALProxy("ALSonar",             ROBOT_IP, ROBOT_PORT)
        self.asr     = ALProxy("ALSpeechRecognition", ROBOT_IP, ROBOT_PORT)
        self.ball    = None
        self.video   = ALProxy("ALVideoDevice",       ROBOT_IP, ROBOT_PORT)
        print("Connected.")

    # ── Setup / teardown ──────────────────────────────────────────────────────

    def setup(self):
        self.motion.wakeUp()
        self.motion.setStiffnesses("Body", 1.0)
        self.posture.goToPosture("StandInit", 0.6)

        self.sonar.subscribe("Soccer")
        time.sleep(0.4)

        try:
            self.asr.unsubscribe("Soccer")
        except Exception:
            pass
        self.asr.setLanguage("English")
        self.asr.setVocabulary(VOCABULARY, False)
        self.asr.subscribe("Soccer")

        try:
            self.ball = ALProxy("ALRedBallDetection", ROBOT_IP, ROBOT_PORT)
            self.ball.subscribe("Soccer")
        except Exception as e:
            print("Ball detection unavailable: " + str(e))

        # Camera setup
        try:
            self.video.setActiveCamera(0)  # top camera
            self._camera = self.video
            print("Camera active.")
        except Exception as e:
            print("Camera setup failed: " + str(e))

        self.tts.say("Soccer mode ready.")

    def shutdown(self):
        self._running = False
        try:
            self.motion.stopMove()
        except Exception:
            pass
        for proxy in (self.asr, self.sonar, self.ball):
            if proxy is not None:
                try:
                    proxy.unsubscribe("Soccer")
                except Exception:
                    pass
        try:
            self.posture.goToPosture("Stand", 0.5)
        except Exception:
            pass
        try:
            self.motion.setStiffnesses("Body", 0.0)
        except Exception:
            pass
        print("Shutdown complete.")

    # ── State helpers ─────────────────────────────────────────────────────────

    def _set_motion_state(self, s):
        with self._lock:
            self._motion_state = s

    def _get_motion_state(self):
        with self._lock:
            return self._motion_state

    def _set_game_state(self, s):
        with self._lock:
            self._game_state = s

    def _get_game_state(self):
        with self._lock:
            return self._game_state

    # ── Sensors ───────────────────────────────────────────────────────────────

    def _read_sonar(self):
        try:
            left  = float(self.memory.getData(
                "Device/SubDeviceList/US/Left/Sensor/Value"))
            right = float(self.memory.getData(
                "Device/SubDeviceList/US/Right/Sensor/Value"))
            return left, right
        except Exception:
            return 2.0, 2.0

    def _read_ball(self):
        """Returns (azimuth_rad, distance_m) or None."""
        if self.ball is None:
            return None
        try:
            data = self.memory.getData("redBallDetected")
            if data and len(data) > 1 and len(data[1]) > 0:
                b = data[1][0]
                return float(b[0]), float(b[2])
        except Exception:
            pass
        return None

    def _capture_frame(self):
        """Grab camera frame from NAO video device."""
        try:
            result = self.video.getImageRemote(self._camera)
            if result is None:
                return None
            width, height, channels, imgBuffer = result[0], result[1], result[2], result[6]
            img = np.frombuffer(imgBuffer, dtype=np.uint8).reshape((height, width, channels))
            return cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        except Exception:
            return None

    def _detect_opponent(self, hsv_frame):
        """Find white NAO blob in frame. Return (cx, cy, distance_m) or None."""
        if hsv_frame is None:
            return None
        mask = cv2.inRange(hsv_frame, WHITE_HSV_LOWER, WHITE_HSV_UPPER)
        return _blob_centroid(mask, estimate_distance=True)

    def _detect_goals(self, hsv_frame):
        """Find yellow goal blobs. Return list of (cx, cy, area)."""
        if hsv_frame is None:
            return []
        mask = cv2.inRange(hsv_frame, YELLOW_HSV_LOWER, YELLOW_HSV_UPPER)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        goals = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # large enough
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    goals.append((cx, cy, area))
        return sorted(goals, key=lambda g: g[0])  # sort by x (left to right)

    # ── Vision thread ─────────────────────────────────────────────────────────

    def _vision_thread(self):
        """Detect opponent, goals, update state."""
        while self._running:
            try:
                frame = self._capture_frame()
                if frame is not None:
                    self._opponent_pos = self._detect_opponent(frame)
                    goals = self._detect_goals(frame)
                    if len(goals) >= 2:
                        self._goal_left = goals[0][:2]
                        self._goal_right = goals[-1][:2]
            except Exception:
                pass
            time.sleep(VISION_POLL_SEC)

    # ── Voice thread ──────────────────────────────────────────────────────────

    def _voice_thread(self):
        while self._running:
            try:
                data = self.memory.getData("WordRecognized")
                if data and len(data) >= 2:
                    word, conf = data[0], float(data[1])
                    if word and conf >= VOICE_CONFIDENCE and word != self._last_word:
                        self._last_word = word
                        word = word.lower().strip()
                        print("Heard: '{}'".format(word))
                        if word == "stop":
                            self.tts.post.say("Stopping")
                            self.motion.stopMove()
                            self._set_motion_state(S_STOPPED)
                        elif word in ("go", "fetch"):
                            self.tts.post.say("Going!")
                            self._set_motion_state(S_WALKING)
            except Exception:
                pass
            time.sleep(VOICE_POLL_SEC)

    # ── Game init ─────────────────────────────────────────────────────────────

    def _calibrate_goals(self):
        """Scan for goal positions. Block until both found."""
        self.tts.say("Scanning for goals.")
        while self._running:
            if self._goal_left and self._goal_right:
                self.tts.say("Goals locked. Left and right identified.")
                return True
            time.sleep(0.5)
        return False

    def _lineup(self):
        """Walk to center and wait for start signal."""
        self.tts.say("Lining up center field.")
        self._set_motion_state(S_WALKING)
        # Simple: walk forward slowly, wait for "go" command
        self.motion.moveToward(0.1, 0.0, 0.0)
        while self._running and self._get_motion_state() == S_WALKING:
            time.sleep(0.5)
        self.motion.stopMove()
        self.tts.say("Ready. Say go to start.")
        self._set_motion_state(S_STOPPED)

    # ── Strategy & movement loop ──────────────────────────────────────────────

    def _decide_strategy(self, ball_pos, opponent_pos):
        """
        Strategy based on ball motion and opponent proximity.

        If ball is MOVING (rolling toward goal):
          Both robots intercept predicted trajectory (run to where it's going)

        If ball is STOPPED:
          - Opponent closer: STALK (block perpendicularly between opponent & goal)
          - We're closer:    CHARGE (run toward ball/goal)

        If ball in KICK range: KICK

        opponent_pos = (x, y, distance_m) from camera
        ball_pos = (azimuth_rad, distance_m) from ball detector
        Returns: (strategy_name, target_azimuth, target_speed)
        """
        if ball_pos is None:
            self._ball_pos_prev = None
            self._ball_vel = None
            # SEARCH: pivot in place, owl-style head scan (no forward movement)
            # Sweep theta direction cycles between +0.5 and -0.5
            self._search_theta *= -1 if abs(self._search_theta) > 0.8 else 1.0
            return ("search", self._search_theta, 0.0)  # (strategy, azimuth_for_sweep, speed=0)

        ball_azi, ball_dist = ball_pos

        # Estimate ball velocity (simple: compare to previous frame)
        # If ball distance is changing significantly, it's moving
        ball_moving = False
        if self._ball_pos_prev is not None:
            prev_azi, prev_dist = self._ball_pos_prev
            dist_delta = ball_dist - prev_dist
            # If distance decreased rapidly, ball is moving toward us
            # If distance stayed roughly same but azimuth changed, ball is rolling sideways
            if abs(dist_delta) > 0.05:  # >5cm per frame = moving
                ball_moving = True
                self._ball_vel = (ball_azi - prev_azi, dist_delta)
        self._ball_pos_prev = ball_pos

        # KICK range (always highest priority when in range)
        if ball_dist < KICK_DIST:
            return ("kick", ball_azi, 0.0)

        # Opponent position
        if opponent_pos:
            opp_x, opp_y, opp_dist = opponent_pos
            opp_azi = (opp_x - CAMERA_RES_W / 2.0) / (CAMERA_RES_W / 2.0) * CAMERA_FOV_H / 2.0
        else:
            opp_dist = None
            opp_azi = 0.0

        # Ball MOVING: intercept trajectory (both robots)
        if ball_moving:
            # Predict where ball is going: current position + velocity
            if self._ball_vel:
                pred_azi, pred_dist = self._ball_vel
                target_azi = ball_azi + pred_azi * 0.5  # extrapolate slightly
            else:
                target_azi = ball_azi
            return ("intercept", target_azi, WALK_VX * 0.8)

        # Ball STOPPED: decide based on who's closer
        if opp_dist is not None and opp_dist < ball_dist + 0.3:
            # Opponent is closer/has control: STALK (defensive block)
            # Position perpendicular to opponent->goal line, face opponent
            return ("stalk", opp_azi, WALK_VX * 0.2)

        # We're closer (or no opponent visible): CHARGE toward ball
        return ("charge", ball_azi, WALK_VX)

    def _movement_loop(self):
        """Main soccer player loop."""
        was_avoiding = False

        while self._running:
            game = self._get_game_state()
            if game != GAME_PLAYING:
                time.sleep(POLL_SEC)
                continue

            if self._get_motion_state() != S_WALKING:
                time.sleep(POLL_SEC)
                continue

            left_m, right_m = self._read_sonar()
            min_dist = min(left_m, right_m)

            ball = self._read_ball()
            opponent = self._opponent_pos

            strategy, target_azi, target_speed = self._decide_strategy(ball, opponent)

            # ── Sonar safety: always active ───────────────────────────────────
            if min_dist < DANGER_DIST:
                was_avoiding = True
                steer = _open_side(left_m, right_m)
                self.motion.moveToward(WALK_VX * 0.15, 0.0, steer * TURN_THETA)
                time.sleep(POLL_SEC)
                continue

            if min_dist < OBS_DIST:
                was_avoiding = True
                steer = _open_side(left_m, right_m)
                self.motion.moveToward(WALK_VX * 0.35, 0.0, steer * TURN_THETA * 0.65)
                time.sleep(POLL_SEC)
                continue

            if min_dist < WARN_DIST:
                was_avoiding = True
                steer = _open_side(left_m, right_m)
                blend = 1.0 - (min_dist - OBS_DIST) / (WARN_DIST - OBS_DIST)
                self.motion.moveToward(
                    target_speed * (0.6 + 0.4 * (1.0 - blend)),
                    0.0,
                    steer * ARC_THETA * (1.0 + blend * 3.0)
                )
                time.sleep(POLL_SEC)
                continue

            # ── Strategy-driven movement ──────────────────────────────────────
            if strategy == "kick":
                self.motion.stopMove()
                self.tts.post.say("Kick!")
                self.posture.goToPosture("StandInit", 0.8)
                time.sleep(1.2)
                self._set_motion_state(S_WALKING)
                continue

            # Blend strategy azimuth with gentle wandering arc
            if was_avoiding:
                self._arc_sign *= -1
                was_avoiding = False

            if strategy in ("charge", "stalk", "search"):
                theta = max(-TURN_THETA, min(TURN_THETA, target_azi * 1.5))
            else:
                theta = ARC_THETA * self._arc_sign

            self.motion.moveToward(target_speed, 0.0, theta)
            time.sleep(POLL_SEC)

    # ── Entry point ───────────────────────────────────────────────────────────

    def run(self):
        self.setup()

        # Spin up threads
        vt = threading.Thread(target=self._vision_thread, name="vision")
        vt.daemon = True
        vt.start()

        vot = threading.Thread(target=self._voice_thread, name="voice")
        vot.daemon = True
        vot.start()

        # Game init sequence
        if not self._calibrate_goals():
            self.shutdown()
            return

        self._lineup()

        # Wait for "go" command to enter PLAYING state
        print("Waiting for 'go' command...")
        while self._get_motion_state() != S_WALKING:
            time.sleep(0.5)
        self._set_game_state(GAME_PLAYING)
        print("Game started!")

        # Movement loop
        mt = threading.Thread(target=self._movement_loop, name="move")
        mt.daemon = True
        mt.start()

        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nCtrl-C received.")
        finally:
            self._set_game_state(GAME_INIT)
            self.shutdown()


if __name__ == "__main__":
    Soccer1v1().run()
