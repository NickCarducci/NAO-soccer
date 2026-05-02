#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
"""
NAO 2v2 Soccer with Live Team Communication

Four NAO 6 robots. Robots coordinate via TTS -- they literally call out to teammates.
Teams compete, speech overlaps, natural game cadence emerges.

Operatives:
  CHARGE    -- pursuing ball
  INTERCEPT -- ball moving, intercept trajectory
  STALK     -- defensive block
  MOVE      -- teammate called you open, position for pass
  SUPPORT   -- defending, waiting for teammate to trap ball
  PASS      -- trapping teammate is passing to you
  SHOOT     -- I have possession, lining up shot

Team Assignment: Random from 4 known IPs, IDs 1-4, teams (1,2 vs 3,4).
Communication: TTS broadcast to alert teammates and opponents.
"""
import os
import sys
import time
import threading
import math
import random

sdk_folder = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib/python2.7/site-packages"
sys.path.append(sdk_folder)
os.environ["DYLD_LIBRARY_PATH"] = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib"

from naoqi import ALProxy
import cv2
import numpy as np

# ── Robot names (4 soccer players) ────────────────────────────────────────────
ROBOT_NAMES = {
    1: "Ronnie",
    2: "Ozil",
    3: "Suarez",
    4: "Messi",
}

# ── Known robot IPs (update with your 4 NAO 6 IPs) ─────────────────────────────
ROBOT_IPS = [
    "172.16.0.29",      # Ronnie
    "172.16.0.30",      # Ozil
    "172.16.0.31",      # Suarez
    "172.16.0.32",      # Messi
]
ROBOT_PORT = 9559

# Random assignment: each robot picks an ID from 1-4
MY_ROBOT_ID = random.randint(1, 4)
MY_ROBOT_IP = ROBOT_IPS[MY_ROBOT_ID - 1]
MY_NAME = ROBOT_NAMES[MY_ROBOT_ID]
MY_TEAM = "A" if MY_ROBOT_ID in (1, 2) else "B"
TEAMMATE_ID = 3 - MY_ROBOT_ID if MY_ROBOT_ID <= 2 else 7 - MY_ROBOT_ID
TEAMMATE_NAME = ROBOT_NAMES[TEAMMATE_ID]

print("=" * 60)
print("PLAYER: {}".format(MY_NAME))
print("ROBOT ID: {}".format(MY_ROBOT_ID))
print("IP: {}".format(MY_ROBOT_IP))
print("TEAM: {}".format(MY_TEAM))
print("TEAMMATE: {} (ID {})".format(TEAMMATE_NAME, TEAMMATE_ID))
print("=" * 60)

# ── Sonar thresholds (metres) ─────────────────────────────────────────────────
WARN_DIST    = 0.80
OBS_DIST     = 0.55
DANGER_DIST  = 0.32

# ── Walking velocities ────────────────────────────────────────────────────────
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

# ── NAO 6 dimensions & camera ─────────────────────────────────────────────────
NAO_HEIGHT = 0.573
NAO_WIDTH  = 0.305
CAMERA_RES_W = 640.0
CAMERA_RES_H = 480.0
CAMERA_FOV_H = math.radians(60.97)
CAMERA_FOCAL_LENGTH = (CAMERA_RES_W / 2.0) / math.tan(CAMERA_FOV_H / 2.0)

# ── Vision thresholds (HSV) ───────────────────────────────────────────────────
WHITE_HSV_LOWER = np.array([0, 0, 180])
WHITE_HSV_UPPER = np.array([180, 50, 255])
YELLOW_HSV_LOWER = np.array([40, 100, 100])
YELLOW_HSV_UPPER = np.array([70, 255, 255])
RED_HSV_LOWER1 = np.array([0, 100, 100])
RED_HSV_UPPER1 = np.array([10, 255, 255])
RED_HSV_LOWER2 = np.array([170, 100, 100])
RED_HSV_UPPER2 = np.array([180, 255, 255])

# ── States ────────────────────────────────────────────────────────────────────
S_STOPPED = "stopped"
S_WALKING = "walking"
GAME_INIT     = "init"
GAME_LINEUP   = "lineup"
GAME_PLAYING  = "playing"

# ── Operatives ────────────────────────────────────────────────────────────────
OP_CHARGE     = "charge"
OP_INTERCEPT  = "intercept"
OP_STALK      = "stalk"
OP_MOVE       = "move"       # get open for pass
OP_SUPPORT    = "support"    # defending, ready to support
OP_PASS       = "pass"       # receiving pass
OP_SHOOT      = "shoot"      # I have ball, lining up shot


def _open_side(left_m, right_m):
    return 1.0 if left_m > right_m else -1.0


def _azimuth_to_goal(ball_pos, goal_pos, field_center_azi=0.0):
    """
    Calculate azimuth perpendicular-to-goal from ball.

    To shoot at goal, position perpendicular to goal direction.
    Returns azimuth to move to (positive = left, negative = right).
    """
    if ball_pos is None or goal_pos is None:
        return 0.0
    # Rough: if goal is to the right, shimmy left (perpendicular)
    # This is approximate without full field coordinates
    return -0.3  # left shimmy (can be tuned)


def _azimuth_to_teammate(teammate_robot_pos):
    """
    Calculate azimuth perpendicular-to-teammate from ball.

    To pass to teammate, position perpendicular to them.
    Returns azimuth to move to.
    """
    if teammate_robot_pos is None:
        return 0.0
    dist, azi = teammate_robot_pos
    # Move perpendicular to teammate direction
    return azi * 0.5


def _blob_centroid(mask, estimate_distance=False):
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
        x, y, w, h = cv2.boundingRect(largest)
        if h < 10:
            return None
        distance_m = (NAO_HEIGHT * CAMERA_FOCAL_LENGTH) / float(h)
        return cx, cy, distance_m
    else:
        return cx, cy, area


class SoccerBot2v2(object):

    def __init__(self):
        self._motion_state  = S_STOPPED
        self._game_state    = GAME_INIT
        self._operative     = None
        self._lock          = threading.Lock()
        self._running       = True
        self._arc_sign      = 1
        self._last_word     = None
        self._search_theta  = 0.5
        self._last_callout  = 0  # throttle TTS spam

        # Vision
        self._camera        = None
        self._other_robots  = []   # list of (distance, azimuth) for other robots
        self._goal_left     = None
        self._goal_right    = None
        self._ball_pos_prev = None
        self._ball_vel      = None
        self._ball_moving   = False
        self._possession    = None  # which robot ID has ball
        self._contested     = False  # 2+ robots in close range
        self._shooting      = False  # in shimmy/positioning phase
        self._shimmy_target = "goal"  # "goal" or "teammate"

        print("Connecting to NAO {}...".format(MY_ROBOT_ID))
        try:
            self.motion  = ALProxy("ALMotion",            MY_ROBOT_IP, ROBOT_PORT)
            self.posture = ALProxy("ALRobotPosture",      MY_ROBOT_IP, ROBOT_PORT)
            self.tts     = ALProxy("ALTextToSpeech",      MY_ROBOT_IP, ROBOT_PORT)
            self.memory  = ALProxy("ALMemory",            MY_ROBOT_IP, ROBOT_PORT)
            self.sonar   = ALProxy("ALSonar",             MY_ROBOT_IP, ROBOT_PORT)
            self.asr     = ALProxy("ALSpeechRecognition", MY_ROBOT_IP, ROBOT_PORT)
            self.ball    = ALProxy("ALRedBallDetection",  MY_ROBOT_IP, ROBOT_PORT)
            self.video   = ALProxy("ALVideoDevice",       MY_ROBOT_IP, ROBOT_PORT)
            print("Robot {} connected.".format(MY_ROBOT_ID))
        except Exception as e:
            print("Connection failed: {}".format(e))
            sys.exit(1)

    # ── Setup / teardown ──────────────────────────────────────────────────────

    def setup(self):
        self.motion.wakeUp()
        self.motion.setStiffnesses("Body", 1.0)
        self.posture.goToPosture("StandInit", 0.6)

        self.sonar.subscribe("Soccer2v2")
        time.sleep(0.4)

        try:
            self.asr.unsubscribe("Soccer2v2")
        except Exception:
            pass
        self.asr.setLanguage("English")
        self.asr.setVocabulary(VOCABULARY, False)
        self.asr.subscribe("Soccer2v2")

        try:
            self.ball.subscribe("Soccer2v2")
        except Exception:
            pass

        try:
            self.video.setActiveCamera(0)
            self._camera = self.video
        except Exception:
            pass

        self._announce("Robot {} Team {} ready!".format(MY_ROBOT_ID, MY_TEAM))

    def shutdown(self):
        self._running = False
        try:
            self.motion.stopMove()
        except Exception:
            pass
        for proxy in (self.asr, self.sonar, self.ball):
            if proxy is not None:
                try:
                    proxy.unsubscribe("Soccer2v2")
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

    # ── State helpers ─────────────────────────────────────────────────────────

    def _set_motion_state(self, s):
        with self._lock:
            self._motion_state = s

    def _get_motion_state(self):
        with self._lock:
            return self._motion_state

    def _set_operative(self, op):
        with self._lock:
            self._operative = op

    def _get_operative(self):
        with self._lock:
            return self._operative

    # ── Communication ─────────────────────────────────────────────────────────

    def _announce(self, msg, priority=False):
        """Robot speaks to alert teammates and opponents."""
        now = time.time()
        if not priority and (now - self._last_callout) < 0.5:  # throttle
            return
        self._last_callout = now
        try:
            self.tts.post.say(msg)
        except Exception:
            pass

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
        try:
            data = self.memory.getData("redBallDetected")
            if data and len(data) > 1 and len(data[1]) > 0:
                b = data[1][0]
                return float(b[0]), float(b[2])
        except Exception:
            pass
        return None

    def _capture_frame(self):
        try:
            result = self.video.getImageRemote(self._camera)
            if result is None:
                return None
            width, height, channels, imgBuffer = result[0], result[1], result[2], result[6]
            img = np.frombuffer(imgBuffer, dtype=np.uint8).reshape((height, width, channels))
            return cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        except Exception:
            return None

    def _detect_robots(self, hsv_frame):
        """Detect all white blobs (other robots). Return list of (cx, cy, distance_m)."""
        if hsv_frame is None:
            return []
        mask = cv2.inRange(hsv_frame, WHITE_HSV_LOWER, WHITE_HSV_UPPER)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        robots = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 50:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    x, y, w, h = cv2.boundingRect(contour)
                    if h > 10:
                        dist = (NAO_HEIGHT * CAMERA_FOCAL_LENGTH) / float(h)
                        robots.append((cx, cy, dist))
        return robots

    def _detect_goals(self, hsv_frame):
        if hsv_frame is None:
            return []
        mask = cv2.inRange(hsv_frame, YELLOW_HSV_LOWER, YELLOW_HSV_UPPER)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        goals = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    goals.append((cx, cy, area))
        return sorted(goals, key=lambda g: g[0])

    # ── Vision thread ─────────────────────────────────────────────────────────

    def _vision_thread(self):
        while self._running:
            try:
                frame = self._capture_frame()
                if frame is not None:
                    # Detect all robots (other NAOs)
                    robots = self._detect_robots(frame)
                    self._other_robots = []
                    for cx, cy, dist in robots:
                        azi = (cx - CAMERA_RES_W / 2.0) / (CAMERA_RES_W / 2.0) * CAMERA_FOV_H / 2.0
                        self._other_robots.append((dist, azi))

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

                        # Human commands
                        if word == "stop":
                            self._announce("Stopping!")
                            self.motion.stopMove()
                            self._set_motion_state(S_STOPPED)
                        elif word in ("go", "fetch"):
                            self._announce("Let's go Team {}!".format(MY_TEAM))
                            self._set_motion_state(S_WALKING)

                        # Team callouts from other robots
                        # Listen for "{NAME}, get open" or "{NAME}, move"
                        if MY_NAME.lower() in word:
                            if "move" in word or "open" in word:
                                self._announce("{}! Getting open!".format(MY_NAME))
                                self._set_operative(OP_MOVE)
                            elif "pass" in word:
                                self._announce("{} got it!".format(MY_NAME))
                                self._set_operative(OP_PASS)
            except Exception:
                pass
            time.sleep(VOICE_POLL_SEC)

    # ── Strategy ──────────────────────────────────────────────────────────────

    def _decide_strategy(self, ball_pos):
        """
        2v2 strategy with team coordination.

        Ball MOVING: all robots INTERCEPT
        Ball STOPPED:
          - If 2+ robots within 0.5m of closest: CONTESTED, all go SUPPORT (defend)
          - If we're closest: CHARGE toward ball
          - If teammate called us (we're in MOVE): reposition for pass
        """
        if ball_pos is None:
            self._ball_pos_prev = None
            self._ball_vel = None
            self._ball_moving = False
            self._search_theta *= -1 if abs(self._search_theta) > 0.8 else 1.0
            return (OP_SUPPORT, self._search_theta, 0.0)

        ball_azi, ball_dist = ball_pos

        # Track ball motion
        self._ball_moving = False
        if self._ball_pos_prev is not None:
            prev_azi, prev_dist = self._ball_pos_prev
            dist_delta = ball_dist - prev_dist
            if abs(dist_delta) > 0.05:
                self._ball_moving = True
                self._ball_vel = (ball_azi - prev_azi, dist_delta)
        self._ball_pos_prev = ball_pos

        # KICK range
        if ball_dist < KICK_DIST:
            return ("kick", ball_azi, 0.0)

        # Ball MOVING: both teams intercept
        if self._ball_moving:
            if self._ball_vel:
                pred_azi, _ = self._ball_vel
                target_azi = ball_azi + pred_azi * 0.5
            else:
                target_azi = ball_azi
            return (OP_INTERCEPT, target_azi, WALK_VX * 0.8)

        # Ball STOPPED: analyze robot positioning
        # Count how many robots are in "close contention" (within 0.5m of closest robot)
        if self._other_robots:
            distances = [d for d, a in self._other_robots]
            closest_robot_dist = min(distances)
            contested_count = sum(1 for d in distances if d < closest_robot_dist + 0.5)
        else:
            closest_robot_dist = None
            contested_count = 0

        # If we're in MOVE mode (teammate called us), reposition for pass
        current_op = self._get_operative()
        if current_op == OP_MOVE:
            # Move toward open space, preferably closer to goal
            return (OP_MOVE, ball_azi * 0.5, WALK_VX * 0.6)

        # If contested (2+ robots close), all go SUPPORT/DEFEND
        if contested_count >= 2:
            self._contested = True
            return (OP_SUPPORT, ball_azi, WALK_VX * 0.2)

        self._contested = False

        # If opponent/other robot is closer: STALK (defend)
        if closest_robot_dist is not None and closest_robot_dist < ball_dist + 0.3:
            return (OP_STALK, ball_azi, WALK_VX * 0.2)

        # We're closer: CHARGE toward ball to trap it
        if closest_robot_dist is None or ball_dist < closest_robot_dist:
            # Check if we've already trapped it
            if ball_dist < 0.35:
                self._shooting = True
                # Call teammate "get open"
                if self._contested:
                    self._announce("{}, get open!".format(TEAMMATE_NAME), priority=True)

                # Find teammate in vision
                teammate_pos = None
                if self._other_robots and len(self._other_robots) > 0:
                    teammate_pos = self._other_robots[0]

                # Check if opponent blocks goal shot path
                if self._other_robots:
                    closest_opponent_dist = min([d for d, a in self._other_robots])
                    if closest_opponent_dist < 0.8 and self._shimmy_target == "goal":
                        # Blocker detected → switch to teammate shimmy
                        self._announce("Passing to {}!".format(TEAMMATE_NAME), priority=True)
                        self._shimmy_target = "teammate"

                # Shimmy perpendicular to current target
                if self._shimmy_target == "teammate" and teammate_pos:
                    target_azi = _azimuth_to_teammate(teammate_pos)
                else:
                    target_azi = _azimuth_to_goal(ball_pos, self._goal_right)

                return (OP_SHOOT, target_azi, WALK_VX * 0.3)

            # Not trapped yet, CHARGE toward ball
            self._shimmy_target = "goal"  # reset shimmy target
            self._shooting = False
            return (OP_CHARGE, ball_azi, WALK_VX)

        # We're closer: default CHARGE
        return (OP_CHARGE, ball_azi, WALK_VX)

    # ── Movement loop ─────────────────────────────────────────────────────────

    def _movement_loop(self):
        was_avoiding = False

        while self._running:
            if self._get_game_state() != GAME_PLAYING:
                time.sleep(POLL_SEC)
                continue

            if self._get_motion_state() != S_WALKING:
                time.sleep(POLL_SEC)
                continue

            left_m, right_m = self._read_sonar()
            min_dist = min(left_m, right_m)

            ball = self._read_ball()

            strategy, target_azi, target_speed = self._decide_strategy(ball)

            # Update operative
            self._set_operative(strategy)

            # ── Sonar safety ──────────────────────────────────────────────────
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
                self._announce("Scoring!", priority=True)
                self.posture.goToPosture("StandInit", 0.8)
                time.sleep(1.2)
                self._set_motion_state(S_WALKING)
                continue

            # If we trapped the ball (ball_dist < 0.3m), we're in possession, ready to shoot
        if ball_dist < 0.35 and strategy == OP_CHARGE:
            self._shooting = True
            # Call teammate "get open" in case we need to pass
            if self._contested:
                self._announce("{}, get open!".format(TEAMMATE_NAME), priority=True)

            # Check if opponent is blocking the shot during our shimmy
            if self._other_robots:
                closest_opponent_dist = min([d for d, a in self._other_robots])
                if closest_opponent_dist < 0.8:
                    # Blocker detected during shooting → make the pass
                    self._announce("Passing to {}!".format(TEAMMATE_NAME), priority=True)
                    self._shooting = False
                    return (OP_PASS, ball_azi * 0.3, WALK_VX * 0.4)

            # No blocker, line up the shot (shimmy around ball)
            return (OP_SHOOT, ball_azi * 0.7, WALK_VX * 0.3)

        self._shooting = False

    def _get_game_state(self):
        with self._lock:
            return self._game_state

    def _set_game_state(self, s):
        with self._lock:
            self._game_state = s

    # ── Calibration ───────────────────────────────────────────────────────────

    def _calibrate_goals(self):
        self._announce("Scanning for goals.")
        while self._running:
            if self._goal_left and self._goal_right:
                self._announce("Goals locked!")
                return True
            time.sleep(0.5)
        return False

    def _lineup(self):
        self._announce("Lining up Team {}!".format(MY_TEAM))
        self._set_motion_state(S_WALKING)
        self.motion.moveToward(0.1, 0.0, 0.0)
        while self._running and self._get_motion_state() == S_WALKING:
            time.sleep(0.5)
        self.motion.stopMove()
        self._announce("Ready. Say go to start.")
        self._set_motion_state(S_STOPPED)

    # ── Entry point ───────────────────────────────────────────────────────────

    def run(self):
        self.setup()

        vt = threading.Thread(target=self._vision_thread, name="vision")
        vt.daemon = True
        vt.start()

        vot = threading.Thread(target=self._voice_thread, name="voice")
        vot.daemon = True
        vot.start()

        if not self._calibrate_goals():
            self.shutdown()
            return

        self._lineup()

        print("Waiting for 'go' command...")
        while self._get_motion_state() != S_WALKING:
            time.sleep(0.5)
        self._set_game_state(GAME_PLAYING)
        self._announce("Game on!")

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
    SoccerBot2v2().run()
