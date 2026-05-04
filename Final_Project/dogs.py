#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
# -*- coding: utf-8 -*-
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
import subprocess

sdk_folder = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib/python2.7/site-packages"
sys.path.append(sdk_folder)
os.environ["DYLD_LIBRARY_PATH"] = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib"

from naoqi import ALProxy

# ── Robot names (4 soccer players) ────────────────────────────────────────────
ROBOT_NAMES = {
    1: "Ronnie",
    2: "Ozil",
    3: "Suarez",
    4: "Messi",
}

# ── Known robot IPs (update with your 4 NAO 6 IPs) ─────────────────────────────
ROBOT_IPS = [
    "172.16.0.8",      # Ronnie
    "172.16.0.144",      # Ozil
    "172.16.0.29",      # Suarez
    "172.16.0.6",      # Messi
]
ROBOT_PORT = 9559

# Random assignment by default; set ROBOT_ID=1..4 to debug one NAO deterministically.
MY_ROBOT_ID = int(os.environ.get("ROBOT_ID", random.randint(1, 4)))
if MY_ROBOT_ID not in ROBOT_NAMES:
    raise ValueError("ROBOT_ID must be 1, 2, 3, or 4")
MY_ROBOT_IP = os.environ.get("ROBOT_IP", ROBOT_IPS[MY_ROBOT_ID - 1])
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
# REMOVED: sonar-based avoidance replaced with vision-based collision avoidance

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

# ── Vision thresholds ─────────────────────────────────────────────────────────
# Color ranges for ALColorBlobDetection (HSV in NAO format)
# Yellow (goals): H:30-45, S:100-255, V:100-255
# White (robots): H:0-360, S:0-50, V:200-255
GOAL_MIN_SIZE = int(os.environ.get("GOAL_MIN_SIZE", "50"))
GOAL_DEBUG = int(os.environ.get("GOAL_DEBUG", "1"))


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
        self._asr_ready     = False  # track if ASR setup succeeded
        self._last_goal_debug = 0

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
        self._wall_detected = False   # desk/wall at frame edge

        print("Connecting to NAO {}...".format(MY_ROBOT_ID))
        try:
            self.motion  = ALProxy("ALMotion",            MY_ROBOT_IP, ROBOT_PORT)
            self.posture = ALProxy("ALRobotPosture",      MY_ROBOT_IP, ROBOT_PORT)
            self.tts     = ALProxy("ALTextToSpeech",      MY_ROBOT_IP, ROBOT_PORT)
            self.memory  = ALProxy("ALMemory",            MY_ROBOT_IP, ROBOT_PORT)
            self.asr     = ALProxy("ALSpeechRecognition", MY_ROBOT_IP, ROBOT_PORT)
            self.ball    = ALProxy("ALRedBallDetection",  MY_ROBOT_IP, ROBOT_PORT)
            self.video   = ALProxy("ALVideoDevice",       MY_ROBOT_IP, ROBOT_PORT)
            print("Robot {} connected.".format(MY_ROBOT_ID))
        except Exception as e:
            print("Connection failed: {}".format(e))
            sys.exit(1)

    # ── Setup / teardown ──────────────────────────────────────────────────────

    def setup(self):
        print("[SETUP] Waking up...")
        sys.stdout.flush()
        try:
            self.motion.wakeUp()
        except Exception as e:
            print("WARNING: wakeUp failed: {}".format(e))
        
        print("[SETUP] Setting stiffness...")
        sys.stdout.flush()
        try:
            self.motion.setStiffnesses("Body", 1.0)
        except Exception as e:
            print("WARNING: setStiffnesses failed: {}".format(e))
        
        print("[SETUP] Moving to StandInit...")
        sys.stdout.flush()
        try:
            self.posture.goToPosture("StandInit", 0.6)
        except Exception as e:
            print("WARNING: goToPosture failed: {}".format(e))

        print("[SETUP] Configuring ASR...")
        sys.stdout.flush()
        try:
            self.asr.unsubscribe("Soccer2v2")
        except Exception:
            pass
        
        try:
            self.asr.stop()
        except Exception:
            pass
        
        try:
            self.asr.pause(True)
            self.asr.setLanguage("English")
            self.asr.setVocabulary(VOCABULARY, False)
            self.asr.subscribe("Soccer2v2")
            self.asr.pause(False)
            self._asr_ready = True
        except Exception as e:
            self._asr_ready = False
            print("ERROR: ASR setup failed: {}".format(e))
            raise RuntimeError("ASR setup failed")

        print("[SETUP] Subscribing to ball detection...")
        sys.stdout.flush()
        # Ball detection via red ball module
        try:
            self.ball.subscribe("Soccer2v2")
        except Exception as e:
            print("WARNING: Could not subscribe to red ball detection: {}".format(e))

        print("[SETUP] Configuring color blob detection...")
        sys.stdout.flush()
        # Color blob detection for goals, robots, and walls (built-in NAO module)
        try:
            if not hasattr(self, 'color_blob'):
                self.color_blob = ALProxy("ALColorBlobDetection", MY_ROBOT_IP, ROBOT_PORT)
            self._setup_color_blob_detection()
        except Exception as e:
            print("WARNING: Could not setup color blob detection: {}".format(e))

        print("[SETUP] Announcing readiness...")
        sys.stdout.flush()
        self._announce("Robot {} Team {} ready!".format(MY_ROBOT_ID, MY_TEAM))

    def shutdown(self):
        self._running = False
        try:
            self.motion.stopMove()
        except Exception:
            pass
        for proxy in (self.asr, self.ball):
            if proxy is not None:
                try:
                    proxy.unsubscribe("Soccer2v2")
                except Exception:
                    pass
        try:
            self.asr.pause(True)
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

    def _read_ball(self):
        try:
            data = self.memory.getData("redBallDetected")
            if data and len(data) > 1 and len(data[1]) > 0:
                b = data[1][0]
                return float(b[0]), float(b[2])
        except Exception:
            pass
        return None
    def _setup_color_blob_detection(self):
        """Configure ALColorBlobDetection for white (robots) and yellow (goals)."""
        try:
            if not hasattr(self, 'color_blob'):
                self.color_blob = ALProxy("ALColorBlobDetection", MY_ROBOT_IP, ROBOT_PORT)
            # Configure detection for white blobs (robots)
            # White = low saturation, high value
            self.color_blob.setColorSpace(0)  # Use HSV color space
        except Exception:
            pass

    def _detect_robots(self):
        """Detect white blobs (other robots) using ALColorBlobDetection.
        Returns list of (distance_m, azimuth_rad) tuples."""
        try:
            blobs = self.memory.getData("ColorBlobDetection/blobs")
            if not blobs or len(blobs) == 0:
                return []
            
            robots = []
            # ALColorBlobDetection blob format: (cx, cy, width, height, x_angle, y_angle, distance)
            for blob in blobs:
                if len(blob) >= 7:
                    cx, cy, w, h, x_ang, y_ang, dist = blob[0], blob[1], blob[2], blob[3], blob[4], blob[5], blob[6]
                    # Filter by distance to identify robots (0.1-3m range)
                    if 0.1 < dist < 3.0:
                        robots.append((float(dist), float(x_ang)))
            return robots
        except Exception:
            return []

    def _debug_goal_blobs(self, blobs, accepted, rejected):
        if not GOAL_DEBUG:
            return
        now = time.time()
        if now - self._last_goal_debug < 1.0:
            return
        self._last_goal_debug = now
        print("[GOAL] raw={} accepted={} rejected={}".format(
            len(blobs), accepted, rejected))
        for blob in blobs[:3]:
            try:
                if len(blob) >= 7:
                    cx, cy, w, h, x_ang, y_ang, dist = (
                        blob[0], blob[1], blob[2], blob[3],
                        blob[4], blob[5], blob[6]
                    )
                    print("[GOAL] raw blob center=({:.0f}, {:.0f}) size=({:.0f}, {:.0f}) area={:.0f} angle=({:.2f}, {:.2f}) dist={:.2f}".format(
                        float(cx), float(cy), float(w), float(h),
                        float(w) * float(h), float(x_ang),
                        float(y_ang), float(dist)))
            except Exception:
                print("[GOAL] raw blob {}".format(blob))
        sys.stdout.flush()

    def _detect_goals(self):
        """Detect yellow blobs (goals) using ALColorBlobDetection.
        Returns list of (cx, cy, area, x_angle, distance_m) sorted left-to-right."""
        try:
            blobs = self.memory.getData("ColorBlobDetection/blobs")
            if not blobs or len(blobs) == 0:
                return []
            
            goals = []
            rejected = 0
            # Filter blobs for yellow-ish colors (soccer goal boundaries)
            for blob in blobs:
                if len(blob) >= 7:
                    cx, cy, w, h, x_ang, y_ang, dist = blob[0], blob[1], blob[2], blob[3], blob[4], blob[5], blob[6]
                    area = w * h
                    if 0.15 < dist < 6.0 and area >= GOAL_MIN_SIZE:
                        goals.append((cx, cy, area, x_ang, dist))
                    else:
                        rejected += 1
            self._debug_goal_blobs(blobs, len(goals), rejected)
            
            # Sort by x position (left to right)
            return sorted(goals, key=lambda g: g[0])
        except Exception:
            return []

    def _detect_wall_edge(self):
        """Check if wall/boundary is close (no blob detection needed).
        Return True if robot is near field edge (stop before collision)."""
        try:
            # Simple approach: if any blob is at extreme distance/angle, wall is close
            blobs = self.memory.getData("ColorBlobDetection/blobs")
            if blobs and len(blobs) > 0:
                for blob in blobs:
                    if len(blob) >= 7:
                        dist = blob[6]
                        # If any blob is very close (< 0.3m) horizontally, wall detected
                        if dist < 0.3:
                            return True
            return False
        except Exception:
            return False
    # ── Vision thread ─────────────────────────────────────────────────────────

    def _vision_thread(self):
        """Monitor color blobs for robots, goals, and walls."""
        self._setup_color_blob_detection()
        
        while self._running:
            try:
                # Detect robots (white blobs)
                robots = self._detect_robots()
                self._other_robots = robots  # Update state
                
                # Detect goals (yellow blobs)
                goals = self._detect_goals()
                if len(goals) >= 2:
                    self._goal_left = goals[0][:2]
                    self._goal_right = goals[-1][:2]
                elif len(goals) == 1:
                    # Only one goal visible; assume center and reflect
                    cx, cy, area, x_ang, dist = goals[0]
                    self._goal_left = (640 - cx, cy)
                    self._goal_right = (cx, cy)
                
                # Detect wall/boundary
                self._wall_detected = self._detect_wall_edge()
            except Exception as e:
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
        while self._running:
            if self._get_game_state() != GAME_PLAYING:
                time.sleep(POLL_SEC)
                continue

            if self._get_motion_state() != S_WALKING:
                time.sleep(POLL_SEC)
                continue

            ball = self._read_ball()
            strategy, target_azi, target_speed = self._decide_strategy(ball)

            # Update operative
            self._set_operative(strategy)

            # ── Strategy-driven movement ──────────────────────────────────────
            if strategy == "kick":
                self.motion.stopMove()
                self._announce("Scoring!", priority=True)
                self.posture.goToPosture("StandInit", 0.8)
                time.sleep(1.2)
                self._set_motion_state(S_STOPPED)
                continue

            # Execute movement toward target with steering
            if target_speed > 0:
                self.motion.moveToward(target_speed, 0.0, target_azi)
            else:
                self.motion.stopMove()

            time.sleep(POLL_SEC)

    def _get_game_state(self):
        with self._lock:
            return self._game_state

    def _set_game_state(self, s):
        with self._lock:
            self._game_state = s

    # ── Calibration ───────────────────────────────────────────────────────────

    def _calibrate_goals(self):
        # Goals are assumed to be at field boundaries; no visual detection needed
        self._announce("Goals ready!")
        return True

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
        try:
            self.setup()

            # Start vision thread (uses built-in ALColorBlobDetection, always available)
            vt = threading.Thread(target=self._vision_thread, name="vision")
            vt.daemon = True
            vt.start()

            vot = threading.Thread(target=self._voice_thread, name="voice")
            vot.daemon = True
            vot.start()

            if not self._calibrate_goals():
                raise RuntimeError("goal calibration failed")

            self._lineup()

            print("Waiting for 'go' command...")
            while self._running and self._get_motion_state() != S_WALKING:
                time.sleep(0.5)
            
            self._set_game_state(GAME_PLAYING)
            self._announce("Game on!")

            mt = threading.Thread(target=self._movement_loop, name="move")
            mt.daemon = True
            mt.start()

            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nCtrl-C received.")
        except Exception as e:
            print("[FATAL] {}".format(e))
        finally:
            self._set_game_state(GAME_INIT)
            self.shutdown()


if __name__ == "__main__":
    # Support launching all four robot controllers from one host.
    # Usage: ./dogs.py --all    -> spawns 4 child processes, one per robot
    if len(sys.argv) > 1 and sys.argv[1] == "--all":
        # Prevent children from re-spawning
        if os.environ.get("RUNNING_CHILD"):
            SoccerBot2v2().run()
        else:
            print("Spawning 4 robot processes...")
            sys.stdout.flush()
            script = os.path.abspath(__file__)
            for rid in range(1, 5):
                env = os.environ.copy()
                env["ROBOT_ID"] = str(rid)
                # ensure ROBOT_IP matches our configured list unless overridden
                try:
                    env["ROBOT_IP"] = ROBOT_IPS[rid - 1]
                except Exception:
                    pass
                env["RUNNING_CHILD"] = "1"
                # Start detached child process
                subprocess.Popen([sys.executable, script], env=env)
                print("  started robot {}".format(rid))
            print("All robot processes launched.")
            sys.exit(0)
    else:
        SoccerBot2v2().run()
