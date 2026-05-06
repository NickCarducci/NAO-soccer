#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
"""
NAO 1v1 Soccer, recovered from the original two-player game.
Two robots, one red ball, two neon-yellow goals, one field.

Vision detection:
  - Built-in color blob detection for opponent/goal/wall hints
  - Red ball via ALRedBallDetection

Strategy:
  - CHARGE: we're closer to ball than opponent -> pursue ball
            # Optional diagnostic dump to help find where ALColorBlobDetection
            # publishes into ALMemory. Set DUMP_ALMEMORY_KEYS=1 in the robot
            # environment to enable this short, safe probe.
            if os.environ.get("DUMP_ALMEMORY_KEYS") == "1":
                try:
                    self._dump_color_blob_diagnostics()
                except Exception:
                    pass
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
import colorsys
import subprocess
import signal

sdk_folder = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib/python2.7/site-packages"
sys.path.append(sdk_folder)
os.environ["DYLD_LIBRARY_PATH"] = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib"

from naoqi import ALProxy
try:
    import cv2
    import numpy as np
except Exception:
    cv2 = None
    np = None


class _PrefixedStream(object):
    """Prefix each emitted line so multi-robot logs remain readable."""

    def __init__(self, stream, prefix):
        self._stream = stream
        self._prefix = prefix
        self._at_line_start = True

    def write(self, data):
        if data is None:
            return
        text = str(data)
        if not text:
            return
        for part in text.splitlines(True):
            if self._at_line_start:
                self._stream.write(self._prefix)
                self._at_line_start = False
            self._stream.write(part)
            if part.endswith("\n"):
                self._at_line_start = True

    def flush(self):
        self._stream.flush()

    def isatty(self):
        try:
            return self._stream.isatty()
        except Exception:
            return False

    def fileno(self):
        return self._stream.fileno()


def _install_robot_log_prefix(robot_id, robot_name):
    prefix = "[R{}:{}] ".format(robot_id, robot_name)
    if not isinstance(sys.stdout, _PrefixedStream):
        sys.stdout = _PrefixedStream(sys.stdout, prefix)
    if not isinstance(sys.stderr, _PrefixedStream):
        sys.stderr = _PrefixedStream(sys.stderr, prefix)

# Connection
ROBOT_NAMES = {
    1: "Suarez",
    2: "Messi",
}

ROBOT_IPS = {
    1: "172.16.0.4",
    2: "172.16.0.29",
}

def _launch_both_players():
    """Start one child controller process per 1v1 robot."""
    script = os.path.abspath(__file__)
    children = []

    print("=" * 60)
    print("Starting 1v1 controllers:")
    for robot_id in sorted(ROBOT_IPS):
        print("  ID {}: {} at {}".format(
            robot_id, ROBOT_NAMES[robot_id], ROBOT_IPS[robot_id]))
    print("=" * 60)
    sys.stdout.flush()

    for robot_id in sorted(ROBOT_IPS):
        env = os.environ.copy()
        env["ROBOT_ID"] = str(robot_id)
        env["DOGS_1V1_CHILD"] = "1"
        env["PYTHONUNBUFFERED"] = "1"
        # Pass the configured ROBOT_IP into the child process
        robot_ip = ROBOT_IPS[robot_id]
        env["ROBOT_IP"] = robot_ip
        children.append(subprocess.Popen([sys.executable, script], env=env))

    try:
        while True:
            running = [p for p in children if p.poll() is None]
            if not running:
                # All children exited
                exit_codes = [p.returncode or 0 for p in children]
                return max(exit_codes) if exit_codes else 0
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopping both 1v1 controllers...")
        # First, try to send SIGINT so child processes can shut down cleanly.
        for child in children:
            try:
                if child.poll() is None:
                    os.kill(child.pid, signal.SIGINT)
            except Exception:
                pass
        # Give children a short grace period to exit cleanly.
        time.sleep(1.0)
        # If any are still running, try terminate (SIGTERM), then kill (SIGKILL).
        for child in children:
            try:
                if child.poll() is None:
                    child.terminate()
            except Exception:
                pass
        time.sleep(1.0)
        for child in children:
            try:
                if child.poll() is None:
                    child.kill()
            except Exception:
                pass
        return 130


if (__name__ == "__main__" and
        "ROBOT_ID" not in os.environ and
        os.environ.get("DOGS_1V1_CHILD") != "1"):
    sys.exit(_launch_both_players())


# Set ROBOT_ID=1..2 to choose a single 1v1 player, or run without ROBOT_ID
# to launch both players.
ROBOT_ID = int(os.environ.get("ROBOT_ID", "1"))
if ROBOT_ID not in ROBOT_IPS:
    raise ValueError("ROBOT_ID must be 1 or 2 for dogs_1v1.py")
ROBOT_IP = os.environ.get("ROBOT_IP", ROBOT_IPS[ROBOT_ID])
ROBOT_NAME = ROBOT_NAMES[ROBOT_ID]
ROBOT_PORT = 9559
SUBSCRIPTION = "Soccer1v1"
COLOR_BLOB_SUBSCRIPTION = SUBSCRIPTION + "ColorBlob"

#  Sonar thresholds (metres) 
WARN_DIST    = 0.80
OBS_DIST     = 0.55
DANGER_DIST  = 0.32

#  Walking velocities (normalised 0-1) 
WALK_VX    = 0.55
ARC_THETA  = 0.12
TURN_THETA = 0.75

#  Ball seeking 
KICK_DIST = 0.25
HEAD_SCAN_YAWS = [-1.0, -0.5, 0.0, 0.5, 1.0]

#  Voice recognition 
VOCABULARY       = ["go", "stop", "fetch"]
VOICE_CONFIDENCE = 0.38
VOICE_POLL_SEC   = 0.15

#  Control loop 
POLL_SEC = 0.10
VISION_POLL_SEC = 0.20
GOAL_CAMERA = int(os.environ.get("GOAL_CAMERA", "0"))  # 0=top, 1=bottom
BALL_CAMERA = int(os.environ.get("BALL_CAMERA", "1"))  # 0=top, 1=bottom
GOAL_CAMERAS = [
    int(c.strip()) for c in os.environ.get("GOAL_CAMERAS", "0,1").split(",")
    if c.strip()
]
GOAL_SCAN_SECONDS = float(os.environ.get("GOAL_SCAN_SECONDS", "6.0"))
LINEUP_BALL_DIST = float(os.environ.get("LINEUP_BALL_DIST", "1.10"))
LINEUP_BALL_TIMEOUT = float(os.environ.get("LINEUP_BALL_TIMEOUT", "25.0"))
BALL_SCAN_CYCLES = int(os.environ.get("BALL_SCAN_CYCLES", "4"))
BALL_SCAN_SECONDS = float(os.environ.get("BALL_SCAN_SECONDS", "18.0"))
BALL_HEAD_PITCH = float(os.environ.get("BALL_HEAD_PITCH", "0.4"))
BALL_CONFIRM_HITS = int(os.environ.get("BALL_CONFIRM_HITS", "1"))
FULL_SCAN_STEPS = int(os.environ.get("FULL_SCAN_STEPS", "3"))
GOAL_HEAD_PITCH = float(os.environ.get("GOAL_HEAD_PITCH", "0.3"))
GOAL_PHYSICAL_WIDTH_M = float(os.environ.get("GOAL_PHYSICAL_WIDTH_M", "0.91"))

# ALColorBlobDetection tracks one RGB target color at a time.
# Neon yellow from Walmart soccer net
GOAL_YELLOW_RGB = (
    int(os.environ.get("GOAL_YELLOW_R", "255")),  # red channel
    int(os.environ.get("GOAL_YELLOW_G", "255")),  # green channel
    int(os.environ.get("GOAL_YELLOW_B", "0")),    # blue channel (0 for yellow)
)
ROBOT_WHITE_RGB = (
    int(os.environ.get("ROBOT_WHITE_R", "245")),
    int(os.environ.get("ROBOT_WHITE_G", "245")),
    int(os.environ.get("ROBOT_WHITE_B", "245")),
)
RED_BALL_RGB = (
    int(os.environ.get("RED_BALL_R", "242")),
    int(os.environ.get("RED_BALL_G", "90")),
    int(os.environ.get("RED_BALL_B", "77")),
)
COLOR_THRESHOLD = int(os.environ.get("COLOR_THRESHOLD", "30"))   # threshold for neon yellow specificity
GOAL_MIN_SIZE = int(os.environ.get("GOAL_MIN_SIZE", "10"))  # minimum blob size to filter noise
ROBOT_MIN_SIZE = int(os.environ.get("ROBOT_MIN_SIZE", "80"))
RED_BALL_MIN_SIZE = int(os.environ.get("RED_BALL_MIN_SIZE", "80"))
GOAL_MIN_RGB_CLUSTER_PIXELS = int(os.environ.get("GOAL_MIN_RGB_CLUSTER_PIXELS", "4"))
GOAL_MIN_RGB_BOX_AREA = float(os.environ.get("GOAL_MIN_RGB_BOX_AREA", "80.0"))
GOAL_MIN_RGB_ASPECT = float(os.environ.get("GOAL_MIN_RGB_ASPECT", "0.35"))
GOAL_MAX_RGB_EST_DIST = float(os.environ.get("GOAL_MAX_RGB_EST_DIST", "12.0"))
GOAL_MIN_YELLOW_DENSITY = float(os.environ.get("GOAL_MIN_YELLOW_DENSITY", "0.03"))
GOAL_USE_MESH_BONUS = int(os.environ.get("GOAL_USE_MESH_BONUS", "0"))
GOAL_MIN_CONFIRM_SAMPLES = int(os.environ.get("GOAL_MIN_CONFIRM_SAMPLES", "2"))
GOAL_WORLD_CLUSTER_TOL = float(os.environ.get("GOAL_WORLD_CLUSTER_TOL", "1.0"))
GOAL_DEBUG = int(os.environ.get("GOAL_DEBUG", "1"))
GOAL_VERBOSE_REJECTS = int(os.environ.get("GOAL_VERBOSE_REJECTS", "0"))
GOAL_LOG_THROTTLE_SEC = float(os.environ.get("GOAL_LOG_THROTTLE_SEC", "1.5"))
CALIB_VERBOSE_YAW = int(os.environ.get("CALIB_VERBOSE_YAW", "0"))
YELLOW_HSV_LOWER_VALUES = (
    int(os.environ.get("GOAL_HSV_H_LOW", "40")),
    int(os.environ.get("GOAL_HSV_S_LOW", "100")),
    int(os.environ.get("GOAL_HSV_V_LOW", "100")),
)
YELLOW_HSV_UPPER_VALUES = (
    int(os.environ.get("GOAL_HSV_H_HIGH", "70")),
    int(os.environ.get("GOAL_HSV_S_HIGH", "255")),
    int(os.environ.get("GOAL_HSV_V_HIGH", "255")),
)
BLACK_MESH_S_MAX = int(os.environ.get("GOAL_BLACK_S_MAX", "90"))
BLACK_MESH_V_MAX = int(os.environ.get("GOAL_BLACK_V_MAX", "80"))
YELLOW_HSV_LOWER = np.array(list(YELLOW_HSV_LOWER_VALUES)) if np is not None else YELLOW_HSV_LOWER_VALUES
YELLOW_HSV_UPPER = np.array(list(YELLOW_HSV_UPPER_VALUES)) if np is not None else YELLOW_HSV_UPPER_VALUES
VIDEO_RESOLUTION = 2       # VGA, 640x480
VIDEO_COLORSPACE_RGB = 11  # kRGBColorSpace
VIDEO_FPS = 10

#  NAO 6 dimensions (metres) 
NAO_HEIGHT = 0.573   # metres
NAO_WIDTH  = 0.305   # shoulder-to-shoulder

#  Camera intrinsics (top camera) 
CAMERA_RES_W = 640.0
CAMERA_RES_H = 480.0
CAMERA_FOV_H = math.radians(60.97)  # horizontal FOV in radians
CAMERA_FOCAL_LENGTH = (CAMERA_RES_W / 2.0) / math.tan(CAMERA_FOV_H / 2.0)

#  States 
S_STOPPED = "stopped"
S_WALKING = "walking"

#  Game states 
GAME_INIT     = "init"      # calibrating goals
GAME_LINEUP   = "lineup"    # waiting at center
GAME_PLAYING  = "playing"


def _open_side(left_m, right_m):
    """Return +1.0 (turn left) or -1.0 (turn right) toward the clearer side."""
    return 1.0 if left_m > right_m else -1.0


def _angle_norm(theta):
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta < -math.pi:
        theta += 2.0 * math.pi
    return theta


def _polar_to_xy(azi, dist):
    return dist * math.cos(azi), dist * math.sin(azi)


def _robot_to_world(point, pose):
    x, y = point
    px, py, pt = pose[0], pose[1], pose[2]
    c, s = math.cos(pt), math.sin(pt)
    return px + c * x - s * y, py + s * x + c * y


def _world_to_robot(point, pose):
    wx, wy = point
    px, py, pt = pose[0], pose[1], pose[2]
    dx, dy = wx - px, wy - py
    c, s = math.cos(pt), math.sin(pt)
    return c * dx + s * dy, -s * dx + c * dy


class Soccer1v1(object):

    def __init__(self):
        _install_robot_log_prefix(ROBOT_ID, ROBOT_NAME)
        self._motion_state  = S_STOPPED
        self._game_state    = GAME_INIT
        self._lock          = threading.Lock()
        self._running       = True
        self._arc_sign      = 1
        self._last_word     = None
        self._last_word_time = 0
        self._voice_ready_at = 0
        self._search_theta  = 0.5  # sweep direction for head-scan
        self._last_callout  = 0
        self._asr_ready     = False
        self._color_blob_mode = None
        self._last_goal_debug = 0
        self._active_camera = GOAL_CAMERA
        self._video_clients = {}
        self._opencv_available = (cv2 is not None and np is not None)
        self._opencv_unavailable_logged = False
        self._throttled_log_times = {}
        self._rgb_stats = {
            "frames": 0,
            "no_yellow": 0,
            "clusters": 0,
            "accept": 0,
            "reject_size": 0,
            "reject_shape": 0,
            "reject_score": 0,
            "reject_dist": 0,
        }

        # Background scanner for non-blocking goal detection
        self._scanner_thread = None
        self._scanner_running = False
        # Background body-turning thread to rotate while scanning
        self._turn_thread = None
        self._turn_running = False
        self._turn_direction = 1
        self._turn_speed = 0.20  # angular speed for moveToward

        # Vision
        self._camera        = None
        self._opponent_pos  = None   # (distance_m, azimuth_rad)
        self._goal_left     = None   # (x, y) field coords
        self._goal_right    = None
        self._goal_points_world = []
        self._goal_candidates_world = []
        self._wall_detected = False
        self._shooting      = False

        # Ball tracking
        self._ball_pos_prev = None
        self._ball_vel      = None   # (vx, vy) estimated velocity
        self._ball_moving   = False
        self._ball_visible  = False
        self._ball_miss_count = 0

        print("=" * 60)
        print("1v1 PLAYER: {} (ID {})".format(ROBOT_NAME, ROBOT_ID))
        print("IP: {}:{}".format(ROBOT_IP, ROBOT_PORT))
        print("=" * 60)
        print("Connecting to NAO at {}:{}...".format(ROBOT_IP, ROBOT_PORT))
        self.motion  = ALProxy("ALMotion",            ROBOT_IP, ROBOT_PORT)
        self.posture = ALProxy("ALRobotPosture",      ROBOT_IP, ROBOT_PORT)
        self.tts     = ALProxy("ALTextToSpeech",      ROBOT_IP, ROBOT_PORT)
        self.memory  = ALProxy("ALMemory",            ROBOT_IP, ROBOT_PORT)
        self.sonar   = ALProxy("ALSonar",             ROBOT_IP, ROBOT_PORT)
        self.asr     = ALProxy("ALSpeechRecognition", ROBOT_IP, ROBOT_PORT)
        self.ball    = ALProxy("ALRedBallDetection",  ROBOT_IP, ROBOT_PORT)
        self.video   = ALProxy("ALVideoDevice",       ROBOT_IP, ROBOT_PORT)
        self.tracker = ALProxy("ALTracker",           ROBOT_IP, ROBOT_PORT)
        print("Connected.")

    #  Setup / teardown 

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

        self.sonar.subscribe(SUBSCRIPTION)
        time.sleep(0.4)

        print("[SETUP] Configuring ASR...")
        sys.stdout.flush()
        try:
            self.asr.unsubscribe(SUBSCRIPTION)
        except Exception:
            pass

        try:
            self.asr.stop()
        except Exception:
            pass

        # Try to reliably pause/stop ASR before configuring vocabulary.
        try:
            try:
                self.asr.pause(True)
            except Exception:
                try:
                    self.asr.stop()
                except Exception:
                    pass
                try:
                    time.sleep(0.05)
                    self.asr.pause(True)
                except Exception:
                    pass

            # Configure ASR (language + vocabulary). If this fails we continue
            # without ASR rather than aborting the whole program.
            try:
                self.asr.setLanguage("English")
                self.asr.setVocabulary(VOCABULARY, False)
                self.asr.subscribe(SUBSCRIPTION)
                try:
                    self.memory.insertData("WordRecognized", ["", 0.0])
                except Exception:
                    pass
                self._last_word = None
                self._last_word_time = time.time()
                self._voice_ready_at = 9999999999.0
                try:
                    self.asr.pause(True)
                except Exception:
                    pass
                self._asr_ready = True
            except Exception as e:
                self._asr_ready = False
                print("WARNING: ASR configuration failed, continuing without ASR: {}".format(e))
                sys.stdout.flush()
        except Exception as e:
            self._asr_ready = False
            print("WARNING: ASR init failed, continuing without ASR: {}".format(e))
            sys.stdout.flush()

        print("[SETUP] Subscribing to ball detection...")
        sys.stdout.flush()
        try:
            print("[SETUP] Goal camera: {}".format(GOAL_CAMERA))
            self.video.setActiveCamera(GOAL_CAMERA)
            print("[SETUP] Red ball camera: {}".format(BALL_CAMERA))
            self.ball.subscribe(SUBSCRIPTION)
        except Exception as e:
            print("WARNING: Could not subscribe to red ball detection: {}".format(e))

        print("[SETUP] Configuring color blob detection...")
        sys.stdout.flush()
        diag_enabled = os.environ.get("DUMP_ALMEMORY_KEYS") == "1"
        if diag_enabled:
            print("[DIAG] DUMP_ALMEMORY_KEYS=1 enabled")
            sys.stdout.flush()
        else:
            print("[DIAG] ALMemory probe disabled (set DUMP_ALMEMORY_KEYS=1 to enable)")
            sys.stdout.flush()
        try:
            self.color_blob = ALProxy("ALColorBlobDetection", ROBOT_IP, ROBOT_PORT)
            try:
                self.color_blob.unsubscribe(COLOR_BLOB_SUBSCRIPTION)
            except Exception:
                pass
            try:
                self.color_blob.subscribe(COLOR_BLOB_SUBSCRIPTION)
                print("[VISION] color_blob subscribed: {}".format(COLOR_BLOB_SUBSCRIPTION))
                sys.stdout.flush()
            except Exception as e:
                print("[VISION] color_blob subscribe failed: {}".format(e))
                sys.stdout.flush()
            self._setup_goal_blob_detection()
        except Exception as e:
            print("WARNING: Could not setup color blob detection: {}".format(e))
        finally:
            if diag_enabled:
                try:
                    self._dump_color_blob_diagnostics()
                except Exception as e:
                    print("[DIAG] diagnostic dump failed: {}".format(e))
                    sys.stdout.flush()

        self._announce("1v1 soccer mode ready.", priority=True)

    def shutdown(self):
        self._running = False
        try:
            self.motion.stopMove()
        except Exception:
            pass
        for proxy in (self.asr, self.sonar, self.ball):
            if proxy is not None:
                try:
                    proxy.unsubscribe(SUBSCRIPTION)
                except Exception:
                    pass
        for client in self._video_clients.values():
            try:
                self.video.unsubscribe(client)
            except Exception:
                pass
        # Unsubscribe color blob detector if present
        try:
            if getattr(self, 'color_blob', None) is not None:
                try:
                    self.color_blob.unsubscribe(COLOR_BLOB_SUBSCRIPTION)
                except Exception:
                    pass
        except Exception:
            pass
        # Stop background scanner
        try:
            self._stop_goal_scanner()
        except Exception:
            pass
        try:
            self.asr.pause(True)
        except Exception:
            pass
        try:
            self.motion.stopMove()
            self.posture.goToPosture("Stand", 0.5)
        except Exception:
            pass
        try:
            self.motion.setStiffnesses("Body", 0.0)
        except Exception:
            pass
        print("Shutdown complete.")

    #  State helpers 

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

    def _announce(self, msg, priority=False):
        now = time.time()
        if not priority and (now - self._last_callout) < 0.5:
            return
        self._last_callout = now
        try:
            self.tts.post.say(msg)
        except Exception as e:
            print("[TTS][ID {}] post.say failed: {}".format(ROBOT_ID, e))
            sys.stdout.flush()
            try:
                self.tts.say(msg)
            except Exception as e2:
                print("[TTS][ID {}] say failed: {}".format(ROBOT_ID, e2))
                sys.stdout.flush()

    def _log_ignored_exception(self, label, exc=None):
        if exc is None:
            print("[WARN][ID {}] {}".format(ROBOT_ID, label))
        else:
            print("[WARN][ID {}] {}: {}".format(ROBOT_ID, label, exc))
        sys.stdout.flush()

    def _log_throttled(self, key, message, min_interval=1.0):
        now = time.time()
        last = self._throttled_log_times.get(key, 0.0)
        if now - last < min_interval:
            return
        self._throttled_log_times[key] = now
        print(message)
        sys.stdout.flush()

    #  Sensors 

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
        # During lineup, prefer the red blob path because the ball is large enough
        # to be stable in color space even when the built-in detector is noisy.
        if self._get_game_state() == GAME_LINEUP:
            try:
                result = self._detect_red_ball_blob()
                if result is not None:
                    if not self._ball_visible:
                        print("[BALL] detected via color blob")
                        self._ball_visible = True
                    self._ball_miss_count = 0
                    return result
            except Exception:
                pass

        # Try ALRedBallDetection first outside lineup
        # Format: [[ts_s, ts_us], [azimuth, elevation, size_x, size_y], [cam_xyz...], [robot_xyz...], confidence]
        try:
            data = self.memory.getData("redBallDetected")
            if data and len(data) > 1 and isinstance(data[1], (list, tuple)) and len(data[1]) >= 3:
                azimuth = float(data[1][0])
                angular_size = float(data[1][2])
                dist = (0.1 / angular_size) if angular_size > 1e-4 else 1.0
                ball_result = (azimuth, dist)
                if not self._ball_visible:
                    print("[BALL] detected via ALRedBallDetection azi={:.3f} dist={:.2f}".format(azimuth, dist))
                    self._ball_visible = True
                self._ball_miss_count = 0
                return ball_result
        except Exception as _ball_ex:
            print("[BALL] ALRedBallDetection parse error: {}".format(_ball_ex))
            sys.stdout.flush()
        # Blob fallback during lineup when ALRedBallDetection did not return.
        if self._get_game_state() == GAME_LINEUP:
            try:
                result = self._detect_red_ball_blob()
                if result is not None:
                    if not self._ball_visible:
                        print("[BALL] detected via color blob")
                        self._ball_visible = True
                    self._ball_miss_count = 0
                    return result
            except Exception:
                pass
        self._ball_miss_count += 1
        if self._ball_visible and self._ball_miss_count >= 5:
            print("[BALL] lost")
            self._ball_visible = False
        return None

    def _set_head_yaw(self, yaw, speed=0.25):
        try:
            self.motion.setAngles("HeadYaw", yaw, speed)
        except Exception:
            pass

    def _set_head_pitch(self, pitch, speed=0.25):
        try:
            self.motion.setAngles("HeadPitch", pitch, speed)
        except Exception:
            pass

    def _center_head(self):
        self._set_head_yaw(0.0)

    def _set_camera(self, camera_id):
        try:
            self._active_camera = camera_id
            self.video.setActiveCamera(camera_id)
        except Exception:
            pass

    def _video_client_for_camera(self, camera_id):
        if camera_id not in self._video_clients:
            name = "{}Video{}{}".format(SUBSCRIPTION, ROBOT_ID, camera_id)
            self._video_clients[camera_id] = self.video.subscribeCamera(
                name, camera_id, VIDEO_RESOLUTION, VIDEO_COLORSPACE_RGB, VIDEO_FPS)
        return self._video_clients[camera_id]

    def _robot_pose(self):
        try:
            return self.motion.getRobotPosition(False)
        except Exception:
            return [0.0, 0.0, 0.0]

    def _ball_world_point(self, ball):
        ball_azi, ball_dist = ball
        return _robot_to_world(_polar_to_xy(ball_azi, ball_dist), self._robot_pose())

    def _store_goal_world_points(self, goals):
        pose = self._robot_pose()
        for goal in goals:
            if len(goal) >= 5:
                area = float(goal[2])
                x_ang, dist = float(goal[3]), float(goal[4])
                if dist <= 0.05 or dist > GOAL_MAX_RGB_EST_DIST:
                    continue
                point = _robot_to_world(_polar_to_xy(x_ang, dist), pose)
                self._goal_candidates_world.append((dist, area, point))

    def _select_nearest_goal_world_point(self):
        if not self._goal_candidates_world:
            return False

        # Require repeated detections from the same world-space neighborhood
        # before locking to avoid one-off yellow false positives.
        best_cluster = None
        for i in range(len(self._goal_candidates_world)):
            _, _, p0 = self._goal_candidates_world[i]
            cluster = []
            for candidate in self._goal_candidates_world:
                dist, area, p = candidate
                if math.hypot(p[0] - p0[0], p[1] - p0[1]) <= GOAL_WORLD_CLUSTER_TOL:
                    cluster.append(candidate)
            if best_cluster is None or len(cluster) > len(best_cluster):
                best_cluster = cluster

        if best_cluster is None or len(best_cluster) < GOAL_MIN_CONFIRM_SAMPLES:
            if GOAL_DEBUG:
                print("[GOAL] candidate cluster too weak: best_cluster={} required={} total_samples={}".format(
                    0 if best_cluster is None else len(best_cluster),
                    GOAL_MIN_CONFIRM_SAMPLES,
                    len(self._goal_candidates_world)))
                sys.stdout.flush()
            return False

        avg_x = sum(c[2][0] for c in best_cluster) / float(len(best_cluster))
        avg_y = sum(c[2][1] for c in best_cluster) / float(len(best_cluster))
        avg_dist = sum(c[0] for c in best_cluster) / float(len(best_cluster))
        max_area = max(c[1] for c in best_cluster)

        self._goal_points_world = [(avg_x, avg_y)]
        print("[GOAL] selected stable cluster dist~{:.2f} area_max={:.0f} point=({:.2f}, {:.2f}) cluster_size={} of {} samples".format(
            avg_dist, max_area, avg_x, avg_y, len(best_cluster), len(self._goal_candidates_world)))
        sys.stdout.flush()
        return True

    def _goal_world_point(self):
        if not self._goal_points_world:
            return None
        x = sum(p[0] for p in self._goal_points_world) / float(len(self._goal_points_world))
        y = sum(p[1] for p in self._goal_points_world) / float(len(self._goal_points_world))
        return x, y

    def _setup_goal_blob_detection(self):
        """Configure ALColorBlobDetection for neon-yellow goals (same abstraction as robot/ball)."""
        try:
            if self._color_blob_mode == "goal":
                return
            try:
                self.color_blob.setColorSpace(0)
            except Exception:
                pass
            self.color_blob.setColor(
                GOAL_YELLOW_RGB[0],
                GOAL_YELLOW_RGB[1],
                GOAL_YELLOW_RGB[2],
                COLOR_THRESHOLD
            )
            self.color_blob.setObjectProperties(GOAL_MIN_SIZE, 0.5, "Unknown")
            try:
                self.color_blob.setAutoExposure(True)
            except Exception:
                pass
            try:
                self.tracker.addTarget("ColorBlob", [
                    GOAL_YELLOW_RGB[0],
                    GOAL_YELLOW_RGB[1],
                    GOAL_YELLOW_RGB[2],
                    COLOR_THRESHOLD,
                    GOAL_MIN_SIZE,
                ])
                self.tracker.track("ColorBlob")
                print("[VISION] ALTracker tracking ColorBlob target")
            except Exception as e:
                print("[VISION] tracker ColorBlob activation failed: {}".format(e))
            self._color_blob_mode = "goal"
            print("[VISION] Color blob target: neon yellow goals rgb={} threshold={}".format(
                GOAL_YELLOW_RGB, COLOR_THRESHOLD))
            sys.stdout.flush()
        except Exception as e:
            print("WARNING: goal blob setup failed: {}".format(e))
            sys.stdout.flush()

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

    def _setup_robot_blob_detection(self):
        """Configure ALColorBlobDetection for white robot/opponent blobs."""
        try:
            if self._color_blob_mode == "robot":
                return
            try:
                self.color_blob.setColorSpace(0)
            except Exception:
                pass
            self.color_blob.setColor(
                ROBOT_WHITE_RGB[0],
                ROBOT_WHITE_RGB[1],
                ROBOT_WHITE_RGB[2],
                COLOR_THRESHOLD
            )
            self.color_blob.setObjectProperties(ROBOT_MIN_SIZE, NAO_HEIGHT, "Unknown")
            try:
                self.color_blob.setAutoExposure(True)
            except Exception:
                pass
            self._color_blob_mode = "robot"
            print("[VISION] Color blob target: white robots rgb={} threshold={}".format(
                ROBOT_WHITE_RGB, COLOR_THRESHOLD))
        except Exception as e:
            print("WARNING: robot blob setup failed: {}".format(e))

    def _setup_red_ball_blob_detection(self):
        """Configure ALColorBlobDetection for the large red ball."""
        try:
            if self._color_blob_mode == "red_ball":
                return
            try:
                self.color_blob.setColorSpace(0)
            except Exception:
                pass
            self.color_blob.setColor(
                RED_BALL_RGB[0],
                RED_BALL_RGB[1],
                RED_BALL_RGB[2],
                COLOR_THRESHOLD
            )
            self.color_blob.setObjectProperties(RED_BALL_MIN_SIZE, 0.25, "Circle")
            try:
                self.color_blob.setAutoExposure(True)
            except Exception:
                pass
            self._color_blob_mode = "red_ball"
            print("[VISION] Color blob target: red ball rgb={} threshold={}".format(
                RED_BALL_RGB, COLOR_THRESHOLD))
        except Exception as e:
            print("WARNING: red ball blob setup failed: {}".format(e))

    def _read_color_blobs(self):
        try:
            data = self.memory.getData("ALTracker/ColorBlobDetected")
            if GOAL_DEBUG and data:
                print("[GOAL] _read_color_blobs got data: type={} len={} first_elem_type={}".format(
                    type(data), len(data), type(data[0]) if len(data) > 0 else "N/A"))
                sys.stdout.flush()
            if data:
                return data
        except Exception as e:
            if GOAL_DEBUG:
                print("[GOAL] _read_color_blobs exception: {}".format(e))
                sys.stdout.flush()
        return []

    def _dump_color_blob_diagnostics(self):
        """Optional diagnostic: probe ALColorBlobDetection proxy and ALMemory
        for any keys or registrations related to color blobs. This is safe and
        wrapped with exception handling; enable by setting
        env `DUMP_ALMEMORY_KEYS=1` on the robot before starting the script.
        """
        print("[DIAG] dumping ALColorBlobDetection diagnostics...")
        sys.stdout.flush()
        # Probe color_blob proxy for subscriber info (if available)
        try:
            subs = None
            color_blob = getattr(self, 'color_blob', None)
            if color_blob is None:
                print("[DIAG] color_blob proxy is not available")
                sys.stdout.flush()
                color_blob = None
            try:
                subs = getattr(color_blob, 'getSubscribers', lambda: None)() if color_blob is not None else None
            except Exception:
                try:
                    subs = getattr(color_blob, 'getSubscribersList', lambda: None)() if color_blob is not None else None
                except Exception:
                    subs = None
            print("[DIAG] color_blob subscribers: {}".format(subs))
        except Exception as e:
            print("[DIAG] color_blob subscriber probe failed: {}".format(e))
        sys.stdout.flush()

        # Try ALMemory inspection helpers (these may or may not exist)
        try:
            get_list_fn = getattr(self.memory, 'getDataListRegisteredInModule', None)
            if get_list_fn is None:
                print("[DIAG] getDataListRegisteredInModule unavailable on this NAOqi build; skipping")
            else:
                for mod in (COLOR_BLOB_SUBSCRIPTION, 'ALColorBlobDetection', 'ColorBlobDetection', SUBSCRIPTION):
                    try:
                        keys = get_list_fn(mod)
                        print("[DIAG] getDataListRegisteredInModule({}) -> {} keys".format(mod, len(keys) if keys else 0))
                        if keys:
                            filtered = [k for k in keys if 'Color' in k or 'color' in k or 'Blob' in k]
                            print("[DIAG]   filtered: {}".format(filtered[:50]))
                    except Exception:
                        # Method exists but this module isn't introspectable on this firmware.
                        pass
        except Exception:
            pass
        sys.stdout.flush()

        # getDataList on this firmware takes a string filter; so, use it to find real keys
        for prefix in ("ColorBlob", "Color", "Blob", "blob", "Vision"):
            try:
                keys = self.memory.getDataList(prefix)
                if keys:
                    print("[DIAG] getDataList('{}') -> {}".format(prefix, keys[:30]))
                else:
                    print("[DIAG] getDataList('{}') -> empty".format(prefix))
            except Exception as e:
                print("[DIAG] getDataList('{}') failed: {}".format(prefix, e))
        sys.stdout.flush()

        # Probe direct methods on the color_blob proxy itself
        color_blob = getattr(self, 'color_blob', None)
        if color_blob is not None:
            for method in ('getObjectList', 'getBlobList', 'getOutput',
                           'getBlobs', 'getNearestObject', 'getResult'):
                try:
                    fn = getattr(color_blob, method, None)
                    if fn is None:
                        continue
                    result = fn()
                    print("[DIAG] color_blob.{}() -> {}".format(method, result))
                except Exception:
                    # Skip unsupported proxy methods silently to keep logs actionable.
                    pass
        sys.stdout.flush()

        # Probe known key variants
        probe_keys = [
            'ColorBlobDetection/blobs', 'ColorBlobDetection/Blobs',
            'ColorBlobDetection/LastDetection', 'ColorBlobDetected',
            COLOR_BLOB_SUBSCRIPTION + '/blobs', COLOR_BLOB_SUBSCRIPTION + '/Blobs'
        ]
        for k in probe_keys:
            try:
                v = self.memory.getData(k)
                print("[DIAG] getData({}) -> type={}".format(k, type(v)))
            except Exception as e:
                print("[DIAG] getData({}) -> {}".format(k, e))
        sys.stdout.flush()

    def _capture_hsv_frame(self, camera_id):
        if cv2 is None or np is None:
            return None
        try:
            client = self._video_client_for_camera(camera_id)
            result = self.video.getImageRemote(client)
            if result is None:
                return None
            width, height, channels, img_buffer = (
                result[0], result[1], result[2], result[6])
            img = np.frombuffer(img_buffer, dtype=np.uint8)
            img = img.reshape((height, width, channels))
            return cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        except Exception as e:
            if GOAL_DEBUG:
                print("[GOAL] frame capture failed: {}".format(e))
                sys.stdout.flush()
            return None

    def _detect_goals_from_frame(self, camera_id):
        frame = self._capture_hsv_frame(camera_id)
        if frame is None:
            return []
        
        # Check if HSV thresholds are available
        if YELLOW_HSV_LOWER is None or YELLOW_HSV_UPPER is None:
            return []
        
        # Debug: sample HSV values from center of frame
        if GOAL_DEBUG and frame is not None and len(frame.shape) == 3:
            center_y, center_x = frame.shape[0] // 2, frame.shape[1] // 2
            sample_h = int(frame[center_y, center_x, 0])
            sample_s = int(frame[center_y, center_x, 1])
            sample_v = int(frame[center_y, center_x, 2])
            print("[GOAL] center pixel HSV=({}, {}, {}) camera={}".format(sample_h, sample_s, sample_v, camera_id))
            sys.stdout.flush()
        
        mask = cv2.inRange(frame, YELLOW_HSV_LOWER, YELLOW_HSV_UPPER)
        
        # Save debug frame if requested
        debug_dir = os.environ.get("GOAL_DEBUG_FRAMES")
        if debug_dir:
            try:
                import time as time_module
                ts = int(time_module.time() * 1000) % 100000
                cv2.imwrite("{}/mask_{}_c{}.png".format(debug_dir, ts, camera_id), mask)
                bgr_frame = cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)
                cv2.imwrite("{}/frame_{}_c{}.png".format(debug_dir, ts, camera_id), bgr_frame)
            except Exception as e:
                if GOAL_DEBUG:
                    print("[GOAL] frame save failed: {}".format(e))
        
        found = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = found[0] if len(found) == 2 else found[1]

        goals = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 100:
                continue
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            x_ang = ((float(cx) - CAMERA_RES_W / 2.0) /
                     (CAMERA_RES_W / 2.0)) * (CAMERA_FOV_H / 2.0)
            goals.append((cx, cy, area, x_ang, 0.0))

        if GOAL_DEBUG:
            mask_white = cv2.countNonZero(mask)
            print("[GOAL] frame camera={} mask_pixels={} contours={} candidates={}".format(
                camera_id, mask_white, len(contours), len(goals)))
            sys.stdout.flush()
        return sorted(goals, key=lambda g: g[0])

    def _detect_goals_from_rgb_frame(self, camera_id):
        try:
            self._rgb_stats["frames"] += 1
            client = self._video_client_for_camera(camera_id)
            result = None
            for _ in range(2):
                result = self.video.getImageRemote(client)
                if result is not None and len(result) >= 7 and result[6] is not None:
                    break
                time.sleep(0.05)
            if result is None or len(result) < 7 or result[6] is None:
                if GOAL_DEBUG:
                    print("[GOAL] RGB fallback camera={} missing image buffer".format(camera_id))
                    sys.stdout.flush()
                return []
            width, height, channels, img_buffer = (
                result[0], result[1], result[2], result[6])
            if channels != 3:
                return []
            if img_buffer is None:
                if GOAL_DEBUG:
                    print("[GOAL] RGB fallback camera={} returned None buffer".format(camera_id))
                    sys.stdout.flush()
                return []
            try:
                img_buffer = bytearray(img_buffer)
            except Exception:
                if GOAL_DEBUG:
                    print("[GOAL] RGB fallback camera={} could not normalize buffer".format(camera_id))
                    sys.stdout.flush()
                return []

            # Pure-Python HSV test that matches the configured yellow thresholds.
            goal_pixels = []
            black_pixels = []
            step = 4
            hsv_low = YELLOW_HSV_LOWER
            hsv_high = YELLOW_HSV_UPPER
            sampled_pixels = 0
            for y in range(0, height, step):
                row_offset = y * width * channels
                for x in range(0, width, step):
                    offset = row_offset + x * channels
                    r = ord(img_buffer[offset]) if isinstance(img_buffer[offset], str) else img_buffer[offset]
                    g = ord(img_buffer[offset + 1]) if isinstance(img_buffer[offset + 1], str) else img_buffer[offset + 1]
                    b = ord(img_buffer[offset + 2]) if isinstance(img_buffer[offset + 2], str) else img_buffer[offset + 2]
                    sampled_pixels += 1
                    h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
                    h = int(h * 179.0)
                    s = int(s * 255.0)
                    v = int(v * 255.0)
                    if (hsv_low[0] <= h <= hsv_high[0] and
                            hsv_low[1] <= s <= hsv_high[1] and
                            hsv_low[2] <= v <= hsv_high[2]):
                        goal_pixels.append((x, y))
                    elif s <= BLACK_MESH_S_MAX and v <= BLACK_MESH_V_MAX:
                        black_pixels.append((x, y))

            if not goal_pixels:
                self._rgb_stats["no_yellow"] += 1
                if GOAL_DEBUG:
                    self._log_throttled(
                        "rgb_no_yellow_cam{}".format(camera_id),
                        "[GOAL] RGB fallback camera={} found no yellow pixels out of {} samples".format(
                            camera_id, sampled_pixels),
                        min_interval=GOAL_LOG_THROTTLE_SEC)
                return []

            neighbor_limit = step * 3
            clusters = []
            remaining = list(goal_pixels)
            while remaining:
                seed = remaining.pop()
                cluster = [seed]
                expanded = True
                while expanded:
                    expanded = False
                    still_remaining = []
                    for point in remaining:
                        close = False
                        for cluster_point in cluster:
                            if (abs(point[0] - cluster_point[0]) <= neighbor_limit and
                                    abs(point[1] - cluster_point[1]) <= neighbor_limit):
                                close = True
                                break
                        if close:
                            cluster.append(point)
                            expanded = True
                        else:
                            still_remaining.append(point)
                    remaining = still_remaining
                clusters.append(cluster)
            self._rgb_stats["clusters"] += len(clusters)

            best_candidate = None
            reject_shape = 0
            reject_size = 0
            reject_score = 0
            reject_dist = 0
            for cluster in clusters:
                xs = [p[0] for p in cluster]
                ys = [p[1] for p in cluster]
                min_x, max_x = min(xs), max(xs)
                min_y, max_y = min(ys), max(ys)
                cx = int((min_x + max_x) / 2.0)
                cy = int((min_y + max_y) / 2.0)
                box_w = max_x - min_x + step
                box_h = max_y - min_y + step
                if box_w < 12 or box_h < 8:
                    reject_size += 1
                    continue
                area = float(box_w * box_h)
                sample_cols = max(1, int(math.ceil(float(box_w) / float(step))))
                sample_rows = max(1, int(math.ceil(float(box_h) / float(step))))
                box_samples = float(sample_cols * sample_rows)
                yellow_density = float(len(cluster)) / box_samples
                black_in_box = 0
                for bx, by in black_pixels:
                    if min_x - step <= bx <= max_x + step and min_y - step <= by <= max_y + step:
                        black_in_box += 1
                black_density = float(black_in_box) / box_samples
                edge_pixels = 0
                inner_pixels = 0
                for px, py in cluster:
                    if (px <= min_x + step or px >= max_x - step or
                            py <= min_y + step or py >= max_y - step):
                        edge_pixels += 1
                    else:
                        inner_pixels += 1
                edge_ratio = float(edge_pixels) / float(max(1, len(cluster)))
                x_ang = ((float(cx) - CAMERA_RES_W / 2.0) /
                         (CAMERA_RES_W / 2.0)) * (CAMERA_FOV_H / 2.0)
                ang_width = max(math.radians(1.0), (float(box_w) / CAMERA_RES_W) * CAMERA_FOV_H)
                est_dist = GOAL_PHYSICAL_WIDTH_M / (2.0 * math.tan(ang_width / 2.0))
                if GOAL_DEBUG and (
                        len(cluster) >= max(6, GOAL_MIN_RGB_CLUSTER_PIXELS - 2) and
                        area >= (GOAL_MIN_RGB_BOX_AREA * 0.75) and
                        est_dist <= 12.0):
                    self._log_throttled(
                        "rgb_cluster_cam{}".format(camera_id),
                        "[GOAL] RGB cluster cam={} yellow={} black={} edge={:.2f} density={:.3f}/{:.3f} box=({}, {}) size=({}, {}) est_dist={:.2f}".format(
                            camera_id, len(cluster), black_in_box, edge_ratio, yellow_density,
                            black_density, cx, cy, box_w, box_h, est_dist),
                        min_interval=0.25)
                aspect = float(box_w) / float(max(1, box_h))
                if len(cluster) < GOAL_MIN_RGB_CLUSTER_PIXELS or area < GOAL_MIN_RGB_BOX_AREA:
                    reject_size += 1
                    continue
                if aspect < GOAL_MIN_RGB_ASPECT:
                    reject_shape += 1
                    continue
                required_yellow_density = GOAL_MIN_YELLOW_DENSITY
                # Thin yellow rims can enclose mostly dark interiors. Treat dark fill
                # as a soft relaxation signal, not a hard requirement.
                if black_density >= 0.20:
                    required_yellow_density *= 0.60
                if edge_ratio >= 0.65:
                    required_yellow_density *= 0.80
                required_yellow_density = max(0.01, required_yellow_density)
                if yellow_density < required_yellow_density:
                    reject_score += 1
                    continue
                if not (0.25 <= est_dist <= GOAL_MAX_RGB_EST_DIST):
                    reject_dist += 1
                    continue

                candidate = (cx, cy, area, x_ang, est_dist)
                if best_candidate is None or len(cluster) > best_candidate[0]:
                    best_candidate = (len(cluster), candidate)

            self._rgb_stats["reject_size"] += reject_size
            self._rgb_stats["reject_shape"] += reject_shape
            self._rgb_stats["reject_score"] += reject_score
            self._rgb_stats["reject_dist"] += reject_dist

            if best_candidate is not None:
                self._rgb_stats["accept"] += 1
                if GOAL_DEBUG:
                    c = best_candidate[1]
                    self._log_throttled(
                        "rgb_accept_cam{}".format(camera_id),
                        "[GOAL] RGB accepted candidate cam={} center=({}, {}) area={:.0f} azi={:.2f} dist={:.2f} clusters={} rejects(s={},sh={},sc={},d={})".format(
                            camera_id, int(c[0]), int(c[1]), float(c[2]), float(c[3]), float(c[4]),
                            len(clusters), reject_size, reject_shape, reject_score, reject_dist),
                        min_interval=0.5)
                return [best_candidate[1]]
            if GOAL_DEBUG and GOAL_VERBOSE_REJECTS and clusters:
                self._log_throttled(
                    "rgb_reject_cam{}".format(camera_id),
                    "[GOAL] RGB saw {} yellow clusters on cam{} but accepted none; rejects(s={},sh={},sc={},d={})".format(
                        len(clusters), camera_id, reject_size, reject_shape, reject_score, reject_dist),
                    min_interval=GOAL_LOG_THROTTLE_SEC)
        except Exception as e:
            if GOAL_DEBUG:
                print("[GOAL] RGB fallback failed: {}".format(e))
                sys.stdout.flush()
        return []

    def _detect_red_ball_blob(self):
        try:
            self._setup_red_ball_blob_detection()
            blobs = self._read_color_blobs()
            if not blobs:
                return None
            candidates = []
            for blob in blobs:
                if len(blob) >= 7:
                    w, h, x_ang, dist = blob[2], blob[3], blob[4], blob[6]
                    area = w * h
                    if 0.15 < dist < 5.0 and area >= RED_BALL_MIN_SIZE:
                        candidates.append((area, float(x_ang), float(dist)))
            if not candidates:
                return None
            candidates.sort(reverse=True)
            _, azi, dist = candidates[0]
            return azi, dist
        except Exception:
            return None

    def _detect_opponent(self):
        """Return nearest likely opponent as (distance_m, azimuth_rad)."""
        try:
            self._setup_robot_blob_detection()
            robots = self._detect_robots()
            if not robots:
                return None
            return min(robots, key=lambda r: r[0])
        except Exception:
            return None

    def _detect_robots(self):
        """Detect white blobs using ALColorBlobDetection."""
        try:
            blobs = self._read_color_blobs()
            if not blobs:
                return []
            robots = []
            for blob in blobs:
                if len(blob) >= 7:
                    x_ang, dist = blob[4], blob[6]
                    if 0.1 < dist < 3.0:
                        robots.append((float(dist), float(x_ang)))
            return robots
        except Exception:
            return []

    def _detect_goals(self):
        """Detect yellow goal from ALTracker/ColorBlobDetected.
        Format: [[x,y,z,wx,wy,conf], [filtered_x,y,z,...], [ts_s,ts_us], camId, extra]
        x,y,z are position in torso frame (meters). Azimuth = atan2(y, x).
        """
        try:
            self._setup_goal_blob_detection()
            data = self._read_color_blobs()
            if not data:
                if GOAL_DEBUG:
                    self._log_throttled(
                        "no_almemory_blobs",
                        "[GOAL] no ALMemory blobs; trying RGB frame fallback",
                        min_interval=GOAL_LOG_THROTTLE_SEC)
                for camera_id in GOAL_CAMERAS:
                    goals = self._detect_goals_from_rgb_frame(camera_id)
                    if goals:
                        return goals
                return []
            if GOAL_DEBUG:
                print("[GOAL] raw blob data: {} elements, type={}".format(len(data), type(data)))
                if len(data) > 0:
                    print("[GOAL]   data[0]={}, type={}".format(data[0], type(data[0])))
                sys.stdout.flush()
            if not isinstance(data[0], list) or len(data[0]) < 3:
                return []
            pos = data[0]  # use raw (not filtered) position
            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
            dist = math.sqrt(x*x + y*y + z*z)
            x_ang = math.atan2(y, x)
            angular_size = float(pos[4]) if len(pos) > 4 else 0.1
            if GOAL_DEBUG:
                print("[GOAL] parsed: x={:.3f} y={:.3f} z={:.3f} dist={:.3f} azi={:.3f} size={:.3f}".format(
                    x, y, z, dist, x_ang, angular_size))
                sys.stdout.flush()
            if 0.1 < dist < 8.0:
                cx = int((x_ang / CAMERA_FOV_H + 0.5) * CAMERA_RES_W)
                if GOAL_DEBUG:
                    print("[GOAL] VALID: dist in range, returning detection")
                    sys.stdout.flush()
                return [(cx, 240, angular_size * 1000, x_ang, dist)]
            else:
                if GOAL_DEBUG:
                    print("[GOAL] REJECTED: dist {:.3f} outside range [0.1, 8.0]".format(dist))
                    sys.stdout.flush()
            return []
        except Exception as e:
            print("[GOAL] parse error: {}".format(e))
            sys.stdout.flush()
            return []

    # -------------------- Background goal scanner --------------------
    def _start_goal_scanner(self):
        """Start a background thread that continuously scans with the head
        and collects goal blob candidates into `_goal_candidates_world`.
        This allows `_calibrate_goals` to be non-blocking.
        """
        if self._scanner_running:
            return
        print("[SCANNER] starting background goal scanner...")
        sys.stdout.flush()
        self._scanner_running = True
        self._scanner_thread = threading.Thread(target=self._goal_scanner_thread)
        self._scanner_thread.daemon = True
        self._scanner_thread.start()
        # Start a companion turning thread so the robot slowly rotates
        try:
            self._start_scanner_turning()
        except Exception:
            pass

    def _stop_goal_scanner(self):
        self._scanner_running = False
        if self._scanner_thread is not None:
            try:
                self._scanner_thread.join(timeout=1.0)
            except Exception:
                pass
            self._scanner_thread = None
        # Stop turning thread too
        try:
            self._stop_scanner_turning()
        except Exception:
            pass

    def _goal_scanner_thread(self):
        """Continuously sweep head and collect goal candidates.
        This thread only moves the head and reads color blobs; it avoids
        moving the base so it can run concurrently with other actions.
        """
        try:
            print("[SCANNER] thread entered main loop")
            sys.stdout.flush()
            while self._scanner_running and self._running:
                for camera_id in GOAL_CAMERAS:
                    if not (self._scanner_running and self._running):
                        break
                    self._set_camera(camera_id)
                    # Ensure head is pitched downward to scan ground-level goals
                    try:
                        self._set_head_pitch(GOAL_HEAD_PITCH, speed=0.25)
                        # small pause to let pitch settle
                        time.sleep(0.08)
                    except Exception:
                        pass
                    # brief pause to let camera switch
                    time.sleep(0.08)
                    for yaw in HEAD_SCAN_YAWS:
                        if not (self._scanner_running and self._running):
                            break
                        try:
                            self._set_head_yaw(yaw)
                            time.sleep(0.10)
                            goals = self._detect_goals()
                            if goals:
                                # store observed goals as world points
                                self._store_goal_world_points(goals)
                                try:
                                    print("[SCANNER] found {} goals on camera {} yaw {:.2f}; candidates_world={}".format(
                                        len(goals), camera_id, yaw, len(self._goal_candidates_world)))
                                    sys.stdout.flush()
                                except Exception:
                                    pass
                        except Exception:
                            pass
                # small delay between full cycles
                time.sleep(0.2)
        except Exception:
            pass

    def _turning_thread(self):
        """Continuously rotate the robot slowly while scanner runs.
        Uses moveToward for smooth, interruptible rotation bursts.
        """
        try:
            print("[TURN] turning thread entered main loop")
            sys.stdout.flush()
            # Continuous gentle rotation to avoid jitter from repeated start/stops.
            while self._turn_running and self._running:
                try:
                    ang = self._turn_speed * self._turn_direction
                    # Issue a sustained low-speed rotation command
                    self.motion.moveToward(0.0, 0.0, ang)
                    # Sleep for a longer interval to keep command steady
                    time.sleep(0.9)
                except Exception:
                    try:
                        self.motion.stopMove()
                    except Exception:
                        pass
                    time.sleep(0.2)
            # Ensure any residual motion stops when thread exits
            try:
                self.motion.stopMove()
            except Exception:
                pass
        except Exception:
            pass

    def _start_scanner_turning(self):
        if self._turn_running:
            return
        self._turn_running = True
        self._turn_thread = threading.Thread(target=self._turning_thread)
        self._turn_thread.daemon = True
        self._turn_thread.start()

    def _stop_scanner_turning(self):
        self._turn_running = False
        if self._turn_thread is not None:
            try:
                self._turn_thread.join(timeout=1.0)
            except Exception:
                pass
            self._turn_thread = None
        # Make sure robot stops turning
        try:
            self.motion.stopMove()
        except Exception:
            pass

    def _detect_wall_edge(self):
        try:
            blobs = self._read_color_blobs()
            if blobs:
                for blob in blobs:
                    if len(blob) >= 7 and blob[6] < 0.3:
                        return True
            return False
        except Exception:
            return False

    #  Vision thread 

    def _vision_thread(self):
        """Detect opponent, goals, update state."""
        while self._running:
            try:
                game_state = self._get_game_state()
                if game_state == GAME_PLAYING:
                    self._opponent_pos = self._detect_opponent()
                elif game_state == GAME_INIT:
                    goals = self._detect_goals()
                    if len(goals) >= 2:
                        self._store_goal_world_points(goals[:2])
                        self._goal_left = goals[0][:2]
                        self._goal_right = goals[-1][:2]
                    elif len(goals) == 1:
                        self._store_goal_world_points(goals)
                        if GOAL_DEBUG:
                            print("[GOAL] ignoring single goal blob until a second stable goal is found")
                            sys.stdout.flush()
                self._wall_detected = self._detect_wall_edge()
            except Exception:
                pass
            time.sleep(VISION_POLL_SEC)

    #  Voice thread 

    def _voice_thread(self):
        while self._running:
            try:
                data = self.memory.getData("WordRecognized")
                if data and len(data) >= 2:
                    word, conf = data[0], float(data[1])
                    now = time.time()
                    if now < self._voice_ready_at:
                        time.sleep(VOICE_POLL_SEC)
                        continue
                    if word and conf >= VOICE_CONFIDENCE and (
                            word != self._last_word or
                            now - self._last_word_time > 1.5):
                        self._last_word = word
                        self._last_word_time = now
                        word = word.lower().strip()
                        print("Heard: '{}'".format(word))
                        if word == "stop":
                            self._announce("Stopping")
                            self.motion.stopMove()
                            self._set_motion_state(S_STOPPED)
                        elif word in ("go", "fetch"):
                            self._announce("Going!")
                            self._set_motion_state(S_WALKING)
            except Exception:
                pass
            time.sleep(VOICE_POLL_SEC)

    #  Game init 

    def _head_sweep(self, dwell=0.25):
        for yaw in HEAD_SCAN_YAWS:
            self._set_head_yaw(yaw)
            time.sleep(dwell)
            yield yaw

    def _full_body_scan(self, label, stop_when=None, max_seconds=None):
        """Scan all around using head sweeps plus body rotation sectors."""
        self._announce(label, priority=True)
        start = time.time()
        step_theta = (2.0 * math.pi) / float(max(1, FULL_SCAN_STEPS))

        for step in range(FULL_SCAN_STEPS):
            for yaw in self._head_sweep():
                if stop_when is not None and stop_when():
                    self.motion.stopMove()
                    self._center_head()
                    return True
                if max_seconds is not None and time.time() - start >= max_seconds:
                    self.motion.stopMove()
                    self._center_head()
                    return False

            if step < FULL_SCAN_STEPS - 1:
                try:
                    self.motion.moveTo(0.0, 0.0, step_theta)
                except Exception:
                    self.motion.moveToward(0.0, 0.0, 0.25)
                    time.sleep(abs(step_theta) / 0.25)
                    self.motion.stopMove()

        self.motion.stopMove()
        self._center_head()
        return stop_when() if stop_when is not None else True

    def _calibrate_goals(self):
        """Full 360 sweep to find the neon yellow goal behind the robot.
        Does 4 head sweeps separated by 90 degree body turns so the robot
        faces all directions including its own goal.
        """
        self._announce("Scanning for goals.", priority=True)
        if YELLOW_HSV_LOWER is not None and YELLOW_HSV_UPPER is not None:
            print("[CALIB] HSV range: H=[{}-{}], S=[{}-{}], V=[{}-{}]".format(
                YELLOW_HSV_LOWER[0], YELLOW_HSV_UPPER[0],
                YELLOW_HSV_LOWER[1], YELLOW_HSV_UPPER[1],
                YELLOW_HSV_LOWER[2], YELLOW_HSV_UPPER[2]))
        print("[CALIB] RGB target: {}".format(GOAL_YELLOW_RGB))
        print("[CALIB] OpenCV HSV fallback available: {}".format(self._opencv_available))
        if not self._opencv_available and not self._opencv_unavailable_logged:
            print("[CALIB] OpenCV/numpy unavailable on this robot runtime; skipping HSV frame fallback attempts")
            self._opencv_unavailable_logged = True
        # Probe ALMemory for tracker-related keys
        try:
            for prefix in ("ALTracker", "ColorBlob", "color"):
                keys = self.memory.getDataList(prefix)
                if keys:
                    print("[CALIB] ALMemory keys starting with '{}': {}".format(prefix, keys[:5]))
        except Exception as e:
            print("[CALIB] ALMemory key probe failed: {}".format(e))
        sys.stdout.flush()
        # Keep a fixed downward pitch while scanning yellow goal blobs.
        self._set_head_pitch(GOAL_HEAD_PITCH)
        time.sleep(0.3)

        for step in range(4):
            print("[CALIB] sweep {} of 4".format(step + 1))
            sys.stdout.flush()
            self._rgb_stats = {
                "frames": 0,
                "no_yellow": 0,
                "clusters": 0,
                "accept": 0,
                "reject_size": 0,
                "reject_shape": 0,
                "reject_score": 0,
                "reject_dist": 0,
            }
            yaw_hits = 0
            yaw_misses = 0
            for yaw in HEAD_SCAN_YAWS:
                if not self._running:
                    break
                self._set_head_yaw(yaw)
                time.sleep(0.35)
                
                # Check if blob data exists in ALMemory
                raw_data = self.memory.getData("ALTracker/ColorBlobDetected")
                if CALIB_VERBOSE_YAW:
                    print("[CALIB] yaw={:.1f} raw_data_exists={}".format(yaw, raw_data is not None and len(raw_data) > 0 if isinstance(raw_data, list) else raw_data is not None))
                    sys.stdout.flush()
                
                # Try ALTracker blob detection first
                goals = self._detect_goals()
                if goals:
                    yaw_hits += 1
                    print("[CALIB] found {} goals via ALTracker".format(len(goals)))
                    sys.stdout.flush()
                    self._store_goal_world_points(goals)
                    if len(self._goal_candidates_world) >= 2 and self._select_nearest_goal_world_point():
                        self.motion.stopMove()
                        self._center_head()
                        self._announce("Goal locked.", priority=True)
                        return True
                else:
                    yaw_misses += 1
                    # Debug: show raw memory data
                    raw_blobs = self._read_color_blobs()
                    if not raw_blobs and CALIB_VERBOSE_YAW:
                        print("[CALIB] no blobs in ALMemory at yaw={:.1f}".format(yaw))
                        sys.stdout.flush()
                
                # Fall back to frame-based HSV detection
                if self._opencv_available:
                    try:
                        for camera_id in GOAL_CAMERAS:
                            frame_goals = self._detect_goals_from_frame(camera_id)
                            if frame_goals:
                                self._store_goal_world_points(frame_goals)
                                print("[CALIB] found {} goals via HSV frame cam{}".format(len(frame_goals), camera_id))
                                sys.stdout.flush()
                                if len(self._goal_candidates_world) >= 2 and self._select_nearest_goal_world_point():
                                    self.motion.stopMove()
                                    self._center_head()
                                    self._announce("Goal locked.", priority=True)
                                    return True
                    except Exception as e:
                        print("[CALIB] frame detection failed: {}".format(e))
                        sys.stdout.flush()
                else:
                    pass

            print("[CALIB] sweep {} summary: goal_hits={} misses={} candidates={} rgb(frames={} accept={} no_yellow={} clusters={} rejects[s={},sh={},sc={},d={}])".format(
                step + 1,
                yaw_hits,
                yaw_misses,
                len(self._goal_candidates_world),
                self._rgb_stats["frames"],
                self._rgb_stats["accept"],
                self._rgb_stats["no_yellow"],
                self._rgb_stats["clusters"],
                self._rgb_stats["reject_size"],
                self._rgb_stats["reject_shape"],
                self._rgb_stats["reject_score"],
                self._rgb_stats["reject_dist"]))
            sys.stdout.flush()

            if not self._running:
                break
            if step < 3:
                try:
                    self.motion.moveTo(0.0, 0.0, math.pi / 2.0)
                except Exception:
                    self._log_ignored_exception("goal calibration body turn failed")

        self.motion.stopMove()
        self._center_head()
        if len(self._goal_candidates_world) >= 2 and self._select_nearest_goal_world_point():
            self._announce("One goal visible.", priority=True)
            return True

        print("[CALIB] no goals found in full 360 sweep")
        sys.stdout.flush()
        self._announce("No goals found.", priority=True)
        return False

    def _goal_midpoint_azimuth(self):
        if not (self._goal_left and self._goal_right):
            return None
        center_x = (float(self._goal_left[0]) + float(self._goal_right[0])) / 2.0
        error = (center_x - CAMERA_RES_W / 2.0) / (CAMERA_RES_W / 2.0)
        return error * (CAMERA_FOV_H / 2.0)

    def _center_on_goals(self):
        """Turn until the detected goal midpoint is near camera center."""
        self._announce("Centering on goals.", priority=True)
        deadline = time.time() + 3.0

        while self._running and time.time() < deadline:
            target_azi = self._goal_midpoint_azimuth()
            if target_azi is None:
                self.motion.moveToward(0.0, 0.0, 0.2)
                time.sleep(0.2)
                continue
            if abs(target_azi) < 0.08:
                break
            theta = max(-TURN_THETA * 0.45, min(TURN_THETA * 0.45, target_azi * 1.5))
            self.motion.moveToward(0.0, 0.0, theta)
            time.sleep(0.2)

        self.motion.stopMove()

    def _find_ball_for_lineup(self):
        """Scan all around until the red ball is visible."""
        found = [None]
        confirmed_hits = [0]
        start = time.time()
        self._set_camera(BALL_CAMERA)
        # Tilt head down, the ball is on the ground
        self._set_head_pitch(BALL_HEAD_PITCH)
        time.sleep(0.2)
        try:
            # Clear stale ALMemory hit so lineup does not immediately "find" an old ball.
            self.memory.insertData("redBallDetected", [])
        except Exception:
            pass
        self._setup_red_ball_blob_detection()

        def seen_ball():
            result = self._read_ball()
            if result is None:
                confirmed_hits[0] = 0
                return False
            found[0] = result
            confirmed_hits[0] += 1
            if confirmed_hits[0] < max(1, BALL_CONFIRM_HITS):
                print("[BALL] candidate seen; waiting for confirmation {}/{}".format(
                    confirmed_hits[0], max(1, BALL_CONFIRM_HITS)))
                sys.stdout.flush()
                return False
            if not self._ball_visible:
                print("[BALL] confirmed detection")
                self._ball_visible = True
            return True

        max_seconds = LINEUP_BALL_TIMEOUT if LINEUP_BALL_TIMEOUT > 0 else BALL_SCAN_SECONDS
        cycle = 0
        while self._running and (time.time() - start) < max_seconds:
            cycle += 1
            print("[BALL] lineup scan cycle {} of {}".format(cycle, BALL_SCAN_CYCLES))
            sys.stdout.flush()
            for yaw in HEAD_SCAN_YAWS:
                if not self._running or (time.time() - start) >= max_seconds:
                    break
                self._set_head_yaw(yaw)
                time.sleep(0.35)
                raw = self._read_ball()
                if raw is not None:
                    found[0] = raw
                    confirmed_hits[0] += 1
                    if confirmed_hits[0] >= max(1, BALL_CONFIRM_HITS):
                        if not self._ball_visible:
                            print("[BALL] confirmed detection")
                            self._ball_visible = True
                        return found[0]
                else:
                    confirmed_hits[0] = 0
                # Candidate seen but not yet confirmed, so: recheck in-place before moving head
                if confirmed_hits[0] > 0:
                    time.sleep(0.25)
                    if seen_ball():
                        return found[0]
            if cycle >= BALL_SCAN_CYCLES:
                break
            try:
                t = threading.Thread(target=self.motion.moveTo, args=(0.0, 0.0, math.pi / 2.0))
                t.daemon = True
                t.start()
                t.join(timeout=4.0)
            except Exception:
                pass
            time.sleep(0.2)
            post_raw = self._read_ball()
            print("[BALL] post-rotate raw={}".format(post_raw))
            sys.stdout.flush()
            if post_raw is not None:
                found[0] = post_raw
                confirmed_hits[0] += 1
                if confirmed_hits[0] >= max(1, BALL_CONFIRM_HITS):
                    print("[BALL] confirmed post-rotate, returning {}".format(found[0]))
                    sys.stdout.flush()
                    return found[0]
            else:
                confirmed_hits[0] = 0
            self._announce("Still looking for red ball.", priority=True)
        print("[BALL] scan exhausted, found[0]={}".format(found[0]))
        sys.stdout.flush()
        return None

    def _face_ball(self, timeout=3.0):
        """Rotate until the red ball is centered in the camera."""
        deadline = time.time() + timeout
        last_ball = None

        while self._running and time.time() < deadline:
            ball = self._read_ball()
            if ball is None:
                self.motion.moveToward(0.0, 0.0, 0.2)
                time.sleep(0.2)
                continue

            last_ball = ball
            ball_azi, ball_dist = ball
            if abs(ball_azi) < 0.08:
                self.motion.stopMove()
                return ball

            theta = max(-TURN_THETA * 0.5, min(TURN_THETA * 0.5, ball_azi * 1.5))
            self.motion.moveToward(0.0, 0.0, theta)
            time.sleep(0.2)

        self.motion.stopMove()
        return last_ball

    def _move_between_goal_and_ball(self):
        """Use local geometry to move to a point between goal and red ball."""
        ball = self._find_ball_for_lineup()
        if ball is None:
            print("[LINEUP] ball not found after full scan")
            sys.stdout.flush()
            self._announce("I cannot see the red ball.", priority=True)
            return False
        print("[LINEUP] ball found: {}".format(ball))
        sys.stdout.flush()

        self._announce("Lining up.", priority=True)

        # Re-detect goal from current pose so world coords match the current odometry frame.
        self._goal_candidates_world = []
        self._setup_goal_blob_detection()
        time.sleep(0.3)
        for _yaw in HEAD_SCAN_YAWS:
            self._set_head_yaw(_yaw)
            time.sleep(0.35)
            _goals = self._detect_goals()
            if _goals:
                self._store_goal_world_points(_goals)
        self._center_head()
        if not self._select_nearest_goal_world_point():
            print("[LINEUP] goal re-detect failed, using stale position")
            sys.stdout.flush()
        self._setup_red_ball_blob_detection()
        time.sleep(0.2)

        goal_world = self._goal_world_point()
        if goal_world is None:
            print("[LINEUP] _goal_world_point is None - goals were never detected")
            sys.stdout.flush()
            self._announce("Goal geometry missing.", priority=True)
            return False

        ball = self._face_ball(timeout=2.0) or ball
        ball_world = self._ball_world_point(ball)
        goal_to_ball_x = ball_world[0] - goal_world[0]
        goal_to_ball_y = ball_world[1] - goal_world[1]
        goal_to_ball_len = math.hypot(goal_to_ball_x, goal_to_ball_y)
        print("[LINEUP] goal={} ball_world={} dist={:.2f}".format(goal_world, ball_world, goal_to_ball_len))
        sys.stdout.flush()
        if goal_to_ball_len < 0.2:
            self._announce("Goal and ball too close.", priority=True)
            return False

        ux = goal_to_ball_x / goal_to_ball_len
        uy = goal_to_ball_y / goal_to_ball_len
        standoff = min(LINEUP_BALL_DIST, max(0.25, goal_to_ball_len * 0.75))
        target_world = (
            ball_world[0] - ux * standoff,
            ball_world[1] - uy * standoff
        )

        pose = self._robot_pose()
        target_robot = _world_to_robot(target_world, pose)
        face_world = math.atan2(
            ball_world[1] - target_world[1],
            ball_world[0] - target_world[0]
        )
        dtheta = _angle_norm(face_world - pose[2])

        self._announce("Moving between goal and ball.", priority=True)
        print("[LINEUP] goal={} ball={} target={} move=({:.2f}, {:.2f}, {:.2f})".format(
            goal_world, ball_world, target_world, target_robot[0], target_robot[1], dtheta))
        sys.stdout.flush()

        try:
            self.motion.moveTo(target_robot[0], target_robot[1], dtheta)
        except Exception as e:
            print("WARNING: geometric lineup move failed: {}".format(e))
            return False

        self._face_ball(timeout=2.0)
        return True

    def _lineup(self):
        """Move between the goal and red ball, then wait for start signal."""
        self._set_game_state(GAME_LINEUP)
        # Stop the background scanner - goal calibration is done.
        try:
            self._stop_goal_scanner()
        except Exception:
            pass
        # Pre-switch blob to red ball mode and let it settle for one frame
        # before the ball scan starts, so the first read isn't stale.
        try:
            self._setup_red_ball_blob_detection()
            time.sleep(0.3)
        except Exception:
            pass
        lined_up = self._move_between_goal_and_ball()
        self.motion.stopMove()
        self._center_head()
        if not lined_up:
            self._announce("Lineup failed.", priority=True)
            return
        if self._asr_ready:
            try:
                self.memory.insertData("WordRecognized", ["", 0.0])
            except Exception:
                pass
            self._last_word = None
            self._last_word_time = time.time()
            self._voice_ready_at = self._last_word_time + 0.5
            try:
                self.asr.pause(False)
            except Exception:
                pass
        self._announce("Ready. Say go to start.", priority=True)
        self._set_motion_state(S_STOPPED)

    #  Strategy & movement loop 

    def _decide_strategy(self, ball_pos, opponent_pos):
        """
        Strategy based on ball motion and opponent proximity.

        If ball is MOVING (rolling toward goal):
          Both robots intercept predicted trajectory (run to where it's going)

        If ball is STOPPED:
          - Opponent closer: STALK (block perpendicularly between opponent & goal)
          - We're closer:    CHARGE (run toward ball/goal)

        If ball in KICK range: KICK

        opponent_pos = (distance_m, azimuth_rad) from blob detection
        ball_pos = (azimuth_rad, distance_m) from ball detector
        Returns: (strategy_name, target_azimuth, target_speed)
        """
        if ball_pos is None:
            self._ball_pos_prev = None
            self._ball_vel = None
            self._ball_moving = False
            # SEARCH: pivot in place, owl-style head scan (no forward movement)
            # Sweep theta direction cycles between +0.5 and -0.5
            self._search_theta *= -1 if abs(self._search_theta) > 0.8 else 1.0
            return ("search", self._search_theta, 0.0)  # (strategy, azimuth_for_sweep, speed=0)

        ball_azi, ball_dist = ball_pos

        # Estimate ball velocity (simple: compare to previous frame)
        # If ball distance is changing significantly, it's moving
        self._ball_moving = False
        if self._ball_pos_prev is not None:
            prev_azi, prev_dist = self._ball_pos_prev
            dist_delta = ball_dist - prev_dist
            # If distance decreased rapidly, ball is moving toward us
            # If distance stayed roughly same but azimuth changed, ball is rolling sideways
            if abs(dist_delta) > 0.05:  # >5cm per frame = moving
                self._ball_moving = True
                self._ball_vel = (ball_azi - prev_azi, dist_delta)
        self._ball_pos_prev = ball_pos

        # KICK range (always highest priority when in range)
        if ball_dist < KICK_DIST:
            return ("kick", ball_azi, 0.0)

        # Opponent position
        if opponent_pos:
            opp_dist, opp_azi = opponent_pos
        else:
            opp_dist = None
            opp_azi = 0.0

        # Ball MOVING: intercept trajectory (both robots)
        if self._ball_moving:
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

        # We are close enough to trap/line up: shimmy around the ball for a shot.
        if ball_dist < 0.35:
            self._shooting = True
            return ("shoot", -0.3, WALK_VX * 0.3)

        self._shooting = False

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

            #  Sonar safety: always active 
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

            if self._wall_detected:
                was_avoiding = True
                self.motion.moveToward(WALK_VX * 0.15, 0.0, TURN_THETA)
                time.sleep(POLL_SEC)
                continue

            #  Strategy-driven movement 
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

            if strategy in ("charge", "stalk", "search", "shoot"):
                theta = max(-TURN_THETA, min(TURN_THETA, target_azi * 1.5))
            else:
                theta = ARC_THETA * self._arc_sign

            self.motion.moveToward(target_speed, 0.0, theta)
            time.sleep(POLL_SEC)

    #  Entry point 

    def run(self):
        try:
            self.setup()

            # Spin up threads
            vt = threading.Thread(target=self._vision_thread, name="vision")
            vt.daemon = True
            vt.start()

            vot = threading.Thread(target=self._voice_thread, name="voice")
            vot.daemon = True
            vot.start()

            # Game init sequence
            max_calib_attempts = int(os.environ.get("MAX_CALIB_ATTEMPTS", "3"))
            calib_attempts = 0
            calibration_succeeded = False
            while self._running and calib_attempts < max_calib_attempts:
                if self._calibrate_goals():
                    calibration_succeeded = True
                    break
                print("[GOAL] calibration attempt {} of {}; press Ctrl-C to stop.".format(
                    calib_attempts + 1, max_calib_attempts))
                sys.stdout.flush()
                time.sleep(1.0)
                calib_attempts += 1
            if not calibration_succeeded:
                print("[GOAL] Goal calibration failed after {} attempts. Aborting lineup.".format(
                    max_calib_attempts))
                sys.stdout.flush()
                self._announce("Calibration failed. Cannot proceed without goal detection.", priority=True)
                return

            max_lineup_attempts = int(os.environ.get("MAX_LINEUP_ATTEMPTS", "3"))
            lineup_succeeded = False
            for lineup_attempt in range(1, max_lineup_attempts + 1):
                if not self._running:
                    break
                print("[LINEUP] attempt {} of {}".format(lineup_attempt, max_lineup_attempts))
                sys.stdout.flush()
                self._lineup()
                if self._get_motion_state() == S_STOPPED:
                    lineup_succeeded = True
                    break
                print("[LINEUP] attempt {} failed; retrying...".format(lineup_attempt))
                sys.stdout.flush()
                time.sleep(1.0)
            if not lineup_succeeded:
                print("[LINEUP] all {} attempts failed. Aborting.".format(max_lineup_attempts))
                sys.stdout.flush()
                self._announce("Lineup failed completely.", priority=True)
                return
            while self._running and self._get_motion_state() != S_WALKING:
                time.sleep(0.5)
            self._set_game_state(GAME_PLAYING)
            self._setup_robot_blob_detection()
            print("Game started!")

            # Movement loop
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
    # Create instance and install SIGINT handler so Ctrl-C cleanly shuts down
    _soccer = Soccer1v1()

    def _handle_sigint(sig, frame):
        try:
            print("\nSIGINT received, shutting down...")
        except Exception:
            pass
        try:
            _soccer._running = False
            _soccer.shutdown()
        except Exception:
            pass
        # exit with 130 to match shell convention for SIGINT
        try:
            sys.exit(130)
        except Exception:
            os._exit(130)

    signal.signal(signal.SIGINT, _handle_sigint)
    try:
        signal.signal(signal.SIGTERM, _handle_sigint)
    except Exception:
        pass
    _soccer.run()
