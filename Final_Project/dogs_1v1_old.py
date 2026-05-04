#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
"""
NAO 1v1 Soccer, recovered from the original two-player game.
Two robots, one red ball, two neon-yellow goals, one field.

Vision detection:
  - Built-in color blob detection for opponent/goal/wall hints
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
import subprocess

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

# Connection
ROBOT_NAMES = {
    1: "Suarez",
    2: "Messi",
}

ROBOT_IPS = {
    1: "172.16.0.29",
    2: "172.16.0.6",
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
        children.append(subprocess.Popen([sys.executable, script], env=env))

    try:
        while True:
            running = [p for p in children if p.poll() is None]
            if not running:
                return max([p.returncode or 0 for p in children])
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopping both 1v1 controllers...")
        for child in children:
            if child.poll() is None:
                child.terminate()
        time.sleep(1.0)
        for child in children:
            if child.poll() is None:
                child.kill()
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
GOAL_SCAN_SECONDS = float(os.environ.get("GOAL_SCAN_SECONDS", "0.0"))
LINEUP_BALL_DIST = float(os.environ.get("LINEUP_BALL_DIST", "1.10"))
LINEUP_BALL_TIMEOUT = float(os.environ.get("LINEUP_BALL_TIMEOUT", "0.0"))
FULL_SCAN_STEPS = int(os.environ.get("FULL_SCAN_STEPS", "3"))

# ALColorBlobDetection tracks one RGB target color at a time.
GOAL_YELLOW_RGB = (
    int(os.environ.get("GOAL_YELLOW_R", "230")),
    int(os.environ.get("GOAL_YELLOW_G", "255")),
    int(os.environ.get("GOAL_YELLOW_B", "0")),
)
ROBOT_WHITE_RGB = (
    int(os.environ.get("ROBOT_WHITE_R", "245")),
    int(os.environ.get("ROBOT_WHITE_G", "245")),
    int(os.environ.get("ROBOT_WHITE_B", "245")),
)
RED_BALL_RGB = (
    int(os.environ.get("RED_BALL_R", "220")),
    int(os.environ.get("RED_BALL_G", "0")),
    int(os.environ.get("RED_BALL_B", "0")),
)
COLOR_THRESHOLD = int(os.environ.get("COLOR_THRESHOLD", "70"))
GOAL_MIN_SIZE = int(os.environ.get("GOAL_MIN_SIZE", "50"))
ROBOT_MIN_SIZE = int(os.environ.get("ROBOT_MIN_SIZE", "80"))
RED_BALL_MIN_SIZE = int(os.environ.get("RED_BALL_MIN_SIZE", "80"))
GOAL_DEBUG = int(os.environ.get("GOAL_DEBUG", "1"))
YELLOW_HSV_LOWER = np.array([
    int(os.environ.get("GOAL_HSV_H_LOW", "20")),
    int(os.environ.get("GOAL_HSV_S_LOW", "80")),
    int(os.environ.get("GOAL_HSV_V_LOW", "80")),
]) if np is not None else None
YELLOW_HSV_UPPER = np.array([
    int(os.environ.get("GOAL_HSV_H_HIGH", "80")),
    int(os.environ.get("GOAL_HSV_S_HIGH", "255")),
    int(os.environ.get("GOAL_HSV_V_HIGH", "255")),
]) if np is not None else None
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

        try:
            self.asr.pause(True)
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
            self.asr.pause(True)
            self._asr_ready = True
        except Exception as e:
            self._asr_ready = False
            print("ERROR: ASR setup failed: {}".format(e))
            raise RuntimeError("ASR setup failed")

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
        try:
            self.color_blob = ALProxy("ALColorBlobDetection", ROBOT_IP, ROBOT_PORT)
            try:
                self.color_blob.unsubscribe(COLOR_BLOB_SUBSCRIPTION)
            except Exception:
                pass
            self.color_blob.subscribe(COLOR_BLOB_SUBSCRIPTION)
            self._setup_goal_blob_detection()
        except Exception as e:
            print("WARNING: Could not setup color blob detection: {}".format(e))

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
        try:
            self.color_blob.unsubscribe(COLOR_BLOB_SUBSCRIPTION)
        except Exception:
            pass
        for client in self._video_clients.values():
            try:
                self.video.unsubscribe(client)
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
        except Exception:
            pass

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
        try:
            data = self.memory.getData("redBallDetected")
            if data and len(data) > 1 and len(data[1]) > 0:
                b = data[1][0]
                if not self._ball_visible:
                    print("[BALL] detected")
                    self._ball_visible = True
                self._ball_miss_count = 0
                return float(b[0]), float(b[2])
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
                point = _robot_to_world(_polar_to_xy(x_ang, dist), pose)
                self._goal_candidates_world.append((dist, area, point))

    def _select_nearest_goal_world_point(self):
        if not self._goal_candidates_world:
            return False
        dist, area, point = min(self._goal_candidates_world, key=lambda g: g[0])
        self._goal_points_world = [point]
        print("[GOAL] selected nearest candidate dist={:.2f} area={:.0f} point=({:.2f}, {:.2f}) from {} samples".format(
            dist, area, point[0], point[1], len(self._goal_candidates_world)))
        sys.stdout.flush()
        return True

    def _goal_world_point(self):
        if not self._goal_points_world:
            return None
        x = sum(p[0] for p in self._goal_points_world) / float(len(self._goal_points_world))
        y = sum(p[1] for p in self._goal_points_world) / float(len(self._goal_points_world))
        return x, y

    def _setup_goal_blob_detection(self):
        """Configure ALColorBlobDetection for neon-yellow goal blobs."""
        try:
            if self._color_blob_mode == "goal":
                return
            self.color_blob.setColor(
                GOAL_YELLOW_RGB[0],
                GOAL_YELLOW_RGB[1],
                GOAL_YELLOW_RGB[2],
                COLOR_THRESHOLD
            )
            self.color_blob.setObjectProperties(GOAL_MIN_SIZE, 0.35, "Unknown")
            try:
                self.color_blob.setAutoExposure(True)
            except Exception:
                pass
            self._color_blob_mode = "goal"
            print("[VISION] Color blob target: neon yellow goals rgb={} threshold={}".format(
                GOAL_YELLOW_RGB, COLOR_THRESHOLD))
        except Exception as e:
            print("WARNING: goal blob setup failed: {}".format(e))

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
            blobs = self.memory.getData("ColorBlobDetection/blobs")
            if blobs:
                return blobs
        except Exception:
            pass
        return []

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
        try:
            mask = cv2.inRange(frame, YELLOW_HSV_LOWER, YELLOW_HSV_UPPER)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        except ValueError:
            _, contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        goals = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < GOAL_MIN_SIZE:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if h < 4:
                continue
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            x_ang = ((float(cx) - CAMERA_RES_W / 2.0) /
                     (CAMERA_RES_W / 2.0)) * (CAMERA_FOV_H / 2.0)
            dist = (0.35 * CAMERA_FOCAL_LENGTH) / float(h)
            if 0.15 < dist < 6.0:
                goals.append((cx, cy, area, x_ang, dist))

        if GOAL_DEBUG:
            print("[GOAL] frame camera={} candidates={}".format(camera_id, len(goals)))
            sys.stdout.flush()
        return sorted(goals, key=lambda g: g[0])

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
        """Detect likely yellow goal blobs from ALColorBlobDetection."""
        try:
            self._setup_goal_blob_detection()
            blobs = self._read_color_blobs()
            if not blobs:
                self._debug_goal_blobs([], 0, 0)
                return self._detect_goals_from_frame(self._active_camera)
            goals = []
            rejected = 0
            for blob in blobs:
                if len(blob) >= 7:
                    cx, cy, w, h, x_ang, dist = blob[0], blob[1], blob[2], blob[3], blob[4], blob[6]
                    area = w * h
                    if 0.15 < dist < 6.0 and area >= GOAL_MIN_SIZE:
                        goals.append((cx, cy, area, x_ang, dist))
                    else:
                        rejected += 1
            self._debug_goal_blobs(blobs, len(goals), rejected)
            if goals:
                return sorted(goals, key=lambda g: g[0])
            return self._detect_goals_from_frame(self._active_camera)
        except Exception:
            return []

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
                        cx, cy, area, x_ang, dist = goals[0]
                        self._goal_left = (640 - cx, cy)
                        self._goal_right = (cx, cy)
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
        """Scan all around, then lock onto the nearest visible goal."""
        self._setup_goal_blob_detection()
        self._goal_points_world = []
        self._goal_candidates_world = []

        self._announce("Scanning all around for goals.", priority=True)
        start = time.time()
        step_theta = (2.0 * math.pi) / float(max(1, FULL_SCAN_STEPS))

        for step in range(FULL_SCAN_STEPS):
            for camera_id in GOAL_CAMERAS:
                self._set_camera(camera_id)
                print("[GOAL] scanning camera {}".format(camera_id))
                sys.stdout.flush()
                time.sleep(0.15)
                for yaw in self._head_sweep(dwell=0.35):
                    goals = self._detect_goals()
                    if goals:
                        self._store_goal_world_points(goals)
                        nearest = min(goals, key=lambda g: g[4])
                        print("[GOAL] candidate camera={} yaw={:.2f} dist={:.2f} area={:.0f} center=({:.0f}, {:.0f})".format(
                            camera_id, yaw, nearest[4], nearest[2], nearest[0], nearest[1]))
                        sys.stdout.flush()
                    if GOAL_SCAN_SECONDS > 0 and time.time() - start >= GOAL_SCAN_SECONDS:
                        break
                if GOAL_SCAN_SECONDS > 0 and time.time() - start >= GOAL_SCAN_SECONDS:
                    break
            if GOAL_SCAN_SECONDS > 0 and time.time() - start >= GOAL_SCAN_SECONDS:
                break
            if step < FULL_SCAN_STEPS - 1:
                try:
                    self.motion.moveTo(0.0, 0.0, step_theta)
                except Exception:
                    self.motion.moveToward(0.0, 0.0, 0.25)
                    time.sleep(abs(step_theta) / 0.25)
                    self.motion.stopMove()

        self.motion.stopMove()
        self._center_head()

        if self._select_nearest_goal_world_point():
            self._announce("Nearest goal locked.", priority=True)
            return True

        self._announce("I cannot see a goal.", priority=True)
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
        self._set_camera(BALL_CAMERA)
        self._setup_red_ball_blob_detection()

        def seen_ball():
            found[0] = self._read_ball() or self._detect_red_ball_blob()
            if found[0] is not None and not self._ball_visible:
                print("[BALL] detected by red blob")
                self._ball_visible = True
            return found[0] is not None

        max_seconds = LINEUP_BALL_TIMEOUT if LINEUP_BALL_TIMEOUT > 0 else None
        while self._running:
            if self._full_body_scan(
                    "Scanning all around for red ball.",
                    stop_when=seen_ball,
                    max_seconds=max_seconds):
                return found[0]
            if max_seconds is not None:
                return None
            self._announce("Still looking for red ball.", priority=True)
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
            self._announce("I cannot see the red ball.", priority=True)
            return False

        goal_world = self._goal_world_point()
        if goal_world is None:
            self._announce("Goal geometry missing.", priority=True)
            return False

        ball = self._face_ball(timeout=2.0) or ball
        ball_world = self._ball_world_point(ball)
        goal_to_ball_x = ball_world[0] - goal_world[0]
        goal_to_ball_y = ball_world[1] - goal_world[1]
        goal_to_ball_len = math.hypot(goal_to_ball_x, goal_to_ball_y)
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
        self._announce("Lining up.", priority=True)
        self._move_between_goal_and_ball()
        self.motion.stopMove()
        self._center_head()
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
            while self._running and not self._calibrate_goals():
                print("[GOAL] calibration retrying; press Ctrl-C to stop.")
                sys.stdout.flush()
                time.sleep(1.0)

            self._lineup()

            # Wait for "go" command to enter PLAYING state
            print("Waiting for 'go' command...")
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
    Soccer1v1().run()
