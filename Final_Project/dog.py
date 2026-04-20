#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
"""
NAO Soccer-Dog Brain
One unified behaviour: forward drive + sonar avoidance + ball seeking.

Movement priority (highest to lowest):
  1. DANGER sonar     -- stop, Roomba-spin until clear, resume
  2. Ball visible     -- steer azimuth toward ball; kick at KICK_DIST
                         (sonar danger still overrides mid-approach)
  3. OBSTACLE sonar   -- hard curve + slow
  4. WARN sonar       -- preemptive curve (blended strength)
  5. CLEAR            -- gentle alternating arc

Voice is a human safety override only:
  "go" / "fetch"  -- resume after a stop command
  "stop"          -- pause all motion (sonar danger resumes automatically)
"""
import os
import sys
import time
import threading

sdk_folder = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib/python2.7/site-packages"
sys.path.append(sdk_folder)
os.environ["DYLD_LIBRARY_PATH"] = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib"

from naoqi import ALProxy

# ── Connection ────────────────────────────────────────────────────────────────
ROBOT_IP   = "172.16.0.29"
ROBOT_PORT = 9559

# ── Sonar thresholds (metres) ─────────────────────────────────────────────────
WARN_DIST    = 0.80   # start preemptive curve
OBS_DIST     = 0.55   # commit to hard curve + slow
DANGER_DIST  = 0.32   # stop and spin (Roomba)
CLEAR_DIST   = 1.00   # threshold to exit Roomba spin

# ── Walking velocities (normalised 0-1) ───────────────────────────────────────
WALK_VX    = 0.55   # forward speed in clear zone
ARC_THETA  = 0.12   # gentle arc magnitude in clear zone
TURN_THETA = 0.75   # theta during Roomba spin / hard curve

# ── Ball seeking ──────────────────────────────────────────────────────────────
KICK_DIST = 0.25   # metres -- trigger kick posture

# ── Voice recognition ─────────────────────────────────────────────────────────
VOCABULARY       = ["go", "stop", "fetch"]
VOICE_CONFIDENCE = 0.38
VOICE_POLL_SEC   = 0.15

# ── Control loop ──────────────────────────────────────────────────────────────
POLL_SEC = 0.10

# ── States ────────────────────────────────────────────────────────────────────
S_STOPPED = "stopped"
S_WALKING = "walking"
S_TURNING = "turning"


def _open_side(left_m, right_m):
    """Return +1.0 (turn left) or -1.0 (turn right) toward the clearer side."""
    return 1.0 if left_m > right_m else -1.0


class Brain(object):

    def __init__(self):
        self._state     = S_STOPPED
        self._lock      = threading.Lock()
        self._running   = True
        self._arc_sign  = 1       # alternates after each Roomba turn
        self._last_word = None    # deduplicate ASR events

        print("Connecting to NAO at {}:{}...".format(ROBOT_IP, ROBOT_PORT))
        self.motion  = ALProxy("ALMotion",            ROBOT_IP, ROBOT_PORT)
        self.posture = ALProxy("ALRobotPosture",      ROBOT_IP, ROBOT_PORT)
        self.tts     = ALProxy("ALTextToSpeech",      ROBOT_IP, ROBOT_PORT)
        self.memory  = ALProxy("ALMemory",            ROBOT_IP, ROBOT_PORT)
        self.sonar   = ALProxy("ALSonar",             ROBOT_IP, ROBOT_PORT)
        self.asr     = ALProxy("ALSpeechRecognition", ROBOT_IP, ROBOT_PORT)
        self.ball    = None   # ALRedBallDetection -- wired up in setup()
        print("Connected.")

    # ── Setup / teardown ──────────────────────────────────────────────────────

    def setup(self):
        self.motion.wakeUp()
        self.motion.setStiffnesses("Body", 1.0)
        self.posture.goToPosture("StandInit", 0.6)

        self.sonar.subscribe("NaoBrain")
        time.sleep(0.4)

        try:
            self.asr.unsubscribe("NaoBrain")
        except Exception:
            pass
        self.asr.setLanguage("English")
        self.asr.setVocabulary(VOCABULARY, False)
        self.asr.subscribe("NaoBrain")

        try:
            self.ball = ALProxy("ALRedBallDetection", ROBOT_IP, ROBOT_PORT)
            self.ball.subscribe("NaoBrain")
            print("Ball detection active.")
        except Exception as e:
            print("Ball detection unavailable: " + str(e))

        self.tts.say("Ready.")

    def shutdown(self):
        self._running = False
        try:
            self.motion.stopMove()
        except Exception:
            pass
        for proxy in (self.asr, self.sonar, self.ball):
            if proxy is not None:
                try:
                    proxy.unsubscribe("NaoBrain")
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

    # ── State ─────────────────────────────────────────────────────────────────

    def _set_state(self, s):
        with self._lock:
            self._state = s

    def _get_state(self):
        with self._lock:
            return self._state

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
        """Returns (azimuth_rad, distance_m) or None. azimuth > 0 = ball left of centre."""
        if self.ball is None:
            return None
        try:
            data = self.memory.getData("redBallDetected")
            # [timestamp, [[azimuth, elevation, distance, ...]], ...]
            if data and len(data) > 1 and len(data[1]) > 0:
                b = data[1][0]
                return float(b[0]), float(b[2])
        except Exception:
            pass
        return None

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
                            self._set_state(S_STOPPED)
                        elif word in ("go", "fetch"):
                            self.tts.post.say("Going!")
                            self._set_state(S_WALKING)
            except Exception:
                pass
            time.sleep(VOICE_POLL_SEC)

    # ── Movement loop ─────────────────────────────────────────────────────────

    def _movement_loop(self):
        while self._running:
            if self._get_state() != S_WALKING:
                time.sleep(POLL_SEC)
                continue

            left_m, right_m = self._read_sonar()
            min_dist = min(left_m, right_m)

            # ── 1. DANGER: Roomba spin until clear ────────────────────────────
            if min_dist < DANGER_DIST:
                self.motion.stopMove()
                self._set_state(S_TURNING)
                spin_dir = _open_side(left_m, right_m)
                self.motion.moveToward(0.0, 0.0, spin_dir * TURN_THETA)
                self.tts.post.say("Obstacle!")

                while self._running and self._get_state() == S_TURNING:
                    l, r = self._read_sonar()
                    if min(l, r) >= CLEAR_DIST:
                        break
                    time.sleep(POLL_SEC)

                self._arc_sign = int(spin_dir)
                self.motion.stopMove()
                time.sleep(0.15)
                self._set_state(S_WALKING)
                continue

            # ── 2. Ball visible: steer toward it ─────────────────────────────
            ball = self._read_ball()
            if ball is not None:
                azimuth, dist_m = ball

                if dist_m < KICK_DIST:
                    self.motion.stopMove()
                    self.tts.post.say("Kick!")
                    # TODO: replace with actual kick joint motion
                    self.posture.goToPosture("StandInit", 0.8)
                    time.sleep(1.2)
                    self._set_state(S_WALKING)
                    continue

                steer_theta = max(-TURN_THETA, min(TURN_THETA, azimuth * 1.5))
                approach_speed = WALK_VX * min(1.0, dist_m / 0.5)
                self.motion.moveToward(approach_speed, 0.0, steer_theta)
                time.sleep(POLL_SEC)
                continue

            # ── 3. OBSTACLE: hard curve + slow ────────────────────────────────
            if min_dist < OBS_DIST:
                steer = _open_side(left_m, right_m)
                self.motion.moveToward(WALK_VX * 0.35, 0.0, steer * TURN_THETA * 0.65)
                time.sleep(POLL_SEC)
                continue

            # ── 4. WARN: preemptive blend curve ───────────────────────────────
            if min_dist < WARN_DIST:
                steer = _open_side(left_m, right_m)
                blend = 1.0 - (min_dist - OBS_DIST) / (WARN_DIST - OBS_DIST)
                self.motion.moveToward(
                    WALK_VX * (0.6 + 0.4 * (1.0 - blend)),
                    0.0,
                    steer * ARC_THETA * (1.0 + blend * 3.0)
                )
                time.sleep(POLL_SEC)
                continue

            # ── 5. CLEAR: gentle alternating arc ──────────────────────────────
            self.motion.moveToward(WALK_VX, 0.0, ARC_THETA * self._arc_sign)
            time.sleep(POLL_SEC)

    # ── Entry point ───────────────────────────────────────────────────────────

    def run(self):
        self.setup()

        threading.Thread(target=self._voice_thread,  name="voice").start()
        threading.Thread(target=self._movement_loop, name="move").start()

        print("Say 'go' to start, 'stop' to pause.  Ctrl-C to quit.")
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nCtrl-C received.")
        finally:
            self.shutdown()


if __name__ == "__main__":
    Brain().run()
