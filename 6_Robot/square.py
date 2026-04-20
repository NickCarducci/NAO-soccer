#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
import os
import sys
import time
import math
import threading

# 1. The folder path (MUST end at site-packages)
sdk_folder = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib/python2.7/site-packages"
sys.path.append(sdk_folder)

# 2. Tell the system where the 'binaries' are (the .so files)
os.environ["DYLD_LIBRARY_PATH"] = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib"

try:
    from naoqi import ALProxy
    print("SDK successfully loaded!")
except ImportError as e:
    print("Still cannot find naoqi. Check if this file exists:")
    print(sdk_folder + "/naoqi.py")
    print("Error details: " + str(e))
    sys.exit(1)


ROBOT_IP = "172.16.0.29"
ROBOT_PORT = 9559

ARM_JOINTS = [
    "LShoulderPitch", "LShoulderRoll",
    "RShoulderPitch", "RShoulderRoll",
]
HEAD_JOINTS = ["HeadYaw", "HeadPitch"]
RIGHT_ARM_JOINTS = [
    "RShoulderPitch", "RShoulderRoll", "RElbowYaw",
    "RElbowRoll", "RWristYaw", "RHand",
]

ARMS_DOWN = [1.5, 0.18, 1.5, -0.18]
ARMS_UP = [0.2, 0.45, 0.2, -0.45]

HEAD_CENTER = [0.0, -0.10]
HEAD_DOWN = [0.0, 0.20]
HEAD_LEFT = [0.45, -0.10]
HEAD_RIGHT = [-0.45, -0.10]
RIGHT_ARM_DOWN = [1.55, -0.18, 1.20, 0.45, 0.0, 0.0]
RIGHT_ARM_WAVE_START = [-1.0, -0.10, 1.20, 0.45, 0.0, 1.0]


def speak_side():
    time.sleep(0.5)
    tts.say("Check this out!")
    time.sleep(1.5)
    tts.say("Walking the talk.")


def flash_leds_side(stop_event):
    colors = [0x00FF0000, 0x0000FF00, 0x000000FF, 0x00FFFFFF]
    color_index = 0
    while not stop_event.is_set():
        leds.fadeRGB("AllLeds", colors[color_index], 0.2)
        color_index = (color_index + 1) % len(colors)
        time.sleep(0.1)
    leds.fadeRGB("AllLeds", 0x00000000, 0.2)


def wave_arm_side():
    motion.angleInterpolation(RIGHT_ARM_JOINTS, RIGHT_ARM_WAVE_START, 0.8, True)
    motion.angleInterpolation(
        ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"],
        [
            [-1.0, -1.0, -1.0, -1.0],
            [-0.5, 0.3, -0.5, -0.1],
            [1.20, 1.20, 1.20, 1.20],
            [0.45, 0.45, 0.45, 0.45],
            [0.0, 0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0, 1.0],
        ],
        [
            [0.35, 0.70, 1.05, 1.40],
            [0.35, 0.70, 1.05, 1.40],
            [0.35, 0.70, 1.05, 1.40],
            [0.35, 0.70, 1.05, 1.40],
            [0.35, 0.70, 1.05, 1.40],
            [0.35, 0.70, 1.05, 1.40],
        ],
        True,
    )
    motion.angleInterpolation(RIGHT_ARM_JOINTS, RIGHT_ARM_DOWN, 0.8, True)


def nod_head_side(stop_event):
    while not stop_event.is_set():
        motion.angleInterpolation(HEAD_JOINTS, HEAD_DOWN, 0.35, True)
        if stop_event.is_set():
            break
        motion.angleInterpolation(HEAD_JOINTS, HEAD_CENTER, 0.35, True)
    motion.angleInterpolation(HEAD_JOINTS, HEAD_CENTER, 0.25, True)


def walk_forward_side():
    motion.moveTo(0.3, 0.0, 0.0)


def turn_corner():
    motion.moveTo(0.0, 0.0, math.pi / 2.0)


def run_square():
    side_behaviors = [
        speak_side,
        flash_leds_side,
        wave_arm_side,
        nod_head_side,
    ]
    for index, behavior in enumerate(side_behaviors):
        print("Running side %d" % (index + 1))
        walk_done = threading.Event()

        def walk_and_signal():
            try:
                walk_forward_side()
            finally:
                walk_done.set()

        if behavior in [flash_leds_side, nod_head_side]:
            action_thread = threading.Thread(target=behavior, args=(walk_done,))
        else:
            action_thread = threading.Thread(target=behavior)
        walk_thread = threading.Thread(target=walk_and_signal)

        action_thread.start()
        walk_thread.start()

        action_thread.join()
        walk_thread.join()

        turn_corner()
        time.sleep(0.1)

posture = None
motion = None
tts = None
leds = None

try:
    posture = ALProxy("ALRobotPosture", ROBOT_IP, ROBOT_PORT)
    motion = ALProxy("ALMotion", ROBOT_IP, ROBOT_PORT)
    tts = ALProxy("ALTextToSpeech", ROBOT_IP, ROBOT_PORT)
    leds = ALProxy("ALLeds", ROBOT_IP, ROBOT_PORT)

    posture.goToPosture("Stand", 0.5)
    motion.setStiffnesses("Body", 1.0)
    tts.say("Alright!")
    run_square()
    tts.say("Until next time...")

except KeyboardInterrupt:
    print("Stopped.")
except Exception as e:
    print("Error connecting to robot: ")
    print(e)
    sys.exit(1)
finally:
    if motion is not None:
        try:
            motion.angleInterpolation(ARM_JOINTS, ARMS_DOWN, 0.6, True)
        except Exception:
            pass
        try:
            motion.angleInterpolation(HEAD_JOINTS, HEAD_CENTER, 0.4, True)
        except Exception:
            pass
    if posture is not None:
        try:
            posture.goToPosture("Stand", 0.5)
        except Exception:
            pass
    if motion is not None:
        try:
            motion.setStiffnesses("Body", 0.0)
        except Exception:
            pass
    if leds is not None:
        try:
            leds.fadeRGB("AllLeds", 0x00000000, 0.2)
        except Exception:
            pass
