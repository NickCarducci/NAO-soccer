#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
import os
import sys
import time
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

ARMS_DOWN = [1.5, 0.18, 1.5, -0.18]
ARMS_UP = [0.2, 0.45, 0.2, -0.45]

HEAD_CENTER = [0.0, -0.10]
HEAD_DOWN = [0.0, 0.20]
HEAD_LEFT = [0.45, -0.10]
HEAD_RIGHT = [-0.45, -0.10]

SPOKEN_LINES = [
    "Nodding, waving, and speaking.",
    "The whole package.",
    "I'm more real than ever.",
]


def arm_motion_loop():
    for _ in range(4):
        motion.angleInterpolation(ARM_JOINTS, ARMS_UP, 0.7, True)
        time.sleep(0.15)
        motion.angleInterpolation(ARM_JOINTS, ARMS_DOWN, 0.7, True)
        time.sleep(0.15)


def head_motion_loop():
    for index in range(4):
        if index % 2 == 0:
            motion.angleInterpolation(HEAD_JOINTS, HEAD_DOWN, 0.45, True)
            motion.angleInterpolation(HEAD_JOINTS, HEAD_CENTER, 0.45, True)
        else:
            motion.angleInterpolation(HEAD_JOINTS, HEAD_LEFT, 0.45, True)
            motion.angleInterpolation(HEAD_JOINTS, HEAD_RIGHT, 0.45, True)
            motion.angleInterpolation(HEAD_JOINTS, HEAD_CENTER, 0.35, True)
        time.sleep(0.1)


def speech_loop():
    for line in SPOKEN_LINES:
        tts.say(line)
        time.sleep(0.4)


posture = None
motion = None
tts = None

try:
    posture = ALProxy("ALRobotPosture", ROBOT_IP, ROBOT_PORT)
    motion = ALProxy("ALMotion", ROBOT_IP, ROBOT_PORT)
    tts = ALProxy("ALTextToSpeech", ROBOT_IP, ROBOT_PORT)

    posture.goToPosture("Stand", 0.5)
    motion.setStiffnesses("Body", 1.0)
    motion.angleInterpolation(ARM_JOINTS, ARMS_DOWN, 0.6, True)
    motion.angleInterpolation(HEAD_JOINTS, HEAD_CENTER, 0.4, True)
    tts.say("Let me dance, in layers.")

    arm_thread = threading.Thread(target=arm_motion_loop)
    head_thread = threading.Thread(target=head_motion_loop)
    speech_thread = threading.Thread(target=speech_loop)

    arm_thread.start()
    head_thread.start()
    speech_thread.start()

    arm_thread.join()
    head_thread.join()
    speech_thread.join()

    motion.angleInterpolation(ARM_JOINTS, ARMS_DOWN, 0.6, True)
    motion.angleInterpolation(HEAD_JOINTS, HEAD_CENTER, 0.4, True)
    tts.say("Done.")

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
