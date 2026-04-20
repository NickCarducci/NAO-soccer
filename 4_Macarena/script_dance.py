#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
import os
import sys
import time

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


ROBOT_IP = "172.16.0.3"
ROBOT_PORT = 9559

ARM_JOINTS = [
    "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand",
    "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand",
]
HEAD_JOINTS = ["HeadYaw", "HeadPitch"]
FULL_JOINTS = ARM_JOINTS + HEAD_JOINTS

ARMS_DOWN = [
    1.55, 0.18, -1.20, -0.45, 0.0, 0.0,
    1.55, -0.18, 1.20, 0.45, 0.0, 0.0,
]
HEAD_CENTER = [0.0, -0.10]
KEYFRAMES = [
    (
        "arms_up_head_left",
        "First pose. Reach up and lean left.",
        [
            -0.85, 0.42, -0.55, -0.20, 0.0, 1.0,
            -0.85, -0.20, 0.35, 0.15, 0.0, 1.0,
            0.45, -0.18,
        ],
        0.9,
    ),
    (
        "arms_down_head_right",
        "Now drop low and look right.",
        [
            1.20, 0.25, -1.10, -0.55, 0.0, 0.0,
            1.20, -0.25, 1.10, 0.55, 0.0, 0.0,
            -0.45, -0.18,
        ],
        0.9,
    ),
    (
        "funky_left",
        "Hit the funky left side.",
        [
            -0.55, 0.70, -0.90, -0.35, 0.0, 1.0,
            0.10, -0.10, 0.40, 0.20, 0.0, 1.0,
            0.25, -0.22,
        ],
        0.8,
    ),
    (
        "funky_right",
        "And swing it back to the right.",
        [
            0.10, 0.10, -0.40, -0.20, 0.0, 1.0,
            -0.55, -0.70, 0.90, 0.35, 0.0, 1.0,
            -0.25, -0.22,
        ],
        0.8,
    ),
]


def connect():
    try:
        motion = ALProxy("ALMotion", ROBOT_IP, ROBOT_PORT)
        tts = ALProxy("ALTextToSpeech", ROBOT_IP, ROBOT_PORT)
        posture = ALProxy("ALRobotPosture", ROBOT_IP, ROBOT_PORT)
        print("Success: Robot connected!")
        return motion, tts, posture
    except Exception as e:
        print("Error connecting to robot:")
        print(e)
        sys.exit(1)


def move_arms(motion, target_angles, duration):
    motion.angleInterpolation(ARM_JOINTS, target_angles, duration, True)


def move_head(motion, target_angles, duration):
    motion.angleInterpolation(HEAD_JOINTS, target_angles, duration, True)


def move_full_pose(motion, target_angles, duration):
    motion.angleInterpolation(FULL_JOINTS, target_angles, duration, True)


def perform_dance(motion, tts):
    for name, line, keyframe, duration in KEYFRAMES:
        print("Moving to:", name)
        tts.say(line)
        move_full_pose(motion, keyframe, duration)
        time.sleep(0.2)


def main():
    motion, tts, posture = connect()
    motion.wakeUp()
    posture.goToPosture("Stand", 0.5)
    motion.setStiffnesses("Body", 1.0)
    move_head(motion, HEAD_CENTER, 0.3)
    move_arms(motion, ARMS_DOWN, 0.6)
    tts.say("Time to dance.")

    perform_dance(motion, tts)

    move_arms(motion, ARMS_DOWN, 0.8)
    move_head(motion, HEAD_CENTER, 0.4)
    posture.goToPosture("Stand", 0.5)


if __name__ == "__main__":
    main()
