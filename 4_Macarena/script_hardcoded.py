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


NEUTRAL = {
    "LShoulderPitch": 1.55,
    "LShoulderRoll": 0.18,
    "LElbowYaw": -1.20,
    "LElbowRoll": -0.45,
    "LWristYaw": 0.0,
    "LHand": 0.0,
    "RShoulderPitch": 1.55,
    "RShoulderRoll": -0.18,
    "RElbowYaw": 1.20,
    "RElbowRoll": 0.45,
    "RWristYaw": 0.0,
    "RHand": 0.0,
}

STEP_UPDATES = [
    ("right_arm_out_only", 1.0, "Right arm out", {
        "RShoulderPitch": 0.0,
        "RShoulderRoll": -0.65,
        "RElbowYaw": 0.0,
        "RElbowRoll": 0.0,
        "RHand": 1.0,
    }),
    ("right_and_left_arms_out", 1.0, "Left arm out", {
        "LShoulderPitch": 0.0,
        "LShoulderRoll": 0.65,
        "LElbowYaw": 0.0,
        "LElbowRoll": 0.0,
        "LHand": 1.0,
    }),
    ("right_palm_up_only", 0.9, "Right palm up", {
        "RWristYaw": 1.8,
    }),
    ("right_and_left_palms_up", 0.9, "Left palm up", {
        "LWristYaw": -1.8,
    }),
    ("right_hand_to_left_shoulder_only", 1.0, "Right hand to left shoulder", {
        "RShoulderPitch": 0.0,
        "RShoulderRoll": 0.75,
        "RElbowYaw": 0.75,
        "RElbowRoll": 0.20,
    }),
    ("right_and_left_hands_crossed", 1.0, "Left hand to right shoulder", {
        "LShoulderPitch": 0.0,
        "LShoulderRoll": -0.75,
        "LElbowYaw": -0.75,
        "LElbowRoll": -0.20,
        "RWristYaw": 0.0,
        "LWristYaw": 0.0,
    }),
    ("hands_behind_head", 1.1, "Hands behind head", {
        "LShoulderPitch": -0.85,
        "LShoulderRoll": 0.22,
        "LElbowYaw": -0.55,
        "LElbowRoll": -0.75,
        "LWristYaw": 0.0,
        "RShoulderPitch": -0.85,
        "RShoulderRoll": -0.22,
        "RElbowYaw": 0.55,
        "RElbowRoll": 0.75,
        "RWristYaw": 0.0,
    }),
    ("hands_on_hips", 1.0, "Hands on hips", {
        "LShoulderPitch": 1.1,
        "LShoulderRoll": 0.35,
        "LElbowYaw": -1.0,
        "LElbowRoll": -0.9,
        "LHand": 1.0,
        "RShoulderPitch": 1.1,
        "RShoulderRoll": -0.35,
        "RElbowYaw": 1.0,
        "RElbowRoll": 0.9,
        "RHand": 1.0,
    }),
]


def apply_joint_updates(motion, current_pose, joint_updates, duration):
    for joint_name, angle in joint_updates.items():
        current_pose[joint_name] = angle
    angles = [current_pose[joint] for joint in ARM_JOINTS]
    motion.angleInterpolation(ARM_JOINTS, angles, duration, True)


def go_to_full_pose(motion, pose_map, duration):
    angles = [pose_map[joint] for joint in ARM_JOINTS]
    motion.angleInterpolation(ARM_JOINTS, angles, duration, True)


def do_macarena(motion, tts, posture):
    current_pose = dict(NEUTRAL)
    motion.wakeUp()
    posture.goToPosture("StandInit", 0.6)
    motion.setStiffnesses("Body", 1.0)
    motion.setAngles(["HeadPitch", "HeadYaw"], [-0.12, 0.0], 0.15)
    go_to_full_pose(motion, current_pose, 1.2)
    tts.say("Macarena")

    for pose_name, duration, spoken_text, joint_updates in STEP_UPDATES:
        print("Moving to:", pose_name)
        tts.say(spoken_text)
        apply_joint_updates(motion, current_pose, joint_updates, duration)
        time.sleep(0.15)

    time.sleep(0.5)
    go_to_full_pose(motion, NEUTRAL, 1.2)
    posture.goToPosture("StandInit", 0.5)


def main():
    motion, tts, posture = connect()
    do_macarena(motion, tts, posture)


if __name__ == "__main__":
    main()
