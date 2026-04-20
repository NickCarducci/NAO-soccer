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
TRIGGER_WORD = "hello"
MIN_CONFIDENCE = 0.4

RIGHT_ARM_JOINTS = [
    "RShoulderPitch",
    "RShoulderRoll",
    "RElbowYaw",
    "RElbowRoll",
    "RWristYaw",
    "RHand",
]

RIGHT_ARM_DOWN = [1.55, -0.18, 1.20, 0.45, 0.0, 0.0]
RIGHT_ARM_UP = [-1.0, -0.10, 1.20, 0.45, 0.0, 1.0]


def connect():
    try:
        motion = ALProxy("ALMotion", ROBOT_IP, ROBOT_PORT)
        tts = ALProxy("ALTextToSpeech", ROBOT_IP, ROBOT_PORT)
        posture = ALProxy("ALRobotPosture", ROBOT_IP, ROBOT_PORT)
        speech = ALProxy("ALSpeechRecognition", ROBOT_IP, ROBOT_PORT)
        memory = ALProxy("ALMemory", ROBOT_IP, ROBOT_PORT)
        print("Success: Robot connected!")
        return motion, tts, posture, speech, memory
    except Exception as e:
        print("Error connecting to robot:")
        print(e)
        sys.exit(1)


def raise_right_arm(motion):
    motion.angleInterpolation(RIGHT_ARM_JOINTS, RIGHT_ARM_UP, 1.2, True)


def wave_right_hand(motion):
    times = [
        [0.4, 0.8, 1.2, 1.6, 2.0],
        [0.4, 0.8, 1.2, 1.6, 2.0],
        [0.4, 0.8, 1.2, 1.6, 2.0],
        [0.4, 0.8, 1.2, 1.6, 2.0],
        [0.4, 0.8, 1.2, 1.6, 2.0],
        [0.4, 0.8, 1.2, 1.6, 2.0],
    ]
    keys = [
        [-1.0, -1.0, -1.0, -1.0, -1.0],
        [-0.5, 0.3, -0.5, 0.3, -0.1],
        [1.20, 1.20, 1.20, 1.20, 1.20],
        [0.45, 0.45, 0.45, 0.45, 0.45],
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0, 1.0, 1.0],
    ]
    motion.angleInterpolation(RIGHT_ARM_JOINTS, keys, times, True)


def lower_right_arm(motion):
    motion.angleInterpolation(RIGHT_ARM_JOINTS, RIGHT_ARM_DOWN, 1.2, True)


def clear_word(memory):
    try:
        memory.insertData("WordRecognized", [])
    except Exception:
        pass


def wait_for_hello(tts, speech, memory):
    speech.pause(True)
    speech.setLanguage("English")
    speech.setVocabulary([TRIGGER_WORD], False)
    speech.pause(False)
    speech.subscribe("wave_hello_listener")
    clear_word(memory)
    tts.say("Say hello and I will wave.")

    try:
        while True:
            data = memory.getData("WordRecognized")
            print("WordRecognized:", data)
            if isinstance(data, list) and len(data) >= 2 and data[1] >= MIN_CONFIDENCE:
                raw_word = data[0]
                clear_word(memory)
                if isinstance(raw_word, basestring) and TRIGGER_WORD in raw_word.lower():
                    return
            time.sleep(0.1)
    finally:
        try:
            speech.unsubscribe("wave_hello_listener")
        except Exception:
            pass


def main():
    motion, tts, posture, speech, memory = connect()
    motion.wakeUp()
    posture.goToPosture("Stand", 0.5)
    motion.setStiffnesses("Body", 1.0)
    wait_for_hello(tts, speech, memory)
    tts.say("Hello!")

    raise_right_arm(motion)
    wave_right_hand(motion)
    lower_right_arm(motion)

    time.sleep(0.3)
    posture.goToPosture("Stand", 0.5)


if __name__ == "__main__":
    main()
