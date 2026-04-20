#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
import sys
import os
import time
import math

# 1. The folder path (MUST end at site-packages)
sdk_folder = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib/python2.7/site-packages"
sys.path.append(sdk_folder)

# 2. Tell the system where the 'binaries' are (the .so files)
os.environ["DYLD_LIBRARY_PATH"] = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib"

try:
    from naoqi import ALProxy
    import vision_definitions
    print("SDK successfully loaded!")
except ImportError as e:
    print("Still cannot find naoqi. Check if this file exists:")
    print(sdk_folder + "/naoqi.py")
    print("Error details: " + str(e))
    sys.exit(1)

# Robot Connection
ROBOT_IP = "172.16.0.5"

tts = ALProxy("ALTextToSpeech", ROBOT_IP, 9559)
asr = ALProxy("ALSpeechRecognition", ROBOT_IP, 9559)
memory = ALProxy("ALMemory", ROBOT_IP, 9559)

DEBUG = True
MIN_CONFIDENCE = 0.365

asr.pause(True)
asr.setLanguage("English")
vocabulary = ["forward", "right", "left", "stand", "sit", "stop", "little"]
asr.setVocabulary(vocabulary, False)
asr.pause(False)

# Simulate hearing a word
# word = "good morning"

# TODO: Respond differently based on time of day greeting
# Use if/elif/else to give appropriate responses

motion = ALProxy("ALMotion", ROBOT_IP, 9559)
posture = ALProxy("ALRobotPosture", ROBOT_IP, 9559)

motion.setStiffnesses("Body", 1.0)
posture.goToPosture("Stand", 0.5)

tts.say("Ready when you are.")

def clear_word():
    try:
        memory.insertData("WordRecognized", [])
    except Exception:
        pass


def listen_for_command():
    clear_word()
    while True:
        data = memory.getData("WordRecognized")
        if DEBUG:
            print("WordRecognized:", data)
        if isinstance(data, list) and len(data) >= 2:
            if data[1] > MIN_CONFIDENCE:
                raw = data[0]
                clear_word()
                if isinstance(raw, basestring):
                    text = raw.lower()
                    if "little" in text and "right" in text:
                        return "right_little"
                    if "little" in text and "left" in text:
                        return "left_little"
                    for cmd in vocabulary:
                        if cmd in text:
                            return cmd
        time.sleep(0.1)


try:
    # Listen module
    asr.subscribe("asr_subscriber_main")
    clear_word()
    time.sleep(0.5)

    moving = False
    while True:
        try:
            moving = bool(motion.moveIsActive())
        except Exception:
            pass

        command = listen_for_command()

        if command == "forward":
            tts.say("Movin' on out now.")
            motion.post.moveTo(1.0, 0, 0)     # Walk 1 meter forward (async)
            moving = True
        elif command == "right":
            tts.say("No problem.")
            motion.post.moveTo(0, 0, -math.pi / 2)   # Turn 90 deg right (async)
            moving = True
        elif command == "right_little":
            tts.say("Okay...")
            motion.post.moveTo(0, 0, -math.pi / 4)   # Turn 45 deg right (async)
            moving = True
        elif command == "left":
            tts.say("Yeah, alright.")
            motion.post.moveTo(0, 0, math.pi / 2)   # Turn 90 deg left (async)
            moving = True
        elif command == "left_little":
            tts.say("Sure...")
            motion.post.moveTo(0, 0, math.pi / 4)   # Turn 45 deg left (async)
            moving = True
        elif command == "stand":
            tts.say("Right away.")
            posture.goToPosture("Stand", 0.5)
        elif command == "sit":
            tts.say("Finally.")
            posture.goToPosture("Sit", 0.5)
        elif command == "stop":
            motion.stopMove()
            if moving:
                moving = False
                tts.say("Woah what's wrong?")
            else:
                tts.say("Disconnecting...")
                break
        else:
            tts.say("Don't tell me what to do.")
            if DEBUG:
                print("Unrecognized command:", command)
finally:
    motion.stopMove()
    motion.setStiffnesses("Body", 0.0)
    try:
        asr.unsubscribe("asr_subscriber_main")
    except Exception:
        pass
