#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
import os
import sys
import time
import random

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

try:
    tts     = ALProxy("ALTextToSpeech", ROBOT_IP, 9559)
    leds    = ALProxy("ALLeds",         ROBOT_IP, 9559)
    motion  = ALProxy("ALMotion",       ROBOT_IP, 9559)
    posture = ALProxy("ALRobotPosture", ROBOT_IP, 9559)
    memory  = ALProxy("ALMemory",       ROBOT_IP, 9559)
except Exception as e:
    print("Error connecting to robot: ")
    print(e)
    sys.exit(1)

# ---------------------------------------------------------------------------
# State machine: IDLE -> ALERT -> ACTIVE -> IDLE
# ---------------------------------------------------------------------------

def enter_idle():
    leds.fadeRGB("FaceLeds", 0x0000FF, 0.5)   # blue
    tts.say("Waiting...")

def enter_alert():
    leds.fadeRGB("FaceLeds", 0xFFFF00, 0.5)   # yellow
    tts.say("Something detected!")

def head_touched():
    front  = memory.getData("FrontTactilTouched")
    middle = memory.getData("MiddleTactilTouched")
    rear   = memory.getData("RearTactilTouched")
    return any(v is not None and v >= 1.0 for v in [front, middle, rear])

def bumper_pressed():
    left  = memory.getData("LeftBumperPressed")
    right = memory.getData("RightBumperPressed")
    return (left is not None and left >= 1.0) or \
           (right is not None and right >= 1.0)

QUESTIONS = [
    "Will it rain tomorrow?",
    "Am I going to have a good day?",
    "Is pizza the best food?",
    "Will I ace my next exam?",
    "Is today a lucky day?",
]

def nod():
    """Nod head: pitch down then back up twice."""
    motion.setStiffnesses("HeadPitch", 1.0)
    for _ in range(2):
        motion.setAngles("HeadPitch", 0.4, 0.2)
        time.sleep(0.4)
        motion.setAngles("HeadPitch", -0.1, 0.2)
        time.sleep(0.4)
    motion.setAngles("HeadPitch", 0.0, 0.2)
    time.sleep(0.3)
    motion.setStiffnesses("HeadPitch", 0.0)

def shake():
    """Shake head: yaw left and right twice."""
    motion.setStiffnesses("HeadYaw", 1.0)
    for _ in range(2):
        motion.setAngles("HeadYaw", 0.4, 0.2)
        time.sleep(0.4)
        motion.setAngles("HeadYaw", -0.4, 0.2)
        time.sleep(0.4)
    motion.setAngles("HeadYaw", 0.0, 0.2)
    time.sleep(0.3)
    motion.setStiffnesses("HeadYaw", 0.0)

def crystal_ball():
    leds.fadeRGB("FaceLeds", 0x800080, 0.5)   # purple = mystical
    question = random.choice(QUESTIONS)
    tts.say("Consulting the crystal ball... " + question)
    time.sleep(1.0)

    if random.random() < 0.5:
        leds.fadeRGB("FaceLeds", 0x00FF00, 0.3)
        tts.say("Yes!")
        nod()
    else:
        leds.fadeRGB("FaceLeds", 0xFF0000, 0.3)
        tts.say("No!")
        shake()

def enter_active():
    leds.fadeRGB("FaceLeds", 0x00FF00, 0.5)   # green
    motion.setStiffnesses("Body", 1.0)
    tts.say("Moving!")
    motion.moveToward(1.0, 0, 0)               # walk forward continuously

    while True:
        if bumper_pressed():
            motion.stopMove()
            tts.say("Stopping.")
            posture.goToPosture("Stand", 0.5)
            crystal_ball()
            motion.setStiffnesses("Body", 0.0)
            return
        if head_touched():
            motion.stopMove()
            tts.say("Trying again!")
            time.sleep(0.5)
            motion.moveToward(1.0, 0, 0)
        time.sleep(0.1)

posture.goToPosture("Stand", 0.5)

print("Starting. Press Ctrl+C to stop.")
try:
    enter_active()
except KeyboardInterrupt:
    print("Stopped.")
