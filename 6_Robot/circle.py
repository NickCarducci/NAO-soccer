#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
import os
import sys
import time
import threading
import math

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

dancing = True


def wrapped_angle_delta(current_angle, previous_angle):
    delta = current_angle - previous_angle
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


def head_layer():
    while dancing:
        motion.angleInterpolation(HEAD_JOINTS, HEAD_LEFT, 0.35, True)
        if not dancing:
            break
        motion.angleInterpolation(HEAD_JOINTS, HEAD_RIGHT, 0.35, True)
        if not dancing:
            break
        motion.angleInterpolation(HEAD_JOINTS, HEAD_DOWN, 0.30, True)
        if not dancing:
            break
        motion.angleInterpolation(HEAD_JOINTS, HEAD_CENTER, 0.30, True)


def led_layer():
    colors = [0x00FF0000, 0x0000FF00, 0x000000FF, 0x00FFFFFF]
    index = 0
    while dancing:
        leds.fadeRGB("AllLeds", colors[index], 0.2)
        index = (index + 1) % len(colors)
        time.sleep(0.15)
    leds.fadeRGB("AllLeds", 0x00000000, 0.2)


def walk_circle():
    global dancing
    completed_rotation = 0.0
    previous_theta = motion.getRobotPosition(False)[2]

    # Forward + rotation together creates a true curved walk with visible diameter.
    motion.setWalkTargetVelocity(0.65, 0.0, 0.18, 0.8)
    try:
        while completed_rotation < (2.0 * math.pi):
            time.sleep(0.1)
            current_theta = motion.getRobotPosition(False)[2]
            completed_rotation += abs(wrapped_angle_delta(current_theta, previous_theta))
            previous_theta = current_theta
            print("Rotation progress: %.2f / %.2f" % (completed_rotation, 2.0 * math.pi))
    finally:
        dancing = False
        motion.stopMove()

posture = None
motion = None
tts = None
leds = None
head_thread = None
led_thread = None

try:
    posture = ALProxy("ALRobotPosture", ROBOT_IP, ROBOT_PORT)
    motion = ALProxy("ALMotion", ROBOT_IP, ROBOT_PORT)
    tts = ALProxy("ALTextToSpeech", ROBOT_IP, ROBOT_PORT)
    leds = ALProxy("ALLeds", ROBOT_IP, 9559)

    posture.goToPosture("Stand", 0.5)
    motion.setStiffnesses("Body", 1.0)

    tts.say("Circle dance starting!")
    head_thread = threading.Thread(target=head_layer)
    led_thread = threading.Thread(target=led_layer)

    head_thread.start()
    led_thread.start()

    walk_circle()

    head_thread.join()
    led_thread.join()
    tts.say("That's the circle-dance.")

except KeyboardInterrupt:
    print("Stopped.")
except Exception as e:
    print("Error connecting to robot: ")
    print(e)
    sys.exit(1)
finally:
    if motion is not None:
        try:
            motion.stopMove()
        except Exception:
            pass
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
