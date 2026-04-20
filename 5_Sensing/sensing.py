#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
import os
import sys
import time
import ast

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
    motion  = ALProxy("ALMotion",       ROBOT_IP, 9559)
    posture = ALProxy("ALRobotPosture", ROBOT_IP, 9559)
    tts     = ALProxy("ALTextToSpeech", ROBOT_IP, 9559)
    leds    = ALProxy("ALLeds",         ROBOT_IP, 9559)
    memory  = ALProxy("ALMemory",       ROBOT_IP, 9559)

    posture.goToPosture("Stand", 0.5)
    tts.say("Mirror mode! I will copy my right arm with my left.")

    # Read right arm angles
    right_angles = motion.getAngles(
        ["RShoulderPitch", "RShoulderRoll", "RElbowRoll"], True
    )


    # First, move right arm to a pose
    motion.angleInterpolation(
        ["RShoulderPitch", "RShoulderRoll", "RElbowRoll"],
        [0.2, -0.5, 1.0],
        [1.0, 1.0, 1.0],
        True
    )

    tts.say("Now mirroring to left arm!")

    # Read the right arm's current angles after the pose
    r_pitch, r_roll, r_elbow = motion.getAngles(
        ["RShoulderPitch", "RShoulderRoll", "RElbowRoll"], True
    )

    # Mirror: pitch keeps sign, roll and elbow roll are negated
    motion.angleInterpolation(
        ["LShoulderPitch", "LShoulderRoll", "LElbowRoll"],
        [r_pitch, -r_roll, -r_elbow],
        [1.0, 1.0, 1.0],
        True
    )

    tts.say("Mirrored!")
    time.sleep(1.0)
    posture.goToPosture("Stand", 0.5)

    # -----------------------------------------------------------------------
    # Eye color mixer FSM
    # 3 consecutive states per round:
    #   State 1 (FRONT)  -> wait for front button  -> toggle RED
    #   State 2 (MIDDLE) -> wait for middle button  -> toggle GREEN
    #   State 3 (REAR)   -> wait for rear button    -> toggle BLUE
    #                       -> show combined color, announce it, loop
    # 8 possible combinations (2^3)
    # -----------------------------------------------------------------------

    COLOR_NAMES = {
        (0, 0, 0): "off",
        (1, 0, 0): "red",
        (0, 1, 0): "green",
        (0, 0, 1): "blue",
        (1, 1, 0): "yellow",
        (1, 0, 1): "magenta",
        (0, 1, 1): "cyan",
        (1, 1, 1): "white",
    }

    r, g, b = 0, 0, 0

    STEPS = [
        ("FrontTactilTouched",  "front",  "red"),
        ("MiddleTactilTouched", "middle", "green"),
        ("RearTactilTouched",   "rear",   "blue"),
    ]

    def wait_for_button(memory_key, timeout=15):
        """Block until the given tactile button is pressed, then released.
        Returns True on press, False if timeout expires."""
        deadline = time.time() + timeout
        while True:
            val = memory.getData(memory_key)
            if val is not None and val >= 1.0:
                break
            if time.time() > deadline:
                return False
            time.sleep(0.05)
        while True:
            val = memory.getData(memory_key)
            if val is None or val < 1.0:
                break
            time.sleep(0.05)
        return True

    tts.say("Eye color mixer! Press front for red, middle for green, rear for blue.")

    BUTTONS = [
        ("FrontTactilTouched",  0),  # index into [r, g, b]
        ("MiddleTactilTouched", 1),
        ("RearTactilTouched",   2),
    ]
    prev = [0, 0, 0]  # track previous state to detect rising edge
    components = [r, g, b]
    deadline = time.time() + 15

    while True:
        if time.time() > deadline:
            tts.say("Timed out. Goodbye!")
            break

        pressed_any = False
        for key, idx in BUTTONS:
            val = memory.getData(key)
            cur = 1 if (val is not None and val >= 1.0) else 0
            if cur == 1 and prev[idx] == 0:  # rising edge - new press
                components[idx] = 1 - components[idx]
                pressed_any = True
                deadline = time.time() + 15  # reset timeout on any press
            prev[idx] = cur

        if pressed_any:
            r, g, b = components
            color_hex = (r * 0xFF) << 16 | (g * 0xFF) << 8 | (b * 0xFF)
            color_name = COLOR_NAMES.get((r, g, b), "unknown")
            leds.fadeRGB("FaceLeds", color_hex, 0.3)
            tts.say("Color is " + color_name + "!")
            print("Color: " + color_name + " (#{:06X})".format(color_hex))

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopped.")
except Exception as e:
    print("Error connecting to robot: ")
    print(e)
    sys.exit(1)