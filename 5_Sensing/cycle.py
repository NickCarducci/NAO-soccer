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
    leds   = ALProxy("ALLeds",         ROBOT_IP, 9559)
    tts    = ALProxy("ALTextToSpeech", ROBOT_IP, 9559)
    memory = ALProxy("ALMemory",       ROBOT_IP, 9559)
    
    tts.say("Traffic light starting!")

    # Red light
    leds.fadeRGB("FaceLeds", 0xFF0000, 0.5)
    tts.say("Stop!")
    time.sleep(3)

    # Yellow light
    leds.fadeRGB("FaceLeds", 0xFFFF00, 0.5)
    tts.say("Get ready...")
    time.sleep(3)

    # Green light
    leds.fadeRGB("FaceLeds", 0x00FF00, 0.5)
    tts.say("Go!")

    print("Traffic light done. Starting mood FSM...")

    # Mood FSM: HAPPY -> SAD -> ANGRY -> EXCITED -> HAPPY
    MOODS = [
        ("HAPPY",   0x00FF00, "I am happy!"),
        ("SAD",     0x0000FF, "I am sad."),
        ("ANGRY",   0xFF0000, "I am angry!"),
        ("EXCITED", 0xFFFF00, "I am excited!"),
    ]

    def head_touched():
        return any(
            memory.getData(k) >= 1.0
            for k in ["FrontTactilTouched", "MiddleTactilTouched", "RearTactilTouched"]
        )

    mood_index = 0  # start at HAPPY
    deadline = time.time() + 15

    while True:
        if head_touched():
            tts.say("Stopping. Goodbye!")
            break
        if time.time() > deadline:
            tts.say("Timed out. Goodbye!")
            break

        name, color, phrase = MOODS[mood_index]
        print("Mood state: " + name)
        leds.fadeRGB("FaceLeds", color, 0.5)
        tts.say(phrase)

        # Wait 3s but check for head touch or timeout each 0.1s
        wait_until = time.time() + 3
        while time.time() < wait_until:
            if head_touched() or time.time() > deadline:
                break
            time.sleep(0.1)

        # Transition to next mood, wrapping back to HAPPY
        mood_index = (mood_index + 1) % len(MOODS)

except KeyboardInterrupt:
    print("Stopped.")
except Exception as e:
    print("Error connecting to robot: ")
    print(e)
    sys.exit(1)