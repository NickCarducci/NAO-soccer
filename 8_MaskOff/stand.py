#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
import os
import sys

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

motion = None
posture = None
tts = None

try:
    motion = ALProxy("ALMotion", ROBOT_IP, ROBOT_PORT)
    posture = ALProxy("ALRobotPosture", ROBOT_IP, ROBOT_PORT)
    tts = ALProxy("ALTextToSpeech", ROBOT_IP, ROBOT_PORT)

    print("Success: Robot connected!")
    motion.wakeUp()
    motion.setStiffnesses("Body", 1.0)
    posture.goToPosture("StandInit", 0.6)
    tts.say("Standing up.")

except KeyboardInterrupt:
    print("Stopped.")
except Exception as e:
    print("Error connecting to robot: ")
    print(e)
    sys.exit(1)
