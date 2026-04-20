import sys
sys.path.append("/Users/nicholascarducci/Desktop/naoqi-sqk/lib/python2.7/site-packages/naoqi.py")
from naoqi import ALProxy
import vision_definitions # Another common one you'll need

# Replace with your robot's IP (e.g., "192.168.1.10")
ROBOT_IP = "172.16.0.7"

try:
    tts = ALProxy("ALTextToSpeech", ROBOT_IP, 9559)
    tts.say("Hello Nicholas, I am connected!")
    print("Success: Robot spoke!")
except Exception as e:
    print("Error connecting to robot: ")
    print(e)
