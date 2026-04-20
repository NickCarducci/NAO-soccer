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


ROBOT_IP = "172.16.0.5"
ROBOT_PORT = 9559
SUBSCRIBER_NAME = "mask_off_face_detector"
POLL_INTERVAL = 0.2

tts = None
memory = None
face_proxy = None


def face_detected(data):
    return isinstance(data, list) and len(data) > 1


def clear_face_memory():
    try:
        memory.insertData("FaceDetected", [])
    except Exception:
        pass


try:
    tts = ALProxy("ALTextToSpeech", ROBOT_IP, ROBOT_PORT)
    memory = ALProxy("ALMemory", ROBOT_IP, ROBOT_PORT)
    face_proxy = ALProxy("ALFaceDetection", ROBOT_IP, ROBOT_PORT)

    face_proxy.subscribe(SUBSCRIBER_NAME, 500, 0.0)
    clear_face_memory()
    tts.say("Looking for a face.")

    while True:
        face_data = memory.getData("FaceDetected")
        print("FaceDetected:", face_data)
        if face_detected(face_data):
            tts.say("Hello, human.")
            break
        time.sleep(POLL_INTERVAL)

except KeyboardInterrupt:
    print("Stopped.")
except Exception as e:
    print("Error connecting to robot: ")
    print(e)
    sys.exit(1)
finally:
    if face_proxy is not None:
        try:
            face_proxy.unsubscribe(SUBSCRIBER_NAME)
        except Exception:
            pass
