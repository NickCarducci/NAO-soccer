#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
import os
import sys
import time


if __name__ == "__main__":
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
SUBSCRIBER_NAME = "recognize_object_announcer"
POLL_INTERVAL = 0.2

# In a Python dictionary, the key acts like the "case" part of a switch/case.
# Keep the prefixes aligned with however NAO or Choregraphe labels the trained objects.
# The spoken message can still be more natural than the raw canonical prefix.
OBJECT_PREFIX_MESSAGES = {
    "cube-": "I see a face of the cube.",
    "square-": "I see a square face.",
    "page-shape-": "I see a page with a shape.",
    "page-": "I see a page.",
    "book-": "I see a book.",
    "card-": "I see a card.",
    "paper-": "I see a sheet of paper.",
    "house-": "I see a side of the house.",
    "car-": "I see a side of the car.",
    "bottle-": "I see a bottle.",
    "can-": "I see a can.",
    "cup-": "I see a cup.",
}

# List of possible sides for suffix parsing
SIDES = ["front", "back", "left", "right", "top", "bottom"]

# After announcing the object, NAO walks forward a short distance toward it.
APPROACH_DISTANCE_METERS = 0.25

tts = None
memory = None
motion = None
posture = None
vision = None


def bumper_pressed():
    try:
        right = memory.getData("RightBumperPressed")
        left = memory.getData("LeftBumperPressed")
        return right == 1.0 or left == 1.0
    except RuntimeError:
        return False


def clear_picture_memory():
    try:
        memory.insertData("PictureDetected", [])
    except Exception:
        pass


def get_detected_object_names(data):
    # PictureDetected is a nested list. Parse defensively so the code tolerates
    # slight differences in the returned structure.
    detected_names = []
    try:
        if not data or len(data) < 2:
            return detected_names

        for item in data[1]:
            if not isinstance(item, list):
                continue
            for part in item:
                if isinstance(part, list):
                    for value in part:
                        if isinstance(value, basestring): # type: ignore
                            name = value.strip()
                            if name and name not in detected_names:
                                detected_names.append(name)
    except Exception:
        pass
    return detected_names



def parse_object_label(object_name):
    """
    Returns (prefix, side, suffix, message)
    """
    # Sort prefixes by length descending to prefer more specific matches
    for prefix in sorted(OBJECT_PREFIX_MESSAGES.keys(), key=len, reverse=True):
        message = OBJECT_PREFIX_MESSAGES[prefix]
        if object_name.startswith(prefix):
            suffix = object_name[len(prefix):]
            for side in SIDES:
                if suffix.startswith(side):
                    return prefix, side, suffix, message
            return prefix, None, suffix, message
    return None, None, object_name, "I found an object."



def announce_object(object_name):
    prefix, side, suffix, message = parse_object_label(object_name)
    # Stepwise instructional announcements
    tts.say(message)
    if side:
        tts.say("The side is " + side + ".")
    tts.say("The object label is " + object_name + ".")


def walk_toward_object():
    motion.moveTo(APPROACH_DISTANCE_METERS, 0.0, 0.0)


if __name__ == "__main__":
    try:
        motion = ALProxy("ALMotion", ROBOT_IP, ROBOT_PORT)
        posture = ALProxy("ALRobotPosture", ROBOT_IP, ROBOT_PORT)
        tts = ALProxy("ALTextToSpeech", ROBOT_IP, ROBOT_PORT)
        memory = ALProxy("ALMemory", ROBOT_IP, ROBOT_PORT)
        vision = ALProxy("ALVisionRecognition", ROBOT_IP, ROBOT_PORT)

        motion.wakeUp()
        motion.setStiffnesses("Body", 1.0)
        posture.goToPosture("StandInit", 0.6)

        vision.subscribe(SUBSCRIBER_NAME, 500, 0.0)
        clear_picture_memory()

        tts.say("Looking for a recognized object. Press either foot bumper to stop.")

        last_announced_object = ""

        while True:
            if bumper_pressed():
                tts.say("Goodbye!")
                break

            try:
                picture_data = memory.getData("PictureDetected")
            except RuntimeError:
                picture_data = []

            object_names = get_detected_object_names(picture_data)
            if object_names:
                print("Detected objects:", object_names)

            for object_name in object_names:
                if object_name != last_announced_object:
                    announce_object(object_name)
                    walk_toward_object()
                    last_announced_object = object_name
                    clear_picture_memory()
                    break

            if not object_names:
                last_announced_object = ""

            time.sleep(POLL_INTERVAL)

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
        if vision is not None:
            try:
                vision.unsubscribe(SUBSCRIBER_NAME)
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
