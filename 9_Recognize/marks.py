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


ROBOT_IP = "172.16.0.29"
ROBOT_PORT = 9559
SUBSCRIBER_NAME = "recognize_object_detector"
POLL_INTERVAL = 0.2


# NAOMark ID to action mapping (switch/case style)
MARK_ACTIONS = {
    80: "walk_forward",
    119: "turn_left",  # You can change to "turn_right" if you want the other direction
}

# Motion parameters for each action
MOTION_PARAMS = {
    "walk_forward":  (0.3, 0.0, 0.0, 0.8),   # x, y, theta, freq
    "walk_backward": (-0.3, 0.0, 0.0, 0.8),
    "turn_left":     (0.0, 0.0, 0.4, 0.8),
    "turn_right":    (0.0, 0.0, -0.4, 0.8),
    "stop":          (0.0, 0.0, 0.0, 0.0),
}

def perform_action(action, angle=None):
    if action == "stop":
        motion.stopMove()
        tts.say("Stop mark seen. Motion stopped.")
        return "stopped"
    if action == "turn_left" and angle is not None:
        # Turn toward the detected NAOMark's angle (alpha)
        # Clamp angle to reasonable range for safety
        max_theta = 0.8
        min_theta = -0.8
        theta = max(min(angle, max_theta), min_theta)
        motion.setWalkTargetVelocity(0.0, 0.0, theta, 0.8)
        tts.say("Turning toward NAOMark angle {:.2f}".format(theta))
        return action
    params = MOTION_PARAMS.get(action)
    if params:
        x, y, theta, freq = params
        motion.setWalkTargetVelocity(x, y, theta, freq)
        tts.say("Action: {}".format(action.replace("_", " ")))
        return action
    else:
        tts.say("Unknown action.")
        return None

tts = None
memory = None
landmark = None
motion = None
posture = None
current_action = None


def clear_landmark_memory():
    try:
        memory.insertData("LandmarkDetected", [])
    except Exception:
        pass


def bumper_pressed():
    try:
        right = memory.getData("RightBumperPressed")
        left = memory.getData("LeftBumperPressed")
        return right == 1.0 or left == 1.0
    except RuntimeError:
        return False


# Returns a list of (mark_id, alpha) tuples. Alpha is the horizontal position (angle in radians).
def get_detected_marks_with_angles(data):
    marks = []
    try:
        if not data or len(data) < 2:
            return marks
        for mark_info in data[1]:
            if not isinstance(mark_info, list) or len(mark_info) < 2:
                continue
            extra_info = mark_info[1]
            if isinstance(extra_info, list) and len(extra_info) > 0:
                mark_id = extra_info[0]
                # The horizontal angle (alpha) is usually at index 1 in mark_info[0]
                alpha = mark_info[0][1] if len(mark_info[0]) > 1 else 0.0
                marks.append((mark_id, alpha))
    except Exception:
        pass
    return marks


def stop_turning():
    global current_action
    if current_action != "stop":
        perform_action("stop")
        current_action = "stop"


try:
    motion = ALProxy("ALMotion", ROBOT_IP, ROBOT_PORT)
    posture = ALProxy("ALRobotPosture", ROBOT_IP, ROBOT_PORT)
    tts = ALProxy("ALTextToSpeech", ROBOT_IP, ROBOT_PORT)
    memory = ALProxy("ALMemory", ROBOT_IP, ROBOT_PORT)
    landmark = ALProxy("ALLandMarkDetection", ROBOT_IP, ROBOT_PORT)

    motion.wakeUp()
    motion.setStiffnesses("Body", 1.0)
    posture.goToPosture("StandInit", 0.6)

    landmark.subscribe(SUBSCRIBER_NAME, 500, 0.0)
    clear_landmark_memory()


    tts.say(
        "Show me NAOMark 80 to walk forward, or 119 to turn. "
        "Press either foot bumper to quit."
    )


    while True:
        if bumper_pressed():
            tts.say("Goodbye!")
            break

        try:
            landmark_data = memory.getData("LandmarkDetected")
        except RuntimeError:
            landmark_data = []

        detected_marks = get_detected_marks_with_angles(landmark_data)
        if detected_marks:
            print("Detected NAOMarks:", detected_marks)

        # For each detected mark, perform the mapped action
        action_taken = False
        for mark_id, angle in detected_marks:
            action = MARK_ACTIONS.get(mark_id)
            if action:
                if action != current_action:
                    if action == "turn_left":
                        perform_action(action, angle)
                    else:
                        perform_action(action)
                    current_action = action
                    action_taken = True
                break  # Only act on the first recognized mark

        # If no mapped mark is visible and the robot is moving, stop
        if not action_taken and current_action and current_action != "stop":
            perform_action("stop")
            current_action = "stop"

        clear_landmark_memory()
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
    if landmark is not None:
        try:
            landmark.unsubscribe(SUBSCRIBER_NAME)
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
