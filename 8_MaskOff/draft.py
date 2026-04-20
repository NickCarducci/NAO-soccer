#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
# -*- coding: utf-8 -*-
import json
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

try:
    import requests
except ImportError:
    requests = None

try:
    from dotenv import load_dotenv as dotenv_load
except ImportError:
    dotenv_load = None

try:
    text_type = unicode
except NameError:
    text_type = str


ROBOT_IP = "172.16.0.5"
ROBOT_PORT = 9559
FACE_SUBSCRIBER_NAME = "face_off_face_detector"
SOUND_SUBSCRIBER_NAME = "face_off_sound_localizer"
POLL_INTERVAL = 0.2
GREETING_COOLDOWN_SECONDS = 10.0
OPENAI_URL = "https://api.openai.com/v1/chat/completions"
LEARNED_FACE_NAMES = ["first human", "second human"]

tts = None
memory = None
camera = None
motion = None
face_proxy = None
sound_proxy = None


def simple_load_dotenv(env_path):
    if not os.path.exists(env_path):
        return

    env_file = open(env_path, "r")
    try:
        for raw_line in env_file:
            line = raw_line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            key = key.strip()
            value = value.strip().strip('"').strip("'")
            if key and key not in os.environ:
                os.environ[key] = value
    finally:
        env_file.close()


def load_env_file():
    env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")
    if dotenv_load is not None:
        dotenv_load(env_path)
    else:
        simple_load_dotenv(env_path)


def say(text):
    if text is None:
        return
    if isinstance(text, text_type):
        spoken = text.encode("ascii", "ignore")
    else:
        spoken = str(text)
    tts.say(spoken)


def bumper_pressed():
    try:
        right = memory.getData("RightBumperPressed")
        left = memory.getData("LeftBumperPressed")
        return right == 1.0 or left == 1.0
    except RuntimeError:
        return False


def face_detected(face_data):
    return bool(face_data) and len(face_data) > 1 and len(face_data[1]) > 0


def get_recognized_faces(face_data):
    try:
        if not face_data or len(face_data) < 2 or len(face_data[1]) == 0:
            return []

        time_filtered = face_data[1][len(face_data[1]) - 1]
        if len(time_filtered) == 2 and time_filtered[0] in [2, 3]:
            return time_filtered[1]
    except Exception:
        pass
    return []


def turn_toward_sound():
    try:
        sound_data = memory.getData("SoundLocated")
        if sound_data and len(sound_data) >= 2:
            azimuth = sound_data[1][0]
            elevation = sound_data[1][1]
            motion.setAngles(["HeadYaw", "HeadPitch"], [azimuth, -elevation], 0.3)
    except RuntimeError:
        pass
    except Exception as e:
        print("Sound tracking error: %s" % e)


def clear_face_memory():
    try:
        memory.insertData("FaceDetected", [])
    except Exception:
        pass


def learn_face(name):
    while True:
        if bumper_pressed():
            say("Goodbye!")
            return False

        say("Please show me your face, " + name + ".")
        time.sleep(1.5)

        success = face_proxy.learnFace(name)
        if success:
            say("Got it. I have learned your face, " + name + ".")
            return True

        say("I could not see you clearly. Please try again.")


def ensure_known_faces():
    known_faces = []
    try:
        known_faces = face_proxy.getLearnedFacesList()
    except Exception:
        pass

    missing_faces = [name for name in LEARNED_FACE_NAMES if name not in known_faces]
    if not missing_faces:
        print("Known faces already stored: %s" % ", ".join(LEARNED_FACE_NAMES))
        return True

    say("I need to learn two faces before I can recognize them by name.")
    for name in missing_faces:
        if not learn_face(name):
            return False

    say("I have learned both faces. Show me a face I know.")
    return True


def get_openai_api_key():
    load_env_file()
    return os.getenv("OPENAI_API_KEY")


def get_chatgpt_greeting(name, api_key):
    fallback = ("Hello, " + name + "!").encode("ascii", "ignore")

    if not api_key or requests is None:
        return fallback

    body = {
        "model": "gpt-3.5-turbo",
        "messages": [
            {
                "role": "system",
                "content": (
                    "You are a friendly robot named NAO. "
                    "Generate short warm greetings. "
                    "One or two sentences. No emoji."
                ),
            },
            {
                "role": "user",
                "content": "Generate a greeting for " + name,
            },
        ],
        "max_tokens": 60,
    }

    try:
        response = requests.post(
            OPENAI_URL,
            headers={
                "Content-Type": "application/json",
                "Authorization": "Bearer " + api_key,
            },
            data=json.dumps(body),
            timeout=10,
        )
        response.raise_for_status()
        result = response.json()
        greeting = result["choices"][0]["message"]["content"].strip()
        return greeting.encode("ascii", "ignore")
    except Exception as e:
        print("API error: %s" % e)
        return fallback


def connect_proxies():
    global tts, memory, camera, motion, face_proxy, sound_proxy

    tts = ALProxy("ALTextToSpeech", ROBOT_IP, ROBOT_PORT)
    memory = ALProxy("ALMemory", ROBOT_IP, ROBOT_PORT)
    camera = ALProxy("ALVideoDevice", ROBOT_IP, ROBOT_PORT)
    motion = ALProxy("ALMotion", ROBOT_IP, ROBOT_PORT)
    face_proxy = ALProxy("ALFaceDetection", ROBOT_IP, ROBOT_PORT)
    sound_proxy = ALProxy("ALSoundLocalization", ROBOT_IP, ROBOT_PORT)


def start_services():
    camera.setActiveCamera(0)
    face_proxy.subscribe(FACE_SUBSCRIBER_NAME, 500, 0.0)
    sound_proxy.subscribe(SOUND_SUBSCRIBER_NAME)
    clear_face_memory()


def stop_services():
    if face_proxy is not None:
        try:
            face_proxy.unsubscribe(FACE_SUBSCRIBER_NAME)
        except Exception:
            pass
    if sound_proxy is not None:
        try:
            sound_proxy.unsubscribe(SOUND_SUBSCRIBER_NAME)
        except Exception:
            pass


def run_face_off():
    api_key = get_openai_api_key()
    if not api_key:
        print("OPENAI_API_KEY not found in .env. Using fallback greetings.")
    if requests is None:
        print("requests is not installed. Using fallback greetings.")

    connect_proxies()
    start_services()

    if not ensure_known_faces():
        return

    say("I'm ready to greet faces and face any sounds. If you press my foot bumper, then I won't anymore.")

    greeted_any_face = False
    last_greeted_name = ""
    last_greeted_time = 0

    while True:
        if bumper_pressed():
            say("Goodbye!")
            break

        turn_toward_sound()

        try:
            face_data = memory.getData("FaceDetected")
        except RuntimeError:
            face_data = []

        if face_detected(face_data):
            if not greeted_any_face:
                say("Hello, human.")
                greeted_any_face = True
        else:
            greeted_any_face = False
            last_greeted_name = ""

        recognized_names = get_recognized_faces(face_data)
        for name in recognized_names:
            time_since_last = time.time() - last_greeted_time
            if name != last_greeted_name or time_since_last > GREETING_COOLDOWN_SECONDS:
                greeting = get_chatgpt_greeting(name, api_key)
                say(greeting)
                last_greeted_name = name
                last_greeted_time = time.time()

        time.sleep(POLL_INTERVAL)


try:
    run_face_off()
except KeyboardInterrupt:
    print("Stopped.")
except Exception as e:
    print("Error connecting to robot: ")
    print(e)
    sys.exit(1)
finally:
    stop_services()
