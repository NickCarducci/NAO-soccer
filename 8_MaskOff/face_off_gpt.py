#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
# -*- coding: utf-8 -*-
# Face learning and greeting with ChatGPT-generated greetings.
import os
import sys
import time
import json
import requests
from dotenv import load_dotenv

sdk_folder = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib/python2.7/site-packages"
sys.path.append(sdk_folder)
os.environ["DYLD_LIBRARY_PATH"] = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib"

try:
    from naoqi import ALProxy
    print("SDK successfully loaded!")
except ImportError as e:
    print("Still cannot find naoqi. Check if this file exists:")
    print(sdk_folder + "/naoqi.py")
    print("Error details: " + str(e))
    sys.exit(1)

load_dotenv(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env"))
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

if not OPENAI_API_KEY:
    print("ERROR: OPENAI_API_KEY not found.")
    print("Make sure your .env file exists and contains OPENAI_API_KEY=your_key_here")
    sys.exit(1)

ROBOT_IP = "172.16.0.6"
ROBOT_PORT = 9559
OPENAI_URL = "https://api.openai.com/v1/chat/completions"
POLL_INTERVAL = 0.2
GREETING_COOLDOWN_SECONDS = 10.0
LEARNED_FACE_NAMES = ["Bob", "Larry"]

# Set up proxies
tts = ALProxy("ALTextToSpeech",  ROBOT_IP, ROBOT_PORT)
memory = ALProxy("ALMemory",      ROBOT_IP, ROBOT_PORT)
camera = ALProxy("ALVideoDevice", ROBOT_IP, ROBOT_PORT)
face_proxy = ALProxy("ALFaceDetection", ROBOT_IP, ROBOT_PORT)
sound_proxy = ALProxy("ALSoundLocalization", ROBOT_IP, ROBOT_PORT)
motion = ALProxy("ALMotion", ROBOT_IP, ROBOT_PORT)

camera.setActiveCamera(0)
face_proxy.subscribe("face_off_hardcoded")
tts.say("Ready. Show me a face I know and I will greet them. Press my foot bumper to stop.")
print("Running. Press a foot bumper or Ctrl+C to stop.")

def say(text):
    tts.say(str(text))

def get_chatgpt_greeting(name):
    try:
        print("Asking ChatGPT for a greeting for " + name + "...")
        body = {
            "model": "gpt-3.5-turbo",
            "messages": [
                {
                    "role": "system",
                    "content": (
                        "You are a friendly robot named NAO. "
                        "Generate short, warm, creative greetings for people. "
                        "Keep responses to one or two sentences. "
                        "Do not use any special characters or emoji."
                    )
                },
                {
                    "role": "user",
                    "content": "Generate a greeting for a person named " + name + "."
                }
            ],
            "max_tokens": 60
        }
        response = requests.post(
            OPENAI_URL,
            headers={
                "Content-Type": "application/json",
                "Authorization": "Bearer " + OPENAI_API_KEY
            },
            data=json.dumps(body),
            timeout=10
        )
        result = response.json()
        if "choices" not in result:
            print("Unexpected API response: " + str(result))
            raise KeyError("choices")
        greeting = result["choices"][0]["message"]["content"]
        greeting = greeting.strip().encode("ascii", "ignore")
        print("ChatGPT says: " + greeting)
        return greeting
    except Exception as e:
        print("ChatGPT API error: " + str(e))
        return ("Hello, " + name + "! Great to see you.").encode("ascii", "ignore")

def bumper_pressed():
    try:
        right = memory.getData("RightBumperPressed")
        left = memory.getData("LeftBumperPressed")
        return right == 1.0 or left == 1.0
    except RuntimeError:
        return False

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

def clear_learned_faces():
    try:
        face_proxy.forgetAllFaces()
        print("Cleared all learned faces.")
        return
    except RuntimeError as e:
        print("forgetAllFaces unavailable, falling back to forgetPerson: %s" % e)
    except Exception as e:
        print("Could not clear all faces directly, falling back to forgetPerson: %s" % e)

    try:
        known_faces = face_proxy.getLearnedFacesList()
    except Exception as e:
        print("Could not list learned faces: %s" % e)
        return

    for name in known_faces:
        try:
            face_proxy.forgetPerson(name)
            print("Forgot learned face: %s" % name)
        except Exception as e:
            print("Could not forget learned face %s: %s" % (name, e))

def main():
    try:
        clear_learned_faces()
        for name in LEARNED_FACE_NAMES:
            say("Thank you. " + name + ", please keep your face in front of me while I learn it.")
            time.sleep(1.5)
            while not face_proxy.learnFace(name):
                say("I could not see you clearly. Please try again.")
                time.sleep(1.5)
        say("I have learned both faces. Show me a face I know.")
        say("I am ready. Press my foot bumper to stop.")
        last_greeted = ""
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
            names = get_recognized_faces(face_data)
            if names:
                print("Detected faces: " + str(names))
            elif face_data:
                print("Face data present but no recognized names: " + str(face_data))
            for name in names:
                if name in LEARNED_FACE_NAMES:
                    time_since_last = time.time() - last_greeted_time
                    if name != last_greeted or time_since_last > GREETING_COOLDOWN_SECONDS:
                        say(get_chatgpt_greeting(name))
                        last_greeted = name
                        last_greeted_time = time.time()
            if not names:
                last_greeted = ""
            time.sleep(POLL_INTERVAL)
    except KeyboardInterrupt:
        pass
    finally:
        face_proxy.unsubscribe("face_off_hardcoded")
        print("Done.")

if __name__ == "__main__":
    main()
