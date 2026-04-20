# -*- coding: utf-8 -*-
# MODULE 7 - FACE OFF
# Task 3: AI-Powered Greetings with ChatGPT (Advanced Task)
#
# NAO recognizes faces and uses ChatGPT to generate a unique greeting
# for each person it sees.
#
# HOW TO STOP:
#   - Press either foot bumper on NAO (recommended)
#   - Or press Ctrl+C in the terminal
#
# SETUP BEFORE RUNNING:
#   1. Your instructor will provide you with an OpenAI API key
#   2. Create a file called .env in the same folder as this script
#   3. Add this line to it: OPENAI_API_KEY=your_key_here
#   4. Install required packages: pip install requests python-dotenv
#   5. Run Task 2 first so NAO has learned the faces

import os
import time
import json
import requests
from dotenv import load_dotenv
from naoqi import ALProxy

# -- Load API key from .env file ---------------------------------------------
load_dotenv(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env"))
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

if not OPENAI_API_KEY:
    print("ERROR: OPENAI_API_KEY not found.")
    print("Make sure your .env file exists and contains OPENAI_API_KEY=your_key_here")
    exit(1)

# -- Connection settings --
IP         = "192.168.x.x"   # TODO: replace with your robot's IP address
PORT       = 9559
OPENAI_URL = "https://api.openai.com/v1/chat/completions"

# -- Set up proxies --
tts         = ALProxy("ALTextToSpeech",  IP, PORT)
memory      = ALProxy("ALMemory",        IP, PORT)
camera      = ALProxy("ALVideoDevice",   IP, PORT)
face_detect = ALProxy("ALFaceDetection", IP, PORT)

# -- Setup --
camera.setActiveCamera(0)
face_detect.subscribe("chatgpt_task")
tts.say("Ready. Show me a face I know and I will greet them. Press my foot bumper to stop.")
print("Running. Press a foot bumper or Ctrl+C to stop.")


# -- Helper: ask ChatGPT to generate a greeting ------------------------------
def get_chatgpt_greeting(name):
    """
    Sends a prompt to the ChatGPT API and returns a short greeting string.
    Falls back to a simple greeting if the API call fails.

    API structure:
      - system role: sets ChatGPT's personality
      - user role:   the actual request
      - response at: result["choices"][0]["message"]["content"]

    encode("ascii", "ignore") converts the response to plain ASCII
    because NAO's text-to-speech cannot handle Unicode characters.
    """
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

        result   = response.json()
        greeting = result["choices"][0]["message"]["content"]
        greeting = greeting.strip().encode("ascii", "ignore")

        print("ChatGPT says: " + greeting)
        return greeting

    except Exception as e:
        print("ChatGPT API error: " + str(e))
        return ("Hello, " + name + "! Great to see you.").encode("ascii", "ignore")


# -- Helper: parse recognized names from ALMemory ----------------------------
def get_recognized_faces(face_data):
    """
    Parses FaceDetected and returns a list of recognized name strings.
    Returns [] if no faces are recognized.
    """
    try:
        if not face_data or len(face_data) < 2 or len(face_data[1]) == 0:
            return []
        time_filtered = face_data[1][len(face_data[1]) - 1]
        if len(time_filtered) == 2 and time_filtered[0] in [2, 3]:
            return time_filtered[1]
    except Exception:
        pass
    return []


# -- Recognition loop --------------------------------------------------------
last_greeted      = ""
last_greeted_time = 0

try:
    while True:

        # -- Foot bumper stop -------------------------------------------------
        try:
            right = memory.getData("RightBumperPressed")
            left  = memory.getData("LeftBumperPressed")
            if right == 1.0 or left == 1.0:
                tts.say("Goodbye!")
                break
        except RuntimeError:
            pass

        # -- Face recognition and ChatGPT greeting ----------------------------
        try:
            face_data = memory.getData("FaceDetected")
            names = get_recognized_faces(face_data)
        except RuntimeError:
            names = []

        for name in names:
            time_since_last = time.time() - last_greeted_time
            if name != last_greeted or time_since_last > 10.0:
                greeting = get_chatgpt_greeting(name)
                tts.say(greeting)
                last_greeted      = name
                last_greeted_time = time.time()

        if not names:
            last_greeted = ""

        time.sleep(0.2)

except KeyboardInterrupt:
    pass

finally:
    face_detect.unsubscribe("chatgpt_task")
    print("Done.")
