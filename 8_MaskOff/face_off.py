#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
# -*- coding: utf-8 -*-
# Combined Module 7 script:
# detects faces, learns named faces, recognizes them later,
# turns toward sound, and optionally uses ChatGPT for greetings.
import json
import os
import select
import subprocess
import sys
import time

# Add the NAOqi Python SDK to the import path.
sdk_folder = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib/python2.7/site-packages"
sys.path.append(sdk_folder)

# Tell macOS where the NAOqi shared libraries live.
os.environ["DYLD_LIBRARY_PATH"] = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib"

try:
    # ALProxy connects Python code to individual NAOqi services.
    from naoqi import ALProxy
    print("SDK successfully loaded!")
except ImportError as e:
    print("Still cannot find naoqi. Check if this file exists:")
    print(sdk_folder + "/naoqi.py")
    print("Error details: " + str(e))
    sys.exit(1)

try:
    # Used only for the optional OpenAI API call.
    import requests
except ImportError:
    requests = None

try:
    # Optional helper for loading .env files.
    from dotenv import load_dotenv as dotenv_load
except ImportError:
    dotenv_load = None

try:
    # Python 2 has unicode; Python 3 does not. This alias keeps editors happy.
    text_type = unicode
except NameError:
    text_type = str


# Robot network settings.
ROBOT_IP = "172.16.0.29"
ROBOT_PORT = 9559

# Names used when subscribing to robot services.
FACE_SUBSCRIBER_NAME = "face_off_face_detector"
SOUND_SUBSCRIBER_NAME = "face_off_sound_localizer"
ASR_SUBSCRIBER_NAME = "face_off_name_listener"

# Loop timing and API configuration.
POLL_INTERVAL = 0.2
GREETING_COOLDOWN_SECONDS = 10.0
OPENAI_URL = "https://api.openai.com/v1/chat/completions"

# The two faces this script expects to learn and recognize.
LEARNED_FACE_NAMES = []


# Global proxy variables are assigned once the robot connection succeeds.
tts = None
memory = None
camera = None
motion = None
face_proxy = None
sound_proxy = None
speech = None


def simple_load_dotenv(env_path):
    # Minimal .env parser used if python-dotenv is unavailable.
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
    # Load environment variables from a .env file next to this script.
    env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")
    if dotenv_load is not None:
        dotenv_load(env_path)
    else:
        simple_load_dotenv(env_path)


def say(text):
    # Keep spoken text ASCII-only to avoid NAO TTS encoding issues.
    if text is None:
        return
    if isinstance(text, text_type):
        spoken = text.encode("ascii", "ignore")
    else:
        spoken = str(text)
    tts.say(spoken)


def bumper_pressed():
    # Either foot bumper is used as the stop button.
    try:
        right = memory.getData("RightBumperPressed")
        left = memory.getData("LeftBumperPressed")
        return right == 1.0 or left == 1.0
    except RuntimeError:
        return False


def head_button_pressed():
    # The front head tactile sensor triggers "learn another face" mode.
    try:
        return memory.getData("FrontTactilTouched") == 1.0
    except RuntimeError:
        return False


def face_detected(face_data):
    # ALFaceDetection writes a nested list to FaceDetected.
    # A non-empty face_data[1] means at least one face is visible.
    return bool(face_data) and len(face_data) > 1 and len(face_data[1]) > 0


def get_recognized_faces(face_data):
    # Parse the recognized-name summary from FaceDetected.
    try:
        if not face_data or len(face_data) < 2 or len(face_data[1]) == 0:
            return []

        # The last entry is the time-filtered recognition result.
        time_filtered = face_data[1][len(face_data[1]) - 1]
        if len(time_filtered) == 2 and time_filtered[0] in [2, 3]:
            return time_filtered[1]
    except Exception:
        pass
    return []


def face_is_unrecognized(face_data):
    # [4] means a face stayed visible long enough to be tracked but was not recognized.
    try:
        if not face_data or len(face_data) < 2 or len(face_data[1]) == 0:
            return False

        time_filtered = face_data[1][len(face_data[1]) - 1]
        return len(time_filtered) == 1 and time_filtered[0] == 4
    except Exception:
        return False


def clear_memory_key(key):
    try:
        memory.insertData(key, [])
    except Exception:
        pass


def configure_name_listener():
    # Set NAO's speech recognizer to listen for a minimal vocabulary (to keep the service running).
    speech.pause(True)
    speech.setLanguage("English")
    # Use a minimal vocabulary to keep ALSpeechRecognition active, but accept any heard word.
    speech.setVocabulary(["yes", "no"], False)
    speech.pause(False)


def listen_for_spoken_name(timeout_seconds, allow_known_names):
    # Listen for a spoken name from ALSpeechRecognition, accept any heard word.
    start_time = time.time()
    clear_memory_key("WordRecognized")

    while time.time() - start_time < timeout_seconds:
        if bumper_pressed():
            return None

        try:
            data = memory.getData("WordRecognized")
        except RuntimeError:
            data = []

        if isinstance(data, list) and len(data) >= 2 and data[1] > 0.35:
            heard = data[0]
            clear_memory_key("WordRecognized")
            if isinstance(heard, basestring):
                spoken_name = heard.strip().lower()
                existing_names = []
                try:
                    existing_names = [name.lower() for name in face_proxy.getLearnedFacesList()]
                except Exception:
                    pass

                if spoken_name in existing_names and not allow_known_names:
                    say("I already know " + spoken_name + ". Please say a different name.")
                else:
                    return spoken_name
        time.sleep(0.1)

    return None


def validate_candidate_name(candidate_name, allow_known_names):
    # Accept any input as a name, no validation.
    if not isinstance(candidate_name, basestring):
        return None
    return candidate_name.strip()


def read_terminal_name_nonblocking(allow_known_names):
    # Check for terminal input without blocking the robot loop.
    try:
        ready, _, _ = select.select([sys.stdin], [], [], 0)
    except Exception:
        return None

    if not ready:
        return None

    typed_name = sys.stdin.readline()
    if not typed_name:
        return None

    validated_name = validate_candidate_name(typed_name, allow_known_names)
    return validated_name


def ask_for_name(prompt_text, allow_known_names, timeout_seconds):
    # Always finish prompt, listen for both spoken and terminal input for the full timeout, only exit on bumper press.
    say(prompt_text)
    print("Waiting for a spoken name or terminal input for up to %.1f seconds..." % timeout_seconds)
    print("Terminal fallback: type any name and press Enter.")

    start_time = time.time()
    clear_memory_key("WordRecognized")
    received_name = None

    while time.time() - start_time < timeout_seconds:
        if bumper_pressed():
            return None

        # Check for terminal input
        typed_name = read_terminal_name_nonblocking(allow_known_names)
        if typed_name is not None and not received_name:
            say("I will use the typed name " + typed_name + ".")
            clear_memory_key("WordRecognized")
            received_name = typed_name

        # If no terminal input, try external speech-to-text (Python 3 Whisper helper)
        if not received_name:
            try:
                # Call the Python 3 Whisper script for offline speech-to-text
                result = subprocess.check_output([
                    "python3", "record_and_transcribe_whisper.py", str(timeout_seconds)
                ], cwd=os.path.dirname(os.path.abspath(__file__)))
                stt_name = result.decode("utf-8").strip()
                if stt_name:
                    say("I will use the spoken name " + stt_name + ".")
                    received_name = stt_name
            except Exception as e:
                print("[Speech-to-text error]", e)

        if received_name:
            return received_name

        time.sleep(0.1)

    say("I did not catch a name.")
    return None


def turn_toward_sound():
    # Read sound localization and rotate the head toward the sound source.
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
    # Clear any stale FaceDetected data before the loop starts.
    try:
        memory.insertData("FaceDetected", [])
    except Exception:
        pass


def learn_face(prompt_for_name=True):
    # Ask for a name, then keep asking until the robot successfully stores a face under this name.
    while True:
        if bumper_pressed():
            say("Goodbye!")
            return False

        if prompt_for_name:
            name = ask_for_name("What is your name?", allow_known_names=True, timeout_seconds=10.0)
            if not name:
                say("I did not catch your name. Let's try again.")
                continue
        else:
            # Use the provided name argument (for initial two faces)
            pass

        say("Thank you. " + name + ", please keep your face in front of me while I learn it.")
        time.sleep(1.5)

        # This is where the face is actually learned and saved by name.
        success = face_proxy.learnFace(name)
        if success:
            say("Got it. I have learned your face, " + name + ".")
            return True

        say("I could not see you clearly. Please try again.")


    # Always clear the robot's stored face database at startup.
    try:
        face_proxy.forgetAllFaces()
        print("Cleared all learned faces.")
    except Exception:
        pass

    # Teach the two faces at startup, prompting for a name each time.
    say("I need to learn two faces before I can recognize them by name.")
    for _ in range(2):
        if not learn_face(prompt_for_name=True):
            return False

    say("I have learned both faces. Show me a face I know.")
    return True


def get_openai_api_key():
    # Read the API key from .env.
    load_env_file()
    return os.getenv("OPENAI_API_KEY")


def get_chatgpt_greeting(name, api_key):
    # Fallback greeting used if the API is unavailable.
    fallback = ("Hello, " + name + "!").encode("ascii", "ignore")

    if not api_key or requests is None:
        return fallback

    # Chat Completions request payload.
    body = {
        "model": "gpt-3.5-turbo",
        "messages": [
            {
                # system sets the style/personality of the response.
                "role": "system",
                "content": (
                    "You are a friendly robot named NAO. "
                    "Generate short warm greetings. "
                    "One or two sentences. No emoji."
                ),
            },
            {
                # user asks for a greeting for the recognized person.
                "role": "user",
                "content": "Generate a greeting for " + name,
            },
        ],
        "max_tokens": 60,
    }

    try:
        # Send the request to OpenAI and extract the returned text.
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
        # Fall back gracefully if the API call fails.
        print("API error: %s" % e)
        return fallback


def connect_proxies():
    # Build one proxy per robot service used in this script.
    global tts, memory, camera, motion, face_proxy, sound_proxy, speech

    tts = ALProxy("ALTextToSpeech", ROBOT_IP, ROBOT_PORT)
    memory = ALProxy("ALMemory", ROBOT_IP, ROBOT_PORT)
    camera = ALProxy("ALVideoDevice", ROBOT_IP, ROBOT_PORT)
    motion = ALProxy("ALMotion", ROBOT_IP, ROBOT_PORT)
    face_proxy = ALProxy("ALFaceDetection", ROBOT_IP, ROBOT_PORT)
    sound_proxy = ALProxy("ALSoundLocalization", ROBOT_IP, ROBOT_PORT)
    speech = ALProxy("ALSpeechRecognition", ROBOT_IP, ROBOT_PORT)


def start_services():
    # Use the top camera and start face and sound services.
    camera.setActiveCamera(0)
    face_proxy.subscribe(FACE_SUBSCRIBER_NAME, 500, 0.0)
    sound_proxy.subscribe(SOUND_SUBSCRIBER_NAME)
    configure_name_listener()
    speech.subscribe(ASR_SUBSCRIBER_NAME)
    clear_face_memory()
    clear_memory_key("WordRecognized")


def stop_services():
    # Always unsubscribe so services do not stay active after the script ends.
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
    if speech is not None:
        try:
            speech.unsubscribe(ASR_SUBSCRIBER_NAME)
        except Exception:
            pass


def run_face_off():
    # Load the OpenAI key if available. The script can still run without it.
    api_key = get_openai_api_key()
    if not api_key:
        print("OPENAI_API_KEY not found in .env. Using fallback greetings.")
    if requests is None:
        print("requests is not installed. Using fallback greetings.")

    connect_proxies()
    start_services()

    # Make sure the two named faces are stored before trying to recognize them.
    if not ensure_known_faces():
        return

    # Opening spoken prompt for the combined assignment behavior.
    say("I am ready. Press my foot bumper to stop. Press my head button if you want me to learn another face by hearing a spoken name.")

    # Prevent repeated generic greetings while the same face stays in view.
    greeted_any_face = False

    # Prevent repeated recognized-name greetings too frequently.
    last_greeted_name = ""
    last_greeted_time = 0
    last_learn_time = 0
    SUPPRESS_UNKNOWN_PROMPT_SECONDS = 5.0

    while True:
        # Stop immediately if either foot bumper is pressed.
        if bumper_pressed():
            say("Goodbye!")
            break

        # Above-and-beyond flow: pressing the head button enters "learn more faces" mode.
        # After learning, control returns to the same recognition loop.
        if head_button_pressed():
            if not head_button_latched:
                head_button_latched = True
                extra_name = ask_for_name(
                    "Please say a new name and let me look at you.",
                    allow_known_names=False,
                    timeout_seconds=8.0,
                )
                if extra_name:
                    say("I heard " + extra_name + ". I'll remember you.")
                    if learn_face(extra_name):
                        last_greeted_name = ""
                        last_greeted_time = 0
                        unknown_face_prompted = False
                        last_learn_time = time.time()
                        say("I can now recognize " + extra_name + ", too.")
                else:
                    say("Remaining in recognition mode.")
        else:
            head_button_latched = False

        # Keep turning toward the latest sound.
        turn_toward_sound()

        try:
            # Read the latest face detection / recognition data from ALMemory.
            face_data = memory.getData("FaceDetected")
        except RuntimeError:
            face_data = []

        # Basic task behavior: greet when any face is seen.
        if face_detected(face_data):
            if not greeted_any_face:
                say("Hello, human.")
                greeted_any_face = True
        else:
            # Reset when the face disappears so the next appearance can be greeted.
            greeted_any_face = False
            last_greeted_name = ""
            unknown_face_prompted = False

        # Intermediate/advanced behavior: recognize any learned faces by name.
        recognized_names = get_recognized_faces(face_data)
        for name in recognized_names:
            time_since_last = time.time() - last_greeted_time
            if name != last_greeted_name or time_since_last > GREETING_COOLDOWN_SECONDS:
                # Advanced task behavior: ask ChatGPT for a dynamic greeting.
                greeting = get_chatgpt_greeting(name, api_key)
                say(greeting)
                last_greeted_name = name
                last_greeted_time = time.time()

        # If the face is visible but not recognized, allow the user to teach a new name.
        suppress_prompt = (time.time() - last_learn_time) < SUPPRESS_UNKNOWN_PROMPT_SECONDS
        if face_is_unrecognized(face_data) and not recognized_names and not unknown_face_prompted and not suppress_prompt:
            new_name = ask_for_name(
                "Who are you, then? What's your name?",
                allow_known_names=False,
                timeout_seconds=8.0,
            )
            unknown_face_prompted = True

            if new_name:
                say("Okay. I will learn " + new_name + " now.")
                if learn_face(new_name):
                    last_greeted_name = ""
                    last_greeted_time = 0
                    last_learn_time = time.time()
            else:
                say("Okay. I will skip learning this face for now.")

        # Small sleep keeps the loop responsive without running too aggressively.
        time.sleep(POLL_INTERVAL)


def ensure_known_faces():
    # Always clear the robot's stored face database at startup.
    try:
        face_proxy.forgetAllFaces()
        print("Cleared all learned faces.")
    except Exception:
        pass

    # Teach the two faces at startup, prompting for a name each time.
    say("I need to learn two faces before I can recognize them by name.")
    for _ in range(2):
        if not learn_face(prompt_for_name=True):
            return False

    say("I have learned both faces. Show me a face I know.")
    return True


def main():
    try:
        # Start the full script.
        run_face_off()
    except KeyboardInterrupt:
        print("Stopped.")
    except Exception as e:
        print("Error connecting to robot: ")
        print(e)
        sys.exit(1)
    finally:
        # Cleanup runs no matter how the script exits.
        stop_services()


if __name__ == "__main__":
    main()
