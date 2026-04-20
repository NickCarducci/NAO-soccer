#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
import sys
import os
import time

# 1. The folder path (MUST end at site-packages)
sdk_folder = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib/python2.7/site-packages"
sys.path.append(sdk_folder)

# 2. Tell the system where the 'binaries' are (the .so files)
os.environ["DYLD_LIBRARY_PATH"] = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib"

try:
    from naoqi import ALProxy
    import vision_definitions
    print("SDK successfully loaded!")
except ImportError as e:
    print("Still cannot find naoqi. Check if this file exists:")
    print(sdk_folder + "/naoqi.py")
    print("Error details: " + str(e))
    sys.exit(1)

# Robot Connection
ROBOT_IP = "172.16.0.5"

try:
    tts = ALProxy("ALTextToSpeech", ROBOT_IP, 9559)
    memory = ALProxy("ALMemory", ROBOT_IP, 9559)
    speech = ALProxy("ALSpeechRecognition", ROBOT_IP, 9559)
    sound = ALProxy("ALSoundDetection", ROBOT_IP, 9559)
    
    print("Success: Robot spoke!")
except Exception as e:
    print("Error connecting to robot: ")
    print(e)
    sys.exit(1)

INITIAL_PROMPT = "How are you doing today?  I only can hear 'good', 'bad', or 'so-so'."
DEFAULT_PROMPT = "Let's try that again.  Tell me whether your day was good, bad, or so-so."
RESPONSES = {
    "good": "Yeah well it looks like rain.",
    "bad": "Ah chin up, you look ugly when you're upset.",
    "so-so": "Life is like a roller coaster\n"
             "It has some ups and downs\n"
             "Sometimes you can take it slow or very fast\n"
             "It maybe hard to breath at times\n"
             "But you have to push yourself and keep going\n"
             "Your bar is your safety\n"
             "It's like your family and friends\n"
             "You hold on tight and you don't let go\n"
             "But sometimes you might throw your hands up\n"
             "Because your friends and family will always be with you\n"
             "Just like that bar keeping you safe at all times"
}

DEBUG = True


def clear_memory_key(key):
    try:
        memory.insertData(key, [])
    except Exception:
        pass


def wait_for_reply(timeout_seconds):
    start_time = time.time()
    last_activity = start_time
    last_prompt_time = 0.0

    clear_memory_key("WordRecognized")
    clear_memory_key("SoundDetected")

    while True:
        now = time.time()

        if now - last_activity >= timeout_seconds:
            return "timeout", None

        word_data = memory.getData("WordRecognized")
        if DEBUG:
            print("WordRecognized:", word_data)
        if isinstance(word_data, list) and len(word_data) >= 2 and word_data[1] > 0.4:
            last_activity = now
            word_raw = word_data[0]
            clear_memory_key("WordRecognized")
            word_norm = ""
            if isinstance(word_raw, basestring):
                word_norm = word_raw.lower()
            for key in RESPONSES:
                if key in word_norm:
                    return "recognized", key
            if now - last_prompt_time > 1.0:
                tts.say(DEFAULT_PROMPT)
                last_prompt_time = now

        sound_data = memory.getData("SoundDetected")
        if DEBUG and sound_data:
            print("SoundDetected:", sound_data)
        if sound_data:
            last_activity = now
            clear_memory_key("SoundDetected")
            if now - last_prompt_time > 1.0:
                tts.say(DEFAULT_PROMPT)
                last_prompt_time = now

        time.sleep(0.1)


try:
    speech.pause(True)
    speech.setLanguage("English")
    speech.setVocabulary(["good", "bad", "so-so"], True)
    speech.pause(False)
    speech.subscribe("asr_subscriber_main")

    sound.setParameter("Sensitivity", 0.6)
    sound.subscribe("sound_subscriber_main")

    tts.say(INITIAL_PROMPT)

    status, word = wait_for_reply(30)

    if status == "timeout":
        tts.say("Times up!  What a waste of time.")
    elif status == "recognized":
        tts.say(RESPONSES[word])
    else:
        tts.say("Times up!  What a waste of time.")

    tts.say("\\vct=50\\Brrrp!\\vct=100\\ Goodbye!")
finally:
    try:
        speech.unsubscribe("asr_subscriber_main")
    except Exception:
        pass
    try:
        sound.unsubscribe("sound_subscriber_main")
    except Exception:
        pass
