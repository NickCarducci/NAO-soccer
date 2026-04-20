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

tts = ALProxy("ALTextToSpeech", ROBOT_IP, 9559)
asr = ALProxy("ALSpeechRecognition", ROBOT_IP, 9559)
memory = ALProxy("ALMemory", ROBOT_IP, 9559)

asr.pause(True)
asr.setLanguage("English")
vocabulary = ["cake", "pie", "chocolate", "cheese", "apple", "pumpkin"]
asr.setVocabulary(vocabulary, False)
asr.pause(False)

def clear_word():
    try:
        memory.insertData("WordRecognized", [])
    except Exception:
        pass


def listen_for(choices, timeout_seconds):
    start = time.time()
    clear_word()
    while time.time() - start < timeout_seconds:
        data = memory.getData("WordRecognized")
        if isinstance(data, list) and len(data) >= 2 and data[1] > 0.2:
            raw = data[0]
            clear_word()
            if isinstance(raw, basestring):
                text = raw.lower()
                if "back" in text or "again" in text:
                    return "repeat"
                for choice in choices:
                    if choice in text:
                        return choice
        time.sleep(0.1)
    return None

try:
    asr.subscribe("asr_subscriber_main")

    while True:
        tts.say("Do you prefer cake or pie?")
        choice = listen_for(["cake", "pie"], 5)

        if choice == "repeat":
            continue
        if choice == "cake":
            while True:
                tts.say("Nice... but chocolate or cheesecake?")
                follow_up = listen_for(["chocolate", "cheese"], 5)
                if follow_up == "repeat":
                    break
                if follow_up == "chocolate":
                    tts.say("Chocolate cake - nice.  You obviously have your life together.")
                    break
                elif follow_up == "cheese":
                    tts.say("Do you even know where cheese comes from?")
                    break
                else:
                    tts.say("I said: chocolate or cheesecake?")
            if follow_up != "repeat":
                break
        elif choice == "pie":
            while True:
                tts.say("Okay - how about between apple and pumpkin?")
                follow_up = listen_for(["apple", "pumpkin"], 5)
                if follow_up == "repeat":
                    break
                if follow_up == "apple":
                    tts.say("Apple pie needs vanilla ice cream to contrast.")
                    break
                elif follow_up == "pumpkin":
                    tts.say("Pumpkin pie is honestly nasty.")
                    break
                else:
                    tts.say("I said: apple or pumpkin?")
            if follow_up != "repeat":
                break
        else:
            tts.say("I said: cake or pie for desert?")
finally:
    try:
        asr.unsubscribe("asr_subscriber_main")
    except Exception:
        pass
