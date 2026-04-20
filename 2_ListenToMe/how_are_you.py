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
vocabulary = ["good", "bad", "so-so"]
asr.setVocabulary(vocabulary, False)
asr.pause(False)

# Simulate hearing a word
# word = "good morning"

# TODO: Respond differently based on time of day greeting
# Use if/elif/else to give appropriate responses
tts.say("How are you?")

# Listen module
asr.subscribe("asr_subscriber_main")
import time
time.sleep(5)  # Listen for 5 seconds

# Get result
result = memory.getData("WordRecognized")
word = result[0]  # The recognized word

# Respond based on what was heard
if word == "good":
    tts.say("That's good to hear!")
elif word == "bad":
    tts.say("I hope your day gets better!")
elif word == "so-so":
    tts.say("Same.")
else:
    tts.say("I didn't catch that.")

asr.unsubscribe("asr_subscriber_main")
