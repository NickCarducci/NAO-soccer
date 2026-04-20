#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
import os
import sys

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

tts = None
video_proxy = None


def list_subscribers():
    subscribers = video_proxy.getSubscribers()
    print("Current ALVideoDevice subscribers:")
    if not subscribers:
        print("  (none)")
        return []
    for subscriber in subscribers:
        print("  - %s" % subscriber)
    return subscribers


def unsubscribe_all(subscribers):
    if not subscribers:
        print("No subscribers to unsubscribe.")
        return
    for subscriber in subscribers:
        try:
            video_proxy.unsubscribe(subscriber)
            print("Unsubscribed: %s" % subscriber)
        except Exception as e:
            print("Failed to unsubscribe %s: %s" % (subscriber, e))


try:
    tts = ALProxy("ALTextToSpeech", ROBOT_IP, ROBOT_PORT)
    video_proxy = ALProxy("ALVideoDevice", ROBOT_IP, ROBOT_PORT)

    subscribers = list_subscribers()
    unsubscribe_all(subscribers)
    print("")
    print("After cleanup:")
    list_subscribers()
    tts.say("Video subscriptions cleaned up.")

except KeyboardInterrupt:
    print("Stopped.")
except Exception as e:
    print("Error connecting to robot: ")
    print(e)
    sys.exit(1)
