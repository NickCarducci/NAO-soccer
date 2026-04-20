#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
import sys
import os

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

# Robot Connection
ROBOT_IP = "172.16.0.5"

try:
    memory = ALProxy("ALMemory", ROBOT_IP, 9559)
except Exception as e:
    print("Error connecting to robot: ")
    print(e)
    sys.exit(1)


def show_subscribers(key):
    try:
        subs = memory.getSubscribers(key)
        print("%s subscribers:" % key)
        for sub in subs:
            print("  - %s" % sub)
        if not subs:
            print("  (none)")
    except Exception as e:
        print("Could not read subscribers for %s: %s" % (key, e))


show_subscribers("WordRecognized")
show_subscribers("SoundDetected")
