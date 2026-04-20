#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7
import os, sys, time

sdk_folder = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib/python2.7/site-packages"
sys.path.append(sdk_folder)
os.environ["DYLD_LIBRARY_PATH"] = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib"

from naoqi import ALProxy

memory = ALProxy("ALMemory", "172.16.0.3", 9559)

print("Tap the head buttons. Ctrl+C to stop.")
while True:
    front  = memory.getData("FrontTactilTouched")
    middle = memory.getData("MiddleTactilTouched")
    rear   = memory.getData("RearTactilTouched")
    print("Front: {}  Middle: {}  Rear: {}".format(front, middle, rear))
    time.sleep(0.2)
