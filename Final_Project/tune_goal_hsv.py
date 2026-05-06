#!/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python
"""
HSV color range tuner for neon yellow soccer goals.

This version runs on Python 2.7 inside the pyenv environment and avoids
OpenCV, which is not available here. It captures a frame from the robot,
thresholds it in pure numpy, writes preview images to /tmp, and lets you
iterate on HSV values from the terminal.

Usage:
    ROBOT_IP=172.16.0.2 ./tune_goal_hsv.py

Optional overrides:
    CAMERA_ID=0 or 1
    H_LOW=40 H_HIGH=70 S_LOW=100 S_HIGH=255 V_LOW=100 V_HIGH=255
"""
import os
import sys
import time
import math

if sys.version_info[0] != 2:
    py2 = "/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python"
    os.execv(py2, [py2] + sys.argv)

sdk_folder = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib/python2.7/site-packages"
sys.path.append(sdk_folder)
os.environ["DYLD_LIBRARY_PATH"] = "/Users/nicholascarducci/Desktop/naoqi-sqk/lib"

from naoqi import ALProxy

try:
    import numpy as np
except ImportError:
    print("ERROR: numpy is required in the Python 2.7 environment.")
    print("Install it with:")
    print("  /Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python -m pip install numpy==1.16.6")
    sys.exit(1)

ROBOT_IP = os.environ.get("ROBOT_IP", "172.16.0.2")
ROBOT_PORT = 9559
CAMERA_ID = int(os.environ.get("CAMERA_ID", "0"))  # 0=top, 1=bottom

VIDEO_RESOLUTION = 2       # VGA, 640x480
VIDEO_COLORSPACE_RGB = 11  # kRGBColorSpace
VIDEO_FPS = 10

H_LOW = int(os.environ.get("H_LOW", "40"))
H_HIGH = int(os.environ.get("H_HIGH", "70"))
S_LOW = int(os.environ.get("S_LOW", "100"))
S_HIGH = int(os.environ.get("S_HIGH", "255"))
V_LOW = int(os.environ.get("V_LOW", "100"))
V_HIGH = int(os.environ.get("V_HIGH", "255"))

OUTPUT_DIR = os.environ.get("GOAL_DEBUG_FRAMES", "/tmp")


def _capture_rgb_frame(video, client):
    try:
        result = video.getImageRemote(client)
        if result is None:
            return None
        width, height, channels, img_buffer = result[0], result[1], result[2], result[6]
        img = np.frombuffer(img_buffer, dtype=np.uint8)
        return img.reshape((height, width, channels))
    except Exception as e:
        print("Frame capture failed: {}".format(e))
        return None


def _rgb_to_hsv_opencv_style(rgb):
    rgb = rgb.astype(np.float32) / 255.0
    r = rgb[:, :, 0]
    g = rgb[:, :, 1]
    b = rgb[:, :, 2]

    maxc = np.maximum(np.maximum(r, g), b)
    minc = np.minimum(np.minimum(r, g), b)
    delta = maxc - minc

    h = np.zeros_like(maxc)
    nonzero = delta > 0.0

    mask = (maxc == r) & nonzero
    h[mask] = ((g[mask] - b[mask]) / delta[mask]) % 6.0

    mask = (maxc == g) & nonzero
    h[mask] = ((b[mask] - r[mask]) / delta[mask]) + 2.0

    mask = (maxc == b) & nonzero
    h[mask] = ((r[mask] - g[mask]) / delta[mask]) + 4.0

    h = (h * 30.0) % 180.0
    s = np.zeros_like(maxc)
    s[maxc > 0.0] = (delta[maxc > 0.0] / maxc[maxc > 0.0]) * 255.0
    v = maxc * 255.0

    hsv = np.dstack((h, s, v)).astype(np.uint8)
    return hsv


def _threshold_hsv(hsv, h_low, h_high, s_low, s_high, v_low, v_high):
    h = hsv[:, :, 0]
    s = hsv[:, :, 1]
    v = hsv[:, :, 2]
    return ((h >= h_low) & (h <= h_high) &
            (s >= s_low) & (s <= s_high) &
            (v >= v_low) & (v <= v_high))


def _write_pgm(path, mask):
    height, width = mask.shape
    with open(path, "wb") as handle:
        handle.write("P5\n{} {}\n255\n".format(width, height))
        handle.write(mask.astype(np.uint8).tostring())


def _write_ppm(path, rgb):
    height, width, _ = rgb.shape
    with open(path, "wb") as handle:
        handle.write("P6\n{} {}\n255\n".format(width, height))
        handle.write(rgb.astype(np.uint8).tostring())


def _print_current_values():
    print("Current thresholds:")
    print("  H_LOW={} H_HIGH={}".format(H_LOW, H_HIGH))
    print("  S_LOW={} S_HIGH={}".format(S_LOW, S_HIGH))
    print("  V_LOW={} V_HIGH={}".format(V_LOW, V_HIGH))


def _print_save_instructions():
    print("\nSave these values into dogs_1v1.py or export them before launch:")
    print("  GOAL_HSV_H_LOW={} GOAL_HSV_H_HIGH={}".format(H_LOW, H_HIGH))
    print("  GOAL_HSV_S_LOW={} GOAL_HSV_S_HIGH={}".format(S_LOW, S_HIGH))
    print("  GOAL_HSV_V_LOW={} GOAL_HSV_V_HIGH={}".format(V_LOW, V_HIGH))


def main():
    global H_LOW, H_HIGH, S_LOW, S_HIGH, V_LOW, V_HIGH

    print("Connecting to NAO at {}:{}...".format(ROBOT_IP, ROBOT_PORT))
    try:
        video = ALProxy("ALVideoDevice", ROBOT_IP, ROBOT_PORT)
        video.setActiveCamera(CAMERA_ID)
        client = video.subscribeCamera(
            "GoalHSVTuner", CAMERA_ID, VIDEO_RESOLUTION,
            VIDEO_COLORSPACE_RGB, VIDEO_FPS)
        print("Connected. Camera {}.".format(CAMERA_ID))
    except Exception as e:
        print("ERROR: Could not connect to NAO: {}".format(e))
        return 1

    print("")
    print("This environment does not have OpenCV, so the tuner runs in the terminal.")
    print("Each iteration saves /tmp/goal_tuner_raw.ppm and /tmp/goal_tuner_mask.pgm.")
    print("Open those files in Preview if you want a visual check.")
    print("Enter 6 numbers to change thresholds: H_LOW H_HIGH S_LOW S_HIGH V_LOW V_HIGH")
    print("Press Enter to reuse the current values, 's' to print export lines, or 'q' to quit.")

    try:
        while True:
            _print_current_values()

            rgb = _capture_rgb_frame(video, client)
            if rgb is None:
                time.sleep(0.2)
                continue

            hsv = _rgb_to_hsv_opencv_style(rgb)
            mask = _threshold_hsv(hsv, H_LOW, H_HIGH, S_LOW, S_HIGH, V_LOW, V_HIGH)
            white_pixels = int(mask.sum())
            total_pixels = int(mask.size)
            ratio = 0.0 if total_pixels == 0 else float(white_pixels) / float(total_pixels)

            raw_path = os.path.join(OUTPUT_DIR, "goal_tuner_raw.ppm")
            mask_path = os.path.join(OUTPUT_DIR, "goal_tuner_mask.pgm")
            try:
                _write_ppm(raw_path, rgb)
                _write_pgm(mask_path, mask.astype(np.uint8) * 255)
            except Exception as e:
                print("Warning: could not write preview images: {}".format(e))

            print("Detected pixels: {} / {} ({:.3f}%)".format(
                white_pixels, total_pixels, ratio * 100.0))
            print("Preview files:")
            print("  {}".format(raw_path))
            print("  {}".format(mask_path))

            response = raw_input("New thresholds or command: ").strip()
            if not response:
                print("")
                continue
            if response.lower() in ("q", "quit", "exit"):
                break
            if response.lower() == "s":
                _print_save_instructions()
                print("")
                continue

            parts = response.split()
            if len(parts) != 6:
                print("Expected 6 integers or a command. Try again.\n")
                continue

            try:
                H_LOW = max(0, min(179, int(parts[0])))
                H_HIGH = max(0, min(179, int(parts[1])))
                S_LOW = max(0, min(255, int(parts[2])))
                S_HIGH = max(0, min(255, int(parts[3])))
                V_LOW = max(0, min(255, int(parts[4])))
                V_HIGH = max(0, min(255, int(parts[5])))
                print("")
            except Exception:
                print("Could not parse those values. Try again.\n")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            video.unsubscribe(client)
        except Exception:
            pass
        print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
