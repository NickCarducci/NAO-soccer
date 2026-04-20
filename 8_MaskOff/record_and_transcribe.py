#!/usr/bin/env python3
"""
record_and_transcribe.py

Records audio from the default microphone and transcribes it to text using Vosk (offline, no API key required).
Usage:
    python3 record_and_transcribe.py [seconds]

Outputs the recognized text to stdout.
"""
import sys
import queue
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import json

# Duration of recording (seconds)
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
SAMPLE_RATE = 16000
MODEL_PATH = "vosk-model-small-en-us-0.15"  # Download from https://alphacephei.com/vosk/models

try:
    model = Model(MODEL_PATH)
except Exception as e:
    print("Could not load Vosk model. Download and unzip a model from https://alphacephei.com/vosk/models and set MODEL_PATH.")
    print(e)
    sys.exit(1)

q = queue.Queue()
def callback(indata, frames, time, status):
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

rec = KaldiRecognizer(model, SAMPLE_RATE)
print("Speak now ({} seconds)...".format(DURATION))
with sd.RawInputStream(samplerate=SAMPLE_RATE, blocksize = 8000, dtype='int16', channels=1, callback=callback):
    for _ in range(int(SAMPLE_RATE / 8000 * DURATION)):
        data = q.get()
        if rec.AcceptWaveform(data):
            pass

result = rec.FinalResult()
try:
    text = json.loads(result)["text"]
except Exception:
    text = ""
print(text.strip())
