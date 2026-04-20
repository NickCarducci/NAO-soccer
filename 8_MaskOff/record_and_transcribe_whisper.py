#!/usr/bin/env python3
"""
record_and_transcribe_whisper.py

Records audio from the default microphone and transcribes it to text using OpenAI Whisper (offline, no API key required).
Usage:
    python3 record_and_transcribe_whisper.py [seconds]

Outputs the recognized text to stdout.
"""
import sys
import tempfile
import sounddevice as sd
import numpy as np
import whisper

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
SAMPLE_RATE = 16000
MODEL = "base"  # You can use "tiny", "base", "small", "medium", "large"

print("Speak now ({} seconds, Whisper)...".format(DURATION))

# Record audio
recording = sd.rec(int(DURATION * SAMPLE_RATE), samplerate=SAMPLE_RATE, channels=1, dtype='float32')
sd.wait()

# Save to a temporary WAV file
with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmpfile:
    import scipy.io.wavfile
    scipy.io.wavfile.write(tmpfile.name, SAMPLE_RATE, (recording * 32767).astype(np.int16))
    wav_path = tmpfile.name

# Load Whisper model
model = whisper.load_model(MODEL)
result = model.transcribe(wav_path, fp16=False)
text = result["text"].strip()
print(text)
