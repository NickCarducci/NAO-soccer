#!/usr/bin/env python3
"""
record_and_transcribe_vosk_or_voskless.py

Records audio and transcribes it to text using Vosk if available, otherwise falls back to the built-in Python speech_recognition library with the offline Sphinx recognizer (no API key required).

Usage:
    python3 record_and_transcribe_vosk_or_voskless.py [seconds]

Outputs the recognized text to stdout.
"""
import sys
import queue

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0

try:
    from vosk import Model, KaldiRecognizer
    import sounddevice as sd
    import json
    SAMPLE_RATE = 16000
    MODEL_PATH = "vosk-model-small-en-us-0.15"
    model = Model(MODEL_PATH)
    q = queue.Queue()
    def callback(indata, frames, time, status):
        q.put(bytes(indata))
    rec = KaldiRecognizer(model, SAMPLE_RATE)
    print("Speak now ({} seconds, Vosk)...".format(DURATION))
    with sd.RawInputStream(samplerate=SAMPLE_RATE, blocksize=8000, dtype='int16', channels=1, callback=callback):
        for _ in range(int(SAMPLE_RATE / 8000 * DURATION)):
            data = q.get()
            if rec.AcceptWaveform(data):
                pass
    result = rec.FinalResult()
    text = json.loads(result)["text"]
    print(text.strip())
except Exception as vosk_error:
    print("[Vosk unavailable, falling back to Sphinx]", file=sys.stderr)
    try:
        import speech_recognition as sr
        r = sr.Recognizer()
        with sr.Microphone() as source:
            print("Speak now ({} seconds, Sphinx)...".format(DURATION))
            audio = r.listen(source, timeout=DURATION, phrase_time_limit=DURATION)
        try:
            text = r.recognize_sphinx(audio)
        except Exception as e:
            text = ""
        print(text.strip())
    except Exception as sr_error:
        print("[No offline speech recognition available]", file=sys.stderr)
        print("")
