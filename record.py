import serial
import numpy as np
from scipy.io.wavfile import write
import time
import os

ser = serial.Serial('COM9', 115200, timeout=1)
OUTPUT_DIR = "speech_data/START"
NUM_RECORDINGS = 50
FS = 8000  # fixed (recommended for your AVR system)
os.makedirs(OUTPUT_DIR, exist_ok=True)

print("Waiting for START...")

for rec in range(NUM_RECORDINGS):

    samples = []
    recording = False

    print(f"\n--- Recording {rec+1}/{NUM_RECORDINGS} ---")

    while True:

        if not recording:
            # WAIT FOR START
            line = ser.readline()

            if b"START" in line:
                print("Recording started")
                start = time.time()
                samples = []
                recording = True

        else:
            # RECORD AUDIO STREAM
            byte = ser.read(1)

            if not byte:
                continue

            # STOP detection
            if byte == b'S':
                rest = ser.read(4)  # "TOP\n"
                if rest == b"TOP\n":
                    print("Recording stopped")
                    end = time.time()
                    break
                else:
                    samples.append(byte[0])
                    for b in rest:
                        samples.append(b)
            else:
                samples.append(byte[0])

    # ---------- Convert to WAV ----------
    audio = np.array(samples, dtype=np.uint8)

    print("Samples:", len(samples))

    # Convert 8-bit unsigned → signed PCM
    audio = (audio.astype(np.int16) - 128) * 256

    # Save file
    filename = os.path.join(
        OUTPUT_DIR,
        f"START_{rec+1:02d}.wav"
    )    
    write(filename, FS, audio)

    print("Saved:", filename)

print(f"\nAll {NUM_RECORDINGS} recordings completed.")