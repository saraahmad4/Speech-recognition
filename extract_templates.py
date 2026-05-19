import os
import numpy as np
from scipy.io import wavfile

# =========================================================
# Configuration
# =========================================================
DATA_DIR = "speech_data"
TARGET_SR = 8000
FRAME_SIZE = 128
NUM_FRAMES = 50
NUM_SEGMENTS = 5
FRAMES_PER_SEGMENT = 10
PREROLL_FRAMES = 5
DEADZONE = 10
EPSILON = 1e-12

# =========================================================
# Audio helpers
# =========================================================

def load_wav_mono(filepath, target_sr=TARGET_SR):
    """Load WAV, convert stereo to mono, normalize, and resample to 8 kHz."""
    sr, audio = wavfile.read(filepath)

    if audio.ndim > 1:
        audio = np.mean(audio, axis=1)

    if audio.dtype == np.uint8:
        audio = audio.astype(np.float32) - 128.0
    elif audio.dtype == np.int16:
        audio = audio.astype(np.float32)
    elif audio.dtype == np.int32:
        audio = (audio.astype(np.float32) / 2**15)
    elif audio.dtype == np.float32 or audio.dtype == np.float64:
        audio = audio.astype(np.float32)
    else:
        audio = audio.astype(np.float32)

    max_val = np.max(np.abs(audio))
    if max_val > 0:
        audio = audio / max_val

    if sr != target_sr and len(audio) > 1:
        audio = resample_audio(audio, sr, target_sr)

    return audio


def resample_audio(audio, original_sr, target_sr):
    """Resample using linear interpolation so we keep dependencies light."""
    original_len = len(audio)
    if original_len == 0:
        return audio

    target_len = int(np.round(original_len * target_sr / original_sr))
    if target_len < 1:
        target_len = 1

    positions = np.linspace(0, original_len - 1, target_len)
    return np.interp(positions, np.arange(original_len), audio).astype(np.float32)


def frame_generator(audio, frame_size=FRAME_SIZE):
    """Yield fixed-size frames, padding the final incomplete frame with zeros."""
    total = len(audio)
    num_frames = int(np.ceil(total / frame_size))

    for idx in range(num_frames):
        start = idx * frame_size
        end = start + frame_size
        frame = audio[start:end]
        if len(frame) < frame_size:
            frame = np.pad(frame, (0, frame_size - len(frame)), mode="constant")
        yield frame

# =========================================================
# Embedded-like frame feature functions
# =========================================================

def remove_dc(frame):
    """Mirror embedded C mean-removal per frame."""
    return frame - np.mean(frame)


def frame_energy(frame):
    # Convert normalized float back to int16-equivalent scale
    frame_int = (frame * 32768.0).astype(np.float32)
    frame_int = frame_int - np.mean(frame_int)
    return np.mean(frame_int ** 2)  # now same scale as C's energy_acc/FRAME_SIZE


def frame_zcr(frame, deadzone=DEADZONE):
    """Deadzone-based zero crossing rate as in AVR code."""
    frame = (frame * 32768.0).astype(np.float32)
    frame = remove_dc(frame)
    signs = np.zeros(len(frame), dtype=np.int8)
    signs[frame > deadzone] = 1
    signs[frame < -deadzone] = -1

    crossings = 0
    prev_sign = 0
    for sign in signs:
        if sign == 0:
            continue
        if prev_sign != 0 and sign != prev_sign:
            crossings += 1
        prev_sign = sign

    return crossings / float(len(frame) - 1)

# =========================================================
# VAD and segmentation logic
# =========================================================

def estimate_noise_floor(silent_energies):
    """Estimate noise floor from early silence frames before speech."""
    if len(silent_energies) == 0:
        return 0.0
    return float(np.mean(silent_energies))


def choose_vad_threshold(noise_floor):
    """Choose a threshold similar to embedded VAD: a multiple of the noise floor."""
    return max(noise_floor * 5.0, 1e-5)


def collect_speech_frames(frames, required_frames=NUM_FRAMES, preroll=PREROLL_FRAMES):
    """Implement preroll + VAD collection until we gather 50 frames."""
    preroll_buf = []
    speech_frames = []
    silent_energies = []
    vad_triggered = False
    threshold = None

    for frame in frames:
        energy = frame_energy(frame)
        if not vad_triggered:
            silent_energies.append(energy)
            preroll_buf.append(frame)
            if len(preroll_buf) > preroll:
                preroll_buf.pop(0)

            if threshold is None and len(silent_energies) >= preroll:
                threshold = choose_vad_threshold(estimate_noise_floor(silent_energies[:preroll]))

            if threshold is not None and energy >= threshold:
                vad_triggered = True
                speech_frames.extend(preroll_buf)
                preroll_buf = []
        else:
            speech_frames.append(frame)

        if len(speech_frames) >= required_frames:
            break

    if not vad_triggered:
        # No speech trigger: use the first available frames and pad if needed.
        speech_frames = list(frames)[:required_frames]

    while len(speech_frames) < required_frames:
        speech_frames.append(np.zeros(FRAME_SIZE, dtype=np.float32))

    return speech_frames, silent_energies, threshold

# =========================================================
# Feature vector generation
# =========================================================

def segment_features(frames, noise_floor):
    """Compute 5 segments of average energy and zcr, then remove noise floor."""
    segment_energies = []
    segment_zcrs = []

    for seg in range(NUM_SEGMENTS):
        start = seg * FRAMES_PER_SEGMENT
        end = start + FRAMES_PER_SEGMENT
        segment = frames[start:end]
        energies = [frame_energy(frame) for frame in segment]
        zcrs = [frame_zcr(frame) for frame in segment]

        avg_energy = float(np.mean(energies))
        avg_zcr = float(np.mean(zcrs))

        segment_energies.append(max(avg_energy - noise_floor, 0.0))
        segment_zcrs.append(avg_zcr)

    return np.array(segment_energies, dtype=np.float32), np.array(segment_zcrs, dtype=np.float32)


def normalize_energies(energies):
    """Normalize energy vector by its maximum, matching the template format."""
    max_energy = np.max(energies)
    if max_energy <= 0:
        return np.zeros_like(energies)
    return energies / (max_energy + EPSILON)


def extract_feature_vector(audio):
    """Process one recording and produce a 10D feature vector."""
    frames = list(frame_generator(audio, FRAME_SIZE))
    speech_frames, silent_energies, threshold = collect_speech_frames(frames)

    noise_floor = estimate_noise_floor(silent_energies[:PREROLL_FRAMES])
    energies, zcrs = segment_features(speech_frames, noise_floor)
    energies = normalize_energies(energies)

    vector = np.concatenate([energies, zcrs], axis=0)
    return vector, threshold, noise_floor

# =========================================================
# Directory traversal and output
# =========================================================

def extract_templates_from_dataset(data_dir=DATA_DIR):
    """Walk through speech_data folders and generate a template per word."""
    templates = {}
    for word in sorted(os.listdir(data_dir)):
        word_dir = os.path.join(data_dir, word)
        if not os.path.isdir(word_dir):
            continue

        vectors = []
        print(f"\n=== WORD: {word} ===")

        for filename in sorted(os.listdir(word_dir)):
            if not filename.lower().endswith(".wav"):
                continue

            filepath = os.path.join(word_dir, filename)
            audio = load_wav_mono(filepath)
            vector, threshold, noise_floor = extract_feature_vector(audio)
            vectors.append(vector)

            vec_str = ", ".join(f"{x:.4f}" for x in vector)
            print(f"{filename}: [{vec_str}]  // threshold={threshold:.6f} noise={noise_floor:.6f}")

        if not vectors:
            print(f"Warning: no WAV files found for word '{word}'")
            continue

        template = np.mean(np.vstack(vectors), axis=0)
        templates[word] = template

        template_str = ", ".join(f"{x:.4f}" for x in template)
        print(f"TEMPLATE {word}: {{{template_str}}}, // {word}")

    return templates

# =========================================================
# Main runner
# =========================================================

def main():
    if not os.path.isdir(DATA_DIR):
        raise FileNotFoundError(f"Speech data directory not found: {DATA_DIR}")

    templates = extract_templates_from_dataset(DATA_DIR)

    print("\n=== FINAL AVERAGED TEMPLATES ===")
    for word, template in templates.items():
        template_str = ", ".join(f"{x:.4f}" for x in template)
        print(f"{{{template_str}}}, // {word}")

    output_path = "templates_generated.h"
    with open(output_path, "w") as out:
        out.write("// Generated templates for AVR project\n")
        out.write("// Format: {E1, E2, E3, E4, E5, Z1, Z2, Z3, Z4, Z5}, // WORD\n\n")
        for word, template in templates.items():
            template_str = ", ".join(f"{x:.4f}" for x in template)
            out.write(f"{{{template_str}}}, // {word}\n")

    print(f"\nSaved averaged templates to {output_path}")


if __name__ == "__main__":
    main()
