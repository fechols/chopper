"""Make the rotor sound: Military_Helicopter_Start_Idle_Away.wav -> 16 kHz mono DLT5 with a crossfaded loop.

    python tools/make-rotor.py

Needs FFmpeg on PATH and numpy. Steps:
1. FFmpeg: (L+R)/2 and resample to 16 kHz (float).
2. Loop points: B near 1:45 and A near 1:20, chosen so the 0.4 s before B and the 0.4 s before A
   correlate best (in step with the rotor's beat).
3. Crossfade the 0.4 s before B into the 0.4 s before A, so sample B is sample A: the sampler's
   hardware jump B -> A (`format::AUDIO_*`) is seamless. Linear when the two match well (correlated
   sound adds in amplitude), equal-power otherwise.
4. Cut at B, encode with tools/encode-digital-music (`--profile mono16k3`), write rotor.dlt and
   rotor.json, and a preview of what the C64U plays: the start-up then three loops, the right channel
   10 ms late (the Haas trick).
The encoded files are checked in, so builds don't need FFmpeg.
"""
import json, subprocess, sys, wave
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / 'Assets/Sound_Effects/Military_Helicopter_Start_Idle_Away.wav'
OUT = ROOT / 'Assets/Sound_Effects'
TMP = ROOT / 'target/rotor'
RATE = 16000
A0, B0 = 80.0, 105.0        # 1:20, 1:45
SEARCH = 0.5                # seconds either way
FADE = int(0.4 * RATE)
HAAS = 160                  # 10 ms
BOOKS = 5000                # about one per 21 ms, as sueme64's soundtrack


def load():
    raw = subprocess.run(['ffmpeg', '-v', 'error', '-i', str(SRC), '-af', 'pan=mono|c0=0.5*c0+0.5*c1',
                          '-ar', str(RATE), '-f', 'f32le', '-'], capture_output=True, check=True).stdout
    return np.frombuffer(raw, dtype='<f4').astype(np.float64)


def ncc(win, region):
    """Normalised correlation of `win` at every offset in `region`."""
    n = len(win)
    w = win - win.mean()
    c = np.correlate(region, w, 'valid')
    s = np.convolve(region, np.ones(n), 'valid')
    s2 = np.convolve(region ** 2, np.ones(n), 'valid')
    var = np.maximum(s2 - s * s / n, 1e-12)
    return c / np.sqrt(var * (w @ w))


def loop_points(x):
    """(a, b, rho): the window ending at b best matches the one ending at a."""
    best = (-2.0, 0, 0)
    lo = int((A0 - SEARCH) * RATE) - FADE
    region = x[lo:int((A0 + SEARCH) * RATE)]
    for b in range(int((B0 - SEARCH) * RATE), int((B0 + SEARCH) * RATE), RATE // 100):
        r = ncc(x[b - FADE:b], region)
        k = int(np.argmax(r))
        if r[k] > best[0]:
            best = (float(r[k]), lo + k + FADE, b)
    rho, a, b = best
    # refine b to the sample around the coarse pick
    for bb in range(b - RATE // 200, b + RATE // 200):
        r = ncc(x[bb - FADE:bb], region)
        k = int(np.argmax(r))
        if r[k] > rho:
            rho, a, b = float(r[k]), lo + k + FADE, bb
    return a, b, rho


def write_wav(path, x, channels=1):
    pcm = np.clip(np.round(x * 32767), -32768, 32767).astype('<i2')
    with wave.open(str(path), 'wb') as w:
        w.setnchannels(channels)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(pcm.tobytes())


def read_wav(path):
    with wave.open(str(path)) as w:
        n, ch, width = w.getnframes(), w.getnchannels(), w.getsampwidth()
        data = w.readframes(n)
    a = np.frombuffer(data, dtype='<i2' if width == 2 else 'u1').astype(np.float64)
    a = a / 32768 if width == 2 else (a - 128) / 128
    return a.reshape(-1, ch)[:, 0]


def main():
    TMP.mkdir(parents=True, exist_ok=True)
    x = load()
    a, b, rho = loop_points(x)
    t = (np.arange(FADE) + 0.5) / FADE
    if rho >= 0.5:
        out_w, in_w, law = 1 - t, t, 'linear'
    else:
        out_w, in_w, law = np.cos(t * np.pi / 2), np.sin(t * np.pi / 2), 'equal-power'
    y = x[:b].copy()
    y[b - FADE:b] = x[b - FADE:b] * out_w + x[a - FADE:a] * in_w
    print(f'loop {a / RATE:.4f} s -> {b / RATE:.4f} s ({(b - a) / RATE:.3f} s), match {rho:.3f}, {law} fade {FADE / RATE:.1f} s')
    src = TMP / 'rotor-source.wav'
    write_wav(src, y)
    dlt, decoded = OUT / 'rotor.dlt', TMP / 'rotor-dlt5.wav'
    exe = ROOT / 'tools/encode-digital-music/target/release/encode-digital-music'
    if not (exe.with_suffix('.exe').exists() or exe.exists()):
        subprocess.run(['cargo', 'build', '--release', '--manifest-path', str(ROOT / 'tools/encode-digital-music/Cargo.toml')], check=True)
    subprocess.run([str(exe), '--profile', 'mono16k3', '--codebooks', str(BOOKS), str(src), str(dlt), str(decoded)], check=True)
    frames = int.from_bytes(dlt.read_bytes()[8:12], 'little')
    assert frames == b, f'the encoder made {frames} frames of {b}'
    meta = {'source': SRC.name, 'rate': RATE, 'frames': b, 'loop_start': a, 'loop_end': b,
            'fade_samples': FADE, 'fade_law': law, 'match': round(rho, 4), 'haas_samples': HAAS, 'codebooks': BOOKS}
    (OUT / 'rotor.json').write_text(json.dumps(meta, indent=2) + '\n')
    # what the sampler plays: 0..b, then a..b three more times; right voice 160 samples late
    d = read_wav(decoded)
    left = np.concatenate([d[:b]] + [d[a:b]] * 3)
    right = np.concatenate([np.zeros(HAAS), left[:-HAAS]])
    write_wav(TMP / 'rotor-preview.wav', np.stack([left, right], 1).reshape(-1), channels=2)
    print(f'wrote {dlt} ({dlt.stat().st_size} bytes), rotor.json, preview {TMP / "rotor-preview.wav"}')


if __name__ == '__main__':
    sys.exit(main())
