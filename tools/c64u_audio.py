"""Record the C64U's audio stream (UDP: a 2-byte sequence number, then 192 stereo frames of signed
16-bit little-endian, about 48 kHz) to a WAV, and check it: lost packets, clicks (sample steps far
outside the recording's own high-pass level), and the left/right offset (the Haas delay, 10 ms).

Start the stream first (c64u `start_stream audio <this PC>:11001`), then:

    python tools/c64u_audio.py [--seconds 20] [--port 11001] [--out target/rotor/c64u.wav]
    python tools/c64u_audio.py --hrtf
    python tools/c64u_audio.py --wav target/rotor/c64u.wav

`--hrtf` checks the sound effects' placing (`sfx::place`) in the recording just made; `--wav` checks it in
that file instead, without recording (nothing is overwritten). In 40 ms
windows, the level difference between the ears (dB, right over left) and their time difference (the
cross-correlation's peak within 1 ms: positive, the left ear later), then whether the two agree (a
source on the right both louder and earlier on the right). Mute the rotor first (Audio Mixer: Vol
Sampler L/R OFF): its 10 ms Haas delay is not the effects' placing.
"""
import argparse
import socket
import wave
from pathlib import Path

import numpy as np

RATE = 47983  # PAL; NTSC machines stream at about 47940


def record(port, seconds):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 22)
    s.bind(('0.0.0.0', port))
    s.settimeout(3)
    chunks, seqs = [], []
    want = int(seconds * RATE / 192)
    while len(chunks) < want:
        data, _ = s.recvfrom(2048)
        seqs.append(int.from_bytes(data[:2], 'little'))
        chunks.append(np.frombuffer(data[2:], dtype='<i2').reshape(-1, 2))
    lost = sum(((b - a) & 0xFFFF) - 1 for a, b in zip(seqs, seqs[1:]))
    return np.concatenate(chunks).astype(np.float64), lost


def hrtf(x, window=0.04, quiet=100.0):
    """The ears' level and time differences window by window, and how often they agree (see the module).
    (Not high-passed: the engines are low drones, most of what is sounding.)"""
    n, lag = int(window * RATE), int(0.001 * RATE)
    rows = []
    for k in range(0, len(x) - n, n):
        w = x[k:k + n] - x[k:k + n].mean(axis=0)
        left, right = w[:, 0], w[:, 1]
        rl, rr = np.sqrt((left ** 2).mean()), np.sqrt((right ** 2).mean())
        if max(rl, rr) < quiet:
            continue
        ild = 20 * np.log10(max(rr, 1e-9) / max(rl, 1e-9))
        corr = [float(left[max(0, -d):len(left) - max(0, d)] @ right[max(0, d):len(right) - max(0, -d)]) for d in range(-lag, lag + 1)]
        # (The peak at d > 0: the right ear d samples later, so the left ear's lag is -d.)
        itd = (int(np.argmax(corr)) - lag) / RATE * 1000
        rows.append((k / RATE, ild, -itd))
    if not rows:
        print('hrtf: nothing loud enough')
        return
    for t, ild, itd in rows[::max(1, len(rows) // 40)]:
        bar = '#' * int(min(abs(ild), 15))
        print(f'{t:7.2f} s  level {ild:+6.1f} dB {"R" if ild > 0 else "L"} {bar:<15}  lag {itd:+.2f} ms (left later +)')
    lateral = [(ild, itd) for _, ild, itd in rows if abs(ild) >= 1.5 and abs(itd) >= 0.06]
    agree = sum(1 for ild, itd in lateral if (ild > 0) == (itd > 0))
    print(f'hrtf: {len(rows)} windows sounding; {len(lateral)} off the centre (level 1.5 dB+, lag a line+), '
          f'{agree} ({100 * agree / max(1, len(lateral)):.0f}%) with level and lag on the same side; '
          f'largest lag {max(abs(r[2]) for r in rows):.2f} ms')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=20)
    ap.add_argument('--port', type=int, default=11001)
    ap.add_argument('--out', default='target/rotor/c64u.wav')
    ap.add_argument('--hrtf', action='store_true', help="check the effects' placing (see the module)")
    ap.add_argument('--wav', help="analyse this recording's placing (--hrtf) instead of recording")
    args = ap.parse_args()
    if args.wav:
        with wave.open(args.wav, 'rb') as w:
            hrtf(np.frombuffer(w.readframes(w.getnframes()), dtype='<i2').reshape(-1, 2).astype(np.float64))
        return
    x, lost = record(args.port, args.seconds)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(out), 'wb') as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(x.astype('<i2').tobytes())
    left, right = x[:, 0], x[:, 1]
    print(f'{len(x) / RATE:.1f} s recorded to {out}; {lost} packets lost; rms L {np.sqrt((left ** 2).mean()):.0f} R {np.sqrt((right ** 2).mean()):.0f}')
    # Left/right offset: the lag (right later) with the best correlation, up to 30 ms.
    n = min(len(x), RATE * 5)
    a, b = left[:n] - left[:n].mean(), right[:n] - right[:n].mean()
    lags = range(0, int(0.03 * RATE))
    corr = [float(a[:n - k] @ b[k:]) for k in lags]
    k = int(np.argmax(corr))
    print(f'right voice lags left by {k} frames = {k / RATE * 1000:.2f} ms')
    # Clicks: second differences far beyond the recording's own spread.
    for name, c in (('left', left), ('right', right)):
        d2 = np.abs(np.diff(c, 2))
        level = np.percentile(d2, 99.9)
        spikes = np.flatnonzero(d2 > 4 * max(level, 64))
        groups = np.split(spikes, np.flatnonzero(np.diff(spikes) > RATE // 100) + 1) if len(spikes) else []
        times = ', '.join(f'{g[0] / RATE:.2f}' for g in groups[:12])
        print(f'{name}: 99.9% second difference {level:.0f}; {len(groups)} click-like spikes{" at " + times + " s" if times else ""}')
    if args.hrtf:
        hrtf(x)


if __name__ == '__main__':
    main()
