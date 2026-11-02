"""Record the C64U's audio stream (UDP: a 2-byte sequence number, then 192 stereo frames of signed
16-bit little-endian, about 48 kHz) to a WAV, and check it: lost packets, clicks (sample steps far
outside the recording's own high-pass level), and the left/right offset (the Haas delay, 10 ms).

Start the stream first (c64u `start_stream audio <this PC>:11001`), then:

    python tools/c64u_audio.py [--seconds 20] [--port 11001] [--out target/rotor/c64u.wav]
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=20)
    ap.add_argument('--port', type=int, default=11001)
    ap.add_argument('--out', default='target/rotor/c64u.wav')
    args = ap.parse_args()
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


if __name__ == '__main__':
    main()
