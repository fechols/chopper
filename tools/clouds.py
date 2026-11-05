#!/usr/bin/env python3
"""Horizon thunderheads: Ludo.ai stills -> C64 multicolour code maps at every roll angle.

For each cloud shape (`SHAPES` below, `clouds::SHAPES` of them, in order), the Ludo still
(`Assets/Clouds/ludo/<file>`, a photorealistic cloud; Ludo returns it cut out, else it is keyed off the
chroma-key green it was asked for: alpha from how much greener than red and blue a pixel is, the green
spill taken out) is cropped to the cloud (or to the
box given), and for each roll angle (`clouds::ANGLES` steps of `clouds::STEP_DEG` degrees, -30..+30,
index `a` turning it `(a - LEVEL) * STEP_DEG` degrees clockwise on the screen) rotated at full
resolution, then downscaled by area averaging (premultiplied alpha) to the shape's height in lines,
squashed to the projection's pixel aspect (`geom`: 80 multicolour pixels across for 133 lines up), and
thresholded on alpha at 50%. Inside, the brightness picks the density of blue (code 3) dots in a 4x4
ordered dither on the frame's own grid: solid at the cloud's darkest (its base) down to `MIN_DOTS` of
16 at its brightest (the sunlit tops). Code 1 (the sky's light blue) is never drawn: the light between
the dots is the sky itself, transparent.

Writes `Assets/Clouds/baked/clouds.bin`, read by `src/clouds.rs` (`include_bytes!`):

    b"CLD1", shapes u8, angles u8,
    per shape, per angle: w u8, h u8, ax u8, ay u8 (the anchor: the base's centre), then the w*h pixels
    row-major, run-length coded as `tools/effects.py`'s: one byte per run, (value << 5) | (run - 1),
    value 0-3 a colour code, 4 transparent, run 1-32.

    python tools/clouds.py                 # bake
    python tools/clouds.py --preview DIR   # also PNG previews: the keyed masters and every 5th angle
"""

import math
import os
import sys

import numpy as np
from PIL import Image, ImageFilter

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LUDO = os.path.join(ROOT, "Assets", "Clouds", "ludo")
BAKED = os.path.join(ROOT, "Assets", "Clouds", "baked")

# `clouds::ANGLES`, `clouds::STEP_DEG`.
ANGLES, STEP_DEG = 31, 2
LEVEL = ANGLES // 2
# Multicolour pixels per line for a square on the screen (`geom`: sx = 80 X/Z, sy = 133 Y/Z).
ASPECT = 80 / 133
# The faintest dots (of 16) inside the cloud: its sunlit tops.
MIN_DOTS = 3
# Parts thinner than this (of the cloud's height) are cut away (`key`).
THIN = 0.04
# `clouds::REACH_*`: no frame may reach further from its anchor (multicolour pixels across, lines up
# and down).
REACH_X, REACH_UP, REACH_DOWN = 40, 64, 24

# The shapes, in `clouds` order: (Ludo still, crop box (left, top, right, bottom) in its pixels or None
# for the whole keyed cloud, height in lines when level).
SHAPES = [
    ("anvil_still-v1.png", None, 28),
    ("cluster_still-v4.png", None, 30),
    ("anvil_still-v3.png", None, 34),
    ("cluster_still-v2.png", None, 38),
]

BAYER = np.array([[0, 8, 2, 10], [12, 4, 14, 6], [3, 11, 1, 9], [15, 7, 13, 5]])


def key(path, box):
    """The still keyed: (alpha, luminance) float arrays in 0..1, cropped to the cloud."""
    src = Image.open(path)
    img = np.asarray(src.convert("RGBA"), dtype=np.float64) / 255.0
    if box:
        l, t, r, b = box
        img = img[t:b, l:r]
    r, g, b = img[..., 0], img[..., 1], img[..., 2]
    if src.mode == "RGBA" and img[..., 3].min() < 0.5:
        # Ludo cut it out already.
        alpha = img[..., 3]
    else:
        green = g - np.maximum(r, b)
        alpha = np.clip(1.0 - (green - 0.12) / 0.3, 0.0, 1.0)
        # The green spill on the cloud's edges: green no brighter than the brighter of red and blue.
        g = np.minimum(g, np.maximum(r, b))
    lum = 0.3 * r + 0.59 * g + 0.11 * b
    # An opening (erode, then dilate) takes out what is thinner than about `THIN` of the cloud's height:
    # Ludo's flat shadow skirt under it and the anvil's frayed edges, which shrink to long one-pixel
    # wedges and spikes.
    ys, xs = np.nonzero(alpha > 0.5)
    k = max(3, int(round((ys.max() - ys.min()) * THIN)) | 1)
    a8 = Image.fromarray((alpha * 255).astype(np.uint8), "L")
    alpha = np.minimum(alpha, np.asarray(a8.filter(ImageFilter.MinFilter(k)).filter(ImageFilter.MaxFilter(k)), dtype=np.float64) / 255.0)
    ys, xs = np.nonzero(alpha > 0.5)
    if len(xs) == 0:
        raise SystemExit(f"{path}: nothing left after keying")
    pad = 4
    t, b2 = max(ys.min() - pad, 0), min(ys.max() + pad + 1, alpha.shape[0])
    l, r2 = max(xs.min() - pad, 0), min(xs.max() + pad + 1, alpha.shape[1])
    return alpha[t:b2, l:r2], lum[t:b2, l:r2]


def frame(alpha, lum, lines, angle, lo, hi):
    """One frame: the code map (None transparent, else 3), and its anchor (ax, ay)."""
    h0, w0 = alpha.shape
    th = math.radians((angle - LEVEL) * STEP_DEG)
    # PIL turns counter-clockwise by its angle: ours is clockwise on the screen.
    rot = lambda a: np.asarray(Image.fromarray(a.astype(np.float32), "F").rotate(-math.degrees(th), resample=Image.BICUBIC, expand=True))
    ra, rp = rot(alpha), rot(alpha * lum)
    h1, w1 = ra.shape
    # The base's centre, level at (w0/2, the bottom row's bottom), through the turn about the centre.
    dx, dy = w0 / 2.0 - w0 / 2.0, h0 - h0 / 2.0
    c, s = math.cos(th), math.sin(th)
    bx, by = w1 / 2.0 + c * dx - s * dy, h1 / 2.0 + s * dx + c * dy
    # Down to `lines` lines for the level height h0, pixels ASPECT as many across.
    k = lines / h0
    ow, oh = max(1, round(w1 * k * ASPECT)), max(1, round(h1 * k))
    box = lambda a: np.asarray(Image.fromarray(a, "F").resize((ow, oh), resample=Image.BOX))
    sa, sp = box(ra), box(rp)
    ax, ay = bx * ow / w1, by * oh / h1
    opaque = sa >= 0.5
    l = np.where(opaque, sp / np.maximum(sa, 1e-6), 1.0)
    t = np.clip((l - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    dots = np.round(16 - (16 - MIN_DOTS) * t)
    ys, xs = np.nonzero(opaque)
    x0, x1, y0, y1 = xs.min(), xs.max() + 1, ys.min(), ys.max() + 1
    codes = []
    for y in range(y0, y1):
        for x in range(x0, x1):
            on = opaque[y, x] and BAYER[(y - y0) & 3, (x - x0) & 3] < dots[y, x]
            codes.append(3 if on else None)
    w, h = x1 - x0, y1 - y0
    axi, ayi = round(ax - x0), round(ay - y0)
    if not (axi <= REACH_X and w - axi <= REACH_X and ayi <= REACH_UP and h - ayi <= REACH_DOWN):
        raise SystemExit(f"a frame past the reach: {w}x{h}, anchor ({axi}, {ayi})")
    if not (0 <= axi < 256 and 0 <= ayi < 256 and w < 256 and h < 256):
        raise SystemExit(f"a frame too big: {w}x{h}, anchor ({axi}, {ayi})")
    return codes, w, h, axi, ayi


def runs(codes):
    out = bytearray()
    i = 0
    while i < len(codes):
        v = 4 if codes[i] is None else codes[i]
        n = 1
        while i + n < len(codes) and n < 32 and (4 if codes[i + n] is None else codes[i + n]) == v:
            n += 1
        out.append(v << 5 | (n - 1))
        i += n
    return out


def main():
    preview = None
    if "--preview" in sys.argv:
        preview = sys.argv[sys.argv.index("--preview") + 1]
        os.makedirs(preview, exist_ok=True)
    out = bytearray(b"CLD1") + bytes([len(SHAPES), ANGLES])
    for si, (name, crop, lines) in enumerate(SHAPES):
        alpha, lum = key(os.path.join(LUDO, name), crop)
        inside = lum[alpha > 0.5]
        # The brightness range: the darkest few percent solid, the brightest few the faintest dots.
        lo, hi = np.percentile(inside, 4), np.percentile(inside, 96)
        if preview:
            m = np.dstack([lum * 255] * 3 + [alpha * 255]).astype(np.uint8)
            Image.fromarray(m, "RGBA").save(os.path.join(preview, f"master{si}.png"))
        sheet = []
        for a in range(ANGLES):
            codes, w, h, ax, ay = frame(alpha, lum, lines, a, lo, hi)
            out += bytes([w, h, ax, ay]) + runs(codes)
            if a % 5 == 0:
                sheet.append((codes, w, h))
        print(f"shape {si}: {name}, {lines} lines level; frame sizes {sheet[0][1]}x{sheet[0][2]} .. {sheet[-1][1]}x{sheet[-1][2]}")
        if preview:
            sw, sh = sum(w for _, w, _ in sheet) + 4 * len(sheet), max(h for _, _, h in sheet)
            img = Image.new("RGB", (2 * sw, sh), (108, 94, 181))
            px, x0 = img.load(), 0
            for codes, w, h in sheet:
                for y in range(h):
                    for x in range(w):
                        if codes[y * w + x] is not None:
                            px[x0 + 2 * x, y] = px[x0 + 2 * x + 1, y] = (53, 40, 121)
                x0 += 2 * w + 8
            img.resize((img.width * 3, img.height * 3), Image.NEAREST).save(os.path.join(preview, f"shape{si}.png"))
    os.makedirs(BAKED, exist_ok=True)
    path = os.path.join(BAKED, "clouds.bin")
    with open(path, "wb") as f:
        f.write(out)
    print(f"{path}: {len(out)} bytes")


if __name__ == "__main__":
    main()
