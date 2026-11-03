#!/usr/bin/env python3
"""Effect sprites: Ludo.ai animations -> C64 multicolour code maps at every scale.

For each effect (`EFFECTS` below, the same order as `src/effects.rs`), the Ludo spritesheet
(`Assets/Effects/ludo/<name>/<name>_anim-1.png`, its grid from `anim.json`) is sliced, the chosen
frames are cropped to one box that holds all of them (so the animation stays registered), and every
scale (4, 6, .. 64 lines tall) is downscaled straight from that high-res master by area averaging
(premultiplied alpha), squashed to the projection's pixel aspect (`geom`: 80 multicolour pixels across
for 133 lines up per unit of X/Z, Y/Z), thresholded on alpha and mapped onto the view's four global
colour codes (`format::COLORS`: 0 green = the grass, 1 light blue, 2 brown, 3 blue) by nearest
reference colour, per effect. Green (code 0) is never used: it is the ground.

Writes `Assets/Effects/baked/<name>.bin`, read by `src/effects_bake.rs` (`include_bytes!`):

    b"EFX1", frames u8, scales u8,
    per scale: w u8, h u8, ax u8, ay u8, then per frame the w*h pixels row-major, run-length coded:
    one byte per run, (value << 5) | (run - 1), value 0-3 a colour code, 4 transparent, run 1-32.

    python tools/effects.py            # bake every effect
    python tools/effects.py --preview DIR  # also quick PNG previews of the quantised frames
"""

import json
import os
import sys

from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LUDO = os.path.join(ROOT, "Assets", "Effects", "ludo")
BAKED = os.path.join(ROOT, "Assets", "Effects", "baked")

# Scales: 4, 6, .., 64 lines (`effects::SCALES`, `effects::lines`).
MIN_LINES, STEP_LINES, SCALES = 4, 2, 31
# Multicolour pixels per line for a square in the world: `geom`'s projection, sx = 80 X/Z, sy = 133 Y/Z.
ASPECT = 80 / 133
MAX_FRAMES = 8

# Reference colours (source RGB) -> code, per effect: each pixel takes the code of its nearest reference.
FIRE = [
    ((255, 255, 255), 1), ((255, 240, 150), 1), ((255, 215, 60), 1),  # white-hot, yellow: the flash
    ((245, 150, 40), 2), ((220, 90, 20), 2), ((150, 40, 15), 2),  # orange, red: the fire's body
    ((15, 15, 15), 3), ((60, 55, 55), 3), ((95, 95, 100), 3),  # black, dark smoke, debris
    ((175, 175, 180), 1),  # light smoke, dust highlights
]
DIRT = [
    ((255, 255, 255), 1), ((255, 230, 150), 1),  # sparks, the flash
    ((235, 215, 190), 2), ((200, 170, 140), 2), ((150, 110, 75), 2),  # dust, dirt
    ((90, 65, 45), 3), ((40, 30, 25), 3), ((110, 110, 110), 3),  # clods, debris, shadow
]
WATER = [
    ((255, 255, 255), 1), ((215, 230, 240), 1), ((170, 200, 225), 1),  # foam, spray
    ((110, 150, 190), 3), ((60, 90, 130), 3), ((10, 10, 20), 3),  # shaded water, outline
]
SMOKE = [
    ((150, 165, 190), 1), ((125, 140, 165), 1),  # sunlit tops
    ((90, 100, 120), 3), ((55, 60, 75), 3), ((15, 15, 20), 3),  # body, outline
    ((255, 200, 60), 1), ((240, 120, 30), 2), ((140, 40, 10), 2),  # the fire at the foot
]
# A fire's flames: yellow and orange light, red the edges.
FLAMES = [
    ((255, 255, 255), 1), ((255, 240, 150), 1), ((255, 215, 60), 1), ((245, 150, 40), 1),
    ((220, 70, 20), 2), ((150, 30, 15), 2),
    ((15, 15, 15), 3), ((60, 55, 55), 3), ((95, 95, 100), 3), ((140, 140, 145), 3),
]
FLASH = [
    ((255, 255, 255), 1), ((255, 240, 160), 1), ((255, 210, 70), 1),  # core, yellow
    ((245, 150, 40), 2), ((220, 100, 20), 2),  # orange rim
    ((20, 10, 5), 3), ((90, 40, 10), 3),  # outline
]

# name, Ludo frames used (sheet order), reference colours, anchor ("base": bottom centre, "centre"),
# master crop: fraction of the cell cut off at the bottom (Ludo's ground strip), and the share of the
# alpha mass the box keeps at either side (stray debris far out is cut).
EFFECTS = [
    dict(name="explosion", frames=[1, 3, 4, 6, 8, 10, 12, 14], ref=FIRE, anchor="base", cut_bottom=0.0, keep=0.995),
    dict(name="impact", frames=[0, 1, 3, 5, 7, 9], ref=DIRT, anchor="base", cut_bottom=0.0, keep=0.99, ground=True),
    dict(name="splash", frames=[1, 3, 4, 6, 9, 10], ref=WATER, anchor="base", cut_bottom=0.0, keep=0.995),
    dict(name="smoke", frames=[0, 2, 4, 6, 8, 10, 14, 15], ref=SMOKE, anchor="base", cut_bottom=0.0, keep=0.998),
    dict(name="muzzle", frames=[14, 0, 2, 3], ref=FLASH, anchor="centre", cut_bottom=0.0, keep=1.0),
    dict(name="wreck_fire", frames=[0, 1, 3, 5, 7, 9, 13, 15], ref=FLAMES, anchor="base", cut_bottom=0.0, keep=0.998),
]

ALPHA = 128  # opaque from half coverage


def frames_of(e):
    d = os.path.join(LUDO, e["name"])
    meta = json.load(open(os.path.join(d, "anim.json")))["result"]
    sheet = Image.open(os.path.join(d, f"{e['name']}_anim-1.png")).convert("RGBA")
    cols, rows = meta["num_cols"], meta["num_rows"]
    cw, ch = sheet.width // cols, sheet.height // rows
    cells = [sheet.crop((c * cw, r * ch, (c + 1) * cw, (r + 1) * ch)) for r in range(rows) for c in range(cols)]
    cells = cells[: meta["num_frames"]]
    out = [cells[i] for i in e["frames"]]
    if e.get("ground"):
        out = strip_ground(out)
    return out


def strip_ground(frames):
    """Cut Ludo's ground strip away (a plank at the foot, the same in every frame), and whatever falls
    below it: find it in the first frame (the lowest opaque row and the rows above it with the same
    ends), then crop every frame at its top."""
    f = frames[0]
    a = f.getchannel("A").load()
    w, h = f.size

    def ends(y):
        xs = [x for x in range(w) if a[x, y] >= ALPHA]
        return (xs[0], xs[-1]) if xs else None

    y = h - 1
    while ends(y) is None:
        y -= 1
    l0, r0 = ends(y)
    while y > 0:
        e = ends(y - 1)
        if e is None or abs(e[0] - l0) > 16 or abs(e[1] - r0) > 16:
            break
        y -= 1
    # And the clods lying on it.
    y -= 12
    return [fr.crop((0, 0, fr.width, y)) for fr in frames]


def box(frames, keep):
    """One crop box for all frames: rows/columns from the union's alpha mass, `keep` of it."""
    w = max(f.width for f in frames)
    h = max(f.height for f in frames)
    colmass = [0] * w
    rowmass = [0] * h
    for f in frames:
        a = f.getchannel("A")
        px = a.load()
        for y in range(f.height):
            for x in range(f.width):
                v = px[x, y]
                if v >= ALPHA:
                    colmass[x] += 1
                    rowmass[y] += 1

    def span(mass, keep):
        total = sum(mass)
        cut = total * (1 - keep) / 2
        lo, acc = 0, 0
        while acc + mass[lo] <= cut:
            acc += mass[lo]
            lo += 1
        hi, acc = len(mass) - 1, 0
        while acc + mass[hi] <= cut:
            acc += mass[hi]
            hi -= 1
        return lo, hi + 1

    x0, x1 = span(colmass, keep)
    y0, y1 = span(rowmass, 1.0)  # never cut the top or the foot
    return x0, y0, x1, y1


def nearest(rgb, ref):
    r, g, b = rgb
    best, code = None, 3
    for (rr, gg, bb), c in ref:
        d = 3 * (r - rr) ** 2 + 4 * (g - gg) ** 2 + 2 * (b - bb) ** 2
        if best is None or d < best:
            best, code = d, c
    return code


def quantise(im, ref):
    """RGBA (straight alpha) -> list of codes (None transparent)."""
    out = []
    for r, g, b, a in im.get_flattened_data():
        out.append(nearest((r, g, b), ref) if a >= ALPHA else None)
    return out


def downscale(master, w, h):
    # Area average with premultiplied alpha, so transparent pixels' colour does not bleed in.
    pm = master.convert("RGBa").resize((w, h), Image.BOX)
    return pm.convert("RGBA")


def rle(codes):
    out = bytearray()
    i = 0
    while i < len(codes):
        v = 4 if codes[i] is None else codes[i]
        n = 1
        while i + n < len(codes) and n < 32 and (4 if codes[i + n] is None else codes[i + n]) == v:
            n += 1
        out.append(v << 5 | (n - 1))
        i += n
    return bytes(out)


def bake(e, preview=None):
    frames = frames_of(e)
    assert len(frames) <= MAX_FRAMES
    x0, y0, x1, y1 = box(frames, e["keep"])
    masters = [f.crop((x0, y0, x1, y1)) for f in frames]
    mw, mh = x1 - x0, y1 - y0
    out = bytearray(b"EFX1")
    out += bytes([len(frames), SCALES])
    sheets = []
    for s in range(SCALES):
        h = MIN_LINES + STEP_LINES * s
        w = max(1, round(h * mw / mh * ASPECT))
        assert w <= 255
        ax = w // 2
        ay = h - 1 if e["anchor"] == "base" else h // 2
        out += bytes([w, h, ax, ay])
        scale = []
        for m in masters:
            codes = quantise(downscale(m, w, h), e["ref"])
            out += rle(codes)
            scale.append(codes)
        sheets.append((w, h, scale))
    os.makedirs(BAKED, exist_ok=True)
    path = os.path.join(BAKED, e["name"] + ".bin")
    with open(path, "wb") as f:
        f.write(out)
    print(f"{e['name']}: {len(frames)} frames, master {mw}x{mh}, 64 lines = {sheets[-1][0]} px wide, {len(out)} bytes")
    if preview:
        save_preview(e["name"], sheets, preview)


# The view's colours (`palette::PALETTE[format::COLORS]`).
RGB = [(76, 180, 66), (109, 106, 239), (91, 56, 0), (44, 41, 177)]


def save_preview(name, sheets, dirname):
    """Every 3rd scale's frames, pixels doubled across (true aspect), on the grass, zoomed 2x."""
    rows = [sheets[s] for s in list(range(0, SCALES, 3)) + [SCALES - 1]]
    n = len(rows[0][2])
    cw = max(2 * w for w, _, _ in rows) + 4
    W = n * cw + 4
    H = sum(h + 4 for _, h, _ in rows) + 4
    im = Image.new("RGB", (W, H), (40, 40, 40))
    px = im.load()
    y = 4
    for w, h, frames in rows:
        for k, codes in enumerate(frames):
            x = 4 + k * cw
            for yy in range(h):
                for xx in range(w):
                    c = codes[yy * w + xx]
                    col = RGB[0] if c is None else RGB[c]
                    px[x + 2 * xx, y + yy] = col
                    px[x + 2 * xx + 1, y + yy] = col
        y += h + 4
    im = im.resize((W * 2, H * 2), Image.NEAREST)
    os.makedirs(dirname, exist_ok=True)
    im.save(os.path.join(dirname, name + ".png"))


def main():
    preview = None
    if "--preview" in sys.argv:
        preview = sys.argv[sys.argv.index("--preview") + 1]
    only = [a for a in sys.argv[1:] if not a.startswith("--") and a != preview]
    for e in EFFECTS:
        if not only or e["name"] in only:
            bake(e, preview)


if __name__ == "__main__":
    main()
