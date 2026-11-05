# Horizon clouds

The thunderheads on the horizon (`src/clouds.rs`), made with [Ludo.ai](https://ludo.ai) (`ludo` CLI)
and baked into the REU as pre-rendered sprites, each shape at every roll angle (31 steps of 2°, ±30°).

## Pipeline

1. `ludo image create`: photorealistic thunderhead stills (`ludo/*_still-v1..4.png`, cut out by Ludo;
   the jobs are logged in `ludo/ludo-assets.json`).
2. `python tools/clouds.py`: the chosen stills (its `SHAPES`) are cut to the cloud, with an opening that
   removes what is thinner than 4% of the cloud's height (Ludo's thin shadow skirt, frayed anvil edges).
   Each one is rotated to every angle at full resolution, then area-averaged down to its height in lines at
   the projection's pixel aspect (80 multicolour pixels : 133 lines). Pixels are kept where the alpha
   is at least 50%, and their brightness sets the density of blue (code 3) dots in a 4×4 ordered dither:
   solid at the darkest (the storm's base) down to 3/16 at the brightest (the sunlit tops). Writes
   `baked/clouds.bin`. `--preview DIR` also writes the keyed masters and every 5th angle.
3. `cargo run --release -- bake` encodes them (`clouds::bake`); `chopper clouds` writes a sheet drawn back
   out of the image (`output/clouds.png`).

## What is used

| Shape | Still | Height (lines, level) |
|---|---|---|
| 0 | `anvil_still-v1` | 28 |
| 1 | `cluster_still-v4` | 30 |
| 2 | `anvil_still-v3` | 34 |
| 3 | `cluster_still-v2` | 38 |

## Commands

Stills (`--n 4`, 0.5 credits per image):

```sh
ludo image create --image-type sprite-vfx --art-style "Photorealistic 3D" --perspective "Side-Scroll" \
  --aspect-ratio ar_1_1 --n 4 --out Assets/Clouds/ludo --name anvil_still --prompt \
  "A single towering cumulonimbus thunderstorm cloud seen from far away at the horizon, side view: a dark, flat, rain-shadowed base, a massive billowing tower of cauliflower convection rising above it, and a wide flat anvil spreading out at the top. Dramatic, menacing, strong light and shadow. The whole cloud fully in frame and centered with clear space around it, isolated on a plain solid flat bright green background (chroma key), no ground, no sky, no rain, no lightning"

ludo image create --image-type sprite-vfx --art-style "Photorealistic 3D" --perspective "Side-Scroll" \
  --aspect-ratio ar_16_9 --n 4 --out Assets/Clouds/ludo --name cluster_still --prompt \
  "A cluster of distant thunderstorm clouds seen from far away at the horizon, side view: two or three towering cumulonimbus cells of different heights with dark heavy flat bases, billowing cauliflower towers and ragged tops, one beginning to spread into an anvil. Dramatic, menacing, strong light and shadow. The whole cluster fully in frame and centered with clear space around it, isolated on a plain solid flat bright green background (chroma key), no ground, no sky, no rain, no lightning"
```
