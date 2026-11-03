# Effect sprites

Explosions, hits, splashes, smoke, muzzle flashes and burning wrecks for the 3D view, made with
[Ludo.ai](https://ludo.ai) (`ludo` CLI 0.2.1, 2026-11-02) and baked into the REU as pre-rendered
sprites at 31 sizes each (`src/effects.rs`, `src/effects_bake.rs`).

## Pipeline

1. `ludo image create`: four stills per effect, `sprite-vfx`, Cel-Shaded, side view (`*_still-v1..4.png`).
2. `ludo sprite animate`: the chosen still animated with Hydra, 3 s, up to 16 frames, full resolution
   (`*_anim-1.png` the sheet, `-2.mp4` the video, `-3.mp3` Hydra's sound; `anim.json` the job's result).
3. `python tools/effects.py`: picks the frames, crops them to one box, and downscales every size from
   that high-res master (area average, premultiplied alpha), squashed to the projection's pixel aspect
   (80 multicolour pixels : 133 lines), alpha-thresholded at 50% and mapped by nearest reference colour
   onto the view's codes: 1 light blue (flash, foam, highlights), 2 brown (fire body, dirt, dust),
   3 blue (smoke, debris, outlines). Code 0 (green, the grass) is never used. Writes `baked/<name>.bin`.
4. `cargo run --release -- bake` encodes them (`src/effects_bake.rs`); `chopper effects` writes contact
   sheets drawn back out of the image (`output/effects/`).

## What was used

| Effect | Still | Frames of the sheet | Loops |
|---|---|---|---|
| explosion | v1 | 1 3 4 6 8 10 12 14 | no |
| impact | v2 (Ludo's ground strip cut away) | 0 1 3 5 7 9 | no |
| splash | v1 | 1 3 4 6 9 10 | no |
| smoke | v1 | 0 2 4 6 8 10 14 15 | yes |
| muzzle | v3 | 14 0 2 3 | no |
| wreck_fire | v1 | 0 1 3 5 7 9 13 15 | yes |

## Commands

Stills (each `--n 4`, 0.5 credits per image):

```sh
ludo image create --image-type sprite-vfx --art-style "Cel-Shaded" --perspective "Side-Scroll" \
  --aspect-ratio ar_1_1 --n 4 --out Assets/Effects/ludo/explosion --name explosion_still --prompt \
  "A military vehicle explosion seen from the side: a big bold fireball burst on the ground with flying dark debris chunks and dust, billowing into a thick rolling smoke cloud. Bold simple silhouette, high contrast, chunky shapes, few large shapes rather than fine detail, isolated on a plain white background"
```

| Effect | Aspect | Prompt |
|---|---|---|
| impact | ar_1_1 | A bullet impact hitting the dirt ground seen from the side: a small upward splash of dirt clods, shrapnel sparks and dust puff in a fan shape. Bold simple silhouette, high contrast, chunky shapes, isolated on a plain white background |
| splash | ar_1_1 | A bullet hitting water seen from the side: a tall vertical spray column of white and light blue water with droplets flying outward and a crown splash at the base. Bold simple silhouette, high contrast, chunky shapes, isolated on a plain dark background |
| smoke | ar_3_4 | A thick column of dark grey-blue smoke rising from a burning wreck on the ground, seen from the side: billowing puffs that widen as they rise, darker at the base. Bold simple silhouette, high contrast, chunky cloud shapes, isolated on a plain white background |
| muzzle | ar_1_1 | A bright gun muzzle flash spark: a sharp star-shaped burst of white-hot light with short radiating spikes, like a tracer round striking metal. Bold simple silhouette, high contrast, chunky shapes, isolated on a plain black background |
| wreck_fire | ar_3_4 | A burning wreck fire seen from the side: tall flickering flames with dark smoke billowing above them, rising from a pile of charred debris on the ground. Bold simple silhouette, high contrast, chunky flame and smoke shapes, isolated on a plain white background |

Animations (Hydra, 3 credits/s × 3 s each):

```sh
ludo sprite animate --model hydra --duration 3 --frames 16 --image-type sprite-vfx --frame-size 0 \
  --no-loop --initial-image Assets/Effects/ludo/explosion/explosion_still-v1.png \
  --out Assets/Effects/ludo/explosion --name explosion_anim --motion-prompt \
  "The vehicle explodes: a bright fireball bursts outward with debris flying out, then it turns into a rolling smoke cloud that rises and thins away. The camera does not move."
```

| Effect | Loop | Motion prompt |
|---|---|---|
| impact | `--no-loop` | A bullet hits the ground: dirt clods and debris burst upward and outward, fall back down, and the dust puff fades away. The camera does not move. |
| splash | `--no-loop` | A bullet hits the water: the spray column shoots up, then collapses and falls back as droplets. The camera does not move. |
| smoke | `--loop` | The smoke column billows and rises continuously in a seamless loop. The camera does not move. |
| muzzle | `--no-loop` | A gun flash: the spark flashes, flickers quickly and vanishes. The camera does not move. |
| wreck_fire | `--loop` | The fire burns: flames flicker and dance while dark smoke billows upward in a seamless loop. The camera does not move. |

## Credits

Stills 6 × 2 = 12, animations 6 × 9 = 54: 66 credits held; the account went from about 1150 to
1088.21 (Hydra charges the seconds actually produced).
