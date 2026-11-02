# Validation

## Milestone 1: the ground plane, three ways (2026-11-01, host model)

`cargo test --release`: 5 pass. `cargo check --no-default-features`: OK. `cargo run --release -- compare`
writes `output/{roto,floor}_*.png` and the 2x2 sheets (reference, dma, dma-pre, cpu4).

Times are modeled, not measured: TALON HD's measured C64U unit costs (`src/cost.rs`) times the work the
byte-exact model did, plus estimated 6502 kernel cycles (`mode7::cycles`). Full 160x200 screen. `cpu*`
excludes the cache fill.

| path | texels across | rotozoom ms | floor ms (horizon at 48) |
|---|---|---|---|
| dma: per-byte DMA, raw plane | 40 | 36.1 | 36.9 |
| dma-pre: per-byte DMA, pre-sampled planes | 160 | 36.0 | 37.3 |
| cpu1: CPU gather, 1 texel a byte | 40 | 14.0 | 9.4 |
| cpu2 | 80 | 23.2 | 15.3 |
| cpu4 | 160 | 41.6 | 27.0 |
| cache fill (128x128, per recentre) | | 6.6 | 6.6 |

(Kernel cycles counted from the bench, below.)

- Per-byte DMA: about 2.25 I/O writes a byte (REU address lo, often mid, command; the C64 address
  auto-increments in memory order) = 2.8 us, plus ~1.55 us of DDA and shadow bookkeeping.
- dma-pre gives full resolution for the same cost as dma (the REU holds 48 directions x 5 zooms).
  Visually it is close to the reference.
- Not checked on hardware: the 1-byte DMA rate and the I/O write cost inside these loops.

## Milestone 2 (offline part): bench kernels' cycle counts (2026-11-01)

`target-c64/bench/` (`bench.S` + `main.c`, llvm-mos `mos-c64-clang`, `bash target-c64/bench/build.sh` in
WSL) builds `dist/bench.prg` (1,582 bytes). `python tools/bench_cycles.py` runs it in the vendored 6502
interpreter and counts cycles per iteration. It does not model the C64U's I/O wait:

| kernel | cycles (excl. I/O stores) | I/O writes | previous estimate |
|---|---|---|---|
| per-byte DMA kernel, a byte | 88.4 | 2.13 | 97 |
| gather, 4 texels, a byte | 322.8 | 0 | 297 |
| gather, 1 texel, a byte | 106.6 | 0 | 87 |

Fit: 72 cycles a texel plus 35 a byte; `mode7::cycles` now uses these numbers. Found while building: the
4-texel loop's `bne` was out of range and llvm-mc encoded it silently (TALON HD's known trap); it now uses
`beq` + `jmp`.

## Milestone 2: C64U bench (2026-11-01, hardware)

First run (from the user's USB stick) finished but ran at 1 MHz: the C64U switches into turbo ~2 s after
`$D031 = $8F` (`$D031` read back `$8F`), and the whole bench took ~1.2 s. `main.c` now repeats a timed
probe until it runs over 10x fast; `$C03F = $EE` if it never does. Second run (MCP `run_prg`,
C64U firmware 1.1.0, FPGA 122, core 1.49, NTSC, IRQs off): 122 probes (~2.4 s), then:

| slot | per iteration | CIA ticks | per iteration (us) | model |
|---|---|---|---|---|
| 0 | empty loop (5 cycles) | 326 / 4096 | 0.080 (16 ns a cycle) | 0.080 |
| 7 | I/O write alone | 4376 / 4096 | 1.07 | 1.26 |
| 1 | 1-byte DMA, command only | 4496 / 4096 | 1.10 | 1.50 |
| 2 | 1-byte DMA, lo + command | 8563 / 4096 | 2.09 | |
| 3 | 1-byte DMA, lo + mid + command | 13053 / 4096 | 3.19 | |
| 4 | per-byte DMA kernel, a byte (2.13 writes) | 17936 / 4096 | **4.38** | 4.34 |
| 5 | CPU gather, 4 texels, a byte | 5277 / 1024 | **5.15** | 5.19 |
| 6 | CPU gather, 1 texel, a byte | 3481 / 2048 | **1.70** | 1.71 |

- A 1-byte DMA costs about one I/O write (1.10 vs 1.07 us): the transfer itself is ~0.03 us. Every
  register write costs ~1.05 us, so the per-byte DMA path is bound by its register writes.
- Here an I/O write is 1.07 us. TALON HD measured 1.26 us with its HUD raster IRQ running. The kernel
  costs land within 1-2% of the model.
- Full 160x200 screen on hardware: per-byte DMA 8000 x 4.38 = 35 ms (any resolution: 40 texels raw,
  160 pre-sampled); CPU gather 4 texels 41 ms, 1 texel 14 ms.

## Milestone 3: the perspective floor on the C64U (2026-11-01, hardware)

`dist/mode7.prg` (3,298 bytes; `target-c64/`, Rust on rust-mos + `asm/floor.S`) with `dist/mode7.reu`
(16 MiB, package version 2). The camera flies the test circuit (2,048-frame path); the floor is lines
48-199 (152 lines, 6,080 bytes); fire on joystick 2 (or `tools/c64u.py mode`) toggles line-doubled.

Host: `cargo test --release` 10 pass; `cargo check --no-default-features` OK; `python tools/emu.py
--frames 4 --doubled-after 2`: every shown frame matches `mode7 frame` byte for byte in both modes. The
6502 cost per DMA: 102 cycles / 2.55 I/O writes (full), 121 / 3.33 per line pair (doubled).

C64U (firmware 1.1.0, NTSC; `tools/c64u.py`, C64U_HOST 192.168.66.178):

| mode | floor | setup (camera + 1.8 KB line table) | frame rate |
|---|---|---|---|
| full, 160x152 | 29.4 ms | 0.57 ms | **30 fps** |
| line-doubled | 18.0 ms | 0.57 ms | **30 fps** |

`dump --hold` at tick 275: **0 mismatched bytes** against the host in both modes.

Two findings from the hardware:
- **The C64U reloads the whole REU address on any address-register write.** Writing any of
  `$DF04-$DF06` reloads all three from the values last *written* to them, so a transfer's
  auto-increment survives only until the next address write. The first build skipped the lo write
  whenever the auto-increment already matched. It then lost lo whenever mid changed: 109 of 8,000 bytes
  wrong at tick 885. A simulation of that rule reproduced the hardware in all but 2 bytes. The kernel
  now writes mid and bank when they change and lo always, last; it never relies on the auto-increment.
  `Machine` and `emu.py` model the reload (`an_address_write_reloads_the_whole_reu_address`). Doubled mode
  (fixed REU address) was never affected.
- **The first build's setup cost 4.4 ms** because Rust added the camera to 304 words through volatile
  peeks and pokes. The kernel now adds it as it loads each cell row: 0.57 ms, and full mode went from
  20 to 30 fps.

Also:
- The REU length register is left at 1 after a transfer, so doubled mode rewrites `LEN = 2` per DMA. The
  model caught this before hardware.
- Line tables sit on a 2 KiB stride so none straddles a 512 KiB bank (TALON HD's rule; `emu.py` asserts
  it per DMA, `every_frame_dma_stays_inside_a_512k_bank`).
- The kernel runs ~20% over the cycle model (5 us a DMA in full mode against ~4.2 predicted). The CPU
  work between DMAs (~1.6 us) seems not to overlap the I/O bus. Doubled mode at 18 ms misses 60 fps by
  ~1.9 ms: the next optimisation target (fewer cycles between DMAs: unrolled lines, immediates).

## Milestone 4: flat-shaded 3D over the floor (emulator; not yet on hardware)

Pipeline (`src/geom.rs`, shared by host and C64, so both compute the same bits):
1. Per-object yaw transform into the floor camera's space.
2. Near-plane clip at 8 texels.
3. Projection matching the floor (`projection_matches_the_floor_camera`).
4. Backface cull by signed area.
5. One-pass 256-bucket depth sort (painter's).
6. Per-triangle set-up in Rust.

Then `asm/tri.S` walks the lines and `asm/poly.S` writes each cell row. Cells covered on all 8 lines get
8 plain stores; only line ends are read-modify-written. Arithmetic goes through `src/fixed.rs`: a
16x16 quarter-square multiply (`asm/fixed.S`, 352 cycles) and division by an 11-bit reciprocal table.
There is no divide per vertex or per edge.

Checks:
- `cargo test --release`: 16 tests. They include the cell writer equal to the per-pixel spec on 4,000
  rows, triangles covering exactly the pixel centres inside them, front/back winding, reciprocal error
  under 1/1024, and the projection against the floor camera.
- `python tools/test_poly.py`: `poly.S` equal to the per-pixel spec on 3,000 random rows.
- `python tools/test_tri.py`: `tri.S` + `poly.S` equal to a port of `geom::walk` on 500 random triangles.
- `python tools/test_fixed.py`: `m7_umul16` exact on 20,121 products.
- `python tools/emu.py --frames 6`: **0 mismatched bytes** against the host in full and doubled mode.

Two traps found on the way:
- An out-of-range branch in `tri.S` was silently mis-encoded and jumped into `poly.S`.
- `cargo test` does not rebuild `target/release/mode7.exe`. After changing shared geometry code, run
  `cargo build --release` before `emu.py`, or the host reference is stale.

Cost, 6502 interpreter at 64 MHz, test scene (54 objects, ~12-19 in view, ~57 triangles drawn):
**geometry 42-46 ms a frame** (`python tools/profile.py`).

| part | ms |
|---|---|
| `geom::draw`'s own code (per-vertex / per-face loops, i32 in Rust) | ~11 |
| multiplies (`m7_umul16` + sign wrappers; ~1,500 a frame) | ~11 |
| triangle set-up (`triangle`, `Edge::new`, `norm`, shifts) | ~9 |
| passing arguments to the asm (`tri_walk` wrapper, `memcpy`) | ~5 |
| rasterisation (`tri.S` + `poly.S`) | ~3.5 |

Before the fixed-point work it was 66 ms: `__mulsi3` cost ~1,500 cycles a call and `__divsi3` ~2,200.

Fill is cheap and everything around it is not. Next step: move the vertex transform, face loop (area
cull, sort insert) and edge set-up into assembly, as sueme64 does.

### Geometry in assembly

`asm/geom.S` now does what `geom::draw` did per vertex, per face and per triangle:
- `geo_xform`: transform and projection.
- `geo_faces`: area cull, depth key and bucket insert.
- `geo_raster`: per-triangle set-up, straight into `tri_walk`.

Rust keeps the per-object cull and the rare near-plane clip. The Rust functions (`geom::xform`, `faces`,
`raster`) stay as the host reference. Triangles and vertices live in fixed byte arrays (`geom::Work` at
`$E000`, `geom::Verts` at `$9400`) that both sides index.

`umul` uses pointer quarter squares: `a·b = f(a+b) - g(b-a+255)`, both lookups `(ptr),y`, with a second
table at `$F100`. It skips partial products whose high bytes are zero.

`tools/branch-audit.py` (from sueme64, extended to numeric labels) runs on every build. It reported no
wrapped branch in 123, and it catches a planted one.

Results:
- `emu.py`: **0 mismatched bytes**, full and doubled.
- **Geometry 19.4-22.5 ms a frame** (from 42-46 ms), ~56 triangles drawn.
- `m7_umul16`: 299 cycles through the C ABI (was 352), exact on 20,256 products.

| part (profile.py) | ms |
|---|---|
| multiplies (`umul`) | 3.1 |
| Rust per-object cull + argument passing | ~2.7 |
| per-triangle set-up (`edge`, `tsetup`, `norm`) | ~2.7 |
| line walk + row writes (`tri.S` + `poly.S`) | ~3.5 |
| bit shifts (`sarv`) | 1.25 |
| vertex loop (`xloop`) | 1.1 |

### On the C64U

Bug found on hardware: buffer B's screen at `$9C00` showed garbage colours. In VIC bank 2 the VIC sees
the character ROM at `$9000-$9FFF`, not RAM. The emulator missed it because it compares bitmaps only.
- Each buffer now has its own `$D018`: screens at `$5C00` (`$78`) and `$8C00` (`$38`). The flip writes
  `$DD00` then `$D018` in vblank.
- The reciprocals and vertex arrays moved into `$9000-$9FFF`, which is RAM to the CPU.
- `emu.py` now fails if a screen lands in the VIC's ROM window.

Measured on the C64U (held frames, `status`):

| mode | tick | floor | geometry | triangles / objects | fps | vs host |
|---|---|---|---|---|---|---|
| full | 1392 | 29.6 ms | 31.1 ms | 97 / 18 | 12 | **0 mismatched bytes** |
| doubled | 1491 | 17.8 ms | 30.0 ms | 90 / 18 | 17 | **0 mismatched bytes** |

Busier parts of the lap draw 90-104 triangles: about 0.32 ms a triangle, all included.

## Object grid and view distance

Large tracks: objects live in the REU and are streamed per visible cell each frame, as the floor
streams per byte.
- **Grid:** 16x16 cells of 16 texels. Each cell has a 3-byte RAM directory entry and its objects sit
  sorted by cell in REU bank 250, fetched with one DMA per non-empty visited cell.
- **Per-angle visibility lists** in bank 251: cells that can hold an object the cull accepts, far to near,
  from a separating-axis test of each cell's possible centres against the acceptance region. Each entry
  carries its nearest depth, so cells beyond the view distance are skipped unfetched. Lists are 91-115
  cells.
- **Global list:** objects whose bounding radius exceeds the 16-texel padding (the gantry and loops)
  are checked every frame.
- **Cull:** objects beyond 96 texels along either axis are rejected first, which keeps every accepted
  object within the lists' 7-cell reach.
- **Test:** `the_grid_visits_every_object_the_cull_accepts`, 3,000 cameras with random view distances.
- **View distance:** `STATUS+20` (texels, 0 = 64), set with `c64u.py view N`. Objects pop in.

Scene: 128 triangle-base cones (near/far LOD), the gantry, 3 ramps and 2 loops. Package version 6.

C64U, held frames read with the machine **paused**: **0 mismatched bytes** in full and doubled mode at
ticks 768, 860, 947 and 1037. The pause matters: earlier reads while the C64 kept flipping caught a buffer
being redrawn and showed 99-506 false mismatches (`c64u.py dump` now pauses).

View distance on the C64U, at one held busy frame (loop in view):

| view (texels) | objects | triangles | geometry |
|---|---|---|---|
| 32 | 10 | 103 | 40.3 ms |
| 48 | 13 | 108 | 42.6 ms |
| 64 | 16 | 112 | 44.6 ms |
| 96 | 23 | 124 | 48.7 ms |

The loop (global, ~60 front faces) dominates this frame. Distance mostly adds cheap far cones.

### Pop-in by size

The fixed view distance became a per-object one. An object is drawn once its bounding radius r would
cover at least `POP` pixels: pop distance `80·r / POP` texels, capped at 96. The cost is one divide per
frame (`pop_scale`) and one multiply per object (`pop_depth`).
- Directory entries carry their cell's largest radius, and a listed cell is skipped unfetched when even
  that object would pop in nearer than the cell's nearest depth.
- `STATUS+20` is now `POP` (pixels, 0 = 6), set with `c64u.py pop N`. Package version 7.
- With the default, cones (radius 5) appear at ~66 texels; ramps, gantry and loops at the 96-texel cap.
- The grid test now draws random pop sizes from 1 to 255.

C64U: held frames **0 mismatched bytes** (full tick 415, doubled tick 504). At one held frame:

| POP (px) | objects | triangles | geometry |
|---|---|---|---|
| 3 | 21 | 125 | 40.0 ms |
| 6 | 17 | 117 | 36.7 ms |
| 12 | 10 | 105 | 31.9 ms |

## Per-band publishing (one buffer)

The second buffer is gone, freeing RAM for the racers' AI. Each frame the C64 builds the screen one
cell row (band) at a time in a 320-byte staging buffer at `$8C00`: the floor row (per-byte DMA into
`STAGE`), then every triangle crossing the band in painter's order. `band_publish` then copies the band
to the bitmap at `$6000` once the raster is not in it. The frame tears between bands, which is accepted:
it is hardly visible in forward motion.

- **Triangles:** each one is set up when its first band comes (`geo_bin` files them by first band in
  painter's order) and takes a slot in a 96-entry pool. It joins the active list by sequence number, is
  walked one row per band (`tri_row`, its edge state saved in the slot) and is retired after its last band.
  When the pool is full the later triangles are dropped; the host reference (`geom::raster_bands`) mirrors
  the rule.
- **Sky bands** are rebuilt only when geometry touches them now or touched them last frame.
- **Floor kernel** split into `floor_begin` / `floor_row` / `floor_end`. Found in the emulator: the floor
  kernel's camera, mode and REU shadows lived in zero page that the triangle set-up now uses between rows.
  The camera and mode moved to RAM (`CAM_RAM` `$C7D4`, `FLOOR_MODE`), and each row restores them and starts
  from a known REU address.
- **RAM:** `$A000-$AEFF` holds the band state. **`$AF00-$BFFF` (4.25 KB) is reserved for AI.**
  `$8D40-$8FFF` and `$9D00-$9FFF` are also unused.
- **Checks:**
  - `band_rendering_equals_whole_triangle_painting` (32 lap frames) and `a_full_pool_drops_the_later_triangles`;
  - `test_tri.py` now drives `tri_row` per row;
  - `emu.py`: 0 mismatched bytes in full and doubled mode;
  - the branch audit caught 2 wrapped branches in `geo_activate` before they ran.

C64U (package version 8), byte-exact on held, paused frames (full tick 484, doubled tick 622):

| mode | fps | bands (floor + triangles + publish) | geometry (transform + sort + bin) | triangles |
|---|---|---|---|---|
| full | 14 | 53.6 ms | 18.7 ms | 122 |
| doubled | 17 | 49.3 ms | 27.1 ms | 167 |

There is no vblank wait before a flip now. Publishing costs ~1.6 ms a frame plus rare raster waits
(`profile.py --render`).
