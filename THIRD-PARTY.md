# Third-party code

Chopper's renderer comes from `../mode7` (same author), which in turn vendors code from `../talonhd`
(TALON HD) and gclass. Copied files keep their provenance notes in their headers.

## mode7 (`../mode7`)

| File | From | Changes |
|---|---|---|
| `src/geom.rs`, `target-c64/asm/geom.S` | mode7 @ `5014bc1` | racers (`car`, `cars`) removed; chopper's 3D camera, grid and bands follow |
| `src/poly.rs`, `src/fixed.rs`, `target-c64/asm/{poly,tri,fixed,reu,startup}.S` | mode7 | unchanged |
| `src/scene.rs`, `src/hw.rs`, `src/model.rs`, `src/layout.rs`, `src/format.rs` | mode7 | adapted to chopper's floor and REU layout |
| `tools/{emu.py,c64u.py,profile.py,test_*.py}` | mode7 | adapted |

## TALON HD (`../talonhd` @ `e0e891c`, by way of mode7 @ `5014bc1`): sprites

| File | From | Changes |
|---|---|---|
| `src/sprite.rs` | mode7 `src/sprite.rs` (`draw_cols` from `talonhd/src/render.rs`) | none |
| `target-c64/asm/blit.S` | mode7 `asm/blit.S` (from `talonhd/target-c64/asm/blit.S`) | none |
| `tools/test_blit.py` | mode7 `tools/test_blit.py` (from `talonhd/tools/test_blit.py`) | chopper names |
| `src/raster3d.rs` | `talonhd/src/raster3d.rs` | none |
| `src/encode.rs` | `talonhd/src/encode.rs` (pixel packing from gclass `a5775bf`) | none; its constants live in `src/format.rs` |

## sueme64 (H.A.V.O.C.)

| File | From | Changes |
|---|---|---|
| `tools/branch-audit.py` | `../sueme64/tools/branch-audit.py` @ `d962bde` | resolves numeric local labels |
