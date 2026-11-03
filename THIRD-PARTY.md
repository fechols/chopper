# Third-party code

Chopper's renderer comes from `../mode7` (same author), which in turn vendors code from `../talonhd`
(TALON HD) and gclass. Copied files keep their provenance notes in their headers.

## mode7 (`../mode7`)

| File | From | Changes |
|---|---|---|
| `src/geom.rs`, `target-c64/asm/geom.S` | mode7 @ `5014bc1` | racers (`car`, `cars`) removed; chopper's 3D camera, grid and bands follow |
| `src/poly.rs`, `src/fixed.rs`, `target-c64/asm/{poly,tri,fixed,reu}.S` | mode7 | unchanged |
| `target-c64/asm/startup.S`, `target-c64/link.ld` | mode7 @ `5014bc1` | chopper's relocated regions and boot section |
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
| `tools/encode-digital-music/` | `../sueme64/tools/encode-digital-music` @ `8685892` (itself extending Second Reality's DLT2 quantizer; Unlicense) | none |
| `src/digital.rs` | the DLT5 header and decode rule of `../sueme64/src/digital.rs` @ `8685892` | mono only, whole-file host decode |

## impact (`../impact` @ `6aa056c`, same author, MIT): baked AI decision trees

| File | From | Changes |
|---|---|---|
| `src/brain.rs` | the node format of `impact/src/brain.rs` (`[feature, threshold, ge, lt]`, `LEAF`, children before parents) | chopper's header and roots; the walker fetches one node per step instead of a 48-byte window, and is Rust shared with the host (impact's is `asm/ai_walk.S`) |
| `src/goap.rs` | the postorder, deduplicating emitter of `impact/src/brain_compile.rs` | the trees come from a GOAP planner (A* over symbols) and ID3, not hand-written policies |

## TALON HD (`../talonhd`, same author, MIT): the HUD's raster IRQ multiplexer

| File | From | Changes |
|---|---|---|
| `target-c64/asm/hud.S` | `talonhd/target-c64/asm/hud.S` (`hud_irq`: a row's Ys and pointers per raster IRQ, rows set up at the previous row's Y+22) | a free X per sprite (`$D000-$D00F`, `$D010`), `$D015` per row, IRQ lines from the table, two tables swapped at the top-border IRQ; `$01` left alone |
| `src/hud.rs` | the timing rules and the block baking style of `talonhd/src/hud.rs` | free-position sprites sorted and packed into rows (`pack`), the waypoint caret, target boxes and the auto-lock seeker |
| `tools/test_hud_irq.py` | `talonhd/tools/test_hud_irq.py` | chopper's table contract |
