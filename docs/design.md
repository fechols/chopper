# Chopper: game design

An open-world combat flight simulator for the 64 MHz C64 Ultimate with a 16 MiB REU: a war simulated
across an island, continuously, which the player joins from the carrier's deck in an attack helicopter.
(A campaign of missions was designed first, in the spirit of Super Huey, and never built: see the
appendix.) This document describes the game as it is; the code is the authority where they differ.

## Principles

- **Looks real:** the instruments, the HUD's symbology, the island, and the helicopter's attitude in the view.
- **Feels real:** the rotor's start-up with the tach following it, the inertia of the flight model (the bank and pitch lag, speed builds and bleeds off), and the sound.
- **The simulation is the game:** nothing is scripted. The war is fought by both sides' units on their own account, whether the player is there or not, and the autopilot is one combatant in it. The player's objectives are their own.
- **Plays like an arcade game:** forgiving damage, no ammunition count, missiles that lock on by themselves.
- **Simple, intuitive controls** (Miyamoto): one joystick, two buttons; the keyboard works too. The helicopter does the fiddly parts (the collective, trim, the landing) for you.

## The island

- **Size:** 2048×2048 texels (about 8 m each: the HUD's kilometres), a radial coastline (a wobble of `0.08 sin(3θ + 1.3) + 0.05 cos 5θ` plus noise) and a diagonal mountain ridge along `x = 0.6y + 0.2`.
- **Pads:** five (`world::PADS`), joined by straight 2-texel roads between consecutive pads (the war's routes also run off-road, `war`).

| Pad | Where (texels) | Half-size | In the war (`war`) |
|---|---|---|---|
| 0 | (1024, 760) | 10 | Home base (allied): hangar, tower, houses. Never captured. |
| 1 | (1180, 900) | 6 | East outpost (allied) |
| 2 | (860, 1060) | 6 | West outpost (enemy garrison) |
| 3 | (1240, 1250) | 6 | Field hospital (allied) |
| 4 | (980, 1400) | 6 | Enemy HQ: garrison and depot |

- **The beachhead:** the enemy's landing on the north shore near u = 980, the start of its front road.
- **Mountains:** three shapes (base radius / height 40/34, 30/22 and 50/28 texels) on a 56-texel lattice jittered ±15, where the elevation is over 0.5, kept clear of the pads. The autopilot's hazard map (`format::HAZARD`) keeps the helicopter off them; nothing else in the simulation uses line of sight.
- **The sea:** the carrier group (the carrier and two destroyers) shuttles between four points about 70 texels offshore; allied gunships loop from it over pads 0, 1 and 3; the enemy's air patrol loops from HQ over pad 2, the hospital and the beach, so the fights over the hospital are in the air. 150 texels beyond the map's edge the SEA lamp lights; at 300 the autopilot takes over and turns for home.

## Flow

- **Splash:** the box art, from the PRG's first instruction until the first frame (the turbo's start-up, the wait for the REU image with the border flashing, the sound's calibration). A multicolor bitmap in VIC bank 2, background blue (`Assets/Splash/chopper_splash.bin`, made from `Assets/Media/Chopper_splash.png` by `tools/encode-splash.py`, after ../sueme64's): the PRG loads its bitmap at `$A000` and its screen at `$8C00` where the VIC shows them, and `asm/splash.S` copies the colours to colour RAM and turns it on before the CRT runs. Nothing the boot writes lands on it; the band code whose pages are under its bitmap is copied there only after the boot, with the screen blanked (`chopper_relocate_band`). The PRG runs to `$BF40` (46,913 bytes).
- **Boot:** on the carrier's deck, 8 texels aft of its middle (`agent::DECK_AFT`), facing the bow (`Agent::boot`, placed at the first flight; at home if the carrier is gone). The rotor spools up for `LIFT` = 80 s (the sound's 1:20): the tach shows it, the RPM lamp is lit until it is at full power, and nothing lifts off before.
- **Attract:** the autopilot flies and fights ("The autopilot", below). Any control (the stick, a button, or W A S D, SPACE, SHIFT) takes over: `MODE_PLAYER`, the heading of the moment held as the course.
- **Hands off:** the copilot (the same pilot, flying to a carrot `CARROT` 256 texels ahead on the course) holds the heading and climbs over what is ahead. The course is retaken whenever left or right is held, and at lift-off.
- **Back to the autopilot** only beyond `SEA_LIMIT` 300 texels off the map (it heads home), or when down.
- **Down:** damage 100, or ditched in the sea (a splash). `combat::init` and `Agent::reset`: a fresh helicopter under the autopilot, on the carrier or at home in turn (the choice flips every 4096 ticks, about 68 s).
- No briefing, debrief, pass or fail: the war is the game.

## The player

- **Flight:** one joystick in port 2 (the second button on the SID's POT X line, `$D419`, the common two-button convention), or the keyboard: W and S forward and back, A and D left and right, SPACE fire, either SHIFT fire 2. With the stick pushed (or fire held) the keys are not read: joystick 2's lines share the keyboard's, and would lose keys or invent them.
  - It is the agent's flight model with the stick in place of the tree's bank and pitch (`Agent::step_with`); the tree still runs every `THINK` 8 ticks, for the height and the copilot.
  - Left and right: a bank of `PLAYER_BANK` 25° (the yaw rate follows the bank, about 23°/s). Wings level, the copilot steers back onto the course.
  - Forward and back: a throttle 0-64, a step a tick (about a second from hover to full). The nose goes down to hold it (at most about 11°) and up to slow (`FLARE`). Speed has inertia: it settles at 4× the pitch with a 32-tick time constant; full throttle is 0.5 texel a tick.
  - Lag: the attitude follows its targets an eighth of the way a tick, capped at `ROLL_RATE` 64 and `PITCH_RATE` 32 (binary degrees a tick).
  - Height is the pilot's: `CRUISE` 16 texels over the ground, climbing over what the five hazard probes see (24, 56 and 112 texels ahead, ±45° at 48).
- **Landing:** throttle at zero, speed under 256 and back held, the copilot lands (`LAND`, -24 a tick in 8.8), anywhere. No hard landings. On the carrier's deck (`DECK_HALF`) the deck carries the helicopter with it (`ride`). Forward lifts off once the rotor is up. Touching the sea is ditching: down.
- **Weapons:** two buttons, one weapon each.

| Weapon | Button | Load | Behaviour |
|---|---|---|---|
| Gun | fire | no count | airborne, while held, a burst every `GUN_EVERY` 6 ticks: a tracer flies from the nose (`TRACER_SPEED` 2 texels a tick for `TRACER_TTL` 60 ticks, so the stream shows where it is going; ground and sea impacts make dust or a splash), and the nearest enemy vehicle in the boresight's cone (`CONE_X` 12 × `CONE_Y` 10 pixels) within `GUN_RANGE` 80 texels is hit with `GUN_HIT` 180/256 odds for `GUN_DAMAGE` 2 |
| Missiles | fire 2 | `MISSILES` 8 | one a press, at the locked target; with no lock, nothing. It homes (at most 8/256 of a turn a tick, climbing to the target's height) at 14/16 texel a tick for 255 ticks, and hits within 3 texels for `MISSILE_DAMAGE` 40. Lost if its target drops off the near list. |

- **Projectiles:** `PROJ_MAX` 6 slots, tracers and missiles together; a missile takes a tracer's slot when none is free.
- **Auto-lock** (`hud`): the diamond reticle is the seeker's field of view (`RETICLE_X` 24 × `RETICLE_Y` 20 pixels round the boresight).
  - When an enemy within `LOCK_RANGE` 250 texels is inside it, the missile system arms by itself.
  - The diamond moves onto that target and closes in (three shapes) as the lock builds over `LOCK_TICKS` 90 (1.5 s), then shows `LOCK`, with the lock beep.
  - If the target leaves the reticle before then, the lock resets.
  - With several inside, the nearest (by camera depth) is chosen, and kept while it stays inside: the lock is on the vehicle (`LOCK_STATE`, its id), not its place in the list, so the vehicles of a formation taking turns at being nearest never restart it.
- **Damage spills:** damage to a platoon falls on its members one at a time (`world_sim::spill`), so a 40-point missile takes five 8-HP trucks.
- **Damage taken:** 0-100 (`DOWN`), in the war's fight phase (below). At `HURT` 50 the autopilot (not the player) goes home. Landed within `PAD` 16 texels of home, or on the deck: rearmed to 8 missiles and repaired a point a frame. There is no fuel.
- **Cockpit** (`cockpit`): six dials (IAS in knots, 2.5 kt a frame, cruise about 80; ATT, 25 roll × 10 pitch frames; ALT in feet to 2999, 31.25 ft a texel; the dual tach, 76 frames; HDG, 2.8° a frame; VSI, 41 frames) and the caution strip: RPM (the rotor below full power), LOW ALT (airborne under 6 texels), AP (the autopilot flying, off the pad; dark while the player has the aircraft) and SEA (150 texels off the map; in place of the others, in a second frame with AP once the autopilot has taken over). The lamp strip's ten frames fill the cockpit's banks to the HUD's shapes (`format::HUD_SHAPES`).
- **A hit:** the hit flash (`player::FLASH`) 12 ticks with the border red, and the picture shakes for `SHAKE_FRAMES` 30 (the fine scroll; the HUD sprites move the other way, to stay put over the view).

## The autopilot

The attract-mode pilot is a full combatant. It drives the same stick and buttons the player does, from its own learned tree (`goap`, `brain`): it flies to seeded random sectors, hunts the nearest enemy within `PREY_RANGE` 250 texels, strafes it, launches on lock, and goes home (at no more than half speed) at `HURT` 50 damage or with no missiles, to be rearmed and repaired on the pad; ditched or down, it starts again. Its decisions are scored like the units' (damage dealt minus taken, plus `KILL_BONUS` 20 for a kill), and its learn nodes move with them.

## The HUD

A head-up display of hi-res single-colour hardware sprites in light green, multiplexed over the 3D
view (talonhd's raster-IRQ multiplexer, extended to free X/Y positions with a sort and row packing: up
to 14 sprites a frame in 4 rows, from 9 sprite blocks):

| Element | Behaviour |
|---|---|
| Boresight | the gun cross, fixed at the view's centre |
| Waypoint caret | over the waypoint (the copilot's carrot in player mode; the autopilot's sector, prey or home); at the view's edge, one of 8 arrows toward it, when it is off-screen or behind (with hysteresis dead astern) |
| Waypoint readout | `WPT` and the range in km (8 m a texel): 0.0-9.9, then 10-19 |
| Target boxes | a bracket round each enemy vehicle in view within `BOX_RANGE` 250 texels: `BOXES` 8 at most, of `TARGETS` 12 listed |
| Lock-on diamond | the seeker reticle round the boresight; when a target enters it, the missiles arm and the diamond moves onto the target, closes in while the lock builds, then shows `LOCK` |

## The simulation

A real war, not a stage set around the player (`world_sim`, `war`):
- **Units:** both sides' units, `COUNT` 480, live in the REU as `RECORD` 32-byte records. Each has a kind, side, health, position, route, orders, target, the tick of its last step, and its own learning trail.
- **Platoons:** each record is a platoon of up to 10 vehicles of its kind (`stats::MEMBERS`: 8 trucks, 10 APCs, 8 tanks, 6 AA guns, 4 SAM launchers, 3 gunboats, 2 missile boats, 4 gunships; radars, the depot, the bunker, the carrier and its destroyers are single). A bit mask marks its fallen; the record's health is its wounded member's. Thousands of vehicles fight at the cost of a few hundred records: the war's CPU goes on records, not vehicles.
- **The opening order of battle** (`war`): the enemy 58 platoons (about 340 vehicles): a beachhead column of 24 platoons, the two pad garrisons and the depot (11), three SAM sites in the ridge's valleys (a radar, 2 SAM launchers and 2 AA guns each, where the site is on land), 4 boats and 4 gunships. The allies 28 (about 190): home defence (6), 8 convoys, 8 platoons at the front, the carrier with 2 destroyers, 3 gunships. The other 394 slots are reinforcements, dead at the start: the sides alternate, cycling APC, tank, AA, tank.
- **Paging:** every other frame (the gauges take the others) `step_page` pages `PAGE` 48 records into RAM and runs one phase over them, each phase its own overlay: **move** (`.text.world`), **decide** (`.text.world3`), **fight** (`.text.world2`); after the last page the **director** (`.text.world4`). A lap is 31 turns, 62 frames, so each unit moves, decides and fights about every two seconds at 30 frames a second (a human reaction time, by design; longer at the C64U's frame rates). Movement is by time (`dt` since the last step), so only decisions and fights slow with the frame rate. Records carry the tick of their last step and `live` extrapolates from it (up to `CATCH_UP` 1023 ticks), so a platoon drawn every frame moves smoothly between its steps, along its turning circle (`stats::TURN`).
- **Away from the player:**
  - units travel along roads, sea lanes and air patrols (`ROUTES`, baked by `war`);
  - the map is 16×16 sectors of 128 texels; a sector holds each side's count there and its **representative** (the unit of that side stepped there last), whom the other side's units in the sector fire at. A fight is dice: `hits = (n · HIT[kind] + dice) >> 8` for `n` members standing, each hit `DAMAGE[kind]`, spread over the target's members (`spill`), applied after the page is written back; `RELOAD` fights are skipped between shots. Only kinds whose `TARGETS` include the foe's class fire;
  - **the front:** each side has a front road (the enemy's: the beach, pad 4, pad 2, mid-island, pad 1, home) clipped to the first pad the other side holds. A pad changes hands when the other side has `CAPTURE` 2 or more platoons in its sector, and more than the owner, for `HOLD_PASSES` 3 director passes in a row (about 90 s); home is never taken. Every pad but home is checked, by its sector (`head::PAD_SECTOR`, baked), on a road or not. The hospital is on no road: only the enemy's air patrol passes over it, a platoon at a time, short of `CAPTURE`, so in practice it holds (a two-hour run: never taken). Sectors themselves have no owner;
  - **the director** runs once a lap and, once `WAVE_TICKS` 30 s have passed, moves the front and gives each side below its strength a budget of `wave` + `wave_per_pad` × the pads the other side holds (`head::KNOBS`, defaults strength 160/170, `per_pad` 24, `wave` 16, `wave_per_pad` 4: a side's strength is its base plus 24 a pad the other side holds, 232 and 218 platoons at the opening ownership);
  - **reinforcements:** a dead slot with a route respawns, after its `WRECK_TICKS` 30 s as a wreck and if its side has budget, two waypoints short of the front on its front road (not inside the fight for the pad: reinforcements arriving in it would make it untakeable; the beachhead or home only once the front has been pushed that far); convoys at home, boats on the sea lane, enemy gunships at pad 4, allied gunships from the carrier's deck (`launch`). Static units (sites, garrisons, the depot, the bunker, home defence) never respawn;
  - **retreat:** a unit falling back turns round on its road (`TURNED`, `BACK`) and keeps going, firing as it goes (a fighting withdrawal), until clear of foes; it waits for `RALLY` 4 friends in its sector, or gives up waiting with `BORED` 32/256 a decision. The carrier never retreats, and repairs itself a point now and then.
- **Near the player:** units within `NEAR_RANGE` 320 texels (each way) take one of `NEAR_MAX` 12 near slots, in page order (the carrier takes the last; a platoon dead under 30 s keeps its slot, drawn as a wreck with a smoke column). `squad` expands them into the nearest `SLOTS` 20 vehicles in formation round the platoon's position (a staggered column on the move, a line abreast fighting or holding, a V in the air; spaced for the units' size: 6 texels on the ground, 10 in the air, 12 at sea, `squad::half`; platoons astern count 192 texels farther), of which `DYN_MAX` 12 are drawn in view, projectiles first. The enemies among them are the HUD's targets, and the player's gun or missile kills the vehicle it hits. A member killed in sight explodes and burns for 20 s; its platoon's wreck stays 30 s.
  - **Their fire at the player** (the fight phase): a near enemy platoon that can hit air, ready and fighting, within `PLAYER_RANGE` 200 texels (|du| + |dv|) rolls `HIT[kind] >> (1 + fast)` (`fast`: airspeed over 6000, three quarters of full), at most `PLAYER_HITS` 2 hits a volley, each `DAMAGE[kind]`. There is no height, facing or line-of-sight check: speed is the player's armour. A platoon that fires at the player skips its sector fight that turn.
- **The battle on the horizon:** fights by units within `FIRE_RANGE` 400 texels of the player show their muzzle flashes and hits wherever they are. With neither side near, the effect is marked far and takes a free slot only; a near effect evicts a far one first, and far effects are drawn last, after the near effects and the wrecks' smoke, so they are what the cap (`FXDYN_MAX` 15 of `FX_MAX` 16) cuts. Their sounds are quieter and duller (`sfx`).

### Units

Low-poly meshes, at most 16 faces each, flat-shaded like the scene, drawn `actors::UNIT_SCALE` 2 times their
modelled size (a truck 3.6 texels long, a tank 2.2 wide, the carrier 28 × 8 with its deck 2.4 above the
sea: at the modelled 1-2 texels a unit was a few pixels at fighting range); wrecks likewise, the player's
tracers and missiles not. Every kind shares the one decision
tree (below); what differs is the stats (`world_sim::stats`): hit points, speed (1/16 texel a tick), the
odds of a hit (of 256, per standing member, per fight), its damage, the reload (fights skipped), what it
can hit (its targets' class), its own class, and its turning circle (`TURN`: about speed · 20 / 2^TURN
texels in radius: a vehicle's 1-3, a gunship's 4, a boat's 5, a destroyer's 10, the carrier's 20, a U-turn
in 17 s).

| Kind | HP | Speed | Hit | Damage | Reload | Hits | Class | Members |
|---|---|---|---|---|---|---|---|---|
| Truck | 8 | 2 | 0 | 0 | 0 | nothing | ground | 8 |
| APC | 20 | 2 | 90 | 3 | 0 | ground | ground | 10 |
| Tank | 40 | 1 | 120 | 8 | 1 | ground | ground | 8 |
| AA gun (static, or with a convoy) | 16 | 1 | 140 | 4 | 0 | air | ground | 6 |
| SAM launcher | 12 | 0 | 100 | 12 | 2 | air | ground | 4 |
| Radar | 10 | 0 | 0 | 0 | 0 | nothing (a target) | ground | 1 |
| Gunboat | 24 | 2 | 90 | 3 | 0 | ground, air, sea | sea | 3 |
| Missile boat | 20 | 2 | 80 | 10 | 2 | air, sea | sea | 2 |
| Gunship | 24 | 6 | 110 | 6 | 0 | ground, air, sea | air | 4 |
| Fuel depot | 30 | 0 | 0 | 0 | 0 | nothing | ground | 1 |
| Bunker | 60 | 0 | 0 | 0 | 0 | nothing | ground | 1 |
| Carrier | 250 | 1 | 60 | 4 | 0 | air | sea | 1 |
| Destroyer | 120 | 2 | 120 | 10 | 1 | ground, air, sea | sea | 1 |

- Tanks and APCs cannot hit air: the player is safe from armour. Radars cue nothing; they are targets.
- Units fire no projectiles: all their fire is dice, shown as a muzzle flash and a hit. The only projectiles are the player's tracers and missiles.
- Gunships fly their patrol loops at a fixed height (enemy 20 texels, allied 16) and speed; they do not use the helicopter's flight model.

### Limits (per frame / at once)

| What | Limit |
|---|---|
| Near platoons | `NEAR_MAX` 12 |
| Vehicles listed / drawn | `squad::SLOTS` 20 / `DYN_MAX` 12 |
| Targets / target boxes | `hud::TARGETS` 12 / `BOXES` 8 |
| Effects / drawn | `FX_MAX` 16 / `FXDYN_MAX` 15; `WRECK_MAX` 6 smoke columns |
| Projectiles | `PROJ_MAX` 6 (the player's) |
| Triangles | `MAX_TRIS` 255 |

## Learned behaviour

One decision tree, planned offline by GOAP (`roles::compile`, as the autopilot's by `goap`) and baked
into the REU (`format::TREES`, 0x0A00 bytes), with a copy per kind: six features (wounded, foes,
outnumbered, ready, armed, mobile) and four moves (advance, hold, attack, retreat). `ARMED` and `MOBILE`
come from the stats (a hit chance; a speed and a route), so a truck's tree never attacks and a SAM site's
never moves. The carrier never retreats.
- **Learn nodes:** where the two best first moves tie within `MARGIN` 1 in plan cost, the tree holds a **learn node**: a byte giving the odds of one branch (as in Impact), from the costs, clamped to 8..248. The plain nodes are shared; each kind has its own learn bytes.
- **Rewards:** every decision is scored on the damage dealt minus the damage taken since the last (`(DEALT − TAKEN) >> reward_shift(kind)`, a platoon's share, clamped to ±15; the player's hits count as taken). Against a per-kind baseline, the last two decisions' learn nodes (the trail) are nudged toward what worked, the older one half as far, at most 32 a step. (A death is credited to the slot's trail at its first decision after it respawns: the trail is not cleared.)
- **Shared weights:** all units of a kind, on **both sides**, share one set of learned bytes (every gunship, every SAM launcher), so the REU cost stays fixed while the whole army keeps adapting, and the allies teach the enemy's tanks as the enemy's teach theirs. The bytes are copied into RAM (`layout::BRAIN_RAM`) for each decide phase and written back.

## Sound

- **The rotor:** a recorded loop, DLT5 in the REU, decoded (`asm/stream.S`, the `stream` overlay, running its first 80 s) into a ring at `$C00000` that the Ultimate's sampler plays round and round, the right channel 10 ms behind the left (`format::HAAS`, one stream for both ears). The sampler's clock divider is measured at boot, and the frame clock starts with the sound, so the flight's time is the sound's.
- **The internal speaker:** `config/chopper-internal-speaker.cfg` is `config/chopper.cfg` plus the C64U's Speaker Mixer (mono): samplers at 0 dB and UltiSIDs at -6 dB, the same as its Audio Mixer for HDMI (+6 and 0 dB clipped the speaker), drive and tape noise off, for players without HDMI audio. (If the mono sum of the Haas pair sounds phasey, Sampler R can go OFF.)
- **Effects on four SIDs:** the C64U's two UltiSIDs, each split in two (`config/chopper.cfg`: range split on A5), are four SIDs: `$D400` and `$D420` panned left, `$D440` and `$D460` right. They make two stereo pairs, so two sounds play at once (`sfx`, its own overlays after the effects pass).
  - **A whole chip per ear:** a sound plays the same three voices on its pair's left and right chips. Each chip's master volume is that ear's gain and its low-pass cutoff that ear's muffling, so the filter colours that one sound only.
  - **Placed (an HRTF on SIDs, `sfx::place`):** every cue comes from the source's direction and distance in 3D, so the cues always agree.
    - **The lateral angle** (the cosine to the ears' axis, with the height: a source straight below is centred, and a pass overhead swings smoothly from one ear to the other) sets the ears' lag (Woodworth's sphere, 0.64 ms abeam), the far ear's level (10.5 dB down abeam) and its muffling (the head's shadow: its cutoff an octave down abeam), from 17-step tables. The bank tilts the ears' axis with the view: banked right, a target below is heard on the right.
    - **Behind**, both cutoffs close (1.25 octaves); **the 3D distance** lowers the gain (1/distance beyond 16 texels) and dulls it (the air: an octave by 512 texels).
    - **Consistent:** levels and cutoffs are worked in 16ths of an octave and added, so each cue moves a sound by the same amount whatever the others do. The shadow is the same octave ahead as behind, and the level gap is the same near as far, until the 4-bit volume runs out of steps (it rounds by ratio, so a far sound still pans; the lag, which doesn't depend on level, carries the direction past that).
    - **Placed every frame** (the one-shots' overlay places every pair: `sfx::aim`, stored at `sfx::PLACED`), so turning pans what is playing. An engine begun starts silent and glides in to its place. Engines are placed a frame late (where their overlay last moved them: placing them after it would need both overlays in the first window at once).
    - **Smooth:** the frame sets each chip's volume and cutoff as targets, and the raster IRQ glides the chips to them at 60 Hz (`asm/hud.S` `sfx_slew`, in the frame's last row, past its video writes: a quarter of the way each video frame; a sound begun snaps its levels while the slews hold off, `sfx::HOLD`), so a pan sounds the same whatever the frame rate.
    - **Every voice of a world sound goes through its chip's filter** (a `const` check), so none escapes the shadow.
  - **The sounds:** the player's gun and missile launches (centred), impacts, splashes, explosions, wrecks catching fire, the war's gunfire (the far battle 6 dB quieter and duller), a hit on the player, the seeker's lock.
  - **Priorities:** a new sound takes a free pair, else the one with the least left of its sound (priority × the louder ear's volume + 1, running down over its length), if it scores at least that: explosions 9, a hit 8, a launch 7, the lock 6, impacts 4, the gun and splashes 3, the war's gunfire 2, crackle 1. The player's own sounds (`OWN`: the gun, a launch, a hit, the lock; counted as 16) keep to one pair that the world's sounds never take, so the gun is never lost to the battle; among themselves a launch cuts in over the gun, and the gun comes back when the launch's gate is released.
  - **Sweeps:** the patches' pitch sweeps run at 60 Hz in the HUD's raster IRQ (its top-border row); the frame triggers, releases, sweeps the cutoff and places.
  - **The ears' lag (ITD):** a sound is gated on the near ear's chip first and on the far one up to 10 raster lines later (`sfx::wait`, in the sound library overlay: a busy wait on `$D012` counting the lines passed, about 0.64 ms abeam, so the CPU's clock doesn't matter and a raster IRQ meanwhile doesn't stretch it). Left and right are mirror images (the bearing rounds toward zero). The lag then follows the player turning, without a restart: the lagging ear's chip runs a 256th slow until the next frame, and the lag is credited with the ticks it really ran slow, so uneven frames don't let it drift. Engines do this all along; a one-shot does it once its pitch sweep is done (the IRQ owns its pitch till then).
- **Engines:** every vehicle that moves has one, from six patches: trucks and APCs; tanks and AA guns (the static ones too: they idle); gunboats and missile boats; destroyers; gunships (a SID rotor); the carrier (the sites, the SAM launchers, radars, the depot and bunker, and projectiles make none). The two nearest by ground distance (`squad::MEMBERS`) take the pairs no effect is using, as held patches (slow attack, released when they drop out of the two). An engine's revs drift up while its vehicle moves and down while it stands (`REVS` 255 at a step a tick: about 4 s from idle to full; a restart begins halfway), raising its pitch by up to a quarter (per patch), and a platoon's vehicles changing places keep its engine going. The carrier is a giant diesel that never revs: a rumble, a 31 Hz triangle and a 37 Hz saw (NTSC; 30 and 36 on PAL) beating at about 6 Hz (the cylinders' throb, without the pops of a pulse) over deep low-passed noise. **Doppler:** an engine's pitch rises while it and the player close and falls once they open, by up to an eighth: its ground distance against last frame's (the engines' overlay has no room for the height), slewed ±8 a frame so a flyby swings over about half a second (the game's speeds are far beyond true Doppler, so it's scaled by ear). An engine is the lowest priority: a near effect of note takes its pair (a free pair first) and the engine starts again once the effect is done; crackle and the far battle's sounds leave engines alone. The player's own rotor stays the sampled loop.
  - **Overlays:** the one-shots (`.text.sfx`, with `place` and its tables) and the engines (`.text.sfxeng`) are two first-window overlays fetched in turn, each with its own patches. Setting and starting a sound from its stored place (`set`, `start`, `track`, `wait`) are a second-window library (`.text.sfxlib`, at `$A000`) that stays loaded across both and runs the sequence (`sfx::frame`). `place` and everything it calls keep off `camera::sin` (its asm multiply puts its callers on llvm-mos's soft stack, a few hundred bytes more code): the sounds have their own sine in 16ths.

## Code overlays and scheduling

The C64 has little room for code beside the frame buffer and the renderer, so most game logic lives in
**REU overlays**:
- **Where the code comes from:** `chopper bake --elf` copies each overlay's code from the C64 build's ELF into the REU, with the build's stamp. The PRG only starts with an image baked from its own build.
- **Where it runs:** three windows over memory that is spent at that point in the frame (`target-c64/link.ld`): window 1 at `$E000` (0x1800 bytes, over the previous frame's triangles, during the set-up), window 2 at `$A000` (0x0F00) and window 3 at `$F010` (0x07F0). `tools/memmap.py` lists every overlay's size and headroom (`docs/memory.md`); several are within tens of bytes of their window, and the stream decoder stops at `$F0FF` for its buffers.
- **Shared helpers stay resident:** the renderer, the projection and HUD placement maths, the sound slews.

The frame (`target-c64/src/main.rs`), in order, each step fetched every frame unless noted:

| Step | Overlay (window) | What |
|---|---|---|
| ai | `ai` (1), `deck` (2) | the flight's catch-up, the camera; the carrier's deck |
| war | `world`, `world3`, `world2`, `world4` (1) | on alternate frames (`TURN` 0) one war phase (move, decide, fight; the director once a lap) over a page of 48 records |
| squad | `squad` (1) | the near platoons as vehicles |
| combat | `combatlib` (2), `combat` (1) | the gun, missiles, hits and damage |
| fx | `fx` (1) | explosions, smoke, flashes |
| sfx | `sfxlib` (2), `sfx`, `sfxeng` (1) | the effects and the engines |
| cockpit | `cockpit` (2) | the gauges that changed, on the frames the war skips; the targets read every frame for the HUD |
| hud | `hud` (1) | placing the markers, the lock |
| actors | `actors` (1) | the dynamic objects for the renderer |
| stream | `stream` (1) | the rotor sample, while it streams |
| geometry | `geomsetup` (1), `clip` (3), `dyn` (2) | the grids' set-up for the view, the cell walk, the triangle binning |

- **Why alternate:** the war is round-robin over its page file, one phase a turn, so both the REU DMA and the CPU cost of the simulation are spread over frames (talonhd's scheme for its AI phases), and the gauges, which can be a frame stale, take the frames between.
- **Timing:** the war's records carry a tick stamp, so a platoon drawn every frame is extrapolated between its steps.

## Appendix: not built, the campaign as designed

The game was first designed as a campaign: the enemy had landed on the north shore and taken the
outposts, one crew at home base was all that was left, and eight missions took the island back, from a
test flight to the assault on the enemy's HQ, each with a briefing (a text screen: name, objectives,
threats, loadout), a pass (every objective done, then land on the home H), a fail (destroyed, out of
fuel, a timed objective lapsed) and a debrief. The mission tables (briefing text in screen codes;
waypoints as `(u, v, kind, radius, timer)`, kind gate, land, hover, overfly or destroy; spawns as
`(type, u, v, yaw, route, site, wave)`; loadout, fuel and time limits) were to live in the REU, and the
director's knobs (`head::KNOBS`) were to be set per mission. None of it was written; the simulation
became the game instead.

| # | Name | Objectives, in waypoint order | Enemies (per site) | Loadout |
|---|---|---|---|---|
| 1 | Shakedown | Lift off; fly through a gate over each of pads 1–4; land on the home H. | none | gun |
| 2 | Medevac | Fly to the field hospital (pad 3); land and hold 10 s while 2 wounded are loaded; return home within 4 minutes. | 2 AA guns beside the route | gun |
| 3 | Recon | Overfly 3 marked sites, within 40 texels of each mark. | each site: 1 radar, 2 AA guns | gun, 2 missiles |
| 4 | Hunter | Stop the convoy on the road from pad 2 to pad 4 before it arrives. | 4 trucks, 2 APCs, 1 AA vehicle | gun, 4 missiles |
| 5 | Sea Lane | Sink the flotilla off the west coast, then hover over a downed pilot's raft for 5 s. | 3 gunboats, 1 missile boat | gun, 4 missiles |
| 6 | SAM Belt | Destroy 3 SAM sites in the ridge's valleys. | each site: 1 radar, 2 SAM launchers, 2 AA guns | gun, 6 missiles |
| 7 | Gunship Duel | Clear the 2-gunship patrol; a second pair intercepts once the first is down. | 2 gunships, then 2 more | gun, 8 missiles |
| 8 | Fortress | Assault the enemy HQ (pad 4): destroy the fuel depot, then the HQ bunker; return home. | 2 SAM sites, 4 AA guns, 2 tanks, 2 gunships, 1 depot, 1 bunker | gun, 8 missiles |

Also designed and not built: fuel; a 600-round gun with every fourth round a tracer and hits tested
along the round's path; per-kind behaviour (radar line of sight over the hazard map cueing SAMs after a
3 s track, proportional-navigation missiles, flak in a cone, a tank that fires only at slow, low
targets, gunboats turning broadside, gunships' attack runs and break-off); human aim (time to acquire,
lead error growing with crossing speed, burst spread); a SAM lamp and an `MSL` launch cue; target
boxes flashing while that enemy fires; SAM, FUEL and DAMAGE caution lamps; lasting wrecks; and
sectors changing hands.
