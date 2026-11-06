# Chopper: game design

A helicopter campaign for the 64 MHz C64 Ultimate with a 16 MiB REU, in the spirit of Super Huey's
mission modes (flight school, rescue, recon, combat), with original missions, enemies and story.

## Principles

- **Looks real:** the instruments, the HUD's symbology, the island, and the helicopter's attitude in the view.
- **Feels real:** the rotor's start-up with the tach following it, the inertia of the flight model (the bank and pitch lag, speed builds and bleeds off), and the sound.
- **Plays like an arcade game:**
  - quick missions, clear objectives;
  - forgiving damage, generous ammunition;
  - missiles that lock on by themselves.
- **Simple, intuitive controls** (Miyamoto): one joystick, two buttons, no keyboard. The helicopter does the fiddly parts (the collective, trim, the landing flare) for you.

## The island

- **Size:** 2048×2048 texels (about 8 m each) with a radial coastline and a diagonal mountain ridge.
- **Pads:** five, joined by dirt roads (`world::PADS`).

| Pad | Where (texels) | In the campaign |
|---|---|---|
| 0 | (1024, 760) | Home base: hangar, tower, houses. Missions start and end here. |
| 1 | (1180, 900) | Outpost East |
| 2 | (860, 1060) | Outpost West: the convoy's start |
| 3 | (1240, 1250) | Field hospital (Medevac) |
| 4 | (980, 1400) | Enemy HQ (Fortress), the convoy's goal |

- **Mountains:** 22–34 texels tall, on a jittered lattice along the ridge. They block radar line of sight (the hazard map), so flying low behind them is the counter to SAMs.

## Story

The enemy has landed on the island's north shore and taken the outposts. One helicopter crew at home
base is all that is left. The missions take the island back, from a test flight to the assault on
the enemy's HQ.

## Flow

- **Title / attract:** the autopilot flies the island (as now). Any control (the stick, a button, or W A S D, SPACE, SHIFT) takes over.
- **Briefing:** a text screen with the mission's name, objectives, the threats expected and the loadout. Fire to fly.
- **Mission:** start on the home pad with the rotor spooling up (the sound's 1:20). The tach shows it, and the RPM lamp is lit until it reaches 100%.
- **Debrief:** pass or fail, with the targets destroyed and the time taken. On a pass, the next mission's briefing follows.
- **Pass:** every objective done, then land on the home H.
- **Fail:**
  - the helicopter is destroyed (damage reaches 100%);
  - fuel runs out;
  - a timed objective lapses (Medevac, Hunter).
- **Progress:** the campaign is played in order. A failed mission is replayed from its briefing.

## Missions

| # | Name | Objectives, in waypoint order | Enemies (per site) | Loadout |
|---|---|---|---|---|
| 1 | Shakedown | Lift off; fly through a gate over each of pads 1–4; land on the home H. Teaches the HUD's waypoint caret, the gauges and landing. | none | gun |
| 2 | Medevac | Fly to the field hospital (pad 3); land and hold 10 s while 2 wounded are loaded; return home within 4 minutes. | 2 AA guns beside the route, light flak | gun |
| 3 | Recon | Overfly 3 marked sites, within 40 texels of each mark. | each site: 1 radar, 2 AA guns | gun, 2 missiles |
| 4 | Hunter | Stop the convoy on the road from pad 2 to pad 4 before it arrives. Every truck must be destroyed. | 4 trucks, 2 APCs, 1 AA vehicle, all moving | gun, 4 missiles |
| 5 | Sea Lane | Sink the flotilla off the west coast, then hover low over a downed pilot's raft for 5 s. | 3 gunboats, 1 missile boat | gun, 4 missiles |
| 6 | SAM Belt | Destroy 3 SAM sites in the ridge's valleys; use the mountains to mask from the radars. | each site: 1 radar, 2 SAM launchers, 2 AA guns | gun, 6 missiles |
| 7 | Gunship Duel | Clear the 2-gunship patrol; a second pair intercepts once the first is down. | 2 gunships, then 2 more | gun, 8 missiles |
| 8 | Fortress | Assault the enemy HQ (pad 4): destroy the fuel depot, then the HQ bunker; return home. | 2 SAM sites (1 radar + 2 launchers each), 4 AA guns, 2 tanks, 2 gunships, 1 depot, 1 bunker | gun, 8 missiles |

Totals: mission 8 has 21 enemies, of which at most 16 are live at once. The gunships spawn when the
depot falls.

## Enemies

Low-poly meshes, at most 16 faces each, flat-shaded like the scene. Ranges are in texels.

| Type | Moves | Weapon | Range | To kill | Behaviour (GOAP tree) |
|---|---|---|---|---|---|
| Truck | on roads, 0.25 texel/tick | none | – | 1 burst | drive the route; stop and scatter when hit nearby |
| APC | on roads | machine gun | 60 | 3 bursts / 1 missile | escort; fire at a low helicopter in range |
| Tank | roads and off-road | cannon (hovering targets only) | 100 | 1 missile (the gun cannot hurt it) | hold, turn the turret, fire when the target is slow and low |
| AA gun (ZSU) | static, or with a convoy | flak bursts in a cone ahead of the target | 160 | 3 bursts / 1 missile | track, lead and fire in bursts; reload |
| Radar | static | none: cues the SAMs | 300, line of sight | 2 bursts / 1 missile | sweep; track a helicopter it can see over the hazard map |
| SAM launcher | static | missile (proportional navigation) after a 3 s radar track | 250 | 2 bursts / 1 missile | launch when its radar has a track; one missile in the air per launcher |
| Gunboat | sea | machine gun | 80 | 4 bursts / 1 missile | patrol; turn broadside to the helicopter and fire |
| Missile boat | sea | SAM, with its own radar | 250 | 1 missile | as a SAM site, afloat |
| Gunship | flies, with the agent's flight model | gun + missiles | 120 | 1 missile / 6 bursts | patrol, intercept, attack runs, break off when damaged |
| Fuel depot, bunker | static | none | – | 8 bursts / 2 missiles | objectives |

- Each enemy is a compact agent (16 bytes: position, heading, state, timers, hit points) stepped at the
  agent's tick. It decides every 8 ticks by walking its role's decision tree (`brain`, one root per
  role), planned offline by GOAP like the autopilot.
- **Warnings:**
  - a radar track lights the SAM lamp;
  - a launch flashes it and puts a launch cue on the HUD;
  - flak bursts and tracers are short-lived triangles.

### Limits (per frame / at once)

| What | Limit |
|---|---|
| Enemies simulated | 16 |
| Enemy triangles | at most 60 extra a frame, within `MAX_TRIS` (255) |
| Enemy missiles in the air | 4 |
| Target boxes on the HUD | 8 |

## The player

- **Flight:** one joystick, or the keyboard: W and S for forward and back, A and D for left and right. This is the agent's flight model with the stick in place of the decision tree's turns and speeds.
  - With the stick pushed, the keys are not read: joystick 2's lines share the keyboard's, and would lose keys or invent them. The keys work whenever the stick is centred.
  - Left and right turn; the helicopter banks into the turn by itself.
  - Forward speeds up, back slows down to a hover.
  - Altitude looks after itself: it holds a cruise height over the ground and climbs over mountains in its way (the autopilot's hazard probes and climb actions).
- **Landing:** hover (slow right down) over a pad's H and it settles onto it. Push forward to lift off again. No hard landings.
- **Weapons:** two buttons, one weapon each: on the keyboard, SPACE is the first (the gun) and either SHIFT the second (missiles).

| Weapon | Button | Load | Behaviour |
|---|---|---|---|
| Gun | fire (button 1) | 600 rounds | fires along the boresight while held, 80 texels. Every 4th round is a tracer, a short bright streak drawn as a thin triangle, so the stream shows where it is going. Hits are tested along the round's path. |
| Missiles | button 2 | 4–8 | fire and forget at the locked target; with no lock, button 2 does nothing |

- **Button 2** is the common 2-button convention: the second button is read on the SID's POT X line (`$D419`). The C64U's joysticks and most modern pads support it. One function reads both buttons, so they can be remapped.
- **Auto-lock:** one missile and one seeker for every target, in the air or on the ground. The diamond reticle is the seeker's field of view, a fixed area around the boresight.
  - When an enemy in range (250) is inside it, the missile system arms by itself.
  - The diamond moves onto that target and closes in as the lock builds over 1.5 s, then shows `LOCK`.
  - If the target leaves the reticle before then, the lock resets.
  - With several targets inside, the nearest one is chosen, and kept while it stays inside: the lock is on that vehicle, not its place in the list, so the vehicles of a formation taking turns at being nearest never restart it.

- **Fuel:** a mission's worth plus a margin. Landing on home refuels.
- **Damage:** 0–100%. At 50% the caution panel lights up; at 100% the helicopter goes down.
- **Cockpit:** the six gauges and the caution lamps (RPM, LOW ALT, AP), with SAM, FUEL and DAMAGE lamps to come.

## Sound

- **The rotor:** a recorded loop, streamed from the REU to the Ultimate's sampler (`asm/stream.S`).
- **The internal speaker:** `config/chopper-internal-speaker.cfg` is `config/chopper.cfg` plus the C64U's Speaker Mixer (mono): samplers at 0 dB and UltiSIDs at -6 dB, the same as its Audio Mixer for HDMI (+6 and 0 dB clipped the speaker), drive and tape noise off, for players without HDMI audio. (The rotor's right channel is its left 10 ms later, `format::HAAS`: if the mono sum sounds phasey, Sampler R can go OFF.)
- **Effects on four SIDs:** the C64U's two UltiSIDs, each split in two (`config/chopper.cfg`: range split on A5), are four SIDs: $D400 and $D420 hard left, $D440 and $D460 hard right. They make two stereo pairs, so two sounds play at once (`sfx`, its own overlays after the effects pass).
  - **A whole chip per ear:** a sound plays the same three voices on its pair's left and right chips. Each chip's master volume is that ear's gain and its low-pass cutoff that ear's muffling, so the filter colours that one sound only.
  - **Placed (a cheap HRTF):** its bearing from the nose pans it; behind the player both cutoffs close (muffled) and the far ear's more (the head's shadow); distance lowers the gain and dulls it. A sound playing is placed again every frame, so turning pans it.
  - **The sounds:** the player's gun and missile launches (centred), impacts, splashes, explosions, wrecks catching fire, the war's gunfire (the far battle quieter and duller), a hit on the player, the seeker's lock.
  - **Priorities:** a new sound takes a free pair, else the one with the least left of its sound (priority × gain, running down over its length), if it scores at least that: explosions 9, a hit 8, a launch 7, the lock 6, impacts 4, the gun and splashes 3, the war's gunfire 2, crackle 1. The gun keeps to one pair (its bursts onto the pair already firing, which nothing else takes from it), so the gun and a missile launch, say, are heard together.
  - **Sweeps:** the patches' pitch sweeps run at 60 Hz in the HUD's raster IRQ (its top-border row); the frame triggers, releases, sweeps the cutoff and places.
  - **The ears' lag (ITD):** a sound is gated on the near ear's chip first and on the far one up to 10 raster lines later (about 0.65 ms abeam, timed on `$D012`, so the CPU's clock doesn't matter; the lines passed are counted, so a raster IRQ meanwhile doesn't stretch it). Left and right are mirror images (the bearing rounds toward zero). An engine's lag follows the player turning without a restart: the lagging ear's chip runs a 256th slow until the next frame, and the lag is credited with the ticks it really ran slow, so uneven frames don't let it drift.
- **Engines:** every vehicle that moves has one: trucks and APCs, tanks and AA tanks, gunboats and missile boats, the carrier and destroyers, gunships (the sites, SAM launchers and projectiles make none). The two nearest by true distance (`squad::MEMBERS`) take the pairs no effect is using, as held patches (slow attack, released when they drop out of the two). An engine's revs drift slowly (about 4 s from idle to full) up while its vehicle moves and down while it stands, raising its pitch by at most a quarter (per patch; a restart begins halfway), and a platoon's vehicles changing places keep its engine going. The carrier is a giant diesel that never revs: a rumble: a 30 Hz triangle and a 36 Hz saw beating at about 6 Hz (the cylinders' throb, without the pops of a pulse) over deep low-passed noise. **Doppler:** an engine's pitch rises while it and the player close and falls once they open, by up to an eighth: its distance against last frame's, slewed so a flyby swings over about half a second (the game's speeds are far beyond true Doppler, so it's scaled by ear). An engine is the lowest priority: a near effect of note takes its pair (a free pair first) and the engine starts again once the effect is done; crackle and the far battle's sounds leave engines alone. The player's own rotor stays the sampled loop.
  - **Overlays:** the one-shots (`.text.sfx`) and the engines (`.text.sfxeng`) are two first-window overlays fetched in turn, each with its own patches. The placing and starting (`place`, `set`, `start`) are a second-window library (`.text.sfxlib`, at $A000) that stays loaded across both and runs the sequence (`sfx::frame`).

## The HUD

A head-up display of hi-res single-colour hardware sprites in light green, multiplexed over the 3D
view (talonhd's raster-IRQ multiplexer, extended to free X/Y positions with a sort and row packing):

| Element | Behaviour |
|---|---|
| Boresight | the gun cross, fixed at the view's centre |
| Waypoint caret | over the current waypoint; at the view's edge, pointing toward it, when it is off-screen or behind |
| Waypoint readout | `WP n` and the range |
| Target boxes | a bracket around each enemy in view (up to 8); they flash while that enemy is firing |
| Lock-on diamond | the seeker reticle around the boresight; when a target enters it, the missiles arm and the diamond moves onto the target, closes in while the lock builds, then shows `LOCK` |
| Launch cue | `MSL` flashing when a SAM is in the air toward you |

## Code overlays and scheduling

The C64 has little room for code beside the frame buffer and the renderer, so most game logic lives in
**REU overlays**:
- **Where the code comes from:** `chopper bake --elf` copies each overlay's code from the C64 build's ELF into the REU, with the build's stamp. The PRG only starts with an image baked from its own build.
- **Where it runs:** each overlay is DMA'd into a window over memory that is spent at that point in the frame. The main window is `$E000-$F0FF`, over the previous frame's triangles, during the set-up.
- **Shared helpers stay resident:** the projection and HUD placement maths.

Two kinds of overlay:

| Kind | When | What |
|---|---|---|
| Every frame | in order, each frame's set-up | work the frame's picture depends on: the flight's catch-up (the camera), the cockpit gauges, placing the HUD's markers, the grids' set-up for the view |
| Round-robin | one per frame, in turn | work that can be a frame or two stale. Enemies: **sense** (line of sight, radar, ranges), **decide** (each role's decision tree), **act** (aim, fire, spawn). Missions: objectives and waves. HUD: choosing the lock-on target. |

- **Why round-robin:** only one slow overlay loads per frame, so both the REU DMA and the CPU cost of the game logic are spread over frames. This is talonhd's scheme for its AI phases.
- **Overlay size:** each is at most 4 KB (its window).
- **Timing:** their results carry a tick stamp, so the every-frame code can extrapolate.

## The living world

A real war, not a stage set around the player:
- **Units:** both sides' units, 480 in all, live in the REU as 32-byte records. Each has a kind, side, health, position, route, orders, target and its own learning trail.
- **Platoons:** each record is a platoon of up to 12 vehicles of its kind (`world_sim::stats::MEMBERS`: 8 trucks, 10 APCs, 8 tanks, 6 AA guns, 4 SAM launchers, 3 gunboats, 2 missile boats, 4 gunships; sites, the carrier and its destroyers are single). A bit mask marks its fallen; the record's health is its wounded member's. A thousand to two thousand vehicles fight at once, at the cost of a few hundred units: the war's CPU goes on records, not vehicles.
  - **Fights:** each standing member fires, the hits rounded by the dice (`world_sim::hits`), and the damage spills over the target platoon's members, one falling at a time (`world_sim::spill`).
  - **Near the player:** each near platoon is drawn as its vehicles in formation round where it is (`squad`): a staggered column on the move, a line abreast fighting or holding, a V in the air. The nearest 20 are listed (those in front first); the nearest dozen are drawn, and the enemies among them are targets. The player's gun or missile kills the vehicle it hits.
  - **The battle on the horizon:** fights within 400 texels of the player show their muzzle flashes and hits wherever they are, in free effect slots only. They are marked far: the fight at hand's next effect takes a far one's slot first, and they are drawn after its effects and the wrecks' smoke, so they are what the effects' cap cuts.
- **Paging:** they are paged into RAM 32 a frame, round-robin, so each steps and decides about **once a second**. That is a human reaction time, chosen on purpose: no perfect aim, from near or far.
- **Away from the player:**
  - units travel along roads, sea lanes and air patrols;
  - they fight by dice against the enemies in their 128-texel sector;
  - reinforcements arrive from the enemy beachhead on the north shore and from home base;
  - sectors change hands, and wrecks stay where they fell.
- **Near the player:** units within 320 texels are promoted into 12 near platoons, expanded into their vehicles (above). Those are drawn every frame, boxed on the HUD, and they shoot with human aim: time to acquire, lead error that grows with crossing speed, burst spread.
- **Missions:** the eight missions are objectives over the real war, such as the actual convoy or the actual SAM sites. The war goes on between and during them.

## Learned behaviour

Each role's decision tree is planned offline by GOAP and baked into the REU, as the autopilot's is.
- **Learn nodes:** where two first actions are nearly as good, the tree holds a **learn node**: a byte giving the odds of one branch, as in Impact.
- **Rewards:** every decision is scored on the damage dealt minus the damage taken, plus a kill or a death. The last two decisions' learn nodes are nudged toward what worked.
- **Shared weights:** all units of a kind share one set of learned bytes (every gunship, every SAM site), so the REU cost stays fixed while the whole army keeps adapting. Allies learn the same way against them.
- **The attract-mode pilot** is a full combatant. It drives the same stick and buttons the player does, from its own learned tree: it hunts targets, masks from SAMs behind mountains, fires and goes home to refuel.

## Mission data (in the REU)

- **Briefing:** text, in screen codes.
- **Waypoints:** `(u, v, kind, radius, timer)`, where the kind is gate, land, hover, overfly or destroy (a set of target ids). The HUD's waypoint is the first undone.
- **Spawns:** `(type, u, v, yaw, route, site, wave)`. Routes are pad-to-pad road polylines. A wave spawns when an objective completes.
- **Loadout,** fuel and time limits.

## Build order

| Phase | Content |
|---|---|
| P0 | This design. |
| P1 | The HUD multiplexer over the attract flight: boresight, the waypoint caret and readout, target boxes and the lock-on diamond demoed on static targets. |
| P2 | Player flight: one stick (turn, speed), assisted altitude and landing. Recorded input keeps the emulator gate byte-exact. |
| P3 | Moving objects (a dynamic list after the cell walk) and the enemy agents, roles and meshes. |
| P4 | Weapons (the gun with tracers on button 1, missiles with auto-lock on button 2), hits, damage, explosions, the SAM warning. |
| P5 | Missions: the tables, objectives, briefing and debrief, landing, fuel. |
| P6 | Balance, sound effects, a full campaign play-test on the C64U. |
