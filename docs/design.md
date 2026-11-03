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

- **Title / attract:** the autopilot flies the island (as now). Fire starts the campaign.
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

- **Flight:** one joystick. This is the agent's flight model with the stick in place of the decision tree's turns and speeds.
  - Left and right turn; the helicopter banks into the turn by itself.
  - Forward speeds up, back slows down to a hover.
  - Altitude looks after itself: it holds a cruise height over the ground and climbs over mountains in its way (the autopilot's hazard probes and climb actions).
- **Landing:** hover (slow right down) over a pad's H and it settles onto it. Push forward to lift off again. No hard landings.
- **Weapons:** two buttons, one weapon each.

| Weapon | Button | Load | Behaviour |
|---|---|---|---|
| Gun | fire (button 1) | 600 rounds | fires along the boresight while held, 80 texels. Every 4th round is a tracer, a short bright streak drawn as a thin triangle, so the stream shows where it is going. Hits are tested along the round's path. |
| Missiles | button 2 | 4–8 | fire and forget at the locked target; with no lock, button 2 does nothing |

- **Button 2** is the common 2-button convention: the second button is read on the SID's POT X line (`$D419`). The C64U's joysticks and most modern pads support it. One function reads both buttons, so they can be remapped.
- **Auto-lock:** one missile and one seeker for every target, in the air or on the ground. The diamond reticle is the seeker's field of view, a fixed area around the boresight.
  - When an enemy in range (250) is inside it, the missile system arms by itself.
  - The diamond moves onto that target and closes in as the lock builds over 1.5 s, then shows `LOCK`.
  - If the target leaves the reticle before then, the lock resets.
  - With several targets inside, the nearest one is chosen.

- **Fuel:** a mission's worth plus a margin. Landing on home refuels.
- **Damage:** 0–100%. At 50% the caution panel lights up; at 100% the helicopter goes down.
- **Cockpit:** the six gauges and the caution lamps (RPM, LOW ALT, AP), with SAM, FUEL and DAMAGE lamps to come.

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
| P2 | Player flight: joystick cyclic, keys for the collective. Recorded input keeps the emulator gate byte-exact. |
| P3 | Moving objects (a dynamic list after the cell walk) and the enemy agents, roles and meshes. |
| P4 | Weapons (the gun with tracers on button 1, missiles with auto-lock on button 2), hits, damage, explosions, the SAM warning. |
| P5 | Missions: the tables, objectives, briefing and debrief, landing, fuel. |
| P6 | Balance, sound effects, a full campaign play-test on the C64U. |
