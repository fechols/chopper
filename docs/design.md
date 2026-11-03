# Chopper: game design

A helicopter campaign for the 64 MHz C64 Ultimate with a 16 MiB REU, in the spirit of Super Huey's
mission modes (flight school, rescue, recon, combat), with original missions, enemies and story.

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
| 2 | Medevac | Fly to the field hospital (pad 3); land and hold 10 s while 2 wounded are loaded; return home within 4 minutes. | 2 AA guns beside the route, light flak | gun, 19 rockets |
| 3 | Recon | Overfly 3 marked sites below 200 ft, within 40 texels of each mark. | each site: 1 radar, 2 AA guns | gun, 19 rockets |
| 4 | Hunter | Stop the convoy on the road from pad 2 to pad 4 before it arrives. Every truck must be destroyed. | 4 trucks, 2 APCs, 1 AA vehicle, all moving | gun, 38 rockets, 4 missiles |
| 5 | Sea Lane | Sink the flotilla off the west coast, then hover low over a downed pilot's raft for 5 s. | 3 gunboats, 1 missile boat | gun, 38 rockets, 4 missiles |
| 6 | SAM Belt | Destroy 3 SAM sites in the ridge's valleys; use the mountains to mask from the radars. | each site: 1 radar, 2 SAM launchers, 2 AA guns | gun, 38 rockets, 6 missiles |
| 7 | Gunship Duel | Clear the 2-gunship patrol; a second pair intercepts once the first is down. | 2 gunships, then 2 more | gun, 38 rockets, 8 missiles |
| 8 | Fortress | Assault the enemy HQ (pad 4): destroy the fuel depot, then the HQ bunker; return home. | 2 SAM sites (1 radar + 2 launchers each), 4 AA guns, 2 tanks, 2 gunships, 1 depot, 1 bunker | gun, 38 rockets, 8 missiles |

Totals: mission 8 has 21 enemies, of which at most 16 are live at once. The gunships spawn when the
depot falls.

## Enemies

Low-poly meshes, at most 16 faces each, flat-shaded like the scene. Ranges are in texels.

| Type | Moves | Weapon | Range | To kill | Behaviour (GOAP tree) |
|---|---|---|---|---|---|
| Truck | on roads, 0.25 texel/tick | none | – | 1 gun burst | drive the route; stop and scatter when hit nearby |
| APC | on roads | machine gun | 60 | 2 rockets | escort; fire at a low helicopter in range |
| Tank | roads and off-road | cannon (hovering targets only) | 100 | 1 missile / 3 rockets | hold, turn the turret, fire when the target is slow and low |
| AA gun (ZSU) | static, or with a convoy | flak bursts in a cone ahead of the target | 160 | 2 rockets | track, lead and fire in bursts; reload |
| Radar | static | none: cues the SAMs | 300, line of sight | 1 rocket | sweep; track a helicopter it can see over the hazard map |
| SAM launcher | static | missile (proportional navigation) after a 3 s radar track | 250 | 1 rocket | launch when its radar has a track; one missile in the air per launcher |
| Gunboat | sea | machine gun | 80 | 3 rockets | patrol; turn broadside to the helicopter and fire |
| Missile boat | sea | SAM, with its own radar | 250 | 1 missile | as a SAM site, afloat |
| Gunship | flies, with the agent's flight model | rockets + gun | 120 | 1 missile / 4 rockets | patrol, intercept, attack runs, break off when damaged |
| Fuel depot, bunker | static | none | – | 3 rockets / 2 missiles | objectives |

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

- **Flight:** the joystick is the cyclic. Left and right bank, which turns; forward and back pitch, which sets speed. The collective (climb and descend) is on keys. This is the agent's flight model with the player's controls in place of the decision tree.
- **Landing:** on a pad's H, slowly, at near-zero climb. A hard landing damages the helicopter.
- **Weapons** (fire to shoot; space cycles the weapon):

| Weapon | Load | Behaviour |
|---|---|---|
| Gun | 600 rounds | hit-scan along the boresight, 80 texels |
| Rockets | 19 / 38 | unguided, fired in pairs along the boresight, 200 texels |
| Missiles | 4–8 | need a lock: the target stays within ±10° of the boresight and in range (250) for 1.5 s; then fire and forget |

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
| Lock-on diamond | on the selected target: closes in while the lock builds, then a small diamond and `LOCK` |
| Launch cue | `MSL` flashing when a SAM is in the air toward you |

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
| P4 | Weapons, hits, damage, explosions, the SAM warning. |
| P5 | Missions: the tables, objectives, briefing and debrief, landing, fuel. |
| P6 | Balance, sound effects, a full campaign play-test on the C64U. |
