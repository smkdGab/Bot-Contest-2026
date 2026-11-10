# Balloon battle practice

Run from `BOTticelli/` with the existing virtual environment active:

```bash
python -m botticelli battle
python -m botticelli battle --config config/battle.yaml --opponents 5 --seed 42
python -m botticelli battle --record recordings/battle.jsonl
python -m botticelli battle --opponents 8 --seed 42 --headless --seconds 30
python -m botticelli replay recordings/battle.jsonl
python -m botticelli replay recordings/battle.jsonl --headless
```

Battle is simulation-only. It never connects to a bot. Manual `drive`,
`gamepad-check`, hardware operation and schema 1 recording playback retain their
existing commands and configuration. Use the separate [battle configuration](config/battle.yaml).

## Controls and rules

Release the driving keys and gamepad A, then **Enter** starts/resumes the entire
round. Drive car 0 with **WASD or the arrow keys**: W/Up forward, S/Down reverse,
A/Left steer left, D/Right steer right. No gamepad or enable key is required for
keyboard play. Releasing the driving keys stops your car; opponents continue.
You can also hold gamepad **A** and use the existing stick/D-pad controls. Pressing
a driving key selects the keyboard; release those keys, then release and press
gamepad A to select the gamepad again. Inputs are never added together. **Space** freezes the entire arena, including sensor
clocks, damage and policy timers. Losing window focus, losing/replacing the active gamepad, stale input, or a
delayed UI frame pauses the round. Release the driving keys and gamepad A, then
press Enter to rearm; reconnection or held controls cannot restart it. Keyboard
play remains available when no gamepad is connected. After gamepad loss, Enter
can deliberately switch to keyboard play. An idle gamepad disconnect does not
interrupt active keyboard play. If your car is eliminated, you can
watch the remaining policies; Enter also allows spectating without a controller.
**R** generates a new random round. **Shift+R** resets the exact resolved setup,
including sensor seeds. Both restart paused. Close the window or Ctrl+C exits.

By default there are 1–5 opponents, one balloon per car, and 1–3 fixed toothpicks
per car. **Multiple toothpicks are an unconfirmed practice setting.** The
[participant guide](../bot-contest-guide.md#mounting-the-toothpick) does not set a
maximum; an older kit list mentions one. Set `car.toothpick_count: 1` to practice
with one. These simulation settings do not establish contest equipment policy.
Contest timing is deferred: the game ends when at most one car survives.

A foreign toothpick's **endpoint** touching a balloon pops it. Shafts, self hits,
and ordinary balloon contact with walls or chassis do not pop it. Balloons are
spheres, tips are sharp points, and toothpicks are rigid segments. Height matters;
walls, obstacles and car bodies shield balloons from a segment passing through
them. There is no bending, deformation, puncture-force or toothpick damage model.

Every car starts at 100 health. A chassis contact also damages a car according to
its attempted contact-point velocity toward the surface, including turning:

```text
damage = 50 × (max(0, closing_speed − 0.10) / 0.50)²
```

Speeds are m/s. Both cars receive the same severity in a car-to-car impact.
Walls, obstacles and toothpicks do not take chassis damage. Damage occurs once
per contact episode; the pair must separate by at least 2 cm to rearm. Pressing
continuously against an obstacle cannot drain health each frame. Health never
repairs. This is an adjustable game approximation, **not a prediction of LEGO
breakage**. Health, the speed threshold, reference speed, multiplier and separation
are configurable under `damage`.

Popped or destroyed cars stop and remain grey solid obstacles. A red X marks a
destroyed chassis. Health and elimination causes remain visible. Damage and pops
are independent; both are recorded when they occur in the same update.
Eliminations are applied together after each update. Last surviving car wins;
no survivors means a draw.

## Configuration and units

All distances are metres, speeds m/s, and configured angles degrees. Runtime and
recorded pose yaw uses radians. Local x is forward, y left and z up, relative to
the rear axle. The body extends `(length − wheelbase)/2` behind the axle and the
remaining length ahead of it. Positive steering turns left.

`car` supplies shared defaults. `cars` maps stable IDs (`'0'` is the player,
`'1'` onward are opponents) to recursive overrides. Each car supports dimensions,
wheelbase, drive limits, sensors, one `balloon`, `toothpick_count`, a `toothpick`
template or explicit `toothpicks` list, and optional `pose: [x, y, yaw_degrees]`.
An explicit list owns its count; a conflicting explicit integer count is rejected.
Attachment `position: [x, y, z]` is relative to the rear axle. Without an explicit
position, `mount` selects `perimeter` (default), `front`, `rear`, `left`, or `right`,
with a random point along the selected edge. Perimeter selection is uniform by
edge length. Toothpicks point outward with `spread_deg` of angular variation
(default 30°); an explicit `direction_deg` remains relative to forward and
overrides that generated direction. Explicit positions are never randomized. A toothpick also has
`length`, `direction_deg` (horizontal, relative to forward) and `elevation_deg`.
Use exact numeric values or `{min: ..., max: ...}` for randomized numeric fields.
Counts use integer ranges. Explicit values stay fixed; unspecified mounts use
the defaults below. The **white arrow, white front bumper and FRONT label** mark
the head of each car, regardless of where its balloon or toothpicks are attached.

| Setting | Default |
|---|---|
| Opponents | Integer 1–5; `--opponents N` overrides config (1–32 supported) |
| Body length × width × height | 0.30 × 0.20 × 0.20 m |
| Wheelbase | 0.20 m |
| Maximum speed × speed limit | 0.60 m/s × 1.0 |
| Maximum steering | 30° |
| Balloon radius | 0.06–0.08 m |
| Balloon mount | Random edge and location around the full chassis perimeter; radius plus 0.02 m clearance; height 0.28 m |
| Toothpicks | Integer 1–3; independently sampled points around the full chassis perimeter |
| Toothpick length/direction | 0.08–0.12 m; outward from the chosen edge ±30°; elevation 0° |
| Toothpick mount height | 0.28 m |
| Arena outer width/height | Independently 4–6 m |
| Arena shape weights | Equal rectangle, irregular convex polygon, L shape |
| L passage minimum | 1 m |
| Wall height/thickness | 0.35 / 0.05 m; thickness extends outward |
| Placement clearance | 0.02 m from walls, other bodies and initial balloon-tip hits |
| Generation attempts | 2,000 per random car start; no automatic count reduction |

`arena.shapes` maps `rectangle`, `convex`, and `l` to nonnegative selection weights.
Set unused shapes to zero. `arena.polygon: [[x,y], ...]` replaces random generation
with one simple connected boundary without holes. L shapes are orthogonal; their
horizontal and vertical passages both meet `passage_min`. Obstacles are static
boxes with `x`, `y` (center), `length`, `width`, `height`. Wall inner faces coincide
with the polygon used for drawing, movement, sensor intersections and shielding.
Malformed boundaries, floor intersections, blocked initial toothpicks, overlapping
starts and initial balloon punctures are rejected. Explicit starts are placed
first. An overcrowded setup fails clearly after bounded attempts.

## Policies, sensing and reproducibility

Each car has its own `HuntingPolicy.reset()` and
`step(observation, time) -> DriveCommand`. Policies receive immutable observations:
seven forward distance rays (right to left), camera balloon bearing, angular
radius and elevation, timestamp/freshness, and contact feedback. No exact world
positions, target IDs, hidden balloons or opponents' health enter the interface.
Own toothpick calibration is supplied at initialization. All actions use the same
starting observation snapshot before the world advances at 50 Hz.

Each car senses at 8 Hz by default. Distance rays span 120°, at 0.15 m height;
the estimated 60° camera is at 0.28 m, both 0.20 m forward of the rear axle. The
camera has a 60° vertical field. Range defaults to 5 m. Height-bearing car bodies,
walls and obstacles occlude observations. Distance noise and whole sensor-frame
loss can be configured independently for each car. Distances below the configured
minimum conservatively report that minimum. These are simplified synthetic
instruments, not calibrated hardware or the manual dashboard's full 512-ray scan.

The initial policy searches clear space, approaches visible balloons, aligns its
least angled toothpick, and prioritizes avoiding nearby obstacles. It forgets
unseen targets after 0.5 s. At most two 0.75 s reverse recovery attempts are allowed
before waiting for space; a clear forward reading resets recovery. Missing/stale
required sensing stops the policy. Recovery has no rear sensor and is bounded,
not a navigation guarantee. Policies can become stuck or leave several survivors;
headless runs end at the duration limit without inventing a winner.

The terminal and window show the seed; an omitted seed is generated. Arena,
attachments, placement, and each car's sensors use independent random streams.
Schema 2 JSONL records contain the **complete resolved setup**, per-step commands
and world snapshots, contact speed/location, damage, pop attacker/victim/toothpick
and position/time, eliminations, pauses, round resets and results. A bounded run
also records an incomplete duration-limit result. Replay draws recorded snapshots
instead of regenerating the world, so generation changes cannot alter old rounds.
It remains offline and retains schema 1 playback. Recording uses exclusive file
creation and the existing bounded writer; writer failures stop the session.

Movement uses the pinned IR-SIM Ackermann integration, with simultaneous contact
rejection and conservative swept bodies. Adaptive substeps limit maximum body or
attachment travel to 5 mm; relative linear tip/sphere intersections are solved
within each substep. Curved attachment motion is approximated by those short
chords. Contact normals use nearby surfaces, not world-center directions.
There is no inertia, friction, bouncing, impulse transfer, chassis deformation,
servo lag or calibrated tyre/drivetrain model. Cars may conservatively stop just
short of contact and can reverse away immediately. Attachments do not constrain
chassis movement or deform against walls; occluded segments cannot score.

## Verification

`python -m pytest` includes the original manual-driving, protocol-loopback,
cleanup, sensors and schema 1 replay tests plus battle generation, shapes and
inward corners, sensor-only hunting, swept tip hits and height/shaft misses,
shielding, crash thresholds, continuous/rearmed contact, simultaneous elimination,
stationary wrecks, pause/rearm, rendering, recording and network-free replay.
The headless command controls every car by policy and requires `--seconds` in
(0, 3600]. Windowed runs accept the same duration bound and optional
`--screenshot PATH` for display checks.

Validation in this environment included deterministic 30-second runs with one,
five and eight opponents, schema 2 CLI recording/replay, and Xvfb inspection of
all three arena families with nine cars. Synthetic input covers keyboard and
controller lifecycle behavior. A physical gamepad has not been exercised.
