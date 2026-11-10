# BOTticelli

One gamepad controller for a front-steered car with a continuous servo driving
the rear differential. Try the practice arena in IR-SIM, then use the same
logical commands with the K10. SensorsScreen is independent and unchanged.

## Install

From the repository root, initialize the pinned submodule, then use **venv and pip**:

```bash
git submodule update --init BOTticelli/vendor/ir-sim
cd BOTticelli
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m pip install --no-deps -e .
```

IR-SIM is pinned to v2.11.0, revision
`96e3d6454dbb6a2bbedeee18bf77e685f4071d3d`, in `vendor/ir-sim`.
Only its core dependencies are installed; its keyboard, ORCA and video extras
are unnecessary. See the [upstream source](https://github.com/hanruihua/ir-sim/tree/96e3d6454dbb6a2bbedeee18bf77e685f4071d3d).
Python 3.12.3 was tested. The code supports Python 3.14 syntax/runtime interfaces,
but 3.14 was unavailable here and remains untested. Exact installed versions are
in [tested-versions.json](tested-versions.json); new recordings include their
own Python and dependency versions.

## Run

Run these commands from `BOTticelli/`, with the venv active:

```bash
python -m botticelli gamepad-check
python -m botticelli battle --config config/battle.yaml
python -m botticelli drive --backend sim --config config/bot.yaml
python -m botticelli drive --backend real --config config/bot.yaml
python -m botticelli replay recordings/session.jsonl
python -m pytest
```

`gamepad-check` opens a dedicated offline checklist showing only A and the
configured directional controls: left stick and optional D-pad. Press and release A, then
try forward, reverse, left and right. Checks turn green as you complete them.
Live values and terminal diagnostics help configure your controller.
No car or arena is simulated in this view.
No controller is required to view the simulation, but propulsion stays disabled.
Close the window or press Ctrl+C to exit.

Optional recording and bounded simulation:

```bash
python -m botticelli drive --backend sim --config config/bot.yaml --record recordings/session.jsonl
python -m botticelli drive --backend sim --headless --seconds 5 --record recordings/headless.jsonl
python -m botticelli replay recordings/headless.jsonl --headless
```

Headless driving is simulation-only and requires `--seconds` (maximum 3600).
It observes the arena with propulsion stopped. The duration starts after
connection. `--seconds` also bounds windowed sessions. For a dashboard check,
add `--seconds 2 --screenshot /tmp/dashboard.png` to a windowed simulation.
Recordings use exclusive creation, so existing files are never overwritten.

## Hardware component tests

Use `python -m botticelli hardware --host YOUR_BOT_IP shell` to test individual
S0–S5 servos, DC motors, sensors, LEDs, display and sound. For example,
`servo angle S3 90 --type 180` sets one positional servo. See the
[hardware CLI guide](HARDWARE.md) for commands, units, cleanup and protocol limits.
`hardware --dry-run ...` validates commands without contacting hardware.

## Controls and calibration

- Release A after startup or controller loss, then hold **A** to drive.
  You can move the stick before pressing A. Vertical controls forward/reverse;
  horizontal controls steering. A held through a disconnection cannot restart motion.
- Release **A** to stop propulsion. LB and B do not control driving.
- The left stick and D-pad work together with `gamepad.dpad: true` (the default).
  A pressed D-pad direction overrides only its corresponding stick axis. For
  example, D-pad right steers right while stick vertical controls speed.
  Set `dpad: false` to use the stick alone.
- Stops preserve the last steering position. Steering can move while enabled
  with the throttle centered.

The example uses a direct linear stick response, an 8% deadzone, and no throttle
or steering ramps. Simulation runs at 50 Hz; hardware commands remain limited
to 20 Hz. Release A to stop immediately. Optional `response_curve`,
`acceleration_rate` and `steering_rate` settings remain available for tuning;
set rates to 0 for direct response.

An alternative layout sets `gamepad.throttle_mode: triggers`: left stick steers,
RT accelerates forward and LT reverses. Hold A as usual; D-pad override remains
available. The example trigger axes (RT 4, LT 5) match the inspected Zikway SDL
mapping; confirm them with gamepad-check for a different controller.

[config/bot.yaml](config/bot.yaml) is a commented example. Dimensions, speeds,
mount position, servo assignments and limits are **estimates, not calibration**.
Its hardware confirmations are deliberately false and its bot IP is empty.

Before hardware use, confirm the assigned drive and steering channels, continuous
drive and positional steering types, forward direction, steering center, safe
steering endpoints, and speed limit. Follow the
[wiring](../ai-wiring.md), [mandatory safety checklist](../ai-mandatory-checklist.md)
and [protocol reference](../binary-protocol.md). Check the mechanism unloaded
and clear, with a physical means to cut power. Set the five `_confirmed` fields
only after these checks, and set `hardware.host` to the bot IP.

Hardware mode requires a terminal and privately prompts for the TFT token.
There is no token CLI argument, YAML field, or environment variable. Never put
a participant token in source, configuration or recordings. A 25% speed limit
is the starting default; it limits servo command percentage, not measured speed.
`forward_sign` and `steering_sign` describe servo wiring direction. Positive
logical steering means left; `steering_min/center/max` describe servo angles.
Measure wheel angle and velocity on the assembled car to tune the simulation's
`max_steering_deg` and `max_speed`.

## How it works

```text
gamepad -> enable/stop rules -> DriveCommand(throttle, steering), each -1..1
                                        -> shared calibration mapping
                                           -> IR-SIM or K10
```

Both backends expose `connect`, `apply`, `observe`, `stop`, and `close`.
pygame polls input and draws simulation at up to 50 Hz (hardware view: 20 Hz). An independent worker
checks enable rules and input expiry, retains only the newest sample, and
limits hardware actuator updates to 20 Hz (simulation uses 50 Hz). Stops bypass that rate limit.
Rendering and recording never wait for network acknowledgments. A stalled UI
expires its input; a request or heartbeat failure ends the driving session.
Pending motion is discarded, with no motion retries or automatic reconnection.
UDP has no transaction IDs, so delayed ACKs cannot provide delivery guarantees.

K10 registers before configuring only the two assigned servo channels. A
separate thread sends heartbeat every 25 ms, and one receive dispatcher routes
LiDAR events separately from serialized acknowledgments. Replies must match
sender, action, status and exact length after stripping the 8-byte diagnostics.
Input expiry is checked again between steering and propulsion writes.

On normal exit, failures and interruption, cleanup sends speed zero individually
to every continuous servo configured or possibly configured by this client,
then best-effort `STOP_ALL_MOTORS`, disables its LiDAR stream, and unregisters.
Cleanup failures are reported. **Network stopping is not a physical guarantee.**
Treat heartbeat timeout and `STOP_ALL_MOTORS` conservatively as DC-motor stops;
neither releases master ownership. The repository's checklist and protocol
mention broader stops in newer firmware, while AGENTS.md specifies the older
DC-only behavior. No sibling firmware checkout was available to resolve this;
explicit continuous-servo zeros are always sent. See the
[existing UDP client lifecycle](../ai-udp-client.md#core-rules).

## Driver console and real sensors

The resizable driver studio uses a detailed vehicle inset, steerable wheels,
compass dial, bounded acceleration traces, vision detections, battery gauge and
LiDAR grid. Dashboard text is limited to instrument labels, values and status.
The SIM/REAL source remains visible. All simulated readings are recorded and
replay through the same instruments as real data.

Simulation models level-body acceleration from pose changes (including gravity),
contact shock events, magnetic heading and a nominal local magnetic field,
pinhole target detections with center-ray occlusion, and battery energy use.
Vision uses a 60-degree estimated camera field, synthetic learned IDs (opponent 1,
obstacles 2 onward), and ideal confidence. It does not reproduce an image sensor,
trained HuskyLens algorithm, reflectivity, magnetic interference, electrical
voltage sag or measured impact waveforms. IMU updates at the 50 Hz simulation rate; compass, vision
and battery telemetry at 10 Hz. Battery capacity and starting charge are editable;
load is estimated at 2 W idle plus up to 8 W drive. Empty batteries stop the
simulated drivetrain. Motion and battery models are estimates, not calibration.
Ground-truth simulation velocity is displayed; real speed is unavailable without
an exposed measurement. No synthetic value is substituted for a missing real sensor.

With `hardware.sensors_enabled: true`, hardware mode requests the documented
IMU, geomagnetic and HuskyLens streams. The single UDP dispatcher decodes and
validates their events independently of actuator ACKs. Battery is read on
connection and every five seconds while propulsion is stopped. A sensor's
explicit stream rejection appears on its card; transport timeouts still end the
session. All streams requested by this client are disabled during cleanup.
Sensor readings are included in observations and offline replay.

The geomagnetic stream setup differs between the repository's binary protocol
(two-byte rate) and SensorsScreen helper (one-byte rate). BOTticelli follows the
[binary protocol](../binary-protocol.md#service-0x09--geomagservice), reports a
rejection instead of guessing a fallback, and needs the matching firmware
handler to resolve this discrepancy. See [sensor prerequisites](../ai-sensors.md).
Hardware telemetry integration has been verified against loopback packets;
physical sensor availability and this firmware compatibility remain unverified.

The LiDAR grid is a distance instrument: each tile is one ray, not a photograph.
Warm colours indicate near returns, cool colours far returns; grey indicates no
valid return. Lower rays often hit the floor. The shaded fan is nominal coverage;
central-row return points are drawn in the arena. Real grid orientation still
needs physical calibration.

## Simulation and LiDAR limits

The editable 4 x 4 m arena has walls, height-bearing obstacles and one opponent
following a closed repeatable route.
The car starts facing north (screen up). Mint vehicle 01 is your car, orange 02 follows the route, and grey shapes are fixed obstacles. White nose
markers show each car's forward direction. The shaded sector shows the LiDAR's
nominal horizontal field of view and maximum range, clipped to the arena;
it is not an obstacle visibility mask and does not account for occlusion.
The example's simulation-only `arena.drive_speed_scale: 8` makes full stick
0.60 m/s instead of 0.075 m/s. It does not change hardware's 25% actuator limit.
Use a scale of 1 for the unscaled speed estimate.

IR-SIM supplies Ackermann kinematics and
collision detection without opening its own window. A penetrating car step is
rejected, allowing reverse on the next tick instead of latching a stop flag.
The opponent follows a perimeter route, pauses when its swept body would hit
a wall, obstacle or your car, and resumes when the obstruction clears. It does
not plan a detour. LiDAR, vision and rendering all use its accepted position.
This is a kinematic approximation: no tyre slip, inertia, servo lag, differential
losses, linkage geometry, or calibrated drivetrain response.

The SEN0682 approximation casts 64 x 8 rays at 8 Hz, nominally across 120 x 20
degrees, using the configured sensor position, height, yaw and tilt. Rays hit
the floor and solid boxes, including the rotated opponent. Nearest surfaces
occlude farther ones. Simulation returns integer millimetres in the 0.1–5 m
range, or zero for no valid return. Optional seeded Gaussian noise and whole
frame loss apply only to simulation. Beam directions and static bounds are
computed once. There is no optical material, reflectivity, multipath, beam
footprint, or measured SEN0682 distortion model.

Both sources use the same row-major heatmap, metre legend and freshness label.
Simulation assumes row 0 at the top and column 0 on the left, using ray centers
inside the nominal field of view. **Physical sensor orientation is unverified.**
Real values, frame IDs, timestamps and wire ordering are preserved without
flips, clipping or interpolation. Out-of-range cells appear grey; raw values
remain in recordings. Freshness uses local receipt time, so missing frames turn
stale instead of being repeated as fresh. LiDAR does not automatically brake.

JSONL contains non-secret configuration and versions, logical command changes,
state changes, observations, and each new LiDAR frame once. A bounded background
writer keeps disk I/O off the driving loop; failure or backpressure stops the
session. Replay streams the file through an offline viewer, never constructs a
hardware backend, and shows position only for simulation recordings. It is a
viewer, not command playback to a bot.

## Verification status

Python 3.12 tests cover hold/release and reconnect rules, directions and limits,
loopback registration failures, unexpected senders/actions, malformed packets,
diagnostics, timeouts and cleanup; IR-SIM forward/reverse/turns and collision
recovery; LiDAR floor/box intersections, occlusion, ordering, noise and freshness;
bounded worker/recording behavior and replay with sockets disabled. A pygame
dashboard was rendered under Xvfb and visually inspected.

Physical gamepad checks, hardware calibration/driving and Python 3.14 execution
remain **pending**. No live motion commands were sent during implementation.
Randomized balloon battle, crash damage and sensor-only opponents are available through
`python -m botticelli battle`; see the [battle practice guide](BATTLE.md).
Contest timing and other drivetrains remain deferred.
