# K10 hardware test CLI

Run from `BOTticelli/` with the existing virtual environment active. This bench
uses the documented K10 transport and operates independently of the driving
configuration: it does not assume that any servo is a wheel or steering servo.

Start an interactive session, using the IP shown by your bot:

```bash
python -m botticelli hardware --host YOUR_BOT_IP shell
```

Type `help` or `servo --help`, `sensor --help`, etc. Type `exit` to finish.
The first protected command privately prompts for the five-character TFT token;
it stays registered for subsequent tests. No history file is written. Do not
run another master client, SensorsScreen, or hardware driver concurrently.

One-shot commands work outside the shell too:

```bash
python -m botticelli hardware --host YOUR_BOT_IP servo angle S3 90 --type 180
python -m botticelli hardware --host YOUR_BOT_IP sensor imu read
python -m botticelli hardware --dry-run servo angle S3 90 --type 180
python -m botticelli hardware capabilities
```

`--dry-run` validates and prints the operations without opening sockets or asking
for a token. `--host`, `--port` (default 24642), `--http-port` (default 80),
`--timeout` and `--geomag-rate-format` go **before** the component command.
Live mode has no default bot address. Output is JSON; watched readings are one
JSON object per fresh frame, suitable for piping into a file or `jq`.

## Servos S0–S5

The following commands are entered at `k10>`; omit the prompt itself:

```text
servo angle S0 90 --type 180
servo angle S5 210 --type 270
servo angle S2 45 --type 180 --min-angle 30 --max-angle 120
servo increment S2 10 --type 180 --min-angle 30 --max-angle 120
servo increment S2 -10 --type 180 --min-angle 30 --max-angle 120
servo read S0 S1 S2 S3 S4 S5
servo type S4 continuous
servo speed S4 15 --seconds 1
servo speed S4 -15 --seconds 0.5
servo speed S4 0
```

`angle` specifies an **absolute angle in degrees**, not a duration. `increment`
specifies a signed number of degrees relative to the **reported commanded angle**.
It reads that value and rejects a target outside your safe limits, then sends
one absolute angle command. Readback is not physical encoder feedback.

Always choose the actual positional type with `--type 180` or `--type 270`.
The type is configured before motion. Default safe limits are 0 through the
selected type's maximum; use narrower `--min-angle` / `--max-angle` limits for
linkages. `--seconds` on positional tests keeps the session active for movement
(default 1 s); it does not request a speed or release positional holding afterward.

Continuous-servo speed is signed percent, −100 through +100. It is automatically
configured as continuous, zeroed, run once, then stopped after `--seconds`
(default 1 s, maximum 60 s). A timed test stops before printing its result, so
slow or blocked console output cannot extend the run. It also stops on Ctrl+C
and failures. No automatic sweep or six-channel motion test is performed.

Before applying power or movement, follow the repository's
[wiring](../ai-wiring.md) and [mandatory checklist](../ai-mandatory-checklist.md).
Keep the mechanism unloaded and clear, and provide a physical means to cut power.
A requested type or angle is not proof that it matches the attached mechanism.

## DC motors and stopping

DC motors use **M1–M4**, unlike the zero-based S0–S5 servo labels:

```text
motor speed M1 15 --seconds 1
motor speed M4 -10 --seconds 0.5
motor read M1 M2 M3 M4
stop
stop --continuous S0 S4 S5
```

`stop` sends the documented DC stop command. Add only known continuous channels
to zero those servos individually. The shell also remembers continuous servos
configured during its own session and zeros them when a timed motion test ends
and during cleanup. Cleanup does not discover or assume other clients' servo types.

**Network stopping is not a physical guarantee.** Conservatively, heartbeat
timeout and `STOP_ALL_MOTORS` stop DC motors only; neither releases master
ownership. Cleanup individually zeros configured continuous servos, best-effort
stops DC motors, disables enabled streams, then unregisters. It attempts every
cleanup step even when earlier steps fail and reports unconfirmed cleanup.
Positional holding is preserved.

## Sensor readings and monitoring

```text
sensor imu read
sensor imu watch --seconds 10 --hz 20
sensor lidar read
sensor lidar watch --seconds 10 --hz 8
sensor intensity read --full
sensor intensity watch --seconds 10 --hz 8 --full
sensor compass read
sensor compass watch --seconds 10 --hz 10
sensor camera read
sensor camera watch --seconds 10 --hz 8
sensor bump watch --seconds 10 --hz 20
sensor all watch --seconds 10 --hz 8
```

IMU readings are in **mg** (including approximately 1000 mg due to gravity on a
stationary, level sensor). Compass headings are degrees and magnetic fields µT.
LiDAR distances are millimetres in the firmware's original row-major order.
Intensity values remain raw firmware units. Camera readings are HuskyLens learned
blocks/arrows, not photographs or general camera video. Frames include timestamps
or sample IDs when supplied by the device and local receipt age.

LiDAR `read` queries the full 64×8 distance matrix. IMU `read` queries acceleration.
Compass, camera and intensity `read` briefly enable their stream and return the
first fresh frame; they are disabled afterward. `watch` duration defaults to
5 s; stream-based `read` defaults to a 2 s deadline. All durations must be in
(0, 60]. Missing data is reported as a timeout, not repeated as a fresh sample.
A bump watch with no impact reports zero events, which is a normal result.

Watched LiDAR/intensity frames show nonzero count, minimum, maximum and mean by
default. Add `--full` for all 512 values. Streams share a single UDP receiver and
are routed separately from acknowledgments. Monitoring displays latest frames;
it is a component test console, not a lossless packet recorder.

Sensor prerequisites and limitations are described in [ai-sensors.md](../ai-sensors.md).
LiDAR requires the documented UART and **5 V** wiring. Compass calibration must
be performed through a supported device interface after assembly. HuskyLens
training is performed **on the sensor**.

## IMU setup and HuskyLens controls

```text
sensor imu config
sensor imu configure --range-code 1 --rate-hz 100 --power-code 0 --hpf-code 1
sensor imu reset
sensor bump config
sensor bump configure --threshold-mg 500 --debounce-ms 100
sensor camera algorithm
sensor camera set-algorithm color
sensor camera set-algorithm tag
sensor camera illumination on
sensor camera illumination off
sensor camera controls
```

IMU range, power and high-pass fields are explicitly **raw firmware codes**.
The example values are illustrative wire values, not a recommended calibration;
read the current configuration and use values supported by your device. The
firmware may reject unsupported values. `reset` resets IMU configuration, not the bot.
Bump thresholds use mg; debounce uses milliseconds. Do not provoke a damaging
collision just to obtain a bump event.

Camera algorithms are `face`, `tracking`, `recognition`, `line`, `color`, and `tag`.
`controls` reports supported controls. HuskyLens V1 RGB/status-light and LCD controls
are known to report unsupported and are not offered as functioning test setters.
Illumination uses the existing SensorsScreen helper's documented 0x71 command.

## LEDs, screen, battery and connection

```text
ping
name
battery
led set L0 L1 L2 --rgb 255 0 0 --brightness 64
led set L3 L4 --rgb 0 0 255 --brightness 64
led read L0 L1 L2 L3 L4
led off L0 L1 L2 L3 L4
board-led set 0 --rgb 0 255 0 --brightness 64
board-led read
board-led off 0
screen app-info
screen huskylens
screen next
screen previous
```

The unified LED service uses L0–L4 (L0–L2 K10, L3–L4 expansion-board LEDs).
The separate board LED service uses indices 0–2, as documented by that service;
these are distinct protocol interfaces, not an inferred shared numbering scheme.
RGB and brightness are raw 0–255 bytes; brightness defaults to 64.
Screen names are `splash`, `app-info`, `huskylens`, `app-log`, `svc-log`, `debug-log`,
and `esp-log`. Ping identifiers are checked; ping has no status byte. Battery
is percent. Read-only queries do not register or configure servos.

## Speaker and stored sound files

```text
sound list
sound status
sound upload beep.wav /absolute/path/to/beep.wav
sound volume 25
sound play beep.wav
sound stop
sound delete beep.wav
```

Sound uses the documented HTTP endpoints (default port 80) and needs no token,
master registration or heartbeat in a sound-only session. Playback must be inactive
before file changes. `sound stop` waits for inactive playback for up to 10 s.
Uploads use the raw validated WAV bytes, replace the same filename, and must leave
128 KiB free. Files must be simple ASCII filenames ending in lowercase `.wav`,
at most 48 bytes, with no paths or `..`. Supported files are PCM, mono/stereo,
8/16-bit, 8000–48000 Hz, at most 300 KiB including headers.

A small test file can be converted with the [sound tutorial](../ai-udp-client.md#sound-convert-store-and-play).
Playback acceptance does not prove sound was audible; inspect `sound status`
and listen. During playback, LiDAR, HuskyLens, UI refresh and shock polling pause;
IMU/compass telemetry is best-effort. Sound stop is not a motor stop.

## Known protocol gaps

The full authority is the [protocol reference](../binary-protocol.md), with
existing [SensorsScreen helpers](../python_example/sensorscreen/protocol.py)
used for supported stream layouts. No firmware checkout was available.

The protocol reference defines the compass stream rate as u16 big-endian;
SensorsScreen currently sends one u8. The default follows the protocol reference.
If your firmware requires the helper format, explicitly start with:

```bash
python -m botticelli hardware --host YOUR_BOT_IP --geomag-rate-format u8 shell
```

There is **no automatic fallback or motion retry** after a timeout. Compass
calibration/configuration and detailed LiDAR configuration/statistics payloads
are absent from the available reference, so the bench does not invent them.
Temperature/humidity, ambient light, microphones, GPIO, IR, ultrasonic and onboard
buttons have no documented remote commands in this firmware. `capabilities`
lists these limitations. WiFi credentials are never requested or logged, and
there is no arbitrary binary-command or participant-token CLI argument.

Transport failures end the session; start a new one deliberately. A five-character
token is prompted privately only for protected operations and is not written to
files, history, JSON output or a configuration. Testing several components in the
shell prompts once. Ctrl+C exits through the same cleanup path.

## Verification

The test suite covers loopback registration/cleanup, S0–S5 mapping, angle limits
and increments, timed positive/negative motor commands, heartbeat independence,
wrong senders/actions, malformed packets, diagnostics removal, timeouts without
motion retries, sensor events arriving before acknowledgments, stream cleanup,
JSON readbacks, sound HTTP operations, interruption, and network-free dry-runs.
Physical device compatibility remains to be checked on your hardware. No live
hardware commands were sent during implementation.
