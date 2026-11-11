# CLI reference

Run from `BOTticelli/` with the virtual environment active. Commands use
`python -m botticelli COMMAND [OPTIONS]`. Every command and subcommand supports
`-h` and `--help`, with DESCRIPTION, ARGUMENTS, OPTIONS and a generic EXAMPLE.
UPPERCASE names are placeholders; `[brackets]` mark optional arguments. Replace
placeholders and omit the brackets when entering a command. Required arguments
are labelled; defaults and choices appear under OPTIONS. Group help also lists
child command syntax. CLI help includes connection options and where to put them.

```bash
python -m botticelli --help
python -m botticelli drive -h
python -m botticelli hardware servo positional angle --help
python -m botticelli hardware sensor imu configure -h
```

## Commands

| Command | Device / environment | Purpose |
|---|---|---|
| `hardware --host HOST shell` | UniHiker K10 and connected devices below | Register once and test interactively |
| `drive` | IR-SIM (default backend) | Practice gamepad driving offline |
| `drive --backend real` | UniHiker K10 / DFR1216 | Drive using calibrated `config/bot.yaml` |
| `gamepad-check` | Connected gamepad; detected name shown in the window | Inspect buttons and axes offline |
| `battle` | Offline simulation | Balloon battle; see [controls](BATTLE.md) |
| `replay FILE.jsonl` | Recorded devices; no hardware connection | View a recording; `--headless` validates it |

`drive` and `battle` accept `--config FILE`, `--record FILE.jsonl`,
`--seconds SECONDS` and `--screenshot FILE`. `--headless` requires `--seconds`;
headless driving is simulation-only. Recordings never overwrite existing files.
Use each command's help for complete options.

## Hardware shell

```bash
python -m botticelli hardware --host HOST shell
```

The token is prompted privately before the first shell prompt. Inside the shell:

```text
help
help servo positional angle
servo positional angle -h
servo positional angle CHANNEL ANGLE [--seconds SECONDS]
servo continuous speed CHANNEL PERCENT [--seconds SECONDS]
dc-motor speed CHANNEL PERCENT [--seconds SECONDS]
sensor imu read
sensor imu bump watch [--seconds SECONDS]
sensor lidar intensity read
exit
```

Hardware commands also work directly. Put connection options **before** the
component command; use `--dry-run` to validate without connecting:

```bash
python -m botticelli hardware --host HOST sensor imu read
python -m botticelli hardware --dry-run servo positional angle CHANNEL ANGLE [--seconds SECONDS]
```

Connection options: `--host HOST`, `--port PORT` (24642), `--http-port PORT` (80),
`--timeout SECONDS` (1), `--geomag-rate-format u16be|u8` (u16be).
Live reads and sound register too. `capabilities`, help and dry-run stay offline.
See [hardware setup and cleanup](HARDWARE.md) before motion.
Reply timeout, motion duration (`--seconds`) and the 25 ms heartbeat are separate.
Timeouts include UDP receive/ignored-packet counts; timed-out motion is never retried.

## Hardware commands

The examples below are shell commands; outside the shell, prefix them with
`python -m botticelli hardware --host HOST`.

| Group | Physical device | Operations / example |
|---|---|---|
| Diagnostics | UniHiker K10; battery via DFR1216 | `capabilities`, `ping`, `name`, `battery` |
| `servo positional` | DFRobot DFR1216 outputs S0..S5 | angle, increment, configure, read; `servo positional angle CHANNEL ANGLE [--seconds SECONDS]` |
| `servo continuous` | DFRobot DFR1216 outputs S0..S5 | speed, configure; `servo continuous speed CHANNEL PERCENT [--seconds SECONDS]` |
| `dc-motor` | DFRobot DFR1216 DC outputs M1..M4 | speed, read; `dc-motor speed CHANNEL PERCENT [--seconds SECONDS]` |
| `stop` | DFRobot DFR1216 actuators | `stop [--continuous CHANNEL ...]` |
| `led` | K10 NeoPixels and DFR1216 WS2812 LEDs | set, read, off; `led set CHANNEL --rgb R G B`; board service: `led board read` |
| `screen` | UniHiker K10 TFT | `screen SCREEN` |
| `sensor lidar` | DFRobot SEN0682 64x8 DTOF LiDAR | read, watch; intensity: `sensor lidar intensity read` or `sensor lidar intensity watch` |
| `sensor imu` | DFRobot SEN0409 LIS2DW12 accelerometer | read, watch, config, configure, reset; impacts: `sensor imu bump watch`, `sensor imu bump config`, `sensor imu bump configure -h` |
| `sensor compass` | DFRobot SEN0619 BMM350 magnetometer | `sensor compass read`; `sensor compass watch` |
| `sensor huskylens` | DFRobot SEN0305 HuskyLens K210 | read, watch, algorithm, set-algorithm, controls, illumination; `sensor huskylens set-algorithm ALGORITHM` |
| `sensor all` | All four sensors above | `sensor all watch [--seconds SECONDS]` includes distance, intensity, acceleration and impacts |
| `sound` | UniHiker K10 speaker | list, status, play, stop, upload, delete, volume; `sound play FILENAME`; `sound upload FILENAME PATH`; `sound volume PERCENT` |

Models follow the [sensor reference](../ai-sensors.md). Servo and motor models
depend on your build; DFR1216 is their controller. LED addresses L0..L2 are K10
NeoPixels; L3/L4 are the two DFR1216 WS2812 LEDs. `led board` uses separate
board-service indexes 0..2, with set/read/off operations.

Positional servos default to **270 degrees, home 0**, with limits 0..270.
Only CHANNEL and ANGLE/DELTA are required for movement; `configure` needs only CHANNEL.
Override with `--type 180`, `--home N`, `--min-angle N` or `--max-angle N` per command.
Target and home must stay within limits. Only selected positional channels are homed.
Continuous servos use S0..S5; DC motors use M1..M4. Both speed commands need CHANNEL
and PERCENT (-100..100), default to one second, then stop; `--seconds SECONDS` changes
the duration (maximum 60). Continuous `configure` needs only CHANNEL.
Readback is commanded state, not physical feedback.
`command held s` reports the client wait time, not measured physical movement.

Each sensor has one group. `sensor imu read/watch` gives XYZ acceleration (mg);
`sensor imu bump watch` gives impact events from the same LIS2DW12 and stream.
No bump events is normal. `sensor lidar read/watch` gives distance (mm);
`sensor lidar intensity read/watch` gives signal strength (raw) from the same SEN0682.
Compass readings contain heading (degrees) and magnetic field (microteslas).
HuskyLens returns detections, not camera images. All views support `watch`;
impact events and `all` have no `read` operation.
Streamed reads/watch accept `--seconds` and `--hz`. LiDAR uses a fixed color
heatmap in color terminals, with the closest in-range distance in metres/mm.
Use `--cell ROW COL` to inspect a row/column, or `--full` for all exact values.
Pipes and `NO_COLOR=1` use ASCII. All 512 cells retain wire order; narrow terminals
split columns into labeled blocks. Sound uploads require PCM WAV files no larger
than 300 KiB.

Help and successful commands return 0; parser errors return 2; validation and
operational errors return 1. Results are terminal tables, never JSON. Sensor
watches refresh in this terminal at up to 4 fps; oversized views and pipes use
complete plain-text snapshots. IDs, device timestamps, units, age and stale state
accompany readings. Errors go to stderr. Ctrl+C or `exit` runs shell cleanup.
Cleanup errors identify the failed stop/home/stream/unregister step, channel,
action code and timeout or firmware status; confirmed later stops clear earlier stop errors.

In the graphical driving/replay dashboard, hover over a LiDAR cell to highlight
it and read its row, column and exact range. Gray means no return; pink X means
out of range. Stale frames are dimmed. Grid coordinates are not world coordinates.
