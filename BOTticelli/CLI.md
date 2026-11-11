# CLI reference

Run from `BOTticelli/` with the virtual environment active. Commands use
`python -m botticelli COMMAND [OPTIONS]`. Every command and subcommand supports
`-h` and `--help`, with usage, arguments, defaults and an example.

```bash
python -m botticelli --help
python -m botticelli drive -h
python -m botticelli hardware servo angle --help
python -m botticelli hardware sensor imu configure -h
```

## Commands

| Command | Purpose |
|---|---|
| `hardware --host IP shell` | Register once and test components interactively |
| `drive --backend sim` | Practice gamepad driving offline |
| `drive --backend real` | Drive the K10 using calibrated `config/bot.yaml` |
| `gamepad-check` | Inspect buttons and axes offline |
| `battle` | Play a simulated balloon battle; see [controls](BATTLE.md) |
| `replay FILE.jsonl` | View a recording offline; `--headless` validates it |

`drive` and `battle` accept `--config FILE`, `--record FILE.jsonl`,
`--seconds N` and `--screenshot FILE.png`. `--headless` requires `--seconds`;
headless driving is simulation-only. Recordings never overwrite existing files.
Use each command's help for complete options.

## Hardware shell

```bash
python -m botticelli hardware --host YOUR_BOT_IP shell
```

The token is prompted privately before the first shell prompt. Inside the shell:

```text
help
help servo angle
servo angle -h
sensor imu read
exit
```

Hardware commands also work directly. Put connection options **before** the
component command; use `--dry-run` to validate without connecting:

```bash
python -m botticelli hardware --host YOUR_BOT_IP sensor imu read
python -m botticelli hardware --dry-run servo angle S0 45 --type 180 --home 90
```

Connection options: `--host IP`, `--port N` (24642), `--http-port N` (80),
`--timeout SECONDS` (0.25), `--geomag-rate-format u16be|u8` (u16be).
Live reads and sound register too. `capabilities`, help and dry-run stay offline.
See [hardware setup and cleanup](HARDWARE.md) before motion.

## Hardware commands

The examples below are shell commands; outside the shell, prefix them with
`python -m botticelli hardware --host YOUR_BOT_IP`.

| Group | Examples / actions |
|---|---|
| Diagnostics | `capabilities`, `ping`, `name`, `battery` |
| Positional servo | `servo angle S0 45 --type 180 --home 90` |
| Relative angle | `servo increment S0 10 --type 180 --home 90` |
| Servo type | `servo type S0 180 --home 90`; `servo type S5 continuous` |
| Continuous servo | `servo speed S5 15 --seconds 1` |
| Servo readback | `servo read S0 S5` |
| DC motors | `motor speed M1 -15 --seconds 1`; `motor read M1 M4` |
| Stop | `stop`; `stop --continuous S0 S5` |
| LEDs | `led set L0 L4 --rgb 255 0 0`; `led read L0`; `led off L0` |
| Board LEDs | `board-led set 0 --rgb 0 255 0`; `board-led read`; `board-led off 0` |
| Display | `screen app-info`; `screen next`; `screen previous` |
| Sensors | `sensor imu read`; `sensor all watch --seconds 5` |
| IMU settings | `sensor imu config`, `sensor imu configure -h`, `sensor imu reset` |
| Bump settings | `sensor bump config`, `sensor bump configure -h` |
| HuskyLens | `sensor camera set-algorithm color`; `sensor camera --help` for algorithm, controls and illumination |
| Sound | `sound list`, `sound status`, `sound play beep.wav`, `sound stop` |
| Sound files | `sound upload beep.wav ./beep.wav`; `sound delete beep.wav` |
| Volume | `sound volume 25` |

Servo speed is percent (-100..100); positional targets and `--home` use degrees.
Use `--min-angle` / `--max-angle` for safe linkage endpoints. Both target and
home must be within limits. Only configured, explicitly selected servos are homed.
Timed motion defaults to one second (maximum 60). Readback is not physical feedback.

Sensor names: `lidar`, `intensity`, `imu`, `compass`, `camera`, `bump`, `all`.
All support `watch`; all except `bump` and `all` support `read`.
Streamed reads/watch accept `--seconds` and `--hz`. Reads show exact numbered
ASCII grids; watches use compact ASCII maps. Add `--full` for exact values while
watching. All 512 cells retain their original row/column order; narrow terminals
split columns into labeled blocks. Units are mm, raw intensity, mg,
degrees/microteslas and HuskyLens detections.
No bump events is normal. Sound uploads require PCM WAV files no larger than 300 KiB.

Help and successful commands return 0; parser errors return 2; validation and
operational errors return 1. Results are terminal tables, never JSON. Sensor
watches refresh in this terminal at up to 4 fps; oversized views and pipes use
complete plain-text snapshots. IDs, device timestamps, units, age and stale state
accompany readings. Errors go to stderr. Ctrl+C or `exit` runs shell cleanup.
