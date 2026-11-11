# BOTticelli

Python control from your computer to the K10 over Wi-Fi, plus offline driving
and balloon-battle practice. CLI tables and live sensor views use
[Rich](https://github.com/Textualize/rich) (MIT); hardware primitives remain standard-library-only.

## Install

From the repository root (Python 3.12):

```bash
git submodule update --init BOTticelli/vendor/ir-sim
cd BOTticelli
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m pip install --no-deps -e .
```

## Start

```bash
python -m botticelli --help
python -m botticelli hardware --host YOUR_BOT_IP shell
python -m botticelli drive --backend sim
python -m botticelli battle
```

For real driving, calibrate [config/bot.yaml](config/bot.yaml), set the host and
confirmation fields, then use `drive --backend real`. Release gamepad A once,
then hold A to drive; release it to stop.

- [CLI reference](CLI.md): commands and examples; every command accepts `-h` / `--help`.
- [Hardware setup and safety](HARDWARE.md).
- [Python primitives](PRIMITIVES.md).
- [Battle controls](BATTLE.md) and [configuration](config/battle.yaml).

Run tests with `python -m pytest`. Tests use mocks and loopback; physical
calibration and device compatibility still require hardware checks.

## Code map

| Files | Purpose |
|---|---|
| `protocol.py` → `primitives.py` | Validated packets, one receiver, heartbeat → small `Robot` operations and session cleanup; standard library only |
| `control.py` | Driving rules, K10 adapter and control worker; hardware calls go through `Robot` |
| `__main__.py`, `hardware.py` | CLI commands and interactive shell |
| `display.py`, `dashboard.py` | Terminal tables, shared distance colors and pygame views |
| `simulation.py`, `battle*.py` | Offline driving and battle practice |
| `config.py`, `recording.py` | Validated settings and record/replay |
