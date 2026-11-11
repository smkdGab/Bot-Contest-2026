# Balloon battle practice

Simulation only; no K10 connection. Run from `BOTticelli/`:

```bash
python -m botticelli battle --opponents 3 --seed 42
python -m botticelli battle --headless --seconds 30 --record recordings/battle.jsonl
python -m botticelli replay recordings/battle.jsonl
```

## Controls

| Key | Action |
|---|---|
| Enter | Start/resume after releasing driving keys and gamepad A |
| WASD / arrows | Drive car 0; releasing keys stops your car |
| Gamepad A + stick/D-pad | Drive with the gamepad instead |
| Space | Pause the entire arena |
| R | Generate a new round |
| Shift+R | Reset the same resolved setup |
| Ctrl+C / close window | Exit |

Focus loss, stale input or active-controller loss pauses the round. Release
controls and press Enter to resume. Keyboard input and gamepad input are not mixed.

## Rules and configuration

A foreign toothpick tip touching a balloon eliminates that car. Chassis impacts
also reduce health. Wrecks remain solid obstacles; the last survivor wins.
Walls, bodies and obstacles shield balloons. These are practice approximations,
not contest policy or predictions of physical damage.

Edit [config/battle.yaml](config/battle.yaml) for arena shapes, car dimensions,
attachments, sensors and damage. `car` supplies defaults; `cars` overrides by
ID (`'0'` is the player). Distances use metres, speeds m/s and configured angles
degrees; recorded pose yaw uses radians. Multiple toothpicks are an unconfirmed
practice setting; consult the [participant guide](../bot-contest-guide.md).

Opponents use synthetic sensor observations rather than hidden world positions.
The model omits inertia, friction, servo lag and material deformation.
A seed repeats the generated setup; recordings store resolved world snapshots
and replay offline. Headless runs control all cars by policy and require a
duration limit. See `battle --help` and [CLI.md](CLI.md).
