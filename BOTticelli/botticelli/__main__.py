"""CLI entry point. Hardware access is confined to explicit drive/bench commands."""
from dataclasses import replace
import math
from pathlib import Path
import sys
import time

from .primitives import private_token, termination_cleanup, finish_cleanup
from .hardware import Parser, command


@termination_cleanup()
def drive(args):
    from .config import load
    from .control import Input
    from .recording import Recorder
    config = load(args.config)
    config.validate(real=args.backend == "real")
    if args.seconds is not None and (not math.isfinite(args.seconds) or not 0 < args.seconds <= 3600):
        raise ValueError("--seconds must be in (0, 3600]")
    if args.headless and (args.backend != "sim" or args.seconds is None):
        raise ValueError("Headless runs require --backend sim and bounded --seconds")
    if args.screenshot and (args.headless or args.backend != "sim" or args.seconds is None):
        raise ValueError("Screenshot checks require a bounded, windowed simulation")
    if args.backend == "real":
        from .control import K10
        backend = K10(config, private_token())
    else:
        from .simulation import Simulation
        backend = Simulation(config)
    from .control import Driver
    recorder = Recorder(args.record, config, args.backend) if args.record else None
    dashboard = None
    driver = Driver(backend, config, recorder)
    try:
        if not args.headless:
            import pygame
            from .dashboard import Dashboard, GamepadReader
            dashboard = Dashboard(config, f"BOTticelli / {args.backend.upper()}")
            gamepad = GamepadReader(config.gamepad)
        driver.start()
        start = None
        next_poll = next_render = time.monotonic()
        while start is None or args.seconds is None or time.monotonic()-start < args.seconds:
            now = time.monotonic()
            if start is None and driver.connected.is_set():
                start = now
            if args.headless:
                driver.submit(Input(now))
            else:
                events = pygame.event.get()
                if any(event.type == pygame.QUIT for event in events):
                    break
                driver.submit(gamepad.poll(events))
                if now >= next_render:
                    dashboard.draw(*driver.snapshot(), source=args.backend.upper(), controller=gamepad.message)
                    next_render = max(next_render + (0.02 if args.backend == "sim" else 0.05), now)
            if driver.error is not None:
                raise driver.error
            next_poll = max(next_poll + 0.02, time.monotonic())
            time.sleep(max(0, next_poll-time.monotonic()))
        if dashboard is not None and args.screenshot:
            dashboard.save(args.screenshot)
    finally:
        try:
            with finish_cleanup():
                if driver.thread.ident is not None:
                    driver.close()
                else:
                    backend.close()
        finally:
            for error in getattr(backend, "cleanup_errors", []):
                print(f"Cleanup not confirmed: {error}", file=sys.stderr)
            if dashboard:
                dashboard.close()
            if recorder:
                recorder.close()
    if driver.error is not None:
        raise driver.error
    from .display import output
    output(dict(test='drive', backend=args.backend, state=driver.state, recording=args.record))


def replay(args):
    from .config import from_dict
    from .control import DriveCommand
    from .recording import lidar_from_dict, observation_from_dict, read
    records = iter(read(args.path))
    metadata = next(records)
    if metadata.get("type") == "metadata" and metadata.get("schema") == 2 and metadata.get("backend") == "battle":
        from .battle_runtime import replay_battle
        return replay_battle(args, metadata, records)
    if metadata.get("type") != "metadata" or metadata.get("schema") != 1:
        raise ValueError("Expected BOTticelli recording schema 1 metadata")
    config = from_dict(metadata["config"])
    if metadata.get("backend") not in ("sim", "real"):
        raise ValueError("Unknown recorded backend")
    if args.headless:
        count = 0
        for record in records:
            if record["type"] == "observation":
                observation_from_dict(record["data"])
            elif record["type"] == "lidar":
                lidar_from_dict(record["data"])
            count += 1
        from .display import output
        output(dict(test='offline replay', records=count, backend=metadata['backend'], python=metadata['python']))
        return
    import pygame
    from .dashboard import Dashboard
    from .control import Observation
    dashboard = Dashboard(config, "BOTticelli / OFFLINE REPLAY")
    command, observation, state = DriveCommand(), Observation(), "Replay"
    start = time.monotonic()
    try:
        for record in records:
            elapsed = record["time"]
            while time.monotonic()-start < elapsed:
                if any(e.type == pygame.QUIT for e in pygame.event.get()):
                    return
                virtual_now = metadata["monotonic_origin"] + time.monotonic()-start
                dashboard.draw(observation, command, state, virtual_now, "OFFLINE REPLAY")
                time.sleep(0.02)
            if record["type"] == "command":
                command = DriveCommand(**record["data"])
            elif record["type"] == "state":
                state = record["data"]
            elif record["type"] == "observation":
                observation = replace(observation_from_dict(record["data"]), lidar=observation.lidar)
                if metadata["backend"] != "sim":
                    observation = replace(observation, pose=None, opponent=None)
            elif record["type"] == "lidar":
                observation = replace(observation, lidar=lidar_from_dict(record["data"]))
    finally:
        dashboard.close()


def gamepad_check(args):
    from .config import load
    import pygame
    from .dashboard import GamepadReader
    from .dashboard import CheckWindow
    from .display import Watch
    config = load(args.config)
    dashboard = CheckWindow(config.gamepad)
    reader = GamepadReader(config.gamepad)
    view = Watch()
    next_draw = 0
    try:
        while True:
            events = pygame.event.get()
            if any(e.type == pygame.QUIT for e in events):
                return
            sample = reader.poll(events)
            details = dict(test='gamepad', state='No controller')
            if reader.joystick is not None:
                j = reader.joystick
                axes = [round(j.get_axis(i), 2) for i in range(j.get_numaxes())]
                buttons = [i for i in range(j.get_numbuttons()) if j.get_button(i)]
                hats = [j.get_hat(i) for i in range(j.get_numhats())]
                details = dict(test='gamepad', axes={str(i):v for i,v in enumerate(axes)},
                    pressed_buttons=buttons, hats={str(i):v for i,v in enumerate(hats)})
            if time.monotonic() >= next_draw:
                view.update({'gamepad':details})
                next_draw = time.monotonic()+.25
            dashboard.draw(reader, sample)
            time.sleep(0.02)
    finally:
        view.close()
        dashboard.close()


def run_battle(args):
    from .battle_runtime import run_battle as run
    return run(args)


def build_parser():
    parser = Parser(prog='python -m botticelli', description='K10 hardware control and offline driving practice.')
    commands = parser.add_subparsers(dest="command", required=True)
    from .hardware import add_parser
    add_parser(commands)
    default_config = str(Path(__file__).resolve().parent.parent / "config" / "bot.yaml")
    check = command(commands, 'gamepad-check', 'Inspect gamepad input offline; close the window to exit.')
    check.add_argument("--config", default=default_config, help='gamepad configuration YAML')
    check.set_defaults(run=gamepad_check)
    control = command(commands, 'drive', 'Drive with a gamepad: hold A to enable, release A to stop. Real mode prompts for the TFT token.')
    control.add_argument("--backend", choices=("sim", "real"), default='sim', help='simulation or K10 hardware')
    control.add_argument("--config", default=default_config, help='vehicle and hardware configuration YAML')
    control.add_argument("--record", metavar="FILE", help='create a new JSONL recording; existing files are not overwritten')
    control.add_argument("--headless", action="store_true", help='offline simulation without a window; requires --seconds')
    control.add_argument("--seconds", type=float, help="bounded run, at most 3600 seconds")
    control.add_argument("--screenshot", help="save a dashboard image at the end of a bounded simulation")
    control.set_defaults(run=drive)
    battle = command(commands, 'battle', 'Run an offline balloon battle. Enter starts, Space pauses, R creates a new round.')
    battle.add_argument("--config", help="separate battle YAML configuration")
    battle.add_argument("--opponents", type=int, help='number of opponents, 1..32; otherwise use YAML')
    battle.add_argument("--seed", type=int, help='repeat a generated setup; otherwise generate a seed')
    battle.add_argument("--record", metavar="FILE", help='create a new battle JSONL recording')
    battle.add_argument("--headless", action="store_true", help='run all cars by policy without a window; requires --seconds')
    battle.add_argument("--seconds", type=float, help='duration limit in (0, 3600] seconds')
    battle.add_argument("--screenshot", help='save the final window image of a bounded run')
    battle.set_defaults(run=run_battle)
    viewer = command(commands, 'replay', 'View a recording offline; recorded commands are never sent to hardware.')
    viewer.add_argument("path", help='JSONL recording to read')
    viewer.add_argument("--headless", action="store_true", help='validate the recording and print a summary without a window')
    viewer.set_defaults(run=replay)
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    try:
        args.run(args)
    except KeyboardInterrupt:
        print("Interrupted; cleanup attempted (see any errors above).", file=sys.stderr)
    except (OSError, RuntimeError, ValueError, TypeError) as error:
        example = getattr(args, 'example', 'python -m botticelli --help')
        parser.exit(1, f'BOTticelli: {error}\nExample: {example}\n')


if __name__ == "__main__":
    main()
