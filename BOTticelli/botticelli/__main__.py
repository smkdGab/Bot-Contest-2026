"""Command line entry point. Hardware imports and token prompt are drive-only."""
import argparse
from dataclasses import replace
import getpass
import math
from pathlib import Path
import sys
import time
import warnings

from .config import from_dict, load
from .control import DriveCommand, Input
from .recording import Recorder, lidar_from_dict, observation_from_dict, read


def drive(args):
    config = load(args.config)
    config.validate(real=args.backend == "real")
    if args.seconds is not None and (not math.isfinite(args.seconds) or not 0 < args.seconds <= 3600):
        raise ValueError("--seconds must be in (0, 3600]")
    if args.headless and (args.backend != "sim" or args.seconds is None):
        raise ValueError("Headless runs require --backend sim and bounded --seconds")
    if args.screenshot and (args.headless or args.backend != "sim" or args.seconds is None):
        raise ValueError("Screenshot checks require a bounded, windowed simulation")
    if args.backend == "real":
        if not sys.stdin.isatty():
            raise ValueError("Run hardware mode in a terminal for private token entry")
        from .k10 import K10
        with warnings.catch_warnings():
            warnings.simplefilter("error", getpass.GetPassWarning)
            try:
                backend = K10(config, getpass.getpass("Five-character TFT token (hidden): "))
            except getpass.GetPassWarning as error:
                raise ValueError("Private token entry unavailable; use a terminal with echo control") from error
    else:
        from .simulation import Simulation
        backend = Simulation(config)
    from .runtime import Driver
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
            if driver.thread.ident is not None:
                driver.close()
        finally:
            for error in getattr(backend, "cleanup_errors", []):
                print(f"Cleanup not confirmed: {error}", file=sys.stderr)
            if dashboard:
                dashboard.close()
            if recorder:
                recorder.close()
    if driver.error is not None:
        raise driver.error


def replay(args):
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
        print(f"Read {count} records offline ({metadata['backend']}, Python {metadata['python']})")
        return
    import pygame
    from .dashboard import Dashboard
    from .lidar import Observation
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
    import pygame
    from .dashboard import GamepadReader
    from .gamepad_check import CheckWindow
    config = load(args.config)
    dashboard = CheckWindow(config.gamepad)
    reader = GamepadReader(config.gamepad)
    previous = None
    try:
        while True:
            events = pygame.event.get()
            if any(e.type == pygame.QUIT for e in events):
                return
            sample = reader.poll(events)
            details = "No controller"
            if reader.joystick is not None:
                j = reader.joystick
                axes = [round(j.get_axis(i), 2) for i in range(j.get_numaxes())]
                buttons = [i for i in range(j.get_numbuttons()) if j.get_button(i)]
                hats = [j.get_hat(i) for i in range(j.get_numhats())]
                details = f"axes={axes} buttons={buttons} hats={hats}"
            if details != previous:
                print(details, flush=True)
                previous = details
            dashboard.draw(reader, sample)
            time.sleep(0.02)
    finally:
        dashboard.close()


def run_battle(args):
    from .battle_runtime import run_battle as run
    return run(args)


def main():
    parser = argparse.ArgumentParser(description="BOTticelli gamepad car control")
    commands = parser.add_subparsers(dest="command", required=True)
    default_config = str(Path(__file__).resolve().parent.parent / "config" / "bot.yaml")
    check = commands.add_parser("gamepad-check", help="inspect input without connecting to hardware")
    check.add_argument("--config", default=default_config)
    check.set_defaults(run=gamepad_check)
    control = commands.add_parser("drive")
    control.add_argument("--backend", choices=("sim", "real"), required=True)
    control.add_argument("--config", default=default_config)
    control.add_argument("--record", metavar="SESSION.jsonl")
    control.add_argument("--headless", action="store_true")
    control.add_argument("--seconds", type=float, help="bounded run, at most 3600 seconds")
    control.add_argument("--screenshot", help="save a dashboard image at the end of a bounded simulation")
    control.set_defaults(run=drive)
    battle = commands.add_parser("battle", help="simulation-only randomized balloon battle")
    battle.add_argument("--config", help="separate battle YAML configuration")
    battle.add_argument("--opponents", type=int)
    battle.add_argument("--seed", type=int)
    battle.add_argument("--record", metavar="BATTLE.jsonl")
    battle.add_argument("--headless", action="store_true")
    battle.add_argument("--seconds", type=float)
    battle.add_argument("--screenshot")
    battle.set_defaults(run=run_battle)
    viewer = commands.add_parser("replay", help="offline viewer; never constructs a hardware backend")
    viewer.add_argument("path")
    viewer.add_argument("--headless", action="store_true")
    viewer.set_defaults(run=replay)
    args = parser.parse_args()
    try:
        args.run(args)
    except KeyboardInterrupt:
        print("Interrupted; cleanup completed.", file=sys.stderr)
    except (OSError, RuntimeError, ValueError, TypeError) as error:
        parser.exit(1, f"BOTticelli: {error}\n")


if __name__ == "__main__":
    main()
