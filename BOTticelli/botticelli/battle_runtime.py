"""Battle input lifecycle, rendering, and offline snapshot playback."""
from dataclasses import asdict
import math
import time

from .battle import Battle
from .battle_config import load_battle
from .battle_input import BattleInput
from .config import Config, Gamepad
from .control import DriveCommand, DrivingRules
from .recording import Recorder


class BattleSession:
    def __init__(self, setup):
        self.world = Battle(setup)
        self.rules = DrivingRules(Gamepad(**setup['gamepad']))
        self.paused = True
        self.reason = 'Enter to start (release driving keys / gamepad A first)'
        self.generation = None

    def update(self, sample, now, *, resume=False, pause=False):
        changed = self.generation is not None and self.generation != sample.generation
        self.generation = sample.generation
        command = self.rules.update(sample, now)
        good = sample.connected and 0 <= now-sample.timestamp <= self.rules.config.stale_seconds
        if good and not sample.a:
            self.rules.ready = True
        if pause or changed or (not good and not self.world.cars[0].eliminated):
            self.paused = True
            self.reason = 'Paused' if pause else 'Input lost or stale: release controls, then Enter'
            self.rules.disable(self.reason)
            # The current release is fresh evidence for deliberate rearming.
            if good and not sample.a:
                self.rules.update(sample, now)
        if resume and not pause and (good or self.world.cars[0].eliminated) and not sample.a:
            self.paused = False
            self.reason = 'Running'
        if self.paused or self.world.finished:
            self.world.commands = {c.id: DriveCommand() for c in self.world.cars}
            return []
        commands = self.world.policy_commands()
        commands[0] = command if good else DriveCommand()
        return self.world.step(commands)


class BattleWindow:
    COLORS = [(80, 235, 184), (255, 179, 90), (107, 163, 255), (227, 119, 223), (245, 222, 110), (120, 212, 233)]

    def __init__(self):
        import pygame
        self.pg = pygame
        pygame.display.init()
        pygame.font.init()
        self.window = pygame.display.set_mode((1200, 840), pygame.RESIZABLE)
        pygame.display.set_caption('BOTticelli / BALLOON BATTLE')
        self.screen = pygame.Surface((1200, 840))
        self.font = pygame.font.SysFont('dejavusans', 17)
        self.small = pygame.font.SysFont('dejavusans', 14)
        self.tiny = pygame.font.SysFont('dejavusans', 10, bold=True)
        self.title = pygame.font.SysFont('dejavusans', 28, bold=True)

    def text(self, text, pos, color=(229, 237, 246), font=None):
        self.screen.blit((font or self.font).render(str(text), True, color), pos)

    def draw(self, setup, snapshot, state, controller=''):
        from .battle_geometry import balloon, body, tooth, transform
        from shapely.geometry import Polygon
        pg = self.pg
        self.screen.fill((9, 13, 19))
        self.text('BALLOON BATTLE', (32, 22), font=self.title)
        self.text(state, (32, 65), (80, 235, 184))
        self.text(f'SIMULATION  /  {snapshot["time"]:.2f} s', (900, 32), (127, 149, 171))
        boundary = Polygon(setup['arena']['polygon'])
        x0, y0, x1, y1 = boundary.bounds
        scale = min(770/(x1-x0), 630/(y1-y0))
        def point(p):
            return round(40+(p[0]-x0)*scale), round(740-(p[1]-y0)*scale)
        outline = [point(p) for p in boundary.exterior.coords]
        pg.draw.polygon(self.screen, (19, 29, 39), outline)
        pg.draw.lines(self.screen, (129, 149, 166), True, outline, max(2, round(setup['arena']['wall_thickness']*scale)))
        for o in setup['arena']['obstacles']:
            pg.draw.polygon(self.screen, (90, 99, 111), [point((o['x']+dx*o['length']/2, o['y']+dy*o['width']/2)) for dx, dy in [(-1,-1),(1,-1),(1,1),(-1,1)]])
        specs = {c['id']: c for c in setup['cars']}
        alive = sum(not c['eliminated'] for c in snapshot['cars'])
        self.text(f'{alive} / {len(specs)} remaining', (860, 110), font=self.title)
        for index, car in enumerate(snapshot['cars']):
            spec = specs[car['id']]
            pose = car['pose']
            color = (106, 111, 120) if car['eliminated'] else self.COLORS[car['id'] % len(self.COLORS)]
            pg.draw.polygon(self.screen, color, [point(p) for p in body(spec, pose).exterior.coords])

            for i in range(len(spec['toothpicks'])):
                base, tip = tooth(spec, pose, i)
                pg.draw.line(self.screen, (206, 191, 157) if not car['eliminated'] else color, point(base), point(tip), 2)
                pg.draw.circle(self.screen, (245, 242, 219) if not car['eliminated'] else color, point(tip), 2)
            center = balloon(spec, pose)
            pg.draw.circle(self.screen, color, point(center), max(2, round(spec['balloon']['radius']*scale)), 2 if car['popped'] else 0)
            if car['health'] <= 0:
                x, y = point(pose)
                pg.draw.line(self.screen, (230, 88, 83), (x-8,y-8), (x+8,y+8), 3)
                pg.draw.line(self.screen, (230, 88, 83), (x-8,y+8), (x+8,y-8), 3)
            # The front is independent of attachment placement: an outlined
            # white arrow, white bumper, and explicit FRONT tag rotate with it.
            front = (spec['length']+spec['wheelbase'])/2
            width = spec['width']
            arrow = [point(transform(pose, (x,y,0))) for x,y in
                     [(front-.015,0), (front-.12,width*.30), (front-.12,-width*.30)]]
            pg.draw.polygon(self.screen, (249,250,252), arrow)
            pg.draw.polygon(self.screen, (13,20,29), arrow, 1)
            pg.draw.line(self.screen, (249,250,252),
                         point(transform(pose,(front,-width*.42,0))),
                         point(transform(pose,(front,width*.42,0))), 3)
            nose = point(transform(pose,(front,0,0)))
            label_center = point(transform(pose,(front+.19,0,0)))
            pg.draw.line(self.screen, (176,192,208), nose, label_center, 1)
            label = self.tiny.render('FRONT',True,(249,250,252),(13,20,29))
            self.screen.blit(label,label.get_rect(center=label_center))
            self.text(str(car['id']), point((pose[0], pose[1]+.19)), color, self.small)
            y = 165+index*49
            label = 'YOU' if car['id'] == 0 else f'CAR {car["id"]}'
            self.text(f'{label}   {car["health"]:.0f} HP', (860,y), color)
            self.text(', '.join(car['causes']) or 'balloon intact', (860,y+22), color, self.small)
        if snapshot['finished']:
            self.text('DRAW' if snapshot['winner'] is None else f'WINNER: CAR {snapshot["winner"]}', (860, 650), (255, 215, 115), self.title)
        self.text(f'Seed: {setup["seed"]}', (32, 770), font=self.small)
        self.text('WASD / arrows drive   Enter resume   Space pause   R new   Shift+R repeat', (32, 800), font=self.small)
        self.text('White arrow / FRONT = car nose', (860, 690), font=self.small)
        self.text(controller[:35], (860, 720), font=self.small)
        self.text('Practice: random toothpick counts', (860, 770), font=self.small)
        self.text('Adjustable crash approximation', (860, 800), font=self.small)
        self.window.blit(pg.transform.smoothscale(self.screen, self.window.get_size()), (0, 0))
        pg.display.flip()

    def close(self):
        self.pg.quit()


def run_battle(args):
    if args.seconds is not None and (not math.isfinite(args.seconds) or not 0 < args.seconds <= 3600):
        raise ValueError('--seconds must be in (0, 3600]')
    if args.headless and args.seconds is None:
        raise ValueError('Headless battle requires bounded --seconds')
    if args.screenshot and args.headless:
        raise ValueError('--screenshot needs a windowed battle')
    setup = load_battle(args.config, seed=args.seed, opponents=args.opponents)
    print(f'Battle seed: {setup["seed"]}; opponents: {len(setup["cars"])-1}', flush=True)
    session = BattleSession(setup)
    recorder = Recorder(args.record, Config(), 'battle', setup=setup) if args.record else None
    window = None
    try:
        if args.headless:
            world = session.world
            if recorder:
                recorder.emit('pause', {'paused': False, 'reason': 'Headless policies'}, recorder.origin)
                recorder.emit('snapshot', world.snapshot(), recorder.origin)
            for _ in range(math.ceil(args.seconds/world.period)):
                events = world.step()
                if recorder:
                    stamp = recorder.origin+world.time
                    recorder.emit('commands', {str(k): asdict(v) for k, v in world.commands.items()}, stamp)
                    for event in events:
                        recorder.emit(event['type'], event, stamp)
                    recorder.emit('snapshot', world.snapshot(), stamp)
                if world.finished:
                    break
            if recorder and not world.finished:
                recorder.emit('result', {'time': world.time, 'complete': False, 'reason': 'duration limit', 'winner': None, 'draw': False}, recorder.origin+world.time)
            print(f'Battle at {world.time:.2f}s: '+('draw' if world.finished and world.winner is None else f'winner {world.winner}' if world.finished else f'{sum(not c.eliminated for c in world.cars)} surviving (duration limit)'))
            return
        import pygame
        from .dashboard import GamepadReader
        window = BattleWindow()
        reader = BattleInput(GamepadReader(Gamepad(**setup['gamepad'])))
        start = time.monotonic()
        next_tick = last_input_poll = start
        previous_pause = None
        while args.seconds is None or time.monotonic()-start < args.seconds:
            now = time.monotonic()
            events = pygame.event.get()
            if any(e.type == pygame.QUIT for e in events):
                break
            keys = [e for e in events if e.type == pygame.KEYDOWN]
            restart = next((e for e in keys if e.key == pygame.K_r), None)
            if restart:
                setup = setup if restart.mod & pygame.KMOD_SHIFT else load_battle(args.config, opponents=args.opponents)
                session = BattleSession(setup)
                print(f'Battle seed: {setup["seed"]}', flush=True)
                if recorder:
                    recorder.emit('setup', setup)
                previous_pause = None
            sample = reader.poll(events)
            # A delayed frame pauses instead of advancing stale commands or
            # replaying a backlog of missed simulation steps.
            delayed = now-last_input_poll > session.rules.config.stale_seconds
            last_input_poll = now
            world_events = session.update(sample, time.monotonic(),
                resume=any(e.key == pygame.K_RETURN for e in keys),
                pause=delayed or any(e.key == pygame.K_SPACE for e in keys))
            if recorder:
                pause_state = (session.paused, session.reason)
                if pause_state != previous_pause:
                    recorder.emit('pause', {'paused': session.paused, 'reason': session.reason})
                    previous_pause = pause_state
                recorder.emit('commands', {str(k): asdict(v) for k,v in session.world.commands.items()})
                for event in world_events:
                    recorder.emit(event['type'], event)
                recorder.emit('snapshot', session.world.snapshot())
            window.draw(setup, session.world.snapshot(), session.reason, reader.message)
            next_tick = max(next_tick+Battle.period, time.monotonic())
            time.sleep(max(0, next_tick-time.monotonic()))
        if args.screenshot:
            pygame.image.save(window.screen, args.screenshot)
    finally:
        if window:
            window.close()
        if recorder:
            recorder.close()


def replay_battle(args, metadata, records):
    """Read recorded snapshots, never construct physics, policies, or a socket."""
    setup = metadata['setup']
    window = None
    try:
        if not args.headless:
            window = BattleWindow()
        start = time.monotonic()
        count = 0
        state = 'OFFLINE REPLAY'
        for record in records:
            count += 1
            if record['type'] == 'setup':
                setup = record['data']
            elif record['type'] == 'pause':
                state = 'OFFLINE REPLAY / '+record['data']['reason']
            elif record['type'] == 'snapshot':
                snapshot = record['data']
                if not math.isfinite(snapshot['time']) or len(snapshot['cars']) != len(setup['cars']):
                    raise ValueError('Invalid battle snapshot')
                if window:
                    while time.monotonic()-start < record['time']:
                        if any(e.type == window.pg.QUIT for e in window.pg.event.get()):
                            return
                        time.sleep(.01)
                    if any(e.type == window.pg.QUIT for e in window.pg.event.get()):
                        return
                    window.draw(setup, snapshot, state)
        if args.headless:
            print(f'Read {count} battle records offline (schema 2)')
    finally:
        if window:
            window.close()
