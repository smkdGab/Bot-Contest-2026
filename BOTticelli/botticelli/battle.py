"""Simultaneous 50 Hz balloon battle using pinned IR-SIM Ackermann dynamics.

Damage is a tunable game approximation, not a LEGO breakage prediction.
"""
from dataclasses import dataclass
import math

import numpy as np

from .battle_geometry import Geometry, balloon, body, contact_normal, interpolate_pose, relative_hit, segment_blocked, tooth, transform
from .battle_policy import BattleObservation, Detection, HuntingPolicy
from .config import Car
from .control import DriveCommand, map_command


@dataclass
class BattleCar:
    spec: dict
    pose: list
    health: float
    popped: bool = False
    eliminated: bool = False
    causes: tuple = ()
    collision: bool = False

    @property
    def id(self):
        return self.spec['id']


class Battle:
    period = .02

    def __init__(self, setup):
        self.setup = setup
        self.geometry = Geometry(setup)
        self.cars = [BattleCar(s, list(s['pose']), setup['damage']['health']) for s in setup['cars']]
        self.time = 0.0
        self.finished = False
        self.winner = None
        self.events = []
        self.episodes = set()
        self.policies = {c.id: HuntingPolicy(c.spec['alignment']) for c in self.cars}
        self.rngs = {c.id: np.random.default_rng(c.spec['sensor_seed']) for c in self.cars}
        self.next_scan = {c.id: 0 for c in self.cars}
        self.observations = {}
        self.commands = {c.id: DriveCommand() for c in self.cars}
        self._sense()

    def snapshot(self):
        return {'time': self.time, 'cars': [{'id': c.id, 'pose': list(c.pose), 'health': c.health,
                'popped': c.popped, 'eliminated': c.eliminated, 'causes': c.causes,
                'collision': c.collision} for c in self.cars],
                'finished': self.finished, 'winner': self.winner}

    def policy_commands(self):
        # Every policy receives only its immutable sensor observation from the
        # same world snapshot. Own attachment calibration is supplied at reset.
        return {c.id: self.policies[c.id].step(self.observations[c.id], self.time)
                if not c.eliminated else DriveCommand() for c in self.cars}

    def _sense(self):
        geometries = {c.id: body(c.spec, c.pose) for c in self.cars}
        for c in self.cars:
            if self.time+1e-9 < self.next_scan[c.id]:
                continue
            s = c.spec['sensors']
            self.next_scan[c.id] += 1/s['hz']
            rng = self.rngs[c.id]
            if rng.random() < s['drop_probability']:
                self.observations.setdefault(c.id, BattleObservation(-1e9, (), (), fresh_seconds=s['stale_seconds']))
                continue
            origin = transform(c.pose, (s['x'], s['y'], s['height']))
            yaw = c.pose[2]
            angles = np.linspace(-math.radians(s['horizontal_deg'])/2, math.radians(s['horizontal_deg'])/2, 7)+yaw
            directions = np.array([[math.cos(a), math.sin(a), 0] for a in angles])
            foreign = [(geometries[o.id], o.spec['height']) for o in self.cars if o.id != c.id]
            ranges = self.geometry.ranges(origin, directions, foreign)
            ranges = np.minimum(ranges, s['maximum'])
            if s['noise_std']:
                ranges += rng.normal(0, s['noise_std'], len(ranges))
            ranges = np.clip(ranges, s['minimum'], s['maximum'])
            origin = transform(c.pose, (s['x'], s['y'], s['camera_height']))
            detections = []
            for o in self.cars:
                if o.id == c.id or o.eliminated:
                    continue
                target = balloon(o.spec, o.pose)
                delta = target-origin
                distance = float(np.linalg.norm(delta))
                bearing = math.atan2(delta[1], delta[0])-yaw
                bearing = math.atan2(math.sin(bearing), math.cos(bearing))
                elevation = math.atan2(delta[2], math.hypot(*delta[:2]))
                if distance > s['maximum'] or abs(bearing) > math.radians(s['camera_deg'])/2 or abs(elevation) > math.radians(30):
                    continue
                solids = [*self.geometry.solids, *((geometries[b.id], b.spec['height']) for b in self.cars)]
                if segment_blocked(origin, target, solids):
                    continue
                detections.append(Detection(bearing, math.asin(min(1, o.spec['balloon']['radius']/max(distance, 1e-9))), elevation))
            self.observations[c.id] = BattleObservation(self.time, tuple(float(v) for v in ranges),
                tuple(detections), c.collision, s['stale_seconds'])

    def _separation(self, key, polygons):
        a, b = key
        if isinstance(b, str):
            solid = self.geometry.walls if b == 'wall' else self.geometry.obstacles[int(b.split(':')[1])]
            return polygons[a].distance(solid)
        return polygons[a].distance(polygons[b])

    @staticmethod
    def _point_velocity(old, new, point, dt):
        omega = math.atan2(math.sin(new[2]-old[2]), math.cos(new[2]-old[2]))/dt
        offset = np.asarray(point)-np.asarray(old[:2])
        return (np.asarray(new[:2])-old[:2])/dt + omega*np.array([-offset[1], offset[0]])

    def step(self, commands=None):
        if self.finished:
            return []
        from irsim.lib.algorithm.kinematics import ackermann_kinematics
        self.cars.sort(key=lambda c: c.id)
        self.events = []
        commands = self.policy_commands() if commands is None else commands
        if set(commands)-{c.id for c in self.cars}:
            raise ValueError('Unknown command car ID')
        mapped = {}
        maximum_travel = 0
        for c in self.cars:
            command = commands.get(c.id, DriveCommand()) if not c.eliminated else DriveCommand()
            self.commands[c.id] = command
            spec = c.spec
            calibration = Car(**{k: spec[k] for k in ('length', 'width', 'wheelbase', 'max_speed', 'speed_limit', 'max_steering_deg')})
            mapped[c.id] = map_command(command, calibration)
            # Bound body and attachment motion to 5 mm per substep, including
            # angular sweep. Sphere hits then use an analytic relative sweep.
            extent = max(spec['length'], np.linalg.norm(spec['balloon']['position'])+spec['balloon']['radius'],
                         *(np.linalg.norm(t['position'])+t['length'] for t in spec['toothpicks']))
            v = abs(mapped[c.id].velocity)
            maximum_travel = max(maximum_travel, v*self.period*(1+extent*math.tan(math.radians(spec['max_steering_deg']))/spec['wheelbase']))
            c.collision = False
        substeps = max(1, math.ceil(maximum_travel/.005))
        if substeps > 500:
            raise ValueError('Setup exceeds bounded collision resolution')
        dt = self.period/substeps
        pending_pops = set()
        pending_damage = {c.id: 0 for c in self.cars}
        cars = {c.id: c for c in self.cars}
        for substep in range(substeps):
            old = {c.id: list(c.pose) for c in self.cars}
            polygons = {c.id: body(c.spec, c.pose) for c in self.cars}
            self.episodes = {k for k in self.episodes if self._separation(k, polygons) < self.setup['damage']['separation']}
            proposed = {}
            for c in self.cars:
                m = mapped[c.id]
                state = np.array([*c.pose[:3], m.wheel_angle]).reshape(4, 1)
                new = ackermann_kinematics(state, np.array([[m.velocity], [m.wheel_angle]]), dt, wheelbase=c.spec['wheelbase'])
                proposed[c.id] = [float(v) for v in new[:3, 0]]
            candidate = {c.id: body(c.spec, proposed[c.id]) for c in self.cars}
            rejected, contacts = set(), {}
            # A simultaneous fixed point: rejecting one car may block another.
            # All contacts in each pass are collected before any rejection.
            while True:
                swept = {}
                for i in cars:
                    hull = polygons[i].union(polygons[i] if i in rejected else candidate[i]).convex_hull
                    angle = abs(proposed[i][2]-old[i][2])
                    angle = abs(math.atan2(math.sin(angle), math.cos(angle)))
                    # The arc bows outside an endpoint hull. This sagitta
                    # bound prevents even a thin corner from tunnelling.
                    sagitta = cars[i].spec['length']*angle*angle/8 if i not in rejected else 0
                    swept[i] = hull.buffer(sagitta) if sagitta > 1e-12 else hull
                blocked = set(rejected)
                for i in sorted(cars):
                    if i in rejected or proposed[i] == old[i]:
                        continue
                    if not self.geometry.boundary.covers(swept[i]):
                        blocked.add(i)
                        contacts[(i, 'wall')] = self.geometry.walls
                    for k, obstacle in enumerate(self.geometry.obstacles):
                        if swept[i].intersects(obstacle):
                            blocked.add(i)
                            contacts[(i, f'obstacle:{k}')] = obstacle
                ids = sorted(cars)
                for index, i in enumerate(ids):
                    for j in ids[index+1:]:
                        if swept[i].intersects(swept[j]):
                            blocked.update((i, j))
                            contacts[(i, j)] = polygons[j]
                if blocked == rejected:
                    break
                rejected = blocked
            event_time = self.time+(substep+1)*dt
            for (i, j), solid in contacts.items():
                cars[i].collision = True
                if isinstance(j, int):
                    cars[j].collision = True
                if (i, j) in self.episodes:
                    continue
                self.episodes.add((i, j))
                point, normal = contact_normal(polygons[i], solid)
                vi = self._point_velocity(old[i], proposed[i], point, dt)
                vj = self._point_velocity(old[j], proposed[j], point, dt) if isinstance(j, int) else np.zeros(2)
                speed = max(0., float((vi-vj)@normal))
                d = self.setup['damage']
                severity = d['multiplier']*(max(0, speed-d['minimum_speed'])/d['reference_speed'])**2
                self.events.append({'type': 'contact', 'time': event_time, 'a': i, 'b': j,
                                    'position': point.tolist(), 'closing_speed': speed})
                if severity:
                    pending_damage[i] += severity
                    if isinstance(j, int):
                        pending_damage[j] += severity
                    self.events.append({'type': 'damage', 'time': event_time, 'a': i, 'b': j, 'amount': severity})
            accepted = {i: old[i] if i in rejected else proposed[i] for i in cars}
            for attacker in self.cars:
                if attacker.eliminated:
                    continue
                tips = [(tooth(attacker.spec, old[attacker.id], i), tooth(attacker.spec, accepted[attacker.id], i)) for i in range(len(attacker.spec['toothpicks']))]
                for victim in self.cars:
                    if victim.id == attacker.id or victim.eliminated or victim.id in pending_pops:
                        continue
                    center0 = balloon(victim.spec, old[victim.id])
                    center1 = balloon(victim.spec, accepted[victim.id])
                    for index, ((base0, tip0), (base1, tip1)) in enumerate(tips):
                        fraction = relative_hit(tip0, tip1, center0, center1, victim.spec['balloon']['radius'])
                        if fraction is None:
                            continue
                        contact = tip0+fraction*(tip1-tip0)
                        base = base0+fraction*(base1-base0)
                        hit_bodies = [(body(c.spec, interpolate_pose(old[c.id], accepted[c.id], fraction)), c.spec['height']) for c in self.cars]
                        if segment_blocked(base, contact, [*self.geometry.solids, *hit_bodies]):
                            continue
                        pending_pops.add(victim.id)
                        self.events.append({'type': 'pop', 'time': self.time+(substep+fraction)*dt,
                            'attacker': attacker.id, 'victim': victim.id, 'toothpick': index,
                            'position': contact.tolist()})
                        break
            for c in self.cars:
                c.pose = accepted[c.id]
        self.time += self.period
        for c in self.cars:
            c.health = max(0., c.health-pending_damage[c.id])
            if c.health < 1e-8:
                c.health = 0.
            c.popped |= c.id in pending_pops
            if not c.eliminated and (c.popped or c.health <= 0):
                c.eliminated = True
                c.causes = tuple(cause for cause, yes in [('balloon', c.popped), ('destroyed', c.health <= 0)] if yes)
                self.commands[c.id] = DriveCommand()
                self.events.append({'type': 'elimination', 'time': self.time, 'car': c.id, 'causes': c.causes})
        survivors = [c.id for c in self.cars if not c.eliminated]
        if len(survivors) <= 1:
            self.finished = True
            self.winner = survivors[0] if survivors else None
            self.events.append({'type': 'result', 'time': self.time, 'winner': self.winner, 'draw': not survivors})
        self._sense()
        return self.events
