"""Battle geometry, sensor-only policy and simultaneous 50 Hz world updates."""
from dataclasses import dataclass
import math

import numpy as np
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import nearest_points

from .config import Car
from .control import DriveCommand, map_command
from .simulation import RayCaster


def transform(pose, point):
    x, y, yaw = pose[:3]
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([x+c*point[0]-s*point[1], y+s*point[0]+c*point[1], point[2]])


def interpolate_pose(start, end, fraction):
    angle = math.atan2(math.sin(end[2]-start[2]), math.cos(end[2]-start[2]))
    return [start[0]+fraction*(end[0]-start[0]), start[1]+fraction*(end[1]-start[1]),
            start[2]+fraction*angle]


def body(spec, pose):
    rear = -(spec['length']-spec['wheelbase'])/2
    front = rear+spec['length']
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return Polygon([(pose[0]+c*x-s*y, pose[1]+s*x+c*y) for x, y in
                    [(rear, -spec['width']/2), (front, -spec['width']/2),
                     (front, spec['width']/2), (rear, spec['width']/2)]])


def balloon(spec, pose):
    return transform(pose, spec['balloon']['position'])


def tooth(spec, pose, index):
    t = spec['toothpicks'][index]
    angle, elevation = map(math.radians, (t['direction_deg'], t['elevation_deg']))
    base = np.array(t['position'])
    tip = base+t['length']*np.array([math.cos(elevation)*math.cos(angle),
                                    math.cos(elevation)*math.sin(angle), math.sin(elevation)])
    return transform(pose, base), transform(pose, tip)


def segment_blocked(start, end, solids, *, trim=1e-7):
    """Intersect a 3D segment with extruded polygons (including inward corners)."""
    start, end = np.asarray(start), np.asarray(end)
    delta = end-start
    for poly, height in solids:
        lo, hi = trim, 1-trim
        if abs(delta[2]) < 1e-12:
            if not 0 <= start[2] <= height:
                continue
        else:
            z0, z1 = sorted((-start[2]/delta[2], (height-start[2])/delta[2]))
            lo, hi = max(lo, z0), min(hi, z1)
        if lo <= hi and poly.intersects(LineString([(start+lo*delta)[:2], (start+hi*delta)[:2]])):
            return True
    return False


def relative_hit(tip0, tip1, center0, center1, radius):
    """First tip/sphere intersection during a linear relative sweep, or None."""
    a, b = np.asarray(tip0)-center0, (np.asarray(tip1)-tip0)-(np.asarray(center1)-center0)
    c = float(a@a-radius*radius)
    if c <= 0:
        return 0.0
    aa, bb = float(b@b), float(2*a@b)
    disc = bb*bb-4*aa*c
    if aa < 1e-20 or disc < 0:
        return None
    t = (-bb-math.sqrt(disc))/(2*aa)
    return t if 0 <= t <= 1 else None


def contact_normal(first, second):
    a, b = nearest_points(first, second)
    delta = np.array(b.coords[0])-a.coords[0]
    if np.linalg.norm(delta) < 1e-9:
        # At exact contact, centroids give the wrong normal for a concave
        # wall band. Use the actual surface edge at the contact point.
        toward = np.array(a.coords[0])-first.centroid.coords[0]
        parts = [second] if second.geom_type == 'Polygon' else second.geoms
        choices = []
        for part in parts:
            for ring in [part.exterior, *part.interiors]:
                vertices = list(ring.coords)
                for start, end in zip(vertices[:-1], vertices[1:]):
                    edge = np.asarray(end)-start
                    length = np.linalg.norm(edge)
                    if length and LineString([start, end]).distance(a) < 1e-8:
                        normal = np.array([-edge[1], edge[0]])/length
                        if normal@toward < 0:
                            normal = -normal
                        choices.append((float(normal@toward), normal))
        delta = max(choices, key=lambda item: item[0])[1] if choices else toward
    norm = np.linalg.norm(delta)
    return (np.array(a.coords[0])+b.coords[0])/2, delta/max(norm, 1e-12)


class Geometry:
    def __init__(self, setup):
        arena = setup['arena']
        self.boundary = Polygon(arena['polygon'])
        # Exterior wall band: the interior edge is exactly the configured boundary.
        self.walls = self.boundary.buffer(arena['wall_thickness'], join_style=2).difference(self.boundary)
        self.solids = [(self.walls, arena['wall_height'])]
        self.obstacles = []
        for o in arena['obstacles']:
            x, y, l, w = o['x'], o['y'], o['length'], o['width']
            p = Polygon([(x-l/2, y-w/2), (x+l/2, y-w/2), (x+l/2, y+w/2), (x-l/2, y+w/2)])
            self.obstacles.append(p)
            self.solids.append((p, o['height']))

    def fits(self, shape, clearance=0):
        return self.boundary.buffer(-clearance).covers(shape) and not any(
            p.buffer(clearance).intersects(shape) for p in self.obstacles)

    def ranges(self, origin, directions, cars):
        """Reuse manual simulation's vectorized slab test for extruded rectangles.

        Cars use local rectangle slabs; walls and obstacles use polygon edges
        and horizontal caps. Wall inner edges are the movement boundary.
        """
        origin, directions = np.asarray(origin), np.asarray(directions)
        nearest = np.full(len(directions), np.inf)
        dz = directions[:, 2]
        np.divide(-origin[2], dz, out=nearest, where=dz < -1e-12)
        # Ray/polygon edges in XY, clipped to the wall/obstacle height. Include
        # horizontal cap intersections for elevated or pitched sensors.
        for poly, height in cars:
            # Rectangles use the same vectorized slab intersection as manual
            # driving; polygon walls below retain exact inward corners.
            corners = np.asarray(poly.exterior.coords)[:4]
            edge = corners[1]-corners[0]
            length = np.linalg.norm(edge)
            axis = edge/length
            normal = np.array([-axis[1], axis[0]])
            width = np.linalg.norm(corners[2]-corners[1])
            rotation = np.array([[axis[0], normal[0], 0], [axis[1], normal[1], 0], [0, 0, 1]])
            local = (origin-np.array([*corners[0], 0]))@rotation
            nearest = np.minimum(nearest, RayCaster.intersections(local, directions@rotation,
                np.zeros(3), np.array([length, width, height])))
        for poly, height in self.solids:
            parts = [poly] if poly.geom_type == 'Polygon' else poly.geoms
            for part in parts:
                for ring in [part.exterior, *part.interiors]:
                    points = np.asarray(ring.coords)
                    for a, b in zip(points[:-1], points[1:]):
                        edge = b-a
                        cross = directions[:, 0]*edge[1]-directions[:, 1]*edge[0]
                        safe = np.where(abs(cross) < 1e-12, np.nan, cross)
                        delta = a-origin[:2]
                        t = (delta[0]*edge[1]-delta[1]*edge[0])/safe
                        u = (delta[0]*directions[:, 1]-delta[1]*directions[:, 0])/safe
                        z = origin[2]+t*dz
                        hit = (t >= 0) & (u >= 0) & (u <= 1) & (z >= 0) & (z <= height)
                        nearest = np.minimum(nearest, np.where(hit, t, np.inf))
                for z in (0, height):
                    if not np.any(abs(dz) > 1e-12):
                        continue
                    t = np.divide(z-origin[2], dz, out=np.full(len(dz), np.inf), where=abs(dz) > 1e-12)
                    for i in np.flatnonzero((t >= 0) & (t < nearest)):
                        if part.covers(Point(*(origin+t[i]*directions[i])[:2])):
                            nearest[i] = t[i]
        return nearest


@dataclass(frozen=True)
class Detection:
    bearing: float  # radians, left positive, relative to the camera
    radius_angle: float  # apparent angular radius; no true target coordinates
    elevation: float


@dataclass(frozen=True)
class BattleObservation:
    timestamp: float
    distances: tuple[float, ...]  # seven forward rays, right to left
    balloons: tuple[Detection, ...]
    collision: bool = False
    fresh_seconds: float = .4


class HuntingPolicy:
    """Search, approach, align, avoid. reset/step are reusable by other policies."""
    def __init__(self, alignment=None):
        self.alignment = alignment or {'direction_deg': 0, 'position': [0, 0, .28]}
        self.reset()

    def reset(self):
        self.target = None
        self.seen = -math.inf
        self.reverse_until = 0
        self.reverses = 0
        self.turn = 1

    def step(self, observation: BattleObservation, time: float) -> DriveCommand:
        if not 0 <= time-observation.timestamp <= observation.fresh_seconds or not observation.distances:
            return DriveCommand()
        rays = observation.distances
        forward = min(rays[2:5])
        if observation.balloons:
            self.target = max(observation.balloons, key=lambda d: d.radius_angle)
            self.seen = time
        elif time-self.seen > .5:
            self.target = None
        if time < self.reverse_until:
            # A front-facing sensor cannot authorize reversing into a known rear
            # obstacle; collision feedback bounds these short recovery attempts.
            return DriveCommand(-.5, -self.turn)
        if forward < .16 or observation.collision:
            self.turn = 1 if sum(rays[4:]) >= sum(rays[:3]) else -1
            if self.reverses >= 2:
                return DriveCommand()
            self.reverses += 1
            self.reverse_until = time+.75
            return DriveCommand(-.5, -self.turn)
        if forward > .45:
            self.reverses = 0
        if forward < (.22 if self.target is not None else .45):
            self.turn = 1 if sum(rays[4:]) >= sum(rays[:3]) else -1
            return DriveCommand(.35, self.turn)
        if self.target is not None:
            desired = math.radians(self.alignment['direction_deg'])
            # The apparent radius estimates distance using the nominal practice
            # balloon radius. Only the observed bearing/size are available here.
            distance = .07/max(.001, math.tan(self.target.radius_angle))
            desired += math.atan2(self.alignment['position'][1], max(.08, distance))
            error = self.target.bearing-desired
            return DriveCommand(.8 if abs(error) < .25 else .4, max(-1, min(1, error*3)))
        return DriveCommand(.55, .6*self.turn)


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
