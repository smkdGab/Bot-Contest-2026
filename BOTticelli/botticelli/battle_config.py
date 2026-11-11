"""Resolve practice settings once; recordings store the resulting geometry."""
from copy import deepcopy
from dataclasses import asdict
import hashlib
import math
import random
import secrets

import yaml
from shapely.geometry import LineString, Point, Polygon

from .config import Car, Config, Gamepad, Lidar
from .battle import Geometry, balloon, body, segment_blocked, tooth, transform

DEFAULTS = {
    'opponents': {'min': 1, 'max': 5}, 'generation_attempts': 2000,
    'arena': {'shapes': {'rectangle': 1, 'convex': 1, 'l': 1},
              'width': {'min': 4, 'max': 6}, 'height': {'min': 4, 'max': 6},
              'passage_min': 1, 'wall_height': .35, 'wall_thickness': .05, 'obstacles': []},
    'car': {'length': .30, 'width': .20, 'height': .20, 'wheelbase': .20,
            'max_speed': .60, 'speed_limit': 1, 'max_steering_deg': 30,
            'sensors': {'x': .20, 'y': 0, 'height': .15, 'camera_height': .28, 'hz': 8,
                        'maximum': 5, 'minimum': .01, 'horizontal_deg': 120,
                        'camera_deg': 60, 'noise_std': 0, 'drop_probability': 0,
                        'stale_seconds': .4},
            'balloon': {'radius': {'min': .06, 'max': .08}, 'clearance': .02,
                        'height': .28, 'mount': 'perimeter'},
            'toothpick_count': {'min': 1, 'max': 3},
            'toothpick': {'length': {'min': .08, 'max': .12},
                          'mount': 'perimeter', 'spread_deg': 30, 'elevation_deg': 0,
                          'height': .28}},
    'cars': {}, 'gamepad': asdict(Gamepad()),
    'damage': {'health': 100, 'minimum_speed': .10, 'reference_speed': .50,
               'multiplier': 50, 'separation': .02},
}


def stream(seed, label):
    return random.Random(int.from_bytes(hashlib.sha256(f'{seed}:{label}'.encode()).digest()[:16], 'big'))


def merge(default, override):
    result = deepcopy(default)
    for key, value in override.items():
        result[key] = merge(result[key], value) if isinstance(value, dict) and isinstance(result.get(key), dict) else deepcopy(value)
    return result


def sample(value, rng, *, integer=False):
    if isinstance(value, dict):
        if set(value) != {'min', 'max'} or value['min'] > value['max']:
            raise ValueError('Random ranges require ordered min and max')
        if integer:
            if any(type(value[k]) is not int for k in ('min', 'max')):
                raise ValueError('Count ranges must be integers')
            return rng.randint(value['min'], value['max'])
        value = rng.uniform(value['min'], value['max'])
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('Expected a finite number or a min/max range')
    if integer and type(value) is not int:
        raise ValueError('Counts must be integers')
    return value


def resolve_values(value, rng):
    if isinstance(value, dict):
        if set(value) == {'min', 'max'}:
            return sample(value, rng)
        return {k: resolve_values(v, rng) for k, v in value.items()}
    if isinstance(value, list):
        return [resolve_values(v, rng) for v in value]
    if isinstance(value, (int, float)) and (isinstance(value, bool) or not math.isfinite(value)):
        raise ValueError('Configuration numbers must be finite')
    return value


def keys(data, allowed, label):
    if not isinstance(data, dict) or set(data)-set(allowed):
        raise ValueError(f'Unknown or invalid {label} settings: {set(data) - set(allowed) if isinstance(data, dict) else data}')



def perimeter_mount(spec, rng, mount='perimeter'):
    """Sample a chassis edge and a point along it; return its outward heading."""
    if mount not in ('perimeter', 'front', 'rear', 'left', 'right'):
        raise ValueError('Attachment mount must be perimeter, front, rear, left, or right')
    length, width = spec['length'], spec['width']
    rear = -(length-spec['wheelbase'])/2
    if mount == 'perimeter':
        mount = rng.choices(('front', 'rear', 'left', 'right'), (width,width,length,length))[0]
    if mount in ('front', 'rear'):
        return [rear+length if mount == 'front' else rear, rng.uniform(-width/2,width/2)], 0 if mount == 'front' else 180
    return [rng.uniform(rear,rear+length), width/2 if mount == 'left' else -width/2], 90 if mount == 'left' else -90


def generate(data=None, *, seed=None, opponents=None):
    data = {} if data is None else data
    keys(data, DEFAULTS, 'battle')
    cfg = merge(DEFAULTS, data)
    seed = secrets.randbits(63) if seed is None else seed
    if type(seed) is not int:
        raise ValueError('Seed must be an integer')
    n = sample(cfg['opponents'] if opponents is None else opponents, stream(seed, 'count'), integer=True)
    if not 1 <= n <= 32:
        raise ValueError('opponents must be in 1..32')
    attempts = cfg['generation_attempts']
    if type(attempts) is not int or not 1 <= attempts <= 100000:
        raise ValueError('generation_attempts must be in 1..100000')
    ar = stream(seed, 'arena')
    arena = resolve_values(cfg['arena'], ar)
    keys(arena, (*DEFAULTS['arena'], 'polygon'), 'arena')
    if min(arena[k] for k in ('width', 'height', 'passage_min', 'wall_height', 'wall_thickness')) <= 0:
        raise ValueError('Arena dimensions must be positive')
    if 'polygon' not in arena:
        shapes = arena['shapes']
        keys(shapes, ('rectangle', 'convex', 'l'), 'arena shape')
        if not shapes or any(v < 0 for v in shapes.values()) or sum(shapes.values()) <= 0:
            raise ValueError('Arena shape weights must be nonnegative with a positive total')
        shape = ar.choices(list(shapes), list(shapes.values()))[0]
        w, h = arena['width'], arena['height']
        if shape == 'rectangle':
            polygon = [(0, 0), (w, 0), (w, h), (0, h)]
        elif shape == 'l':
            p = arena['passage_min']
            if min(w, h) <= p:
                raise ValueError('L arena dimensions must exceed passage_min')
            x, y = ar.uniform(p, (w+p)/2), ar.uniform(p, (h+p)/2)
            polygon = [(0, 0), (w, 0), (w, y), (x, y), (x, h), (0, h)]
        else:
            polygon = list(Polygon([(ar.uniform(0, w*.25), 0), (ar.uniform(w*.75, w), 0),
                                    (w, ar.uniform(h*.25, h*.75)), (ar.uniform(w*.6, w), h),
                                    (ar.uniform(0, w*.4), h), (0, ar.uniform(h*.25, h*.75))]).convex_hull.exterior.coords)[:-1]
        arena['polygon'], arena['shape'] = polygon, shape
    else:
        arena['shape'] = 'custom'
    if not isinstance(arena['polygon'], list) or len(arena['polygon']) < 3 or any(
            not isinstance(point, (list, tuple)) or len(point) != 2 or
            not all(type(v) in (int, float) and math.isfinite(v) for v in point) for point in arena['polygon']):
        raise ValueError('Arena polygon requires at least three finite [x, y] points')
    arena['polygon'] = [list(point) for point in arena['polygon']]
    p = Polygon(arena['polygon'])
    if not p.is_valid or p.is_empty or p.area <= 0 or p.interiors:
        raise ValueError('Arena must be one valid connected polygon without holes')
    for obstacle in arena['obstacles']:
        keys(obstacle, ('x', 'y', 'length', 'width', 'height'), 'obstacle')
        if set(obstacle) != {'x', 'y', 'length', 'width', 'height'} or min(obstacle[k] for k in ('length', 'width', 'height')) <= 0:
            raise ValueError('Invalid obstacle')
    geom = Geometry({'arena': arena})
    if any(not p.covers(o) for o in geom.obstacles):
        raise ValueError('Obstacles must be inside the arena')
    keys(cfg['damage'], DEFAULTS['damage'], 'damage')
    damage = resolve_values(cfg['damage'], stream(seed, 'damage'))
    if min(damage.values()) < 0 or min(damage[k] for k in ('health', 'reference_speed', 'separation')) <= 0:
        raise ValueError('Invalid crash damage settings')
    if not isinstance(cfg['cars'], dict) or any(str(k) not in {str(i) for i in range(n+1)} or not isinstance(v, dict) for k, v in cfg['cars'].items()):
        raise ValueError('cars overrides must use existing stable IDs 0..opponents')
    gamepad = Gamepad(**cfg['gamepad'])
    Config(gamepad=gamepad).validate()
    cars = []
    for identity in range(n+1):
        rng = stream(seed, f'attachments:{identity}')
        spec = merge(cfg['car'], cfg['cars'].get(str(identity), cfg['cars'].get(identity, {})))
        keys(spec, (*DEFAULTS['car'], 'pose', 'toothpicks'), 'car')
        count = sample(spec['toothpick_count'], rng, integer=True)
        pose_template = spec.pop('pose', None)
        spec = resolve_values(spec, rng)
        if pose_template is not None:
            spec['pose'] = pose_template
        if 'toothpicks' in spec:
            if not isinstance(spec['toothpicks'], list):
                raise ValueError('toothpicks must be a list')
            if type(spec['toothpick_count']) is int and spec['toothpick_count'] != len(spec['toothpicks']):
                raise ValueError('toothpick_count conflicts with the explicit toothpicks list')
            count = len(spec['toothpicks'])
        if not 1 <= count <= 16:
            raise ValueError('Practice toothpick count must be in 1..16')
        if not (0 < spec['wheelbase'] <= spec['length'] and spec['width'] > 0 and spec['height'] > 0
                and 0 < spec['max_speed'] <= 2 and 0 < spec['speed_limit'] <= 1
                and 0 < spec['max_steering_deg'] < 80):
            raise ValueError('Invalid battle car dimensions or drive limits')
        b = spec['balloon']
        keys(b, ('position', 'radius', 'clearance', 'height', 'mount'), 'balloon')
        if b['radius'] <= 0 or b['clearance'] < 0:
            raise ValueError('Invalid balloon radius or clearance')
        rear = -(spec['length']-spec['wheelbase'])/2
        edge, heading = perimeter_mount(spec, rng, b.get('mount', 'perimeter'))
        offset = b['radius']+b['clearance']
        b.setdefault('position', [edge[0]+offset*math.cos(math.radians(heading)),
                                  edge[1]+offset*math.sin(math.radians(heading)), b['height']])
        default_t = spec.pop('toothpick')
        keys(default_t, ('position', 'length', 'direction_deg', 'elevation_deg', 'height', 'mount', 'spread_deg'), 'toothpick')
        if 'toothpicks' not in spec:
            # Sample independently, rather than copying one resolved attachment.
            template = merge(cfg['car'], cfg['cars'].get(str(identity), cfg['cars'].get(identity, {})))['toothpick']
            spec['toothpicks'] = [resolve_values(template, rng) for _ in range(count)]
        for t in spec['toothpicks']:
            keys(t, ('position', 'length', 'direction_deg', 'elevation_deg', 'height', 'mount', 'spread_deg'), 'toothpick')
            edge, heading = perimeter_mount(spec, rng, t.get('mount', 'perimeter'))
            spread = t.get('spread_deg', 30)
            if not 0 <= spread <= 90:
                raise ValueError('toothpick spread_deg must be in 0..90')
            if 'position' in t and 'direction_deg' not in t:
                x, y, *_ = t['position']
                heading = min(((abs(x-rear),180), (abs(x-rear-spec['length']),0),
                               (abs(y-spec['width']/2),90), (abs(y+spec['width']/2),-90)))[1]
            t.setdefault('direction_deg', (heading+rng.uniform(-spread,spread)+180)%360-180)
            t.setdefault('elevation_deg', 0)
            t.setdefault('position', [*edge, t.get('height', .28)])
            if 'length' not in t:
                raise ValueError('Each explicit toothpick requires length')
            if not 0 < t['length'] <= 1 or abs(t['elevation_deg']) > 90:
                raise ValueError('Invalid toothpick length or elevation')
        spec['toothpick_count'] = count
        for point in [b['position'], *(t['position'] for t in spec['toothpicks'])]:
            if len(point) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in point):
                raise ValueError('Attachment position must be [x, y, height]')
        keys(spec['sensors'], DEFAULTS['car']['sensors'], 'sensors')
        s = spec['sensors']
        if not (0 < s['minimum'] < s['maximum'] <= 20 and 0 < s['hz'] <= 50 and s['height'] > 0 and s['camera_height'] > 0
                and 0 < s['horizontal_deg'] <= 180 and 0 < s['camera_deg'] <= 180
                and s['noise_std'] >= 0 and 0 <= s['drop_probability'] <= 1 and s['stale_seconds'] > 1/s['hz']):
            raise ValueError('Invalid battle sensors')
        spec['id'] = identity
        spec['sensor_seed'] = stream(seed, f'sensors:{identity}').getrandbits(63)
        spec['alignment'] = min(spec['toothpicks'], key=lambda t: abs((t['direction_deg']+180)%360-180))
        cars.append(spec)
    place = stream(seed, 'placement')
    # Place explicit cars first, so random cars cannot steal an explicit start.
    placed = []
    for spec in sorted(cars, key=lambda c: 'pose' not in c or any(isinstance(v, dict) for v in c['pose'])):
        explicit = spec.get('pose')
        if explicit is not None and (not isinstance(explicit, list) or len(explicit) != 3):
            raise ValueError('pose must be [rear axle x, y, yaw degrees]')
        random_start = explicit is None or any(isinstance(v, dict) for v in explicit)
        for _ in range(attempts if random_start else 1):
            x0, y0, x1, y1 = p.bounds
            values = [sample(v, place) for v in explicit] if explicit is not None else None
            pose = ([values[0], values[1], math.radians(values[2])] if values is not None else
                    [place.uniform(x0, x1), place.uniform(y0, y1), place.uniform(-math.pi, math.pi)])
            spec['pose'] = pose
            if valid_placement(spec, placed, geom):
                placed.append(spec)
                break
        else:
            raise ValueError(f'Cannot fit car {spec["id"]} and its attachments after bounded placement; change setup or seed')
    return {'seed': seed, 'arena': arena, 'cars': cars, 'damage': damage, 'gamepad': asdict(gamepad)}


def valid_placement(spec, others, geom):
    pose = spec['pose']
    chassis = body(spec, pose)
    center, radius = balloon(spec, pose), spec['balloon']['radius']
    if center[2]-radius <= 0 or not geom.fits(chassis, .02) or not geom.fits(Point(*center[:2]).buffer(radius), .02):
        return False
    if center[2]-radius <= spec['height'] and chassis.intersects(Point(*center[:2]).buffer(radius)):
        return False
    s = spec['sensors']
    if not geom.fits(Point(*transform(pose, (s['x'], s['y'], s['height']))[:2]), .02):
        return False
    own_solids = [(chassis, spec['height'])]
    foreign = [(body(o, o['pose']), o['height']) for o in others]
    if any(chassis.buffer(.02).intersects(poly) for poly, _ in foreign):
        return False
    if any(segment_blocked(*tooth(spec, pose, i), [*geom.solids, *own_solids, *foreign])
           or min(p[2] for p in tooth(spec, pose, i)) <= 0
           or not geom.fits(LineString([p[:2] for p in tooth(spec, pose, i)]), .02)
           for i in range(len(spec['toothpicks']))):
        return False
    for o in others:
        oc, r = balloon(o, o['pose']), o['balloon']['radius']
        if math.dist(center, oc) <= radius+r+.02:
            return False
        if center[2]-radius <= o['height'] and body(o, o['pose']).distance(Point(*center[:2])) <= radius+.02:
            return False
        if oc[2]-r <= spec['height'] and chassis.distance(Point(*oc[:2])) <= r+.02:
            return False
        for attacker, victim in ((spec, o), (o, spec)):
            for i in range(len(attacker['toothpicks'])):
                base, tip = tooth(attacker, attacker['pose'], i)
                if math.dist(tip, balloon(victim, victim['pose'])) <= victim['balloon']['radius']+.02:
                    return False
                if segment_blocked(base, tip, [(body(victim, victim['pose']), victim['height'])]):
                    return False
    return True


def load_battle(path=None, **kwargs):
    data = {}
    if path is not None:
        with open(path, encoding='utf8') as source:
            data = yaml.safe_load(source)
    return generate(data, **kwargs)
