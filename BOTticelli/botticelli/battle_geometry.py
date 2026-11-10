"""Height-aware battle geometry. Coordinates are metres from the rear axle."""
import math

import numpy as np
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import nearest_points

from .lidar import RayCaster


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
