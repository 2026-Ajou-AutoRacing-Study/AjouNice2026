"""Frozen v64 geometry for progress guards, not for route generation."""
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from pyproj import Transformer


def project(points, x, y):
    best = (float('inf'), 0.0, 0.0)
    length = 0.0
    for a, b in zip(points, points[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        norm = dx * dx + dy * dy
        if norm < 1e-12:
            continue
        ratio = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / norm))
        distance = math.hypot(x - a[0] - ratio * dx, y - a[1] - ratio * dy)
        if distance < best[0]:
            best = distance, length + ratio * math.sqrt(norm), math.atan2(dy, dx)
        length += math.sqrt(norm)
    return best


def inside(polygon, x, y):
    result = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        if (a[1] > y) != (b[1] > y) and x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]:
            result = not result
    return result


class Geometry:
    def __init__(self, repo, config):
        path = Path(repo) / config['map']['file']
        if hashlib.sha256(path.read_bytes()).hexdigest() != config['map']['sha256']:
            raise ValueError('v64 map SHA mismatch; no route requests permitted')
        projector_path = Path(repo) / config['map']['projector_file']
        if hashlib.sha256(projector_path.read_bytes()).hexdigest() != 'ccb5a8199afd7e64b04b6ac8637419ec4c9cd5ab16bac68a229971a7f7c7859c':
            raise ValueError('v64 projector SHA mismatch')
        cfg = yaml.safe_load(projector_path.read_text())
        if cfg['projector_type'] != 'LocalCartesianUTM':
            raise ValueError('unsupported projector')
        origin = cfg['map_origin']
        zone = int((origin['longitude'] + 180) // 6) + 1
        transform = Transformer.from_crs('EPSG:4326', 'EPSG:%d' % (32600 + zone), always_xy=True)
        ox, oy = transform.transform(origin['longitude'], origin['latitude'])
        root = ET.parse(path).getroot()
        nodes = {}
        for node in root.findall('node'):
            x, y = transform.transform(float(node.attrib['lon']), float(node.attrib['lat']))
            nodes[int(node.attrib['id'])] = (x - ox, y - oy)
        ways = {int(w.attrib['id']): [nodes[int(n.attrib['ref'])] for n in w.findall('nd')]
                for w in root.findall('way')}
        self.lanes = {}
        for r in root.findall('relation'):
            members = {m.attrib['role']: int(m.attrib['ref']) for m in r.findall('member') if m.attrib['type'] == 'way'}
            if all(k in members for k in ('centerline', 'left', 'right')):
                self.lanes[int(r.attrib['id'])] = (ways[members['centerline']],
                    ways[members['left']] + list(reversed(ways[members['right']])))
        self.candidates = set(config['route_a']['expected_path_lanelet_ids'] +
                              config['route_b']['expected_path_lanelet_ids'] +
                              config['start']['accepted_start_lanelet_ids'])

    def locate(self, x, y, yaw):
        candidates = []
        for lane in self.candidates:
            line, polygon = self.lanes[lane]
            distance, arc, heading = project(line, x, y)
            angle = abs(math.atan2(math.sin(yaw - heading), math.cos(yaw - heading)))
            if angle < math.pi / 3 and (inside(polygon, x, y) or distance <= 0.15):
                candidates.append((distance, lane, arc))
        if not candidates:
            return None
        _, lane, arc = min(candidates)
        return lane, arc
