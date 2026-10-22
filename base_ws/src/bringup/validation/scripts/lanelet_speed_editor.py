#!/usr/bin/env python3
"""Pick lanelet footprints in RViz and persist map speed caps for the next run."""
import copy
import json
import math
import os
from pathlib import Path
import threading

import yaml
from planning_route_editor import RouteMap, projection


def contains(polygon, x, y):
    """XY footprint hit test, including the boundary; never snap to another road."""
    inside = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx * dx + dy * dy
        if length2 > 1e-12:
            t = max(0.0, min(1.0, ((x-a[0])*dx + (y-a[1])*dy) / length2))
            if math.hypot(x-a[0]-t*dx, y-a[1]-t*dy) < 1e-5:
                return True
        if (a[1] > y) != (b[1] > y):
            if x < (b[0]-a[0]) * (y-a[1]) / (b[1]-a[1]) + a[0]:
                inside = not inside
    return inside


class SpeedSelections:
    def __init__(self, road, path, speed_kmh=50.0):
        self.road, self.path = road, Path(path).expanduser()
        self.speed_kmh = float(speed_kmh)
        if not math.isfinite(self.speed_kmh) or not 0 < self.speed_kmh <= 150:
            raise ValueError('제한속도는 0 초과 150 km/h 이하여야 합니다.')
        self.route_ids = set(road.definition['route_b']['expected_path_lanelet_ids'])
        self.polygons = {i: [(p.x, p.y, p.z) for p in road.map.laneletLayer[i].polygon3d()]
                         for i in road.lines}
        self.bounds = {i: (min(p[0] for p in poly), max(p[0] for p in poly),
                           min(p[1] for p in poly), max(p[1] for p in poly))
                       for i, poly in self.polygons.items()}
        self.selected = {}
        self.history = []
        if self.path.is_file():
            config = yaml.safe_load(self.path.read_text())
            if (config['schema_version'] != 1 or
                    config['map_sha256'] != road.fingerprint or
                    config['map_file'] != road.definition['map']['file']):
                raise ValueError('속도제한 파일의 기준 지도가 현재 지도와 다릅니다.')
            for key, value in config['lanelet_speed_limits'].items():
                lane, speed = int(key), float(value)
                if lane not in self.polygons or not math.isfinite(speed) or not 0 < speed <= 150:
                    raise ValueError('잘못된 lanelet 또는 속도제한입니다: '+str(key))
                self.selected[lane] = speed

    def hit(self, x, y, z=28.4):
        if not all(math.isfinite(v) for v in (x, y, z)):
            raise ValueError('유효하지 않은 클릭 좌표입니다.')
        hits = []
        for lane, (xmin, xmax, ymin, ymax) in self.bounds.items():
            if xmin-1e-5 <= x <= xmax+1e-5 and ymin-1e-5 <= y <= ymax+1e-5:
                if contains(self.polygons[lane], x, y):
                    distance, _, point, _ = projection(self.road.lines[lane], x, y)
                    hits.append((abs(point[2]-z), distance, lane))
        if not hits:
            raise ValueError('도로 내부를 클릭하세요. 도로 밖에서는 선택하지 않습니다.')
        # Elevation first disambiguates stacked roads; centerline breaks edge ties.
        return min(hits)[2]

    def save(self, selected):
        data = dict(schema_version=1, map_file=self.road.definition['map']['file'],
                    map_sha256=self.road.fingerprint,
                    lanelet_speed_limits={str(i): float(v) for i, v in sorted(selected.items())})
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name(self.path.name+'.tmp')
        with temporary.open('w') as stream:
            yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(self.path))

    def update(self, selected):
        self.save(selected)  # A failed write must leave the visible selection unchanged.
        self.history.append(copy.deepcopy(self.selected))
        self.history = self.history[-50:]
        self.selected = selected

    def toggle(self, x, y, z=28.4):
        lane = self.hit(x, y, z)
        selected = dict(self.selected)
        if lane in selected:
            del selected[lane]
        else:
            selected[lane] = self.speed_kmh
        self.update(selected)
        return lane

    def undo(self):
        if self.history:
            self.save(self.history[-1])
            self.selected = self.history.pop()


class SpeedEditorNode:
    def __init__(self):
        import rospy
        from geometry_msgs.msg import PointStamped
        from std_msgs.msg import String
        from visualization_msgs.msg import MarkerArray
        self.lock = threading.RLock()
        definition = yaml.safe_load(Path(rospy.get_param('~route_definition_file')).read_text())
        self.model = SpeedSelections(RouteMap(rospy.get_param('~repo_root'), definition),
                                     rospy.get_param('~save_file'), rospy.get_param('~speed_kmh', 50.0))
        self.message = 'Lanelet 선택(L)을 누른 후 도로 안쪽을 클릭하세요. 클릭 즉시 저장됩니다.'
        self.last_lane = None
        self.status = rospy.Publisher('/planning/lanelet_speed_editor/status', String, queue_size=1, latch=True)
        self.markers = rospy.Publisher('/planning/lanelet_speed_editor/markers', MarkerArray, queue_size=1, latch=True)
        self.click = rospy.Subscriber('/planning/lanelet_speed_editor/click', PointStamped, self.on_click, queue_size=10)
        self.command = rospy.Subscriber('/planning/lanelet_speed_editor/command', String, self.on_command, queue_size=10)
        self.redraw()

    def on_click(self, msg):
        with self.lock:
            try:
                if msg.header.frame_id != 'map':
                    raise ValueError('RViz Fixed Frame을 map으로 설정하세요.')
                self.last_lane = self.model.toggle(msg.point.x, msg.point.y, msg.point.z)
                self.message = 'Lanelet {} {} · 저장 완료'.format(self.last_lane,
                    '50 km/h 선택' if self.last_lane in self.model.selected else '해제')
            except (ValueError, OSError) as exc:
                self.message = str(exc)
            self.redraw()

    def on_command(self, msg):
        with self.lock:
            try:
                command = json.loads(msg.data)
                if command['action'] == 'undo':
                    self.model.undo()
                elif command['action'] == 'remove':
                    selected = dict(self.model.selected)
                    selected.pop(int(command['lanelet_id']), None)
                    self.model.update(selected)
                elif command['action'] == 'clear':
                    self.model.update({})
                else:
                    raise ValueError('알 수 없는 편집 명령입니다.')
                self.message = '변경 내용을 저장했습니다. 다음 Planning 실행부터 적용됩니다.'
            except (ValueError, KeyError, OSError, TypeError) as exc:
                self.message = str(exc)
            self.redraw()

    def redraw(self):
        from geometry_msgs.msg import Point
        from std_msgs.msg import String
        from visualization_msgs.msg import Marker, MarkerArray
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        def marker(ns, identity, kind, color, width):
            m = Marker()
            m.header.frame_id, m.ns, m.id, m.type = 'map', ns, identity, kind
            m.pose.orientation.w = 1
            m.color.r, m.color.g, m.color.b, m.color.a = color
            m.scale.x = width
            return m
        def edges(m, points, closed=False):
            line = points + points[:1] if closed else points
            for a, b in zip(line, line[1:]):
                m.points.extend([Point(a[0], a[1], a[2]+0.5), Point(b[0], b[1], b[2]+0.5)])
        background = marker('selectable_roads', 0, Marker.LINE_LIST, (0.45, 0.53, 0.61, 0.5), 0.09)
        route = marker('route_b', 0, Marker.LINE_LIST, (0.15, 1.0, 0.4, 0.95), 0.25)
        for lane, polygon in self.model.polygons.items():
            edges(background, polygon, True)
            if lane in self.model.route_ids:
                edges(route, polygon, True)
                edges(route, self.model.road.lines[lane])
        array.markers.extend([background, route])
        for lane in sorted(self.model.route_ids | set(self.model.selected)):
            line = self.model.road.lines[lane]
            # Use a point on the road, rather than the polygon centroid on a curve.
            point = line[len(line)//2]
            selected = lane in self.model.selected
            if selected:
                outline = marker('speed_cap', lane, Marker.LINE_LIST, (1.0, 0.47, 0.02, 1.0), 0.6)
                edges(outline, self.model.polygons[lane], True)
                array.markers.append(outline)
            text = marker('lanelet_labels', lane, Marker.TEXT_VIEW_FACING,
                          (1.0, 0.7, 0.1, 1.0) if selected else (0.5, 1.0, 0.65, 1.0), 1.0)
            text.scale.z = 2.0 if selected else 1.3
            text.pose.position = Point(point[0], point[1], point[2]+1.1)
            text.text = '{}\n{:g} km/h'.format(lane, self.model.selected[lane]) if selected else str(lane)
            array.markers.append(text)
        self.markers.publish(array)
        self.status.publish(String(data=json.dumps(dict(route='Route B', speed_kmh=self.model.speed_kmh,
            selected=[dict(lanelet_id=i, speed_kmh=v) for i,v in sorted(self.model.selected.items())],
            message=self.message, save_file=str(self.model.path), last_lanelet_id=self.last_lane), ensure_ascii=False)))


if __name__ == '__main__':
    import rospy
    rospy.init_node('lanelet_speed_editor')
    editor = SpeedEditorNode()
    rospy.spin()
