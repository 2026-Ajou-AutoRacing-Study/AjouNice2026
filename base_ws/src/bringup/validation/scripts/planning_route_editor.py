#!/usr/bin/env python3
"""Edit competition route drafts without starting a controller or route policy."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import uuid

import lanelet2
import yaml


def projection(line, x, y):
    """Return distance, arc length, interpolated xyz and heading."""
    best = None
    arc = 0.0
    for a, b in zip(line, line[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length < 1e-8:
            continue
        t = max(0.0, min(1.0, ((x-a[0])*dx + (y-a[1])*dy) / length**2))
        point = tuple(a[k] + t * (b[k]-a[k]) for k in range(3))
        candidate = (math.hypot(x-point[0], y-point[1]), arc+t*length,
                     point, math.atan2(dy, dx))
        if best is None or candidate[0] < best[0]:
            best = candidate
        arc += length
    if best is None:
        raise ValueError('도로 중심선에 유효한 구간이 없습니다.')
    return best


class RouteMap:
    def __init__(self, repo, definition):
        self.definition = copy.deepcopy(definition)
        path = Path(repo) / definition['map']['file']
        self.fingerprint = hashlib.sha256(path.read_bytes()).hexdigest()
        if self.fingerprint != definition['map']['sha256']:
            raise ValueError('경로 정의와 지도의 SHA256이 다릅니다.')
        projector = yaml.safe_load((Path(repo) / definition['map']['projector_file']).read_text())
        if projector['projector_type'] != 'LocalCartesianUTM':
            raise ValueError('LocalCartesianUTM 지도만 지원합니다.')
        origin = projector['map_origin']
        self.map, self.load_warnings = lanelet2.io.loadRobust(str(path),
            lanelet2.projection.UtmProjector(lanelet2.io.Origin(
                origin['latitude'], origin['longitude'], origin.get('altitude', 0.0))))
        self.rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
        self.graph = lanelet2.routing.RoutingGraph(self.map, self.rules)
        self.lines = {}
        self.bounds = {}
        for lane in self.map.laneletLayer:
            if not self.rules.canPass(lane):
                continue
            line = [(p.x, p.y, p.z) for p in lane.centerline]
            if len(line) >= 2:
                self.lines[lane.id] = line
                self.bounds[lane.id] = (min(p[0] for p in line), max(p[0] for p in line),
                                       min(p[1] for p in line), max(p[1] for p in line))

    def snap(self, x, y, max_distance=8.0):
        if not all(math.isfinite(v) for v in (x, y)):
            raise ValueError('유효하지 않은 좌표입니다.')
        candidates = []
        for lane, bounds in self.bounds.items():
            if not (bounds[0]-max_distance <= x <= bounds[1]+max_distance and
                    bounds[2]-max_distance <= y <= bounds[3]+max_distance):
                continue
            distance, arc, point, yaw = projection(self.lines[lane], x, y)
            candidates.append((distance, lane, arc, point, yaw))
        if not candidates or min(candidates)[0] > max_distance:
            raise ValueError('도로에서 너무 멉니다. 도로 위를 클릭하세요 (8m 이내).')
        _, lane, arc, point, yaw = min(candidates)
        return dict(x=point[0], y=point[1], z=point[2], yaw_deg=math.degrees(yaw),
                    lanelet_id=lane, lanelet_s_m=arc)

    def route_lines(self, ids):
        return [self.lines[int(i)] for i in ids]

    def leg(self, first, last):
        a, b = int(first['lanelet_id']), int(last['lanelet_id'])
        if a == b:
            ids = [a]
        else:
            route = self.graph.getRoute(self.map.laneletLayer[a], self.map.laneletLayer[b], 0, True)
            if route is None:
                raise ValueError('도로 연결을 찾지 못했습니다: {} → {}'.format(a, b))
            ids = [lane.id for lane in route.shortestPath()]
        lines = self.route_lines(ids)
        # Clip the first/last lane to the edited poses. Do not draw past the goal.
        clipped = []
        for i, line in enumerate(lines):
            start = projection(line, first['x'], first['y'])[1] if i == 0 else 0.0
            end = projection(line, last['x'], last['y'])[1] if i == len(lines)-1 else float('inf')
            if end + 0.05 < start:
                raise ValueError('같은 차로에서 진행 방향 뒤쪽으로 연결할 수 없습니다.')
            points = []
            arc = 0.0
            for p, q in zip(line, line[1:]):
                length = math.hypot(q[0]-p[0], q[1]-p[1])
                if length > 1e-8 and arc+length >= start and arc <= end:
                    for value in (max(arc, start), min(arc+length, end)):
                        t = max(0.0, min(1.0, (value-arc)/length))
                        points.append(tuple(p[k]+t*(q[k]-p[k]) for k in range(3)))
                arc += length
            if len(points) >= 2:
                clipped.append(points)
        return clipped


class RouteDrafts:
    def __init__(self, road):
        self.road = road
        cfg = road.definition
        self.original = {}
        for key in ('route_a', 'route_b'):
            route = cfg[key]
            nodes = []
            if key == 'route_a':
                start = dict(cfg['start']['map_pose'], lanelet_id=cfg['start']['outgoing_lanelet_id'])
            else:
                lane = route['expected_path_lanelet_ids'][0]
                line = road.lines[lane]
                start = dict(x=line[0][0], y=line[0][1], z=line[0][2],
                             yaw_deg=math.degrees(math.atan2(line[1][1]-line[0][1], line[1][0]-line[0][0])),
                             lanelet_id=lane)
            nodes.append(dict(start, label='START'))
            nodes.extend(dict(cfg['poses'][ref], label=ref) for ref in route['waypoint_refs'])
            nodes.append(dict(cfg['poses'][route['goal_ref']], label='GOAL'))
            self.original[key] = nodes
        self.routes = copy.deepcopy(self.original)
        self.history = {'route_a': [], 'route_b': []}
        self.selected = 'route_a'
        self.changed = {'route_a': False, 'route_b': False}

    @property
    def nodes(self):
        return self.routes[self.selected]

    def remember(self):
        self.history[self.selected].append((copy.deepcopy(self.nodes), self.changed[self.selected]))
        self.history[self.selected] = self.history[self.selected][-50:]
        self.changed[self.selected] = True

    def lines(self, key=None):
        key = key or self.selected
        if not self.changed[key]:
            return self.road.route_lines(self.road.definition[key]['expected_path_lanelet_ids'])
        result = []
        for a, b in zip(self.routes[key], self.routes[key][1:]):
            result.extend(self.road.leg(a, b))
        return result

    def insert(self, x, y):
        node = self.road.snap(x, y)
        try:
            lines = self.lines()
        except ValueError:
            lines = [[(a['x'], a['y'], a['z']), (b['x'], b['y'], b['z'])]
                     for a, b in zip(self.nodes, self.nodes[1:])]
        # Determine progress on the drawn route, then insert between its anchors.
        def progress(x, y, minimum=0.0):
            offset, best = 0.0, None
            for line in lines:
                d, s, _, _ = projection(line, x, y)
                total = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(line,line[1:]))
                if offset+s+0.05 >= minimum:
                    candidate = (d, offset+s)
                    if best is None or candidate < best:
                        best = candidate
                offset += total
            return best[1] if best else offset
        target = progress(node['x'], node['y'])
        previous = 0.0
        index = len(self.nodes)-1
        for i, anchor in enumerate(self.nodes[1:-1], 1):
            previous = progress(anchor['x'], anchor['y'], previous)
            if target < previous:
                index = i
                break
        self.remember()
        node['label'] = 'click_' + uuid.uuid4().hex[:6]
        self.nodes.insert(index, node)
        return index

    def move(self, index, x, y):
        if index < 0 or index >= len(self.nodes):
            raise ValueError('유효하지 않은 경유점입니다.')
        if index == 0:
            raise ValueError('기준 시작점은 고정되어 있습니다.')
        point = dict(self.road.snap(x, y), label=self.nodes[index]['label'])
        self.remember()
        self.nodes[index] = point

    def delete(self, index):
        if index <= 0 or index >= len(self.nodes)-1:
            raise ValueError('START와 GOAL은 삭제할 수 없습니다. GOAL은 드래그로 이동하세요.')
        self.remember()
        del self.nodes[index]

    def undo(self):
        if self.history[self.selected]:
            self.routes[self.selected], self.changed[self.selected] = self.history[self.selected].pop()

    def reset(self):
        self.remember()
        self.routes[self.selected] = copy.deepcopy(self.original[self.selected])
        self.changed[self.selected] = False

    def save(self, path):
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        value = {'schema_version': 1, 'frame_id': 'map', 'map_sha256': self.road.fingerprint,
                 'routes': self.routes, 'changed': self.changed}
        temporary = path.with_suffix(path.suffix+'.tmp')
        temporary.write_text(yaml.safe_dump(value, sort_keys=False, allow_unicode=True))
        os.replace(str(temporary), str(path))

    def load(self, path):
        value = yaml.safe_load(Path(path).expanduser().read_text())
        if (not isinstance(value, dict) or value.get('schema_version') != 1 or value.get('frame_id') != 'map' or
                value.get('map_sha256') != self.road.fingerprint):
            raise ValueError('편집 파일 형식 또는 지도 버전이 다릅니다.')
        routes = value.get('routes', {})
        if not isinstance(routes, dict):
            raise ValueError('경로 목록 형식이 올바르지 않습니다.')
        for key in ('route_a', 'route_b'):
            nodes = routes.get(key)
            if not isinstance(nodes, list) or len(nodes) < 2:
                raise ValueError('START와 GOAL이 필요합니다.')
            for node in nodes:
                if not isinstance(node, dict) or not isinstance(node.get('label'), str):
                    raise ValueError('경유점 형식이 올바르지 않습니다.')
                if int(node['lanelet_id']) not in self.road.lines or not all(
                    math.isfinite(float(node[k])) for k in ('x','y','z','yaw_deg')):
                    raise ValueError('유효하지 않은 경유점이 있습니다.')
            if nodes[0] != self.original[key][0]:
                raise ValueError('편집 파일의 기준 시작점이 다릅니다.')
        self.routes = copy.deepcopy(routes)
        self.changed = {key: self.routes[key] != self.original[key] for key in ('route_a','route_b')}
        self.history = {'route_a': [], 'route_b': []}


class RouteEditorNode:
    def __init__(self):
        import rospy
        from interactive_markers.interactive_marker_server import InteractiveMarkerServer
        from std_msgs.msg import String
        from geometry_msgs.msg import PointStamped
        from visualization_msgs.msg import MarkerArray
        self.rospy = rospy
        self.lock = threading.RLock()
        definition = Path(rospy.get_param('~route_definition_file'))
        self.drafts = RouteDrafts(RouteMap(rospy.get_param('~repo_root'),
                                          yaml.safe_load(definition.read_text())))
        self.save_file = rospy.get_param('~save_file')
        self.message = 'Route A/B 전체 경로. 클릭 추가(P), Interact로 점을 드래그하세요.'
        if Path(self.save_file).expanduser().is_file():
            try:
                self.drafts.load(self.save_file)
                self.message = '저장한 편집본을 불러왔습니다.'
            except (ValueError, KeyError, OSError, TypeError, yaml.YAMLError) as exc:
                self.message = '저장본 불러오기 실패: '+str(exc)
        self.selected_point = -1
        self.busy = False
        self.status = rospy.Publisher('/planning/route_editor/status', String, queue_size=1, latch=True)
        self.markers = rospy.Publisher('/planning/route_editor/markers', MarkerArray, queue_size=1, latch=True)
        self.server = InteractiveMarkerServer('planning_route_editor')
        self.click_sub = rospy.Subscriber('/planning/route_editor/add_point', PointStamped,
                                          self.on_click, queue_size=10)
        self.command_sub = rospy.Subscriber('/planning/route_editor/command', String,
                                            self.on_command, queue_size=10)
        self.redraw()

    def on_click(self, message):
        with self.lock:
            try:
                if message.header.frame_id != 'map':
                    raise ValueError('RViz Fixed Frame을 map으로 설정하세요.')
                self.selected_point = self.drafts.insert(message.point.x, message.point.y)
                self.message = '경유점을 추가했습니다. 저장하거나 경로 적용을 누르세요.'
            except (ValueError, KeyError) as exc:
                self.message = str(exc)
            self.redraw()

    def on_command(self, message):
        with self.lock:
            try:
                command = json.loads(message.data)
                action = command['action']
                if action == 'select':
                    if command['route'] not in self.drafts.routes:
                        raise ValueError('Route A/B를 선택하세요.')
                    self.drafts.selected = command['route']
                    self.selected_point = -1
                    self.message = command['route'].upper()+' 선택'
                elif action == 'delete':
                    self.drafts.delete(int(command.get('index', self.selected_point)))
                    self.selected_point = -1
                    self.message = '경유점을 삭제했습니다.'
                elif action == 'undo':
                    self.drafts.undo()
                    self.selected_point = -1
                    self.message = '이전 편집으로 되돌렸습니다.'
                elif action == 'reset':
                    self.drafts.reset()
                    self.selected_point = -1
                    self.message = '선택 경로를 원본으로 복원했습니다.'
                elif action in ('save','load'):
                    path = command.get('path') or self.save_file
                    getattr(self.drafts, action)(path)
                    self.save_file = str(Path(path).expanduser())
                    self.message = ('저장: ' if action == 'save' else '불러옴: ')+self.save_file
                elif action == 'apply':
                    if self.busy:
                        raise ValueError('경로 요청을 처리 중입니다.')
                    self.drafts.lines()  # Reject a disconnected draft before an RPC.
                    self.busy = True
                    self.message = 'Planning에 경로 요청 중...'
                    threading.Thread(target=self.apply_route,
                        args=(copy.deepcopy(self.drafts.nodes),), daemon=True).start()
                else:
                    raise ValueError('알 수 없는 편집 명령입니다.')
            except (ValueError, KeyError, OSError, TypeError, yaml.YAMLError) as exc:
                self.message = str(exc)
            self.redraw()

    def apply_route(self, nodes):
        from autoware_internal_planning_msgs.srv import SetWaypointRoute, SetWaypointRouteRequest
        from geometry_msgs.msg import Pose
        def pose(node):
            p = Pose()
            p.position.x, p.position.y, p.position.z = node['x'], node['y'], node['z']
            yaw = math.radians(node['yaw_deg'])/2
            p.orientation.z, p.orientation.w = math.sin(yaw), math.cos(yaw)
            return p
        try:
            self.rospy.wait_for_service('/planning/set_waypoint_route', timeout=2)
            request = SetWaypointRouteRequest()
            request.header.frame_id = 'map'
            request.header.stamp = self.rospy.Time.now()
            request.goal_pose = pose(nodes[-1])
            request.waypoints = [pose(node) for node in nodes[1:-1]]
            request.uuid.uuid = list(uuid.uuid4().bytes)
            request.allow_modification = False
            response = self.rospy.ServiceProxy('/planning/set_waypoint_route', SetWaypointRoute)(request)
            detail = 'Planning 경로 요청 수락' if response.status.success else 'Planning 경로 요청 거절'
            detail += ': '+response.status.message
        except Exception as exc:
            detail = '경로 요청 실패: '+str(exc)
        with self.lock:
            self.busy = False
            self.message = detail
            self.publish_status()

    def feedback(self, feedback):
        from visualization_msgs.msg import InteractiveMarkerFeedback
        with self.lock:
            try:
                self.selected_point = int(feedback.marker_name.split('_')[-1])
                if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
                    index = self.selected_point
                    original = self.drafts.nodes[index]
                    p = feedback.pose.position
                    if math.hypot(p.x-original['x'], p.y-original['y']) > 0.02:
                        self.drafts.move(index, p.x, p.y)
                        self.message = '도로 중심선으로 경유점을 이동했습니다.'
                    self.redraw()
                elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
                    self.publish_status()
            except (ValueError, IndexError) as exc:
                self.message = str(exc)
                self.redraw()

    def publish_status(self):
        from std_msgs.msg import String
        self.status.publish(String(data=json.dumps({'route': self.drafts.selected,
            'points': self.drafts.nodes, 'selected_index': self.selected_point,
            'message': self.message, 'save_file': self.save_file, 'busy': self.busy}, ensure_ascii=False)))

    def redraw(self):
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        def line_marker(lines, identity, color, width):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns, marker.id = 'competition_route_editor', identity
            marker.type = Marker.LINE_LIST
            marker.pose.orientation.w = 1
            marker.scale.x = width
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            for line in lines:
                for a,b in zip(line,line[1:]):
                    marker.points.extend([Point(a[0],a[1],a[2]+0.3),Point(b[0],b[1],b[2]+0.3)])
            array.markers.append(marker)
        for i,key in enumerate(('route_a','route_b')):
            line_marker(self.drafts.road.route_lines(self.drafts.road.definition[key]['expected_path_lanelet_ids']),
                        i, (0.1,0.6,1.0,0.95) if i == 0 else (0.1,1.0,0.4,0.95), 1.8)
        valid = True
        try:
            lines = self.drafts.lines()
        except ValueError as exc:
            lines = [[(a['x'],a['y'],a['z']),(b['x'],b['y'],b['z'])]
                     for a,b in zip(self.drafts.nodes,self.drafts.nodes[1:])]
            self.message += ' | '+str(exc)
            valid = False
        line_marker(lines, 2, (1.0,0.65,0.05,1.0) if valid else (1.0,0.1,0.1,0.8), 3.2)
        self.server.clear()
        for i,node in enumerate(self.drafts.nodes):
            marker = InteractiveMarker()
            marker.header.frame_id = 'map'
            marker.name = 'point_'+str(i)
            marker.description = '{} {}'.format(i,node['label'])
            marker.pose.position = Point(node['x'],node['y'],node['z']+0.6)
            marker.pose.orientation.w = 1
            marker.scale = 10.0
            control = InteractiveMarkerControl()
            control.name = 'move_on_map'
            control.orientation.w = control.orientation.y = math.sqrt(0.5)
            control.interaction_mode = (InteractiveMarkerControl.BUTTON if i == 0
                                        else InteractiveMarkerControl.MOVE_PLANE)
            control.orientation_mode = InteractiveMarkerControl.FIXED
            control.always_visible = True
            sphere = Marker()
            sphere.type = Marker.SPHERE
            sphere.pose.orientation.w = 1
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 6.0
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = (
                (0.1,1.0,0.2,1.0) if i == 0 else (1.0,0.2,0.2,1.0)
                if i == len(self.drafts.nodes)-1 else (1.0,0.75,0.1,1.0))
            control.markers.append(sphere)
            marker.controls.append(control)
            self.server.insert(marker,self.feedback)
        self.server.applyChanges()
        self.markers.publish(array)
        self.publish_status()


if __name__ == '__main__':
    import rospy
    rospy.init_node('planning_route_editor')
    editor = RouteEditorNode()
    rospy.spin()
