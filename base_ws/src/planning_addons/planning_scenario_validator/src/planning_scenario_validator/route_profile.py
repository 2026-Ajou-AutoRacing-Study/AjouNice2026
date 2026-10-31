"""Explicit competition preferred-lane profiles, preserving each mission's goal."""
import copy
import hashlib
import math
import threading
import time
import xml.etree.ElementTree as ET

import yaml


def normalize(document, resolve):
    value = document.get('route')
    if value is None:
        return None
    if not isinstance(value, dict) or set(value) != {'name', 'definition_file'}:
        raise ValueError('route에는 name과 definition_file을 지정하세요.')
    if value['name'] != 'route_b':
        raise ValueError('지원하지 않는 route.name입니다. route_b를 지정하세요.')
    if document['default_case'].get('waypoints'):
        raise ValueError('Route B 경유점을 사용하므로 default_case.waypoints는 []로 두세요.')
    path = resolve(value['definition_file'])
    data = path.read_bytes()
    definition = yaml.safe_load(data)
    if not isinstance(definition, dict):
        raise ValueError('Route B 정의 파일은 YAML mapping이어야 합니다.')
    route = definition.get(value['name'], {})
    if not isinstance(route, dict):
        raise ValueError('Route B 정의가 유효하지 않습니다.')
    sequence = route.get('expected_path_lanelet_ids')
    if (definition.get('frame_id') != 'map' or not isinstance(sequence, list) or
            not sequence or any(type(i) is not int for i in sequence) or
            len(sequence) != len(set(sequence))):
        raise ValueError('Route B의 선호 차선 목록이 유효하지 않습니다.')
    for key in ('file', 'projector_file', 'sha256'):
        if not definition.get('map', {}).get(key):
            raise ValueError('Route B 지도 계약이 없습니다.')
    return {'name': value['name'], 'definition_file': path,
            'definition_sha256': hashlib.sha256(data).hexdigest(),
            'definition': definition}


class ProfileGeometry:
    def __init__(self, profile, resolve):
        from pyproj import Transformer
        self.profile = profile
        definition = profile['definition']
        path = resolve(definition['map']['file'])
        if hashlib.sha256(path.read_bytes()).hexdigest() != definition['map']['sha256']:
            raise ValueError('Route B 정의와 현재 지도의 SHA256이 다릅니다.')
        projector = yaml.safe_load(resolve(definition['map']['projector_file']).read_text())
        if projector['projector_type'] != 'LocalCartesianUTM':
            raise ValueError('Route B는 LocalCartesianUTM 지도가 필요합니다.')
        origin = projector['map_origin']
        zone = int((origin['longitude']+180)//6)+1
        transform = Transformer.from_crs('EPSG:4326', 'EPSG:{}'.format(
            (32600 if origin['latitude'] >= 0 else 32700)+zone), always_xy=True)
        ox, oy = transform.transform(origin['longitude'], origin['latitude'])
        root = ET.parse(path).getroot()
        nodes = {}
        for node in root.findall('node'):
            x, y = transform.transform(float(node.attrib['lon']), float(node.attrib['lat']))
            tags = {t.attrib['k']: t.attrib['v'] for t in node.findall('tag')}
            nodes[int(node.attrib['id'])] = (x-ox, y-oy,
                float(tags.get('ele', 0))-origin.get('altitude', 0))
        ways = {int(w.attrib['id']): [nodes[int(n.attrib['ref'])] for n in w.findall('nd')]
                for w in root.findall('way')}
        centers = {}
        for relation in root.findall('relation'):
            center = next((m for m in relation.findall('member')
                if m.attrib.get('role') == 'centerline' and m.attrib.get('type') == 'way'), None)
            if center is not None:
                centers[int(relation.attrib['id'])] = ways[int(center.attrib['ref'])]
        self.sequence = definition[profile['name']]['expected_path_lanelet_ids']
        self.lines = [centers[lane] for lane in self.sequence]
        self.lengths = [sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(line,line[1:]))
                        for line in self.lines]
        if any(length < 1e-6 for length in self.lengths):
            raise ValueError('Route B 중심선 길이가 유효하지 않습니다.')

    def locate(self, pose):
        candidates = []
        yaw = math.radians(pose['yaw_deg'])
        for i,line in enumerate(self.lines):
            arc = 0.0
            for a,b in zip(line,line[1:]):
                dx,dy = b[0]-a[0], b[1]-a[1]
                length = math.hypot(dx,dy)
                if length < 1e-8:
                    continue
                heading = math.atan2(dy,dx)
                ratio = max(0,min(1,((pose['x']-a[0])*dx+(pose['y']-a[1])*dy)/length**2))
                distance = math.hypot(pose['x']-a[0]-ratio*dx,pose['y']-a[1]-ratio*dy)
                angle = abs(math.atan2(math.sin(yaw-heading),math.cos(yaw-heading)))
                if angle < math.pi/3 and distance <= 15:
                    candidates.append((distance,i,arc+ratio*length))
                arc += length
        if not candidates:
            raise ValueError('시작점 또는 목적지가 Route B 진행 구간에서 벗어났습니다.')
        _,index,arc = min(candidates)
        return index,arc

    def center_pose(self, index):
        remaining = self.lengths[index]/2
        for a,b in zip(self.lines[index],self.lines[index][1:]):
            length = math.hypot(b[0]-a[0],b[1]-a[1])
            if length > 0 and remaining <= length:
                t = remaining/length
                return dict(x=a[0]+t*(b[0]-a[0]),y=a[1]+t*(b[1]-a[1]),
                    z=a[2]+t*(b[2]-a[2]),yaw_deg=math.degrees(math.atan2(b[1]-a[1],b[0]-a[0])))
            remaining -= length
        raise ValueError('Route B 중심점을 계산할 수 없습니다.')

    def plan(self, start, goal):
        begin,begin_arc = self.locate(start)
        end,end_arc = self.locate(goal)
        if end < begin or (end == begin and end_arc <= begin_arc):
            raise ValueError('Route B 목적지는 시작점보다 진행 방향 앞에 있어야 합니다.')
        return {'name': self.profile['name'], 'goal_preserved': True, 'verified': False,
                'goal': copy.deepcopy(goal),
                'expected_preferred_lanelet_ids': list(self.sequence[begin:end+1]),
                'waypoints': [self.center_pose(i) for i in range(begin+1,end)],
                'definition_sha256': self.profile['definition_sha256'],
                'map_sha256': self.profile['definition']['map']['sha256']}


def preferred_primitives(route, expected):
    """Never silently retain the default branch if a B lane is missing."""
    if len(route.segments) != len(expected):
        raise ValueError('Route B 구간 수 불일치: 실제 {} / 설정 {}'.format(len(route.segments),len(expected)))
    result = []
    for segment,lane in zip(route.segments,expected):
        primitive = next((p for p in segment.primitives if p.id == lane),None)
        if primitive is None:
            raise ValueError('Route B 선호 차선 {}이 route 후보에 없습니다.'.format(lane))
        result.append(copy.deepcopy(primitive))
    return result


def apply_and_verify(plan, monitor, route_uuid, *, timeout, process=None):
    import rospy
    from autoware_planning_msgs.srv import SetPreferredPrimitive,SetPreferredPrimitiveRequest
    from .runtime import RuntimeFailure
    expected = plan['expected_preferred_lanelet_ids']
    route = monitor.route
    if route is None or list(route.uuid.uuid) != list(route_uuid):
        raise RuntimeFailure('Route B 적용 전에 route UUID가 변경됐습니다.')
    plan['before_preferred_lanelet_ids'] = [s.preferred_primitive.id for s in route.segments]
    try:
        primitives = preferred_primitives(route,expected)
    except ValueError as exc:
        raise RuntimeFailure(str(exc)) from exc
    # Waypoints in every B lane normally produce the exact B preference already.
    # Do not turn that successful initial request into an unnecessary reroute.
    if plan['before_preferred_lanelet_ids'] == expected:
        plan['preferred_update_requested'] = False
        plan['verified_preferred_lanelet_ids'] = list(expected)
        plan['verified'] = True
        monitor.event('ROUTE_PROFILE_VERIFIED',profile=plan['name'],preferred_lanelet_ids=expected,
                      goal_preserved=True)
        return
    plan['preferred_update_requested'] = True
    service = '/planning/mission_planning/mission_planner/set_preferred_primitive'
    rospy.wait_for_service(service,timeout=timeout)
    request = SetPreferredPrimitiveRequest()
    request.uuid = copy.deepcopy(route.uuid)
    request.reset = False
    request.preferred_primitives = primitives
    outcome,done = {},threading.Event()
    def invoke():
        try:
            outcome['response'] = rospy.ServiceProxy(service,SetPreferredPrimitive)(request)
        except Exception as exc:
            outcome['error'] = str(exc)
        finally:
            done.set()
    after = time.monotonic()
    threading.Thread(target=invoke,daemon=True).start()
    if not done.wait(timeout):
        raise RuntimeFailure('Route B 선호 차선 요청 시간 초과. 자동 재요청하지 않습니다.')
    if 'error' in outcome or not outcome['response'].status.success:
        raise RuntimeFailure('Route B 선호 차선 적용 실패: '+str(outcome.get('error') or outcome['response'].status))
    def matches():
        if process is not None and process.poll() is not None:
            raise RuntimeFailure('Route B 적용 중 planning 스택이 종료됐습니다.')
        r = monitor.route
        return (r is not None and list(r.uuid.uuid) == list(route_uuid) and
                monitor.stamps.get('route',0) > after and
                [s.preferred_primitive.id for s in r.segments] == expected)
    monitor.wait(matches,timeout,'Route B 선호 차선 확인')
    monitor.wait_route_and_trajectory(route_uuid,after=after,timeout=timeout,process=process)
    if not matches():
        raise RuntimeFailure('Route B 궤적 준비 후 선호 차선이 변경됐습니다.')
    plan['verified_preferred_lanelet_ids'] = [s.preferred_primitive.id for s in monitor.route.segments]
    plan['verified'] = True
    monitor.event('ROUTE_PROFILE_VERIFIED',profile=plan['name'],preferred_lanelet_ids=expected,
                  goal_preserved=True)
