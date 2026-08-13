#!/usr/bin/env python3
"""
lanelet_route_matcher.py

/planning/mission_planning/route (autoware_planning_msgs/LaneletRoute) 기준으로,
"현재 에고 위치부터 route를 따라가며 처음 만나는 신호등 group_id"를 구하는 모듈.

정석(Autoware) 방식: mission_planner가 lanelet2 RoutingGraph로 이미 계산해서 발행하는
route를 그대로 구독하고, lanelet2 지도 자체에서 lanelet -> regulatory_element(신호등)
연결관계를 읽어서 매핑을 만든다. route 순서를 기준으로 앞으로만 탐색하므로,
유클리드 최근접 매칭에서 생길 수 있는 "뒤쪽/다른 분기 신호등 오매칭" 문제가
구조적으로 발생하지 않는다.

의존성: lanelet2 python bindings (ros-noetic-lanelet2, ros-noetic-lanelet2-python).
        설치 안 돼있거나 맵 로드 실패 시 이 매처는 항상 None을 반환하도록 만들어져서,
        호출하는 쪽(traffic_light_node.py)이 안전하게 기존 유클리드 방식으로
        폴백할 수 있다. 즉 이 모듈은 "있으면 쓰고 없으면 그냥 무시되는" 부가 기능이다.

좌표계: lanelet2 맵을 직접 로드해서 쓰기 때문에, ego pose도 반드시 같은 프레임
       (Autoware "map" 프레임, mission_planner가 쓰는 /localization/kinematic_state와
       동일)이어야 한다. 기존 TrafficLightGroupIdMatcher가 쓰던 /Ego_topic +
       오프셋 보정 좌표계와는 다른 프레임이니 섞어 쓰면 안 됨.
"""
import math

import rospy

try:
    from lanelet2 import core, geometry, io, projection, traffic_rules

    LANELET2_AVAILABLE = True
    LANELET2_IMPORT_ERROR = None
except ImportError as exc:  # pragma: no cover
    core = geometry = io = projection = traffic_rules = None
    LANELET2_AVAILABLE = False
    LANELET2_IMPORT_ERROR = exc

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


def _load_projector(map_file, projector_type, projector_file):
    """mission_planner의 lanelet2_route_planner.py와 동일한 규칙으로 projector를 만든다."""
    configured_type = str(projector_type or "LocalCartesianUTM")
    origin_values = None
    if projector_file:
        if yaml is None:
            raise RuntimeError("PyYAML이 필요합니다 (projector_file 파싱)")
        with open(projector_file, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f) or {}
        configured_type = str(config.get("projector_type", configured_type))
        map_origin = config.get("map_origin")
        if map_origin:
            origin_values = (
                float(map_origin["latitude"]),
                float(map_origin["longitude"]),
                float(map_origin.get("altitude", 0.0)),
            )
    if origin_values is None:
        raise RuntimeError(
            "map_origin을 projector_file에서 못 찾음 (LocalCartesianUTM은 필수)"
        )

    origin = io.Origin(origin_values[0], origin_values[1])
    name = configured_type.lower().replace("_", "")
    if name in ("localcartesianutm", "localutm"):
        return projection.UtmProjector(origin, True, False)
    if name == "utm":
        return projection.UtmProjector(origin, False, False)
    if name in ("localcartesian", "enu"):
        return projection.LocalCartesianProjector(origin)
    raise RuntimeError(f"지원하지 않는 projector_type: {configured_type}")


class LaneletRouteGroupIdMatcher:
    """
    route + lanelet2 맵 기반 group_id 매처.

    사용법:
        matcher = LaneletRouteGroupIdMatcher(map_file, projector_file, valid_group_ids)
        matcher.start_subscribers(route_topic, odom_topic)
        ...
        gid = matcher.get_group_id()   # None이면 호출자가 폴백 처리
    """

    def __init__(self, map_file, projector_type, projector_file, valid_group_ids,
                 projector_type_default="LocalCartesianUTM", pose_stale_sec=2.0, route_stale_sec=30.0):
        self.available = False
        self.map = None
        self.lanelet_group_id = {}
        self.valid_group_ids = set(int(g) for g in valid_group_ids)
        self.pose_stale_sec = pose_stale_sec
        self.route_stale_sec = route_stale_sec

        self._route_segments = None  # list[list[int]] (각 segment의 primitive id들, [0]이 preferred)
        self._route_stamp = None
        self._ego_xy = None
        self._ego_stamp = None

        self._sub_route = None
        self._sub_odom = None

        if not LANELET2_AVAILABLE:
            rospy.logwarn(
                f"[lanelet_route_matcher] lanelet2 python bindings 없음 ({LANELET2_IMPORT_ERROR}) "
                "-> route 기반 매칭 비활성화, 기존 방식으로 폴백"
            )
            return

        try:
            projector = _load_projector(map_file, projector_type or projector_type_default, projector_file)
            self.map = io.load(map_file, projector)
            self._build_lanelet_group_id_map()
            self.available = True
            rospy.loginfo(
                f"[lanelet_route_matcher] lanelet2 맵 로드 완료: lanelet {len(self.map.laneletLayer)}개, "
                f"신호등 연결 lanelet {len(self.lanelet_group_id)}개"
            )
        except Exception as e:
            rospy.logwarn(f"[lanelet_route_matcher] 맵 로드/파싱 실패: {e} -> route 기반 매칭 비활성화")
            self.available = False

    def _build_lanelet_group_id_map(self):
        """lanelet -> regulatory_element(신호등) 연결관계를 스캔해서 lanelet_id -> group_id 매핑 구축."""
        for lanelet in self.map.laneletLayer:
            for reg_elem in lanelet.regulatoryElements:
                try:
                    rid = int(reg_elem.id)
                except Exception:
                    continue
                if rid in self.valid_group_ids:
                    self.lanelet_group_id[int(lanelet.id)] = rid
                    break  # 신호등 하나 찾으면 이 lanelet은 됐음

    def start_subscribers(self, route_topic, odom_topic):
        if not self.available:
            return
        from autoware_planning_msgs.msg import LaneletRoute
        from nav_msgs.msg import Odometry

        self._sub_route = rospy.Subscriber(route_topic, LaneletRoute, self._route_callback, queue_size=1)
        self._sub_odom = rospy.Subscriber(odom_topic, Odometry, self._odom_callback, queue_size=1)
        rospy.loginfo(f"[lanelet_route_matcher] 구독 시작: {route_topic}, {odom_topic}")

    def _route_callback(self, msg):
        segments = []
        for seg in msg.segments:
            ids = [int(seg.preferred_primitive.id)] + [
                int(p.id) for p in seg.primitives if int(p.id) != int(seg.preferred_primitive.id)
            ]
            segments.append(ids)
        self._route_segments = segments
        self._route_stamp = rospy.Time.now()
        rospy.loginfo(f"[lanelet_route_matcher] 새 route 수신: {len(segments)}개 구간")

    def _odom_callback(self, msg):
        self._ego_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._ego_stamp = rospy.Time.now()

    def _centerline_min_dist(self, lanelet_id, x, y):
        try:
            lanelet = self.map.laneletLayer[lanelet_id]
        except Exception:
            return None
        centerline = list(lanelet.centerline)
        if not centerline:
            return None
        if len(centerline) == 1:
            p = centerline[0]
            return math.hypot(x - p.x, y - p.y)
        best = None
        for start, end in zip(centerline[:-1], centerline[1:]):
            dx, dy = end.x - start.x, end.y - start.y
            length_sq = dx * dx + dy * dy
            ratio = 0.0
            if length_sq > 1e-12:
                ratio = max(0.0, min(1.0, ((x - start.x) * dx + (y - start.y) * dy) / length_sq))
            px, py = start.x + ratio * dx, start.y + ratio * dy
            d = math.hypot(x - px, y - py)
            if best is None or d < best:
                best = d
        return best

    def _is_fresh(self, stamp, max_age_sec):
        if stamp is None:
            return False
        return (rospy.Time.now() - stamp).to_sec() <= max_age_sec

    def get_group_id(self):
        """route를 앞으로 탐색해서 처음 만나는 신호등 group_id. 못 구하면 None (호출자가 폴백)."""
        if not self.available:
            return None
        if not self._route_segments:
            return None
        if not self._is_fresh(self._route_stamp, self.route_stale_sec):
            return None
        if self._ego_xy is None or not self._is_fresh(self._ego_stamp, self.pose_stale_sec):
            return None

        ex, ey = self._ego_xy

        # 1) route 상에서 지금 에고와 가장 가까운 구간(index) 찾기
        current_index = 0
        best_dist = None
        for idx, primitive_ids in enumerate(self._route_segments):
            for lane_id in primitive_ids:
                d = self._centerline_min_dist(lane_id, ex, ey)
                if d is None:
                    continue
                if best_dist is None or d < best_dist:
                    best_dist, current_index = d, idx

        # 2) 그 지점부터 앞으로 탐색하며 신호등 연결된 lanelet 찾기
        for idx in range(current_index, len(self._route_segments)):
            for lane_id in self._route_segments[idx]:
                gid = self.lanelet_group_id.get(lane_id)
                if gid is not None:
                    return gid
        return None
