#!/usr/bin/env python3
"""
lanelet_route_matcher.py (v7 - 곡선 구간 진행방향 국소 접선으로 계산)

/planning/mission_planning/route (autoware_planning_msgs/LaneletRoute) 기준으로,
"현재 에고 위치부터 route를 따라가며 아직 통과하지 않은 첫 신호등"의 group_id를
구하는 모듈.

v6 -> v7 변경점 (2026-08-19, group_id 오류 의심 원인 수정):
  - 문제 의심: 진행방향(dir_x, dir_y) 계산을 lanelet centerline의 "맨 처음 점 ->
    맨 끝 점"을 직선으로 이어서 구했는데, 이건 직선 도로에서만 정확함. 회전교차로
    (로터리)처럼 곡선인 lanelet에서는, 신호등 위치 근처의 실제 진행방향이 lanelet
    전체를 직선으로 이은 벡터와 크게 다를 수 있어서, 통과 판정(_has_crossed)의
    내적 계산이 부정확해지고 group_id 전환이 엉뚱한 시점에 일어날 수 있음. 오늘
    실주행 화면에 회전교차로 구간이 여러 번 나온 것과 맞물려 유력한 원인으로 판단.
  - 수정: lanelet 전체의 시작~끝이 아니라, 신호등 기준점(전구 위치)에서 가장 가까운
    centerline 구간(local segment)의 방향을 씀. 그 구간의 두 점을 이은 벡터가
    "그 지점 근처에서의 실제 진행방향"을 더 정확히 반영함 - 곡선이어도 국소적으로는
    직선에 가깝다는 성질을 이용.

정지선 대신 신호등 전구 위치(refers) 기준 유지, 판단팀 파라미터와 무관한 독립
전환 마진 유지 - v6에서 확정된 부분은 그대로.

정석(Autoware) 방식: mission_planner가 lanelet2 RoutingGraph로 이미 계산해서 발행하는
route를 그대로 구독하고, lanelet2 지도 자체에서 lanelet -> regulatory_element(신호등)
연결관계를 읽어서 매핑을 만든다.

의존성: lanelet2 python bindings. 설치 안 돼있거나 맵 로드 실패 시 이 매처는 항상
None을 반환하도록 만들어져서, 호출하는 쪽이 안전하게 fallback_group_id를 쓸 수 있다.
"""
import math
import xml.etree.ElementTree as ET

import rospy

try:
    from lanelet2 import core, geometry, io, projection, traffic_rules

    LANELET2_AVAILABLE = True
    LANELET2_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover
    # A Python/lanelet2 ABI mismatch can raise SystemError rather than
    # ImportError (for example the Noetic Python 3.8 binding under Python
    # 3.11).  Treat every import-time failure as an unavailable optional
    # binding so the image inference process can still use the external
    # route-group-id provider.
    core = geometry = io = projection = traffic_rules = None
    LANELET2_AVAILABLE = False
    LANELET2_IMPORT_ERROR = exc

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


DEFAULT_CROSSING_MARGIN_M = 0.0


def _load_projector(map_file, projector_type, projector_file):
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
        raise RuntimeError("map_origin을 projector_file에서 못 찾음 (LocalCartesianUTM은 필수)")

    origin = io.Origin(origin_values[0], origin_values[1])
    name = configured_type.lower().replace("_", "")
    if name in ("localcartesianutm", "localutm"):
        return projection.UtmProjector(origin, True, False)
    if name == "utm":
        return projection.UtmProjector(origin, False, False)
    if name in ("localcartesian", "enu"):
        return projection.LocalCartesianProjector(origin)
    raise RuntimeError(f"지원하지 않는 projector_type: {configured_type}")


def _is_traffic_light_regulatory_element(reg_elem):
    try:
        attrs = reg_elem.attributes
    except Exception:
        return False
    try:
        subtype = attrs["subtype"]
    except Exception:
        try:
            subtype = attrs.get("subtype", "")
        except Exception:
            return False
    return str(subtype) == "traffic_light"


def _parse_reference_latlon(map_file, member_role):
    result = {}
    try:
        tree = ET.parse(map_file)
        root = tree.getroot()
        nodes = {nd.get("id"): (float(nd.get("lat")), float(nd.get("lon"))) for nd in root.findall("node")}
        ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in root.findall("way")}

        for rel in root.findall("relation"):
            tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
            if tags.get("subtype") != "traffic_light":
                continue
            rid = rel.get("id")
            way_ids = [m.get("ref") for m in rel.findall("member") if m.get("role") == member_role]
            pts = []
            for wid in way_ids:
                for nref in ways.get(wid, []):
                    if nref in nodes:
                        pts.append(nodes[nref])
            if not pts:
                continue
            lat = sum(p[0] for p in pts) / len(pts)
            lon = sum(p[1] for p in pts) / len(pts)
            result[rid] = (lat, lon)
    except Exception as e:
        rospy.logwarn(f"[lanelet_route_matcher] {member_role} osm 파싱 실패: {e}")
    return result


def _local_tangent_direction(centerline_points, ref_x, ref_y):
    """centerline_points 중 ref 지점에 가장 가까운 구간(두 점 사이)을 찾아서,
    그 구간의 방향 벡터(정규화)를 반환. 곡선 lanelet에서도 국소적으로는 직선에
    가깝다는 성질을 이용해서, lanelet 전체 시작~끝을 잇는 것보다 정확함.
    실패 시 None."""
    if len(centerline_points) < 2:
        return None

    best_idx, best_dist_sq = 0, float("inf")
    for i in range(len(centerline_points) - 1):
        p1, p2 = centerline_points[i], centerline_points[i + 1]
        mx, my = (p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0
        dist_sq = (mx - ref_x) ** 2 + (my - ref_y) ** 2
        if dist_sq < best_dist_sq:
            best_dist_sq, best_idx = dist_sq, i

    p1, p2 = centerline_points[best_idx], centerline_points[best_idx + 1]
    dx, dy = p2.x - p1.x, p2.y - p1.y
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return None
    return dx / norm, dy / norm


class LaneletRouteGroupIdMatcher:
    """
    route + lanelet2 맵 기반 group_id 매처. 신호등 물리 위치(전구) 기준 통과
    판정, 곡선 구간은 국소 접선 방향으로 정확도 개선.

    사용법:
        matcher = LaneletRouteGroupIdMatcher(map_file, projector_type, projector_file,
                                              excluded_group_ids={201395, 201396})
        matcher.start_subscribers(route_topic, odom_topic)
        ...
        gid = matcher.get_group_id()   # None이면 호출자가 fallback_group_id 사용
    """

    def __init__(self, map_file, projector_type, projector_file, excluded_group_ids=None,
                 projector_type_default="LocalCartesianUTM", pose_stale_sec=2.0,
                 crossing_margin_m=DEFAULT_CROSSING_MARGIN_M,
                 crossing_reference="bulb"):
        if crossing_reference not in ("bulb", "stopline"):
            raise ValueError('crossing_reference는 "bulb" 또는 "stopline"이어야 합니다')
        self.available = False
        self.map = None
        self.lanelet_group_id = {}
        self.crossing_info = {}
        self.excluded_group_ids = set(int(g) for g in (excluded_group_ids or []))
        self.pose_stale_sec = pose_stale_sec
        self.crossing_margin_m = crossing_margin_m
        self.crossing_reference = crossing_reference

        self._route_segments = None
        self._route_stamp = None
        self._route_active = False
        self._ego_xy = None
        self._ego_stamp = None

        self._sub_route = None
        self._sub_route_state = None
        self._sub_odom = None

        if not LANELET2_AVAILABLE:
            rospy.logwarn(
                f"[lanelet_route_matcher] lanelet2 python bindings 없음 ({LANELET2_IMPORT_ERROR}) "
                "-> route 기반 매칭 비활성화"
            )
            return

        if not map_file:
            rospy.logwarn("[lanelet_route_matcher] map_file 없음 -> route 기반 매칭 비활성화")
            return

        try:
            self.projector = _load_projector(map_file, projector_type or projector_type_default, projector_file)
            self.map = io.load(map_file, self.projector)
            self._build_lanelet_group_id_map()
            self._build_crossing_info(map_file)
            self.available = True
            n_missing = len(self.lanelet_group_id) - len(
                set(self.crossing_info.keys()) & set(self.lanelet_group_id.values())
            )
            rospy.loginfo(
                f"[lanelet_route_matcher] lanelet2 맵 로드 완료: lanelet {len(self.map.laneletLayer)}개, "
                f"신호등 연결 lanelet {len(self.lanelet_group_id)}개, "
                f"기준점({self.crossing_reference}) 정보 확보 {len(self.crossing_info)}개, "
                f"전환 마진 {self.crossing_margin_m}m, 진행방향=국소접선(곡선 대응)"
                + (f" (기준점 정보 없는 신호등 {n_missing}개, 통과판정 불가 -> 항상 미통과로 간주)"
                   if n_missing > 0 else "")
            )
        except Exception as e:
            rospy.logwarn(f"[lanelet_route_matcher] 맵 로드/파싱 실패: {e} -> route 기반 매칭 비활성화")
            self.available = False

    def _build_lanelet_group_id_map(self):
        for lanelet in self.map.laneletLayer:
            for reg_elem in lanelet.regulatoryElements:
                if not _is_traffic_light_regulatory_element(reg_elem):
                    continue
                try:
                    rid = int(reg_elem.id)
                except Exception:
                    continue
                if rid in self.excluded_group_ids:
                    continue
                self.lanelet_group_id[int(lanelet.id)] = rid
                break

    def _build_crossing_info(self, map_file):
        member_role = "refers" if self.crossing_reference == "bulb" else "ref_line"
        reference_latlon = _parse_reference_latlon(map_file, member_role)

        group_id_to_lanelet_id = {}
        for lane_id, gid in self.lanelet_group_id.items():
            group_id_to_lanelet_id.setdefault(gid, lane_id)

        for gid, lane_id in group_id_to_lanelet_id.items():
            rid_str = str(gid)
            latlon = reference_latlon.get(rid_str)
            if latlon is None:
                continue
            try:
                gps = core.GPSPoint(latlon[0], latlon[1], 0.0)
                p = self.projector.forward(gps)
                mid_x, mid_y = p.x, p.y
            except Exception as e:
                rospy.logwarn(f"[lanelet_route_matcher] group_id={gid} 기준점 투영 실패: {e}")
                continue

            try:
                lanelet = self.map.laneletLayer[lane_id]
                centerline = list(lanelet.centerline)
                direction = _local_tangent_direction(centerline, mid_x, mid_y)
                if direction is None:
                    continue
                dir_x, dir_y = direction
            except Exception as e:
                rospy.logwarn(f"[lanelet_route_matcher] group_id={gid} 진행방향 계산 실패: {e}")
                continue

            self.crossing_info[gid] = (mid_x, mid_y, dir_x, dir_y)

    def start_subscribers(self, route_topic, odom_topic, route_state_topic="/planning/route_state"):
        if not self.available:
            return
        from autoware_planning_msgs.msg import LaneletRoute, RouteState
        from nav_msgs.msg import Odometry

        self._sub_route = rospy.Subscriber(route_topic, LaneletRoute, self._route_callback, queue_size=1)
        self._sub_route_state = rospy.Subscriber(
            route_state_topic, RouteState, self._route_state_callback, queue_size=1
        )
        self._sub_odom = rospy.Subscriber(odom_topic, Odometry, self._odom_callback, queue_size=1)
        rospy.loginfo(
            f"[lanelet_route_matcher] 구독 시작: {route_topic}, {route_state_topic}, {odom_topic}"
        )

    def _route_callback(self, msg):
        segments = []
        for seg in msg.segments:
            ids = [int(seg.preferred_primitive.id)] + [
                int(p.id) for p in seg.primitives if int(p.id) != int(seg.preferred_primitive.id)
            ]
            segments.append(ids)
        self._route_segments = segments
        rospy.loginfo(f"[lanelet_route_matcher] 새 route 수신: {len(segments)}개 구간")

    def _route_state_callback(self, msg):
        from autoware_planning_msgs.msg import RouteState

        self._route_active = msg.state in (RouteState.SET, RouteState.REROUTING)
        if not self._route_active:
            self._route_segments = None

    def _odom_callback(self, msg):
        self._ego_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._ego_stamp = rospy.Time.now()

    def _is_fresh(self, stamp, max_age_sec):
        if stamp is None:
            return False
        return (rospy.Time.now() - stamp).to_sec() <= max_age_sec

    def _has_crossed(self, gid, ex, ey):
        info = self.crossing_info.get(gid)
        if info is None:
            return False
        mid_x, mid_y, dir_x, dir_y = info
        dot = (ex - mid_x) * dir_x + (ey - mid_y) * dir_y
        return dot > self.crossing_margin_m

    def get_group_id(self):
        if not self.available:
            return None
        if not self._route_active or not self._route_segments:
            return None
        if self._ego_xy is None or not self._is_fresh(self._ego_stamp, self.pose_stale_sec):
            return None

        ex, ey = self._ego_xy

        seen_gids = set()
        for primitive_ids in self._route_segments:
            for lane_id in primitive_ids:
                gid = self.lanelet_group_id.get(lane_id)
                if gid is None or gid in seen_gids:
                    continue
                seen_gids.add(gid)
                if not self._has_crossed(gid, ex, ey):
                    return gid
        return None
