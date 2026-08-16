#!/usr/bin/env python3
"""
lanelet_route_matcher.py (v6 - 판단팀 GO_OUT 연동 분리, 신호등 물리 위치 기준 전환)

/planning/mission_planning/route (autoware_planning_msgs/LaneletRoute) 기준으로,
"현재 에고 위치부터 route를 따라가며 아직 통과하지 않은 첫 신호등"의 group_id를
구하는 모듈.

v5 -> v6 변경점 (2026-08-16, 팀 피드백 반영):
  - 배경: 기존 v4/v5는 전환 기준점으로 lanelet2의 정지선(ref_line)을 쓰고,
    전환 마진(1.79m)도 판단팀 traffic_light.param.yaml의 passed_deadline/
    base_link_to_front/stop_margin을 역산해서 판단팀 GO_OUT 전환 시점에
    맞췄었음.
  - 문제(주상우님 지적): 판단팀 GO_OUT은 애초에 "운전자 신호등"이 아니라
    "보행자 신호등"을 지나는 시점 기준으로 설계된 Autoware 레거시 코드라서,
    저희 목적(운전자 신호등을 지나는 시점)과 대응되지 않음. 판단팀 파라미터를
    정밀하게 따라간 것 자체가 잘못된 기준을 그대로 이식한 것이었음.
  - 수정: 판단팀 GO_OUT 연동을 완전히 끊고, 인지(perception) 목적에 맞는 독립
    기준으로 재설계함.
      1) 기준점: 정지선(ref_line) 대신 신호등이 실제로 서 있는 위치(전구/폴대,
         regulatory element의 refers role 소속 way)를 사용.
      2) 판정 방식: crossing_reference="bulb" 기준점 + lanelet centerline
         진행방향 벡터로, "차량이 신호등 바로 아래를 지나는 지점을 지났는가"를
         정지선 때와 동일한 벡터 내적 방식으로 판정 (기준점만 바뀜, 계산 방식은
         재사용). 즉 신호등 위치를 지나는, 도로 진행방향과 수직인 가상의 선을
         넘었는지로 판정하는 "선 통과" 방식을 채택함 (점 근접이 아니라 선
         통과 방식이, 차로 폭 방향 오차에 덜 민감해서 더 안정적일 것으로 판단).
      3) 전환 마진: 판단팀 값에서 역산하지 않고, 독립적인 기본값을 사용함.
         기본값은 잠정치이며 실측 후 확정 필요 (아래 DEFAULT_CROSSING_MARGIN_M
         주석 참고).
  - 정지선(ref_line) 기준 계산 코드는 옵션으로 남겨둠(crossing_reference="stopline"
    으로 전환 가능) - 신호등 위치 기준이 실측 검증 전 예상과 다르게 나올 경우
    대체 옵션으로 바로 전환 가능하도록.

정석(Autoware) 방식: mission_planner가 lanelet2 RoutingGraph로 이미 계산해서 발행하는
route를 그대로 구독하고, lanelet2 지도 자체에서 lanelet -> regulatory_element(신호등)
연결관계를 읽어서 매핑을 만든다.

신호등 판별: lanelet2 맵 자체에서 regulatory element의 subtype 속성이
"traffic_light"인지 직접 확인 (외부 화이트리스트 불필요). 맵 자체 오류로
확인된 소수의 id는 excluded_group_ids로 받아서 제외.

의존성: lanelet2 python bindings (ros-noetic-lanelet2, ros-noetic-lanelet2-python).
        설치 안 돼있거나 맵 로드 실패 시 이 매처는 항상 None을 반환하도록 만들어져서,
        호출하는 쪽(traffic_light_node.py)이 안전하게 fallback_group_id를 쓸 수 있다.

좌표계: lanelet2 맵을 직접 로드해서 쓰기 때문에, ego pose도 반드시 같은 프레임
       (Autoware "map" 프레임, mission_planner가 쓰는 /localization/kinematic_state와
       동일)이어야 한다.
"""
import math
import xml.etree.ElementTree as ET

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


# 판단팀 파라미터와 무관하게, 저희가 독립적으로 정하는 잠정 마진값(m).
# "신호등 바로 아래를 지나는 가상의 선"을 넘고도 이만큼 더 지나야 다음
# 신호등으로 전환. 0.0으로 두면 신호등 바로 아래를 넘는 즉시 전환.
# TODO(2026-08-16): 아직 실측 검증 안 된 잠정치. 학교에서 실주행하면서
# 카메라가 실제로 신호등을 인식 유지하는 거리와 비교해서 확정 필요.
DEFAULT_CROSSING_MARGIN_M = 0.0


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


def _is_traffic_light_regulatory_element(reg_elem):
    """regulatory element가 신호등(subtype=traffic_light)인지 판별."""
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
    """osm 원본을 직접 파싱해서, regulatory element id -> 지정한 role(member_role)
    way들의 평균 lat/lon 딕셔너리 구축.
    member_role="refers" -> 신호등 전구/폴대 위치
    member_role="ref_line" -> 정지선 위치 (구버전 호환용)"""
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


class LaneletRouteGroupIdMatcher:
    """
    route + lanelet2 맵 기반 group_id 매처. 신호등 물리 위치(기본) 또는 정지선
    통과 여부(옵션)로 신호등 전환을 판단. 판단팀 파라미터와 완전히 독립적.

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
        """
        crossing_reference: "bulb"(기본, 신호등 전구/폴대 위치 기준) 또는
                             "stopline"(정지선 위치 기준, 구버전 호환용).
        """
        if crossing_reference not in ("bulb", "stopline"):
            raise ValueError('crossing_reference는 "bulb" 또는 "stopline"이어야 합니다')

        self.available = False
        self.map = None
        self.lanelet_group_id = {}
        # group_id -> (mid_x, mid_y, dir_x, dir_y) 기준점 + 진행방향(정규화)
        self.crossing_info = {}
        self.excluded_group_ids = set(int(g) for g in (excluded_group_ids or []))
        self.pose_stale_sec = pose_stale_sec
        self.crossing_margin_m = crossing_margin_m
        self.crossing_reference = crossing_reference

        self._route_segments = None  # list[list[int]] (각 segment의 primitive id들, [0]이 preferred)
        self._route_stamp = None  # 참고/디버그용 (매칭 유효성 판단에는 안 씀)
        self._ego_xy = None
        self._ego_stamp = None

        self._sub_route = None
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
                f"전환 마진 {self.crossing_margin_m}m (판단팀 파라미터와 무관한 독립값)"
                + (f" (기준점 정보 없는 신호등 {n_missing}개, 통과판정 불가 -> 항상 미통과로 간주)"
                   if n_missing > 0 else "")
            )
        except Exception as e:
            rospy.logwarn(f"[lanelet_route_matcher] 맵 로드/파싱 실패: {e} -> route 기반 매칭 비활성화")
            self.available = False

    def _build_lanelet_group_id_map(self):
        """lanelet -> regulatory_element(신호등) 연결관계를 스캔해서 lanelet_id -> group_id 매핑 구축.
        신호등 판별은 subtype 속성으로 직접 확인 (외부 화이트리스트 불필요)."""
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
                break  # 신호등 하나 찾으면 이 lanelet은 됐음

    def _build_crossing_info(self, map_file):
        """각 신호등 group_id의 기준점(전구 위치 또는 정지선) + 진행방향 벡터를 미리 계산."""
        member_role = "refers" if self.crossing_reference == "bulb" else "ref_line"
        reference_latlon = _parse_reference_latlon(map_file, member_role)

        # lanelet_id -> group_id 역방향으로, group_id별 대표 lanelet 하나 골라 진행방향 계산용으로 씀
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
                if len(centerline) < 2:
                    continue
                start, end = centerline[0], centerline[-1]
                dx, dy = end.x - start.x, end.y - start.y
                norm = math.hypot(dx, dy)
                if norm < 1e-6:
                    continue
                dir_x, dir_y = dx / norm, dy / norm
            except Exception as e:
                rospy.logwarn(f"[lanelet_route_matcher] group_id={gid} 진행방향 계산 실패: {e}")
                continue

            self.crossing_info[gid] = (mid_x, mid_y, dir_x, dir_y)

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

    def _is_fresh(self, stamp, max_age_sec):
        if stamp is None:
            return False
        return (rospy.Time.now() - stamp).to_sec() <= max_age_sec

    def _has_crossed(self, gid, ex, ey):
        """이 신호등의 기준점(전구 위치 또는 정지선)을 마진까지 포함해서 이미
        통과했는지. 정보 없으면 항상 False(미통과로 간주, 안전)."""
        info = self.crossing_info.get(gid)
        if info is None:
            return False
        mid_x, mid_y, dir_x, dir_y = info
        dot = (ex - mid_x) * dir_x + (ey - mid_y) * dir_y
        return dot > self.crossing_margin_m

    def get_group_id(self):
        """route를 순서대로 훑어서, 아직 기준점을 안 넘은 첫 신호등의 group_id 반환.
        못 구하면 None (호출자가 fallback_group_id 사용)."""
        if not self.available:
            return None
        if not self._route_segments:
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
