"""
build_traffic_light_group_id_table.py

lanelet2 맵의 68개 신호등 regulatory element 위치를, MORAI Ego_topic이 쓰는
좌표계(mgeo_map 원본 local 좌표계와 동일하다고 가정)로 변환해서 룩업 테이블(json)로 저장.

배경 (2026-08-11 검증 결과):
  - lanelet2 원점(UTM52)과 mgeo local_origin_in_global(UTM52) 사이에
    정확한 오프셋 (131.56, 545.91) 확인됨
  - lanelet2_xy = mgeo_local_xy + offset  (y축 반전 불필요, 평균오차 13m로 검증됨)
  - 역으로: mgeo_local_xy = lanelet2_xy - offset
  - Ego_topic의 position이 mgeo_local과 동일 좌표계라는 가정 하에,
    lanelet2 신호등 좌표에서 offset을 빼면 Ego 좌표계 기준 위치가 나옴

주의(가정 검증 필요):
  이 스크립트가 만든 테이블에서, 실제 정지선에 차량을 세우고 캡처한 Ego position
  (A/B/C/D 구간에서 이미 확보한 좌표들)과 가장 가까운 신호등 항목 간 거리가
  대략 5~20m 이내(신호등이 정지선보다 약간 앞/옆에 있는 정도)로 나오는지 확인 필요.
  만약 수십~수백 m씩 차이나면 Ego_topic이 다른 좌표계를 쓰는 것이므로 가정이 틀린 것.

알려진 맵 오류 (Vector Map Builder로 확인됨):
  - id 201395, 201396: lanelet2 맵 자체의 신호등 전구 좌표가 도로에서 멀리
    떨어진 곳에 잘못 찍혀 있음. 이 두 id는 EXCLUDE_IDS에 포함해서 테이블에서
    제외 (맵 수정 전까지는 group_id 매칭 대상에서 빼는 게 안전).

사용법: kcity_map/ 디렉토리에서 실행
  python3 ../scripts/map/build_traffic_light_group_id_table.py
  -> traffic_light_group_id_table.json 생성됨
"""
import json
import xml.etree.ElementTree as ET

from pyproj import Transformer

ORIGIN_LAT, ORIGIN_LON = 37.2380038082, 126.7730819386
LANELET_OSM = "lanelet_map/lanelet2_map_v64.osm"
MGEO_GLOBAL_INFO = "mgeo_map/global_info.json"
OUTPUT_JSON = "traffic_light_group_id_table.json"

# Vector Map Builder로 확인된, 맵 자체 오류로 판단된 id들. 맵 수정 전까지 제외.
EXCLUDE_IDS = {"201395", "201396"}


def load_lanelet_regulatory_elements(osm_path):
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)
    origin_e, origin_n = transformer.transform(ORIGIN_LON, ORIGIN_LAT)

    tree = ET.parse(osm_path)
    root = tree.getroot()
    nodes = {nd.get("id"): (float(nd.get("lat")), float(nd.get("lon"))) for nd in root.findall("node")}
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in root.findall("way")}

    def way_xy(wid):
        coords = [nodes[r] for r in ways.get(wid, []) if r in nodes]
        if not coords:
            return None
        lat = sum(c[0] for c in coords) / len(coords)
        lon = sum(c[1] for c in coords) / len(coords)
        e, n = transformer.transform(lon, lat)
        return (e - origin_e, n - origin_n)

    positions = []
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("subtype") != "traffic_light":
            continue
        refers = [m.get("ref") for m in rel.findall("member") if m.get("role") == "refers"]
        pts = [p for p in (way_xy(w) for w in refers) if p]
        if not pts:
            continue
        x = sum(p[0] for p in pts) / len(pts)
        y = sum(p[1] for p in pts) / len(pts)
        positions.append({"id": rel.get("id"), "lanelet2_x": x, "lanelet2_y": y})
    return positions


def main():
    reg_positions = load_lanelet_regulatory_elements(LANELET_OSM)
    global_info = json.load(open(MGEO_GLOBAL_INFO))

    transformer = Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)
    lanelet_origin_e, lanelet_origin_n = transformer.transform(ORIGIN_LON, ORIGIN_LAT)
    mg_origin_e, mg_origin_n = global_info["local_origin_in_global"][:2]

    offset_e = mg_origin_e - lanelet_origin_e
    offset_n = mg_origin_n - lanelet_origin_n

    table = []
    excluded = []
    for p in reg_positions:
        ego_x = p["lanelet2_x"] - offset_e
        ego_y = p["lanelet2_y"] - offset_n
        entry = {
            "group_id": int(p["id"]),
            "ego_frame_x": round(ego_x, 3),
            "ego_frame_y": round(ego_y, 3),
        }
        if p["id"] in EXCLUDE_IDS:
            excluded.append(entry)
        else:
            table.append(entry)

    with open(OUTPUT_JSON, "w") as f:
        json.dump({"traffic_lights": table, "excluded_map_error_ids": excluded}, f, indent=2)

    print(f"총 {len(reg_positions)}개 중 {len(table)}개 테이블에 포함, {len(excluded)}개 제외(맵 오류: {EXCLUDE_IDS})")
    print(f"저장 완료: {OUTPUT_JSON}")
    print("\n샘플 5개:")
    for e in table[:5]:
        print(f"  group_id={e['group_id']}: ego_frame=({e['ego_frame_x']}, {e['ego_frame_y']})")

    print("\n검증 방법: 이미 확보한 구간별 정지선 Ego 좌표(A/B/C/D)와 이 테이블에서 가장 가까운 "
          "항목 간 거리를 확인해보세요. 5~20m 이내면 좌표계 가정이 맞는 것으로 판단 가능합니다.")


if __name__ == "__main__":
    main()
