"""
compare_traffic_lights_v4.py

v1~v3는 centroid 기반으로 평행이동/회전을 "추정"했는데 잘 안 맞았음.
mgeo_map/global_info.json 확인 결과, 정확한 원점 정보가 있는 걸 발견:
  local_origin_in_global (UTM52) = [302595.0, 4124145.0]
  workspace_origin (아마 lat/lon/alt 또는 다른 단위) = [40.0, -174.0, -925.0]

lanelet2 원점(map_projector_info_v1.yaml)을 UTM52로 변환한 값과 비교하니
차이가 (-131.56, -545.91)로 나옴 -> 이게 "추정값"이 아니라 실제 known offset일
가능성이 높음.

이 스크립트는 "추정" 대신 이 정확한 오프셋을 직접 적용해서, 몇 가지 가능한 조합
(y축 반전 여부, workspace_origin 적용 여부)을 전부 시도해보고 각각의 평균오차를
비교. 오차가 급격히 줄어드는 조합이 정답.
"""
import json
import xml.etree.ElementTree as ET

import numpy as np
from pyproj import Transformer
from scipy.optimize import linear_sum_assignment

ORIGIN_LAT, ORIGIN_LON = 37.2380038082, 126.7730819386
LANELET_OSM = "lanelet_map/lanelet2_map_v74.osm"
MGEO_NODE_SET = "mgeo_map/node_set.json"
MGEO_GLOBAL_INFO = "mgeo_map/global_info.json"


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
        positions.append({"id": rel.get("id"), "x": x, "y": y})
    return positions, (origin_e, origin_n)


def load_mgeo_raw_points(node_set_path):
    """y반전 없이 원본 그대로 로드 (부호 실험은 아래서 따로)"""
    mgeo_nodes = json.load(open(node_set_path))
    return [
        {"id": n["idx"], "x": n["point"][0], "y": n["point"][1]}
        for n in mgeo_nodes
        if n.get("node_type") == "1" and n.get("traffic_light_id")
    ]


def hungarian_avg_dist(la_xy, mg_xy):
    cost = np.linalg.norm(la_xy[:, None, :] - mg_xy[None, :, :], axis=2)
    row_idx, col_idx = linear_sum_assignment(cost)
    dists = cost[row_idx, col_idx]
    return dists.mean(), dists, row_idx, col_idx


def main():
    reg_positions, (lanelet_origin_e, lanelet_origin_n) = load_lanelet_regulatory_elements(LANELET_OSM)
    mgeo_raw = load_mgeo_raw_points(MGEO_NODE_SET)
    global_info = json.load(open(MGEO_GLOBAL_INFO))

    print(f"[lanelet2] regulatory elements: {len(reg_positions)}")
    print(f"[mgeo] traffic light nodes: {len(mgeo_raw)}")

    local_origin_in_global = global_info["local_origin_in_global"]  # UTM52 [e, n, z]
    workspace_origin = global_info.get("workspace_origin")  # 용도 불명, 실험 대상

    mg_origin_e, mg_origin_n = local_origin_in_global[0], local_origin_in_global[1]

    # lanelet2 원점을 UTM52로 이미 계산해둠(위에서) -> 두 원점의 실제 오프셋
    offset_e = mg_origin_e - lanelet_origin_e
    offset_n = mg_origin_n - lanelet_origin_n
    print(f"\nlanelet2 원점(UTM52): ({lanelet_origin_e:.2f}, {lanelet_origin_n:.2f})")
    print(f"mgeo local_origin_in_global(UTM52): ({mg_origin_e:.2f}, {mg_origin_n:.2f})")
    print(f"오프셋(mgeo원점 - lanelet2원점): ({offset_e:.2f}, {offset_n:.2f})")

    la_xy = np.array([[p["x"], p["y"]] for p in reg_positions])
    mg_raw_xy = np.array([[p["x"], p["y"]] for p in mgeo_raw])

    print("\n===== 조합별 평균오차 비교 =====")

    candidates = {}

    # 조합 1: mgeo_local(x,y) + offset 그대로 (y반전 없음)
    candidates["1) offset만, y반전 없음"] = mg_raw_xy + np.array([offset_e, offset_n])

    # 조합 2: mgeo_local(x,-y) + offset (기존 스크립트가 하던 y반전 적용)
    mg_yflip = mg_raw_xy * np.array([1, -1])
    candidates["2) offset + y반전"] = mg_yflip + np.array([offset_e, offset_n])

    # 조합 3: offset의 y부호 반전 (오프셋 계산 자체에 y반전 적용 여부 실험)
    candidates["3) offset(y부호반전) + y반전 없음"] = mg_raw_xy + np.array([offset_e, -offset_n])

    # 조합 4: offset(y부호반전) + y반전
    candidates["4) offset(y부호반전) + y반전"] = mg_yflip + np.array([offset_e, -offset_n])

    for name, mg_transformed in candidates.items():
        avg, dists, row_idx, col_idx = hungarian_avg_dist(la_xy, mg_transformed)
        n_under_5 = int((dists < 5.0).sum())
        n_under_10 = int((dists < 10.0).sum())
        print(f"{name}: 평균오차={avg:8.2f}m / 5m이내={n_under_5}/68 / 10m이내={n_under_10}/68")

    # ---- 조합 1(offset만, y반전 없음)이 정답으로 확인됨 -> 세부 매칭 결과 출력 ----
    best_mg = candidates["1) offset만, y반전 없음"]
    avg, dists, row_idx, col_idx = hungarian_avg_dist(la_xy, best_mg)
    pairs = [
        (reg_positions[i]["id"], mgeo_raw[j]["id"], d)
        for i, j, d in zip(row_idx, col_idx, dists)
    ]
    print(f"\n===== 최종 채택(조합1) 세부 매칭, 평균오차 {avg:.2f}m =====")
    print("거리 큰 순 (outlier 후보):")
    for lid, mid, d in sorted(pairs, key=lambda x: -x[2]):
        print(f"  lanelet2 id={lid:>8} <-> mgeo id={mid:<15} dist={d:.2f}m")


if __name__ == "__main__":
    main()
