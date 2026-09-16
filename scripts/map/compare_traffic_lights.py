"""
lanelet2(osm) 맵과 mgeo 맵의 신호등 위치/개수를 비교하는 스크립트.

사용법: kcity_map/ 디렉토리에서 실행
  cd ~/AjouNice2026/kcity_map
  python3 ../scripts/map/compare_traffic_lights.py

lanelet2: <relation subtype="traffic_light"> regulatory element 단위로 집계
          (refers role way들의 평균 lat/lon -> map_origin 기준 로컬 xy 변환)
mgeo:     node_type=='1' AND traffic_light_id 존재하는 노드 단위로 집계
          (point의 y축 부호는 lanelet2와 반대라 -y로 반전)

두 맵 원점이 달라서 전체 중심점(centroid) 차이만큼 평행이동 보정 후 매칭.
"""
import json
import math
import xml.etree.ElementTree as ET
from pyproj import Transformer

# map_projector_info_v1.yaml 기준
ORIGIN_LAT, ORIGIN_LON = 37.2380038082, 126.7730819386
LANELET_OSM = "lanelet_map/lanelet2_map_v71.osm"
MGEO_NODE_SET = "mgeo_map/node_set.json"


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
    return positions


def load_mgeo_traffic_light_nodes(node_set_path):
    mgeo_nodes = json.load(open(node_set_path))
    return [
        {"id": n["idx"], "x": n["point"][0], "y": -n["point"][1]}
        for n in mgeo_nodes
        if n.get("node_type") == "1" and n.get("traffic_light_id")
    ]


def match_nearest(la_list, mg_list):
    used, results = set(), []
    for la in la_list:
        best, best_d = None, None
        for i, mg in enumerate(mg_list):
            if i in used:
                continue
            d = math.hypot(la["x"] - mg["x"], la["y"] - mg["y"])
            if best_d is None or d < best_d:
                best_d, best = d, i
        if best is not None:
            used.add(best)
            results.append((la["id"], mg_list[best]["id"], best_d))
    return results


def main():
    reg_positions = load_lanelet_regulatory_elements(LANELET_OSM)
    mgeo_positions = load_mgeo_traffic_light_nodes(MGEO_NODE_SET)
    print(f"[lanelet2] regulatory elements: {len(reg_positions)}")
    print(f"[mgeo] node_type==1 AND traffic_light_id set: {len(mgeo_positions)}")

    cx_la = sum(p["x"] for p in reg_positions) / len(reg_positions)
    cy_la = sum(p["y"] for p in reg_positions) / len(reg_positions)
    cx_mg = sum(p["x"] for p in mgeo_positions) / len(mgeo_positions)
    cy_mg = sum(p["y"] for p in mgeo_positions) / len(mgeo_positions)
    dx, dy = cx_la - cx_mg, cy_la - cy_mg
    for p in mgeo_positions:
        p["x"] += dx
        p["y"] += dy

    res = match_nearest(reg_positions, mgeo_positions)
    avg = sum(r[2] for r in res) / len(res)
    print(f"\nafter translation only: matched {len(res)} pairs, avg dist {avg:.2f} m")
    for r in sorted(res, key=lambda x: -x[2]):
        print(f"  {r[0]:>8} <-> {r[1]:<15} dist={r[2]:.2f}m")


if __name__ == "__main__":
    main()
