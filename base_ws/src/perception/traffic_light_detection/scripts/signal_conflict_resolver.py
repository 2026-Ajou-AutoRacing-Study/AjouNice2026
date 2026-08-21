#!/usr/bin/env python3
"""
signal_conflict_resolver.py (v6 - 화살표 전용 gid에 한해 circle을 arrow로 강제 대체)

v5 -> v6 변경점 (2026-08-20, D구간 정답 기준 재정정 이후):
  - D구간(gid=201326)이 실제로는 직진/우회전/좌회전이 다 이어지는 차로지만,
    팀의 실제 주행 계획상 이 신호등에서는 좌회전만 사용한다는 게 확인됨.
  - ground truth 데이터(768장, 화살표가 실제로 켜진 것을 육안 확인한 구간)
    분석 결과, red+green_circle(화살표 없이 원형만) 오탐이 16.5%(raw)로
    가장 큰 실질 오류였음. 기존 규칙(v4/v5)은 이 경우 green_circle을 그냥
    삭제만 해서, 화살표 정보가 완전히 사라진 채로 발행됨.
  - 안전성 확인: "빨강 없이 green_circle만 단독 검출"되는 위험한 케이스
    (진짜 직진 허용으로 오인식될 수 있는 경우)는 768장 중 0건. green_circle은
    항상 red와 함께만 나타나므로, circle을 arrow로 강제 대체해도 새로운
    위험(직진 오인식)이 생기지 않는다는 것을 실측으로 확인.
  - 수정: 명시적으로 확인한 gid(ARROW_ONLY_GIDS)에 한해서만, red+green_circle
    충돌 시 green_circle을 삭제하는 대신 green_arrow(좌회전)로 강제 대체.
    목록에 없는 gid는 기존 동작(단순 삭제) 그대로 유지.
"""

# 팀이 육안으로 확인한, 실제로 화살표(좌회전)만 사용하는 gid 목록.
# 등록 전에 반드시: (1) 육안으로 화살표 전용 운영 확인, (2) "빨강 없이
# circle만 단독 검출"되는 위험 케이스가 실측 데이터에서 0%에 가까운지 확인.
ARROW_ONLY_GIDS = {201326}  # D구간, 2026-08-20 실측 확인(red+green_circle 16.5%, 위험 케이스 0/768)


def _has(color_results, color, shape):
    return any(c == color and s == shape for c, s, _ in color_results)


def has_impossible_conflict(color_results):
    has_red_circle = _has(color_results, "red", "circle")
    has_green_circle = _has(color_results, "green", "circle")
    has_yellow_circle = _has(color_results, "yellow", "circle")
    has_green_arrow = _has(color_results, "green", "arrow")

    red_green_collision = has_red_circle and has_green_circle
    red_yellow_collision = has_red_circle and has_yellow_circle
    fully_orphan_arrow = has_green_arrow and not has_red_circle and not has_green_circle
    return red_green_collision or red_yellow_collision or fully_orphan_arrow


def resolve_conflict(color_results, gid=None):
    """gid: 현재 보고 있는 신호등의 group_id. ARROW_ONLY_GIDS에 등록된 gid면,
    red+green(circle) 충돌 시 green(circle)을 단순 삭제하는 대신 green(arrow)로
    강제 대체함."""
    if not has_impossible_conflict(color_results):
        return color_results

    fixed = list(color_results)
    has_red_circle = _has(fixed, "red", "circle")
    has_green_circle = _has(fixed, "green", "circle")
    has_yellow_circle = _has(fixed, "yellow", "circle")
    has_green_arrow = _has(fixed, "green", "arrow")

    if has_red_circle and has_green_circle:
        fixed = [(c, s, l) for c, s, l in fixed if not (c == "green" and s == "circle")]
        if gid in ARROW_ONLY_GIDS and not has_green_arrow:
            fixed.append(("green", "arrow", True))
        has_green_circle = False

    if has_red_circle and has_yellow_circle:
        fixed = [(c, s, l) for c, s, l in fixed if not (c == "yellow" and s == "circle")]

    has_green_arrow = _has(fixed, "green", "arrow")
    if has_green_arrow and not has_red_circle and not has_green_circle:
        fixed.append(("green", "circle", None))

    return fixed
