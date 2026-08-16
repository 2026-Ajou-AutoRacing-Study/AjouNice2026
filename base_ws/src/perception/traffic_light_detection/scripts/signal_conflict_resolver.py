#!/usr/bin/env python3
"""
signal_conflict_resolver.py

물리적으로 불가능한 신호 조합(원형 빨강 + 원형 초록 동시 검출 등)이 오탐으로
나왔을 때, 판단팀에 그대로 보내지 않고 안전하게 보정/필터링하기 위한 모듈.

주의: 이 파일은 뼈대(skeleton)만 구현된 상태입니다. 실제 규칙은 A~D 대회 구간
전수조사(각 신호등이 실제로 어떤 신호 조합을 쓰는지 확인) 완료 후 확정해야 합니다.
지금은 구조만 잡아두고, 조사 결과가 나오면 CONFLICT_RULES만 채우면 되도록
설계했습니다.

배경 (2026-08-16, 주상우님 피드백):
  - red_green(원형 빨강+원형 초록)은 물리적으로 존재할 수 없는 조합. 오탐 발생 시
    그대로 보내면 안 됨.
  - 다만 오탐이라고 무조건 UNKNOWN/빈 상태로 보내서 판단팀이 보수적으로 정지하게
    두면, 차량이 그 자리에서 아무 판단도 못 하고 계속 서있을 위험이 있음
    ("오탐을 오탐이라고만 하고 끝낼 게 아니라, 어떻게 보낼지 폴백이 있어야 함").
  - 한국 신호 체계상 red_green이 나온다면 실질적으로 red+green_left(직진 정지+
    좌회전 허용)일 가능성이 매우 높음. 다만 red+green_right 케이스가 실제로
    존재하는지는 전수조사로 직접 확인 필요 (기본값으로 단정하지 않음).
  - 이 폴백은 어디까지나 안전망이고, 근본 해결은 애초에 오탐이 안 나도록 인지
    정확도(화살표 판정 로직, 2/3번 액션아이템)를 높이는 것.
"""


# 물리적으로 불가능하다고 판단하는 "원형+원형" 동시 조합.
# (원형+화살표 조합, 예: red+green_left는 정상 신호이므로 여기 해당 안 됨)
IMPOSSIBLE_CIRCLE_COMBOS = {
    frozenset({"red", "green"}),
    # TODO(2026-08-16): red+yellow, yellow+green 등 다른 원형+원형 조합도
    # 전수조사 후 물리적으로 불가능하면 여기 추가.
}


def _circle_colors(color_results):
    """color_results: [(color, shape, is_left), ...] -> 원형(circle)으로 검출된
    색상 집합만 추출."""
    return {color for color, shape, _is_left in color_results if shape == "circle"}


def has_impossible_conflict(color_results):
    """물리적으로 불가능한 원형+원형 조합이 검출됐는지 확인."""
    circles = _circle_colors(color_results)
    for combo in IMPOSSIBLE_CIRCLE_COMBOS:
        if combo.issubset(circles):
            return True
    return False


def resolve_conflict(color_results):
    """오탐(불가능한 조합) 발생 시 보정된 color_results를 반환.

    TODO(2026-08-16): 전수조사 완료 전까지는 미구현. 현재는 오탐 여부만 판단하고,
    보정 규칙은 정해지지 않았으므로 호출부에서 별도 처리 필요.

    조사 완료 후 채워야 할 규칙 예시(전수조사 결과에 따라 확정):
      - red_green 오탐 시, 어느 쪽이 더 우세한지(픽셀 수 등)로 판단해서
        red+green_left 등 실제 존재 가능한 조합으로 보정할지
      - 아니면 그 프레임만 UNKNOWN 처리하고 signal_smoother(큐 투표)에 맡겨서
        이후 프레임들로 자연스럽게 걸러지게 할지
      - red+green_right 조합 존재 여부에 따라 위 규칙이 달라질 수 있음
    """
    if not has_impossible_conflict(color_results):
        return color_results

    raise NotImplementedError(
        "물리적으로 불가능한 신호 조합(오탐) 감지됨. "
        "보정 규칙은 A~D 전수조사 완료 후 확정 필요 (signal_conflict_resolver.py 참고)"
    )
