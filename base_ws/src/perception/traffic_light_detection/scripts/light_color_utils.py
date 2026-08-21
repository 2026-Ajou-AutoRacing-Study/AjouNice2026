"""
light_color_utils.py (v12 - MORPH_OPEN 4x4 커널 추가)

classify_light_color.py의 핵심 색상/모양 판별 로직만 뽑아낸 모듈.
ROS 노드(traffic_light_node.py)에서 import해서 씀.

v11 -> v12 변경점 (2026-08-21, A/B구간 오탐 문제 실측 후 수정):
  - 문제: A/B구간에서는 원형 초록만 켜져 있어야 하는 상황인데, 화살표까지
    같이 검출되는(both) 오탐이 발생. 원본 마스크에 압축/노이즈로 생긴 작은
    파편이 별도 윤곽선으로 잡혀서, 그 파편이 원형도 낮게 나와 화살표로
    오분류되는 것으로 확인(v11에서 MORPH_CLOSE 제거 후 노출된 부작용).
  - 시도: 노이즈 파편만 지우고 큰 덩어리는 보존하는 MORPH_OPEN을 도입,
    커널 크기(2x2~8x8)를 실측 데이터로 스윕 테스트.
  - 결과: 커널 크기에 따라 최적점이 데이터셋마다 다름
    - 전체 데이터(8707장): 4x4가 "둘 다 검출" 최고점(20.9%), 5x5부터 하락
    - D구간(25205장): 2x2~3x3이 오히려 나음, 4x4는 소폭 하락(7.2%->6.4%)
    - A구간(497장, 원형 단독 ground truth): 4x4에서 오탐 0%(연산없음 0.2%)
    - B구간(387장, 원형 단독 ground truth): 4x4에서 정답률 45.2%->83.7%,
      오탐 2.3%->0.3%로 대폭 개선 - 가장 극적인 효과
  - 결정: A/B구간에서 실제로 문제가 되던 것(원형 단독인데 화살표까지 잘못
    붙는 오탐)을 직접 겨냥한 ground truth 데이터 기준으로, 4x4가 가장 큰
    개선을 보여서 채택. D구간의 소폭 손해(약 1%p)보다 A/B구간의 큰 개선
    (오탐률 최대 2%p 감소, 정답률 최대 38.5%p 증가)이 더 크다고 판단.
"""
import cv2
import numpy as np

MIN_LIT_PIXELS = 5
CIRCLE_THRESHOLD = 0.68
MIN_PIXELS_FOR_SHAPE_ANALYSIS = 25

UPSCALE_FACTOR = 3

_OPEN_KERNEL = np.ones((4, 4), np.uint8)

COLOR_RANGES = {
    "red": [
        (np.array([0, 100, 100]), np.array([10, 255, 255])),
        (np.array([160, 100, 100]), np.array([179, 255, 255])),
    ],
    "yellow": [
        (np.array([18, 100, 100]), np.array([35, 255, 255])),
    ],
    "green": [
        (np.array([40, 60, 60]), np.array([90, 255, 255])),
    ],
}

ARROW_CAPABLE_COLORS = {"green"}


def get_color_mask(hsv, color):
    mask = None
    for lower, upper in COLOR_RANGES[color]:
        m = cv2.inRange(hsv, lower, upper)
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    return mask


def classify_shape_single(mask, pixel_count):
    if pixel_count < MIN_PIXELS_FOR_SHAPE_ANALYSIS:
        return "circle", None
    return "circle", None


def classify_shapes_multi(mask, pixel_count):
    """v12: MORPH_OPEN(4x4)으로 노이즈 파편을 먼저 지운 뒤 윤곽선을 찾음."""
    if pixel_count < MIN_PIXELS_FOR_SHAPE_ANALYSIS:
        return [("circle", None)]

    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _OPEN_KERNEL)
    contours, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return [("circle", None)]

    min_contour_area = MIN_PIXELS_FOR_SHAPE_ANALYSIS / 2.0

    found_circle = False
    found_arrow = False
    results = []
    for c in sorted(contours, key=cv2.contourArea, reverse=True):
        area = cv2.contourArea(c)
        if area < min_contour_area:
            continue
        perimeter = cv2.arcLength(c, True)
        if perimeter == 0:
            continue
        circularity = 4 * np.pi * area / (perimeter ** 2)
        if circularity >= CIRCLE_THRESHOLD:
            if not found_circle:
                results.append(("circle", None))
                found_circle = True
        else:
            if not found_arrow:
                results.append(("arrow", True))
                found_arrow = True
        if found_circle and found_arrow:
            break

    if not results:
        return [("circle", None)]
    return results


def classify_crop(crop_bgr):
    """v10: HSV 변환 전에 크롭을 UPSCALE_FACTOR배로 확대(cubic interpolation).
    v12: MORPH_OPEN(4x4)으로 노이즈 파편 제거."""
    upscaled = cv2.resize(
        crop_bgr, None, fx=UPSCALE_FACTOR, fy=UPSCALE_FACTOR, interpolation=cv2.INTER_CUBIC
    )
    hsv = cv2.cvtColor(upscaled, cv2.COLOR_BGR2HSV)
    results = []
    counts = {}
    for color in COLOR_RANGES:
        mask = get_color_mask(hsv, color)
        count = int(np.count_nonzero(mask))
        counts[color] = count
        if count < MIN_LIT_PIXELS:
            continue

        if color in ARROW_CAPABLE_COLORS:
            for shape, is_left in classify_shapes_multi(mask, count):
                results.append((color, shape, is_left))
        else:
            shape, is_left = classify_shape_single(mask, count)
            results.append((color, shape, is_left))

    return results, counts


def pick_ego_lane_box(boxes, center_tolerance_ratio=0.20, y_weight=0.6):
    if not boxes:
        return None
    candidates = [b for b in boxes if abs(b[0] - 0.5) <= center_tolerance_ratio]
    if not candidates:
        candidates = boxes

    def score(b):
        cx, cy = b[0], b[1]
        return abs(cx - 0.5) + y_weight * cy

    return min(candidates, key=score)
