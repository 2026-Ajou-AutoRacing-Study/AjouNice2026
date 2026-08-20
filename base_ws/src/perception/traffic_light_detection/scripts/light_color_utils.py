"""
light_color_utils.py (v11 - MORPH_CLOSE 제거, 업스케일링과 함께 재검증)

classify_light_color.py의 핵심 색상/모양 판별 로직만 뽑아낸 모듈.
ROS 노드(traffic_light_node.py)에서 import해서 씀.

v10 -> v11 변경점 (2026-08-20, 원형+화살표 동시 미검출 원인 규명 후 수정):
  - 문제: "화살표만" 또는 "원형만" 검출되는 케이스(원형+화살표 둘 다 있어야
    정상인데 하나만 잡힘)를 오프라인으로 직접 조사함. HSV 채도/명도, 과다노출
    여부를 확인했지만 전부 정상 범위였음. 대신 결정적 단서 발견: 이런 케이스는
    항상 윤곽선이 "1개"만 잡혔고, 그 원형도 값이 0.49~0.67로 임계값(0.68)
    바로 아래에 몰려 있었음. 마스크를 직접 이미지로 저장해서 눈으로 확인한
    결과, 원형과 화살표 전구가 물리적으로 붙어서 "하나의 이어진 흰 덩어리"로
    보였음 - MORPH_CLOSE(2x2 커널)가 두 전구 사이 좁은 틈을 여전히 메워서
    하나의 윤곽선으로 합쳐버리고 있었음.
  - v6에서 한 번 MORPH_CLOSE를 제거했다가 노이즈로 마스크가 조각나는 부작용
    으로 롤백했었는데, 그때는 업스케일링(v10) 적용 전이었음. 업스케일링이
    먼저 적용된 지금 상태에서 다시 제거를 테스트한 결과, 그때와 달리 노이즈
    부작용 없이 순수하게 전구 병합만 해소됨.
  - 오프라인 실측(8707장, 초록 검출 3644장 기준): "원형+화살표 둘 다 정상
    검출" 비율이 2.9% -> 15.4%로 5배 이상 증가.
"""
import cv2
import numpy as np

MIN_LIT_PIXELS = 5
CIRCLE_THRESHOLD = 0.68
MIN_PIXELS_FOR_SHAPE_ANALYSIS = 25

UPSCALE_FACTOR = 3

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
    """v11: 모폴로지 연산(MORPH_CLOSE)을 쓰지 않음 - 원형과 화살표 전구가
    물리적으로 가까이 붙어 있어서, 이 연산이 둘을 하나의 윤곽선으로 합쳐버리는
    문제가 확인됨. 업스케일링(v10)이 이미 적용된 상태라 노이즈로 인한 마스크
    조각남 부작용은 실측(8707장)으로 확인되지 않음."""
    if pixel_count < MIN_PIXELS_FOR_SHAPE_ANALYSIS:
        return [("circle", None)]

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
    v11: 모폴로지 연산 제거."""
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
