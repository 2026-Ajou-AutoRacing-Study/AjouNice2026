"""
light_color_utils.py (v10 - 업스케일링 전처리 추가)

classify_light_color.py의 핵심 색상/모양 판별 로직만 뽑아낸 모듈.
ROS 노드(traffic_light_node.py)에서 import해서 씀.

v9 -> v10 변경점 (2026-08-19, 오프라인 실측으로 효과 검증된 개선):
  - 시장조사(저해상도 이미지 분류 개선 기법)에서 발견한 "업스케일링(cubic
    interpolation) 전처리" 후보를 실제로 적용하고, 기존에 저장해둔 크롭
    8707장으로 적용 전/후를 오프라인 비교함.
  - 결과: 불가능한 신호 조합(원형+원형 등) 발생률이 15.1% -> 3.7%로 감소
    (약 1/4 수준). 원본에서 오류였던 1318장 중 1056장(80%)이 업스케일링만으로
    정상으로 바뀜. 반대로 정상이 오류가 된 경우는 60장뿐(개선:부작용 비율
    약 17:1). 실전 반영함.
  - 구현: classify_crop 시작 시 HSV 변환 전에 크롭을 3배(cubic) 확대. 이후
    로직(색상 마스크, 원형도 계산 등)은 전혀 안 건드림.
  - 참고: 확대율(3배)은 오프라인 실험에서 쓴 값 그대로이며, 다른 배율과의
    비교는 안 함. 처리시간 증가는 실주행에서 확인 필요.
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

_MORPH_KERNEL = np.ones((2, 2), np.uint8)


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
    if pixel_count < MIN_PIXELS_FOR_SHAPE_ANALYSIS:
        return [("circle", None)]

    smoothed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH_KERNEL)
    contours, _ = cv2.findContours(smoothed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
    """crop_bgr(OpenCV BGR 이미지)을 받아서 (results, counts) 반환.
    v10: HSV 변환 전에 크롭을 UPSCALE_FACTOR배로 확대(cubic interpolation)."""
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
