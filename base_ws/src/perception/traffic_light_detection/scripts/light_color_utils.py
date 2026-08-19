"""
light_color_utils.py (v9 - 근거 부족했던 크롭 최소크기 게이트 제거)

classify_light_color.py의 핵심 색상/모양 판별 로직만 뽑아낸 모듈.
ROS 노드(traffic_light_node.py)에서 import해서 씀.

v8 -> v9 변경점 (2026-08-19, 실측 데이터로 v8의 가정이 틀렸음을 확인):
  - v8에서 "크롭이 700px 미만이면 색 정보가 신뢰 불가"라는 가정으로 최소 크기
    게이트를 추가했었음. 근데 정지 없이 한 번에 접근하는 깨끗한 주행으로
    994장을 재측정한 결과, 600px~4400px 전 구간에서 예외 없이 "빨강만"
    검출됐고, 오히려 "초록도 같이 검출"된 구간은 그보다 작은 400~599px
    구간에서만 나타남. 즉 크롭이 클수록 정확도가 좋아진다는 가정 자체가
    이번 데이터로는 반박됨.
  - 재해석: 이 패턴은 검출 실패가 아니라, 접근하는 동안 신호등이 실제로
    좌회전 화살표 구간에서 빨강 단독 구간으로 전환됐을 가능성이 높음(흔한
    신호 체계 - 좌회전 허용 시간이 끝나면 화살표가 꺼지고 빨강만 남는 방식).
    즉 파이프라인이 처음부터 끝까지 정확하게 인식하고 있었을 가능성이 높음.
  - 수정: 근거가 반박된 MIN_CROP_AREA_FOR_CLASSIFICATION 게이트를 제거함.
    v8이 오히려 "초록도 있었던"(진짜 정보가 있는) 400~599px 구간을 걸러내는
    역효과를 냈던 것으로 확인됨.
  - 결론이 100% 확정된 건 아님 - MORAI 화면의 실제 신호등 변화를 실시간으로
    눈으로 보면서 디버그 라벨과 동시에 비교하는 것이 가장 확실한 다음 검증
    단계로 남아있음.
"""
import cv2
import numpy as np

MIN_LIT_PIXELS = 5
CIRCLE_THRESHOLD = 0.68
MIN_PIXELS_FOR_SHAPE_ANALYSIS = 25

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
    """초록처럼 원형+화살표(좌회전)가 동시에 켜질 수 있는 색상용. 마스크 안의
    유의미한 윤곽선을 전부 훑어서, 원형 덩어리와 화살표 덩어리를 각각 독립적으로
    검출. 둘 다 있으면 둘 다 반환.
    반환: [(shape, is_left), ...] - 최대 2개(circle, arrow). 아무것도 못 찾으면
    [("circle", None)]로 안전하게 폴백."""
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
    results: [(color, shape, is_left), ...] - 초록은 원형+화살표가 동시에
    검출되면 두 개의 항목(green/circle, green/arrow)이 같이 들어갈 수 있음.
    counts: {"red": n, "yellow": n, "green": n}
    """
    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
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
    """boxes: [(cx, cy, w, h, extra), ...] (정규화 0~1 좌표, extra는 원래 데이터 그대로 딸려옴)
    filter_center_light.py와 동일한 위치 휴리스틱으로 "우리 차로" 박스 1개 선택.
    없으면 None.
    """
    if not boxes:
        return None
    candidates = [b for b in boxes if abs(b[0] - 0.5) <= center_tolerance_ratio]
    if not candidates:
        candidates = boxes

    def score(b):
        cx, cy = b[0], b[1]
        return abs(cx - 0.5) + y_weight * cy

    return min(candidates, key=score)
