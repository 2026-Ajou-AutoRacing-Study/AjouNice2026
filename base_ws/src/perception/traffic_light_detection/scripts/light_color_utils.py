"""
light_color_utils.py

classify_light_color.py의 핵심 색상/모양/방향 판별 로직만 뽑아낸 모듈.
ROS 노드(traffic_light_node.py)에서 import해서 씀.
"""
import cv2
import numpy as np

MIN_LIT_PIXELS = 5
CIRCLE_THRESHOLD = 0.68
MIN_PIXELS_FOR_SHAPE_ANALYSIS = 25
MIN_PIXELS_FOR_DIRECTION = 80

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


def get_color_mask(hsv, color):
    mask = None
    for lower, upper in COLOR_RANGES[color]:
        m = cv2.inRange(hsv, lower, upper)
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    return mask


def classify_shape(mask, pixel_count):
    if pixel_count < MIN_PIXELS_FOR_SHAPE_ANALYSIS:
        return "circle", None

    kernel = np.ones((3, 3), np.uint8)
    smoothed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(smoothed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return "unknown", None

    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    perimeter = cv2.arcLength(c, True)
    if perimeter == 0 or area == 0:
        return "unknown", None

    circularity = 4 * np.pi * area / (perimeter ** 2)
    if circularity >= CIRCLE_THRESHOLD:
        return "circle", None

    if pixel_count < MIN_PIXELS_FOR_DIRECTION:
        return "arrow", None

    x, y, w, h = cv2.boundingRect(c)
    M = cv2.moments(c)
    if M["m00"] == 0:
        return "arrow", None
    cx = M["m10"] / M["m00"]
    bbox_center_x = x + w / 2.0

    if w == 0 or abs(cx - bbox_center_x) < 0.08 * w:
        return "arrow", None

    is_left = cx > bbox_center_x
    return "arrow", is_left


def classify_crop(crop_bgr):
    """crop_bgr(OpenCV BGR 이미지)을 받아서 (results, counts) 반환.
    results: [(color, shape, is_left), ...] 활성화된 색상마다 하나씩
    counts: {"red": n, "yellow": n, "green": n}
    """
    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    results = []
    counts = {}
    for color in COLOR_RANGES:
        mask = get_color_mask(hsv, color)
        count = int(np.count_nonzero(mask))
        counts[color] = count
        if count >= MIN_LIT_PIXELS:
            shape, is_left = classify_shape(mask, count)
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
