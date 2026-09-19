#!/usr/bin/env python3
import sys
import glob
import os

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


def classify_shapes_multi(mask, pixel_count):
    if pixel_count < MIN_PIXELS_FOR_SHAPE_ANALYSIS:
        return [("circle", None)]
    smoothed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH_KERNEL)
    contours, _ = cv2.findContours(smoothed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return [("circle", None)]
    min_contour_area = MIN_PIXELS_FOR_SHAPE_ANALYSIS / 2.0
    found_circle, found_arrow = False, False
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
    return results if results else [("circle", None)]


def classify_crop(crop_bgr):
    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    results, counts = [], {}
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
            results.append((color, "circle", None))
    return results, counts


def label_for(results):
    if not results:
        return "off"
    parts = []
    for color, shape, is_left in results:
        if shape == "arrow":
            parts.append(f"{color}_left" if is_left else f"{color}_right")
        else:
            parts.append(color)
    return "+".join(parts)


def main():
    if len(sys.argv) < 2:
        print("사용법: python3 diagnose_crops_offline.py <크롭_폴더> [출력파일.png]")
        sys.exit(1)

    folder = os.path.expanduser(sys.argv[1])
    out_path = sys.argv[2] if len(sys.argv) > 2 else "diagnosis.png"

    files = sorted(glob.glob(os.path.join(folder, "*.png")))
    if not files:
        print(f"'{folder}'에서 png 파일을 못 찾았어요.")
        sys.exit(1)

    print(f"총 {len(files)}장 발견")

    infos = []
    for f in files:
        img = cv2.imread(f)
        if img is None:
            continue
        h, w = img.shape[:2]
        infos.append((h * w, f, img))
    infos.sort(key=lambda x: x[0])

    n_samples = min(48, len(infos))
    step = max(1, len(infos) // n_samples)
    sampled = infos[::step][:n_samples]

    tile_h, tile_w = 140, 200
    cells = []
    for area, f, img in sampled:
        h, w = img.shape[:2]
        results, counts = classify_crop(img)
        label = label_for(results)

        scale = min((tile_w - 10) / w, (tile_h - 40) / h)
        scale = max(scale, 1.0)
        resized = cv2.resize(img, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_NEAREST)

        tile = np.full((tile_h, tile_w, 3), 40, dtype=np.uint8)
        rh, rw = resized.shape[:2]
        y_off = 35
        x_off = max(0, (tile_w - rw) // 2)
        tile[y_off:y_off + min(rh, tile_h - y_off), x_off:x_off + min(rw, tile_w - x_off)] = \
            resized[:min(rh, tile_h - y_off), :min(rw, tile_w - x_off)]

        cv2.putText(tile, f"{w}x{h}={area}px", (3, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(tile, label, (3, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(tile, f"r{counts.get('red',0)} g{counts.get('green',0)}", (3, tile_h - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1, cv2.LINE_AA)
        cells.append(tile)

    n_cols = 6
    n_rows = (len(cells) + n_cols - 1) // n_cols
    grid = np.full((n_rows * tile_h, n_cols * tile_w, 3), 20, dtype=np.uint8)
    for i, tile in enumerate(cells):
        r, c = divmod(i, n_cols)
        grid[r * tile_h:(r + 1) * tile_h, c * tile_w:(c + 1) * tile_w] = tile

    cv2.imwrite(out_path, grid)
    print(f"진단 이미지 저장됨: {out_path} ({len(cells)}장 샘플링)")

    print()
    print("=== 크기 구간별 통계 (전체 데이터 기준) ===")
    buckets = {}
    for area, f, img in infos:
        results, counts = classify_crop(img)
        has_red = any(c == "red" for c, s, l in results)
        has_green = any(c == "green" for c, s, l in results)
        bucket = (area // 200) * 200
        b = buckets.setdefault(bucket, {"count": 0, "both": 0, "red_only": 0, "green_only": 0, "neither": 0})
        b["count"] += 1
        if has_red and has_green:
            b["both"] += 1
        elif has_red:
            b["red_only"] += 1
        elif has_green:
            b["green_only"] += 1
        else:
            b["neither"] += 1

    for k in sorted(buckets.keys()):
        b = buckets[k]
        pct = 100 * b["both"] / b["count"]
        print(f"{k}~{k+199}px: 총{b['count']}개 | 둘다:{b['both']}({pct:.0f}%) "
              f"빨강만:{b['red_only']} 초록만:{b['green_only']} 없음:{b['neither']}")


if __name__ == "__main__":
    main()
