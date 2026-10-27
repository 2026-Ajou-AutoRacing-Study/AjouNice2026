#!/usr/bin/env python3
"""Offline front-camera + actual recorded Planviz viewport, no ROS or driving.

An explicit approximate clock offset is required. No scene is re-simulated and
no missing BEV frame is fabricated from a previous one. Desktop pixels outside
the explicitly approved viewport crop are never included in the output.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess

import record_traffic_light_evidence_video as evidence_video


def screen_index(front_sec, screen_start_front_sec, screen_fps, count):
    elapsed = front_sec-screen_start_front_sec
    if elapsed < 0:
        return None
    index = math.floor(elapsed*screen_fps+.5)
    return index if 0 <= index < count else None


def validate_crop(crop, width, height):
    x, y, w, h = crop
    if min(x, y) < 0 or min(w, h) <= 0 or x+w > width or y+h > height:
        raise ValueError('BEV crop must be a nonempty viewport inside the recorded screen')
    return x, y, w, h


def compose(front, bev, front_sec):
    import cv2
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont
    if front.shape[:2] != (960, 1280):
        raise ValueError('Expected the approved 1280x960 front/footer evidence format')
    canvas = np.zeros((960, 1920, 3), dtype=np.uint8)
    canvas[:, :1280] = front
    if bev is not None:
        factor = min(640/bev.shape[1], 650/bev.shape[0])
        w, h = max(1, round(bev.shape[1]*factor)), max(1, round(bev.shape[0]*factor))
        canvas[45+(650-h)//2:45+(650-h)//2+h, 1280+(640-w)//2:1280+(640-w)//2+w] = cv2.resize(bev, (w,h))
    font = getattr(compose, '_font', None)
    if font is None:
        font = compose._font = ImageFont.truetype(
            '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 23)
    # Draw only the BEV pane; keep the original camera/footer pixels untouched.
    pane = Image.fromarray(canvas[:, 1280:, ::-1].copy())
    draw = ImageDraw.Draw(pane)
    draw.text((16, 5), 'Planviz BEV · 실제 녹화 화면', font=font, fill=(235,235,235))
    if bev is None:
        draw.text((40, 320), '이 시각의 BEV 녹화 없음', font=font, fill=(255,210,100))
    for y, text in [(730, '전방·신호 판단 + 지도·계획 궤적'),
                    (770, '별도 화면 영상과 근사 동기화'),
                    (810, '정밀 지연 측정용 동기화는 아님'),
                    (850, '기존 전방 영상 시각 %.1f초' % front_sec)]:
        draw.text((16,y), text, font=font, fill=(235,235,235))
    canvas[:, 1280:] = np.asarray(pane)[:, :, ::-1]
    return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--front-directory', type=Path, required=True)
    parser.add_argument('--screen-video', type=Path, required=True)
    parser.add_argument('--bev-crop', type=int, nargs=4, required=True, metavar=('X','Y','W','H'))
    parser.add_argument('--screen-start-front-sec', type=float, required=True)
    parser.add_argument('--preview-sec', type=float, default=88.2)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.screen_start_front_sec, args.preview_sec)) or args.preview_sec < 0:
        raise ValueError('Finite timing and nonnegative preview time required')
    output = evidence_video.validate_output(args.output)
    import cv2
    front_path = args.front_directory/'front_evidence.mp4'
    front, screen = cv2.VideoCapture(str(front_path)), cv2.VideoCapture(str(args.screen_video))
    if not front.isOpened() or not screen.isOpened():
        raise ValueError('Both completed source videos must be readable')
    fps, screen_fps = front.get(cv2.CAP_PROP_FPS), screen.get(cv2.CAP_PROP_FPS)
    count = round(screen.get(cv2.CAP_PROP_FRAME_COUNT))
    if fps <= 0 or screen_fps <= 0 or count <= 0:
        raise ValueError('Invalid source frame clock')
    x,y,w,h = validate_crop(args.bev_crop, round(screen.get(cv2.CAP_PROP_FRAME_WIDTH)),
                            round(screen.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    output.mkdir(parents=True)
    command = evidence_video.ffmpeg_arguments(output/'front_planviz.mp4', fps, 'h264_nvenc')
    command[command.index('-video_size')+1] = '1920x960'
    consumed, screen_frame, written = -1, None, 0
    with (output/'ffmpeg.log').open('wb') as log, (output/'frames.jsonl').open('w') as manifest:
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=log)
        try:
            while True:
                ok, image = front.read()
                if not ok:
                    break
                pts = written/fps
                target = screen_index(pts, args.screen_start_front_sec, screen_fps, count)
                if target is not None:
                    while consumed < target:
                        ok, screen_frame = screen.read()
                        if not ok:
                            raise RuntimeError('Recorded BEV screen ended before its reported frame count')
                        consumed += 1
                frame = compose(image, None if target is None else screen_frame[y:y+h,x:x+w], pts)
                process.stdin.write(frame.tobytes())
                manifest.write(json.dumps(dict(frame_index=written, front_pts_sec=pts,
                    screen_frame_index=target, bev_available=target is not None,
                    synchronization='explicit_approximate_offset_not_ROS_frame_sync'))+'\n')
                if written == round(args.preview_sec*fps):
                    cv2.imwrite(str(output/'preview.png'), frame)
                written += 1
        finally:
            front.release()
            screen.release()
            process.stdin.close()
            result = process.wait()
        if result or written == 0:
            raise RuntimeError('Video composition did not complete')
    hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in (front_path, args.screen_video)}
    (output/'metadata.json').write_text(json.dumps(dict(input_sha256=hashes, fps=fps,
        frames=written, viewport_crop=args.bev_crop, screen_start_front_sec=args.screen_start_front_sec,
        synchronization='Approximate alignment of separately recorded screen video; not latency evidence.',
        vehicle_or_ROS_commands=False, front_timeline_unchanged=True), indent=2))
    print(output/'front_planviz.mp4')


if __name__ == '__main__':
    main()
