#!/usr/bin/env python3
"""Re-render immutable evidence with Korean footer and past-only planner records.

Legacy INFO records are sampled (~0.5s), not per-camera-frame annotations.
This adapter never infers planner permission from lamps or trajectory velocity.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess

import record_traffic_light_evidence_video as video


def planner_records(path):
    rows = []
    pattern = re.compile(r'\[(\d+\.\d+)\]: \[signal_decision\] (.*)')
    for line in path.read_text().splitlines():
        match = pattern.search(line)
        if not match:
            continue
        seconds, fraction = match[1].split('.')
        stamp = int(seconds)*10**9 + int(fraction.ljust(9, '0')[:9])
        fields = dict(item.split('=', 1) for item in match[2].split() if '=' in item)
        if not fields.get('reg', '').isdigit() or fields.get('decision') not in ('GO', 'STOP'):
            continue
        value = dict(group_id=int(fields['reg']), require_stop=fields['decision'] == 'STOP',
                     committed=fields.get('committed') == '1', reason=fields.get('reason'))
        try:
            for key in ('target_m', 'official_m', 'brake_m'):
                value[key] = video.clean(float(fields[key])) if key in fields else None
        except ValueError:
            # stdout from multiple processes may interleave. Do not repair or
            # invent a number from a visibly corrupted diagnostic record.
            continue
        rows.append((stamp, value))
    return sorted(rows, key=lambda row: row[0])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--planner-log', type=Path,
                        help='Optional legacy log adapter; omit to preserve recorded planner topic data')
    parser.add_argument('--preview-sec', type=float, default=80.)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = video.validate_output(args.output)
    import cv2
    rows = planner_records(args.planner_log) if args.planner_log is not None else []
    frames = [json.loads(line) for line in (args.source/'frames.jsonl').open()]
    capture = cv2.VideoCapture(str(args.source/'front_evidence.mp4'))
    fps = capture.get(cv2.CAP_PROP_FPS)
    if not capture.isOpened() or not fps:
        raise RuntimeError('Source video unavailable')
    output.mkdir(parents=True)
    offset, latest = 0, {}
    command = video.ffmpeg_arguments(output/'front_evidence.mp4', fps, 'h264_nvenc')
    with (output/'ffmpeg.log').open('wb') as log, (output/'frames.jsonl').open('w') as manifest:
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=log)
        try:
            for index, evidence in enumerate(frames):
                success, image = capture.read()
                if not success:
                    raise RuntimeError('Video ended before manifest')
                now = evidence['write_ros_ns']
                while offset < len(rows) and rows[offset][0] <= now:
                    stamp, value = rows[offset]
                    latest[value['group_id']] = (stamp, value)
                    offset += 1
                runtime = evidence['telemetry']['runtime'].get('value') or {}
                group = (runtime.get('frame_decision') or {}).get('group_id')
                selected = latest.get(group)
                if args.planner_log is not None:
                    evidence['telemetry']['planner_decision'] = {} if selected is None else dict(
                        value=selected[1], receipt_age_sec=(now-selected[0])/1e9,
                        receipt_ros_ns=selected[0], provenance='historical_0.5s_sample_not_exact_camera')
                frame = video.render_frame(image,
                    '과거 판단 0.5초 표본' if args.planner_log is not None else '기록된 판단 토픽', evidence)
                process.stdin.write(frame.tobytes())
                manifest.write(json.dumps(evidence, ensure_ascii=False, allow_nan=False)+'\n')
                if index == min(round(args.preview_sec*fps), len(frames)-1):
                    cv2.imwrite(str(output/'preview.png'), frame)
            if capture.read()[0]:
                raise RuntimeError('Video contains frames absent from manifest')
        finally:
            capture.release()
            process.stdin.close()
            result = process.wait()
        if result:
            raise RuntimeError('ffmpeg failed: see ffmpeg.log')
    hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in (args.source/'frames.jsonl', args.planner_log) if path is not None}
    (output/'metadata.json').write_text(json.dumps(dict(source=str(args.source),
        input_sha256=hashes, frames=len(frames), fps=fps, planner_sample_max_age_sec=.75,
        note=('Derived view with legacy sampled logs.' if args.planner_log is not None else
              'Derived view with original recorded planner topic. No legacy log substitution.')), indent=2))
    print(output/'front_evidence.mp4')


if __name__ == '__main__':
    main()
