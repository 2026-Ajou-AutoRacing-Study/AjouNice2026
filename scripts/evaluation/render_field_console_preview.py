#!/usr/bin/env python3
"""Render the real field-console formatter with clearly labelled synthetic logs.

Developer-only screenshot helper: Pillow and a Korean font are required here,
but neither is a dependency of the runtime terminal wrapper. No ROS is started.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
CONSOLE = ROOT / 'scripts/runtime/field_console.py'
DEFAULT_OUTPUT = ROOT / 'artifacts/field_console_preview_261028'


def korean_font(size):
    """Use the Korean face/index selected by fontconfig, including TTC files."""
    for family in ('Noto Sans Mono CJK KR', 'Noto Sans CJK KR'):
        try:
            result = subprocess.run(
                ['fc-match', '-f', '%{file}\n%{index}\n', family],
                check=True, stdout=subprocess.PIPE, universal_newlines=True)
            path, index = result.stdout.strip().splitlines()[:2]
            return ImageFont.truetype(path, size, index=int(index))
        except (OSError, ValueError, subprocess.SubprocessError):
            continue
    raise RuntimeError('Korean font unavailable: install fonts-noto-cjk')


def load_console():
    spec = importlib.util.spec_from_file_location('field_console_preview', CONSOLE)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def collect_examples(output_dir, width=150):
    """Keep unmodified CLI demo stdout and use its formatter for extra examples."""
    output_dir.mkdir(parents=True, exist_ok=True)
    result = subprocess.run(
        [sys.executable, str(CONSOLE), 'demo', '--log-dir',
         str(output_dir / 'raw'), '--width', str(width)],
        check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        universal_newlines=True)
    (output_dir / 'demo_stdout.txt').write_text(result.stdout, encoding='utf-8')
    console = load_console()
    lines = result.stdout.splitlines()
    panels = [
        {
            'command': '$ field bevfusion',
            'title': '객체검출 · 구간 중 오류가 나면 다음 INFO에도 누적 기록 유지',
            'lines': [line for line in lines if '[객체검출]' in line],
            'note': '다음 콜백 수신 ≠ 추론 복구 확인. 자세한 원인은 원문 로그에서 확인.',
        },
        {
            'command': '$ field traffic',
            'title': '신호등 · 노드의 신호 원문과 미션 선택 경고를 구분',
            'lines': [line for line in lines if '[신호등]' in line],
            'note': 'GO/UNKNOWN은 노드가 출력한 텍스트. 요약기가 신호를 판정하지 않음.',
        },
        {
            'command': '$ field localization',
            'title': '측위 · GPS 음영 설명과 새 로그 없음은 서로 다른 상태',
            'lines': [line for line in lines if '[측위]' in line],
            'note': '출력이 없다는 것만으로 센서 단절이나 정상 주행을 판정하지 않음.',
        },
    ]
    extra_inputs = {
        'traffic': [
            '[WARN] [Traffic map ROI] missing B: route_missing_or_stale',
            '[INFO] [Traffic] B | UNKNOWN | no_lamps',
        ],
        'localization': [
            '[INFO] Fresh GNSS z resumed',
        ],
    }
    for panel, component in zip(panels[1:], ('traffic', 'localization')):
        summary = console.Summary(component, width)
        for raw in extra_inputs[component]:
            immediate = summary.ingest(raw)
            if immediate:
                panel['lines'].append(immediate)
            panel['lines'].append(summary.render())
        if component == 'localization':
            # Simulate no new log lines, not a stopped sensor or failed node.
            panel['lines'].append(summary.render(now=summary.last_activity + 3.2))
    (output_dir / 'preview_examples.json').write_text(json.dumps(
        {'notice': '모의 로그 · 실주행 결과 아님', 'extra_inputs': extra_inputs,
         'panels': panels}, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return panels


def render(output_dir, panels):
    fonts = {size: korean_font(size) for size in (22, 23, 24, 28, 44)}
    width, margin = 2240, 56
    card_heights = [154 + len(panel['lines']) * 43 for panel in panels]
    height = 258 + sum(card_heights) + 24 * len(panels) + 104
    image = Image.new('RGB', (width, height), '#0b1020')
    draw = ImageDraw.Draw(image)
    draw.text((margin, 38), '현장 터미널 · 한국어 한 줄 요약',
              font=fonts[44], fill='#eef4ff')
    draw.text((margin, 110), '출력 예시 · 모의 로그 / 실주행 결과 아님',
              font=fonts[28], fill='#ffcc80')
    draw.text((margin, 163), '기본 1초 요약  |  새 오류 유형은 즉시 알림  |  원문 stdout·stderr 보존',
              font=fonts[24], fill='#aab9d5')
    draw.text((margin, 207), '화면 요약은 운행 안전 판정이나 토픽 감시기가 아닙니다.',
              font=fonts[22], fill='#aab9d5')
    top = 258
    for panel, card_height in zip(panels, card_heights):
        draw.rectangle((margin, top, width - margin, top + card_height),
                       fill='#151e32', outline='#2a3854', width=2)
        draw.text((margin + 22, top + 17), panel['command'],
                  font=fonts[28], fill='#8fd7f3')
        draw.text((margin + 22, top + 64), panel['title'],
                  font=fonts[23], fill='#e0e7f2')
        for index, line in enumerate(panel['lines']):
            color = '#ff9d9d' if '] 오류' in line else '#cbd5e7'
            if '] 주의 |' in line:
                color = '#ffcf8f'
            draw.text((margin + 22, top + 109 + index * 43), line,
                      font=fonts[23], fill=color)
        draw.text((margin + 22, top + card_height - 39), panel['note'],
                  font=fonts[22], fill='#93a4c3')
        top += card_height + 24
    draw.text((margin, top + 6), '원본 로그: artifacts/field_logs/…  |  필터가 메시지를 숨겨도 원문은 삭제하지 않습니다.',
              font=fonts[23], fill='#b6c6dd')
    destination = output_dir / 'console_preview.png'
    image.save(destination)
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    print(render(args.output_dir, collect_examples(args.output_dir)))


if __name__ == '__main__':
    main()
