#!/usr/bin/env python3
"""Render six compact field consoles using the actual formatter and synthetic logs."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
CONSOLE = ROOT / 'scripts/runtime/field_console.py'
DEFAULT_OUTPUT = ROOT / 'artifacts/field_console_preview_261029'


def load_console():
    spec = importlib.util.spec_from_file_location('field_console_preview', CONSOLE)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def collect_examples(output_dir, width=48):
    output_dir.mkdir(parents=True, exist_ok=True)
    console = load_console()
    examples = [
        ('udp_lan', ('C30 G16 I50 L5.8', 'WARN'), [
            '[INFO] receiving sensor data', '[ERROR] Address already in use']),
        ('sensing', ('static 7/7', 'INFO'), [
            '[INFO] ready', '[WARN] TF lookup failed']),
        ('localization', ('50Hz GPS shadow', 'WARN'), [
            '[INFO] update', '[WARN] Reset smooth control alignment: translation_discontinuity']),
        ('bevfusion', ('V3 P1 O2 5.8Hz 260ms', 'INFO'), [
            '[INFO] callback', '[WARN] BEVFusion sync rejected: front differs by 0.137 sec',
            '[WARN] BEVFusion request failed: timeout']),
        ('tracking', ('V3 P1 O2 30.0Hz 5ms', 'INFO'), [
            '[INFO] update', '[WARN] vehicle-only corridor FAIL-OPEN: state missing']),
        ('traffic', ('D RC+GLA 6.7Hz 45ms', 'INFO'), [
            '[INFO] [Traffic] D-2 | GLA+RC', '[WARN] TF lookup failed']),
    ]
    panels = []
    for component, health, messages in examples:
        summary = console.Summary(component, width, SimpleNamespace(snapshot=lambda: health))
        lines = []
        for message in messages:
            summary.ingest(message)
            line = summary.render()
            lines.append({'text': str(line), 'level': line.level})
        entry = 'sensing_lan' if component == 'sensing' else component
        panels.append({'command': '$ field ' + entry, 'lines': lines, 'raw': messages})
    (output_dir / 'preview_examples.json').write_text(
        json.dumps({'notice': 'SIMULATED LOGS / NO LIVE STACK', 'panels': panels}, indent=2) + '\n')
    return panels


def render(output_dir, panels):
    font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'
    mono = ImageFont.truetype(font_path, 18)
    title = ImageFont.truetype(font_path, 23)
    image = Image.new('RGB', (1260, 790), '#11151c')
    draw = ImageDraw.Draw(image)
    draw.text((28, 24), 'FIELD CONSOLES / 48 COLUMNS / 1 SECOND', font=title, fill='#e7edf5')
    draw.text((28, 64), 'SIMULATED / V:vehicle P:pedestrian O:obstacle / ms:source age', font=mono, fill='#ffd077')
    colors = {'INFO': '#8bdf9e', 'WARN': '#ffd077', 'ERROR': '#ff8585'}
    for i, panel in enumerate(panels):
        x, y = 24 + (i % 2) * 620, 112 + (i // 2) * 194
        draw.rectangle((x, y, x + 596, y + 176), fill='#090c10', outline='#364251', width=2)
        draw.text((x + 16, y + 14), panel['command'], font=mono, fill='#a4cef1')
        for j, line in enumerate(panel['lines']):
            draw.text((x + 16, y + 54 + 31 * j), line['text'], font=mono, fill=colors[line['level']])
    draw.text((28, 712), 'Green: received state  Yellow: attention  Red: missing/failure', font=mono, fill='#cbd2dc')
    draw.text((28, 742), 'Seconds only; raw + full-time status saved. TL lamps != planner GO.', font=mono, fill='#cbd2dc')
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
