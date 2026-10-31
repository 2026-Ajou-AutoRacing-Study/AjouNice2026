#!/usr/bin/env python3
"""Illustrate verified native console contracts; never launch ROS or controllers."""
import importlib.util
import math
from pathlib import Path
import textwrap

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]


def main():
    spec = importlib.util.spec_from_file_location(
        'native_control_preview', ROOT / 'base_ws/src/control/trajectory_follower/scripts/control_terminal.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rows = []
    terminal = module.Terminal(lambda line, level: rows.append((line, level)))
    terminal.tick(0, '00.0')
    terminal.sample('vx', 23.45, 1)
    terminal.sample('ay', 0.25, 1)
    terminal.state('OK', '', math.radians(12.3), 1)
    terminal.tick(1, '01.0')
    terminal.sample('vx', 23.45, 1.2)
    terminal.sample('ay', 0.25, 1.2)
    terminal.state('OK', '', math.radians(-40), 1.2)
    terminal.tick(1.2, '01.2')
    terminal.tick(2, '02.0')

    panels = [
        ('$ planning', 'CURRENT: native ROS logs; compact summary not found in fetched team branch', [
            ('[WARN] [...] BPP waiting for lane-driving scenario: received=0 value=<none>', 'WARN'),
            ('[WARN] [...] BPP has no reference output: route_ready=0 odometry=1', 'WARN')]),
        ('$ traj_fol_a   OR   traj_fol_global',
         'SOURCE CONTRACT BELOW; local installed launch is old (rebuild/install required)', rows),
        ('$ planviz', 'CURRENT LOCAL ALIAS: compact input-health display', [
            ('00 RVZ waiting pose', 'WARN'),
            ('01 RVZ map ready pose 50Hz path wait', 'WARN'),
            ('02 RVZ map ready pose 50Hz path 10.0Hz', 'OK'),
            ('03 RVZ path stale 1.2s', 'ERROR')]),
    ]
    image = Image.new('RGB', (1260, 940), '#11151c')
    draw = ImageDraw.Draw(image)
    font = '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'
    mono = ImageFont.truetype(font, 19)
    small = ImageFont.truetype(font, 17)
    title = ImageFont.truetype(font, 24)
    draw.text((26, 20), 'PLANNING / CONTROL / PLANVIZ - VERIFIED CONNECTIONS', font=title, fill='#e7edf5')
    draw.text((26, 61), 'SIMULATED EXAMPLES / NOT A LIVE DRIVE / ROS timestamps abbreviated as [...]',
              font=small, fill='#ffd077')
    colors = {'OK': '#8bdf9e', 'WARN': '#ffd077', 'ERROR': '#ff8585'}
    for i, (command, note, lines) in enumerate(panels):
        top = 106 + i * 258
        draw.rectangle((22, top, 1238, top + 240), fill='#090c10', outline='#364251', width=2)
        draw.text((40, top + 13), command, font=mono, fill='#a4cef1')
        draw.text((40, top + 46), note, font=small, fill='#cbd2dc')
        y = top + 81
        for line, level in lines:
            for wrapped in textwrap.wrap(line, width=99):
                draw.text((40, y), wrapped, font=mono, fill=colors[level])
                y += 30
    draw.text((26, 895), 'CTRL: speed [km/h] / IMU lateral acceleration [m/s^2] / commanded steering [deg]',
              font=small, fill='#cbd2dc')
    out = ROOT / 'artifacts/field_console_default_preview_261029/native_consoles.png'
    out.parent.mkdir(parents=True, exist_ok=True)
    image.save(out)
    print(out)


if __name__ == '__main__':
    main()
