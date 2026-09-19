#!/usr/bin/env python3
"""Verify and rebuild the pinned solver sources included in this release."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

run = Path(__file__).resolve().parents[1]
base = run / 'external/acados'
source = base / 'src'
prefix = base / 'install'
receipt_path = run / 'provenance/acados-source-r001.json'


def command(args):
    subprocess.run([str(a) for a in args], check=True,
                   env={'PATH': '/usr/bin:/bin', 'LANG': 'C.UTF-8'})


if not source.exists():
    raise RuntimeError('Incomplete release: missing included acados sources')
receipt = json.loads(receipt_path.read_text())
for name, digest in receipt['files'].items():
    if '.git' in Path(name).parts:
        continue
    if hashlib.sha256((source / name).read_bytes()).hexdigest() != digest:
        raise RuntimeError('Provider source changed: ' + name)

command(['cmake', '-S', source, '-B', base / 'build',
         '-DCMAKE_INSTALL_PREFIX=' + str(prefix), '-DCMAKE_BUILD_TYPE=Release',
         '-DACADOS_WITH_QPOASES=ON', '-DCMAKE_POSITION_INDEPENDENT_CODE=ON'])
command(['cmake', '--build', base / 'build', '--parallel', '4'])
command(['cmake', '--install', base / 'build'])
if not (prefix / 'interfaces').exists():
    shutil.copytree(source / 'interfaces', prefix / 'interfaces')
(prefix / 'bin').mkdir(exist_ok=True)
renderer = base / 'artifacts/t_renderer'
if not renderer.exists():
    raise RuntimeError('Incomplete release: missing pinned acados renderer artifact')
if 'renderer_sha256' in receipt and hashlib.sha256(renderer.read_bytes()).hexdigest() != receipt['renderer_sha256']:
    raise RuntimeError('Pinned renderer artifact changed')
shutil.copy2(renderer, prefix / 'bin/t_renderer')
receipt['renderer_sha256'] = hashlib.sha256((prefix / 'bin/t_renderer').read_bytes()).hexdigest()
receipt['renderer_note'] = 'Previously selected glibc-compatible v0.0.34 executable; pinned artifact, not rebuilt here.'
command(['/usr/bin/python3', '-m', 'venv', prefix / 'venv'])
python = prefix / 'venv/bin/python3'
requirements = base / 'requirements-lock.txt'
if not requirements.is_file():
    raise RuntimeError('Incomplete release: missing solver-codegen requirements lock')
command([python, '-m', 'pip', 'install', '-r', requirements])
receipt['build_status'] = 'LIBRARY_AND_CODEGEN_ENV_BUILT_NOT_NUMERICAL_PARITY'
(run / 'logs').mkdir(exist_ok=True)
(run / 'logs/acados-build.json').write_text(json.dumps(receipt, indent=2) + '\n')
print('acados installed locally; solver generation is validated by Path Optimizer build.')
