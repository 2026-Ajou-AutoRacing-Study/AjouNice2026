#!/usr/bin/env python3
"""Audit installed ELF resolution and old-prefix leakage in the active overlay."""
import json
import os
from pathlib import Path
import subprocess
import sys
from catkin_pkg.packages import find_packages

root = Path(__file__).resolve().parents[1]
installed = root/'install_isolated'
packages = find_packages(str(root/'src'))
missing_packages = [p.name for p in packages.values() if not (installed/'share'/p.name).is_dir()]
missing_deps, old_prefix, elf_count = [], [], 0
for path in sorted((installed/'lib').rglob('*')):
    if not path.is_file() or path.is_symlink():
        continue
    with path.open('rb') as stream:
        if stream.read(4) != b'\x7fELF':
            continue
    elf_count += 1
    output = subprocess.run(['ldd', str(path)], text=True, capture_output=True).stdout
    dynamic = subprocess.run(['readelf', '-d', str(path)], text=True, capture_output=True).stdout
    if 'not found' in output:
        missing_deps.append(dict(file=str(path.relative_to(root)), errors=[x for x in output.splitlines() if 'not found' in x]))
    if '/home/jihun/Autoware2ros1' in output + dynamic:
        old_prefix.append(str(path.relative_to(root)))
cmake_leaks = []
for path in (installed/'share').rglob('*.cmake'):
    if '/home/jihun/Autoware2ros1' in path.read_text(errors='replace'):
        cmake_leaks.append(str(path.relative_to(root)))
environment = {key: os.environ.get(key, '') for key in ('CMAKE_PREFIX_PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH', 'ROS_PACKAGE_PATH')}
env_leaks = [key for key, value in environment.items() if '/home/jihun/Autoware2ros1' in value or '/base_ws/devel' in value]
result = dict(scope='RELOCATED_INSTALL_LINKAGE_NOT_FUNCTIONAL_RUNTIME', source_packages=len(packages),
              missing_install_share=missing_packages, elf_files_checked=elf_count,
              unresolved_libraries=missing_deps, old_prefix_elf=old_prefix, old_prefix_cmake=cmake_leaks,
              environment_leaks=env_leaks, environment=environment)
print(json.dumps(result, indent=2))
sys.exit(bool(missing_packages or missing_deps or old_prefix or cmake_leaks or env_leaks))
