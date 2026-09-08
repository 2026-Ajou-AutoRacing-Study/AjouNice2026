#!/usr/bin/env python3
"""Reproducible relocation/source/duplicate/ignore audit, no source writes."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from catkin_pkg.packages import find_packages

root = Path(__file__).resolve().parents[1]
manifest = json.loads((root/'provenance/checkpoint-r002.json').read_text())
sources = manifest['source_sha256']
excluded = [p for p in sources if '__pycache__' in Path(p).parts or p.endswith('.pyc')]
bad = []
required = []
for name, digest in sources.items():
    if name in excluded:
        continue
    path = root/'src'/name
    required.append(str(path.relative_to(root.parent)))
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
        bad.append(name)
ignore_lines = subprocess.run(['git', 'check-ignore', '-v', '--stdin'], cwd=root.parent,
    input='\n'.join(required), capture_output=True, text=True).stdout.splitlines()
ignored = [line.split('\t', 1)[1] for line in ignore_lines
           if not line.split('\t', 1)[0].split(':', 2)[2].startswith('!')]
planning = find_packages(str(root/'src'))
base = find_packages(str(root.parent/'base_ws/src'))
duplicates = sorted({p.name for p in planning.values()} & {p.name for p in base.values()})
provider_bad, provider_required, provider_excluded = [], [], []
for provider in ('osqp', 'proxsuite', 'acados'):
    values = json.loads((root/'provenance'/(provider+'-source-r001.json')).read_text())['files']
    for name, digest in values.items():
        if '.git' in Path(name).parts or '__pycache__' in Path(name).parts or name.endswith('.pyc'):
            provider_excluded.append(provider+'/'+name)
            continue
        path = root/'external'/provider/'src'/name
        provider_required.append(str(path.relative_to(root.parent)))
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            provider_bad.append(str(path.relative_to(root)))
force_add = set((root/'provenance/force-add-vendor-paths.txt').read_text().splitlines())
provider_ignore = subprocess.run(['git', 'check-ignore', '-v', '--stdin'], cwd=root.parent,
    input='\n'.join(provider_required), capture_output=True, text=True).stdout.splitlines()
provider_ignored = {line.split('\t', 1)[1] for line in provider_ignore
                   if not line.split('\t', 1)[0].split(':', 2)[2].startswith('!')}
unlisted_provider_ignores = sorted(provider_ignored-force_add)
result = dict(manifest_entries=len(sources), source_files_checked=len(required),
    excluded_generated_bytecode=excluded, changed_or_missing=bad, ignored_required_source=ignored,
    planning_packages=len(planning), base_packages=len(base), duplicate_packages=duplicates,
    checkpoint_sha256=hashlib.sha256((root/'provenance/checkpoint-r002.json').read_bytes()).hexdigest(),
    provider_source_files_checked=len(provider_required), provider_changed_or_missing=provider_bad,
    provider_metadata_or_cache_excluded=len(provider_excluded), provider_force_add_count=len(force_add),
    unlisted_provider_ignores=unlisted_provider_ignores)
print(json.dumps(result, indent=2))
sys.exit(bool(bad or ignored or duplicates or len(planning) != 154 or provider_bad or unlisted_provider_ignores))
