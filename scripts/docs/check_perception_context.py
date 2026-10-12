#!/usr/bin/env python3
"""Read-only routing, snapshot and budget checks; no ROS/GPU/network imports."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import unicodedata
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[2]
AGENTS = (
    'perception/AGENTS.md', 'docs/bevfusion/AGENTS.md', 'dataset/morai_od/AGENTS.md',
    'base_ws/src/perception/AGENTS.md', 'configs/perception/AGENTS.md',
    'base_ws/src/perception_object_pipeline/AGENTS.md',
)
ACTIVE = AGENTS + (
    'docs/perception/README.md', 'docs/bevfusion/CURRENT.md', 'docs/bevfusion/README.md',
    'dataset/morai_od/docs/README.md', 'dataset/morai_od/docs/BASE5_V2_CURRENT.md',
    'dataset/morai_od/docs/MR_ASSET_COLLECTION_CURRENT.md',
    'dataset/morai_od/docs/261007_base5_direct_5k_current_plan.md',
    'dataset/morai_od/docs/261007_base5_5k_capture_runbook.md',
    'docs/bevfusion/261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md',
    'docs/bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md',
    'perception/detection3d/bevfusion/README.md',
    'base_ws/src/perception/bevfusion_ros/README.md',
    'base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md',
    'dataset/morai_od/docs/261010_base5_fresh_environment_runbook.md',
    'docs/perception/legacy/context_261012/README.md',
    'docs/bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md',
    'perception/detection3d/README.md', 'perception/pointpainting/README.md',
    'perception/detection3d/fastbev/README.md',
)
COMPATIBILITY = (
    'docs/bevfusion/261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md',
    'docs/bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md',
    'dataset/morai_od/docs/261007_base5_direct_5k_current_plan.md',
    'dataset/morai_od/docs/261007_base5_5k_capture_runbook.md',
)
LINK = re.compile(r'(?<!!)\[[^\]\n]*\]\(([^)\n]+)\)')
HEADING = re.compile(r'^#{1,6}\s+(.+?)\s*#*\s*$', re.M)
ANCHOR = re.compile(r'<a\s+(?:[^>]*?\s)?(?:id|name)=["\']([^"\']+)["\']')


def slug(value):
    value = re.sub(r'<[^>]*>', '', value).lower()
    value = ''.join(c for c in value if c in '-_' or unicodedata.category(c)[0] not in 'PS')
    return re.sub(r'\s', '-', value)


def anchors(text):
    text = re.sub(r'```[^\n]*\n.*?```', '', text, flags=re.S)
    result = set(ANCHOR.findall(text)); seen = {}
    for value in HEADING.findall(text):
        name = slug(value); count = seen.get(name, 0); seen[name] = count + 1
        result.add(name + ('-' + str(count) if count else ''))
    return result


def references(path, text):
    # Ignore fenced examples so quoted historical examples are not active links.
    text = re.sub(r'```[^\n]*\n.*?```', '', text, flags=re.S)
    for target in LINK.findall(text):
        target = target.strip().strip('<>')
        parts = urlsplit(target)
        if parts.scheme or parts.netloc:
            continue
        yield (path.parent / unquote(parts.path)).resolve() if parts.path else path.resolve(), unquote(parts.fragment)


def audit(root=ROOT):
    errors = []; link_count = 0; cache = {}

    def body(path):
        if path not in cache:
            cache[path] = path.read_text()
        return cache[path]

    def check_link(source, target, fragment):
        nonlocal link_count
        link_count += 1
        if not target.exists():
            errors.append(f'{source}: missing target {target}')
        elif fragment and target.suffix == '.md' and fragment not in anchors(body(target)):
            errors.append(f'{source}: missing anchor {target}#{fragment}')

    counts = {}
    for name in ACTIVE:
        path = root / name
        if not path.is_file():
            errors.append(f'Missing active context: {name}'); continue
        text = body(path); counts[name] = len(text.splitlines())
        if re.search(r'[\u3040-\u30ff\u3400-\u4dbf\u4e00-\u9fff]', text):
            errors.append(f'{name}: unexpected Chinese/Japanese text in active context')
        for target, fragment in references(path, text):
            check_link(name, target, fragment)
        if name in AGENTS:
            if counts[name] > 45:
                errors.append(f'{name}: AGENTS exceeds 45-line routing budget')
            if re.search(r'^\d{4}-\d{2}-\d{2}\s+최신', text, re.M):
                errors.append(f'{name}: historical status added to AGENTS')

    for name, maximum in (
        ('docs/perception/README.md', 65), ('docs/bevfusion/README.md', 65),
        ('dataset/morai_od/docs/README.md', 60), ('docs/bevfusion/CURRENT.md', 125),
        ('dataset/morai_od/docs/BASE5_V2_CURRENT.md', 125),
    ):
        if counts.get(name, 0) > maximum:
            errors.append(f'{name}: exceeds {maximum}-line current/index budget')

    snapshot_dir = root/'docs/perception/legacy/context_261012'
    manifest = json.loads((snapshot_dir/'manifest.json').read_text())
    original_lines = 0; old_agent_lines = 0
    old_by_path = {}
    for record in manifest['files']:
        preserved = root/record['archive_path']; data = preserved.read_bytes()
        if hashlib.sha256(data).hexdigest() != record['sha256']:
            errors.append('Changed original snapshot: ' + record['archive_path'])
        original = subprocess.run(['git', 'show', manifest['source_commit']+':'+record['original_path']],
            cwd=root, capture_output=True, check=True).stdout
        if original != data:
            errors.append('Snapshot differs from source commit: '+record['original_path'])
        old_by_path[record['original_path']] = data.decode()
        original_lines += len(data.decode().splitlines())
        if record['original_path'] in AGENTS:
            old_agent_lines += len(data.decode().splitlines())

    for name in COMPATIBILITY:
        missing = anchors(old_by_path[name]) - anchors(body(root/name))
        if missing:
            errors.append(name + ': lost historical anchors ' + ', '.join(sorted(missing)))

    # Check old incoming links to newly compacted destinations, without requiring
    # unrelated historical documentation to pass this scoped audit.
    destinations = {str((root/n).resolve()) for n in ACTIVE if '/legacy/' not in n}
    tracked = subprocess.run(['git','ls-files','*.md'],cwd=root,capture_output=True,text=True,check=True).stdout
    incoming_count = 0
    for name in tracked.splitlines():
        if '/legacy/' in name or name in ACTIVE:
            continue
        path = root/name
        if not path.is_file():
            continue
        for target, fragment in references(path, body(path)):
            if str(target) in destinations:
                check_link(name, target, fragment); incoming_count += 1

    model = body(root/'docs/bevfusion/CURRENT.md')
    for token in ('E25', 'E9', 'E24', '0.40/0.15/0.10', '0.40/0.10/0.10',
                  'vehicle / pedestrian / obstacle', '512×1056'):
        if token not in model:
            errors.append('Missing model contract marker: '+token)
    dataset = body(root/'dataset/morai_od/docs/BASE5_V2_CURRENT.md')
    for token in ('5,000', '부족 0', 'flexible_v140', 'fresh_suite_final_v142', '3클래스'):
        if token not in dataset:
            errors.append('Missing dataset contract marker: '+token)

    report = dict(errors=errors, active_files=len(ACTIVE), checked_links=link_count,
        checked_incoming_links=incoming_count, preserved_originals=len(manifest['files']),
        preserved_original_lines=original_lines,
        archived_large_agent_lines_before=old_agent_lines,
        same_large_agent_lines_after=sum(counts[n] for n in old_by_path if n in AGENTS),
        all_scoped_agents_lines=sum(counts[n] for n in AGENTS), line_counts=counts)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    args = parser.parse_args()
    result = audit(args.root.resolve())
    print(json.dumps(result, indent=2, ensure_ascii=False))
    raise SystemExit(1 if result['errors'] else 0)
