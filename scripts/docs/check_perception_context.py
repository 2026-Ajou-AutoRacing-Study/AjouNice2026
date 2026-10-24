#!/usr/bin/env python3
"""Read-only current routing, historical snapshot and contract checks; no ROS/GPU/network imports."""
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
    'AGENTS.md', 'perception/AGENTS.md', 'docs/bevfusion/AGENTS.md',
    'dataset/morai_od/AGENTS.md', 'base_ws/src/perception/AGENTS.md',
    'configs/perception/AGENTS.md', 'base_ws/src/perception_object_pipeline/AGENTS.md',
)
# Only current routers, contracts and production handoffs are mandatory context.
# Dated evidence, old plans and reproduction documents are linked on demand.
ACTIVE = AGENTS + (
    'docs/README.md', 'docs/ACTIVE_PRIORITIES.md', 'kcity_map/README.md',
    'docs/perception/README.md', 'docs/bevfusion/CURRENT.md', 'docs/bevfusion/README.md',
    'docs/bevfusion/BASE5_BEVFUSION_PRODUCTION_HANDOFF.md',
    'docs/bevfusion/BASE3_BEVFUSION_PRODUCTION_HANDOFF.md',
    'docs/bevfusion/GT_EVALUATION_COORDINATE_CONTRACT.md',
    'dataset/morai_od/README.md', 'dataset/morai_od/docs/README.md',
    'dataset/morai_od/docs/BASE5_V2_CURRENT.md',
    'dataset/morai_od/docs/MR_ASSET_COLLECTION_CURRENT.md',
    'perception/detection3d/README.md', 'perception/detection3d/bevfusion/README.md',
    'base_ws/src/perception/bevfusion_ros/README.md',
    'base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md',
)
BUDGETS = {
    **{name: 45 for name in AGENTS},
    **{name: 65 for name in ACTIVE if name.endswith('/README.md')},
    'docs/ACTIVE_PRIORITIES.md': 65,
    'docs/bevfusion/CURRENT.md': 125,
    'dataset/morai_od/docs/BASE5_V2_CURRENT.md': 125,
    'base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md': 125,
}
# Former paths come from context_261012; archive destinations follow the 10/24
# old->new manifest. A retained compatibility stub takes precedence if present.
COMPATIBILITY = {
    'docs/bevfusion/261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md':
        'docs/bevfusion/261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md',
    'docs/bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md':
        'docs/bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md',
    'dataset/morai_od/docs/261007_base5_direct_5k_current_plan.md':
        'dataset/morai_od/docs/legacy/base5/261007_base5_direct_5k_current_plan.md',
    'dataset/morai_od/docs/261007_base5_5k_capture_runbook.md':
        'dataset/morai_od/docs/legacy/base5/261007_base5_5k_capture_runbook.md',
}
CONTRACTS = {
    'model': ('docs/bevfusion/CURRENT.md', (
        'E12', 'camera TRT', 'head SDPA', 'cache', 'E12 eager', '--safe',
        '--model e24', '--model e25', '0.40/0.10/0.10', 'compile OFF',
        'vehicle / pedestrian / obstacle', '512×1056', 'Q90 JPEG',
        '30,750', '26,136', '4,614',
    )),
    'dataset': ('dataset/morai_od/docs/BASE5_V2_CURRENT.md', (
        '30,750', '26,136', '4,614', '6환경', '10개 원래 root/infos',
        'selected_ledger.jsonl', 'native GT', 'Q90 JPEG', 'upright 7D',
        '3클래스', 'Animal→Pedestrian', '강제 resume',
    )),
    'production handoff': ('docs/bevfusion/BASE5_BEVFUSION_PRODUCTION_HANDOFF.md', (
        'E12', 'camera TRT', 'head SDPA', 'E12 eager', '0.40/0.10/0.10',
        '512×1056', '--safe',
    )),
    'GT coordinate': ('docs/bevfusion/GT_EVALUATION_COORDINATE_CONTRACT.md', (
        '/localization/kinematic_state', '/Ego_topic', '−1.347°', '중복 보정',
    )),
}
LINK = re.compile(r'(?<!!)\[[^\]\n]*\]\(([^)\n]+)\)')
HEADING = re.compile(r'^#{1,6}\s+(.+?)\s*#*\s*$', re.M)
ANCHOR = re.compile(r'<a\s+(?:[^>]*?\s)?(?:id|name)=["\']([^"\']+)["\']')
HEX = re.compile(r'^[0-9a-f]{64}$')
REVISION = re.compile(r'^[0-9a-f]{7,64}$')


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
    root = Path(root).resolve()
    errors = []; link_count = 0; cache = {}; counts = {}

    def label(path):
        try:
            return str(path.relative_to(root))
        except ValueError:
            return str(path)

    def body(path):
        if path not in cache:
            try:
                cache[path] = path.read_text(encoding='utf-8')
            except (OSError, UnicodeError) as error:
                errors.append(f'Cannot read context {label(path)}: {error}')
                cache[path] = None
        return cache[path]

    def local_path(value, context):
        if not isinstance(value, str) or not value:
            errors.append(f'{context}: missing relative path'); return None
        path = (root/value).resolve()
        if Path(value).is_absolute() or (path != root and root not in path.parents):
            errors.append(f'{context}: path escapes repository: {value}'); return None
        return path

    def git(arguments, context):
        try:
            result = subprocess.run(['git', *arguments], cwd=root, capture_output=True, check=False)
        except OSError as error:
            errors.append(f'{context}: cannot run Git: {error}'); return None
        if result.returncode:
            detail = result.stderr.decode('utf-8', errors='replace').strip()
            errors.append(f'{context}: Git failed ({result.returncode}): {detail}')
            return None
        return result.stdout

    def links(path, text):
        try:
            return list(references(path, text))
        except ValueError as error:
            errors.append(f'{label(path)}: malformed Markdown link: {error}')
            return []

    def check_link(source, target, fragment):
        nonlocal link_count
        link_count += 1
        if not target.exists():
            errors.append(f'{source}: missing target {label(target)}')
        elif fragment and target.suffix == '.md':
            text = body(target)
            if text is not None and fragment not in anchors(text):
                errors.append(f'{source}: missing anchor {label(target)}#{fragment}')

    for name in ACTIVE:
        path = root/name
        if not path.is_file():
            errors.append(f'Missing active context: {name}'); continue
        text = body(path)
        if text is None:
            continue
        counts[name] = len(text.splitlines())
        if re.search(r'[\u3040-\u30ff\u3400-\u4dbf\u4e00-\u9fff]', text):
            errors.append(f'{name}: unexpected Chinese/Japanese text in active context')
        for target, fragment in links(path, text):
            check_link(name, target, fragment)
        if name in AGENTS and re.search(r'^\d{4}-\d{2}-\d{2}\s+최신', text, re.M):
            errors.append(f'{name}: historical status added to AGENTS')
        maximum = BUDGETS.get(name)
        if maximum is not None and counts[name] > maximum:
            errors.append(f'{name}: exceeds {maximum}-line current/index budget')

    # This frozen 10/12 source audit is independent of current archive routing.
    snapshot_path = root/'docs/perception/legacy/context_261012/manifest.json'
    manifest_text = body(snapshot_path)
    manifest = {}
    if manifest_text is not None:
        try:
            manifest = json.loads(manifest_text)
        except json.JSONDecodeError as error:
            errors.append(f'Invalid snapshot manifest: {error}')
    if not isinstance(manifest, dict):
        errors.append('Invalid snapshot manifest: object required'); manifest = {}
    records = manifest.get('files', [])
    if not isinstance(records, list):
        errors.append('Invalid snapshot manifest: files must be a list'); records = []
    source_commit = manifest.get('source_commit')
    valid_revision = isinstance(source_commit, str) and REVISION.fullmatch(source_commit)
    if not valid_revision:
        errors.append('Invalid snapshot manifest: hexadecimal source_commit required')

    original_lines = 0; old_agent_lines = 0; old_by_path = {}
    preserved_originals = 0; verified_sha = 0; verified_git = 0
    for index, record in enumerate(records):
        context = f'Snapshot record {index}'
        if not isinstance(record, dict):
            errors.append(context+': object required'); continue
        old_name = record.get('original_path'); archive_name = record.get('archive_path')
        original_path = local_path(old_name, context)
        preserved = local_path(archive_name, context)
        expected_sha = record.get('sha256')
        if not isinstance(expected_sha, str) or not HEX.fullmatch(expected_sha):
            errors.append(context+': malformed SHA256')
        if original_path is None or preserved is None:
            continue
        if old_name in old_by_path:
            errors.append(context+': duplicate original_path: '+old_name); continue
        try:
            data = preserved.read_bytes()
        except OSError as error:
            errors.append(f'Missing original snapshot {archive_name}: {error}'); continue
        preserved_originals += 1
        if hashlib.sha256(data).hexdigest() != expected_sha:
            errors.append('Changed original snapshot: '+archive_name)
        else:
            verified_sha += 1
        if valid_revision:
            original = git(['show', source_commit+':'+old_name], 'Snapshot source '+old_name)
            if original is not None:
                if original != data:
                    errors.append('Snapshot differs from source commit: '+old_name)
                else:
                    verified_git += 1
        try:
            text = data.decode('utf-8')
        except UnicodeError as error:
            errors.append(f'Invalid original snapshot text {archive_name}: {error}'); continue
        old_by_path[old_name] = text
        original_lines += len(text.splitlines())
        if old_name in AGENTS:
            old_agent_lines += len(text.splitlines())

    compatibility_paths = {}
    for old_name, archive_name in COMPATIBILITY.items():
        old_text = old_by_path.get(old_name)
        if old_text is None:
            errors.append('Missing original compatibility snapshot: '+old_name); continue
        current = root/old_name
        destination = current if current.is_file() else root/archive_name
        if not destination.is_file():
            errors.append(f'Missing compatibility destination: {old_name} -> {archive_name}'); continue
        text = body(destination)
        if text is None:
            continue
        compatibility_paths[old_name] = label(destination)
        missing = anchors(old_text) - anchors(text)
        if missing:
            errors.append(label(destination)+': lost historical anchors '+', '.join(sorted(missing)))

    # Check only incoming links to compacted active destinations, not unrelated
    # historical contents or protected planning/control document internals.
    destinations = {str((root/name).resolve()) for name in ACTIVE}
    tracked = git(['ls-files', '*.md'], 'Incoming context links')
    incoming_count = 0
    for name in tracked.decode('utf-8', errors='replace').splitlines() if tracked is not None else ():
        if '/legacy/' in name or '/evidence/' in name or name in ACTIVE:
            continue
        path = root/name
        if not path.is_file():
            continue
        text = body(path)
        if text is None:
            continue
        for target, fragment in links(path, text):
            if str(target) in destinations:
                check_link(name, target, fragment); incoming_count += 1

    for contract, (name, markers) in CONTRACTS.items():
        text = body(root/name)
        if text is None:
            continue
        for token in markers:
            if token not in text:
                errors.append(f'Missing {contract} contract marker: {token}')

    return dict(errors=errors, active_files=len(ACTIVE), checked_links=link_count,
        checked_incoming_links=incoming_count, preserved_originals=preserved_originals,
        verified_snapshot_sha256=verified_sha, verified_git_snapshots=verified_git,
        preserved_original_lines=original_lines,
        archived_large_agent_lines_before=old_agent_lines,
        same_large_agent_lines_after=sum(counts.get(name, 0) for name in old_by_path if name in AGENTS),
        all_scoped_agents_lines=sum(counts.get(name, 0) for name in AGENTS),
        compatibility_paths=compatibility_paths, line_counts=counts)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    args = parser.parse_args()
    result = audit(args.root.resolve())
    print(json.dumps(result, indent=2, ensure_ascii=False))
    raise SystemExit(1 if result['errors'] else 0)
