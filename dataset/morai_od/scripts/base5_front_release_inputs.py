"""Read-only source/input pins for a newly frozen front-support release.

Source discovery parses local scripts, never imports ROS or external packages.
Old releases need not carry these additional pins; their contract is unchanged.
"""
import ast
import hashlib
import json
from pathlib import Path
import re


SCRIPTS = Path(__file__).resolve().parent
ROOT = SCRIPTS.parents[2]
ENTRYPOINTS = (
    'launch_base5_obstacle_support.py', 'run_base5_fresh_batch.py',
    'run_base5_fresh_completion.py', 'run_base5_flexible_completion.py',
    'run_base5_adaptive_completion.py', 'run_base5_failure_aware_replenishment_fast.py',
    'run_base5_batch_campaign.py', 'run_base5_batch_optimized.py',
    'run_base5_capture_campaign.py', 'run_base5_capture_campaign_spatial.py',
    'run_base5_capture_optimized.py', 'generate_base5_independent.py',
    'base5_spatial_diversity.py', 'base5_front_completion_adapter.py',
    'prepare_base5_obstacle_front_support.py',
    'prepare_base5_front_support_qualification.py',
    'validate_base5_front_recovery_live.py', 'finalize_base5_front_support_release.py',
    'base5_primary_obstacle_range.py', 'prepare_base5_primary_obstacle_range.py',
    'prepare_base5_primary_range_qualification.py', 'validate_base5_primary_range_recovery.py',
    'finalize_base5_primary_range_release.py',
)
LOCAL_PREFIXES = ('base5_', 'run_base5_', 'generate_base5_', 'prepare_base5_',
                  'finalize_base5_', 'launch_base5_', 'validate_base5_',
                  'report_base5_', 'register_base5_')
HEX = re.compile(r'^[0-9a-f]{64}$')


def file_sha256(path):
    path = Path(path)
    if not path.is_file():
        raise ValueError('Missing frozen release file: ' + str(path))
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def verify_hashes(hashes, label='Frozen release input'):
    """Fail closed for malformed, missing or changed pins, including prepare-only."""
    if not isinstance(hashes, dict) or not hashes:
        raise ValueError(label + ' hashes must be a nonempty mapping')
    for path, expected in hashes.items():
        if (not isinstance(path, str) or not Path(path).is_absolute()
                or not isinstance(expected, str) or not HEX.fullmatch(expected)):
            raise ValueError(label + ' has a malformed file pin: ' + str(path))
        if file_sha256(path) != expected:
            raise ValueError(label + ' changed: ' + path)


def collect_runtime(scripts_dir=None, entrypoints=None):
    """Pin local import and literal ``with_name('*.py')`` transitive closure.

    Dynamic subprocess choices are covered by the explicit entrypoint list.
    An external import has no local sibling and is not opened or executed.
    Missing Base5 siblings, literal script references and entrypoints are fatal.
    """
    scripts = Path(scripts_dir or SCRIPTS).resolve()
    pending = [scripts / name for name in (ENTRYPOINTS if entrypoints is None else entrypoints)]
    result = {}
    while pending:
        path = pending.pop().resolve()
        if path.parent != scripts:
            raise ValueError('Runtime pin escaped local scripts: ' + str(path))
        if str(path) in result:
            continue
        digest = file_sha256(path)
        try:
            tree = ast.parse(path.read_text(), filename=str(path))
        except (SyntaxError, UnicodeError) as error:
            raise ValueError('Invalid frozen local source: ' + str(path)) from error
        result[str(path)] = digest
        for node in ast.walk(tree):
            modules = []
            if isinstance(node, ast.Import):
                modules = [alias.name.split('.')[0] for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                if node.level:
                    raise ValueError('Unsupported relative local import: ' + str(path))
                if node.module:
                    modules = [node.module.split('.')[0]]
            for name in modules:
                sibling = scripts / (name + '.py')
                if sibling.is_file():
                    pending.append(sibling)
                elif name.startswith(LOCAL_PREFIXES):
                    raise ValueError('Missing local imported script: ' + str(sibling))
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                    and node.func.attr == 'with_name' and node.args
                    and isinstance(node.args[0], ast.Constant)
                    and isinstance(node.args[0].value, str)
                    and node.args[0].value.endswith('.py')):
                pending.append(scripts / node.args[0].value)
    # Refuse an edit racing closure discovery rather than freezing mixed source.
    verify_hashes(result, 'Frozen local runtime')
    return dict(sorted(result.items()))


def _path(value, base):
    path = Path(value).expanduser()
    if path.is_absolute():
        return path.resolve()
    local = Path(base) / path
    return (local if local.exists() else ROOT / path).resolve()


def _references(value):
    """Yield explicit references, not arbitrary historical prose/path strings."""
    if isinstance(value, dict):
        if 'path' in value and 'sha256' in value:
            if not isinstance(value['path'], str):
                raise ValueError('Explicit input reference requires a string path')
            yield value['path'], value['sha256']
        for key, item in value.items():
            if isinstance(item, str) and key.endswith('_path') and key != 'original_path':
                digest = value.get(key[:-5] + '_sha256')
                if digest is not None:
                    yield item, digest
                elif key == 'frozen_specs_path':
                    raise ValueError('Frozen specs path requires sha256')
            elif isinstance(item, str) and key.endswith('_manifest'):
                digest = value.get(key + '_sha256')
                if digest is not None:
                    yield item, digest
            if isinstance(item, (dict, list)):
                yield from _references(item)
    elif isinstance(value, list):
        for item in value:
            yield from _references(item)


def _default_references(deck):
    # Read the same constants that the real generator uses. No context/ROS/MR
    # initialization is performed; every file read below is an actual data input.
    import generate_base5_independent as gen
    geo, catalog = gen.geo, gen.catalog
    refs = {
        geo.DEFAULT_ROUTE: None, geo.DEFAULT_TEMPLATE: None,
        gen.HIGHWAY_SAMPLE: deck.get('provenance', {}).get('highway_sample_sha256'),
        gen.GEOMETRY: deck.get('provenance', {}).get('geometry_sha256'),
        gen.LIVE_GEOMETRY: None,
        gen.APPEARANCE_POLICY: deck.get('provenance', {}).get('appearance_policy_sha256'),
        catalog.DEFAULT_CATALOG: deck.get('provenance', {}).get('catalog_sha256'),
        geo.PROJECT_ROOT / 'configs/sensors/Sensor_competition_local.json': None,
        geo.PROJECT_ROOT / 'configs/network/ego_network_mr_base5_savesensordata_collision_local.json': None,
    }
    for name in ('link_set.json', 'lane_boundary_set.json',
                 'singlecrosswalk_set.json', 'crosswalk_set.json'):
        expected = {'link_set.json': 'map_sha256',
                    'lane_boundary_set.json': 'map_boundary_sha256'}.get(name)
        refs[geo.DEFAULT_MGEO / name] = deck.get('provenance', {}).get(expected) if expected else None
    inventory = json.loads(catalog.DEFAULT_CATALOG.read_text())
    for filename, expected in inventory['source_sha256'].items():
        path = geo.DEFAULT_RUNTIME_ASSET_DIR / filename
        original = deck.get('provenance', {}).get('runtime_sources', {}).get(filename)
        if original is not None and original != expected:
            raise ValueError('Catalog/runtime provenance hash mismatch: ' + filename)
        refs[path] = expected
    for asset in inventory['assets']:
        if asset.get('geometric_gt_reference'):
            refs.setdefault(geo.PROJECT_ROOT / asset['geometric_gt_reference'], None)
    for path, expected in deck.get('provenance', {}).get('geometry_references', {}).items():
        resolved = _path(path, ROOT)
        previous = refs.get(resolved)
        if previous is not None and previous != expected:
            raise ValueError('Conflicting geometry provenance hashes: ' + str(resolved))
        refs[resolved] = expected
    validator = geo.PROJECT_ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/config/missions'
    for mission in ('static_obstacle', 'dynamic_obstacle'):
        templates = sorted((validator / mission / 'base/scenarios').glob('*.json'))
        if not templates:
            raise ValueError('Missing generator mission templates: ' + mission)
        refs.update({path: None for path in templates})
    return refs


def collect_inputs(manifest, expected_count=5000, default_references=None):
    """Hash the complete immutable input set, verifying declared source digests.

    Returns absolute-path SHA-256 pins directly suitable for release.input_hashes.
    ``default_references`` is a test fixture hook; release callers use defaults.
    No input, scenario, metadata or old release is ever written by this helper.
    """
    manifest = Path(manifest).resolve()
    deck = json.loads(manifest.read_text())
    scenes = deck.get('scenes', [])
    if (isinstance(expected_count, bool) or not isinstance(expected_count, int)
            or expected_count < 1 or deck.get('count') != expected_count
            or isinstance(deck.get('count'), bool)
            or len(scenes) != expected_count):
        raise ValueError('Complete frozen manifest count mismatch')
    ids, scenario_paths, metadata_paths, payloads = set(), set(), set(), set()
    pins, metadata = {}, []
    inspect = []

    def add(path, expected=None, inspect_json=False):
        path = _path(path, manifest.parent)
        actual = file_sha256(path)
        if expected is not None and (not isinstance(expected, str)
                                     or not HEX.fullmatch(expected) or actual != expected):
            raise ValueError('Declared release input hash mismatch: ' + str(path))
        if str(path) in pins and pins[str(path)] != actual:
            raise ValueError('Release input changed during enumeration: ' + str(path))
        is_new = str(path) not in pins
        pins[str(path)] = actual
        if inspect_json and is_new and path.suffix == '.json':
            inspect.append(path)
        return path, actual

    add(manifest)
    for row in scenes:
        ident = row.get('id')
        if not isinstance(ident, str) or not ident or ident in ids:
            raise ValueError('Duplicate or invalid frozen scene identity')
        ids.add(ident)
        scenario, actual = add(row['scenario'], row.get('scenario_sha256'))
        if 'scenario_sha256' not in row:
            raise ValueError('Frozen scene lacks scenario SHA-256: ' + ident)
        meta, _ = add(row['metadata'], row.get('metadata_sha256'))
        if scenario in scenario_paths or meta in metadata_paths or actual in payloads:
            raise ValueError('Frozen manifest repeats scenario/metadata/payload: ' + ident)
        scenario_paths.add(scenario); metadata_paths.add(meta); payloads.add(actual)
        data = json.loads(meta.read_text())
        metadata.append(data)
        # Motion lookup / independent terrain witnesses live in metadata too.
        for reference, digest in _references(data):
            add(reference, digest, inspect_json=True)
    config = deck['config']
    for reference, digest in _references(config):
        add(reference, digest, inspect_json=True)
    if config.get('frozen_specs_path'):
        frozen = json.loads(_path(config['frozen_specs_path'], manifest.parent).read_text())
        if (not isinstance(frozen, list) or len(frozen) != expected_count
                or any(meta.get('spec') != spec for meta, spec in zip(metadata, frozen))):
            raise ValueError('Frozen specs do not match full manifest metadata order')
    references = _default_references(deck) if default_references is None else default_references
    for path, expected in references.items():
        add(path, expected)
    visited = set()
    while inspect:
        path = inspect.pop()
        if path in visited:
            continue
        visited.add(path)
        for reference, digest in _references(json.loads(path.read_text())):
            add(reference, digest, inspect_json=True)
    verify_hashes(pins)
    return dict(sorted(pins.items()))
