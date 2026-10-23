#!/usr/bin/env python3
"""Build or install pinned E12/E24 weights; preserve explicit E25 rollback."""
import argparse
import ast
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import subprocess
import tempfile
from types import SimpleNamespace
import zipfile

from bevfusion_production_profile import ROOT, digest, profile
from prepare_bevfusion_production import DRIVE_ROOT, prepare

DESCRIPTOR = ROOT / 'configs/perception/base5_production_release.json'
SCHEMA = 'ajounice_base5_production_bundle_v1'
WEIGHTS = 'weights/base5_e24_inference.pth'
E12_DESCRIPTOR = ROOT / 'configs/perception/base5_e12_production_release.json'


def descriptor_path(model):
    return E12_DESCRIPTOR if model == 'e12' else DESCRIPTOR


def weights_member(model):
    if model not in ('e12', 'e24'):
        raise ValueError('Base5 checkpoint member requires E12 or E24')
    return 'weights/base5_' + model + '_inference.pth'


def config_files(path, root):
    result = set()
    def visit(path):
        path = path.resolve()
        relative = path.relative_to(root).as_posix()
        if relative in result:
            return
        result.add(relative)
        for node in ast.parse(path.read_text()).body:
            if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == '_base_' for t in node.targets):
                bases = ast.literal_eval(node.value)
                for base in ([bases] if isinstance(bases, str) else bases):
                    visit(path.parent / base)
    visit(path)
    return result


def required_sources(root=ROOT, model='e24'):
    p = profile(model)
    paths = config_files(root / p['model_config'], root)
    paths.update((p['bridge_config'], 'scripts/runtime/bevfusion_production_profile.py',
                  'scripts/runtime/run_bevfusion.sh', 'docker/bevfusion/compose.yaml',
                  'docker/bevfusion/scripts/run_inference_server.sh', 'docker/bevfusion/scripts/healthcheck.py',
                  'base_ws/src/perception/bevfusion_ros/scripts/bevfusion_ros_bridge.py',
                  'base_ws/src/perception/bevfusion_ros/launch/base5_production.launch',
                  'base_ws/src/perception/bevfusion_ros/launch/base3_online.launch'))
    paths.update(('perception/detection3d/bevfusion/__init__.py',
                  'perception/detection3d/bevfusion/view_geometry.py',
                  'perception/detection3d/bevfusion/depth_raster.py',
                  'base_ws/src/perception/bevfusion_ros/src/bevfusion_ros/box_center_origin.py',
                  'base_ws/src/sensor_interface/launch/lidar.launch',
                  'base_ws/src/planning_addons/planning_scenario_validator/src/planning_scenario_validator/models.py',
                  'base_ws/src/planning_addons/planning_scenario_validator/scripts/run_bevfusion_trial.py'))
    # Registry imports need the plugin modules, not unrelated legacy configs,
    # training experiments or benchmark/test scripts.
    for folder in ('perception/detection3d/bevfusion/plugin', 'docker/bevfusion/app'):
        paths.update(path.relative_to(root).as_posix() for path in (root / folder).rglob('*.py'))
    if model == 'e12':
        paths.update(('scripts/runtime/bevfusion_acceleration_profile.py',
                      'docker/bevfusion/Dockerfile.production-tensorrt',
                      'docker/bevfusion/scripts/setup_production_tensorrt.py',
                      'docker/bevfusion/requirements/tensorrt-inference-experiment.txt',
                      'perception/detection3d/bevfusion/scripts/export_base5_camera_tensorrt.py'))
    return sorted(paths)


def build(checkpoint, output, remote_directory=None, model='e12', descriptor_output=None):
    if model not in ('e12', 'e24'):
        raise ValueError('Base5 bundles support only E12 and E24')
    p = profile(model)
    remote_directory = remote_directory or 'model/bevfusion/production/base5_' + model + '_261023'
    if digest(checkpoint) != p['sha256']:
        raise ValueError('Not the selected ' + model.upper() + ' inference checkpoint')
    if output.exists():
        raise FileExistsError('Use a new bundle path; existing handoffs are preserved')
    if descriptor_output is not None and descriptor_output.exists():
        raise FileExistsError('Use a new descriptor path; existing releases are preserved')
    member = weights_member(model)
    sources = {name: digest(ROOT / name) for name in required_sources(model=model)}
    manifest = dict(schema=SCHEMA, model=model, checkpoint_member=member,
                    checkpoint_sha256=p['sha256'], source_files_sha256=sources,
                    runtime=dict(image_dim=[512, 1056], voxel_xy_m=0.05, camera_bev_xy_m=0.20,
                                 depth_step_m=0.25, thresholds=p['thresholds'], amp=True,
                                 parallel_camera=True, compile_parts=[], point_intensity_source='ros255',
                                 depth_raster_policy=p.get('depth_raster_policy'),
                                 camera_tensorrt_requested=p['camera_tensorrt'],
                                 head_attention_sdpa=p['head_attention_sdpa'],head_cache_bev_pos=p['head_cache_bev_pos'],
                                 box_center_output_origin=p.get('box_center_output_origin','legacy_passthrough')))
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        archive.write(checkpoint, member)
        archive.writestr('bundle_manifest.json', json.dumps(manifest, indent=2, sort_keys=True) + '\n')
        archive.writestr('README.txt',
            'AjouNice Base5 ' + model.upper() + ' production handoff\n'
            'Install with the matching Git checkout:\n'
            'bash scripts/runtime/install_base5_production.sh --model ' + model + ' --archive /path/to/this.zip\n'
            'Weights install to ~/bevfusion_artifacts/checkpoints/' + p['checkpoint'] + '.\n'
            'reference/ is an audit copy, not code to overwrite your checkout.\n'
            'No Docker image, credentials, training state, or dataset is included.\n')
        for name in sources:
            archive.write(ROOT / name, 'reference/' + name)
    if any(digest(ROOT / name) != value for name, value in sources.items()):
        raise ValueError('Runtime changed while packing; do not publish this bundle')
    descriptor = dict(schema=SCHEMA, model=model, archive=output.name, checkpoint_member=member,
                      archive_sha256=digest(output), archive_bytes=output.stat().st_size,
                      remote_directory=remote_directory,
                      checkpoint_sha256=p['sha256'], checkpoint_bytes=checkpoint.stat().st_size,
                      source_files_sha256=sources)
    if descriptor_output is not None:
        descriptor_output.parent.mkdir(parents=True, exist_ok=True)
        with descriptor_output.open('x') as stream:
            json.dump(descriptor, stream, indent=2, sort_keys=True)
            stream.write('\n')
    print(json.dumps(descriptor, indent=2, sort_keys=True))
    return descriptor


def validate_descriptor(doc, root=ROOT, *, require_matching_sources=True, model=None):
    """Verify the pinned archive; exact checkout matching is an audit option.

    A reference snapshot authenticates what was shipped, not every future
    compatible checkout. Installing identical E24 weights must not require a
    new 429MiB ZIP after a launcher, diagnostic or opt-in accelerator update.
    Actual current-model/bridge/runtime compatibility belongs to startup and
    preflight, never to a silent source overwrite from the archive.
    """
    selected = doc.get('model')
    if (doc['schema'] != SCHEMA or selected not in ('e12', 'e24') or
            (model is not None and selected != model) or
            doc['checkpoint_sha256'] != profile(selected)['sha256']):
        raise ValueError('Pinned selected E12/E24 bundle descriptor required')
    if doc.get('checkpoint_member', weights_member(selected)) != weights_member(selected):
        raise ValueError('Bundle checkpoint member differs from selected model')
    sources = doc['source_files_sha256']
    if not isinstance(sources, dict) or not sources:
        raise ValueError('Bundle source snapshot is missing')
    for name, expected in sources.items():
        path = PurePosixPath(name)
        if (path.is_absolute() or '..' in path.parts or '\\' in name
                or not isinstance(expected, str) or len(expected) != 64
                or any(c not in '0123456789abcdef' for c in expected)):
            raise ValueError('Invalid bundle source identity: ' + name)
    if not require_matching_sources:
        return
    if set(sources) != set(required_sources(root, selected)):
        raise ValueError('Bundle runtime source coverage differs from this checkout')
    for name, expected in sources.items():
        if digest(root / name) != expected:
            raise ValueError('Runtime differs from the release; pull the matching branch: ' + name)


def validate_archive(path, doc):
    member = weights_member(doc['model'])
    if path.stat().st_size != doc['archive_bytes'] or digest(path) != doc['archive_sha256']:
        raise ValueError('Archive checksum/size mismatch; nothing installed')
    with zipfile.ZipFile(path) as archive:
        seen = set()
        total = 0
        for info in archive.infolist():
            name = PurePosixPath(info.filename)
            mode = info.external_attr >> 16
            if (name.is_absolute() or '..' in name.parts or '\\' in info.filename
                    or info.filename in seen or info.is_dir() or stat.S_ISLNK(mode)):
                raise ValueError('Unsafe or duplicate bundle member')
            seen.add(info.filename)
            total += info.file_size
        if total > 800 * 1024**2:
            raise ValueError('Unexpected uncompressed bundle size')
        manifest = json.loads(archive.read('bundle_manifest.json'))
        expected = {'bundle_manifest.json', 'README.txt', member}
        expected.update('reference/' + name for name in doc['source_files_sha256'])
        if (seen != expected or manifest.get('schema') != SCHEMA or
                manifest.get('model') != doc['model'] or manifest.get('checkpoint_member') != member):
            raise ValueError('Unexpected bundle contents/schema')
        if (manifest.get('checkpoint_sha256') != doc['checkpoint_sha256'] or
                manifest.get('source_files_sha256') != doc['source_files_sha256'] or
                archive.getinfo(member).file_size != doc['checkpoint_bytes']):
            raise ValueError('Bundle identity differs from pinned descriptor')
        for name, expected_hash in doc['source_files_sha256'].items():
            if hashlib.sha256(archive.read('reference/' + name)).hexdigest() != expected_hash:
                raise ValueError('Reference source checksum mismatch: ' + name)


def install(args):
    # The Python API historically selected E24 when no model attribute exists;
    # CLI and shell always supply the new explicit/default E12 selection.
    model = getattr(args, 'model', 'e24')
    if model == 'e25':
        if args.archive is not None:
            raise ValueError('E25 rollback uses existing legacy weights, not a Base5 ZIP')
        prepare(SimpleNamespace(model='e25', source_checkpoint=None, check_only=True))
        return
    doc = json.loads(descriptor_path(model).read_text())
    validate_descriptor(doc, require_matching_sources=False, model=model)
    destination = Path(profile(model)['checkpoint_path'])
    if destination.exists():
        if digest(destination) != doc['checkpoint_sha256']:
            raise ValueError('Different existing weights; preserve them before installing')
        if args.archive:
            validate_archive(args.archive, doc)
        print(json.dumps(dict(event='base5_' + model + '_ready', model=model, checkpoint=str(destination), downloaded=False,
                             runtime_snapshot_enforced=False, runtime_certified=False,
                             next_check='python3 scripts/runtime/preflight_bevfusion_competition.py --safe')))
        return
    if args.check_only:
        raise FileNotFoundError(model.upper() + ' not installed: ' + str(destination))
    archive_path = args.archive
    if archive_path is None:
        cache = Path.home() / 'bevfusion_artifacts/downloads'
        cache.mkdir(parents=True, exist_ok=True)
        archive_path = cache / doc['archive']
        if not archive_path.exists():
            with tempfile.TemporaryDirectory(prefix='base5_download_', dir=cache) as temp:
                downloaded = Path(temp) / doc['archive']
                remote = args.remote + ':' + doc['remote_directory'] + '/' + doc['archive']
                subprocess.run(['rclone', 'copyto', remote, str(downloaded), '--drive-root-folder-id',
                                DRIVE_ROOT, '--progress', '--transfers', '1', '--tpslimit', '2',
                                '--retries', '2', '--low-level-retries', '2'], check=True)
                validate_archive(downloaded, doc)
                os.link(downloaded, archive_path)
    validate_archive(archive_path, doc)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='base5_unzip_', dir=destination.parent) as temp:
        weights = Path(temp) / (model.upper() + '.pth')
        with zipfile.ZipFile(archive_path) as archive, archive.open(weights_member(model)) as source, weights.open('xb') as target:
            shutil.copyfileobj(source, target, length=8 * 1024**2)
        prepare(SimpleNamespace(model=model, source_checkpoint=weights, check_only=False))
    receipt = dict(schema=SCHEMA, model=model, checkpoint=str(destination),
                   checkpoint_sha256=digest(destination), archive_sha256=doc['archive_sha256'],
                   source_files_sha256=doc['source_files_sha256'])
    print(json.dumps(dict(event='base5_' + model + '_installed', model=model, checkpoint=str(destination),
                          checkpoint_sha256=receipt['checkpoint_sha256'],
                          archive_sha256=receipt['archive_sha256'],
                          verified_source_files=len(receipt['source_files_sha256']),
                          runtime_snapshot_enforced=False, runtime_certified=False,
                          next_check='python3 scripts/runtime/preflight_bevfusion_competition.py --safe')))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    pack = sub.add_parser('build')
    pack.add_argument('--checkpoint', type=Path, required=True)
    pack.add_argument('--output', type=Path, required=True)
    pack.add_argument('--model',choices=('e12','e24'),default='e12')
    pack.add_argument('--remote-directory')
    pack.add_argument('--descriptor-output',type=Path)
    deploy = sub.add_parser('install')
    deploy.add_argument('--archive', type=Path)
    deploy.add_argument('--remote', default='ajou_drive')
    deploy.add_argument('--check-only', action='store_true')
    deploy.add_argument('--model',choices=('e12','e24','e25'),default='e12')
    audit = sub.add_parser('audit-runtime-snapshot', help='Strict historical release/checkout byte comparison; not required to install the same weights')
    audit.add_argument('--model',choices=('e12','e24'),default='e12')
    args = parser.parse_args()
    if args.action == 'build':
        build(args.checkpoint, args.output,args.remote_directory,args.model,args.descriptor_output)
    elif args.action == 'audit-runtime-snapshot':
        validate_descriptor(json.loads(descriptor_path(args.model).read_text()),model=args.model)
    else:
        install(args)
