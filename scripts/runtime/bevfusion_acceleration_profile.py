#!/usr/bin/env python3
"""Resolve optional local acceleration without cloud access or GPU imports.

A missing/stale receipt selects the SAME production model eagerly. Runtime
adapter validation remains authoritative for actual GPU/TRT compatibility.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex

SAFE_IMAGE = 'ajounice/bevfusion:team-cu128-v1'
E12_SHA = 'f3ade1bf17375c061ea6c0284d636cfc647c98b745f4534cb365499e1394dba8'
CONFIG_SHA = 'b963b78565699b62e3570081c75330db7cf1e27dba5d49fbcb2b71d27545d978'
CONTAINER_ROOT = Path('/opt/bevfusion_tensorrt')
KEYS = ('BEVFUSION_IMAGE', 'BEVFUSION_CAMERA_TENSORRT_ENGINE',
        'BEVFUSION_CAMERA_TENSORRT_METADATA', 'BEVFUSION_CAMERA_TENSORRT_POLICY',
        'BEVFUSION_HEAD_ATTENTION_SDPA', 'BEVFUSION_HEAD_CACHE_BEV_POS')


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(8 << 20), b''):
            value.update(block)
    return value.hexdigest()


def resolve_acceleration(model, environment, home=None):
    home = Path.home() if home is None else Path(home)
    root = (home / 'bevfusion_artifacts/tensorrt').resolve()
    receipt = Path(environment.get('BEVFUSION_ACCELERATION_RECEIPT') or
                   root / model / 'runtime.json').expanduser()
    selected = dict(zip(KEYS, (SAFE_IMAGE, '', '', 'off',
                              'true' if model == 'e12' else 'false',
                              'true' if model == 'e12' else 'false')))
    result = dict(environment=selected, reason='optional_acceleration_not_prepared',
                  artifact_root=str(root), receipt_path=str(receipt),
                  engine_path=None, metadata_path=None, receipt_loaded=False,
                  managed_image=False, engine_sha256=None)
    if environment.get('BEVFUSION_SAFE_MODE', '').lower() == 'true':
        selected.update(BEVFUSION_HEAD_ATTENTION_SDPA='false', BEVFUSION_HEAD_CACHE_BEV_POS='false')
        result['reason'] = 'safe_same_model_eager'
        return result
    if model == 'e12' and receipt.is_file():
        try:
            data = json.loads(receipt.read_text())
            if (data.get('schema') != 'bevfusion_production_tensorrt_setup_v1' or
                    data.get('complete') is not True or data.get('model') != 'e12' or
                    data.get('checkpoint_sha256') != E12_SHA or data.get('config_sha256') != CONFIG_SHA):
                raise ValueError('Receipt model/config identity differs')
            paths = {}
            for name in ('engine', 'metadata'):
                path = Path(data[name + '_path']).resolve(strict=True)
                relative = path.relative_to(root)
                container = str(CONTAINER_ROOT / relative)
                if data['container_' + name + '_path'] != container or digest(path) != data[name + '_sha256']:
                    raise ValueError('Receipt artifact path/SHA differs: ' + name)
                paths[name] = (str(path), container)
            image = data['image']
            if not isinstance(image, str) or not image or any(c.isspace() for c in image):
                raise ValueError('Invalid local image reference')
            selected.update(BEVFUSION_IMAGE=image,
                            BEVFUSION_CAMERA_TENSORRT_ENGINE=paths['engine'][1],
                            BEVFUSION_CAMERA_TENSORRT_METADATA=paths['metadata'][1],
                            BEVFUSION_CAMERA_TENSORRT_POLICY='prefer')
            result.update(reason='local_target_engine_prepared', receipt_loaded=True,
                          managed_image=True, engine_path=paths['engine'][0], metadata_path=paths['metadata'][0],
                          engine_sha256=data['engine_sha256'])
        except (OSError, ValueError, KeyError, TypeError, UnicodeError) as error:
            result['reason'] = 'optional_receipt_unavailable: ' + str(error)
    prepared_pair = (selected['BEVFUSION_CAMERA_TENSORRT_ENGINE'], selected['BEVFUSION_CAMERA_TENSORRT_METADATA'])
    # Older team installations export the universal baseline image in their
    # local .env.  It is not an explicit opt-out of a newly prepared E12
    # receipt: use policy=off/--safe for that.  Keep other custom images as
    # genuine overrides, without editing the operator's environment file.
    legacy_default_image = (result['receipt_loaded'] and
                            environment.get('BEVFUSION_IMAGE') == SAFE_IMAGE)
    for key in KEYS:
        if key in environment:
            if key == 'BEVFUSION_IMAGE' and legacy_default_image:
                continue
            selected[key] = environment[key]
    if 'BEVFUSION_IMAGE' in environment and not legacy_default_image:
        result['managed_image'] = False
    if ('BEVFUSION_CAMERA_TENSORRT_POLICY' not in environment and
            (selected['BEVFUSION_CAMERA_TENSORRT_ENGINE'] or selected['BEVFUSION_CAMERA_TENSORRT_METADATA'])):
        selected['BEVFUSION_CAMERA_TENSORRT_POLICY'] = 'prefer'
    actual_pair = (selected['BEVFUSION_CAMERA_TENSORRT_ENGINE'], selected['BEVFUSION_CAMERA_TENSORRT_METADATA'])
    if actual_pair != prepared_pair:
        result.update(engine_sha256=None, engine_path=None, metadata_path=None)
    if selected['BEVFUSION_CAMERA_TENSORRT_POLICY'] == 'off':
        selected.update(BEVFUSION_CAMERA_TENSORRT_ENGINE='', BEVFUSION_CAMERA_TENSORRT_METADATA='')
        result.update(engine_sha256=None, engine_path=None, metadata_path=None)
        if environment.get('BEVFUSION_CAMERA_TENSORRT_POLICY') == 'off':
            selected['BEVFUSION_IMAGE'] = SAFE_IMAGE
            result['managed_image'] = False
            result['reason'] = 'camera_tensorrt_policy_off_same_model_eager'
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=('e12', 'e24', 'e25'), default='e12')
    parser.add_argument('--shell', action='store_true')
    args = parser.parse_args()
    result = resolve_acceleration(args.model, os.environ)
    if args.shell:
        for key, value in result['environment'].items():
            if not value and key in ('BEVFUSION_CAMERA_TENSORRT_ENGINE', 'BEVFUSION_CAMERA_TENSORRT_METADATA'):
                print('unset ' + key)
            else:
                print('export {}={}'.format(key, shlex.quote(value)))
        print('export BEVFUSION_TENSORRT_ARTIFACTS={}'.format(shlex.quote(result['artifact_root'])))
        print('export BEVFUSION_ACCELERATION_MANAGED_IMAGE={}'.format(str(result['managed_image']).lower()))
        print('export BEVFUSION_ACCELERATION_REASON={}'.format(shlex.quote(result['reason'])))
        print('export BEVFUSION_ACCELERATION_ENGINE_SHA256={}'.format(shlex.quote(result['engine_sha256'] or '')))
    else:
        print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
