#!/usr/bin/env python3
"""Pack an already-qualified JPEG three-class dataset, without capture/re-QA.

The default is a read-only plan. --run creates one fresh tar.zst with dataset/
and minimal provenance/, then verifies every decompressed archive member using
the existing portable finisher. No SSH, Drive, training or PNG source cleanup.
Native annotation/mask/depth files already in the JPEG package stay lossless.
Only trusted local exporter pickle inputs are supported.
"""
import argparse
from collections import Counter
import importlib.util
import json
import os
from pathlib import Path
import pickle
import sys
import time

SPEC = importlib.util.spec_from_file_location('base5_existing_cpu_finisher',
                                             Path(__file__).with_name('finish.py'))
finish = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(finish)

SCHEMA = 'base5_qualified_training_archive_v1'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def no_symlinks(path, must_exist=True, directory=False):
    raw = Path(path)
    require('..' not in raw.parts, 'Normalized path required')
    path = raw.absolute()
    for part in (path, *path.parents):
        require(not part.is_symlink(), 'Symlink forbidden: ' + str(part))
    if must_exist:
        require(path.is_dir() if directory else path.is_file(), 'Missing/wrong input: ' + str(path))
    return path


def load_selection(path, expected):
    raw = path.read_bytes()
    require(raw.endswith(b'\n'), 'Strict newline-terminated final selected ledger required')
    rows = [json.loads(line) for line in raw.splitlines()]
    require(len(rows) == expected and expected > 0, 'Final selection count differs')
    for key in ('id', 'capture_root', 'scenario_sha256'):
        require(len({r[key] for r in rows}) == expected, 'Duplicate selected ' + key)
    require(len({r.get('source_cell_id', r['id']) for r in rows}) == expected,
            'Duplicate final logical quota slot')
    require(all(r.get('status') == 'accepted_candidate' for r in rows),
            'Unapproved scene cannot enter the training archive')
    return rows


def qualification(path, package, selected, overlay, expected):
    document = finish.read(path)
    require(document.get('schema') == 'base5_training_input_qualification_v1',
            'Successful CPU training input qualification report required')
    candidates = list(document['cohorts'].values()) if 'cohorts' in document else [document]
    matching = [r for r in candidates if Path(r.get('training3_package', '')).absolute() == package]
    require(len(matching) == 1, 'Qualification does not bind exactly this final package')
    cohort = matching[0]
    require(cohort.get('frames') == expected and cohort.get('source_unchanged') is True
            and cohort.get('lost_retained_component_ids') == 0
            and cohort.get('extra_retained_component_ids') == 0
            and cohort.get('training_label_policy') == 'animal_to_pedestrian_v1',
            'Qualification is incomplete or has component/label discrepancies')
    require(cohort.get('training3_manifest_sha256') == finish.digest(package / 'manifest.json'),
            'Qualified final package manifest changed')
    require(cohort.get('source_ledger_sha256') == finish.digest(selected),
            'Qualification does not bind this authoritative final selection')
    require(document.get('split_overlay', {}).get('sha256') == finish.digest(overlay),
            'Qualification does not bind this final split overlay')
    rgb = cohort.get('rgb', {})
    require(rgb.get('images_checked') == expected * 4 and rgb.get('native_jpeg_quality') == 90
            and rgb.get('lossless_auxiliary_files_preserved') is True,
            'Four-camera native Q90/lossless auxiliary qualification required')
    return document, cohort


def prepare(args):
    package = no_symlinks(args.package, directory=True)
    report = no_symlinks(args.qualification_report)
    selected = no_symlinks(args.selected)
    overlay = no_symlinks(args.split_overlay)
    output = no_symlinks(args.output, must_exist=False)
    receipt = no_symlinks(args.receipt_dir, must_exist=False)
    require(output.name.endswith('.tar.zst'), 'One .tar.zst output is required')
    partial = output.with_suffix(output.suffix + '.partial')
    require(not any(p.exists() or p.is_symlink() for p in (output, partial, receipt)),
            'Fresh archive, partial and receipt directory required; never overwrite')
    for protected in (package, selected, report, overlay):
        require(not finish.within(output, protected) and not finish.within(receipt, protected)
                and not finish.within(protected, receipt), 'Archive/receipt overlap protected inputs')
    require(not finish.within(output, receipt) and not finish.within(receipt, output),
            'Archive and receipt paths must be separate')
    require(1 <= args.compression_level <= 19 and 1 <= args.threads <= 16,
            'Bounded compression level/threads required')
    require(args.max_usage_gib > 0 and args.min_free_gib >= 0, 'Invalid disk budget')
    manifest = finish.validate_training_package(package, args.expected_scenes)
    document, cohort = qualification(report, package, selected, overlay, args.expected_scenes)
    rows = load_selection(selected, args.expected_scenes)
    by_token = {r['id']: r for r in rows}
    require(set(manifest.get('provenance', {})) == set(by_token),
            'Final package capture provenance differs from selection')
    for token, row in by_token.items():
        require(manifest['provenance'][token].get('original_capture_root') == row['capture_root']
                and manifest['files'].get('data/' + token + '/scenario.json', {}).get('sha256')
                == row['scenario_sha256'], 'Final scenario/root provenance differs from selection')
    payload = {str(p.relative_to(package)) for p in finish.regular_files(package)}
    require(payload == set(manifest['files']) | {'manifest.json'},
            'Final package contains unregistered scratch/cache/other files')
    require(not any(Path(name).name.endswith('_Intensity.png') and
                    any(part.startswith('CAMERA_') for part in Path(name).parts)
                    for name in payload), 'Camera RGB PNG must not enter JPEG-only archive')
    overlay_doc = finish.read(overlay)
    require(overlay_doc.get('schema') == 'base5_layout_split_overlay_v1', 'Final split overlay required')
    overlay_sha = overlay_doc.get('overlay_sha256')
    require(bool(overlay_sha), 'Overlay logical digest missing')
    counts, found = Counter(), set()
    for split in ('train', 'val', 'test'):
        info = pickle.loads((package / 'infos/512x1056' /
                            ('morai_base5_infos_' + split + '.pkl')).read_bytes())
        require(info['metainfo'].get('layout_split_overlay_sha256') == overlay_sha,
                'Final infos do not bind the provided split overlay')
        for row in info['data_list']:
            token = row['token']
            require(token in by_token and token not in found, 'Final selected token missing/duplicated in infos')
            found.add(token); counts[split] += 1
            require(row.get('split_overlay_sha256') == overlay_sha,
                    'Frame overlay identity differs')
            assignment = overlay_doc['assignments'].get(by_token[token].get('source_cell_id', token))
            require(assignment and assignment['split'] == split
                    and assignment['group_id'] == row['scene_group_id']
                    and assignment['scenario_sha256'] == by_token[token]['scenario_sha256'],
                    'Final split/group/scenario assignment differs')
    require(found == set(by_token) and dict(counts) == cohort['split_frames']
            and dict(counts) == manifest['split_counts'], 'Final split counts/selection differ')
    provenance = dict(selected_ledger=selected, split_overlay=overlay, qualification_report=report)
    pins = {str(p): dict(bytes=p.stat().st_size, sha256=finish.digest(p))
            for p in [package / 'manifest.json', *provenance.values(),
                      Path(__file__).resolve(), Path(finish.__file__).resolve()]}
    roots = [package, output, receipt] + [no_symlinks(p, directory=True) for p in args.usage_root]
    budget = finish.Budget(roots, output, args.max_usage_gib * finish.GIB,
                           args.min_free_gib * finish.GIB)
    members = finish.package_members(package, {})
    members.update({'provenance/selected_ledger.jsonl': selected,
                    'provenance/split_overlay.json': overlay,
                    'provenance/qualification_report.json': report})
    # Only three explicitly named small files enter extra provenance. A
    # directory is not accepted, so no credential/raw/cache tree can sneak in.
    names = {'dataset/' + name for name in payload}
    names |= {'provenance/selected_ledger.jsonl', 'provenance/split_overlay.json',
              'provenance/qualification_report.json'}
    require(set(members) == names, 'Unexpected archive member scope')
    reserve = sum(p.stat().st_size + 2048 for p in members.values()) + 1024 ** 2
    disk = budget.check(reserve)
    contract = dict(schema=SCHEMA, package=str(package), qualification_report=str(report),
        selected=str(selected), split_overlay=str(overlay), expected_scenes=args.expected_scenes,
        output=str(output), receipt_dir=str(receipt), pins=pins, split_counts=dict(counts),
        source_manifest_sha256=finish.digest(package / 'manifest.json'),
        scope='dataset/ plus final selection, split overlay and qualification report; no RGB PNG/cache/raw trees',
        source_pixels='native Q90 JPEG, lossy relative to original PNG; native auxiliaries lossless',
        native_capture_requalification=False, training_started=False, network_transfer=False,
        compression_level=args.compression_level, threads=args.threads)
    return contract, members, budget, disk


def pack(args):
    contract, members, budget, disk = prepare(args)
    result = dict(event='qualified_training_archive_plan', execute=bool(args.run),
        frames=contract['expected_scenes'], split_counts=contract['split_counts'],
        output=contract['output'], receipt_dir=contract['receipt_dir'],
        members=len(members), source_bytes=sum(p.stat().st_size for p in members.values()), disk=disk,
        native_capture_requalification=False, network_transfer=False, training_started=False)
    if not args.run:
        return result
    output, receipt = Path(contract['output']), Path(contract['receipt_dir'])
    output.parent.mkdir(parents=True, exist_ok=True)
    no_symlinks(output.parent, directory=True)
    receipt.mkdir(parents=True, exist_ok=False)
    no_symlinks(receipt, directory=True)
    finish.write_new(receipt / 'intent.json', dict(contract, started_at=time.time()))
    print(json.dumps(dict(event='qualified_training_archive_compressing',
                          output=str(output), members=len(members))), flush=True)
    try:
        # The legacy helper renames its .partial. Keep that internal rename in
        # this fresh owned receipt folder, then publish the requested archive
        # with link(2), which cannot overwrite a destination created meanwhile.
        staging = receipt / 'verified_payload.tar.zst'
        archived = finish.archive_single(members, staging, budget,
                                          args.compression_level, args.threads)
        # archive_single decompresses and hashes every member before publishing.
        # Compare that readback to the qualified payload inventory as well, not
        # merely to a post-qualification snapshot from the compression window.
        manifest = finish.read(Path(contract['package']) / 'manifest.json')
        expected = {'dataset/' + name: item for name, item in manifest['files'].items()}
        for name, item in expected.items():
            require(archived['members'][name] == item, 'Archived payload differs from qualified manifest')
        for path, pin in contract['pins'].items():
            require(Path(path).stat().st_size == pin['bytes'] and finish.digest(path) == pin['sha256'],
                    'Pinned provenance/helper changed during compression')
        os.link(staging, output)
        staging.unlink()
        archived['archive'] = str(output)
        finish.write_new(receipt / 'archive_manifest.json', archived)
        with (receipt / 'SHA256SUMS').open('x') as stream:
            stream.write(archived['sha256'] + '  ' + output.name + '\n')
            stream.flush(); os.fsync(stream.fileno())
        final = dict(result, event='qualified_training_archive_complete', complete=True,
            archive=archived['archive'], bytes=archived['bytes'], sha256=archived['sha256'],
            archive_manifest=str(receipt / 'archive_manifest.json'),
            checksum=str(receipt / 'SHA256SUMS'), verified_decompressed_members=True,
            source_package_unchanged=True, completed_at=time.time())
        finish.write_new(receipt / 'COMPLETE.json', final)
        return final
    except BaseException as error:
        finish.write_new(receipt / 'INCOMPLETE.json', dict(complete=False, error=str(error),
            error_type=type(error).__name__, partial_products_preserved=True,
            note='No source deletion or automatic retry; inspect this fresh output separately.'))
        raise


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('package', 'qualification-report', 'selected', 'split-overlay', 'output', 'receipt-dir'):
        p.add_argument('--' + name, type=Path, required=True)
    p.add_argument('--expected-scenes', type=int, default=5000)
    p.add_argument('--max-usage-gib', type=float, default=100.)
    p.add_argument('--min-free-gib', type=float, default=10.)
    p.add_argument('--usage-root', type=Path, action='append', default=[])
    p.add_argument('--compression-level', type=int, default=6)
    p.add_argument('--threads', type=int, default=2)
    p.add_argument('--run', action='store_true')
    return p


if __name__ == '__main__':
    try:
        print(json.dumps(pack(parser().parse_args()), indent=2))
    except (ValueError, OSError, KeyError, pickle.UnpicklingError) as error:
        print(json.dumps(dict(event='qualified_training_archive_blocked', error=str(error))),
              file=sys.stderr)
        raise SystemExit(2)
