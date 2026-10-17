#!/usr/bin/env python3
"""v158a 경로의 Python 3.8 누적 거리만 복원하는 명시적 portable adapter.

원형 generator/extent/지도/manifest는 수정하지 않는다. 다른 Python의 hypot
마지막 비트 차이는 원본 좌표와 동결 route SHA를 모두 입증한 뒤에만 허용한다.
import/verify는 읽기 전용이며 export만 지정한 새 witness 파일을 생성한다.
"""
import argparse
from contextlib import contextmanager
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import sys


SCHEMA = 'base5_portable_canonical_route_witness_v1'
POLICY = 'v158a-python38-canonical-arc-only-v1'
CANONICAL_MANIFEST_SHA256 = '4f7fc3b31e23688b43913f2457d570cf014c19e04b2c2e391b463647809e02c5'
CANONICAL_ROUTE_SHA256 = 'beb686e71457bf93b20a80d06b98cd413d47bf21c08770e98469ad5ba5260b4c'
CANONICAL_POINTS = 4392
MAXIMUM_ARC_DIFFERENCE_M = 1e-9
AXES = ('x', 'y', 'z', 'link_idx')


class RouteCompatibilityError(RuntimeError):
    """계약 불일치는 부족분이 아니며 어떤 지도/QA 검사도 우회하지 않는다."""


def require(value, message):
    if not value:
        raise RouteCompatibilityError(message)


def sha256(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            value.update(block)
    return value.hexdigest()


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
        allow_nan=False).encode()).hexdigest()


def read(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError, TypeError) as error:
        raise RouteCompatibilityError('Canonical route input missing or malformed: '+str(path)) from error


def no_symlink(path):
    path = Path(path).expanduser().absolute()
    require('..' not in path.parts, 'Normalized canonical route path required')
    require(all(not part.is_symlink() for part in (path, *path.parents)),
            'Canonical route input/output cannot use symlinks')
    return path


def source_files(generator, extent, manifest, mgeo=None):
    """호출자가 넘긴 실제 module/file을 읽는다. witness의 임의 경로는 실행하지 않는다."""
    geo = generator.geo
    paths = dict(manifest=Path(manifest), generator=Path(generator.__file__),
        geometry=Path(geo.__file__), extent=Path(extent.__file__),
        map_link=(Path(mgeo) if mgeo is not None else geo.DEFAULT_MGEO)/'link_set.json',
        guide=Path(geo.DEFAULT_ROUTE))
    return {name:dict(logical_path=str(no_symlink(path)), sha256=sha256(no_symlink(path)))
            for name,path in paths.items()}


def manifest_contract(manifest):
    require(sha256(manifest) == CANONICAL_MANIFEST_SHA256, 'Only the exact frozen v158a manifest is supported')
    deck = read(manifest)
    require(deck.get('count') == len(deck.get('scenes', [])) == 5000,
            'Complete frozen v158a 5K source required')
    require(deck.get('config', {}).get('static_mission_extent', {}).get('route_sha256')
            == CANONICAL_ROUTE_SHA256, 'Frozen extent canonical route SHA differs')
    return deck


def coordinates(route):
    require(isinstance(route, list) and len(route) == CANONICAL_POINTS,
            'Exactly 4392 canonical route points required')
    rows = []
    for row in route:
        require(isinstance(row, dict) and all(axis in row for axis in AXES), 'Complete route coordinates required')
        require(all(type(row[axis]) in (int,float) and math.isfinite(row[axis]) for axis in AXES[:3]),
                'Finite numeric canonical XYZ required')
        require(isinstance(row['link_idx'], str) and row['link_idx'], 'Explicit canonical route link required')
        rows.append({axis:row[axis] for axis in AXES})
    return rows


def arc_values(values):
    require(isinstance(values, list) and len(values) == CANONICAL_POINTS,
            'Complete canonical cumulative distance array required')
    require(all(type(value) in (int,float) and math.isfinite(value) for value in values),
            'Finite numeric cumulative route distance required')
    require(values[0] == 0. and all(after > before for before,after in zip(values,values[1:])),
            'Strictly increasing route distances starting at zero required')
    return list(values)


def export_witness(manifest, output, *, generator, extent, mgeo=None):
    """원래 생산 환경 Python 3.8에서만 새 canonical 증거를 만든다. 수집은 하지 않는다."""
    require(sys.version_info[:2] == (3,8), 'Canonical witness must be exported using Python 3.8')
    manifest_contract(manifest)
    output = no_symlink(output)
    require(not output.exists(), 'Fresh canonical witness output required; existing evidence is preserved')
    sources = source_files(generator, extent, manifest, mgeo)
    require(str(output) not in {item['logical_path'] for item in sources.values()}, 'Witness cannot replace a frozen input')
    geo = generator.geo
    guide,_ = geo.load_route(geo.DEFAULT_ROUTE)
    links = geo.load_json((Path(mgeo) if mgeo is not None else geo.DEFAULT_MGEO)/'link_set.json')
    built = geo.build_mgeo_guided_competition_route(guide, links)
    route = built[0] if isinstance(built, tuple) else built
    points = coordinates(route);arc = arc_values(geo.route_arc_coordinates(route))
    require(extent.route_sha256(dict(route=points, arc=arc)) == CANONICAL_ROUTE_SHA256,
            'Python 3.8 reconstruction did not reproduce the frozen canonical route SHA')
    document = dict(schema=SCHEMA, policy=POLICY, helper_sha256=sha256(__file__),
        producer_python=sys.version, producer_python_major_minor=[3,8], source_files=sources,
        manifest_sha256=CANONICAL_MANIFEST_SHA256, canonical_route_sha256=CANONICAL_ROUTE_SHA256,
        point_count=CANONICAL_POINTS, route_points=points, canonical_arc_m=arc,
        coordinates_sha256=digest(points), canonical_arc_sha256=digest(arc),
        maximum_arc_difference_m=MAXIMUM_ARC_DIFFERENCE_M,
        allowed_mutation="context['arc'] only", route_coordinates_modified=False,
        native_qa_unchanged=True, native_capture_executed=False, bulk_authorized=False)
    require(sources == source_files(generator, extent, manifest, mgeo), 'Frozen route inputs changed during export')
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = (json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
    with output.open('xb') as stream:
        stream.write(payload);stream.flush();os.fsync(stream.fileno())
    return output


def verified_witness(witness, *, generator, extent, manifest, expected_witness_sha256, mgeo=None):
    witness = no_symlink(witness)
    require(isinstance(expected_witness_sha256, str) and len(expected_witness_sha256) == 64
        and all(ch in '0123456789abcdef' for ch in expected_witness_sha256)
        and sha256(witness) == expected_witness_sha256, 'Explicit pinned canonical witness SHA required')
    doc = read(witness);manifest_contract(manifest)
    require(doc.get('schema') == SCHEMA and doc.get('policy') == POLICY
        and doc.get('helper_sha256') == sha256(__file__)
        and doc.get('producer_python_major_minor') == [3,8]
        and doc.get('manifest_sha256') == CANONICAL_MANIFEST_SHA256
        and doc.get('canonical_route_sha256') == CANONICAL_ROUTE_SHA256
        and doc.get('point_count') == CANONICAL_POINTS
        and doc.get('maximum_arc_difference_m') == MAXIMUM_ARC_DIFFERENCE_M
        and doc.get('allowed_mutation') == "context['arc'] only"
        and doc.get('route_coordinates_modified') is False
        and doc.get('native_qa_unchanged') is True
        and doc.get('native_capture_executed') is False and doc.get('bulk_authorized') is False,
        'Canonical witness profile, code or authority changed')
    current = source_files(generator, extent, manifest, mgeo)
    sources = doc.get('source_files', {})
    require(set(sources) == set(current)
        and all(isinstance(sources[name],dict) and sources[name].get('sha256') == item['sha256']
                for name,item in current.items()), 'Frozen map/guide/generator/extent/manifest bytes changed')
    points,arc = coordinates(doc.get('route_points')),arc_values(doc.get('canonical_arc_m'))
    require(digest(points) == doc.get('coordinates_sha256') and digest(arc) == doc.get('canonical_arc_sha256')
        and extent.route_sha256(dict(route=points, arc=arc)) == CANONICAL_ROUTE_SHA256,
        'Canonical coordinates/arc do not reproduce the original frozen route SHA')
    return doc


def verify_context(ctx, witness, *, generator, extent, manifest, expected_witness_sha256, mgeo=None):
    """좌표는 exact, 누적 거리만 <=1nm 차이인지 입증한다. ctx를 변경하지 않는다."""
    doc = verified_witness(witness, generator=generator, extent=extent, manifest=manifest,
        expected_witness_sha256=expected_witness_sha256, mgeo=mgeo)
    require(isinstance(ctx,dict), 'Generator context mapping required')
    points,arc = coordinates(ctx.get('route')),arc_values(ctx.get('arc'))
    require(points == doc['route_points'] and digest(points) == doc['coordinates_sha256'],
            'Actual MGeo route coordinates, order or link identity changed')
    differences = [abs(actual-canonical) for actual,canonical in zip(arc,doc['canonical_arc_m'])]
    require(max(differences) <= MAXIMUM_ARC_DIFFERENCE_M,
            'Route distance differs beyond the explicit 1e-9 m compatibility bound')
    return dict(passed=True, policy=POLICY, witness_sha256=expected_witness_sha256,
        manifest_sha256=CANONICAL_MANIFEST_SHA256, route_points=CANONICAL_POINTS,
        original_route_sha256=extent.route_sha256(ctx), canonical_route_sha256=CANONICAL_ROUTE_SHA256,
        actual_python=sys.version, changed_arc_entries=sum(value != 0 for value in differences),
        maximum_arc_difference_m=max(differences), maximum_allowed_arc_difference_m=MAXIMUM_ARC_DIFFERENCE_M,
        coordinates_exact=True, source_files_verified=True, context_modified=False,
        native_qa_unchanged=True, native_capture_executed=False, bulk_authorized=False)


def restore_context(ctx, witness, *, generator, extent, manifest, expected_witness_sha256, mgeo=None):
    """이미 검증된 새 context의 arc 하나만 원본으로 교체한다."""
    verify_context(ctx,witness,generator=generator,extent=extent,manifest=manifest,
        expected_witness_sha256=expected_witness_sha256,mgeo=mgeo)
    doc = verified_witness(witness,generator=generator,extent=extent,manifest=manifest,
        expected_witness_sha256=expected_witness_sha256,mgeo=mgeo)
    previous = ctx['arc']
    ctx['arc'] = list(doc['canonical_arc_m'])
    try:
        require(extent.route_sha256(ctx) == CANONICAL_ROUTE_SHA256,
                'Restored arc failed the unchanged original extent SHA check')
    except BaseException:
        ctx['arc'] = previous
        raise
    return ctx


@contextmanager
def installed(witness, *, generator, extent, manifest, expected_witness_sha256):
    """명시 scope에서만 원형 gen.context를 감싼다. 전역 수학/QA는 수정하지 않는다."""
    verified_witness(witness,generator=generator,extent=extent,manifest=manifest,
        expected_witness_sha256=expected_witness_sha256)
    original = generator.context
    def context(runtime, mgeo):
        fresh = original(runtime, mgeo)
        return restore_context(fresh,witness,generator=generator,extent=extent,manifest=manifest,
            expected_witness_sha256=expected_witness_sha256,mgeo=mgeo)
    generator.context = context
    try:
        yield
    finally:
        changed = generator.context is not context
        generator.context = original
        require(not changed, 'Canonical context adapter ownership changed during its scope')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['export','verify'])
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--witness', type=Path, required=True)
    parser.add_argument('--witness-sha256')
    parser.add_argument('--scripts', type=Path, required=True,
        help='Explicit frozen dataset/morai_od/scripts directory; no simulator or ROS access')
    args = parser.parse_args();sys.path.insert(0,str(no_symlink(args.scripts)))
    gen = importlib.import_module('generate_base5_independent')
    extent = importlib.import_module('base5_static_mission_extent')
    if args.action == 'export':
        path = export_witness(args.manifest,args.witness,generator=gen,extent=extent)
        print(json.dumps(dict(path=str(path),sha256=sha256(path),capture_executed=False)))
    else:
        ctx = gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
        print(json.dumps(verify_context(ctx,args.witness,generator=gen,extent=extent,
            manifest=args.manifest,expected_witness_sha256=args.witness_sha256),indent=2))


if __name__ == '__main__':
    main()
