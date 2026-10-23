#!/usr/bin/env python3
"""새 환경의 실제 manifest에 묶인 누적 거리 호환 증거.

지도·좌표·link·경로 순서는 exact로 대조한다. 생산 Python에서 동결한
route SHA를 재현할 때만 witness를 만들고, 소비 Python의 arc 오차가
1e-9m 이하일 때만 arc를 복원한다. 기존 Sunny13 witness는 재사용하지 않는다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
from contextlib import contextmanager
import hashlib
import importlib
import json
import math
import os
from pathlib import Path

SCHEMA = 'base5_fresh_environment_route_witness_v1'
POLICY = 'exact-new-manifest-coordinates-canonical-arc-only-v1'
MAXIMUM_ARC_DIFFERENCE_M = 1e-9
ENVIRONMENTS = ('sunny11', 'sunny13', 'sunny15', 'foggy11', 'foggy13', 'foggy15')
AXES = ('x', 'y', 'z', 'link_idx')


def require(value, message):
    if not value:raise RuntimeError(message)


def physical(value):
    path = Path(value).expanduser().absolute()
    require('..' not in path.parts and all(not p.is_symlink() for p in (path, *path.parents)),
            'Exact nonsymlink route path required')
    return path


def sha(path):
    result = hashlib.sha256()
    with physical(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):result.update(block)
    return result.hexdigest()


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
        allow_nan=False).encode()).hexdigest()


def read(path):return json.loads(physical(path).read_text())


def manifest_contract(manifest):
    deck = read(manifest)
    require(deck.get('count') == len(deck.get('scenes', [])) == 5000,
            'Complete new-environment 5K manifest required')
    environment = deck.get('config', {}).get('environment_tag')
    expected = deck.get('config', {}).get('static_mission_extent', {}).get('route_sha256')
    require(environment in ENVIRONMENTS and type(expected) is str and len(expected) == 64
            and all(c in '0123456789abcdef' for c in expected), 'Declared environment and canonical route SHA required')
    return deck, environment, expected


def sources(generator, extent, manifest, mgeo=None):
    geo = generator.geo
    paths = dict(manifest=manifest, generator=generator.__file__, geometry=geo.__file__,
        extent=extent.__file__, map_link=(Path(mgeo) if mgeo else geo.DEFAULT_MGEO)/'link_set.json',
        guide=geo.DEFAULT_ROUTE)
    return {name:dict(logical_path=str(physical(path)), sha256=sha(path)) for name,path in paths.items()}


def coordinates(route):
    require(type(route) is list and len(route) >= 2, 'At least two actual route points required')
    result = []
    for row in route:
        require(type(row) is dict and all(key in row for key in AXES)
            and all(type(row[key]) in (int,float) and math.isfinite(row[key]) for key in AXES[:3])
            and type(row['link_idx']) is str and row['link_idx'], 'Finite XYZ and explicit link identity required')
        result.append({key:row[key] for key in AXES})
    return result


def arcs(values, count):
    require(type(values) is list and len(values) == count
        and all(type(v) in (int,float) and math.isfinite(v) for v in values)
        and values[0] == 0 and all(b > a for a,b in zip(values, values[1:])),
        'Complete strictly increasing canonical arc required')
    return list(values)


def export_witness(manifest, output, *, generator, extent, mgeo=None):
    _, environment, expected = manifest_contract(manifest)
    output = physical(output)
    require(not output.exists(), 'Fresh witness output required')
    fixed = sources(generator, extent, manifest, mgeo)
    geo = generator.geo
    guide, _ = geo.load_route(geo.DEFAULT_ROUTE)
    links = geo.load_json((Path(mgeo) if mgeo else geo.DEFAULT_MGEO)/'link_set.json')
    built = geo.build_mgeo_guided_competition_route(guide, links)
    route = built[0] if isinstance(built, tuple) else built
    points = coordinates(route); arc = arcs(geo.route_arc_coordinates(route), len(points))
    require(extent.route_sha256(dict(route=points, arc=arc)) == expected,
            'Actual producer route does not reproduce the new manifest canonical SHA')
    document = dict(schema=SCHEMA, policy=POLICY, helper_sha256=sha(__file__),
        environment=environment, manifest_sha256=sha(manifest), canonical_route_sha256=expected,
        producer_python=sys.version, source_files=fixed, route_points=points, canonical_arc_m=arc,
        point_count=len(points), coordinates_sha256=digest(points), canonical_arc_sha256=digest(arc),
        maximum_arc_difference_m=MAXIMUM_ARC_DIFFERENCE_M, allowed_mutation="context['arc'] only",
        route_coordinates_modified=False, native_qa_unchanged=True,
        native_capture_executed=False, bulk_authorized=False)
    require(fixed == sources(generator, extent, manifest, mgeo), 'Route sources changed during export')
    with open(output,'x',opener=lambda name, flags: os.open(name, flags | os.O_NOFOLLOW, 0o600)) as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2, allow_nan=False);stream.write('\n')
    return output


def verified_witness(witness, *, generator, extent, manifest, expected_witness_sha256, mgeo=None):
    require(type(expected_witness_sha256) is str and len(expected_witness_sha256) == 64
        and sha(witness) == expected_witness_sha256, 'Explicit exact new-environment witness SHA required')
    document = read(witness)
    _, environment, expected = manifest_contract(manifest)
    require(document.get('schema') == SCHEMA and document.get('policy') == POLICY
        and document.get('helper_sha256') == sha(__file__)
        and document.get('environment') == environment and document.get('manifest_sha256') == sha(manifest)
        and document.get('canonical_route_sha256') == expected
        and document.get('maximum_arc_difference_m') == MAXIMUM_ARC_DIFFERENCE_M
        and document.get('allowed_mutation') == "context['arc'] only"
        and document.get('route_coordinates_modified') is False
        and document.get('native_qa_unchanged') is True and document.get('native_capture_executed') is False
        and document.get('bulk_authorized') is False, 'New environment route witness profile changed')
    require(document.get('source_files') == sources(generator, extent, manifest, mgeo),
            'New manifest/map/guide/generator/extent bytes or logical paths changed')
    points = coordinates(document.get('route_points')); arc = arcs(document.get('canonical_arc_m'), len(points))
    require(document.get('point_count') == len(points) and digest(points) == document.get('coordinates_sha256')
        and digest(arc) == document.get('canonical_arc_sha256')
        and extent.route_sha256(dict(route=points, arc=arc)) == expected, 'Canonical new route SHA does not match')
    return document


def restore_context(context, witness, *, generator, extent, manifest, expected_witness_sha256, mgeo=None):
    document = verified_witness(witness, generator=generator, extent=extent, manifest=manifest,
        expected_witness_sha256=expected_witness_sha256, mgeo=mgeo)
    points = coordinates(context.get('route')); arc = arcs(context.get('arc'), len(points))
    require(points == document['route_points'], 'Actual route XYZ/order/link differs; no correction allowed')
    maximum = max(abs(a-b) for a,b in zip(arc, document['canonical_arc_m']))
    require(maximum <= MAXIMUM_ARC_DIFFERENCE_M, 'Arc difference exceeds 1e-9m compatibility bound')
    previous = context['arc'];context['arc'] = list(document['canonical_arc_m'])
    try:require(extent.route_sha256(context) == document['canonical_route_sha256'], 'Restored canonical route SHA differs')
    except BaseException:
        context['arc'] = previous
        raise
    return context


@contextmanager
def installed(witness, *, generator, extent, manifest, expected_witness_sha256):
    verified_witness(witness, generator=generator, extent=extent, manifest=manifest,
        expected_witness_sha256=expected_witness_sha256)
    original = generator.context
    def checked(runtime, mgeo):
        return restore_context(original(runtime, mgeo), witness, generator=generator, extent=extent,
            manifest=manifest, expected_witness_sha256=expected_witness_sha256, mgeo=mgeo)
    generator.context = checked
    try:yield
    finally:
        changed = generator.context is not checked;generator.context = original
        require(not changed, 'Fresh route context adapter ownership changed')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scripts', type=Path, required=True)
    args = parser.parse_args(argv);sys.path.insert(0,str(physical(args.scripts)))
    gen = importlib.import_module('generate_base5_independent')
    extent = importlib.import_module('base5_static_mission_extent')
    path = export_witness(args.manifest,args.output,generator=gen,extent=extent)
    print(json.dumps(dict(path=str(path),sha256=sha(path),native_captures=0,bulk_authorized=False)))
    return 0


if __name__ == '__main__':raise SystemExit(main())
