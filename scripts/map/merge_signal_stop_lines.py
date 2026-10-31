#!/usr/bin/env python3
"""Rebase an isolated signal-stop patch onto a teammate's map, without shifts.

Strict three-way merge: preserve every incoming byte except qualified stop-node
coordinates and the explicitly requested MetaInfo map_version. Conflicting stop
edits are refused; source maps and existing outputs are never overwritten.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from shift_signal_stop_lines import GROUPS


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def structure(element):
    return (element.tag, tuple(sorted(element.attrib.items())),
            tuple(structure(child) for child in element))


def index_map(raw):
    root = ET.fromstring(raw)
    index = {}
    for kind in ('node', 'way', 'relation'):
        for item in root.findall(kind):
            key = (kind, item.get('id'))
            if key in index or key[1] is None:
                raise ValueError('Duplicate or missing object ID: '+str(key))
            index[key] = item
    for (kind, ident), item in index.items():
        references = ([('node', nd.get('ref')) for nd in item.findall('nd')] if kind == 'way'
            else [(m.get('type'), m.get('ref')) for m in item.findall('member')] if kind == 'relation' else [])
        if any(ref not in index for ref in references):
            raise ValueError('Dangling reference: '+str((kind, ident)))
    return root, index


def qualified_nodes(index):
    selected = set()
    for gid in GROUPS.values():
        relation = index[('relation', str(gid))]
        if {t.get('k'): t.get('v') for t in relation.findall('tag')}.get('subtype') != 'traffic_light':
            raise ValueError('Expected traffic regulatory element')
        lines = [m.get('ref') for m in relation.findall('member')
                 if m.get('type') == 'way' and m.get('role') == 'ref_line']
        if len(lines) != 1:
            raise ValueError('Require one explicit stop line per mission')
        way = index[('way', lines[0])]
        if {t.get('k'): t.get('v') for t in way.findall('tag')}.get('subtype') != 'stop_line':
            raise ValueError('Expected stop-line way')
        ids = {nd.get('ref') for nd in way.findall('nd')}
        if len(ids) != 2 or selected & ids:
            raise ValueError('Require two exclusive stop nodes')
        for (kind, wid), other in index.items():
            if kind == 'way' and wid != lines[0] and any(nd.get('ref') in ids for nd in other.findall('nd')):
                raise ValueError('Stop node shared with another way')
        selected.update(ids)
    return selected


def node_block(text, nid):
    matches = list(re.finditer(r'<node\b(?=[^>]*\bid="'+re.escape(nid)+r'")[^>]*>.*?</node>', text, re.S))
    if len(matches) != 1:
        raise ValueError('Require one tagged node block: '+nid)
    return matches[0].group()


def without_coordinates(node):
    return (tuple(sorted((k, v) for k, v in node.attrib.items() if k not in ('lat', 'lon'))),
            tuple(structure(child) for child in node
                  if not (child.tag == 'tag' and child.get('k') in ('local_x', 'local_y'))))


def merge_stop_lines(base, ours, incoming, map_version):
    if isinstance(map_version, bool) or not isinstance(map_version, int) or map_version <= 0:
        raise ValueError('Explicit positive integer map version required')
    base_root, bi = index_map(base)
    _, oi = index_map(ours)
    incoming_root, ti = index_map(incoming)
    targets = qualified_nodes(bi)
    if targets != qualified_nodes(oi) or targets != qualified_nodes(ti):
        raise ValueError('Stop-node identity changed')
    # Never apply coordinates to a different ref_line/mission association.
    for gid in GROUPS.values():
        rel = bi[('relation', str(gid))]
        line = next(m.get('ref') for m in rel.findall('member') if m.get('role') == 'ref_line')
        for key in (('relation', str(gid)), ('way', line)):
            if structure(bi[key]) != structure(oi[key]) or structure(bi[key]) != structure(ti[key]):
                raise ValueError('Stop graph/identity conflict: '+str(key))
    base_text, ours_text, result = (raw.decode('utf-8') for raw in (base, ours, incoming))
    ours_reversed = ours_text
    replacements = []
    already_applied = []
    for nid in sorted(targets, key=int):
        before, after = node_block(base_text, nid), node_block(ours_text, nid)
        if without_coordinates(bi[('node', nid)]) != without_coordinates(oi[('node', nid)]):
            raise ValueError('Our patch changes non-coordinate node properties')
        node = oi[('node', nid)]
        coordinate_values = [node.get(key) for key in ('lat', 'lon')]
        coordinate_values += [t.get('v') for t in node.findall('tag') if t.get('k') in ('local_x', 'local_y')]
        for value in coordinate_values:
            if not math.isfinite(float(value)):
                raise ValueError('Nonfinite coordinate')
        ours_reversed = ours_reversed.replace(after, before, 1)
        theirs = node_block(result, nid)
        if theirs == after:
            already_applied.append(int(nid))
        elif theirs == before:
            result = result.replace(theirs, after, 1)
            replacements.append((int(nid), theirs, after))
        else:
            # Even incoming metadata differences are refused, never discarded.
            raise ValueError('Incoming stop-node edit conflict: '+nid)
    if ours_reversed.encode() != base:
        raise ValueError('Our source contains changes outside the approved coordinate patch')
    metadata = incoming_root.findall('MetaInfo')
    if len(metadata) != 1 or metadata[0].get('map_version') is None:
        raise ValueError('Require one explicit MetaInfo map_version')
    header = re.findall(r'<MetaInfo\b[^>]*>', result)
    if len(header) != 1:
        raise ValueError('Unique MetaInfo header required')
    old_header = header[0]
    new_header, count = re.subn(r'\bmap_version="[^"]*"', 'map_version="'+str(map_version)+'"', old_header)
    if count != 1:
        raise ValueError('Unsupported map_version serialization')
    result = result.replace(old_header, new_header, 1)
    reversed_result = result.replace(new_header, old_header, 1)
    for _, old, new in replacements:
        reversed_result = reversed_result.replace(new, old, 1)
    if reversed_result.encode() != incoming:
        raise AssertionError('Unexpected change outside stop coordinates/version header')
    output = result.encode()
    _, merged = index_map(output)
    for key in ti:
        expected = oi[key] if key[0] == 'node' and key[1] in targets else ti[key]
        if structure(merged[key]) != structure(expected):
            raise AssertionError('Merged object differs from qualified source: '+str(key))
    return output, dict(base_sha256=sha(base), stop_patch_sha256=sha(ours), incoming_sha256=sha(incoming),
        merged_sha256=sha(output), map_version=map_version,
        incoming_map_version=metadata[0].get('map_version'),
        qualified_node_ids=sorted(map(int, targets)), changed_node_ids=[r[0] for r in replacements],
        already_applied_node_ids=already_applied,
        incoming_objects={kind: len(incoming_root.findall(kind)) for kind in ('node', 'way', 'relation')},
        changed_outside_qualified_nodes='MetaInfo map_version only',
        teammate_changes_byte_preserved=True, dangling_references=0,
        runtime_defaults_changed=False, live_revalidation_performed=False)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('base', 'ours', 'incoming', 'output', 'report'):
        p.add_argument('--'+name, type=Path, required=True)
    p.add_argument('--map-version', type=int, required=True)
    args = p.parse_args()
    inputs = [args.base, args.ours, args.incoming]
    if (args.output.exists() or args.report.exists() or args.output.resolve() == args.report.resolve()
            or args.output.resolve() in [p.resolve() for p in inputs]):
        p.error('Use fresh output/report paths; never overwrite source maps')
    originals = [p.read_bytes() for p in inputs]
    output, report = merge_stop_lines(*originals, args.map_version)
    report['source_paths'] = dict(zip(('base', 'ours', 'incoming'), map(lambda p: str(p.resolve()), inputs)))
    if any(p.read_bytes() != raw for p, raw in zip(inputs, originals)):
        raise ValueError('A source map changed during qualification')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('xb') as stream:
        stream.write(output)
    with args.report.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
