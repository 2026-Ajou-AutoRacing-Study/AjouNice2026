#!/usr/bin/env python3
"""Register explicit ID/mode/prepared-manifest mappings; never controls MORAI."""
import argparse
from pathlib import Path

from base5_shadow_ground_reuse import register, DEFAULT_SENSOR


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sensor', type=Path, default=DEFAULT_SENSOR)
    parser.add_argument('--selected-ledger', type=Path, required=True,
        help='Explicit immutable historical selection approving each prepared payload')
    parser.add_argument('--prepared', action='append', required=True,
        help='exact_source:SOURCE_ID:/path/manifest.json or same_quota_replacement:SOURCE_ID:/path/manifest.json')
    args = parser.parse_args()
    choices = []
    for value in args.prepared:
        mode, source_id, path = value.split(':', 2)
        choices.append(dict(source_id=source_id, mode=mode, prepared_manifest=path,
            selected_ledger=str(args.selected_ledger)))
    print(register(args.manifest, choices, args.output, args.sensor))
