#!/usr/bin/env python3
"""Apply one offline visibility criterion to existing A/B/C/D stop reports.

No map movement, driving or runtime promotion. This is a retrospective engineering
acceptance check, not independent pixel GT or a prediction of every future stop.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


GROUPS = dict(A=201351, B=201365, C=201290, D=201326)
CRITERIA = dict(image_size=[1280, 720], front_bumper_offset_m=3.845,
                minimum_stop_duration_sec=3., median_top_range_px=[90., 105.],
                minimum_observed_top_px=80., maximum_abs_line_gap_m=.5)


def check_report(report, mission):
    if report['group_id'] != GROUPS[mission] or report['mission'] != mission:
        raise ValueError('Mission identity mismatch')
    if report['front_bumper_offset_m'] != CRITERIA['front_bumper_offset_m']:
        raise ValueError('Inconsistent vehicle reference point')
    windows = []
    for w in report['stationary_windows']:
        if w['duration_sec'] < CRITERIA['minimum_stop_duration_sec']:
            continue
        margin = w.get('stopped_detector_top_margin_px')
        gap = w.get('front_bumper_gap_to_map_line_m')
        if not margin or not gap:
            continue
        values = [margin['min'], margin['p50'], gap['min'], gap['max']]
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
            raise ValueError('Nonfinite measurement')
        low, high = CRITERIA['median_top_range_px']
        checks = dict(median_top_in_band=low <= margin['p50'] <= high,
                      observed_minimum_top=margin['min'] >= CRITERIA['minimum_observed_top_px'],
                      stop_near_map_line=max(abs(gap['min']), abs(gap['max'])) <=
                      CRITERIA['maximum_abs_line_gap_m'])
        windows.append(dict(duration_sec=w['duration_sec'], median_top_px=margin['p50'],
            minimum_top_px=margin['min'], checks=checks, geometry_pass=all(checks.values()),
            # Missing selection must remain visible; never turn a geometric pass
            # into a claim of uninterrupted perception or overall safe driving.
            selection_status=w.get('selection_status', {})))
    approach = report.get('approach_detector_top_margin_px')
    approach_min = approach['min'] if approach else None
    if approach_min is not None and not math.isfinite(approach_min):
        raise ValueError('Nonfinite approach measurement')
    approach_pass = approach_min is not None and approach_min >= CRITERIA['minimum_observed_top_px']
    return dict(mission=mission, eligible_stops=windows,
                observed_approach_minimum_top_px=approach_min,
                observed_approach_pass=approach_pass,
                status=('insufficient_evidence' if not windows else
                        'geometry_pass' if approach_pass and all(w['geometry_pass'] for w in windows) else 'review'))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error('Use a fresh output; preserve previous evidence')
    paths = {m: a.root/(('map_v91_B' if m == 'B' else f'map_v91_corridor_{m}')+
                        '_clearance_analysis_261026_v1/measurements.json') for m in GROUPS}
    reports = {m: json.loads(path.read_text()) for m, path in paths.items()}
    if len({r['geometry_sha256'] for r in reports.values()}) != 1:
        p.error('Reports do not use the same evaluation geometry')
    result = dict(schema_version=1, criteria=CRITERIA,
        scope='Retrospective selected-housing diagnostic; not independent validation or runtime promotion',
        selection_continuity_fixed=False, runtime_default_changed=False,
        measurement_sources={m: dict(path=str(path.resolve()),
            sha256=hashlib.sha256(path.read_bytes()).hexdigest(), bag=reports[m]['bag'])
            for m, path in paths.items()},
        groups=[check_report(reports[m], m) for m in GROUPS])
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
