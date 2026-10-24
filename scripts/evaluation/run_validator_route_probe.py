#!/usr/bin/env python3
"""Research-only validator entry point verifying the actual route before driving.

No production launch or validator resume guard is changed. This wrapper uses the
ordinary validator lifecycle (P/brake, route, controller, stop/P, owned cleanup).
The additional route assertion prevents a short near-exit route being reported
as a ring-traversal test. Use a fresh authoring YAML, never a frozen execution.
"""
import argparse
import hashlib
from pathlib import Path
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]


def contains_in_order(actual, required):
    remaining = iter(actual)
    return all(any(value == expected for value in remaining) for expected in required)


def source_receipt():
    package = ROOT / 'base_ws/src/perception_object_pipeline'
    install = ROOT / 'base_ws/install_aw149'
    pairs = [(package / 'scripts/map_based_prediction_node.py',
              install / 'lib/perception_object_pipeline/map_based_prediction_node.py')]
    pairs += [(package / 'src/perception_object_pipeline' / name,
               install / 'lib/python3/dist-packages/perception_object_pipeline' / name)
              for name in ('map_based_prediction.py', 'mixed_horizon.py')]
    pairs += [(package / 'launch' / name,
               install / 'share/perception_object_pipeline/launch' / name)
              for name in ('object_pipeline.launch', 'object_prediction.launch')]
    def digest(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()
    return dict(wrapper_sha256=digest(Path(__file__).resolve()),
                source_installed=[dict(source=str(source), source_sha256=digest(source),
                                       installed=str(installed), installed_sha256=digest(installed))
                                  for source, installed in pairs],
                map_sha256=digest(ROOT / 'kcity_map/lanelet_map/lanelet2_map_v85.osm'))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', type=Path)
    parser.add_argument('--require-lanelet', type=int, nargs='+', required=True)
    parser.add_argument('--check', action='store_true', help='Check YAML only, no MORAI actions')
    args = parser.parse_args(argv)
    sys.path.insert(0, str(ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/src'))
    from planning_scenario_validator import runner
    from planning_scenario_validator.runtime import ScenarioFailure, atomic_json
    from planning_scenario_validator.executions import is_snapshot
    if is_snapshot(args.config) or args.config.name == 'config_snapshot.yaml':
        parser.error('Route probes require a fresh authoring YAML, not a past execution')
    original = runner.request_route
    receipt = source_receipt()

    def verified_request(case, monitor, **kwargs):
        result = original(case, monitor, **kwargs)
        route = monitor.route
        actual = [] if route is None else [segment.preferred_primitive.id for segment in route.segments]
        passed = contains_in_order(actual, args.require_lanelet)
        evidence = dict(receipt, required_lanelets_in_order=args.require_lanelet,
                        actual_preferred_lanelets=actual, verified_before_drive=passed,
                        production_changed=False)
        atomic_json(monitor.events_path.parent / 'route_probe_contract.json', evidence)
        monitor.event('EVALUATION_ROUTE_VERIFIED' if passed else 'EVALUATION_ROUTE_MISMATCH',
                      required_lanelets=args.require_lanelet, actual_preferred_lanelets=actual)
        if not passed:
            raise ScenarioFailure('EVALUATION_ROUTE_MISMATCH',
                                  'Actual route does not traverse the required test lanelets')
        return result

    with patch.object(runner, 'request_route', verified_request):
        return runner.main([str(args.config)] + (['--check'] if args.check else []))


if __name__ == '__main__':
    raise SystemExit(main())
