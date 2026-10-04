#!/usr/bin/env python3
"""One frozen three-run MORAI round, then a joint visual report. No detached jobs."""
import argparse
import json
import subprocess
import sys
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--round', type=int, required=True)
    p.add_argument('--lc-speed-refresh', action='store_true')
    p.add_argument('--lc-diverse-search', action='store_true')
    p.add_argument('--lc-strict-maneuver-safety', action='store_true')
    p.add_argument('--lc-measured-motion-diagnostic-only', action='store_true')
    p.add_argument('--po-c2-bridge', action='store_true')
    p.add_argument('--interpolate-rising-velocity', action='store_true')
    p.add_argument('--lc-partial-acceleration-candidates', action='store_true')
    p.add_argument('--lc-continuous-bounded-output', action='store_true')
    p.add_argument('--lc-defer-pre-adjacency-normal', action='store_true')
    p.add_argument('--lc-adaptive-positive-grid', action='store_true')
    p.add_argument('--lc-future-mandatory-anticipation', action='store_true')
    p.add_argument('--lc-anchor-invariant-curvature', action='store_true')
    p.add_argument('--lc-post-end-bounded-tail', action='store_true')
    p.add_argument('--lc-full-envelope-preference', action='store_true')
    p.add_argument('--lc-c2-reference-window', action='store_true')
    p.add_argument('--lc-world-candidate', action='store_true',
                   help='Additional world-coordinate NORMAL LC candidate after native generation rejection; default OFF')
    p.add_argument('--lc-exact-frenet-endpoint', action='store_true',
                   help='Include the exact native Frenet terminal sample; default OFF, all safety checks retained')
    p.add_argument('--lc-candidate-reason-audit', action='store_true')
    p.add_argument('--lc-fresh-commit-input', action='store_true')
    p.add_argument('--cruise-reset-on-lead-change', action='store_true')
    p.add_argument('--cruise-precise-future-admission', action='store_true')
    p.add_argument('--lc-skip-unrequested-avoidance-search', action='store_true')
    p.add_argument('--lc-path-hazard-preferred-request', action='store_true')
    p.add_argument('--od-gt-velocity-observation', action='store_true')
    p.add_argument('--lc-physical-endpoint-consistency', action='store_true')
    p.add_argument('--bpp-final-ego-poll', action='store_true')
    p.add_argument('--lc-reachable-admission', action='store_true')
    p.add_argument('--lc-rss-positive-grid', action='store_true')
    p.add_argument('--lc-certified-target-only-speed-reference', action='store_true')
    p.add_argument('--lc-defer-unhelpful-preferred-request', action='store_true')
    p.add_argument('--root', type=Path,
                   default=Path('planner_logs/analysis/obstaclecruise_rounds_100'))
    a = p.parse_args()
    if a.lc_exact_frenet_endpoint and not (a.lc_strict_maneuver_safety and
            a.lc_continuous_bounded_output and a.lc_partial_acceleration_candidates and
            a.lc_physical_endpoint_consistency):
        p.error('exact Frenet endpoint requires strict, continuous, partial and physical-endpoint checks')
    if a.lc_world_candidate and not (a.lc_strict_maneuver_safety and
            a.lc_continuous_bounded_output and a.lc_partial_acceleration_candidates and
            a.lc_reachable_admission and a.lc_physical_endpoint_consistency):
        p.error('world candidate requires strict, continuous, partial, reachable and physical-endpoint checks')
    if not 1 <= a.round <= 100:
        p.error('round must be 1..100 (three runs per round)')
    if a.lc_measured_motion_diagnostic_only and not a.lc_strict_maneuver_safety:
        p.error('measured-motion diagnostic mode requires strict planned/current-state checks')
    if a.lc_continuous_bounded_output and not (a.lc_strict_maneuver_safety and a.lc_partial_acceleration_candidates):
        p.error('continuous bounded output requires strict checks and the partial candidate grid')
    if a.lc_defer_pre_adjacency_normal and not a.lc_continuous_bounded_output:
        p.error('pre-adjacency deferral requires continuous bounded output')
    if a.lc_reachable_admission and not a.lc_continuous_bounded_output:
        p.error('reachable admission requires continuous bounded output')
    if a.lc_c2_reference_window and not (a.lc_continuous_bounded_output and a.lc_physical_endpoint_consistency):
        p.error('reference window requires strict continuous output and physical endpoint consistency')
    if a.lc_fresh_commit_input and not a.lc_continuous_bounded_output:
        p.error('fresh commit input requires strict continuous bounded output')
    if a.lc_path_hazard_preferred_request and not a.lc_defer_unhelpful_preferred_request:
        p.error('path hazard requires the existing NORMAL preferred-request deferral')
    if (a.lc_rss_positive_grid or a.lc_certified_target_only_speed_reference or a.lc_defer_unhelpful_preferred_request) and not a.lc_continuous_bounded_output:
        p.error('certified reference, RSS positive grid and preferred request deferral require continuous bounded output')
    if a.lc_rss_positive_grid and not a.lc_adaptive_positive_grid:
        p.error('RSS positive grid requires the bounded adaptive family')
    if (a.lc_adaptive_positive_grid or a.lc_future_mandatory_anticipation or a.lc_anchor_invariant_curvature or a.lc_post_end_bounded_tail or a.lc_full_envelope_preference or a.lc_physical_endpoint_consistency) and not a.lc_continuous_bounded_output:
        p.error('adaptive grid and future-route anticipation require continuous bounded output')
    folder = a.root / ('round%03d' % a.round)
    folder.mkdir(parents=True, exist_ok=False)
    (folder/'round.json').write_text(json.dumps(dict(
        round=a.round, max_rounds=100, runs_per_round=3,
        detector='od_gt', localization='alias launch defaults',
        approved_lc_speed_refresh=a.lc_speed_refresh,
        lc_diverse_candidate_search=a.lc_diverse_search,
        lc_strict_maneuver_safety=a.lc_strict_maneuver_safety,
        lc_measured_motion_diagnostic_only=a.lc_measured_motion_diagnostic_only,
        po_c2_bridge=a.po_c2_bridge,
        interpolate_rising_velocity=a.interpolate_rising_velocity,
        lc_partial_acceleration_candidates=a.lc_partial_acceleration_candidates,
        lc_continuous_bounded_output=a.lc_continuous_bounded_output,
        lc_defer_pre_adjacency_normal=a.lc_defer_pre_adjacency_normal,
        lc_adaptive_positive_grid=a.lc_adaptive_positive_grid,
        lc_future_mandatory_anticipation=a.lc_future_mandatory_anticipation,
        lc_anchor_invariant_curvature=a.lc_anchor_invariant_curvature,
        lc_post_end_bounded_tail=a.lc_post_end_bounded_tail,
        lc_full_envelope_preference=a.lc_full_envelope_preference,
        lc_c2_reference_window=a.lc_c2_reference_window,
        lc_world_candidate=a.lc_world_candidate,
        lc_exact_frenet_endpoint=a.lc_exact_frenet_endpoint,
        lc_candidate_reason_audit=a.lc_candidate_reason_audit,
        lc_fresh_commit_input=a.lc_fresh_commit_input,
        cruise_reset_on_lead_change=a.cruise_reset_on_lead_change,
        cruise_precise_future_admission=a.cruise_precise_future_admission,
        lc_skip_unrequested_avoidance_search=a.lc_skip_unrequested_avoidance_search,
        lc_path_hazard_preferred_request=a.lc_path_hazard_preferred_request,
        od_gt_velocity_observation=a.od_gt_velocity_observation,
        lc_physical_endpoint_consistency=a.lc_physical_endpoint_consistency,
        bpp_final_ego_poll=a.bpp_final_ego_poll,
        lc_reachable_admission=a.lc_reachable_admission,
        lc_rss_positive_grid=a.lc_rss_positive_grid,
        lc_certified_target_only_speed_reference=a.lc_certified_target_only_speed_reference,
        lc_defer_unhelpful_preferred_request=a.lc_defer_unhelpful_preferred_request,
        baseline='../obstaclecruise_gt_100',
        policy='same runtime for three replicates; failures retained; simulator reset between runs',
        mission='collision-free efficient roundabout merge, highway passing/following, tollgate endpoint'),
        indent=2, ensure_ascii=False))
    scripts = Path(__file__).resolve().parent
    result = subprocess.run([sys.executable, str(scripts/'run_obstaclecruise_batch.py'),
        '--first', '1', '--last', '3', '--output', str(folder), '--detector', 'od_gt',
        '--speed-limit-kph', '120', '--overtaking', '--collect-all', '--freeze-runtime'] +
        (['--lc-speed-refresh'] if a.lc_speed_refresh else []) +
        (['--lc-diverse-search'] if a.lc_diverse_search else []) +
        (['--lc-strict-maneuver-safety'] if a.lc_strict_maneuver_safety else []) +
        (['--lc-measured-motion-diagnostic-only'] if a.lc_measured_motion_diagnostic_only else []) +
        (['--po-c2-bridge'] if a.po_c2_bridge else []) +
        (['--interpolate-rising-velocity'] if a.interpolate_rising_velocity else []) +
        (['--lc-partial-acceleration-candidates'] if a.lc_partial_acceleration_candidates else []) +
        (['--lc-continuous-bounded-output'] if a.lc_continuous_bounded_output else []) +
        (['--lc-defer-pre-adjacency-normal'] if a.lc_defer_pre_adjacency_normal else []) +
        (['--lc-adaptive-positive-grid'] if a.lc_adaptive_positive_grid else []) +
        (['--lc-future-mandatory-anticipation'] if a.lc_future_mandatory_anticipation else []) +
        (['--lc-anchor-invariant-curvature'] if a.lc_anchor_invariant_curvature else []) +
        (['--lc-post-end-bounded-tail'] if a.lc_post_end_bounded_tail else []) +
        (['--lc-full-envelope-preference'] if a.lc_full_envelope_preference else []) +
        (['--lc-c2-reference-window'] if a.lc_c2_reference_window else []) +
        (['--lc-world-candidate'] if a.lc_world_candidate else []) +
        (['--lc-exact-frenet-endpoint'] if a.lc_exact_frenet_endpoint else []) +
        (['--lc-candidate-reason-audit'] if a.lc_candidate_reason_audit else []) +
        (['--lc-fresh-commit-input'] if a.lc_fresh_commit_input else []) +
        (['--cruise-reset-on-lead-change'] if a.cruise_reset_on_lead_change else []) +
        (['--cruise-precise-future-admission'] if a.cruise_precise_future_admission else []) +
        (['--lc-skip-unrequested-avoidance-search'] if a.lc_skip_unrequested_avoidance_search else []) +
        (['--lc-path-hazard-preferred-request'] if a.lc_path_hazard_preferred_request else []) +
        (['--od-gt-velocity-observation'] if a.od_gt_velocity_observation else []) +
        (['--lc-physical-endpoint-consistency'] if a.lc_physical_endpoint_consistency else []) +
        (['--bpp-final-ego-poll'] if a.bpp_final_ego_poll else []) +
        (['--lc-reachable-admission'] if a.lc_reachable_admission else []) +
        (['--lc-rss-positive-grid'] if a.lc_rss_positive_grid else []) +
        (['--lc-certified-target-only-speed-reference'] if a.lc_certified_target_only_speed_reference else []) +
        (['--lc-defer-unhelpful-preferred-request'] if a.lc_defer_unhelpful_preferred_request else []))
    subprocess.run([sys.executable, str(scripts/'report_obstaclecruise_round.py'), str(folder)],
                   check=True)
    raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
