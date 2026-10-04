#!/usr/bin/env python3
"""Foreground official-scenario experiment loop. Stops for review on failed runs.

UDP and sensing must already run. BEVFusion is optional shadow data for GT runs,
and required when --detector bevfusion is selected. Owns only its launches.
Run after sourcing source_aw149.sh. Evidence is excluded from source control.
"""
import argparse
import ast
import hashlib
import json
import math
import signal
import shutil
import subprocess
import sys
import time
from pathlib import Path
import rospy
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from std_msgs.msg import String
from autoware_planning_msgs.msg import Trajectory
from autoware_perception_msgs.msg import DetectedObjects, ObjectClassification


def velocity_observation_count(message):
    """Validate actual simulator vehicle measurements, not just launch params."""
    count = 0
    for obj in message.objects:
        if not any(c.label == ObjectClassification.CAR for c in obj.classification):
            continue
        k = obj.kinematics
        v = k.twist_with_covariance.twist.linear
        if (k.has_twist and k.has_twist_covariance and
                math.isfinite(v.x) and math.isfinite(v.y) and
                math.hypot(v.x, v.y) <= 50.0 and
                math.isfinite(k.twist_with_covariance.covariance[0]) and
                k.twist_with_covariance.covariance[0] > 0):
            count += 1
    return count


def terminate(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cruise-reset-on-lead-change', action='store_true',
                   help='Reset obstacle-specific Cruise state on actual lead UUID change')
    p.add_argument('--cruise-precise-future-admission', action='store_true',
                   help='Require native precise future collision proof before admitting an outside object rejected by rough gates')
    p.add_argument('--lc-path-hazard-preferred-request', action='store_true',
                   help='Reconsider NORMAL preferred request deferral for a slow object entering the actual input path within the existing horizon')
    p.add_argument('--lc-skip-unrequested-avoidance-search', action='store_true',
                   help='Skip idle avoidance LC generation only when its special request is absent')
    p.add_argument('--first', type=int, required=True)
    p.add_argument('--last', type=int, default=100)
    p.add_argument('--output', type=Path, default=Path('planner_logs/analysis/obstaclecruise_50'))
    p.add_argument('--scenario', default='oc_official_start/30')
    p.add_argument('--detector', choices=['bevfusion', 'od_gt'], default='od_gt')
    p.add_argument('--speed-limit-kph', type=float, default=0.)
    p.add_argument('--overtaking', action='store_true',
                   help='Enable the opt-in one-shot safe RTC overtaking experiment')
    p.add_argument('--collect-all', action='store_true',
                   help='Keep stopped simulator failures as samples and reset for the remaining runs')
    p.add_argument('--freeze-runtime', action='store_true',
                   help='Refuse the next run if its executable/config/script manifest changed')
    p.add_argument('--lc-speed-refresh', action='store_true',
                   help='Enable safety-rechecked approved LC speed refresh for this experiment')
    p.add_argument('--lc-diverse-search', action='store_true',
                   help='Prioritize diverse existing LC samples without changing safety limits')
    p.add_argument('--lc-strict-maneuver-safety', action='store_true',
                   help='Require validated active/abort/braking paths; fail trial on no-safe-maneuver')
    p.add_argument('--lc-measured-motion-diagnostic-only', action='store_true',
                   help='Record the constant-current-acceleration future model as a diagnostic; retain planned and current-state checks')
    p.add_argument('--po-c2-bridge', action='store_true',
                   help='Try certified C2 tail candidates, retaining baseline when rejected')
    p.add_argument('--interpolate-rising-velocity', action='store_true',
                   help='Transfer rising spatial speeds without the legacy held-low plateau')
    p.add_argument('--lc-partial-acceleration-candidates', action='store_true',
                   help='Check bounded progress-ranked speed alternatives before a full-stop fallback')
    p.add_argument('--lc-continuous-bounded-output', action='store_true',
                   help='Publish freshly checked bounded speed profiles on every active normal LC cycle')
    p.add_argument('--lc-defer-pre-adjacency-normal', action='store_true',
                   help='Defer a scoped normal merge reservation until legal adjacency, preserving mandatory reserve')
    p.add_argument('--lc-adaptive-positive-grid', action='store_true',
                   help='Add bounded cap-feasible small positive acceleration probes to the unchanged safety gate')
    p.add_argument('--lc-future-mandatory-anticipation', action='store_true',
                   help='Consider a legal adjacent route chain leading to the future mandatory preferred lane')
    p.add_argument('--lc-anchor-invariant-curvature', action='store_true',
                   help='Preserve immutable curvature caps across a collinear ego-anchor insertion')
    p.add_argument('--lc-post-end-bounded-tail', action='store_true',
                   help='Check bounded lane-following output after the physical LC endpoint, before native SUCCESS')
    p.add_argument('--lc-full-envelope-preference', action='store_true',
                   help='Prefer safe candidate geometry with reachable full maneuver speed envelope; safety gates unchanged')
    p.add_argument('--lc-c2-reference-window', action='store_true',
                   help='Scoped endpoint-preserving LC reference interpolation experiment; native safety gates unchanged')
    p.add_argument('--lc-world-candidate', action='store_true',
                   help='Additional world-coordinate NORMAL LC candidate after native generation rejection; default OFF')
    p.add_argument('--lc-exact-frenet-endpoint', action='store_true',
                   help='Include the exact native Frenet terminal sample; default OFF, all safety checks retained')
    p.add_argument('--lc-candidate-reason-audit', action='store_true',
                   help='Log paired chosen/lower-lateral candidate rejection reasons after selection at 1 Hz')
    p.add_argument('--lc-fresh-commit-input', action='store_true',
                   help='Re-poll actual BPP inputs for a coherent LC final-output safety context')
    p.add_argument('--od-gt-velocity-observation', action='store_true',
                   help='Simulator-only GT velocity measurement with existing EKF fusion; not production detector validation')
    p.add_argument('--lc-physical-endpoint-consistency', action='store_true',
                   help='Bind normal LC endpoint metadata to the certified actual zero-shift path point')
    p.add_argument('--bpp-final-ego-poll', action='store_true',
                   help='Refresh latest ego inputs after other BPP polling, without changing time guards')
    p.add_argument('--lc-reachable-admission', action='store_true',
                   help='Require an actual-state reachable safety witness before normal LC commitment')
    p.add_argument('--lc-rss-positive-grid', action='store_true',
                   help='Fill positive acceleration gaps while retaining all RSS checks and the family bound')
    p.add_argument('--lc-certified-target-only-speed-reference', action='store_true',
                   help='Use target road speed only after exact full vehicle exit certification')
    p.add_argument('--lc-defer-unhelpful-preferred-request', action='store_true',
                   help='Defer an unhelpful initial NORMAL preferred-lane request after route/traffic proof')
    a = p.parse_args()
    if a.lc_exact_frenet_endpoint and not (a.lc_strict_maneuver_safety and
            a.lc_continuous_bounded_output and a.lc_partial_acceleration_candidates and
            a.lc_physical_endpoint_consistency):
        p.error('exact Frenet endpoint requires strict, continuous, partial and physical-endpoint checks')
    if not 1 <= a.first <= a.last <= 100:
        p.error('expected 1 <= first <= last <= 100')
    if not 0 <= a.speed_limit_kph <= 140:
        p.error('speed limit must be zero (production) or <= 140 km/h')
    if a.lc_measured_motion_diagnostic_only and not a.lc_strict_maneuver_safety:
        p.error('measured-motion diagnostic mode requires strict planned/current-state checks')
    if a.lc_continuous_bounded_output and not (a.lc_strict_maneuver_safety and a.lc_partial_acceleration_candidates):
        p.error('continuous bounded output requires strict checks and the partial candidate grid')
    if a.lc_defer_pre_adjacency_normal and not a.lc_continuous_bounded_output:
        p.error('pre-adjacency deferral requires continuous bounded output')
    if a.lc_reachable_admission and not a.lc_continuous_bounded_output:
        p.error('reachable admission requires continuous bounded output')
    if a.lc_world_candidate and not (a.lc_strict_maneuver_safety and
            a.lc_continuous_bounded_output and a.lc_partial_acceleration_candidates and
            a.lc_reachable_admission and a.lc_physical_endpoint_consistency):
        p.error('world candidate requires strict, continuous, partial, reachable and physical-endpoint checks')
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
    if a.od_gt_velocity_observation and a.detector != 'od_gt':
        p.error('GT velocity observation requires the explicit od_gt simulator detector')
    if (a.lc_adaptive_positive_grid or a.lc_future_mandatory_anticipation or a.lc_anchor_invariant_curvature or a.lc_post_end_bounded_tail or a.lc_full_envelope_preference or a.lc_physical_endpoint_consistency) and not a.lc_continuous_bounded_output:
        p.error('adaptive grid and future-route anticipation require continuous bounded output')
    scripts = Path(__file__).resolve().parent
    if a.od_gt_velocity_observation:
        source_node=scripts.parent.parent/'base_ws/src/perception_object_pipeline/scripts/morai_gt_detection_node.py'
        installed_node=scripts.parent.parent/'base_ws/install_aw149/lib/perception_object_pipeline/morai_gt_detection_node.py'
        if ast.dump(ast.parse(source_node.read_text())) != ast.dump(ast.parse(installed_node.read_text())):
            raise RuntimeError('source/install detector node mismatch; reconfigure and install before GT velocity trial')
    rospy.init_node('obstaclecruise_batch', anonymous=True)
    state = dict(ego=None, ego_time=0., route={}, trajectory_time=0., trajectory_points=0)
    def ego(m):
        state.update(ego=m, ego_time=time.monotonic())
    def route(m):
        state['route'] = json.loads(m.data)
    def trajectory(m):
        state.update(trajectory_time=m.header.stamp.to_sec(), trajectory_points=len(m.points))
    rospy.Subscriber('/Ego_topic', EgoVehicleStatus, ego, queue_size=1)
    rospy.Subscriber('/competition_route_requester/request_status', String, route, queue_size=1)
    rospy.Subscriber('/planning/trajectory', Trajectory, trajectory, queue_size=1)
    publishers = rospy.get_master().getSystemState()[2][0]
    if any(topic == '/ctrl_cmd' and nodes for topic, nodes in publishers):
        raise RuntimeError('existing control publisher; stop it before starting batch')
    brake = [rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)]
    timer = rospy.Timer(rospy.Duration(.05), lambda _: brake[0].publish(
        CtrlCmd(longlCmdType=1, brake=1.0)) if brake[0] is not None else None)
    owned, logs = [], []
    frozen_manifest = None
    interpolation_param='/planning/interpolate_rising_velocity'
    interpolation_existed=rospy.has_param(interpolation_param)
    previous_interpolation=rospy.get_param(interpolation_param,False)
    partial_param='/planning/lc_partial_acceleration_candidates'
    partial_existed=rospy.has_param(partial_param)
    previous_partial=rospy.get_param(partial_param,False)
    measured_param='/planning/lc_measured_motion_diagnostic_only'
    measured_existed=rospy.has_param(measured_param)
    previous_measured=rospy.get_param(measured_param,False)
    continuous_param='/planning/lc_continuous_bounded_output'
    continuous_existed=rospy.has_param(continuous_param)
    previous_continuous=rospy.get_param(continuous_param,False)
    adjacency_param='/planning/lc_defer_pre_adjacency_normal'
    adjacency_existed=rospy.has_param(adjacency_param)
    previous_adjacency=rospy.get_param(adjacency_param,False)
    adaptive_param='/planning/lc_adaptive_positive_grid'
    adaptive_existed=rospy.has_param(adaptive_param)
    previous_adaptive=rospy.get_param(adaptive_param,False)
    future_param='/planning/lc_future_mandatory_anticipation'
    anchor_param='/planning/lc_anchor_invariant_curvature'
    anchor_existed=rospy.has_param(anchor_param)
    previous_anchor=rospy.get_param(anchor_param,False)
    post_end_param='/planning/lc_post_end_bounded_tail'
    post_end_existed=rospy.has_param(post_end_param)
    previous_post_end=rospy.get_param(post_end_param,False)
    envelope_param='/planning/lc_full_envelope_preference'
    reference_window_param='/planning/lc_c2_reference_window'
    world_candidate_param='/planning/lc_world_candidate'
    exact_frenet_endpoint_param='/planning/lc_exact_frenet_endpoint'
    exact_frenet_endpoint_existed=rospy.has_param(exact_frenet_endpoint_param)
    previous_exact_frenet_endpoint=rospy.get_param(exact_frenet_endpoint_param,False)
    world_candidate_existed=rospy.has_param(world_candidate_param)
    previous_world_candidate=rospy.get_param(world_candidate_param,False)
    reference_window_existed=rospy.has_param(reference_window_param)
    previous_reference_window=rospy.get_param(reference_window_param,False)
    candidate_audit_param='/planning/lc_candidate_reason_audit'
    candidate_audit_existed=rospy.has_param(candidate_audit_param)
    previous_candidate_audit=rospy.get_param(candidate_audit_param,False)
    fresh_commit_param='/planning/lc_fresh_commit_input'
    fresh_commit_existed=rospy.has_param(fresh_commit_param)
    previous_fresh_commit=rospy.get_param(fresh_commit_param,False)
    cruise_identity_param='/planning/cruise_reset_on_lead_change'
    cruise_precise_param='/planning/cruise_precise_future_admission'
    cruise_precise_existed=rospy.has_param(cruise_precise_param)
    previous_cruise_precise=rospy.get_param(cruise_precise_param,False)
    cruise_identity_existed=rospy.has_param(cruise_identity_param)
    previous_cruise_identity=rospy.get_param(cruise_identity_param,False)
    avoidance_search_param='/planning/lc_skip_unrequested_avoidance_search'
    path_hazard_param='/planning/lc_path_hazard_preferred_request'
    path_hazard_existed=rospy.has_param(path_hazard_param)
    previous_path_hazard=rospy.get_param(path_hazard_param,False)
    avoidance_search_existed=rospy.has_param(avoidance_search_param)
    previous_avoidance_search=rospy.get_param(avoidance_search_param,False)
    envelope_existed=rospy.has_param(envelope_param)
    previous_envelope=rospy.get_param(envelope_param,False)
    endpoint_param='/planning/lc_physical_endpoint_consistency'
    endpoint_existed=rospy.has_param(endpoint_param)
    previous_endpoint=rospy.get_param(endpoint_param,False)
    ego_poll_param='/planning/bpp_final_ego_poll'
    ego_poll_existed=rospy.has_param(ego_poll_param)
    previous_ego_poll=rospy.get_param(ego_poll_param,False)
    admission_param='/planning/lc_reachable_admission'
    admission_existed=rospy.has_param(admission_param)
    previous_admission=rospy.get_param(admission_param,False)
    rss_positive_param='/planning/lc_rss_positive_grid'
    rss_positive_existed=rospy.has_param(rss_positive_param)
    previous_rss_positive=rospy.get_param(rss_positive_param,False)
    certified_reference_param='/planning/lc_certified_target_only_speed_reference'
    certified_reference_existed=rospy.has_param(certified_reference_param)
    previous_certified_reference=rospy.get_param(certified_reference_param,False)
    preferred_defer_param='/planning/lc_defer_unhelpful_preferred_request'
    preferred_defer_existed=rospy.has_param(preferred_defer_param)
    previous_preferred_defer=rospy.get_param(preferred_defer_param,False)
    future_existed=rospy.has_param(future_param)
    previous_future=rospy.get_param(future_param,False)
    def launch(label, cmd, out):
        log = (out/(label+'.log')).open('w'); logs.append(log)
        process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
        owned.append(process)
    try:
        for number in range(a.first, a.last+1):
            if shutil.disk_usage(a.output.parent).free < 15 * 1024**3:
                raise RuntimeError('less than 15 GiB free; refuse an incomplete evidence recording')
            out = a.output/('official%03d' % number)
            prep = a.output/('prepare%03d' % number)
            prep.mkdir(parents=True, exist_ok=False)
            if out.exists():
                raise RuntimeError('trial output already exists')
            with (prep/'source_at_start.patch').open('w') as patch:
                subprocess.run(['git','diff'],stdout=patch,check=True)
            # Untracked harness files and installed binaries are not covered by git diff.
            repo = scripts.parent.parent
            evidence_files = list(scripts.glob('*.py')) + list(scripts.glob('*.launch')) + [
                repo/'planning_ws/install_isolated/lib/libautoware_behavior_path_planner_full_source.so',
                repo/'planning_ws/install_isolated/lib/libautoware_behavior_path_planner_common.so',
                repo/'planning_ws/install_isolated/lib/libautoware_behavior_path_lane_change_module.so',
                repo/'planning_ws/install_isolated/lib/libautoware_behavior_path_avoidance_by_lane_change_module.so',
                repo/'planning_ws/install_isolated/lib/libautoware_motion_velocity_obstacle_cruise_module.so',
                repo/'planning_ws/install_isolated/lib/libautoware_rtc_interface.so',
                repo/'planning_ws/install_isolated/lib/libautoware_path_optimizer.so',
                repo/'planning_ws/install_isolated/lib/libautoware_path_smoother.so',
                repo/'planning_ws/install_isolated/lib/libautoware_velocity_smoother.so',
                repo/'planning_ws/install_isolated/lib/libautoware_planning_validator_trajectory_checker_full_source.so',
                repo/'planning_ws/install_isolated/share/autoware_launch/config/ros1/planning/behavior_path_modules.yaml',
                repo/'planning_ws/install_isolated/share/autoware_launch/config/ros1/planning/behavior_velocity_modules.yaml',
                repo/'planning_ws/install_isolated/share/autoware_launch/config/ros1/planning/motion_velocity_modules.yaml',
                repo/'planning_ws/install_isolated/share/autoware_launch/config/ros1/planning/scenario_planning/common/autoware_velocity_smoother/JerkFiltered.param.yaml',
                repo/'planning_ws/install_isolated/share/autoware_launch/config/ros1/planning/scenario_planning/lane_driving/motion_planning/path_smoother/elastic_band_smoother.param.yaml',
                repo/'planning_ws/install_isolated/share/autoware_launch/config/ros1/planning/scenario_planning/lane_driving/motion_planning/autoware_path_optimizer/path_optimizer.param.yaml',
                repo/'configs/planning/longitudinal_acceleration_limits.yaml',
                repo/'base_ws/install_aw149/lib/ekf_multi_object_tracking/ekf_multi_object_tracking',
                repo/'base_ws/src/ekf-multi-object-tracking/config/config.yaml',
                repo/'base_ws/src/ekf-multi-object-tracking/launch/ekf_multi_class_object_tracking.launch',
                repo/'base_ws/src/ekf-multi-object-tracking/src/ekf_multi_object_tracking_node.cpp',
                repo/'base_ws/src/ekf-multi-object-tracking/include/fixed_output_scheduler.hpp',
                repo/'base_ws/src/ekf-multi-object-tracking/test/test_fixed_output_scheduler.cpp',
                repo/'base_ws/src/ekf-multi-object-tracking/CMakeLists.txt']
            # git diff omits new production headers/tests. Preserve them with
            # the binary so a frozen run remains reproducible after later edits.
            new_planning_files = subprocess.check_output([
                'git', 'ls-files', '--others', '--exclude-standard', '-z', '--',
                'planning_ws/src'], cwd=repo).decode().split('\0')
            evidence_files += [repo/name for name in new_planning_files if name]
            evidence_files += [repo/'base_ws/src/perception_object_pipeline'/name for name in (
                'src/perception_object_pipeline/morai_gt_detection.py',
                'scripts/morai_gt_detection_node.py', 'launch/morai_gt_detection.launch',
                'test/test_morai_gt_detection.py')]
            evidence_files += [repo/'base_ws/install_aw149'/name for name in (
                'lib/python3/dist-packages/perception_object_pipeline/morai_gt_detection.py',
                'lib/perception_object_pipeline/morai_gt_detection_node.py',
                'share/perception_object_pipeline/launch/morai_gt_detection.launch')]
            evidence_files = list(dict.fromkeys(evidence_files))
            manifest = {
                str(path.relative_to(repo)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in evidence_files if path.is_file()}
            # Installed binary equality alone does not freeze tracked source.
            # Also reject edits to this campaign's production source/config,
            # while allowing ignored bag analysis and HTML reports to progress.
            tracked_source_patch = subprocess.check_output([
                'git', 'diff', 'HEAD', '--', 'planning_ws/src', 'scripts/scenario',
                'configs/planning', 'base_ws/src/perception_object_pipeline',
                'base_ws/src/planning_addons/ajounice_planning_integration'], cwd=repo)
            manifest['tracked_campaign_source_patch_sha256'] = hashlib.sha256(
                tracked_source_patch).hexdigest()
            manifest['experiment_lc_world_candidate'] = a.lc_world_candidate
            manifest['experiment_lc_exact_frenet_endpoint'] = a.lc_exact_frenet_endpoint
            if a.freeze_runtime and frozen_manifest is not None and manifest != frozen_manifest:
                raise RuntimeError('round runtime changed; refuse mixed-configuration replicate')
            frozen_manifest = manifest
            (prep/'runtime_manifest.json').write_text(json.dumps(manifest, indent=2))
            if number == a.first:
                # Preserve the executable/config baseline once for all replicates.
                snapshot_root = a.output/'runtime_snapshot'
                snapshot_root.mkdir(exist_ok=False)
                for path in evidence_files:
                    if path.is_file():
                        destination = snapshot_root/path.relative_to(repo)
                        destination.parent.mkdir(parents=True, exist_ok=True)
                        shutil.copy2(path, destination)
                untracked = subprocess.check_output(
                    ['git', 'ls-files', '--others', '--exclude-standard', '-z'], cwd=repo
                ).decode().split('\0')
                for name in untracked:
                    if not name or not name.startswith(('planning_ws/src/', 'scripts/scenario/')):
                        continue
                    path = repo/name
                    if path.is_file() and path.suffix in ('.py', '.hpp', '.cpp', '.h', '.yaml', '.launch'):
                        destination = snapshot_root/name
                        destination.parent.mkdir(parents=True, exist_ok=True)
                        shutil.copy2(path, destination)
                (snapshot_root/'commit.txt').write_text(subprocess.check_output(
                    ['git', 'rev-parse', 'HEAD'], cwd=repo).decode())
                shutil.copy2(prep/'source_at_start.patch', snapshot_root/'source.patch')
                tracker=repo/'base_ws/src/ekf-multi-object-tracking'
                (snapshot_root/'tracker_commit.txt').write_text(subprocess.check_output(
                    ['git','rev-parse','HEAD'],cwd=tracker).decode())
                (snapshot_root/'tracker_source.patch').write_text(subprocess.check_output(
                    ['git','diff'],cwd=tracker).decode())
            (prep/'experiment.json').write_text(json.dumps(dict(
                scenario=a.scenario, detector=a.detector, speed_limit_kph=a.speed_limit_kph,
                overtaking=a.overtaking,
                passing_intent=a.overtaking,
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
                note=('Simulator GT position and velocity observations feed the same EKF/predictor; not production perception validation.'
                      if a.od_gt_velocity_observation else
                      'GT replaces detections only; same tracker and predictor. BEV may remain active as shadow.')),indent=2))
            state.update(route={}, trajectory_time=0., trajectory_points=0)
            print('TRIAL', number, '/', a.last, 'RESET', flush=True)
            for kind, name in [('scenario',a.scenario), ('network','ego_network_competition_local')]:
                subprocess.run([sys.executable,str(scripts/'morai_msc.py'),kind,name],check=True,timeout=20)
            until = time.monotonic()+12
            while time.monotonic()<until:
                m=state['ego']
                if m and time.monotonic()-state['ego_time']<.3 and math.hypot(
                        m.position.x+12.815,m.position.y-160.987)<1 and abs(m.velocity.x)<.1:
                    break
                time.sleep(.1)
            else:
                raise RuntimeError('scenario reset pose/speed not verified')
            # Use the exact localization alias defaults, without runner-only overrides.
            launch('localization',['roslaunch','bringup','localization_gnss_imu_gt_ctra.launch'],prep)
            detection_topic='/perception/object_recognition/detection/objects'
            if a.detector == 'od_gt':
                detection_topic='/evaluation/od_gt/detection/objects'
                launch('od_gt',['roslaunch','perception_object_pipeline','morai_gt_detection.launch',
                    'output_objects:='+detection_topic,
                    'include_velocity:='+str(a.od_gt_velocity_observation).lower()],prep)
            launch('tracking',['roslaunch','perception_object_pipeline','object_pipeline.launch',
                'start_detector:=false','prediction_model:=map_based','maximum_tracking_internal_prediction_age_sec:=0.5',
                'detection_objects:='+detection_topic,
                'detection_velocity_fusion_mode:='+('2' if a.od_gt_velocity_observation else '0'),
                'detection_velocity_noise_std_mps:=1.0'],prep)
            planning_command=['roslaunch','ajounice_planning_integration','competition_planning.launch',
                'enable_traffic_light_planner:=false']
            rospy.set_param('/planning/approved_lc_speed_refresh', a.lc_speed_refresh)
            rospy.set_param('/planning/lc_diverse_candidate_search', a.lc_diverse_search)
            rospy.set_param('/planning/lc_strict_maneuver_safety', a.lc_strict_maneuver_safety)
            rospy.set_param(measured_param,a.lc_measured_motion_diagnostic_only)
            rospy.set_param('/planning/mpt_c2_bridge', a.po_c2_bridge)
            rospy.set_param('/planning/interpolate_rising_velocity', a.interpolate_rising_velocity)
            rospy.set_param(partial_param,a.lc_partial_acceleration_candidates)
            rospy.set_param(continuous_param,a.lc_continuous_bounded_output)
            rospy.set_param(adjacency_param,a.lc_defer_pre_adjacency_normal)
            rospy.set_param(adaptive_param,a.lc_adaptive_positive_grid)
            rospy.set_param(future_param,a.lc_future_mandatory_anticipation)
            rospy.set_param(anchor_param,a.lc_anchor_invariant_curvature)
            rospy.set_param(post_end_param,a.lc_post_end_bounded_tail)
            rospy.set_param(envelope_param,a.lc_full_envelope_preference)
            rospy.set_param(reference_window_param,a.lc_c2_reference_window)
            rospy.set_param(world_candidate_param,a.lc_world_candidate)
            rospy.set_param(exact_frenet_endpoint_param,a.lc_exact_frenet_endpoint)
            rospy.set_param(candidate_audit_param,a.lc_candidate_reason_audit)
            rospy.set_param(fresh_commit_param,a.lc_fresh_commit_input)
            rospy.set_param(cruise_identity_param,a.cruise_reset_on_lead_change)
            rospy.set_param(cruise_precise_param,a.cruise_precise_future_admission)
            rospy.set_param(avoidance_search_param,a.lc_skip_unrequested_avoidance_search)
            rospy.set_param(path_hazard_param,a.lc_path_hazard_preferred_request)
            rospy.set_param(endpoint_param,a.lc_physical_endpoint_consistency)
            rospy.set_param(ego_poll_param,a.bpp_final_ego_poll)
            rospy.set_param(admission_param,a.lc_reachable_admission)
            rospy.set_param(rss_positive_param,a.lc_rss_positive_grid)
            rospy.set_param(certified_reference_param,a.lc_certified_target_only_speed_reference)
            rospy.set_param(preferred_defer_param,a.lc_defer_unhelpful_preferred_request)
            rospy.set_param('/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/lane_change/passing_intent/enable_simulator_experiment', a.overtaking)
            for direction in ('left', 'right'):
                rtc_prefix = '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/external_request_lane_change_' + direction
                rospy.set_param(rtc_prefix+'/auto_mode_single_candidate', a.overtaking)
                rospy.set_param(rtc_prefix+'/auto_mode_lease_sec', 1.0)
            if a.speed_limit_kph:
                planning_command=['roslaunch',str(scripts/'planning_speed_limited_trial.launch'),
                    'speed_limit_kph:='+str(a.speed_limit_kph)]
            launch('planning',planning_command,prep)
            until=time.monotonic()+90
            while time.monotonic()<until:
                if any(x.poll() is not None for x in owned):
                    raise RuntimeError('stack process exited during readiness')
                if state['route'].get('phase')=='VERIFIED' and state['trajectory_points']>2 and rospy.Time.now().to_sec()-state['trajectory_time']<.5:
                    break
                time.sleep(.2)
            else:
                raise RuntimeError('route/trajectory not ready')
            if a.overtaking:
                launch('overtaking_policy', [sys.executable,
                    str(scripts/'overtaking_trial_policy.py'), '--enable-simulator-experiment'], prep)
                rospy.wait_for_message('/evaluation/overtaking_policy/status', String, timeout=30)
                if any(x.poll() is not None for x in owned):
                    raise RuntimeError('stack/policy exited during overtaking readiness')
            if a.od_gt_velocity_observation:
                verified=[]
                deadline=time.monotonic()+15
                while len(verified)<3 and time.monotonic()<deadline:
                    message=rospy.wait_for_message(detection_topic,DetectedObjects,timeout=3)
                    count=velocity_observation_count(message)
                    age=(rospy.Time.now()-message.header.stamp).to_sec()
                    if count and -.01<=age<=.5:
                        verified.append(dict(stamp=message.header.stamp.to_sec(),
                                             frame=message.header.frame_id,vehicle_twists=count))
                if len(verified)<3 or rospy.get_param('/configure/detection_velocity_fusion_mode',0)!=2:
                    raise RuntimeError('GT velocity input not verified; keep parked and stop infrastructure trial')
                (prep/'velocity_input_gate.json').write_text(json.dumps(dict(
                    verified=True,mode=2,frames=verified),indent=2))
                print('VERIFIED GT VELOCITY INPUT',verified,flush=True)
            print('TRIAL',number,'/',a.last,'DRIVE',flush=True)
            publisher=brake[0]; brake[0]=None; publisher.unregister(); time.sleep(.3)
            driver=subprocess.Popen([sys.executable,str(scripts/'drive_planning_trial.py'),
                '--output',str(out),'--seconds','150'])
            try:
                driver.wait(timeout=210)
            finally:
                terminate(driver)
                brake[0]=rospy.Publisher('/ctrl_cmd',CtrlCmd,queue_size=1)
            if driver.returncode != 0:
                raise RuntimeError('driver process failed; stop round for infrastructure review')
            for process in reversed(owned): terminate(process)
            owned.clear()
            for log in logs: log.close()
            logs.clear()
            shutil.copyfile(prep/'source_at_start.patch', out/'source.patch')
            shutil.copyfile(prep/'experiment.json', out/'experiment.json')
            shutil.copyfile(prep/'runtime_manifest.json', out/'runtime_manifest.json')
            with (out/'evaluation.log').open('w') as evaluation_log:
                subprocess.run([sys.executable,str(scripts/'evaluate_planning_trial.py'),
                                str(out/'planning.bag')], stdout=evaluation_log, check=True)
            result=json.loads((out/'evaluation.json').read_text())
            print('TRIAL',number,'/',a.last,'EVALUATED',json.dumps({key:result.get(key) for key in (
                'evaluation_waypoint_reached', 'driving_time_to_evaluation_waypoint_sec',
                'max_speed_mps', 'geometric_overlap_ids', 'unresolved_near_collision_ids',
                'invalid_counts', 'warning_counts')}),flush=True)
            if (result['overlap_collision_ids'] or result.get('geometric_overlap_ids') or result['unresolved_near_collision_ids']
                    or not result['evaluation_waypoint_reached']
                    or (out/'termination.txt').read_text().strip() != 'evaluation_waypoint_reached'):
                print('REVIEW REQUIRED; failed sample retained',flush=True)
                if not a.collect_all:
                    break
    finally:
        for process in reversed(owned): terminate(process)
        for log in logs: log.close()
        if brake[0] is not None:
            for _ in range(20):
                brake[0].publish(CtrlCmd(longlCmdType=1,brake=1.0)); time.sleep(.05)
        timer.shutdown()
        if interpolation_existed:
            rospy.set_param(interpolation_param,previous_interpolation)
        elif rospy.has_param(interpolation_param):
            rospy.delete_param(interpolation_param)
        if partial_existed:
            rospy.set_param(partial_param,previous_partial)
        elif rospy.has_param(partial_param):
            rospy.delete_param(partial_param)
        if measured_existed:
            rospy.set_param(measured_param,previous_measured)
        elif rospy.has_param(measured_param):
            rospy.delete_param(measured_param)
        if continuous_existed:
            rospy.set_param(continuous_param,previous_continuous)
        elif rospy.has_param(continuous_param):
            rospy.delete_param(continuous_param)
        if adjacency_existed:
            rospy.set_param(adjacency_param,previous_adjacency)
        elif rospy.has_param(adjacency_param):
            rospy.delete_param(adjacency_param)
        if adaptive_existed:
            rospy.set_param(adaptive_param,previous_adaptive)
        elif rospy.has_param(adaptive_param):
            rospy.delete_param(adaptive_param)
        if future_existed:
            rospy.set_param(future_param,previous_future)
        elif rospy.has_param(future_param):
            rospy.delete_param(future_param)
        if anchor_existed:
            rospy.set_param(anchor_param,previous_anchor)
        elif rospy.has_param(anchor_param):
            rospy.delete_param(anchor_param)
        if post_end_existed:
            rospy.set_param(post_end_param,previous_post_end)
        elif rospy.has_param(post_end_param):
            rospy.delete_param(post_end_param)
        if envelope_existed:
            rospy.set_param(envelope_param,previous_envelope)
        elif rospy.has_param(envelope_param):
            rospy.delete_param(envelope_param)
        if reference_window_existed:
            rospy.set_param(reference_window_param,previous_reference_window)
        elif rospy.has_param(reference_window_param):
            rospy.delete_param(reference_window_param)
        if world_candidate_existed:
            rospy.set_param(world_candidate_param,previous_world_candidate)
        elif rospy.has_param(world_candidate_param):
            rospy.delete_param(world_candidate_param)
        if exact_frenet_endpoint_existed:
            rospy.set_param(exact_frenet_endpoint_param,previous_exact_frenet_endpoint)
        elif rospy.has_param(exact_frenet_endpoint_param):
            rospy.delete_param(exact_frenet_endpoint_param)
        if candidate_audit_existed:
            rospy.set_param(candidate_audit_param,previous_candidate_audit)
        elif rospy.has_param(candidate_audit_param):
            rospy.delete_param(candidate_audit_param)
        if fresh_commit_existed:
            rospy.set_param(fresh_commit_param,previous_fresh_commit)
        elif rospy.has_param(fresh_commit_param):
            rospy.delete_param(fresh_commit_param)
        if cruise_identity_existed:
            rospy.set_param(cruise_identity_param,previous_cruise_identity)
        elif rospy.has_param(cruise_identity_param):
            rospy.delete_param(cruise_identity_param)
        if cruise_precise_existed:
            rospy.set_param(cruise_precise_param,previous_cruise_precise)
        elif rospy.has_param(cruise_precise_param):
            rospy.delete_param(cruise_precise_param)
        if avoidance_search_existed:
            rospy.set_param(avoidance_search_param,previous_avoidance_search)
        elif rospy.has_param(avoidance_search_param):
            rospy.delete_param(avoidance_search_param)
        if path_hazard_existed:
            rospy.set_param(path_hazard_param,previous_path_hazard)
        elif rospy.has_param(path_hazard_param):
            rospy.delete_param(path_hazard_param)
        if endpoint_existed:
            rospy.set_param(endpoint_param,previous_endpoint)
        elif rospy.has_param(endpoint_param):
            rospy.delete_param(endpoint_param)
        if ego_poll_existed:
            rospy.set_param(ego_poll_param,previous_ego_poll)
        elif rospy.has_param(ego_poll_param):
            rospy.delete_param(ego_poll_param)
        if admission_existed:
            rospy.set_param(admission_param,previous_admission)
        elif rospy.has_param(admission_param):
            rospy.delete_param(admission_param)
        if rss_positive_existed:
            rospy.set_param(rss_positive_param,previous_rss_positive)
        elif rospy.has_param(rss_positive_param):
            rospy.delete_param(rss_positive_param)
        if certified_reference_existed:
            rospy.set_param(certified_reference_param,previous_certified_reference)
        elif rospy.has_param(certified_reference_param):
            rospy.delete_param(certified_reference_param)
        if preferred_defer_existed:
            rospy.set_param(preferred_defer_param,previous_preferred_defer)
        elif rospy.has_param(preferred_defer_param):
            rospy.delete_param(preferred_defer_param)


if __name__ == '__main__':
    main()
