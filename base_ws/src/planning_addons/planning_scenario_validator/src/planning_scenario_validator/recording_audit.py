"""Inspect indexed bag metadata, not requested topic names or replayed messages."""
from .models import selected_model
from .topic_contract import model_topics, MEASUREMENT, PLANNER_TIME_PATTERNS, DOWNSTREAM_TIMING
from .recording_options import (mode_label, BASIC, ANALYSIS, LATENCY, LOCALIZATION,
                               diagnostics_enabled, localization_analysis_enabled)
import re
SENSORS = ['/sensor/lidar1'] + ['/sensor/camera/{}/compressed'.format(name)
    for name in ('front', 'front_left', 'front_right', 'back')]
CORE = ['/Ego_topic', '/Competition_topic', '/Object_topic', '/CollisionData',
        '/localization/kinematic_state', '/localization/control_kinematic_state', '/tf', '/tf_static',
        '/perception/object_recognition/detection/objects',
        MEASUREMENT,
        '/perception/object_recognition/tracking/objects', '/perception/object_recognition/objects',
        '/planning/mission_planning/route',
        '/planning/scenario_planning/lane_driving/behavior_planning/path', '/planning/trajectory', '/ctrl_cmd']

# This is a count-based evidence gate, not an outage/heading/time-axis certification.
LOCALIZATION_REPLAY = ['/Ego_topic', '/Competition_topic', '/competition/raw_twist',
                       '/sensor/gps', '/sensor/imu', '/tf_static']
LOCALIZATION_COMMON = LOCALIZATION_REPLAY + [
    '/Object_topic', '/CollisionData', '/vehicle/status/gear', '/vehicle/status/control_mode',
    '/tf', '/localization/kinematic_state', '/localization/acceleration',
    '/perception/object_recognition/detection/objects', MEASUREMENT,
    '/perception/object_recognition/tracking/objects', '/perception/object_recognition/objects',
    '/planning/mission_planning/route', '/planning/trajectory', '/ctrl_cmd',
    '/control/command/steering', '/control/autoware/controller_valid',
    '/control/autoware/failure_reason', '/purepursuit_autoware/debug/lateral_error']
LOCALIZATION_FUSION = ['/localization/imu', '/localization/gt/orientation',
                       '/localization/gt/twist', '/localization/gnss/odom',
                       '/localization/odometry/local', '/localization/odom',
                       '/localization/control_kinematic_state', '/localization/initialpose']


def localization_required_topics(cfg):
    """GT pose comparison has no EKF/normalized-motion/Smooth-state publishers."""
    topics = list(LOCALIZATION_COMMON)
    if cfg['stack']['pose_source'] == 'localization':
        topics += LOCALIZATION_FUSION
    return list(dict.fromkeys(topics))


def profile_contract_warnings(cfg):
    """Report stale frozen lists without rewriting an execution's inputs."""
    if not cfg['recording'].get('bag_enabled', True) or mode_label(cfg['recording']) not in (BASIC, ANALYSIS, LATENCY, LOCALIZATION):
        return []
    expected = [MEASUREMENT]
    if cfg['stack']['detector'] == 'bevfusion':
        contract = model_topics(selected_model(cfg['stack']))
        expected += [contract['status'], contract['latency_ms']]
    if localization_analysis_enabled(cfg['recording']):
        expected += localization_required_topics(cfg)
    missing = [t for t in expected if t not in cfg['recording']['topics']]
    label = '측위 분석' if localization_analysis_enabled(cfg['recording']) else '모델 진단'
    return ['녹화 목록에 {} 토픽 누락 (이전 실행 사본일 수 있음): '.format(label) + ', '.join(dict.fromkeys(missing))] if missing else []


def audit_bag(path, cfg, *, driving_started=False):
    import rosbag
    with rosbag.Bag(str(path), 'r') as bag:
        info = bag.get_type_and_topic_info().topics
        counts = {topic: int(row.message_count) for topic, row in info.items()}
    requested = cfg['recording']['topics']
    localization_requested = localization_analysis_enabled(cfg['recording'])
    strict = mode_label(cfg['recording']) in (BASIC, ANALYSIS, LATENCY, LOCALIZATION)
    required = list(CORE) if strict else [t for t in CORE if t in requested]
    if cfg['stack']['pose_source'] != 'localization':
        if '/localization/control_kinematic_state' in required:
            required.remove('/localization/control_kinematic_state')
    model_required = []
    if cfg['stack']['detector'] == 'bevfusion':
        contract = model_topics(selected_model(cfg['stack']))
        model_required = [contract['status'], contract['latency_ms'], contract['measurement_objects']]
        required += model_required if strict else [t for t in model_required if t in requested]
    if cfg['stack'].get('traffic_light', False) and (strict or '/perception/traffic_light_recognition/traffic_signals' in requested):
        required.append('/perception/traffic_light_recognition/traffic_signals')
    if cfg['recording'].get('include_sensors', False):
        required += SENSORS
    localization_required = localization_required_topics(cfg) if localization_requested else []
    required += localization_required
    required = list(dict.fromkeys(required))
    missing = [topic for topic in required if counts.get(topic, 0) == 0]
    rows = {topic: {'message_count': counts.get(topic, 0),
                    'required_for_drive': topic in required,
                    'required_for_localization_replay': localization_requested and topic in LOCALIZATION_REPLAY,
                    'required_for_localization_assessment': topic in localization_required,
                    'recorded': counts.get(topic, 0) > 0} for topic in dict.fromkeys(requested + required + list(counts))}
    planner_counts = {topic: count for topic, count in counts.items()
                      if any(re.fullmatch(pattern, topic) for pattern in PLANNER_TIME_PATTERNS)}
    planner_requested = diagnostics_enabled(cfg['recording'])
    planner_complete = any(planner_counts.values())
    planner_stages = {stage: sum(count for topic, count in planner_counts.items()
                                if '/' + stage + '/' in topic)
                      for stage in ('behavior_path_planner', 'motion_velocity_planner')}
    missing_planner_stages = [stage for stage, count in planner_stages.items() if not count]
    warnings = ['필수 녹화 토픽 누락: ' + ', '.join(missing)] if missing else []
    if planner_requested and not planner_complete:
        warnings.append('planner processing-time 메시지 0개 (미발행 또는 녹화 누락)')
    elif planner_requested and missing_planner_stages:
        warnings.append('핵심 planner 계산시간 누락: ' + ', '.join(missing_planner_stages))
    analysis_topics = list(DOWNSTREAM_TIMING) if diagnostics_enabled(cfg['recording']) else []
    if analysis_topics and cfg['stack']['detector'] == 'bevfusion':
        analysis_topics.append(contract['frame_timing'])
    missing_analysis = [topic for topic in analysis_topics if counts.get(topic, 0) == 0]
    if missing_analysis:
        warnings.append('분석 진단 메시지 누락: ' + ', '.join(missing_analysis))
    missing_localization_replay = [topic for topic in LOCALIZATION_REPLAY if counts.get(topic, 0) == 0] if localization_requested else []
    missing_localization_assessment = [topic for topic in localization_required if counts.get(topic, 0) == 0]
    localization_replay_complete = localization_requested and not missing_localization_replay
    localization_assessment_complete = localization_requested and driving_started and not missing
    perception_analysis_complete = diagnostics_enabled(cfg['recording']) and driving_started and not missing and not missing_analysis and not missing_planner_stages
    return {'schema_version': 2, 'bag': str(path), 'driving_started': driving_started,
            'status': 'WARN' if warnings else 'OK', 'warnings': warnings,
            'topics': rows, 'missing_required': missing,
            'not_requested_required': [t for t in required if t not in requested],
            'missing_requested': [t for t in requested if not counts.get(t, 0)],
            'complete_for_drive': driving_started and not missing,
            'complete_for_latency': bool(model_required) and all(counts.get(t, 0) > 0 for t in model_required)
                                    and not missing_analysis and (not planner_requested or not missing_planner_stages),
            'complete_for_analysis': perception_analysis_complete or localization_assessment_complete,
            'analysis_kind': 'localization' if localization_requested else 'perception_latency' if diagnostics_enabled(cfg['recording']) else None,
            'complete_for_localization_replay': localization_replay_complete,
            'complete_for_localization_assessment': localization_assessment_complete,
            'localization_recording': {
                'requested': localization_requested, 'pose_source': cfg['stack']['pose_source'],
                'required_for_replay': list(LOCALIZATION_REPLAY) if localization_requested else [],
                'required_for_assessment': localization_required,
                'missing_for_replay': missing_localization_replay,
                'missing_for_assessment': missing_localization_assessment,
                'conditionally_optional': list(LOCALIZATION_FUSION) if cfg['stack']['pose_source'] != 'localization' else [],
                'count_validation_only': True,
                'outage_phase_coverage_verified': False,
                'note': 'Counts > 0 prove recording only, not GPS validity, pre/during/post-outage coverage, heading/TF/time-axis agreement, independent GT quality, or physical standstill. Verify these separately before adoption.'},
            'missing_analysis_topics': missing_analysis,
            'model_telemetry_topics': model_required,
            'planner_processing_time': {'requested': planner_requested, 'topics': planner_counts,
                                        'message_count': sum(planner_counts.values()),
                                        'core_stage_counts': planner_stages, 'missing_core_stages': missing_planner_stages,
                                        'recorded': planner_complete},
            're_inference_possible': all(counts.get(t, 0) > 0 for t in SENSORS),
            'clock_recorded': counts.get('/clock', 0) > 0,
            'note': '0 messages means not recorded. Optional/debug topics may be absent. No sensors means no offline re-inference.'}
