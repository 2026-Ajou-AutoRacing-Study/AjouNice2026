"""Inspect indexed bag metadata, not requested topic names or replayed messages."""
from .models import selected_model
from .topic_contract import model_topics, MEASUREMENT, PLANNER_TIME_PATTERNS
from .recording_options import mode_label, BASIC, ANALYSIS, LATENCY
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


def profile_contract_warnings(cfg):
    """Report stale frozen lists without rewriting an execution's inputs."""
    if not cfg['recording'].get('bag_enabled', True) or mode_label(cfg['recording']) not in (BASIC, ANALYSIS, LATENCY):
        return []
    expected = [MEASUREMENT]
    if cfg['stack']['detector'] == 'bevfusion':
        contract = model_topics(selected_model(cfg['stack']))
        expected += [contract['status'], contract['latency_ms']]
    missing = [t for t in expected if t not in cfg['recording']['topics']]
    return ['녹화 목록에 모델 진단 토픽 누락 (이전 실행 사본일 수 있음): ' + ', '.join(missing)] if missing else []


def audit_bag(path, cfg, *, driving_started=False):
    import rosbag
    with rosbag.Bag(str(path), 'r') as bag:
        info = bag.get_type_and_topic_info().topics
        counts = {topic: int(row.message_count) for topic, row in info.items()}
    requested = cfg['recording']['topics']
    strict = mode_label(cfg['recording']) in (BASIC, ANALYSIS, LATENCY)
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
    required = list(dict.fromkeys(required))
    missing = [topic for topic in required if counts.get(topic, 0) == 0]
    rows = {topic: {'message_count': counts.get(topic, 0),
                    'required_for_drive': topic in required,
                    'recorded': counts.get(topic, 0) > 0} for topic in dict.fromkeys(requested + required + list(counts))}
    planner_counts = {topic: count for topic, count in counts.items()
                      if any(re.fullmatch(pattern, topic) for pattern in PLANNER_TIME_PATTERNS)}
    planner_requested = bool(cfg['recording'].get('topic_regex')) or mode_label(cfg['recording']) == LATENCY
    planner_complete = any(planner_counts.values())
    warnings = ['필수 녹화 토픽 누락: ' + ', '.join(missing)] if missing else []
    if planner_requested and not planner_complete:
        warnings.append('planner processing-time 메시지 0개 (미발행 또는 녹화 누락)')
    return {'schema_version': 2, 'bag': str(path), 'driving_started': driving_started,
            'status': 'WARN' if warnings else 'OK', 'warnings': warnings,
            'topics': rows, 'missing_required': missing,
            'not_requested_required': [t for t in required if t not in requested],
            'missing_requested': [t for t in requested if not counts.get(t, 0)],
            'complete_for_drive': driving_started and not missing,
            'complete_for_latency': bool(model_required) and all(counts.get(t, 0) > 0 for t in model_required)
                                    and (not planner_requested or planner_complete),
            'model_telemetry_topics': model_required,
            'planner_processing_time': {'requested': planner_requested, 'topics': planner_counts,
                                        'message_count': sum(planner_counts.values()),
                                        'recorded': planner_complete},
            're_inference_possible': all(counts.get(t, 0) > 0 for t in SENSORS),
            'clock_recorded': counts.get('/clock', 0) > 0,
            'note': '0 messages means not recorded. Optional/debug topics may be absent. No sensors means no offline re-inference.'}
