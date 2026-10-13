"""Inspect indexed bag metadata, not requested topic names or replayed messages."""
from .models import selected_model
SENSORS = ['/sensor/lidar1'] + ['/sensor/camera/{}/compressed'.format(name)
    for name in ('front', 'front_left', 'front_right', 'back')]
CORE = ['/Ego_topic', '/Competition_topic', '/Object_topic', '/CollisionData',
        '/localization/kinematic_state', '/localization/control_kinematic_state', '/tf', '/tf_static',
        '/perception/object_recognition/detection/objects',
        '/perception/object_recognition/tracking/measurement_objects',
        '/perception/object_recognition/tracking/objects', '/perception/object_recognition/objects',
        '/planning/mission_planning/route',
        '/planning/scenario_planning/lane_driving/behavior_planning/path', '/planning/trajectory', '/ctrl_cmd']


def audit_bag(path, cfg, *, driving_started=False):
    import rosbag
    with rosbag.Bag(str(path), 'r') as bag:
        info = bag.get_type_and_topic_info().topics
        counts = {topic: int(row.message_count) for topic, row in info.items()}
    required = list(CORE)
    if cfg['stack']['pose_source'] != 'localization':
        required.remove('/localization/control_kinematic_state')
    if cfg['stack']['detector'] == 'bevfusion':
        base = '/perception/bevfusion' if selected_model(cfg['stack']) == 'e25' else '/perception/base5_trial'
        required += [base + '/status', base + '/latency_ms']
    if cfg['stack'].get('traffic_light', False):
        required.append('/perception/traffic_light_recognition/traffic_signals')
    if cfg['recording'].get('include_sensors', False):
        required += SENSORS
    requested = cfg['recording']['topics']
    missing = [topic for topic in required if counts.get(topic, 0) == 0]
    rows = {topic: {'message_count': counts.get(topic, 0),
                    'required_for_drive': topic in required,
                    'recorded': counts.get(topic, 0) > 0} for topic in dict.fromkeys(requested + required + list(counts))}
    return {'schema_version': 1, 'bag': str(path), 'driving_started': driving_started,
            'topics': rows, 'missing_required': missing,
            'complete_for_drive': driving_started and not missing,
            're_inference_possible': all(counts.get(t, 0) > 0 for t in SENSORS),
            'clock_recorded': counts.get('/clock', 0) > 0,
            'note': '0 messages means not recorded. Optional/debug topics may be absent. No sensors means no offline re-inference.'}
