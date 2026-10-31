"""Detector telemetry names shared by execution, recording and bag audits."""
DETECTION = '/perception/object_recognition/detection/objects'
MEASUREMENT = '/perception/object_recognition/tracking/measurement_objects'
DOWNSTREAM_TIMING = ['/tracked_object_motion_policy/diagnostics/processing_time_ms',
                     '/map_based_prediction/diagnostics/processing_time_ms']
MODEL_NAMESPACES = {'e25': '/perception/bevfusion',
                    'base5_e12': '/perception/base5_trial',
                    'base5_e24': '/perception/base5_trial',
                    'base5_e9': '/perception/base5_trial'}
PLANNER_TIME_PATTERNS = [
    '^/planning/.*/debug/(.*/)?(processing_time.*|calculation_time)$',
    '^/planning/.*/(processing_time_ms|debug/processing_time_ms_diag)$',
]


def model_topics(preset):
    namespace = MODEL_NAMESPACES[preset]
    return {'detection': DETECTION, 'measurement_objects': MEASUREMENT,
            'status': namespace + '/status', 'latency_ms': namespace + '/latency_ms',
            'frame_timing': namespace + '/frame_timing'}


def all_telemetry_topics():
    return list(dict.fromkeys(topic for preset in MODEL_NAMESPACES
                             for key, topic in model_topics(preset).items()
                             if key in ('status', 'latency_ms')))


def validate_trial_remaps(root, preset):
    """Fail before starting a model if its existing launch disagrees with the contract."""
    from pathlib import Path
    import xml.etree.ElementTree as ET
    path = Path(root) / 'base_ws/src/perception/bevfusion_ros/launch/base5_trial.launch'
    remaps = {node.get('from'): node.get('to') for node in ET.parse(path).iter('remap')}
    source, target = model_topics('e25'), model_topics(preset)
    for key in ('status', 'latency_ms'):
        if remaps.get(source[key]) != target[key]:
            raise ValueError('Base5 launch 녹화 토픽 계약 불일치: {}'.format(key))
