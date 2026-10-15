"""Explicit localization candidates and reproducible source/install contracts.

Candidate settings are launch arguments, not an arbitrary roslaunch/rosparam
escape hatch. The live MR sensor, heading and time contracts remain pinned.
"""
import hashlib
import json
import math
from pathlib import Path

import yaml


CONFIG_DEFAULTS = {
    'localization_manager_config': 'configs/localization/localization_manager.yaml',
    'measurement_noise_config': 'configs/localization/gnss_imu_gt_noise_sensor6_ideal.yaml',
    'local_ekf_base_config': 'configs/localization/robot_localization_gnss_imu_local.yaml',
    'global_ekf_base_config': 'configs/localization/robot_localization_gnss_imu_global.yaml',
    'local_ekf_gt_config': 'configs/localization/robot_localization_gnss_imu_gt_ctra_local.yaml',
    'global_ekf_gt_config': 'configs/localization/robot_localization_gnss_imu_gt_ctra_global.yaml',
}
MR_PINNED_ARGS = {
    'gt_source': 'competition', 'use_heading_correction': True,
    'heading_yaw_offset_deg': -1.347, 'publish_sensor_tf': False,
    'publish_control_state': True, 'use_sim_time': False,
}
BOOL_ARGS = {'use_quantized_gnss_control_state', 'enable_relocation_guard',
             'relocation_guard_monitor_only', 'use_lateral_velocity_constraint',
             'control_state_use_vehicle_pivot'}
NONNEGATIVE_ARGS = {'control_translation_tau_sec', 'control_yaw_tau_sec',
                    'quantized_control_yaw_tau_sec',
                    'control_state_reset_translation_error_m',
                    'control_state_reset_yaw_error_deg', 'relocation_guard_cooldown_sec'}
NONNEGATIVE_ARGS |= {'control_lateral_tau_sec', 'control_max_lateral_correction_rate_mps'}
NONNEGATIVE_ARGS |= {'gnss_motion_compensation_sec',
                    'relocation_guard_stationary_speed_max_mps',
                    'relocation_guard_future_tolerance_sec',
                    'relocation_guard_reacquisition_grace_sec'}
POSITIVE_ARGS = {'relocation_guard_threshold_m', 'lateral_velocity_stddev_mps',
                'gnss_motion_max_age_sec', 'relocation_guard_max_sample_gap_sec',
                'relocation_guard_stationary_hold_sec', 'relocation_guard_max_motion_age_sec',
                'relocation_guard_gnss_stationary_hold_sec',
                'relocation_guard_gnss_stationary_max_span_m',
                'relocation_guard_max_input_age_sec',
                'relocation_guard_reset_ack_timeout_sec',
                'relocation_guard_reset_ack_position_tolerance_m',
                'relocation_guard_reset_ack_yaw_tolerance_deg'}
POSITIVE_INTEGER_ARGS = {'relocation_guard_consecutive_samples',
                         'relocation_guard_reset_ack_consecutive_samples'}
ENUM_ARGS = {'gnss_quantization_input_mode': {'off', 'cell_transition'},
             # The lower node also has a manual-pose mode; deliberately do not
             # expose it in unattended validator runs (it can disagree with Ego).
             'initial_position_source': {'ego', 'official_start'}}
LOCALIZATION_DEFAULTS = {
    'gnss_motion_compensation_sec': 0.0, 'gnss_motion_max_age_sec': 0.1,
    'use_lateral_velocity_constraint': True, 'lateral_velocity_stddev_mps': 0.20,
    'control_state_use_vehicle_pivot': True, 'control_lateral_tau_sec': 0.1,
    'control_max_lateral_correction_rate_mps': 0.3,
    'use_quantized_gnss_control_state': False, 'control_translation_tau_sec': 0.5,
    'control_yaw_tau_sec': 0.5, 'quantized_control_yaw_tau_sec': 0.0,
    'control_state_reset_translation_error_m': 0.5,
    'control_state_reset_yaw_error_deg': 20.0, 'initial_position_source': 'ego',
    'gnss_quantization_input_mode': 'off', 'enable_relocation_guard': False,
    'relocation_guard_monitor_only': True, 'relocation_guard_threshold_m': 5.0,
    'relocation_guard_consecutive_samples': 3, 'relocation_guard_cooldown_sec': 5.0,
    'relocation_guard_max_sample_gap_sec': 0.5,
    'relocation_guard_stationary_speed_max_mps': 0.05,
    'relocation_guard_stationary_hold_sec': 1.0,
    'relocation_guard_gnss_stationary_hold_sec': 1.0,
    'relocation_guard_gnss_stationary_max_span_m': 0.3,
    'relocation_guard_max_motion_age_sec': 0.1,
    'relocation_guard_max_input_age_sec': 0.5,
    'relocation_guard_future_tolerance_sec': 0.05,
    'relocation_guard_reacquisition_grace_sec': 1.0,
    'relocation_guard_reset_ack_timeout_sec': 2.0,
    'relocation_guard_reset_ack_position_tolerance_m': 0.2,
    'relocation_guard_reset_ack_yaw_tolerance_deg': 5.0,
    'relocation_guard_reset_ack_consecutive_samples': 2,
}
SAFETY_DEFAULTS = {
    'startup_timeout_sec': 20.0, 'stop_timeout_sec': 10.0,
    'freshness_sec': 1.0, 'stationary_hold_sec': 1.0, 'speed_max_mps': 0.05,
    'require_valid_gps': True, 'park_after_run': True, 'stop_on_collision': True,
}


def _mapping(stack, key, required_pose):
    value = stack.get(key, {})
    if not isinstance(value, dict):
        raise ValueError('stack.{}는 YAML mapping이어야 합니다'.format(key))
    if any(not isinstance(name, str) for name in value):
        raise ValueError('stack.{}의 설정 키는 문자열이어야 합니다'.format(key))
    if value and stack.get('pose_source') != required_pose:
        raise ValueError('stack.{}는 pose_source: {}에서만 사용합니다'.format(key, required_pose))
    return value


def _number(value, key, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('{}는 YAML 숫자여야 합니다'.format(key))
    try:
        number = float(value)
    except OverflowError as error:
        raise ValueError('{}는 유한한 숫자여야 합니다'.format(key)) from error
    if not math.isfinite(number) or number < 0 or (positive and number == 0):
        raise ValueError('{}는 유한한 {}여야 합니다'.format(key, '양수' if positive else '0 이상 숫자'))
    return number


def _config_path(value, resolve_path):
    if not isinstance(value, str) or not value.strip():
        raise ValueError('localization config 경로는 비어 있지 않은 문자열이어야 합니다')
    path = Path(resolve_path(value))
    if path.suffix.lower() not in ('.yaml', '.yml') or not path.is_file():
        raise ValueError('localization config YAML 파일이 없습니다: {}'.format(path))
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError('localization config YAML을 읽을 수 없습니다: {}'.format(path)) from error
    if not isinstance(data, dict):
        raise ValueError('localization config YAML 최상위는 mapping이어야 합니다: {}'.format(path))
    return str(path.resolve())


def normalize_localization_launch_args(stack, resolve_path=None, *, check_paths=True):
    """Return explicit candidate overrides; omitted settings retain production."""
    value = _mapping(stack, 'localization_args', 'localization')
    allowed = set(CONFIG_DEFAULTS) | BOOL_ARGS | NONNEGATIVE_ARGS | POSITIVE_ARGS | set(ENUM_ARGS) | POSITIVE_INTEGER_ARGS
    unknown = set(value) - allowed
    if unknown:
        raise ValueError('지원하지 않거나 MR에서 고정된 localization_args: {}'.format(', '.join(sorted(unknown))))
    result = {}
    for key, raw in value.items():
        label = 'stack.localization_args.' + key
        if key in CONFIG_DEFAULTS:
            if not isinstance(raw, str) or not raw.strip():
                raise ValueError('{}는 config YAML 경로여야 합니다'.format(label))
            result[key] = _config_path(raw, resolve_path) if check_paths else raw
        elif key in BOOL_ARGS:
            if type(raw) is not bool:
                raise ValueError('{}는 true/false여야 합니다'.format(label))
            result[key] = raw
        elif key in ENUM_ARGS:
            if not isinstance(raw, str) or raw not in ENUM_ARGS[key]:
                raise ValueError('{} 허용값: {}'.format(label, ', '.join(sorted(ENUM_ARGS[key]))))
            result[key] = raw
        elif key in POSITIVE_INTEGER_ARGS:
            if type(raw) is not int or raw < 1:
                raise ValueError('{}는 1 이상 정수여야 합니다'.format(label))
            result[key] = raw
        else:
            result[key] = _number(raw, label, positive=key in POSITIVE_ARGS)
            if key in ('control_state_reset_yaw_error_deg',
                       'relocation_guard_reset_ack_yaw_tolerance_deg') and result[key] > 180:
                raise ValueError('{}는 180도 이하여야 합니다'.format(label))
            if key == 'gnss_motion_compensation_sec' and result[key] > 0.1:
                raise ValueError('{}는 0.1초 이하여야 합니다'.format(label))
    return result


def normalize_gt_pose_launch_args(stack, resolve_path=None, *, check_paths=True):
    """Opt-in GT correction; absence preserves the existing uncorrected alias."""
    value = _mapping(stack, 'gt_pose_args', 'maptf_morai')
    if set(value) - {'use_heading_correction', 'heading_yaw_offset_deg', 'localization_manager_config'}:
        raise ValueError('gt_pose_args에는 heading correction과 localization_manager_config만 허용합니다')
    result = {}
    if 'use_heading_correction' in value:
        if type(value['use_heading_correction']) is not bool:
            raise ValueError('gt_pose_args.use_heading_correction은 true/false여야 합니다')
        result['use_heading_correction'] = value['use_heading_correction']
    if 'heading_yaw_offset_deg' in value:
        raw = value['heading_yaw_offset_deg']
        try:
            offset = float(raw) if not isinstance(raw, bool) and isinstance(raw, (int, float)) else float('nan')
        except OverflowError:
            offset = float('nan')
        if not math.isfinite(offset):
            raise ValueError('gt_pose_args.heading_yaw_offset_deg는 유한한 숫자여야 합니다')
        result['heading_yaw_offset_deg'] = offset
    if 'localization_manager_config' in value:
        raw = value['localization_manager_config']
        if not isinstance(raw, str) or not raw.strip():
            raise ValueError('gt_pose_args.localization_manager_config는 config YAML 경로여야 합니다')
        result['localization_manager_config'] = _config_path(raw, resolve_path) if check_paths else raw
    return result


def normalize_localization_safety(document):
    """Opt-in stationary/freshness guard; unrelated campaigns stay unchanged."""
    validation = document.get('validation', {})
    if not isinstance(validation, dict):
        raise ValueError('validation은 YAML mapping이어야 합니다')
    value = validation.get('localization_safety', {})
    if not isinstance(value, dict):
        raise ValueError('validation.localization_safety는 YAML mapping이어야 합니다')
    if any(not isinstance(name, str) for name in value):
        raise ValueError('validation.localization_safety의 설정 키는 문자열이어야 합니다')
    unknown = set(value) - (set(SAFETY_DEFAULTS) | {'enabled'})
    if unknown:
        raise ValueError('지원하지 않는 localization_safety 설정: ' + ', '.join(sorted(unknown)))
    enabled = value.get('enabled', False)
    if type(enabled) is not bool:
        raise ValueError('validation.localization_safety.enabled는 true/false여야 합니다')
    result = dict(SAFETY_DEFAULTS) if enabled else {}
    result['enabled'] = enabled
    for key, raw in value.items():
        if key in ('enabled', 'require_valid_gps', 'park_after_run', 'stop_on_collision'):
            if type(raw) is not bool:
                raise ValueError('validation.localization_safety.{}는 true/false여야 합니다'.format(key))
            result[key] = raw
        else:
            result[key] = _number(raw, 'validation.localization_safety.' + key, positive=True)
    return result


def effective_localization_args(cfg, root):
    """All active MR localization arguments, including resolved config files."""
    root = Path(root).resolve()
    result = dict(MR_PINNED_ARGS, **LOCALIZATION_DEFAULTS)
    result.update({key: str(root / relative) for key, relative in CONFIG_DEFAULTS.items()})
    result.update(cfg.get('localization_launch_args', {}))
    # Defense in depth: pinned values win even if a caller skipped normalization.
    result.update(MR_PINNED_ARGS)
    result['sensor_config'] = str(cfg['network']['sensor_json'])
    result['repo_root'] = str(root)
    return result


def effective_gt_pose_args(cfg, root):
    """Direct GT launch arguments with the legacy alias semantics preserved."""
    root = Path(root).resolve()
    return {'ego_topic': '/Ego_topic', 'start_lanelet_map': False,
            'use_heading_correction': False, 'heading_yaw_offset_deg': -1.347,
            'use_sim_time': False, 'repo_root': str(root),
            'localization_manager_config': str(root / CONFIG_DEFAULTS['localization_manager_config']),
            **cfg.get('gt_pose_launch_args', {})}


def _sha(path):
    path = Path(path)
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_localization_contract(cfg, root, *, install_root=None):
    """Read-only provenance for both fusion and GT pose, including od_gt runs.

Config paths are receipts; their *role and bytes*, not the authoring/snapshot
absolute path, identify a candidate. Missing installed artifacts are explicit
None entries rather than being silently treated as verified installations.
"""
    root = Path(root).resolve()
    install_root = Path(install_root or root / 'base_ws/install_aw149').resolve()
    pose_source = cfg['stack']['pose_source']
    args = effective_localization_args(cfg, root) if pose_source == 'localization' else effective_gt_pose_args(cfg, root)
    configs = {key: args[key] for key in CONFIG_DEFAULTS if key in args}
    configs.update({
        'controller_common': str(root / 'base_ws/src/control/trajectory_follower/config/trajectory_follower_autoware.yaml'),
        'controller_pp': str(root / 'base_ws/src/control/trajectory_follower/config/purepursuit_autoware.yaml'),
        'controller_pi': str(root / 'base_ws/src/control/trajectory_follower/config/longi_pi_autoware.yaml'),
        'vehicle': str(root / 'configs/vehicle/ioniq5.yaml'),
        'map_tf': str(root / 'configs/tf/morai_lanelet_static_tf.yaml'),
        'official_start': str(root / 'base_ws/src/competition/competition_rule_monitor/config/kcity_v85_rules.yaml'),
    })
    config_files = {key: {'path': path, 'sha256': _sha(path)} for key, path in configs.items()}
    for key, row in config_files.items():
        if row['sha256'] is None:
            raise ValueError('localization 계약 config 파일 누락 {}: {}'.format(key, row['path']))
    source_files = {}
    installed_files = {}
    for package in ('bringup', 'localization/localization_manager', 'tf_pub'):
        source = root / 'base_ws/src' / package
        package_name = Path(package).name
        for directory, pattern in (('launch', '*.launch'), ('scripts', '*.py')):
            for path in sorted((source / directory).rglob(pattern)):
                relative = str(path.relative_to(root))
                source_files[relative] = _sha(path)
                installed = install_root / ('lib/' + package_name + '/' + path.name if directory == 'scripts'
                                            else 'share/' + package_name + '/launch/' + str(path.relative_to(source / directory)))
                installed_files[str(installed.relative_to(install_root))] = _sha(installed)
    for relative in (
        'base_ws/src/planning_addons/planning_scenario_validator/src/planning_scenario_validator/localization_contract.py',
        'base_ws/src/planning_addons/planning_scenario_validator/src/planning_scenario_validator/runner.py',
        'base_ws/src/planning_addons/planning_scenario_validator/src/planning_scenario_validator/runtime.py',
        'base_ws/src/planning_addons/planning_scenario_validator/launch/parking_hold.launch',
        'base_ws/src/control/trajectory_follower/launch/pp_pi_autoware.launch',
        'base_ws/src/sensor_interface/src/morai_ego_udp_receiver.cpp',
        'base_ws/src/sensor_interface/src/morai_sensor_udp_receiver.cpp',
        'base_ws/src/sensor_interface/src/morai_ctrl_cmd_udp_sender.cpp',
        'base_ws/src/sensor_interface/launch/morai_udp_interface.launch',
        'base_ws/src/tf_pub/src/ego_tf_broadcaster.cpp',
        'base_ws/src/tf_pub/src/ego_tf_broadcaster_deg_correction.cpp',
    ):
        source_files[relative] = _sha(root / relative)
    for directory in ('base_ws/src/localization/localization_manager/src',
                      'base_ws/src/control/trajectory_follower/src',
                      'base_ws/src/control/trajectory_follower/include'):
        for path in sorted((root / directory).rglob('*')):
            if path.is_file() and path.suffix in ('.py', '.cpp', '.hpp', '.h'):
                source_files[str(path.relative_to(root))] = _sha(path)
    controller = root / 'base_ws/src/control/trajectory_follower'
    for directory, pattern in (('launch', '*.launch'), ('config', '*.yaml')):
        for path in sorted((controller / directory).rglob(pattern)):
            relative = 'share/trajectory_follower/' + str(path.relative_to(controller))
            installed_files[relative] = _sha(install_root / relative)
    installed_files['share/sensor_interface/launch/morai_udp_interface.launch'] = _sha(
        install_root / 'share/sensor_interface/launch/morai_udp_interface.launch')
    for name in ('localization_contract', 'runner', 'runtime'):
        relative = 'lib/python3/dist-packages/planning_scenario_validator/' + name + '.py'
        installed_files[relative] = _sha(install_root / relative)
    relative = 'share/planning_scenario_validator/launch/parking_hold.launch'
    installed_files[relative] = _sha(install_root / relative)
    for package, executable in (
        ('sensor_interface', 'morai_ego_udp_receiver'), ('sensor_interface', 'morai_sensor_udp_receiver'),
        ('sensor_interface', 'morai_ctrl_cmd_udp_sender'),
        ('tf_pub', 'ego_tf_broadcaster'), ('tf_pub', 'ego_tf_broadcaster_deg_correction'),
        ('trajectory_follower', 'purepursuit_autoware'), ('trajectory_follower', 'longi_pi_autoware'),
    ):
        relative = 'lib/{}/{}'.format(package, executable)
        installed_files[relative] = _sha(install_root / relative)
    stable_args = {key: value for key, value in args.items() if key not in CONFIG_DEFAULTS and key not in ('sensor_config', 'repo_root')}
    identity = {'pose_source': pose_source, 'launch_args': stable_args,
                'config_sha256': {key: row['sha256'] for key, row in config_files.items()},
                'source_files_sha256': source_files, 'installed_files_sha256': installed_files,
                'sensor_sha256': _sha(cfg['network']['sensor_json']),
                'localization_safety': cfg.get('localization_safety', {'enabled': False})}
    digest = hashlib.sha256(json.dumps(identity, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return {'schema_version': 1, **identity, 'launch_args': args,
            'controller_state_topic': '/localization/control_kinematic_state' if pose_source == 'localization' else '/localization/kinematic_state',
            'config_files': config_files, 'install_root': str(install_root),
            'missing_installed_files': [name for name, digest in installed_files.items() if digest is None],
            'fingerprint': digest}
