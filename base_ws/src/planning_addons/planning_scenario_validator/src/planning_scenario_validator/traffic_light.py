"""Traffic runtime controls and compatibility fields for the yellow policy."""
import importlib.util
import math
import os
from pathlib import Path

DEFAULT_PYTHON = "~/.local/share/ajounice/traffic_light_gpu/bin/python"
MODE_LABELS = {"system": "시스템 Python", "virtualenv": "가상환경 Python"}
LABEL_MODES = {label: mode for mode, label in MODE_LABELS.items()}
DEFAULT_PROFILE = 'v5_candidate'
PROFILE_LABELS = {'v5_candidate': 'v5 지도+CNN 후보', 'legacy': '기존 YOLO+HSV'}
LABEL_PROFILES = {label: profile for profile, label in PROFILE_LABELS.items()}
DEFAULT_MODELS_ROOT = '~/.local/share/ajounice/traffic_light_models/v5_3d29ee7f'
YELLOW_POLICY_IMPLEMENTATION = 'lanelet_braking_v1'
YELLOW_BRAKING_DEFAULTS = {'signal_compliance_yellow_deceleration_mps2': 8.0}
TIMED_POLICY_DEFAULTS = {
    'signal_compliance_required_green_observations': 1,
    'signal_compliance_min_yellow_sec': 2.0,
    'signal_compliance_yellow_time_margin_sec': 0.2,
    'signal_compliance_enable_timed_yellow': False,
}


def timed_policy_options(stack):
    """Read old fields for compatibility/provenance, not to enable timed yellow."""
    result = {key: stack.get(key, default) for key, default in TIMED_POLICY_DEFAULTS.items()}
    required = result['signal_compliance_required_green_observations']
    if type(required) is not int or not 1 <= required <= 3:
        raise ValueError('stack.signal_compliance_required_green_observations는 1~3 정수입니다')
    enabled = result['signal_compliance_enable_timed_yellow']
    if type(enabled) is not bool:
        raise ValueError('stack.signal_compliance_enable_timed_yellow는 true/false입니다')
    for key in ('signal_compliance_min_yellow_sec', 'signal_compliance_yellow_time_margin_sec'):
        value = result[key]
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError('stack.{}는 유한한 숫자여야 합니다'.format(key))
        result[key] = float(value)
    # These values no longer participate in a decision. In particular, an old
    # minimum/margin combination must not reject an otherwise runnable campaign.
    return result


def yellow_braking_options(stack):
    """Yellow-only v*response_delay + v^2/(2*a); not the controller's limit."""
    key = 'signal_compliance_yellow_deceleration_mps2'
    value = stack.get(key, YELLOW_BRAKING_DEFAULTS[key])
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError('stack.{}는 유한한 양수여야 합니다'.format(key))
    return {key: float(value)}


def launch_options(stack, *, legacy_snapshot=False):
    """Resolve opt-in controls without migrating an immutable old execution.

    Before these switches existed, yellow passthrough and debug image publishing
    were enabled but no debug viewer was owned by Validator. Only old snapshots
    use that compatibility default; new authoring documents default both OFF.
    """
    for key in ('traffic_light_yellow_passthrough', 'traffic_light_debug_view'):
        if key in stack and type(stack[key]) is not bool:
            raise ValueError('stack.{}는 true/false여야 합니다'.format(key))
    result = dict(yellow_passthrough=stack.get('traffic_light_yellow_passthrough', legacy_snapshot),
                  debug_view=stack.get('traffic_light_debug_view', False),
                  publish_debug_image=stack.get('traffic_light_debug_view', legacy_snapshot))
    policy = timed_policy_options(stack)
    braking = yellow_braking_options(stack)
    # Preserve the effective old snapshot shape/hash when no new field exists.
    # New execution preparation freezes all four fields explicitly.
    if any(key in stack for key in TIMED_POLICY_DEFAULTS):
        result.update(policy)
    if any(key in stack for key in YELLOW_BRAKING_DEFAULTS):
        result.update(braking)
    return result


def profile_name(stack):
    profile = stack.get('traffic_light_profile', DEFAULT_PROFILE)
    if not isinstance(profile, str) or profile not in PROFILE_LABELS:
        raise ValueError('stack.traffic_light_profile은 legacy 또는 v5_candidate입니다')
    return profile


def candidate_assets(stack, root, *, check_exists=False):
    if profile_name(stack) != 'v5_candidate':
        return None
    value = stack.get('traffic_light_models_root', DEFAULT_MODELS_ROOT)
    if not isinstance(value, str) or not value.strip():
        raise ValueError('신호등 후보 모델 경로가 필요합니다')
    directory = Path(os.path.expandvars(value)).expanduser()
    directory = directory if directory.is_absolute() else Path(root) / directory
    script = Path(root) / 'scripts/runtime/traffic_light_candidate.py'
    try:
        spec = importlib.util.spec_from_file_location('validator_traffic_candidate_assets', script)
        assets = importlib.util.module_from_spec(spec); spec.loader.exec_module(assets)
        release = assets.load_release(Path(root) / 'configs/perception/traffic_light_v5_candidate_release.json')
        models = (assets.validate_install(directory, release) if check_exists
                  else assets.paths(directory, release))
    except (OSError, ValueError) as error:
        raise ValueError('신호등 v5 후보 준비 실패: {}. install_traffic_light_candidate.sh를 실행하세요'.format(error)) from error
    return dict(root=str(directory), models={key: str(path) for key, path in models.items()}, release=release)


def profile_check_arguments(stack, root):
    selected = candidate_assets(stack, root, check_exists=True)
    if selected is None:
        return []
    return ['--weights', selected['models']['yolo'], '--cnn-checkpoint', selected['models']['cnn'],
            '--imgsz', '960', '--expected-weights-sha256', selected['release']['models']['yolo']['sha256'],
            '--expected-cnn-sha256', selected['release']['models']['cnn']['sha256'], '--require-team-versions']


def runtime_mode(stack):
    # Old YAMLs with a custom Conda interpreter continue to work unchanged.
    mode = stack.get("traffic_light_runtime")
    if profile_name(stack) == 'v5_candidate' and mode in (None, 'system'):
        # Old authoring YAMLs may explicitly contain system Python. The new
        # default needs the managed v5 environment; no user-site pip mutation.
        mode = 'virtualenv'
    if mode is None:
        mode = "virtualenv" if stack.get("traffic_light_python", "/usr/bin/python3") != "/usr/bin/python3" else "system"
    if not isinstance(mode, str) or mode not in MODE_LABELS:
        raise ValueError("stack.traffic_light_runtime은 system 또는 virtualenv입니다")
    return mode


def runtime_settings(stack, root, *, check_exists=False):
    profile_name(stack)
    mode = runtime_mode(stack)
    device = str(stack.get("traffic_light_device", "0")).strip()
    if device != "cpu" and not (device.isascii() and device.isdecimal()):
        raise ValueError("신호등 장치는 cpu 또는 GPU 번호(예: 0)여야 합니다")
    value = "/usr/bin/python3" if mode == "system" else stack.get("traffic_light_python", DEFAULT_PYTHON)
    if profile_name(stack) == 'v5_candidate' and (
            stack.get('traffic_light_runtime') == 'system' or value == '/usr/bin/python3'):
        value = DEFAULT_PYTHON
    if not isinstance(value, str) or not value.strip():
        raise ValueError("신호등 가상환경 Python 실행 파일 경로가 필요합니다")
    python = Path(os.path.expandvars(value)).expanduser()
    python = python if python.is_absolute() else Path(root) / python
    # Do not resolve symlinks: a venv's bin/python symlink identifies its prefix.
    python = Path(os.path.abspath(python))
    if check_exists and (not python.is_file() or not os.access(python, os.X_OK)):
        raise ValueError("신호등 Python이 없거나 실행할 수 없습니다: {}. 가이드의 환경 설치부터 진행하세요".format(python))
    if check_exists:
        candidate_assets(stack, root, check_exists=True)
    return str(python), device
