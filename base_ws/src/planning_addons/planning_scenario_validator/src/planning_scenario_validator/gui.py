"""Small Tk frontend for the existing YAML/CLI campaign runner.

No ROS node or decision logic lives here. Operators edit only everyday choices;
goals, networks and advanced contracts stay read-only. Quick tests are separate.
"""
import copy
import getpass
import hashlib
import json
import math
import os
import queue
import re
import signal
import shutil
import subprocess
import threading
from pathlib import Path

import yaml

from .config import (CONFIG_SCHEMA_VERSION, DEFAULT_UPLOAD_REMOTE, EXECUTION_CONTRACT, ROOT,
                     UPLOAD_PROFILES, resolve_path, validate_campaign_document)
from .config import load_campaign, scenario_sources
from .layout import normalize_bar_setting, resolve_bar_height
from .traffic_light import DEFAULT_PYTHON, LABEL_MODES, MODE_LABELS, runtime_mode, runtime_settings
from . import executions
from . import batch
from .authoring import annotated_yaml, pasted_yaml_path, quick_test_document, updated_yaml
from .scenario_selection import choose_scenarios
from .models import PRESETS, LABEL_PRESETS, DEFAULT_MODEL
from . import recording_options as bag_options
from .startup_audit import stream_console


PACKAGE = ROOT / "base_ws/src/planning_addons/planning_scenario_validator"
if not PACKAGE.is_dir():
    PACKAGE = ROOT / "base_ws/install_aw149/share/planning_scenario_validator"
EXAMPLES = PACKAGE / "config/missions/static_obstacle/base"
MISSIONS = PACKAGE / "config/missions"
MISSION_LABELS = {
    'full_course': '전체 코스 주행',
    'highway': '고주로', 'roundabout': '회전교차로',
    'static_obstacle': '정적 장애물', 'dynamic_obstacle': '동적 장애물',
    'gps_shadow': 'GPS 음영 구간',
}
USERS = PACKAGE / "config/users"
NAME_RE = re.compile(r"[a-zA-Z0-9][a-zA-Z0-9_-]{1,59}\Z")
UPLOAD_PROFILE_LABELS = {
    "basic": "기본 공유 — CSV·JSON",
    "bag": "bag 공유 — 기본 + rosbag",
    "video": "영상 공유 — 기본 + MP4",
    "full": "원본 공유 — 기본 + rosbag + MP4",
}
UPLOAD_LABEL_PROFILES = {label: key for key, label in UPLOAD_PROFILE_LABELS.items()}
EDITABLE_FIELDS = {
    'campaign_name': ('campaign_name',), 'glob': ('scenarios', 'glob'),
    'repeat': ('scenarios', 'repeat'), 'pose_source': ('stack', 'pose_source'),
    'detector': ('stack', 'detector'), 'traffic_light': ('stack', 'traffic_light'),
    'bevfusion_model': ('stack', 'bevfusion_model'),
    'selected_topics': ('recording', 'selected_topics'),
    'video': ('recording', 'video', 'enabled'),
    'upload_enabled': ('upload', 'enabled'), 'upload_profile': ('upload', 'profile'),
}


def default_name(template):
    path = Path(template)
    mission = mission_directory(path)
    stem = mission.name + "_starter" if path.stem == "example" and mission else path.stem
    base = re.sub(r"[^a-zA-Z0-9_-]", "_", stem)[:35]
    return base


def campaign_examples():
    # Do not expose scenario .case.yaml files as executable campaigns.
    return sorted(MISSIONS.glob("*/base/*.yaml"))


def available_missions():
    return [key for key in MISSION_LABELS if (MISSIONS / key / 'base/example.yaml').is_file()]


def mission_yaml_choices(mission):
    """Only authoring YAMLs owned by one fixed mission, not arbitrary paths."""
    if mission not in available_missions():
        raise ValueError('제공된 담당 미션 중 하나를 선택하세요')
    root = MISSIONS / mission
    choices = {}
    for folder, prefix in (('workspace', '내 작업'), ('base', '예제')):
        directory = root / folder
        for path in sorted(directory.rglob('*')) if directory.is_dir() else ():
            if (not path.is_file() or path.suffix.lower() not in ('.yaml', '.yml')
                    or path.name.endswith('.case.yaml') or executions.is_snapshot(path)
                    or (path.parent / 'execution.json').exists()):
                continue
            # Do not let a workspace symlink expose another mission/shared base.
            if not is_within(path, directory) or mission_directory(path) != root:
                continue
            if folder == 'workspace' and not is_personal(path):
                continue
            choices['{} / {}'.format(prefix, path.relative_to(directory))] = path
    return choices


def default_mission_yaml(mission):
    choices = mission_yaml_choices(mission)
    for preferred in (MISSIONS / mission / 'workspace' / (mission + '_starter.yaml'),
                      MISSIONS / mission / 'base/example.yaml'):
        if preferred in choices.values():
            return preferred
    if choices:
        return next(iter(choices.values()))
    raise ValueError('이 미션에 실행용 YAML이 없습니다')


def load_document(path):
    path = resolve_path(path)
    if path.name.endswith(".case.yaml"):
        raise ValueError("이 파일은 실행용 YAML이 아닌 예전 장면별 보조 설정입니다.\n미션 base/example.yaml 또는 workspace의 캠페인 YAML을 선택하세요.")
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    validate_campaign_document(value, allow_legacy=True)
    return value


def gui_settings_path():
    base = Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config")
    return base / "ajounice/planning_scenario_validator/gui_settings.json"


def remember_yaml(path):
    """Remember only a successfully opened/saved file, not unsaved form edits."""
    settings = gui_settings_path()
    temporary = settings.with_name(settings.name + ".tmp.{}".format(os.getpid()))
    try:
        settings.parent.mkdir(parents=True, exist_ok=True)
        temporary.write_text(json.dumps({"last_yaml": str(Path(path).resolve())},
                                        ensure_ascii=False) + "\n", encoding="utf-8")
        os.replace(temporary, settings)
    except OSError as error:
        print("GUI 설정 저장 실패 (캠페인에는 영향 없음): {}".format(error), flush=True)
    finally:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass


def remembered_yaml():
    try:
        settings = json.loads(gui_settings_path().read_text(encoding="utf-8"))
        path = resolve_path(settings["last_yaml"])
        form_values(load_document(path))
        return path if path.is_file() else None
    except (OSError, ValueError, TypeError, KeyError, AttributeError, yaml.YAMLError):
        return None


def form_values(document):
    """Expose common choices; preserve every other YAML key without interpretation."""
    scene = document.get("scenarios", {})
    case = document.get("default_case", {})
    goal = case.get("goal", {})
    stack = document.get("stack", {})
    recording = document.get("recording", {})
    network = document.get("network", {})
    morai = document.get("morai", {})
    validation = document.get("validation", {})
    upload = document.get("upload", {})
    result = {
        "campaign_name": document.get("campaign_name", ""),
        "directory": scene.get("directory", ""),
        "glob": scene.get("glob", "*.json"),
        "repeat": scene.get("repeat", 1),
        "timeout_sec": case.get("timeout_sec", 180),
        "pose_source": stack.get("pose_source", "localization"),
        "detector": stack.get("detector", "od_gt"),
        'bevfusion_model': PRESETS[stack.get('bevfusion_model', DEFAULT_MODEL)]['label'],
        'recording_mode': bag_options.mode_label(recording),
        'selected_topics': json.dumps(recording['selected_topics'], ensure_ascii=False) if 'selected_topics' in recording else '',
        "traffic_light": stack.get("traffic_light", False),
        "traffic_light_runtime": MODE_LABELS[runtime_mode(stack)],
        "traffic_light_python": stack.get("traffic_light_python", DEFAULT_PYTHON) if stack.get("traffic_light_python") != "/usr/bin/python3" else DEFAULT_PYTHON,
        "traffic_light_device": str(stack.get("traffic_light_device", "0")),
        "topic_profile": recording.get("topic_profile", ""),
        "include_sensors": recording.get("include_sensors", False),
        "video": recording.get("video", {}).get("enabled", False),
        "video_layout": recording.get("video", {}).get("window_layout", "manual"),
        "bottom_bar_px": recording.get("video", {}).get("bottom_bar_px", "auto"),
        "sensor_json": network.get("sensor_json", ""),
        "ego_network_json": network.get("ego_network_json", ""),
        "scenario_load_host": morai.get("scenario_load_host", "127.0.0.1"),
        "goal_tolerance_m": validation.get("goal_tolerance_m", 2.0),
        "upload_enabled": upload.get("enabled", False),
        "upload_profile": UPLOAD_PROFILE_LABELS.get(upload.get("profile", "basic"), upload.get("profile")),
        "upload_remote": upload.get("rclone_remote") or DEFAULT_UPLOAD_REMOTE,
    }
    for key in ("x", "y", "z", "yaw_deg"):
        result["goal_" + key] = goal.get(key, 0)
    return result


def finite_number(value, label, *, positive=False):
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError("{}은 숫자여야 합니다".format(label)) from error
    if not math.isfinite(number) or (positive and number <= 0):
        raise ValueError("{}은 {}유한한 숫자여야 합니다".format(
            label, "0보다 큰 " if positive else ""))
    return number


def build_document(template, values):
    """Transform form values into a validated personal YAML document."""
    result = copy.deepcopy(template)
    result["schema_version"] = CONFIG_SCHEMA_VERSION
    name = str(values["campaign_name"]).strip()
    if not NAME_RE.fullmatch(name):
        raise ValueError("캠페인 이름은 영문/숫자로 시작하는 2~60자 ASCII여야 합니다")
    result["campaign_name"] = name
    result.setdefault("scenarios", {})["directory"] = str(values["directory"]).strip()
    result["scenarios"]["glob"] = str(values["glob"]).strip()
    if not result["scenarios"]["directory"] or not result["scenarios"]["glob"]:
        raise ValueError("시나리오 폴더와 파일 패턴이 필요합니다")
    goal = result.setdefault("default_case", {}).setdefault("goal", {})
    for key in ("x", "y", "z", "yaw_deg"):
        goal[key] = finite_number(values["goal_" + key], "목적지 " + key)
    timeout = finite_number(values["timeout_sec"], "주행 제한 시간", positive=True)
    result["default_case"]["timeout_sec"] = timeout
    stack = result.setdefault("stack", {})
    for key in ("pose_source", "detector"):
        stack[key] = str(values[key])
    stack["traffic_light"] = bool(values["traffic_light"])
    model = LABEL_PRESETS.get(values.get('bevfusion_model', PRESETS[DEFAULT_MODEL]['label']))
    if model is None:
        raise ValueError('모델 프리셋을 선택하세요')
    if model != template['stack'].get('bevfusion_model', DEFAULT_MODEL):
        stack['bevfusion_model'] = model
    mode = LABEL_MODES.get(str(values["traffic_light_runtime"]))
    if mode is None:
        raise ValueError("신호등 실행 환경을 선택하세요")
    old = form_values(template)
    # Never change existing resume hashes merely by opening/saving an old YAML.
    if any(str(values[key]) != str(old[key]) for key in (
            "traffic_light_runtime", "traffic_light_python", "traffic_light_device")):
        stack["traffic_light_runtime"] = mode
        stack["traffic_light_python"] = str(values["traffic_light_python"]).strip()
        stack["traffic_light_device"] = str(values["traffic_light_device"]).strip()
    runtime_settings(stack, ROOT)
    recording = result.setdefault("recording", {})
    recording["topic_profile"] = str(values["topic_profile"]).strip()
    recording["include_sensors"] = bool(values["include_sensors"])
    recording.setdefault("video", {})["enabled"] = bool(values["video"])
    if values.get('recording_mode', '기본 녹화') != old['recording_mode']:
        apply_recording_mode(result, values['recording_mode'])
    if values.get('selected_topics', '') != old['selected_topics']:
        if values['selected_topics']:
            recording['selected_topics'] = bag_options.checked_topics(json.loads(values['selected_topics']))
        else:
            recording.pop('selected_topics', None)
    layout = str(values["video_layout"])
    if layout not in ("manual", "split"):
        raise ValueError("녹화 화면 배치는 manual 또는 split이어야 합니다")
    if "window_layout" in recording["video"] or layout != "manual":
        recording["video"]["window_layout"] = layout
    if layout == "split":
        recording["video"]["focus_planviz"] = False
    bottom_bar = normalize_bar_setting(values["bottom_bar_px"])
    if "bottom_bar_px" in recording["video"] or bottom_bar != "auto":
        recording["video"]["bottom_bar_px"] = bottom_bar
    network = result.setdefault("network", {})
    for key in ("sensor_json", "ego_network_json"):
        network[key] = str(values[key]).strip()
    host = str(values["scenario_load_host"]).strip()
    if not host:
        raise ValueError("ScenarioLoad IP가 비어 있습니다")
    result.setdefault("morai", {})["scenario_load_host"] = host
    tolerance = finite_number(values["goal_tolerance_m"], "도착 허용 거리", positive=True)
    result.setdefault("validation", {})["goal_tolerance_m"] = tolerance
    profile = UPLOAD_LABEL_PROFILES.get(str(values["upload_profile"]))
    if profile not in UPLOAD_PROFILES:
        raise ValueError("Google Drive 공유 모드를 선택하세요")
    enabled = bool(values["upload_enabled"])
    remote = str(values["upload_remote"]).strip()
    if enabled and not remote:
        raise ValueError("자동 업로드에는 Google Drive rclone 경로가 필요합니다")
    if "upload" in result or enabled or remote != DEFAULT_UPLOAD_REMOTE or profile != "basic":
        upload = result.setdefault("upload", {})
        upload["enabled"] = enabled
        if enabled or upload.get("rclone_remote") or remote != DEFAULT_UPLOAD_REMOTE:
            upload["rclone_remote"] = remote
        if "profile" in upload or enabled or profile != "basic":
            upload["profile"] = profile
    result["resume"] = True
    bag_options.ensure_video_display(recording)
    return result


def apply_recording_mode(document, label):
    bag_options.apply_mode(document, label)


def operator_document(template, latest, values, include):
    """Merge only GUI-authorized edits into the latest saved YAML.

    Fixed settings (including concurrent VSCode changes) are never rewritten.
    Same-field conflicts require reloading rather than silently overwriting.
    """
    baseline = form_values(template)
    proposed = copy.deepcopy(template)
    for key, path in EDITABLE_FIELDS.items():
        if str(values[key]) == str(baseline[key]):
            continue
        value = values[key]
        if key == 'repeat':
            try:
                value = int(str(value))
            except ValueError as error:
                raise ValueError('반복 횟수는 1~1000 정수입니다') from error
        elif key == 'upload_profile':
            value = UPLOAD_LABEL_PROFILES.get(str(value))
            if value is None:
                raise ValueError('공유 모드를 선택하세요')
        elif key == 'bevfusion_model':
            value = LABEL_PRESETS.get(str(value))
            if value is None:
                raise ValueError('모델 프리셋을 선택하세요')
        elif key == 'selected_topics':
            if not value:
                proposed.setdefault('recording', {}).pop('selected_topics', None)
                continue
            value = bag_options.checked_topics(json.loads(value))
        elif key in ('traffic_light', 'video', 'include_sensors', 'upload_enabled'):
            value = bool(value)
        else:
            value = str(value).strip()
        target = proposed
        for part in path[:-1]:
            target = target.setdefault(part, {})
        target[path[-1]] = value
    if include is None:
        proposed['scenarios'].pop('include', None)
    else:
        proposed['scenarios']['include'] = list(include)
    paths = list(EDITABLE_FIELDS.values()) + [('scenarios', 'include')]
    if values['recording_mode'] != baseline['recording_mode']:
        apply_recording_mode(proposed, values['recording_mode'])
        paths += bag_options.RECORDING_PATHS + [('recording', 'video', 'enabled')]
        # The dialog can select exact topics while changing from a preset to custom.
        if values['recording_mode'] == bag_options.CUSTOM and values['selected_topics']:
            proposed['recording']['selected_topics'] = bag_options.checked_topics(json.loads(values['selected_topics']))
    if values['selected_topics'] != baseline['selected_topics'] and values['selected_topics']:
        # The dialog's explicit list is authoritative, including any sensor removals.
        # Do not append legacy include_sensors behind the user's back.
        proposed['recording']['include_sensors'] = False
        paths += [('recording', 'include_sensors')]
    missing = object()

    def at(document, path):
        for part in path:
            if not isinstance(document, dict) or part not in document:
                return missing
            document = document[part]
        return document

    result = copy.deepcopy(latest)
    for path in paths:
        old, new, saved = at(template, path), at(proposed, path), at(latest, path)
        if old == new:
            continue
        if saved != old and saved != new:
            raise ValueError('{}은 VSCode와 GUI 양쪽에서 바뀌었습니다. 다시 불러온 뒤 수정하세요'.format('.'.join(path)))
        target = result
        for part in path[:-1]:
            target = target.setdefault(part, {})
        if new is missing:
            target.pop(path[-1], None)
        else:
            target[path[-1]] = new
    if (bool(values['upload_enabled']) and not baseline['upload_enabled']
            and not result.get('upload', {}).get('rclone_remote')):
        # Older personal files may not have an upload destination yet. Fill the
        # displayed default only when enabling upload, keeping custom paths.
        result.setdefault('upload', {})['rclone_remote'] = DEFAULT_UPLOAD_REMOTE
    # A video-OFF agent YAML may contain enabled only; the checkbox must also
    # supply the hidden X11 address, without replacing explicit VSCode settings.
    bag_options.ensure_video_display(result.get('recording', {}))
    validate_campaign_document(result)
    return result


def mission_directory(source):
    if source is None:
        return None
    try:
        parts = resolve_path(source).relative_to(MISSIONS.resolve()).parts
    except ValueError:
        return None
    if len(parts) >= 2 and parts[1] in ("base", "workspace"):
        return MISSIONS / parts[0]
    return None


def is_personal(path):
    mission = mission_directory(path)
    if mission and is_within(path, mission / "base"):
        return False  # A workspace symlink must not make shared base editable.
    return is_within(path, USERS) or bool(mission and is_within(path, mission / "workspace"))


def personal_path(document, user=None, source=None):
    mission = mission_directory(source)
    if mission:
        return mission / "workspace" / (document["campaign_name"] + ".yaml")
    username = re.sub(r"[^a-zA-Z0-9_-]", "_", user or getpass.getuser())
    return USERS / username / (document["campaign_name"] + ".yaml")


def personal_directory(source=None):
    """Open only authoring workspaces, never shared base or execution records."""
    if source is not None and is_personal(source):
        return resolve_path(source).parent
    directory = personal_path({"campaign_name": "unused"}, source=source).parent.resolve()
    if not is_personal(directory / "unused.yaml"):
        raise ValueError("개인 작업 폴더가 아닌 경로는 열 수 없습니다")
    return directory


def open_personal_folder(source=None):
    directory = personal_directory(source)
    directory.mkdir(parents=True, exist_ok=True)
    subprocess.Popen(["xdg-open", str(directory)],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return directory


def copy_base_to_workspace(source):
    """Copy missing inputs, preserve all user edits, create a distinct campaign."""
    mission = mission_directory(source)
    if mission is None:
        raise ValueError("미션별 base 예제 또는 workspace YAML을 먼저 선택하세요")
    base = mission / "base"
    workspace = mission / "workspace"
    for path in (base / "scenarios").iterdir():
        if not path.is_file() or path.suffix.lower() != ".json":
            continue
        target = workspace / "scenarios" / path.name
        target.parent.mkdir(parents=True, exist_ok=True)
        content = path.read_bytes()
        # Exclusive creation prevents overwriting an existing scenario.
        try:
            with target.open("xb") as stream:
                try:
                    stream.write(content)
                except OSError:
                    target.unlink(missing_ok=True)
                    raise
        except FileExistsError:
            pass
    if (base / "README.md").is_file():
        # Keep links valid in workspace and preserve the user's own notes.
        note = workspace / "README.md"
        note.parent.mkdir(parents=True, exist_ok=True)
        try:
            with note.open("x", encoding="utf-8") as stream:
                try:
                    stream.write(
                        "# 내 미션 작업 폴더\n\n"
                        "시작점·목적지·배치 의도·확장 예시·담당 AI 프롬프트는 "
                        "[공유 미션 안내](../base/README.md)를 먼저 보세요.\n\n"
                        "이 폴더의 YAML·JSON·메모는 Git 제외입니다. 개인 시나리오를 수정한 내용은 "
                        "공유 안내나 manifest에 자동 반영되지 않습니다. 변경 조건과 배우 실기 확인 여부를 "
                        "여기에 메모해 두세요. GUI 복사를 다시 해도 기존 파일과 메모는 덮어쓰지 않습니다.\n")
                except OSError:
                    note.unlink(missing_ok=True)
                    raise
        except FileExistsError:
            pass
    if is_personal(source) and resolve_path(source).is_file():
        return resolve_path(source)
    template = resolve_path(source) if is_within(source, base) else base / "example.yaml"
    document = load_document(template)
    name = default_name(template)
    while True:
        document["campaign_name"] = name
        path = personal_path(document, source=source)
        if path.exists():
            return path  # Reopen the reusable personal YAML; never overwrite edits.
        break
    document["scenarios"]["directory"] = str((workspace / "scenarios").relative_to(ROOT))
    save_personal(document, path)
    return path


def save_personal(document, path, *, text=None, expected_text=None):
    """Authoring YAMLs are reusable; only snapshots and old legacy inputs are locked."""
    path = Path(path)
    if executions.is_snapshot(path) or (path.parent / 'execution.json').exists():
        raise ValueError('실행 사본은 수정할 수 없습니다. 원본 개인 YAML을 편집하세요')
    mission = mission_directory(path)
    if is_within(path, EXAMPLES) or (mission and is_within(path, mission / "base")):
        raise ValueError("공유 base/예제 YAML은 수정하지 않습니다. workspace에 저장하세요")
    output = resolve_path(document.get("output_root", "planner_logs/validation")) / document["campaign_name"]
    state_path = output / "campaign_state.json"
    if state_path.is_file():
        state = json.loads(state_path.read_text(encoding="utf-8"))
        if (state.get("execution_contract") != EXECUTION_CONTRACT or not path.is_file() or
                state.get("config_path", str(path.resolve())) != str(path.resolve()) or
                (state.get("config_sha256") and
                 state["config_sha256"] != hashlib.sha256(path.read_bytes()).hexdigest()) or
                load_document(path) != document):
            raise ValueError("실행된 캠페인의 YAML은 수정할 수 없습니다. 새 캠페인 이름을 사용하세요")
        return path
    if path.is_file():
        old = load_document(path)
        if old == document:
            return path
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp.{}".format(os.getpid()))
    try:
        if expected_text is not None and path.read_text(encoding='utf-8') != expected_text:
            raise ValueError('저장 중 YAML이 다른 곳에서 바뀌었습니다. 다시 불러오세요')
        content = annotated_yaml(document) if text is None else text
        if yaml.safe_load(content) != document:
            raise ValueError('저장 내용이 설정과 일치하지 않습니다. 원본은 변경하지 않았습니다.')
        temporary.write_text(content, encoding="utf-8")
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)
    return path


def campaign_output(document):
    return resolve_path(document.get("output_root", "planner_logs/validation")) / document["campaign_name"]


def result_counts(state):
    results = state.get("results", {})
    return {key: sum(row.get("status") == key for row in results.values())
            for key in ("PASS", "FAIL", "ERROR")}


def completion_label(code, state):
    """Show startup causes separately from legitimate per-case drive failures."""
    if code not in (0, 130) and state and state.get('startup_error'):
        failure = state['startup_error']
        return '실행 준비 실패 [{}] {} | 종료 코드 {} · stack_logs/startup_error.log 확인'.format(
            failure.get('phase', 'UNKNOWN'), failure.get('error', ''), code)
    if state is None:
        return "실행 종료 (코드 {}). 결과 파일 없음 — 로그를 확인하세요.".format(code)
    counts = result_counts(state)
    total = state.get("total", "?")
    completed = len(state.get("results", {}))
    label = "검증 종료 {}/{} | 성공 {} / 주행 실패 {} / 시스템 오류 {} | 종료 코드 {}".format(
        completed, total, counts["PASS"], counts["FAIL"], counts["ERROR"], code)
    if state.get("upload_errors"):
        label += " | Drive 업로드 오류 {}건 (로컬 결과 보존)".format(len(state["upload_errors"]))
    warnings = sum(bool(row.get('recording_warning')) for row in state.get('results', {}).values())
    if warnings:
        label += ' | 녹화 경고 {}건 (주행 판정과 별개)'.format(warnings)
    analyzed = sum(bool(row.get('analysis_complete')) for row in state.get('results', {}).values())
    shared = sum(row.get('share_status') == 'VERIFIED' for row in state.get('results', {}).values())
    label += ' | 분석 가능 {} / 공유 검증 완료 {}'.format(analyzed, shared)
    return label


def render_result_table(table, state, result_paths):
    """Update rows in place so a selected failure stays selected while polling."""
    results = state.get("results", {})
    for old in table.get_children():
        if old not in results:
            table.delete(old)
            result_paths.pop(old, None)
    for name, row in sorted(results.items()):
        values = (name, row.get("status", ""), row.get("reason", ""),
                  row.get("failure_category", ""),
                  "" if row.get("status") == "PASS" else row.get("suspected_stage", ""))
        if table.exists(name):
            if tuple(table.item(name, "values")) != values:
                table.item(name, values=values)
        else:
            table.insert("", "end", iid=name, values=values)
        result_paths[name] = Path(row["result_path"])


def is_within(path, directory):
    try:
        Path(path).resolve().relative_to(Path(directory).resolve())
        return True
    except ValueError:
        return False


def main(*, on_ready=None, runner_executable=None, batch_executable=None):
    """Open the GUI; optional hooks let Xvfb tests drive buttons without ROS."""
    import tkinter as tk
    from tkinter import filedialog, font as tkfont, messagebox, ttk

    examples = campaign_examples()
    if not examples:
        raise RuntimeError("검증기 예제 YAML이 없습니다: {}".format(EXAMPLES))
    restored = remembered_yaml()
    missions = available_missions()
    if not missions:
        raise RuntimeError('미션 base/example.yaml이 없습니다: {}'.format(MISSIONS))
    if restored not in [path for key in missions for path in mission_yaml_choices(key).values()]:
        restored = None  # Preserve old files, but no external-path input in the GUI.
    initial = restored or default_mission_yaml('static_obstacle' if 'static_obstacle' in missions else missions[0])
    runner = str(runner_executable or ROOT / "planning_validator")

    root = tk.Tk()
    # Tk windows do not reliably advertise _NET_WM_PID on X11. A unique title
    # lets the outer-geometry helper select this instance, not another GUI.
    root.title("AjouNice 주행 시나리오 검증기 [{}]".format(os.getpid()))
    root.geometry("1010x810")
    root.minsize(850, 650)
    frame = ttk.Frame(root, padding=10)
    frame.pack(fill="both", expand=True)
    source = tk.StringVar()
    mission_value = tk.StringVar()
    yaml_value = tk.StringVar()
    picker_choices = {}
    status = tk.StringVar(value="준비됨 — MORAI GUI에서 Sensor/Ego Network를 먼저 Load·Connect하세요.")
    live_status = tk.StringVar(value="")
    vars_ = {}
    field_widgets = {}
    quick_vars = {}
    run_summary = tk.StringVar()
    active = {"process": None, "check_process": None, "config": None,
              "document": None, "execution": None, "stopping": False, "mode": "single", "batch_document": None}
    try:
        batch_queue = batch.read_queue()
    except (OSError, ValueError) as error:
        # Preserve the damaged file; explicit reset is required before writing.
        batch_queue = {"schema_version": 1, "items": [], "run_directory": None, "load_error": str(error)}
    batch_buttons = []
    batch_preview = {}
    rerun_failed = tk.BooleanVar(value=False)
    result_paths = {}
    events = queue.Queue()
    poll_after = {"id": None}
    compact = {"enabled": False, "previous_geometry": None,
               "previous_zoomed": False, "previous_fullscreen": False}

    monitor_panel = ttk.LabelFrame(frame, text="실시간 캠페인 현황", padding=6)
    monitor_panel.pack(fill="x", pady=(0, 4))
    status_label = ttk.Label(monitor_panel, textvariable=status, wraplength=1840)
    status_label.pack(anchor="w")
    live_label = ttk.Label(monitor_panel, textvariable=live_status, wraplength=1840)
    live_label.pack(anchor="w")

    def read_source(path, fresh_name=False):
        if any(active.get(key) and active[key].poll() is None for key in ("process", "check_process")):
            raise ValueError("실행/검사 중에는 다른 YAML을 열 수 없습니다")
        path = resolve_path(pasted_yaml_path(path))
        document = load_document(path)
        values = form_values(document)
        changed_source = path != active.get('source_path')
        active['loading'] = True
        active["template"] = document
        active["source_path"] = Path(path)
        active["config"] = Path(path) if is_personal(path) else None
        active["document"] = document if active["config"] else None
        active["mode"] = "single"
        active['execution'] = None
        active['history_root'] = None
        active['scenario_include'] = copy.deepcopy(document['scenarios'].get('include'))
        result_paths.clear()
        for item in table.get_children():
            table.delete(item)
        live_status.set("")
        source.set(str(path))
        refresh_mission_picker(path)
        for key, value in values.items():
            vars_[key].set(value)
        active['loading'] = False
        update_recording_widgets()
        update_run_summary()
        if quick_vars:
            quick_vars['detector'].set(values['detector'])
            quick_vars['pose_source'].set(values['pose_source'])
            quick_vars['traffic_light'].set(values['traffic_light'])
            if changed_source and quick_table.selection():
                quick_table.selection_remove(quick_table.selection())
            refresh_quick_choices()
        status.set("설정 로드: {} — {}".format(Path(path).name,
                   "이전 v1 설정: 새 v2 예제를 복사해서 사용하세요."
                   if document.get("schema_version") == 1 else "목적지·제한시간은 모든 장면에 공통 적용"))
        remember_yaml(path)

    top = ttk.LabelFrame(frame, text="내 미션 선택하기", padding=8)
    top.pack(fill="x")
    ttk.Label(top, text='담당 미션').grid(row=0, column=0, sticky='w', padx=4)
    mission_box = ttk.Combobox(top, textvariable=mission_value,
                               values=[MISSION_LABELS[key] for key in missions], state='readonly', width=17)
    mission_box.grid(row=0, column=1, sticky='ew', padx=4)
    ttk.Label(top, text='실험 YAML').grid(row=0, column=2, sticky='w', padx=4)
    source_box = ttk.Combobox(top, textvariable=yaml_value, state='readonly', width=45)
    source_box.grid(row=0, column=3, columnspan=2, sticky='ew', padx=4)

    def selected_mission():
        for key in missions:
            if MISSION_LABELS[key] == mission_value.get():
                return key
        raise ValueError('담당 미션을 선택하세요')

    def refresh_mission_picker(path):
        mission = mission_directory(path)
        if mission is None or mission.name not in missions:
            # Internal compatibility/test hooks can still open legacy files.
            # They are never offered as selectable authoring inputs.
            yaml_value.set('')
            return
        mission_value.set(MISSION_LABELS[mission.name])
        picker_choices.clear()
        picker_choices.update(mission_yaml_choices(mission.name))
        source_box.configure(values=list(picker_choices))
        yaml_value.set(next((label for label, value in picker_choices.items()
                             if value.resolve() == Path(path).resolve()), ''))

    def select_source(_event):
        try:
            if _event is None:
                path = active.get('source_path')
                # Reload current file and refresh the list of newly added YAMLs.
                read_source(path)
            else:
                choices = mission_yaml_choices(selected_mission())
                if yaml_value.get() not in choices:
                    raise ValueError('선택한 미션의 YAML 목록에서 고르세요')
                read_source(choices[yaml_value.get()])
        except Exception as error:
            refresh_mission_picker(active.get('source_path'))
            messagebox.showerror("설정 로드 실패", str(error))

    def select_mission(_event=None):
        try:
            read_source(default_mission_yaml(selected_mission()))
        except Exception as error:
            refresh_mission_picker(active.get('source_path'))
            messagebox.showerror('미션 선택', str(error))

    mission_box.bind('<<ComboboxSelected>>', select_mission)
    source_box.bind("<<ComboboxSelected>>", select_source)

    def browse_personal_folder():
        try:
            if any(active.get(key) and active[key].poll() is None for key in ("process", "check_process")):
                raise ValueError("실행/검사 중에는 입력 파일을 정리하지 마세요. 중단·정리가 끝난 뒤 작업 폴더를 여세요")
            directory = open_personal_folder(active.get("source_path"))
            status.set("내 작업 폴더: {} — 파일 탐색기에서 삭제·이름 변경 가능. 현재 열린 YAML을 삭제했다면 다른 YAML을 열어 주세요.".format(directory))
        except Exception as error:
            messagebox.showerror("작업 폴더 열기 실패", str(error))

    folder_button = ttk.Button(top, text="내 작업 폴더 열기", command=browse_personal_folder)
    folder_button.grid(row=1, column=4, padx=4, pady=3)

    def copy_workspace():
        try:
            if any(active.get(key) and active[key].poll() is None for key in ("process", "check_process")):
                raise ValueError("실행/검사 중에는 작업 폴더를 준비할 수 없습니다")
            path = copy_base_to_workspace(active.get("source_path"))
            read_source(path)
            status.set("workspace 준비 완료 (기존 시나리오 유지): {}".format(path))
        except Exception as error:
            messagebox.showerror("베이스 복사 실패", str(error))

    copy_button = ttk.Button(top, text="베이스 → 내 작업 폴더 복사 (최초 1회 실행)", command=copy_workspace)
    copy_button.grid(row=1, column=3, padx=4, pady=3, sticky='ew')
    def edit_yaml():
        try:
            if any(active.get(key) and active[key].poll() is None for key in ('process', 'check_process')):
                raise ValueError('실행/검사 중에는 YAML 편집을 열지 않습니다. 안전 중단 후 편집하세요')
            path = resolve_path(pasted_yaml_path(source.get()))
            if not path.is_file():
                raise ValueError('먼저 존재하는 YAML을 불러오세요')
            mission = mission_directory(path)
            if mission and is_within(path, mission / 'base'):
                raise ValueError('공유 예제 대신 workspace 복사본을 편집하세요')
            if executions.is_snapshot(path) or (path.parent / 'execution.json').exists():
                raise ValueError('실행 사본은 편집하지 않습니다. 원본 개인 YAML을 여세요')
            command = ['code', str(path)] if shutil.which('code') else ['xdg-open', str(path)]
            subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as error:
            messagebox.showerror('YAML 편집', str(error))
    edit_button = ttk.Button(top, text='YAML 편집 (VSCode)', command=edit_yaml)
    edit_button.grid(row=2, column=4, padx=4, pady=3)
    top.columnconfigure(3, weight=1)

    notebook = ttk.Notebook(frame)
    notebook.pack(fill="x", pady=8)
    basic = ttk.Frame(notebook, padding=10)
    advanced = ttk.Frame(notebook, padding=10)
    notebook.add(basic, text="실행")
    notebook.add(advanced, text="설정 확인 (YAML 수정)")
    # One scrollable read-only settings pane instead of a second environment editor.
    settings_canvas = tk.Canvas(advanced, height=275, highlightthickness=0)
    settings_scroll = ttk.Scrollbar(advanced, orient='vertical', command=settings_canvas.yview)
    settings_canvas.configure(yscrollcommand=settings_scroll.set)
    settings_scroll.pack(side='right', fill='y')
    settings_canvas.pack(side='left', fill='both', expand=True)
    settings_body = ttk.Frame(settings_canvas)
    settings_window = settings_canvas.create_window((0, 0), window=settings_body, anchor='nw')
    settings_body.bind('<Configure>', lambda _e: settings_canvas.configure(scrollregion=settings_canvas.bbox('all')))
    settings_canvas.bind('<Configure>', lambda e: settings_canvas.itemconfigure(settings_window, width=e.width))
    advanced = settings_body
    traffic_tab = ttk.Frame(advanced)

    def add_field(parent, row, key, label, *, choices=None, boolean=False):
        var = tk.BooleanVar() if boolean else tk.StringVar()
        vars_[key] = var
        ttk.Label(parent, text=label, width=19).grid(row=row, column=0, sticky="w", pady=3)
        if boolean:
            widget = ttk.Checkbutton(parent, variable=var)
        elif choices:
            widget = ttk.Combobox(parent, textvariable=var, values=choices, state="readonly", width=70)
        else:
            widget = ttk.Entry(parent, textvariable=var, width=80)
        widget.grid(row=row, column=1, sticky="ew", pady=3)
        editable = key in EDITABLE_FIELDS
        widget.configure(state=('readonly' if choices else 'normal') if editable
                         else ('disabled' if boolean or choices else 'readonly'))
        if not editable and not boolean and not choices:
            widget.configure(style='Yaml.TEntry')
        field_widgets[key] = widget
        parent.columnconfigure(1, weight=1)
        return widget

    add_field(basic, 0, "campaign_name", "실험 이름")
    ttk.Style(root).configure('Yaml.TEntry', foreground='#777777', fieldbackground='#eeeeee')
    add_field(basic, 1, "directory", "시나리오 JSON 폴더")
    add_field(basic, 2, "glob", "전체 실행 파일 패턴")
    selection_button = ttk.Button(basic, text='장면 선택…', command=lambda: select_scenarios())
    selection_button.grid(row=2, column=2, padx=4)
    batch_buttons.append(selection_button)
    add_field(basic, 3, 'repeat', 'JSON당 반복 횟수')
    for row, (key, label) in enumerate((("x", "목적지 map X (m)"), ("y", "목적지 map Y (m)"),
                                        ("z", "목적지 map Z (m)"), ("yaw_deg", "목적지 yaw (deg)")), 3):
        add_field(advanced, row - 3, "goal_" + key, label)
    add_field(advanced, 4, "timeout_sec", "공통 주행 제한 (초)")
    add_field(basic, 4, "pose_source", "측위", choices=["localization", "maptf_morai"])
    add_field(basic, 5, "detector", "객체 탐지", choices=["od_gt", "bevfusion"])
    add_field(basic, 6, 'bevfusion_model', 'BEVFusion 모델', choices=[v['label'] for v in PRESETS.values()])
    vars_['selected_topics'] = tk.StringVar()
    vars_['include_sensors'] = tk.BooleanVar()  # Legacy YAML state only; no second GUI control.
    add_field(basic, 7, 'recording_mode', 'rosbag 녹화', choices=list(bag_options.MODES))
    field_widgets['recording_mode'].configure(state='readonly')
    topics_button = ttk.Button(basic, text='토픽 선택…', command=lambda: select_bag_topics())
    topics_button.grid(row=7, column=2, padx=4)
    toggle_bar = ttk.Frame(basic)
    toggle_bar.grid(row=8, column=0, columnspan=3, sticky='w', pady=4)
    for key, label in (('traffic_light', '신호등 인지'), ('video', '화면 영상 녹화'),
                       ('upload_enabled', 'Drive 자동 업로드')):
        vars_[key] = tk.BooleanVar()
        widget = ttk.Checkbutton(toggle_bar, text=label, variable=vars_[key])
        widget.pack(side='left', padx=6)
        field_widgets[key] = widget
    add_field(basic, 9, "upload_profile", "공유 모드", choices=list(UPLOAD_PROFILE_LABELS.values()))
    ttk.Label(basic, textvariable=run_summary, wraplength=900).grid(row=10, column=0, columnspan=3, sticky='w', pady=4)
    ttk.Label(basic, text='GUI 수정 → YAML 저장 → 설정 검사·실험 시작 | VSCode 저장 후에는 불러오기/새로고침',
              foreground='#285b86').grid(row=11, column=0, columnspan=3, sticky='w')
    recording_summary = tk.StringVar()
    ttk.Label(basic, textvariable=recording_summary, wraplength=900,
              foreground='#856333').grid(row=12, column=0, columnspan=3, sticky='w')
    def form_recording():
        recording = copy.deepcopy(active.get('template', {}).get('recording', {}))
        label = vars_['recording_mode'].get()
        if label != bag_options.mode_label(recording):
            bag_options.apply_mode({'recording': recording}, label)
        recording['include_sensors'] = vars_['include_sensors'].get()
        selected = vars_['selected_topics'].get()
        if label == bag_options.CUSTOM and selected:
            recording['selected_topics'] = json.loads(selected)
        return recording

    def update_recording_widgets(*_):
        label = vars_['recording_mode'].get()
        enabled = label != bag_options.OFF
        running = active.get('ui_running', False) or any(active.get(key) and active[key].poll() is None for key in ('process', 'check_process'))
        field_widgets['video'].configure(state='disabled' if label == bag_options.ANALYSIS or running else 'normal')
        topics_button.configure(state='normal' if enabled and not running else 'disabled')
        try:
            topics = bag_options.effective_topics(form_recording(), resolve_path)
            sensor_count = len(set(topics) & set(bag_options.SENSORS))
            text = ('bag OFF · 실시간 판정·JSON·CSV 유지 · 영상은 별도 선택' if not enabled else
                    'bag {}개 토픽 · 원본 센서 {}/5개 · {}'.format(len(topics), sensor_count,
                    '사후 재추론 가능' if sensor_count == 5 else '원본 센서 부족: 사후 재추론 불가'))
            profile = next((key for key, value in UPLOAD_PROFILE_LABELS.items()
                            if value == vars_['upload_profile'].get()), 'basic')
            warnings = bag_options.sharing_warnings(form_recording(), {'profile': profile})
            recording_summary.set(text + ('\n' + warnings[0] if warnings else '')
                                  + '\nMORAI 센서·모델 유지. 분석 프리셋은 상세 진단도 발행 · 다음 실행부터 적용')
        except (ValueError, OSError):
            recording_summary.set('토픽 선택을 확인하세요. 화면 영상은 bag과 별도로 켜고 끕니다.')

    def select_recording_mode(_=None):
        label = vars_['recording_mode'].get()
        if label in (bag_options.LIGHT, bag_options.BASIC, bag_options.ANALYSIS, bag_options.LATENCY, bag_options.LOCALIZATION):
            vars_['selected_topics'].set('')
            vars_['include_sensors'].set(label == bag_options.ANALYSIS)
        if label == bag_options.ANALYSIS:
            vars_['video'].set(True)
        update_recording_widgets()
        changed()

    def select_bag_topics():
        try:
            if any(active.get(key) and active[key].poll() is None for key in ('process', 'check_process')):
                raise ValueError('실행/검사 중에는 녹화 토픽을 변경할 수 없습니다. 종료 후 다음 실행에 적용하세요.')
            current = bag_options.effective_topics(form_recording(), resolve_path)
            selected = bag_options.choose_topics(root, bag_options.topic_catalog(ROOT, current), current)
            if selected is not None:
                vars_['recording_mode'].set(bag_options.CUSTOM)
                vars_['selected_topics'].set(json.dumps(selected, ensure_ascii=False))
                vars_['include_sensors'].set(False)  # Explicit list is the entire selection.
                update_recording_widgets()
                changed()
        except Exception as error:
            show_error('녹화 토픽 선택', error)
    field_widgets['recording_mode'].bind('<<ComboboxSelected>>', select_recording_mode)
    vars_['upload_profile'].trace_add('write', lambda *_: None if active.get('loading') else update_recording_widgets())
    for key in ('include_sensors', 'selected_topics', 'video'):
        vars_[key].trace_add('write', lambda *_: None if active.get('loading') else update_recording_widgets())

    for row, key, label in ((10, 'topic_profile', 'rosbag 토픽 JSON'), (11, 'sensor_json', 'MORAI Sensor JSON'),
                            (12, 'ego_network_json', 'MORAI Ego Network JSON'), (13, 'scenario_load_host', 'ScenarioLoad IP'),
                            (14, 'goal_tolerance_m', '목적지 허용 반경 (m)'), (15, 'video_layout', '녹화 화면 배치'),
                            (16, 'bottom_bar_px', 'GUI 하단 높이'), (17, 'upload_remote', 'Drive rclone 경로')):
        add_field(advanced, row, key, label)
    traffic_tab.grid(row=18, column=0, columnspan=3, sticky='ew')

    traffic_mode = add_field(traffic_tab, 0, "traffic_light_runtime", "실행 환경",
                             choices=list(MODE_LABELS.values()))
    traffic_python = add_field(traffic_tab, 1, "traffic_light_python", "가상환경 Python")
    add_field(traffic_tab, 2, "traffic_light_device", "신호등 장치 (0/cpu)")
    ttk.Label(traffic_tab, text="시스템 선택: /usr/bin/python3 (기존 환경 유지). 가상환경 선택: 위 실행 파일 사용.\n"
              "GPU 번호 0이 기본값입니다. CPU 시험은 cpu로 입력하세요. 신호등 off이면 환경이 없어도 실행됩니다.",
              wraplength=750).grid(row=3, column=0, columnspan=3, sticky="w", pady=8)

    ttk.Label(traffic_tab, text="설치는 최초 1회입니다. Conda activate는 필요 없습니다.\n"
              "실행 버튼을 누르면 선택한 Python으로 모델/GPU/ROS import를 먼저 검사합니다.\n"
              "환경 변경은 원본 YAML에 저장하고 새 실험을 시작하세요. 이전 기록은 당시 환경 설정으로 재개합니다.", wraplength=750).grid(
        row=5, column=0, columnspan=3, sticky="w", pady=8)

    def update_traffic_widgets(*_):
        traffic_mode.configure(state="disabled")
        traffic_python.configure(state="readonly")

    vars_["traffic_light"].trace_add("write", update_traffic_widgets)
    vars_["traffic_light_runtime"].trace_add("write", update_traffic_widgets)

    def update_run_summary():
        try:
            spec = {'directory': vars_['directory'].get(), 'glob': vars_['glob'].get()}
            if active.get('scenario_include') is not None:
                spec['include'] = active['scenario_include']
            count = len(scenario_sources(spec))
            repeat = int(vars_['repeat'].get())
            selected = active.get('scenario_include') is not None
            selection_button.configure(text='선택 {}개…'.format(count) if selected else '장면 선택…')
            run_summary.set('JSON {}개 × 반복 {}회 = 총 {}회 · {} · 랜덤 범위는 YAML 설정 유지'.format(
                count, repeat, count * repeat, '선택 목록' if selected else '파일 패턴'))
        except (ValueError, OSError):
            run_summary.set('파일 패턴·반복 횟수를 확인하세요. 장면 선택에서 전체 실행(패턴)으로 돌아갈 수 있습니다.')

    def changed(*_):
        if not active.get('loading'):
            status.set('설정 수정됨 — 상단 YAML 저장을 누르세요. 설정 검사·새 실행 전에도 자동 저장합니다.')
            update_run_summary()

    def select_scenarios():
        try:
            if any(active.get(key) and active[key].poll() is None for key in ('process', 'check_process')):
                raise ValueError('실행/검사 중에는 장면 선택을 바꿀 수 없습니다')
            applied, include = choose_scenarios(root, resolve_path(vars_['directory'].get()),
                                               vars_['glob'].get(), active.get('scenario_include'))
            if applied:
                active['scenario_include'] = include
                changed()
        except Exception as error:
            show_error('장면 선택', error)

    def pattern_changed(*_):
        if not active.get('loading'):
            active['scenario_include'] = None
            changed()

    for key in EDITABLE_FIELDS:
        vars_[key].trace_add('write', pattern_changed if key == 'glob' else changed)
    vars_['recording_mode'].trace_add('write', changed)

    def collect():
        path = resolve_path(pasted_yaml_path(source.get()))
        mission = mission_directory(path)
        if mission and is_within(path, mission / 'base'):
            raise ValueError("공유 예제는 바로 실행하지 않습니다. '베이스 → 내 작업 폴더 복사'부터 누르세요")
        if executions.is_snapshot(path) or (path.parent / 'execution.json').exists():
            raise ValueError('실행 사본 대신 원본 YAML을 선택하세요. 이어가기는 기록 탭에서 합니다')
        if path != active.get('source_path'):
            read_source(path)
        latest = load_document(path)
        values = {key: value.get() for key, value in vars_.items()}
        return operator_document(active['template'], latest, values, active.get('scenario_include')), path

    def save():
        if any(active.get(key) and active[key].poll() is None for key in ('process', 'check_process')):
            raise ValueError("실험 실행 중에는 GUI 저장을 잠급니다. 중단 후 원본 YAML을 수정·저장하세요")
        document, path = collect()
        original = path.read_text(encoding='utf-8')
        if yaml.safe_load(original) != document:
            save_personal(document, path, text=updated_yaml(original, document), expected_text=original)
        read_source(path)
        active.update(config=path, document=document)
        remember_yaml(path)
        status.set("YAML 저장 완료: {} (같은 개인 파일; 고정값·주석 유지)".format(path))
        return path

    def show_error(label, error):
        status.set("{}: {}".format(label, error))
        messagebox.showerror(label, str(error))

    def command_reader(process, label, log_path=None):
        try:
            stream_console(process, lambda line: events.put(('line', line)), log_path)
        except Exception as error:
            events.put(("line", "출력 수집 오류: {}".format(error)))
        finally:
            process.stdout.close()
            events.put(("done", label, process.wait()))

    def check():
        if active["check_process"] and active["check_process"].poll() is None:
            messagebox.showinfo("설정 검사 중", "현재 설정 검사가 끝날 때까지 기다려 주세요.")
            return
        try:
            path = save()
        except Exception as error:
            show_error("설정 검사 준비 실패", error)
            return
        status.set("설정 검사 중...")
        try:
            process = subprocess.Popen([runner, str(path), "--dry-run"],
                                       cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, bufsize=1)
        except OSError as error:
            show_error("설정 검사 실행 실패", error)
            return
        active["check_process"] = process
        set_running_ui(True, allow_stop=False)
        threading.Thread(target=command_reader, args=(process, "check"), daemon=True).start()

    def check_environment(gpu=False, live=False):
        try:
            if any(active.get(key) and active[key].poll() is None for key in ("process", "check_process")):
                raise ValueError("다른 실행/검사가 끝난 뒤 환경 검사를 시작하세요. 실행 중 수신 상태는 모니터의 실시간 값으로 확인하세요")
            path = save()
            command = ["/usr/bin/python3", str(PACKAGE / "scripts/planning_validator_doctor.py"), str(path)]
            if gpu:
                if not messagebox.askokcancel("GPU 검사", "컨테이너 GPU 연산과 선택된 신호등 실제 추론을 검사합니다. 잠시 GPU 메모리를 사용합니다. 계속할까요?"):
                    return
                command.append("--gpu")
            if live:
                command.append("--live")
            process = subprocess.Popen(command, cwd=str(ROOT), stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True, bufsize=1)
            active["check_process"] = process
            set_running_ui(True, allow_stop=False)
            status.set("환경 검사 중 — 아래 로그에서 PASS/WARN/FAIL과 해결 방법을 확인하세요")
            threading.Thread(target=command_reader, args=(process, "environment"), daemon=True).start()
        except Exception as error:
            show_error("환경 검사 실패", error)

    planning_tab = ttk.Frame(notebook, padding=8)
    notebook.insert(1, planning_tab, text='planning 빠른 테스트')
    ttk.Label(planning_tab, text='현재 YAML의 목적지·제한시간 그대로 JSON 1~2개를 한 번씩 시험합니다.\n'
              '원본 YAML 수정 없음 · 반복/파라미터 랜덤 OFF · 업로드 OFF · 결과/녹화는 별도 보관.',
              wraplength=900).pack(anchor='w', pady=4)
    quick_search = tk.StringVar()
    ttk.Entry(planning_tab, textvariable=quick_search).pack(fill='x')
    quick_list = ttk.Frame(planning_tab)
    quick_list.pack(fill='x', pady=4)
    quick_table = ttk.Treeview(quick_list, columns=('json',), show='headings', height=4, selectmode='extended')
    quick_table.heading('json', text='JSON 선택 (Ctrl/Shift로 최대 2개, 위 입력칸은 파일명 검색)')
    quick_scroll = ttk.Scrollbar(quick_list, orient='vertical', command=quick_table.yview)
    quick_table.configure(yscrollcommand=quick_scroll.set)
    quick_scroll.pack(side='right', fill='y')
    quick_table.pack(side='left', fill='x', expand=True)
    quick_files = {}

    def refresh_quick_choices(*_):
        old = quick_table.selection()
        for item in quick_table.get_children():
            quick_table.delete(item)
        try:
            directory = resolve_path(vars_['directory'].get())
            pattern = vars_['glob'].get()
            files = scenario_sources({'directory': str(directory), 'glob': pattern})
            quick_files.clear()
            for path in files:
                if quick_search.get().lower() in path.name.lower():
                    quick_files[path.name] = path
                    quick_table.insert('', 'end', iid=path.name, values=(path.name,))
            keep = [name for name in old if name in quick_files]
            if keep:
                quick_table.selection_set(keep)
            elif not quick_search.get():
                preferred = active.get('template', {}).get('scenarios', {}).get('include', [])
                default = [name for name in preferred if name in quick_files][:2]
                if not default:
                    default = list(quick_files)[:1]
                if default:
                    quick_table.selection_set(default)
        except (ValueError, OSError):
            quick_files.clear()

    quick_search.trace_add('write', refresh_quick_choices)
    quick_options = ttk.Frame(planning_tab)
    quick_options.pack(fill='x', pady=4)
    for key, label, values in (('detector', '탐지 대조', ['bevfusion', 'od_gt']),
                               ('pose_source', '측위 대조', ['localization', 'maptf_morai'])):
        ttk.Label(quick_options, text=label).pack(side='left', padx=4)
        quick_vars[key] = tk.StringVar()
        widget = ttk.Combobox(quick_options, textvariable=quick_vars[key], values=values, state='readonly', width=16)
        widget.pack(side='left', padx=4)
        batch_buttons.append(widget)
    quick_vars['traffic_light'] = tk.BooleanVar()
    quick_traffic = ttk.Checkbutton(quick_options, text='신호등', variable=quick_vars['traffic_light'])
    quick_traffic.pack(side='left', padx=6)
    batch_buttons.append(quick_traffic)

    def start_quick():
        try:
            if any(active.get(key) and active[key].poll() is None for key in ('process', 'check_process')):
                raise ValueError('현재 실행/검사가 끝난 뒤 빠른 테스트를 시작하세요')
            template, _ = collect()
            document = quick_test_document(template, list(quick_table.selection()),
                detector=quick_vars['detector'].get(), pose_source=quick_vars['pose_source'].get(),
                traffic_light=quick_vars['traffic_light'].get())
            validate_campaign_document(document)
            path = PACKAGE / 'config/internal/generated' / ('planning_quick_{}.yaml'.format(os.getpid()))
            save_personal(document, path)
            return start(config_path=path)
        except Exception as error:
            show_error('빠른 테스트 준비 실패', error)

    quick_run_button = ttk.Button(planning_tab, text='선택한 JSON 빠른 실행', command=start_quick)
    quick_run_button.pack(anchor='w', pady=3)
    batch_buttons.append(quick_run_button)
    environment_bar = ttk.Frame(planning_tab)
    environment_bar.pack(anchor='w', pady=4)
    for label, action in (("환경 검사", check_environment),
                          ("ROS 수신 검사", lambda: check_environment(live=True))):
        button = ttk.Button(environment_bar, text=label, command=action)
        button.pack(side="left", padx=3)
        batch_buttons.append(button)

    def copy_connection_paths():
        try:
            path = (active['execution']['path'] if active.get('execution') and active.get('mode') == 'single'
                    else resolve_path(pasted_yaml_path(source.get())))
            document = load_document(path)
            root.clipboard_clear()
            root.clipboard_append("Ego Network: {}\nSensor: {}".format(
                resolve_path(document["network"]["ego_network_json"]), resolve_path(document["network"]["sensor_json"])))
            status.set("MORAI에 Load할 두 JSON 경로를 복사했습니다. 기록 선택 시 해당 실험 사본 기준입니다. 연결은 MORAI GUI에서 직접 합니다.")
        except Exception as error:
            show_error('연결 JSON 경로 확인', error)
    ttk.Button(environment_bar, text="연결 JSON 경로 복사", command=copy_connection_paths).pack(side="left", padx=3)

    def start(execution=None, config_path=None):
        if active["check_process"] and active["check_process"].poll() is None:
            messagebox.showinfo("설정 검사 중", "현재 설정 검사가 끝난 후 캠페인을 시작하세요.")
            return
        if active["process"] and active["process"].poll() is None:
            messagebox.showinfo("이미 실행 중", "한 번에 한 캠페인만 실행할 수 있습니다.")
            return
        try:
            if execution is None:
                path = resolve_path(config_path) if config_path else save()
                document = load_document(path)
            else:
                cfg = executions.load_execution(execution, check_install=runner_executable is None)
                path = cfg['path']
                document = load_document(path)
        except Exception as error:
            show_error("실행 준비 실패", error)
            return
        video_enabled = document["recording"]["video"]["enabled"]
        split = document["recording"]["video"].get("window_layout") == "split"
        note = ("전체 화면을 녹화하며 RViz·MORAI를 반반 배치하고 GUI를 하단 모니터 바로 바꿉니다. "
                "창 배치가 실패하면 수동으로 조정해 주세요. 중단은 GUI 버튼 또는 터미널 Ctrl+C입니다."
                if video_enabled and split else
                "영상은 전체 화면을 녹화합니다. 필요하면 직접 GUI 창을 최소화하고 "
                "Planviz를 전면에 두세요. 중단은 GUI 버튼이나 실행 터미널 Ctrl+C로 가능합니다."
                if video_enabled else "실행 중 GUI의 중단 버튼을 사용할 수 있습니다.")
        action_note = ('이전 실험의 보관된 설정·시나리오로 재개합니다. 현재 편집 중인 YAML은 사용하지 않습니다.\n' if execution else
                       '새 결과 폴더에 현재 설정·시나리오 사본을 보관하고 처음부터 실행합니다.\n')
        if not messagebox.askokcancel("실험 재개" if execution else "새 실험 시작", action_note + "MORAI GUI에서 해당 Sensor/Ego Network JSON을 Load·Connect했나요?\n\n" + note):
            return
        try:
            if execution is None:
                cfg = executions.prepare_execution(path, check_install=runner_executable is None)
            command = [runner, str(cfg['path']), '--resume=true']
            if execution and rerun_failed.get():
                command.append("--rerun-failed")
            process = subprocess.Popen(command, cwd=str(ROOT),
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, bufsize=1, start_new_session=True)
        except (OSError, ValueError) as error:
            show_error("캠페인 실행 실패", error)
            return
        active.update(process=process, execution=cfg, document=document, stopping=False, mode="single")
        refresh_history()
        set_running_ui(True)
        if video_enabled and document["recording"]["video"].get("window_layout") == "split":
            set_compact_mode(True, document["recording"]["video"].get("bottom_bar_px", "auto"))
        status.set("주행 검증 실행 중: {}".format(cfg['name']))
        threading.Thread(target=command_reader,
            args=(process, "run", cfg['output'] / 'stack_logs/runner_console.log'), daemon=True).start()

    def stop(*, force=False):
        process = active["process"]
        if process and process.poll() is None and not active["stopping"]:
            if force or messagebox.askyesno("안전 중단", "현재 주행을 중단하고 기록을 보존할까요?\n다음 실행에서 이어갈 수 있습니다."):
                active["stopping"] = True
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                status.set("중단 요청 중 — 녹화와 ROS 스택 정리를 기다립니다")

    batch_tab = ttk.Frame(notebook, padding=8)
    notebook.add(batch_tab, text="실행 목록 / 배치")
    batch_note = tk.StringVar(value="YAML 여러 개를 순서대로 실행합니다. 목록은 자동 저장되고 중단 후 재개됩니다.")
    ttk.Label(batch_tab, textvariable=batch_note, wraplength=900).pack(anchor="w")
    batch_table = ttk.Treeview(batch_tab, columns=("name", "scenes", "minutes", "state", "counts"), show="headings", height=5)
    for key, title, width in (("name", "실험 YAML", 270), ("scenes", "장면", 60),
                              ("minutes", "예상 분", 75), ("state", "상태", 120), ("counts", "P/F/E", 95)):
        batch_table.heading(key, text=title)
        batch_table.column(key, width=width)
    batch_table.pack(fill="x", pady=4)
    batch_bar = ttk.Frame(batch_tab)
    batch_bar.pack(fill="x")

    def refresh_batch():
        try:
            value = batch.read_queue() if not batch_queue.get("load_error") else batch_queue
            batch_queue.update(value)
            state = batch.batch_state(value)
        except (OSError, ValueError) as error:
            batch_note.set("목록/결과 읽기 실패: {} — 기존 파일은 보존했습니다".format(error))
            return None
        states = {item["path"]: item for item in state["items"]} if state else {}
        ids = set()
        total_scenes, total_seconds = 0, 0
        for index, path in enumerate(value["items"]):
            iid = str(index)
            ids.add(iid)
            row = states.get(str(resolve_path(path)), {})
            if row:
                name, n, seconds = row["campaign"], row["scene_count"], row["estimate_sec"]
            else:
                key = str(path)
                try:
                    stamp = resolve_path(path).stat().st_mtime_ns
                    if key not in batch_preview or batch_preview[key][0] != stamp:
                        cfg = load_campaign(path, check_install=False)
                        batch_preview[key] = (stamp, cfg["name"], len(cfg["cases"]), sum(case.timeout_sec + 35 for case in cfg["cases"]))
                    _, name, n, seconds = batch_preview[key]
                except (ValueError, OSError):
                    name, n, seconds = Path(path).name, 0, 0
            total_scenes += n
            total_seconds += seconds
            counts = row.get("counts", {})
            values = (name, n, round(seconds / 60, 1), row.get("status", "대기"),
                      "{}/{}/{}".format(counts.get("PASS", 0), counts.get("FAIL", 0), counts.get("ERROR", 0)))
            if batch_table.exists(iid):
                batch_table.item(iid, values=values)
            else:
                batch_table.insert("", "end", iid=iid, values=values)
        for iid in batch_table.get_children():
            if iid not in ids:
                batch_table.delete(iid)
        if not batch_queue.get("load_error"):
            batch_note.set("YAML {}개 / 장면 {}개 / 약 {:.0f}분 (제한시간+준비 추정) · bag·영상은 각 YAML 설정대로 (원본 센서 고용량) | {}".format(
                len(value["items"]), total_scenes, total_seconds / 60,
                state.get("status", "미실행") if state else "미실행"))
        return state

    def edit_batch(operation):
        try:
            if any(active.get(key) and active[key].poll() is None for key in ("process", "check_process")):
                raise ValueError("실행/검사 중에는 목록을 바꿀 수 없습니다")
            if batch_queue.get("load_error") and operation != "reset":
                raise ValueError("목록 파일이 손상됐습니다. 파일을 백업하고 목록 초기화를 선택하세요")
            if batch_queue.get("run_directory") and operation != "reset":
                raise ValueError("이미 실행한 목록은 재개용으로 고정됩니다. 새 목록으로 바꾸려면 목록 초기화를 먼저 누르세요. 결과는 삭제하지 않습니다")
            items = list(batch_queue["items"])
            if operation == "current":
                items.append(str(save().resolve()))
            elif operation == "files":
                mission = selected_mission()
                allowed = {path.resolve() for path in mission_yaml_choices(mission).values() if is_personal(path)}
                selected = filedialog.askopenfilenames(initialdir=str(MISSIONS / mission / 'workspace'),
                                                       filetypes=[("내 미션 YAML", "*.yaml *.yml")])
                for path in selected:
                    if resolve_path(path) not in allowed:
                        raise ValueError('선택한 미션의 workspace YAML만 추가하세요. 예제는 먼저 복사합니다.')
                    document = load_document(path)
                    if document["schema_version"] != 2:
                        raise ValueError("이전 YAML은 먼저 열어서 v2 개인 캠페인으로 저장하세요")
                    if mission_directory(path) and is_within(resolve_path(path), mission_directory(path) / "base"):
                        raise ValueError("공유 base 대신 GUI에서 개인 YAML로 저장한 파일을 추가하세요")
                    items.append(str(resolve_path(path)))
            elif operation == "reset":
                if not messagebox.askyesno("목록 초기화", "실행 목록만 비울까요? 캠페인·배치 결과와 YAML은 삭제하지 않습니다. 이전 배치 재개 정보는 결과 폴더의 queue.json에 보존됩니다."):
                    return
                items = []
                if batch_queue.get("load_error") and batch.queue_path().exists():
                    shutil.copy2(batch.queue_path(), batch.queue_path().with_name("batch.broken.{}.json".format(os.getpid())))
            else:
                selected = batch_table.selection()
                if not selected:
                    return
                index = int(selected[0])
                if operation == "remove":
                    items.pop(index)
                elif operation in ("up", "down"):
                    target = index + (-1 if operation == "up" else 1)
                    if 0 <= target < len(items):
                        items[index], items[target] = items[target], items[index]
            if len(set(items)) != len(items):
                raise ValueError("같은 YAML을 두 번 추가할 수 없습니다")
            batch_queue.clear()
            batch_queue.update(schema_version=1, items=items, run_directory=None)
            batch.write_queue(batch_queue)
            refresh_batch()
        except Exception as error:
            show_error("목록 편집 실패", error)

    def check_batch():
        try:
            configs, rows = batch.plan(batch_queue["items"])
            batch_note.set("파일·배치 검사 통과: YAML {}개 / 장면 {}개. 실제 모델·센서 연결은 환경 검사로 확인하세요.".format(
                len(rows), sum(row["scene_count"] for row in rows)))
        except Exception as error:
            show_error("배치 검사 실패", error)

    def start_batch(new=False):
        try:
            if any(active.get(key) and active[key].poll() is None for key in ("process", "check_process")):
                raise ValueError("현재 실행/검사가 끝난 뒤 배치를 시작하세요")
            if batch_queue.get("load_error"):
                raise ValueError(batch_queue["load_error"])
            state = batch.batch_state(batch_queue) if not new else None
            if not new and state is None:
                raise ValueError('재개할 배치가 없습니다. 새 배치 시작을 누르세요')
            configs = ([executions.load_execution(row['output']) for row in state['items']]
                       if state else batch.plan(batch_queue['items'])[0])
            if not messagebox.askokcancel("새 배치 시작" if new else "배치 재개", "MORAI에 이 목록의 공통 Sensor/Ego Network를 Load·Connect했나요?\n" +
                    ('각 YAML의 새 입력 사본으로 처음부터 실행합니다. 이전 결과는 보존됩니다.' if new else
                     '보관된 입력으로 완료 장면은 건너뛰고 중단 장면부터 재개합니다.') + '\n주행 FAIL은 계속, 시스템 오류는 중단합니다.'):
                return
            if new:
                batch_queue['run_directory'] = None
                batch.write_queue(batch_queue)
            command = ([str(batch_executable)] if batch_executable else
                       ["/usr/bin/python3", str(PACKAGE / "scripts/planning_validator_batch.py")]) + [str(batch.queue_path())]
            process = subprocess.Popen(command, cwd=str(ROOT), stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True)
            active.update(process=process, execution=None, stopping=False, mode="batch", batch_document=None, batch_error=None)
            set_running_ui(True)
            video = configs[0]["recording"].get("video", {})
            if video.get("enabled") and video.get("window_layout") == "split":
                set_compact_mode(True, video.get("bottom_bar_px", "auto"))
            status.set("배치 시작 준비: YAML {}개".format(len(configs)))
            threading.Thread(target=command_reader, args=(process, "batch"), daemon=True).start()
        except Exception as error:
            show_error("배치 시작 실패", error)

    def restore_batch():
        try:
            if active["process"] and active["process"].poll() is None:
                raise ValueError("실행 중에는 이전 목록을 열 수 없습니다")
            path = filedialog.askopenfilename(initialdir=str(ROOT / "planner_logs/validation_batches"), filetypes=[("이전 배치 queue.json", "queue.json")])
            if path:
                value = batch.read_queue(path)
                batch.batch_state(value)  # Missing state is an actionable error.
                batch.write_queue(value)
                batch_queue.clear()
                batch_queue.update(value)
                refresh_batch()
        except Exception as error:
            show_error("이전 배치 열기 실패", error)

    def clone_batch():
        try:
            if any(active.get(key) and active[key].poll() is None for key in ("process", "check_process")):
                raise ValueError("실행/검사 중에는 새 실행으로 복사할 수 없습니다")
            if not batch_queue["items"]:
                raise ValueError("복사할 목록이 없습니다")
            if not messagebox.askyesno("새 배치 준비", "같은 YAML 목록으로 새 배치를 준비할까요? YAML은 복사하지 않고 기존 결과와 입력 사본은 보존합니다."):
                return
            batch_queue['run_directory'] = None
            batch.write_queue(batch_queue)
            refresh_batch()
        except Exception as error:
            show_error("새 실행 복사 실패", error)

    def open_local_result(path):
        try:
            path = resolve_path(path)
            if not path.exists():
                raise ValueError("아직 생성되지 않았습니다: {}".format(path))
            subprocess.Popen(["xdg-open", str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except (ValueError, OSError) as error:
            show_error("파일 열기 실패", error)

    def open_batch_result(csv=False):
        directory = batch_queue.get("run_directory")
        if not directory:
            messagebox.showinfo("결과 없음", "배치를 실행하면 결과 폴더가 생성됩니다")
            return
        open_local_result(resolve_path(directory) / "summary.csv" if csv else directory)

    batch_menu = ttk.Menubutton(batch_bar, text='목록 관리 ▾')
    menu = tk.Menu(batch_menu, tearoff=False)
    for label, action in (('선택 YAML 제거', lambda: edit_batch('remove')), ('위로', lambda: edit_batch('up')),
                          ('아래로', lambda: edit_batch('down')), ('목록 초기화', lambda: edit_batch('reset')),
                          ('이전 목록 열기', restore_batch), ('같은 목록 새로 준비', clone_batch)):
        menu.add_command(label=label, command=action)
    batch_menu.configure(menu=menu)
    batch_menu.grid(row=1, column=0, padx=3, pady=3, sticky='ew')
    batch_buttons.append(batch_menu)
    for index, (label, action) in enumerate((("현재 YAML 추가", lambda: edit_batch("current")), ("YAML 여러 개 추가", lambda: edit_batch("files")),
                          ("전체 검사", check_batch),
                          ("새 배치 시작", lambda: start_batch(new=True)), ("배치 재개", start_batch),
                          ("배치 결과 폴더", open_batch_result), ("배치 CSV", lambda: open_batch_result(csv=True)))):
        button = ttk.Button(batch_bar, text=label, command=action)
        button.grid(row=0 if index < 5 else 1, column=index if index < 5 else index - 4,
                    padx=3, pady=3, sticky="ew")
        if index < 5:
            batch_buttons.append(button)
    def open_batch_item(_event):
        selected = batch_table.selection()
        if selected:
            try:
                path = resolve_path(batch_queue["items"][int(selected[0])])
                mission = mission_directory(path)
                if mission is None or mission.name not in available_missions():
                    raise ValueError('이전 외부 YAML은 미션 workspace로 옮긴 뒤 선택하세요. 기존 배치 재개는 그대로 가능합니다.')
                read_source(path)
                notebook.select(basic)
            except Exception as error:
                show_error("YAML 열기 실패", error)
    batch_table.bind("<Double-1>", open_batch_item)
    refresh_batch()

    history_tab = ttk.Frame(notebook, padding=8)
    notebook.add(history_tab, text='실험 기록 / 재개')
    ttk.Label(history_tab, text='새 실행은 현재 YAML, 재개는 기록에 보관된 설정·시나리오를 사용합니다. 원본을 수정해도 기록은 유지됩니다.',
              wraplength=900).pack(anchor='w')
    history_table = ttk.Treeview(history_tab, columns=('name', 'time', 'state', 'progress', 'counts'),
                                show='headings', height=6)
    for key, label, width in (('name', '실험 이름', 230), ('time', '시작 시간 (PC 현지)', 200),
                              ('state', '상태', 110), ('progress', '완료/전체', 80), ('counts', 'P/F/E', 80)):
        history_table.heading(key, text=label)
        history_table.column(key, width=width)
    history_table.pack(fill='x', pady=5)

    def refresh_history():
        document = active.get('template') or {}
        root_path = active.get('history_root') or resolve_path(document.get('output_root', 'planner_logs/validation'))
        rows = executions.history(root_path)
        old_selection = history_table.selection()
        for iid in history_table.get_children():
            history_table.delete(iid)
        for row in rows:
            counts = row['counts']
            label = {'READY': '준비', 'RUNNING': '진행 중', 'DONE': '완료', 'INTERRUPTED': '중단/미완료',
                     'DAMAGED': '기록 손상'}.get(row['status'], row['status'])
            history_table.insert('', 'end', iid=row['directory'], values=(row['name'], executions.local_time(row['created_at']),
                label + (' (이전)' if row['legacy'] else ''), '{}/{}'.format(row['completed'], row['total']),
                '{}/{}/{}'.format(counts.get('PASS', 0), counts.get('FAIL', 0), counts.get('ERROR', 0))))
        for iid in old_selection:
            if history_table.exists(iid):
                history_table.selection_set(iid)
        return rows

    def select_execution(directory):
        if any(active.get(key) and active[key].poll() is None for key in ('process', 'check_process')):
            raise ValueError('현재 실행/검사가 끝난 뒤 이전 실험을 선택하세요')
        cfg = executions.load_execution(directory, check_install=False)
        active.update(execution=cfg, mode='single')
        update_progress()
        status.set('실험 기록 선택: {} — 재개 시 현재 편집값 대신 보관된 입력을 사용합니다'.format(cfg['output'].name))
        return cfg

    def resume_selected():
        try:
            selected = history_table.selection()
            if not selected:
                raise ValueError('재개할 실험 기록을 선택하세요')
            start(execution=selected[0])
        except (ValueError, OSError) as error:
            show_error('실험 재개 실패', error)

    def browse_execution():
        path = filedialog.askopenfilename(initialdir=str(ROOT / 'planner_logs/validation'),
            filetypes=[('실험 기록', 'execution.json campaign_state.json')])
        if path:
            try:
                cfg = select_execution(Path(path).parent)
                # Keep the authoring fields untouched; only change the history root.
                active['history_root'] = cfg['output'].parent
                refresh_history()
                history_table.selection_set(str(cfg['output']))
            except (ValueError, OSError) as error:
                show_error('기록 열기 실패', error)

    history_bar = ttk.Frame(history_tab)
    history_bar.pack(fill='x')
    for label, action in (('새로고침', refresh_history), ('선택 실험 재개', resume_selected),
                          ('다른 폴더 기록 열기', browse_execution)):
        button = ttk.Button(history_bar, text=label, command=action)
        button.pack(side='left', padx=3)
        batch_buttons.append(button)
    def on_history_select(_event):
        if any(active.get(key) and active[key].poll() is None for key in ('process', 'check_process')):
            return
        selected = history_table.selection()
        if selected:
            try:
                select_execution(selected[0])
            except (ValueError, OSError) as error:
                show_error('기록 읽기 실패', error)
    history_table.bind('<<TreeviewSelect>>', on_history_select)

    def open_results():
        if active.get("mode") == "batch" and batch_queue.get("run_directory"):
            subprocess.Popen(["xdg-open", batch_queue["run_directory"]], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return
        document = active.get("document")
        if not document:
            try:
                document, _ = collect()
            except Exception as error:
                show_error("결과 폴더 열기 실패", error)
                return
        path = active['execution']['output'] if active.get('execution') else campaign_output(document)
        if path.is_dir():
            subprocess.Popen(["xdg-open", str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            messagebox.showinfo("결과 없음", "아직 생성되지 않았습니다: {}".format(path))

    controls = ttk.Frame(monitor_panel)
    controls.pack(fill="x", pady=5)
    control_buttons = {}
    primary_controls = ttk.Frame(controls)
    primary_controls.pack(side='left')
    auxiliary_controls = ttk.Frame(controls)
    auxiliary_controls.pack(side='right')
    def save_clicked():
        try:
            save()
        except Exception as error:
            show_error('YAML 저장 실패', error)

    for label, text, action, background, foreground, hover in (
            ('YAML 저장', 'YAML 저장', save_clicked, '#e5ecf3', '#344b62', '#d7e2ed'),
            ('불러오기/새로고침', '불러오기/새로고침', lambda: select_source(None), '#edf0f3', '#384b5c', '#e0e5eb'),
            ('새 실험 시작', '▶ 새 실험 시작', start, '#e2eee6', '#304c3b', '#d2e3d8')):
        button = tk.Button(primary_controls, text=text, command=action,
                           background=background, foreground=foreground, activebackground=hover,
                           activeforeground=foreground, disabledforeground='#8c939a',
                           font=('TkDefaultFont', 11, 'bold'), padx=12, pady=4,
                           relief='flat', borderwidth=0, highlightthickness=1,
                           highlightbackground='#b9c4ce')
        button.pack(side='left', padx=(0, 6))
        control_buttons[label] = button
    save_button = control_buttons['YAML 저장']
    load_button = control_buttons['불러오기/새로고침']
    auxiliary_button = ttk.Menubutton(auxiliary_controls, text='보조 기능 ▾')
    auxiliary_menu = tk.Menu(auxiliary_button, tearoff=False)
    auxiliary_menu.add_command(label='설정 검사', command=check)
    auxiliary_menu.add_command(label='실험 기록', command=lambda: (refresh_history(), notebook.select(history_tab)))
    auxiliary_menu.add_separator()
    auxiliary_menu.add_command(label='결과 폴더', command=open_results)
    auxiliary_button.configure(menu=auxiliary_menu)
    auxiliary_button.pack(side='left', padx=4)
    stop_button = ttk.Button(auxiliary_controls, text='중단', command=stop)
    stop_button.pack(side='left', padx=4)
    control_buttons['중단'] = stop_button
    rerun_checkbox = ttk.Checkbutton(controls, text="재개 시 FAIL도 재실행", variable=rerun_failed)
    rerun_checkbox.pack_forget()
    ttk.Checkbutton(history_bar, text='FAIL도 다시 시험', variable=rerun_failed).pack(side='left', padx=8)
    compact_button = ttk.Button(auxiliary_controls, text="모니터 모드",
                                command=lambda: toggle_compact_mode())
    compact_button.pack(side="right", padx=4)

    def set_running_ui(running, *, allow_stop=True):
        active['ui_running'] = running
        source_box.configure(state="disabled" if running else "readonly")
        mission_box.configure(state="disabled" if running else "readonly")
        copy_button.configure(state="disabled" if running else "normal")
        for button in (load_button, edit_button, folder_button):
            button.configure(state='disabled' if running else 'normal')
        for button in batch_buttons:
            button.configure(state="disabled" if running else ('readonly' if isinstance(button, ttk.Combobox) else "normal"))
        for key in list(EDITABLE_FIELDS) + ['recording_mode']:
            if key not in field_widgets:
                continue
            widget = field_widgets[key]
            widget.configure(state='disabled' if running else ('readonly' if isinstance(widget, ttk.Combobox) else 'normal'))
        update_recording_widgets()
        for label in ('YAML 저장', '불러오기/새로고침', "새 실험 시작"):
            control_buttons[label].configure(state="disabled" if running else "normal")
        for index in (0, 1):
            auxiliary_menu.entryconfigure(index, state='disabled' if running else 'normal')
        control_buttons["중단"].configure(state="normal" if running and allow_stop else "disabled")

    set_running_ui(False)

    columns = ("scenario", "status", "reason", "category", "stage")
    table = ttk.Treeview(frame, columns=columns, show="headings", height=8)
    for key, title, width in (("scenario", "시나리오", 340), ("status", "결과", 70),
                              ("reason", "판정", 140), ("category", "관측 실패 유형", 150),
                              ("stage", "조사 우선 단계", 170)):
        table.heading(key, text=title)
        table.column(key, width=width, anchor="w")
    table.pack(fill="both", expand=True, pady=4)

    def show_detail(_event):
        selected = table.selection()
        if not selected:
            return
        path = result_paths.get(selected[0])
        if not path:
            return
        try:
            result = json.loads(path.read_text(encoding="utf-8"))
            analysis = result.get("failure_analysis", {})
            details = [analysis.get("summary", "분석 없음"), "", "관측 근거:"]
            details.extend("- " + item for item in analysis.get("evidence", []))
            details += ["", "의심 단계: {} (확신도 {})".format(
                analysis.get("suspected_stage", "?"), analysis.get("stage_confidence", "?")),
                "근본 원인 확정: 아니오", "", str(path)]
            messagebox.showinfo("실패 근거 — {}".format(result.get("scenario", "")), "\n".join(details))
        except (OSError, ValueError) as error:
            messagebox.showerror("결과 읽기 실패", str(error))

    table.bind("<Double-1>", show_detail)
    detail_note = ttk.Label(frame, text="실패 장면을 더블클릭하면 관측 수치와 의심 단계를 볼 수 있습니다. 원인 확정은 아닙니다.")
    detail_note.pack(anchor="w")
    log = tk.Text(frame, height=9, wrap="word", state="disabled")
    log.pack(fill="both", expand=True)

    def set_compact_mode(enabled, bottom_bar_px="auto"):
        if enabled == compact["enabled"]:
            return
        if enabled:
            bottom_bar_px = normalize_bar_setting(bottom_bar_px)
        compact["enabled"] = enabled
        if enabled:
            root.update_idletasks()
            compact["previous_geometry"] = root.geometry()
            compact["previous_zoomed"] = bool(root.attributes("-zoomed"))
            compact["previous_fullscreen"] = bool(root.attributes("-fullscreen"))
            for widget in (top, notebook, table, detail_note, log):
                widget.pack_forget()
            area_x, area_y = 0, 0
            area_width, area_height = root.winfo_screenwidth(), root.winfo_screenheight()
            try:
                import gi
                gi.require_version("Gdk", "3.0")
                from gi.repository import Gdk
                gdk_display = Gdk.Display.get_default()
                monitor = gdk_display.get_monitor_at_point(
                    root.winfo_rootx() + root.winfo_width() // 2,
                    root.winfo_rooty() + root.winfo_height() // 2)
                workarea = monitor.get_workarea()
                area_x, area_y = workarea.x, workarea.y
                area_width, area_height = workarea.width, workarea.height
            except (ImportError, ValueError, AttributeError):
                pass
            bar = resolve_bar_height(area_height, bottom_bar_px)
            compact_width = area_width // 2
            primary_controls.pack_forget()
            rerun_checkbox.pack_forget()
            # Keep the return button above status text, even if a long status
            # would otherwise consume the entire fixed-height monitor bar.
            controls.pack_forget()
            controls.pack(fill="x", pady=(0, 2), before=status_label)
            char_width = max(1, tkfont.nametofont("TkDefaultFont").measure("M"))
            label_chars = max(20, (compact_width - 50) // char_width)
            status_label.configure(wraplength=0, width=label_chars)
            live_label.configure(wraplength=0, width=label_chars)
            root.minsize(min(850, compact_width), 90)
            # Tk's -zoomed request is asynchronous on X11. A geometry request
            # made before the WM acknowledges unmaximize is silently ignored.
            root.attributes("-fullscreen", False)
            root.attributes("-zoomed", False)
            root.state("normal")

            def place_after_unmaximize(remaining=12):
                if not compact["enabled"]:
                    return
                if (root.attributes("-zoomed") or root.attributes("-fullscreen")) and remaining:
                    root.attributes("-fullscreen", False)
                    root.attributes("-zoomed", False)
                    root.after(100, place_after_unmaximize, remaining - 1)
                    return
                # First place via Tk using the *measured* frame offset. Wnck
                # then sets the exact outer rectangle, including decorations.
                root.update_idletasks()
                frame_top = max(0, root.winfo_rooty() - root.winfo_y())
                root.geometry("{}x{}+{}+{}".format(
                    compact_width, max(90, bar - frame_top),
                    area_x, area_y + area_height - bar))

                def place_outer_window():
                    if not compact["enabled"]:
                        return
                    command = ["/usr/bin/python3", str(PACKAGE / "scripts/place_monitor_bar.py"),
                               root.title(), str(area_x), str(area_y + area_height - bar),
                               str(compact_width), str(bar)]
                    try:
                        subprocess.run(command, capture_output=True, timeout=3, check=False)
                    except (OSError, subprocess.TimeoutExpired):
                        pass  # Tk's measured-frame placement remains usable.

                root.after(150, place_outer_window)

            root.after(100, place_after_unmaximize)
            root.attributes("-topmost", True)
            compact_button.configure(text="전체 설정 보기")
        else:
            root.attributes("-topmost", False)
            primary_controls.pack(side='left', before=auxiliary_controls)
            controls.pack_forget()
            controls.pack(fill="x", pady=5)
            status_label.configure(wraplength=1840, width=0)
            live_label.configure(wraplength=1840, width=0)
            top.pack(fill="x")
            notebook.pack(fill="x", pady=8)
            table.pack(fill="both", expand=True, pady=4)
            detail_note.pack(anchor="w")
            log.pack(fill="both", expand=True)
            root.minsize(850, 650)
            root.geometry(compact["previous_geometry"] or "1010x810")
            if compact["previous_fullscreen"]:
                root.after(100, lambda: root.attributes("-fullscreen", True))
            elif compact["previous_zoomed"]:
                root.after(100, lambda: root.attributes("-zoomed", True))
            compact_button.configure(text="모니터 모드")

    def toggle_compact_mode():
        if compact["enabled"]:
            set_compact_mode(False)
            return
        running = active["process"] and active["process"].poll() is None
        if running and (active.get("batch_document") or active.get("document") or active["template"])["recording"]["video"].get("window_layout") != "split":
            messagebox.showwarning(
                "현재 캠페인은 전체화면 설정",
                "현재 실행 중인 YAML은 manual 배치라 장면마다 RViz를 최대화합니다. "
                "이 실험을 중단하고 원본 YAML을 split 배치로 설정한 뒤 새 실험을 시작해야 지속적으로 반반 배치됩니다.")
        try:
            set_compact_mode(True, vars_["bottom_bar_px"].get())
        except ValueError as error:
            messagebox.showerror("모니터 바 높이", str(error))

    def append(line):
        log.configure(state="normal")
        log.insert("end", line + "\n")
        if int(log.index("end-1c").split(".")[0]) > 500:
            log.delete("1.0", "100.0")
        log.see("end")
        log.configure(state="disabled")

    def update_progress():
        try:
            document = (load_document(active['execution']['path']) if active.get('execution')
                        else active.get("document"))
        except (OSError, ValueError):
            status.set('실험 사본을 읽을 수 없습니다. 기록 폴더와 로그를 확인하세요.')
            return None
        prefix = (active['execution']['output'].name + ' | ') if active.get('execution') else ''
        batch_config = None
        if active.get("mode") == "batch":
            state = refresh_batch()
            if state:
                index = state.get("current_index")
                if index is not None:
                    item = state["items"][index]
                    prefix = "배치 {}/{} · {} | ".format(index + 1, len(state["items"]), item["campaign"])
                    batch_config = resolve_path(item.get('execution_path', item["path"]))
                    try:
                        document = load_document(batch_config)
                        active["batch_document"] = document
                    except (OSError, ValueError):
                        return None
                if state["status"] != "RUNNING":
                    done = sum(row["status"] == "DONE" for row in state["items"])
                    totals = {key: sum(row.get("counts", {}).get(key, 0) for row in state["items"]) for key in ("PASS", "FAIL", "ERROR")}
                    status.set(active.get("batch_error") or "배치 {} · YAML {}/{} · 성공 {} 실패 {} 오류 {}".format(
                        state["status"], done, len(state["items"]), totals["PASS"], totals["FAIL"], totals["ERROR"]))
            if document is None:
                return None
        if not document:
            return None
        path = campaign_output(document) / "campaign_state.json"
        try:
            state = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None
        config_path = batch_config or (active['execution']['path'] if active.get('execution') else active.get("config"))
        if config_path and state.get("config_path") != str(config_path.resolve()):
            return None
        results = state.get("results", {})
        render_result_table(table, state, result_paths)
        counts = result_counts(state)
        current = state.get("current")
        if isinstance(current, dict):
            current = "{} / {}".format(current.get("scenario", "?"), current.get("phase", "?"))
        current = current or "대기"
        if active["process"] and active["process"].poll() is None:
            upload_note = " | Drive 오류 {}건".format(len(state["upload_errors"])) if state.get("upload_errors") else ""
            status.set(prefix + "장면 {}/{} | 성공 {} 실패 {} 오류 {} | 현재 {}{}".format(
                len(results), state.get("total", "?"), counts["PASS"], counts["FAIL"], counts["ERROR"],
                current, upload_note))
            current_state = state.get("current")
            metrics = (current_state.get("metrics") or {}) if isinstance(current_state, dict) else {}
            if "elapsed_sec" in metrics:
                live_status.set("실시간 주행 {:.1f}초 | 목적지까지 {:.1f}m | 속도 {:.1f}m/s | 충돌 {}회".format(
                    metrics.get("elapsed_sec", 0.0), metrics.get("goal_distance_m", 0.0),
                    metrics.get("speed_mps", 0.0), metrics.get("collision_count", 0)))
            else:
                live_status.set("현재 단계: {}".format(current))
        else:
            live_status.set("")
        return state

    def poll():
        while True:
            try:
                event = events.get_nowait()
            except queue.Empty:
                break
            if event[0] == "line":
                append(event[1])
            elif event[0] == "done":
                label, code = event[1:]
                append("{} 종료 코드: {}".format(label, code))
                if label in ("run", "batch"):
                    active["process"] = None
                    if label == "batch" and code not in (0, 130):
                        active["batch_error"] = "배치 실행 오류 (코드 {}) — 입력 변경·환경 오류를 로그에서 확인하세요. 이전 결과는 보존됩니다.".format(code)
                        status.set(active["batch_error"])
                    set_running_ui(False)
                    root.deiconify()
                    root.lift()
                    state = update_progress()
                    if label == "run":
                        status.set(completion_label(code, state))
                    refresh_history()
                else:
                    active["check_process"] = None
                    set_running_ui(False)
                    status.set(("환경 검사 FAIL 없음 — WARN·연결 상태도 로그에서 확인하세요" if code == 0 else "환경 검사 실패 — 로그의 해결 방법을 확인하세요")
                               if label == "environment" else "설정 검사 {}".format("통과" if code == 0 else "실패 (로그 확인)"))
        update_progress()
        poll_after["id"] = root.after(500, poll)

    def on_destroy(event):
        if event.widget is root:
            active["closed"] = True
            # Delayed geometry/interrupt callbacks must not run after this Tk
            # instance closes (including repeated Xvfb GUI tests).
            for identifier in root.tk.call("after", "info"):
                try:
                    root.after_cancel(identifier)
                except tk.TclError:
                    pass
            poll_after["id"] = None

    def on_close():
        if active["check_process"] and active["check_process"].poll() is None:
            messagebox.showinfo("설정 검사 중", "설정 검사가 끝난 후 창을 닫아 주세요.")
            return
        if active["process"] and active["process"].poll() is None:
            messagebox.showinfo("실행 중", "먼저 중단 버튼을 눌러 ROS와 녹화를 정리하세요.")
            return
        root.destroy()

    def on_interrupt(_signum, _frame):
        # Terminal Ctrl+C must stop the child runner so rosbag and MP4 finalize.
        if active["process"] and active["process"].poll() is None:
            print("안전 중단 요청: 녹화 마감과 ROS 정리를 기다립니다", flush=True)
            stop(force=True)
        else:
            root.after(0, root.destroy)

    previous_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, on_interrupt)
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.bind("<Destroy>", on_destroy)
    read_source(initial, fresh_name=restored is None)
    refresh_history()
    if on_ready is not None:
        on_ready({"root": root, "fields": vars_, "actions": {
            "save": save, "check": check, "start": start, "stop": stop,
            'set_running_ui': set_running_ui,
            'select_scenarios': select_scenarios,
            'select_bag_topics': select_bag_topics,
            "environment_check": check_environment,
            "copy_workspace": copy_workspace, "open_yaml": read_source,
            'select_mission': select_mission, 'select_yaml': lambda: select_source(True),
            "reload_yaml": lambda: select_source(None),
            "quick_start": start_quick, "quick_refresh": refresh_quick_choices,
            "open_personal_folder": browse_personal_folder,
            'refresh_history': refresh_history, 'select_execution': select_execution,
            'resume_execution': lambda directory: start(execution=directory),
            "batch_edit": edit_batch, "batch_check": check_batch, "batch_start": start_batch,
            "batch_refresh": refresh_batch,
            "batch_clone": clone_batch, "batch_restore": restore_batch,
            "update_progress": update_progress, "compact": set_compact_mode}, "active": active,
            "status": status, "table": table, "log": log, "batch_table": batch_table,
            "field_widgets": field_widgets, "source_entry": source_box, 'save_button': save_button,
            'topics_button': topics_button, 'recording_summary': recording_summary,
            'mission_box': mission_box, 'mission_value': mission_value, 'yaml_value': yaml_value,
            'picker_choices': picker_choices,
            'start_button': control_buttons['새 실험 시작'], 'reload_button': load_button,
            'primary_controls': primary_controls, 'auxiliary_menu': auxiliary_menu,
            'auxiliary_button': auxiliary_button, 'stop_button': stop_button,
            "quick_table": quick_table, "quick_fields": quick_vars,
            'history_table': history_table,
            "batch_queue": batch_queue, "batch_note": batch_note,
            "live_status": live_status, "rerun_failed": rerun_failed,
            "compact_state": compact, "compact_button": compact_button})
    try:
        if not active.get("closed"):
            poll_after["id"] = root.after(500, poll)
            root.mainloop()
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
    return 0
