"""Small Tk frontend for the existing YAML/CLI campaign runner.

No ROS node or decision logic lives here. The form edits a personal YAML, then
starts the same `planning_validator` command used by terminal operators.
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
from datetime import datetime
from pathlib import Path

import yaml

from .config import (CONFIG_SCHEMA_VERSION, DEFAULT_UPLOAD_REMOTE, EXECUTION_CONTRACT, ROOT,
                     UPLOAD_PROFILES, resolve_path, validate_campaign_document)
from .config import load_campaign
from .layout import normalize_bar_setting, resolve_bar_height
from .traffic_light import DEFAULT_PYTHON, LABEL_MODES, MODE_LABELS, runtime_mode, runtime_settings
from . import batch


PACKAGE = ROOT / "base_ws/src/planning_addons/planning_scenario_validator"
if not PACKAGE.is_dir():
    PACKAGE = ROOT / "base_ws/install_aw149/share/planning_scenario_validator"
EXAMPLES = PACKAGE / "config/missions/static_obstacle/base"
MISSIONS = PACKAGE / "config/missions"
USERS = PACKAGE / "config/users"
NAME_RE = re.compile(r"[a-zA-Z0-9][a-zA-Z0-9_-]{1,59}\Z")
UPLOAD_PROFILE_LABELS = {
    "basic": "기본 공유 — CSV·JSON",
    "bag": "bag 공유 — 기본 + rosbag",
    "video": "영상 공유 — 기본 + MP4",
    "full": "원본 공유 — 기본 + rosbag + MP4",
}
UPLOAD_LABEL_PROFILES = {label: key for key, label in UPLOAD_PROFILE_LABELS.items()}


def default_name(template):
    path = Path(template)
    mission = mission_directory(path)
    stem = mission.name + "_starter" if path.stem == "example" and mission else path.stem
    base = re.sub(r"[^a-zA-Z0-9_-]", "_", stem)[:35]
    return "{}_{}".format(base, datetime.now().strftime("%y%m%d_%H%M%S"))


def campaign_examples():
    # Do not expose scenario .case.yaml files as executable campaigns.
    return sorted(MISSIONS.glob("*/base/*.yaml"))


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
        "timeout_sec": case.get("timeout_sec", 180),
        "pose_source": stack.get("pose_source", "localization"),
        "detector": stack.get("detector", "od_gt"),
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
    return is_within(path, USERS) or bool(mission and is_within(path, mission / "workspace"))


def personal_path(document, user=None, source=None):
    mission = mission_directory(source)
    if mission:
        return mission / "workspace" / (document["campaign_name"] + ".yaml")
    username = re.sub(r"[^a-zA-Z0-9_-]", "_", user or getpass.getuser())
    return USERS / username / (document["campaign_name"] + ".yaml")


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
    document = load_document(base / "example.yaml")
    name = default_name(base / "example.yaml")
    initial = name
    index = 2
    while True:
        document["campaign_name"] = name
        path = personal_path(document, source=source)
        if not path.exists() and not campaign_output(document).exists():
            break
        name = "{}_{}".format(initial[:55], index)
        index += 1
    document["scenarios"]["directory"] = str((workspace / "scenarios").relative_to(ROOT))
    save_personal(document, path)
    return path


def save_personal(document, path):
    """Never mutate a campaign's YAML once its state exists; resume needs same hash."""
    path = Path(path)
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
    header = ("# GUI에서 저장한 개인 캠페인입니다. 이 파일은 Git에 포함되지 않습니다.\n"
              "# 이 YAML의 목적지·주행 제한시간은 선택한 모든 시나리오에 동일하게 적용됩니다.\n"
              "# 목적지나 실행 조건이 달라지면 새 이름의 YAML을 만드세요.\n"
              "# 실행 후에는 resume 해시 보호를 위해 수정하지 말고 새 campaign_name을 쓰세요.\n")
    temporary = path.with_name(path.name + ".tmp.{}".format(os.getpid()))
    try:
        temporary.write_text(header + yaml.safe_dump(document, allow_unicode=True, sort_keys=False),
                             encoding="utf-8")
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
    """CLI exit 0 can include legitimate FAIL cases: never call that a pass."""
    if state is None:
        return "실행 종료 (코드 {}). 결과 파일 없음 — 로그를 확인하세요.".format(code)
    counts = result_counts(state)
    total = state.get("total", "?")
    completed = len(state.get("results", {}))
    label = "검증 종료 {}/{} | 성공 {} / 주행 실패 {} / 시스템 오류 {} | 종료 코드 {}".format(
        completed, total, counts["PASS"], counts["FAIL"], counts["ERROR"], code)
    if state.get("upload_errors"):
        label += " | Drive 업로드 오류 {}건 (로컬 결과 보존)".format(len(state["upload_errors"]))
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
    # Start first-time operators with the short, previously validated demo.
    # The five-case stress campaign is available from the same dropdown.
    preferred = EXAMPLES / "static_pair_mission_gt_split.yaml"
    restored = remembered_yaml()
    initial = restored or (preferred if preferred.exists() else examples[0])
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
    status = tk.StringVar(value="준비됨 — MORAI GUI에서 Sensor/Ego Network를 먼저 Load·Connect하세요.")
    live_status = tk.StringVar(value="")
    vars_ = {}
    active = {"process": None, "check_process": None, "config": None,
              "document": None, "stopping": False, "mode": "single", "batch_document": None}
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
        path = resolve_path(path)
        document = load_document(path)
        values = form_values(document)
        if fresh_name:
            values["campaign_name"] = default_name(path)
        active["template"] = document
        active["source_path"] = Path(path)
        active["config"] = Path(path) if is_personal(path) else None
        active["document"] = document if active["config"] else None
        active["mode"] = "single"
        result_paths.clear()
        for item in table.get_children():
            table.delete(item)
        live_status.set("")
        source.set(str(path))
        for key, value in values.items():
            vars_[key].set(value)
        status.set("설정 로드: {} — {}".format(Path(path).name,
                   "이전 설정: 저장하면 공통 설정 v2로 전환. 기존 실행은 새 캠페인 이름을 사용하세요."
                   if document.get("schema_version") == 1 else "목적지·제한시간은 모든 장면에 공통 적용"))
        remember_yaml(path)

    top = ttk.LabelFrame(frame, text="설정 파일 (미션 base → workspace / 기존 개인 YAML도 사용 가능)", padding=8)
    top.pack(fill="x")
    choices = [str(p) for p in examples]
    source_box = ttk.Combobox(top, textvariable=source, values=choices, state="readonly", width=100)
    source_box.grid(row=0, column=0, sticky="ew", padx=4)
    def select_source(_event):
        try:
            read_source(source.get(), fresh_name=True)
        except Exception as error:
            source.set(str(active.get("source_path", "")))
            messagebox.showerror("설정 로드 실패", str(error))
    source_box.bind("<<ComboboxSelected>>", select_source)

    def choose_personal():
        mission = mission_directory(active.get("source_path"))
        directory = mission / "workspace" if mission else USERS
        path = filedialog.askopenfilename(initialdir=str(directory), filetypes=[("YAML", "*.yaml *.yml")])
        if path:
            try:
                read_source(path)
            except Exception as error:
                messagebox.showerror("설정 로드 실패", str(error))

    personal_button = ttk.Button(top, text="내 YAML 열기 / resume", command=choose_personal)
    personal_button.grid(row=0, column=1, padx=4)

    def copy_workspace():
        try:
            if any(active.get(key) and active[key].poll() is None for key in ("process", "check_process")):
                raise ValueError("실행/검사 중에는 작업 폴더를 준비할 수 없습니다")
            path = copy_base_to_workspace(active.get("source_path"))
            read_source(path)
            status.set("workspace 준비 완료 (기존 시나리오 유지): {}".format(path))
        except Exception as error:
            messagebox.showerror("베이스 복사 실패", str(error))

    copy_button = ttk.Button(top, text="베이스 → 내 작업 폴더 복사", command=copy_workspace)
    copy_button.grid(row=1, column=1, padx=4)
    top.columnconfigure(0, weight=1)
    ttk.Label(top, text="처음 연결은 MORAI MR GUI에서 직접 합니다. 시나리오 전환만 자동입니다.", foreground="#a44").grid(row=1, column=0, sticky="w", pady=5)

    notebook = ttk.Notebook(frame)
    notebook.pack(fill="x", pady=8)
    basic = ttk.Frame(notebook, padding=10)
    advanced = ttk.Frame(notebook, padding=10)
    traffic_tab = ttk.Frame(notebook, padding=10)
    notebook.add(basic, text="기본 설정")
    notebook.add(advanced, text="고급 / 네트워크")
    notebook.add(traffic_tab, text="신호등 환경")

    def add_field(parent, row, key, label, *, choices=None, browse=False, boolean=False):
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
        if browse:
            def pick():
                selected = filedialog.askdirectory() if browse == "dir" else filedialog.askopenfilename()
                if selected:
                    var.set(selected)
            ttk.Button(parent, text="찾기", command=pick).grid(row=row, column=2, padx=5)
        parent.columnconfigure(1, weight=1)
        return widget

    add_field(basic, 0, "campaign_name", "캠페인 이름")
    add_field(basic, 1, "directory", "시나리오 JSON 폴더", browse="dir")
    add_field(basic, 2, "glob", "파일 선택 패턴")
    for row, (key, label) in enumerate((("x", "목적지 map X (m)"), ("y", "목적지 map Y (m)"),
                                        ("z", "목적지 map Z (m)"), ("yaw_deg", "목적지 yaw (deg)")), 3):
        add_field(basic, row, "goal_" + key, label)
    add_field(basic, 7, "timeout_sec", "공통 주행 제한 (초)")
    add_field(basic, 8, "pose_source", "측위", choices=["localization", "maptf_morai"])
    add_field(basic, 9, "detector", "객체 탐지", choices=["od_gt", "bevfusion"])
    ttk.Label(basic, text="신호등 인지", width=19).grid(row=10, column=0, sticky="w", pady=3)
    traffic_row = ttk.Frame(basic)
    traffic_row.grid(row=10, column=1, sticky="ew")
    vars_["traffic_light"] = tk.BooleanVar()
    ttk.Checkbutton(traffic_row, text="on", variable=vars_["traffic_light"]).pack(side="left")
    ttk.Label(traffic_row, text="실행 환경 설정: 신호등 환경 탭").pack(side="left", padx=10)
    add_field(basic, 11, "video", "화면 영상 녹화", boolean=True)
    add_field(basic, 12, "include_sensors", "bag에 카메라/LiDAR", boolean=True)
    add_field(basic, 13, "upload_enabled", "Drive 자동 업로드", boolean=True)
    add_field(basic, 14, "upload_profile", "공유 모드", choices=list(UPLOAD_PROFILE_LABELS.values()))

    add_field(advanced, 0, "topic_profile", "rosbag 토픽 JSON", browse="file")
    add_field(advanced, 1, "sensor_json", "MORAI Sensor JSON", browse="file")
    add_field(advanced, 2, "ego_network_json", "MORAI Ego Network JSON", browse="file")
    add_field(advanced, 3, "scenario_load_host", "ScenarioLoad IP")
    add_field(advanced, 4, "goal_tolerance_m", "목적지 허용 반경 (m)")
    add_field(advanced, 5, "video_layout", "녹화 화면 배치", choices=["manual", "split"])
    add_field(advanced, 6, "bottom_bar_px", "GUI 하단 높이 (auto/px/%)")
    add_field(advanced, 7, "upload_remote", "Drive rclone 경로")
    ttk.Label(advanced, text="split: RViz 왼쪽, MORAI 오른쪽, GUI 하단. Drive 공유는 rclone remote를 각 PC에서 설정해야 합니다.").grid(row=8, column=0, columnspan=3, sticky="w", pady=8)

    traffic_mode = add_field(traffic_tab, 0, "traffic_light_runtime", "실행 환경",
                             choices=list(MODE_LABELS.values()))
    traffic_python = add_field(traffic_tab, 1, "traffic_light_python", "가상환경 Python", browse="file")
    add_field(traffic_tab, 2, "traffic_light_device", "신호등 장치 (0/cpu)")
    ttk.Label(traffic_tab, text="시스템 선택: /usr/bin/python3 (기존 환경 유지). 가상환경 선택: 위 실행 파일 사용.\n"
              "GPU 번호 0이 기본값입니다. CPU 시험은 cpu로 입력하세요. 신호등 off이면 환경이 없어도 실행됩니다.",
              wraplength=750).grid(row=3, column=0, columnspan=3, sticky="w", pady=8)

    def copy_traffic_install():
        root.clipboard_clear()
        root.clipboard_append("cd ~/AjouNice2026\nbash scripts/runtime/setup_traffic_light_env.sh")
        status.set("공용 가상환경 설치 명령을 복사했습니다. 별도 터미널에 붙여넣으세요. 설치 후 가상환경을 선택하세요.")

    ttk.Button(traffic_tab, text="공용 가상환경 설치 명령 복사", command=copy_traffic_install).grid(
        row=4, column=0, columnspan=3, sticky="w", pady=8)
    ttk.Label(traffic_tab, text="설치는 최초 1회입니다. Conda activate는 필요 없습니다.\n"
              "실행 버튼을 누르면 선택한 Python으로 모델/GPU/ROS import를 먼저 검사합니다.\n"
              "환경 변경 후 이미 실행한 캠페인은 새 이름으로 저장하세요.", wraplength=750).grid(
        row=5, column=0, columnspan=3, sticky="w", pady=8)

    def update_traffic_widgets(*_):
        enabled = vars_["traffic_light"].get()
        traffic_mode.configure(state="readonly" if enabled else "disabled")
        traffic_python.configure(state="normal" if enabled and vars_["traffic_light_runtime"].get() == MODE_LABELS["virtualenv"] else "disabled")

    vars_["traffic_light"].trace_add("write", update_traffic_widgets)
    vars_["traffic_light_runtime"].trace_add("write", update_traffic_widgets)

    def collect():
        values = {key: var.get() for key, var in vars_.items()}
        document = build_document(active["template"], values)
        path = active["source_path"]
        if not is_personal(path):
            path = personal_path(document, source=path)
        elif path.stem != document["campaign_name"]:
            path = personal_path(document, source=path)
        return document, path

    def save():
        if active["process"] and active["process"].poll() is None:
            raise ValueError("캠페인 실행 중에는 YAML을 변경할 수 없습니다. 중단 후 새 이름으로 저장하세요")
        document, path = collect()
        try:
            save_personal(document, path)
        except ValueError:
            if not (campaign_output(document) / "campaign_state.json").is_file():
                raise
            if not messagebox.askyesno(
                    "새 캠페인으로 복사",
                    "이 캠페인은 이미 실행돼 설정을 바꾸면 기존 결과와 resume할 수 없습니다.\n"
                    "기존 기록은 보존하고 변경된 설정을 새 캠페인 이름으로 저장할까요?"):
                raise ValueError("기존 캠페인 설정은 변경하지 않았습니다. 그대로 재개하거나 새 이름을 입력하세요")
            fresh = default_name(document["campaign_name"])
            base = fresh
            suffix = 2
            while (personal_path({"campaign_name": fresh}, source=active["source_path"]).exists() or
                   campaign_output({**document, "campaign_name": fresh}).exists()):
                fresh = "{}_{}".format(base[:57], suffix)
                suffix += 1
            vars_["campaign_name"].set(fresh)
            document, path = collect()
            save_personal(document, path)
        active.update(config=path, document=document)
        remember_yaml(path)
        status.set("개인 YAML 저장: {}".format(path))
        return path

    def show_error(label, error):
        status.set("{}: {}".format(label, error))
        messagebox.showerror(label, str(error))

    def save_clicked():
        try:
            save()
        except Exception as error:
            show_error("저장 실패", error)

    def command_reader(process, label):
        try:
            for line in process.stdout:
                events.put(("line", line.rstrip("\n")))
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

    environment_bar = ttk.Frame(advanced)
    environment_bar.grid(row=9, column=0, columnspan=3, sticky="w", pady=8)
    for label, action in (("환경 검사", check_environment),
                          ("실제 GPU 검사", lambda: check_environment(gpu=True)),
                          ("ROS 수신 검사", lambda: check_environment(live=True))):
        button = ttk.Button(environment_bar, text=label, command=action)
        button.pack(side="left", padx=3)
        batch_buttons.append(button)

    def copy_connection_paths():
        document, _ = collect()
        root.clipboard_clear()
        root.clipboard_append("Ego Network: {}\nSensor: {}".format(
            resolve_path(document["network"]["ego_network_json"]), resolve_path(document["network"]["sensor_json"])))
        status.set("MORAI에 Load할 두 JSON 경로를 복사했습니다. 최초 연결은 MORAI GUI에서 직접 합니다.")
    ttk.Button(environment_bar, text="연결 JSON 경로 복사", command=copy_connection_paths).pack(side="left", padx=3)

    def start():
        if active["check_process"] and active["check_process"].poll() is None:
            messagebox.showinfo("설정 검사 중", "현재 설정 검사가 끝난 후 캠페인을 시작하세요.")
            return
        if active["process"] and active["process"].poll() is None:
            messagebox.showinfo("이미 실행 중", "한 번에 한 캠페인만 실행할 수 있습니다.")
            return
        if vars_["video"].get() and vars_["video_layout"].get() == "manual":
            if messagebox.askyesno(
                    "RViz 전체화면 설정 확인",
                    "현재 녹화 화면 배치가 manual이라 장면마다 RViz를 최대화합니다.\n"
                    "RViz 왼쪽·MORAI 오른쪽·GUI 하단의 split 배치로 변경할까요?"):
                vars_["video_layout"].set("split")
        try:
            path = save()
        except Exception as error:
            show_error("실행 준비 실패", error)
            return
        video_enabled = active["document"]["recording"]["video"]["enabled"]
        split = active["document"]["recording"]["video"].get("window_layout") == "split"
        note = ("전체 화면을 녹화하며 RViz·MORAI를 반반 배치하고 GUI를 하단 모니터 바로 바꿉니다. "
                "창 배치가 실패하면 수동으로 조정해 주세요. 중단은 GUI 버튼 또는 터미널 Ctrl+C입니다."
                if video_enabled and split else
                "영상은 전체 화면을 녹화합니다. 필요하면 직접 GUI 창을 최소화하고 "
                "Planviz를 전면에 두세요. 중단은 GUI 버튼이나 실행 터미널 Ctrl+C로 가능합니다."
                if video_enabled else "실행 중 GUI의 중단 버튼을 사용할 수 있습니다.")
        if not messagebox.askokcancel("캠페인 시작", "MORAI GUI에서 YAML의 Sensor/Ego Network JSON을 Load·Connect했나요?\n\n" + note):
            return
        try:
            command = [runner, str(path)]
            if rerun_failed.get():
                command.append("--rerun-failed")
            process = subprocess.Popen(command, cwd=str(ROOT),
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, bufsize=1, start_new_session=True)
        except OSError as error:
            show_error("캠페인 실행 실패", error)
            return
        active.update(process=process, stopping=False, mode="single")
        set_running_ui(True)
        if video_enabled and active["document"]["recording"]["video"].get("window_layout") == "split":
            set_compact_mode(True, active["document"]["recording"]["video"].get("bottom_bar_px", "auto"))
        status.set("주행 검증 실행 중: {}".format(active["document"]["campaign_name"]))
        threading.Thread(target=command_reader, args=(process, "run"), daemon=True).start()

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
    for key, title, width in (("name", "캠페인 YAML", 270), ("scenes", "장면", 60),
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
            batch_note.set("YAML {}개 / 장면 {}개 / 약 {:.0f}분 (제한시간+준비 추정) · 기본 녹화 약 {:.1f}GiB; 센서 포함 시 별도 | {}".format(
                len(value["items"]), total_scenes, total_seconds / 60, total_seconds / 60 * 60 / 1024,
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
                selected = filedialog.askopenfilenames(initialdir=str(MISSIONS), filetypes=[("캠페인 YAML", "*.yaml *.yml")])
                for path in selected:
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

    def start_batch():
        try:
            if any(active.get(key) and active[key].poll() is None for key in ("process", "check_process")):
                raise ValueError("현재 실행/검사가 끝난 뒤 배치를 시작하세요")
            if batch_queue.get("load_error"):
                raise ValueError(batch_queue["load_error"])
            configs, _ = batch.plan(batch_queue["items"])
            if not messagebox.askokcancel("배치 시작 / 재개", "MORAI에 이 목록의 공통 Sensor/Ego Network를 Load·Connect했나요?\n완료한 YAML은 건너뛰고 중단된 장면부터 재개합니다. 주행 FAIL은 계속, 시스템 오류는 중단합니다."):
                return
            command = ([str(batch_executable)] if batch_executable else
                       ["/usr/bin/python3", str(PACKAGE / "scripts/planning_validator_batch.py")]) + [str(batch.queue_path())]
            process = subprocess.Popen(command, cwd=str(ROOT), stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True)
            active.update(process=process, stopping=False, mode="batch", batch_document=None, batch_error=None)
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
            documents = [(path, load_document(path)) for path in batch_queue["items"]]
            if not messagebox.askyesno("새 실행으로 복사", "같은 조건의 새 개인 YAML과 campaign_name을 만들어 처음부터 실행할 목록을 준비할까요? 기존 결과는 보존됩니다."):
                return
            paths = []
            for index, (source_path, document) in enumerate(documents, 1):
                document = build_document(document, form_values(document))
                name = default_name(document["campaign_name"] + "_repeat")[:55] + "_" + str(index)
                document["campaign_name"] = name
                mission = mission_directory(source_path)
                directory = (mission or MISSIONS / "static_obstacle") / "workspace"
                path = directory / (name + ".yaml")
                suffix = 1
                while path.exists() or campaign_output(document).exists():
                    suffix += 1
                    document["campaign_name"] = name[:54] + "_" + str(suffix)
                    path = directory / (document["campaign_name"] + ".yaml")
                save_personal(document, path)
                paths.append(str(path))
            batch_queue.clear()
            batch_queue.update(schema_version=1, items=paths, run_directory=None)
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

    for index, (label, action) in enumerate((("현재 YAML 추가", lambda: edit_batch("current")), ("YAML 여러 개 추가", lambda: edit_batch("files")),
                          ("제거", lambda: edit_batch("remove")), ("↑", lambda: edit_batch("up")), ("↓", lambda: edit_batch("down")),
                          ("목록 초기화", lambda: edit_batch("reset")), ("이전 목록 열기", restore_batch),
                          ("전체 검사", check_batch), ("배치 실행 / 재개", start_batch), ("새 실행으로 복사", clone_batch),
                          ("배치 결과 폴더", open_batch_result), ("배치 CSV", lambda: open_batch_result(csv=True)),
                          ("짧은 사용법", lambda: open_local_result(ROOT / "docs/planning/SCENARIO_VALIDATOR_RUNBOOK.md")),
                          ("첫 설치 안내", lambda: open_local_result(ROOT / "docs/planning/SCENARIO_VALIDATOR_TEAM_NOTION_GUIDE.md")))):
        button = ttk.Button(batch_bar, text=label, command=action)
        button.grid(row=index // 5, column=index % 5, padx=3, pady=3, sticky="ew")
        if index < 10:
            batch_buttons.append(button)
    def open_batch_item(_event):
        selected = batch_table.selection()
        if selected:
            try:
                read_source(batch_queue["items"][int(selected[0])])
                notebook.select(basic)
            except Exception as error:
                show_error("YAML 열기 실패", error)
    batch_table.bind("<Double-1>", open_batch_item)
    refresh_batch()

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
        path = campaign_output(document)
        if path.is_dir():
            subprocess.Popen(["xdg-open", str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            messagebox.showinfo("결과 없음", "아직 생성되지 않았습니다: {}".format(path))

    controls = ttk.Frame(monitor_panel)
    controls.pack(fill="x", pady=5)
    control_buttons = {}
    for label, action in (("개인 YAML 저장", save_clicked), ("설정 검사", check),
                          ("실행 / resume", start), ("중단", stop), ("결과 폴더", open_results)):
        button = ttk.Button(controls, text=label, command=action)
        button.pack(side="left", padx=4)
        control_buttons[label] = button
    rerun_checkbox = ttk.Checkbutton(controls, text="기존 FAIL도 다시 실행", variable=rerun_failed)
    rerun_checkbox.pack(side="left", padx=12)
    compact_button = ttk.Button(controls, text="모니터 모드",
                                command=lambda: toggle_compact_mode())
    compact_button.pack(side="right", padx=4)

    def set_running_ui(running, *, allow_stop=True):
        source_box.configure(state="disabled" if running else "readonly")
        personal_button.configure(state="disabled" if running else "normal")
        copy_button.configure(state="disabled" if running else "normal")
        for button in batch_buttons:
            button.configure(state="disabled" if running else "normal")
        for label in ("개인 YAML 저장", "설정 검사", "실행 / resume"):
            control_buttons[label].configure(state="disabled" if running else "normal")
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
            for label in ("개인 YAML 저장", "설정 검사", "실행 / resume"):
                control_buttons[label].pack_forget()
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
            for label in ("개인 YAML 저장", "설정 검사", "실행 / resume"):
                control_buttons[label].pack(side="left", padx=4, before=control_buttons["중단"])
            rerun_checkbox.pack(side="left", padx=12)
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
        if not running and vars_["video"].get():
            # The operator chose the split-screen monitor layout. Persist that
            # choice in the form so the runner will not maximize RViz per case.
            vars_["video_layout"].set("split")
        elif running and (active.get("batch_document") or active.get("document") or active["template"])["recording"]["video"].get("window_layout") != "split":
            messagebox.showwarning(
                "현재 캠페인은 전체화면 설정",
                "현재 실행 중인 YAML은 manual 배치라 장면마다 RViz를 최대화합니다. "
                "이 캠페인을 중단하고 새 이름으로 split 배치를 설정해 재실행해야 지속적으로 반반 배치됩니다.")
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
        document = active.get("document")
        prefix = ""
        batch_config = None
        if active.get("mode") == "batch":
            state = refresh_batch()
            if state:
                index = state.get("current_index")
                if index is not None:
                    item = state["items"][index]
                    prefix = "배치 {}/{} · {} | ".format(index + 1, len(state["items"]), item["campaign"])
                    batch_config = resolve_path(item["path"])
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
        config_path = batch_config or active.get("config")
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
    if on_ready is not None:
        on_ready({"root": root, "fields": vars_, "actions": {
            "save": save, "check": check, "start": start, "stop": stop,
            "environment_check": check_environment,
            "copy_workspace": copy_workspace, "open_yaml": read_source,
            "batch_edit": edit_batch, "batch_check": check_batch, "batch_start": start_batch,
            "batch_refresh": refresh_batch,
            "batch_clone": clone_batch, "batch_restore": restore_batch,
            "update_progress": update_progress, "compact": set_compact_mode}, "active": active,
            "status": status, "table": table, "log": log, "batch_table": batch_table,
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
