"""Small Tk frontend for the existing YAML/CLI campaign runner.

No ROS node or decision logic lives here. The form edits a personal YAML, then
starts the same `planning_validator` command used by terminal operators.
"""
import copy
import getpass
import json
import math
import os
import queue
import re
import signal
import subprocess
import threading
from datetime import datetime
from pathlib import Path

import yaml

from .config import ROOT, resolve_path


PACKAGE = ROOT / "base_ws/src/planning_addons/planning_scenario_validator"
if not PACKAGE.is_dir():
    PACKAGE = ROOT / "base_ws/install_aw149/share/planning_scenario_validator"
EXAMPLES = PACKAGE / "config/examples"
USERS = PACKAGE / "config/users"
NAME_RE = re.compile(r"[a-zA-Z0-9][a-zA-Z0-9_-]{1,59}\Z")


def default_name(template):
    base = re.sub(r"[^a-zA-Z0-9_-]", "_", Path(template).stem)[:35]
    return "{}_{}".format(base, datetime.now().strftime("%y%m%d_%H%M%S"))


def load_document(path):
    value = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("YAML 최상위는 mapping이어야 합니다")
    return value


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
    result = {
        "campaign_name": document.get("campaign_name", ""),
        "directory": scene.get("directory", ""),
        "glob": scene.get("glob", "*.json"),
        "timeout_sec": case.get("timeout_sec", 180),
        "pose_source": stack.get("pose_source", "localization"),
        "detector": stack.get("detector", "od_gt"),
        "traffic_light": stack.get("traffic_light", False),
        "topic_profile": recording.get("topic_profile", ""),
        "include_sensors": recording.get("include_sensors", False),
        "video": recording.get("video", {}).get("enabled", False),
        "sensor_json": network.get("sensor_json", ""),
        "ego_network_json": network.get("ego_network_json", ""),
        "scenario_load_host": morai.get("scenario_load_host", "127.0.0.1"),
        "goal_tolerance_m": validation.get("goal_tolerance_m", 2.0),
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
    recording = result.setdefault("recording", {})
    recording["topic_profile"] = str(values["topic_profile"]).strip()
    recording["include_sensors"] = bool(values["include_sensors"])
    recording.setdefault("video", {})["enabled"] = bool(values["video"])
    network = result.setdefault("network", {})
    for key in ("sensor_json", "ego_network_json"):
        network[key] = str(values[key]).strip()
    host = str(values["scenario_load_host"]).strip()
    if not host:
        raise ValueError("ScenarioLoad IP가 비어 있습니다")
    result.setdefault("morai", {})["scenario_load_host"] = host
    tolerance = finite_number(values["goal_tolerance_m"], "도착 허용 거리", positive=True)
    result.setdefault("validation", {})["goal_tolerance_m"] = tolerance
    result["resume"] = True
    return result


def personal_path(document, user=None):
    username = re.sub(r"[^a-zA-Z0-9_-]", "_", user or getpass.getuser())
    return USERS / username / (document["campaign_name"] + ".yaml")


def save_personal(document, path):
    """Never mutate a campaign's YAML once its state exists; resume needs same hash."""
    path = Path(path)
    output = resolve_path(document.get("output_root", "planner_logs/validation")) / document["campaign_name"]
    if path.is_file():
        old = load_document(path)
        if old == document:
            return path
        if (output / "campaign_state.json").exists():
            raise ValueError("실행된 캠페인의 YAML은 수정할 수 없습니다. 새 캠페인 이름을 사용하세요")
    path.parent.mkdir(parents=True, exist_ok=True)
    header = ("# GUI에서 저장한 개인 캠페인입니다. 이 파일은 Git에 포함되지 않습니다.\n"
              "# 시나리오별 다른 목적지/시간은 같은 이름의 .case.yaml에 적습니다.\n"
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
    return "검증 종료 {}/{} | 성공 {} / 주행 실패 {} / 시스템 오류 {} | 종료 코드 {}".format(
        completed, total, counts["PASS"], counts["FAIL"], counts["ERROR"], code)


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


def main(*, on_ready=None, runner_executable=None):
    """Open the GUI; optional hooks let Xvfb tests drive buttons without ROS."""
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk

    examples = sorted(EXAMPLES.glob("*.yaml"))
    if not examples:
        raise RuntimeError("검증기 예제 YAML이 없습니다: {}".format(EXAMPLES))
    preferred = EXAMPLES / "static_pair_stress_gt.yaml"
    initial = preferred if preferred.exists() else examples[0]
    runner = str(runner_executable or ROOT / "planning_validator")

    root = tk.Tk()
    root.title("AjouNice 주행 시나리오 검증기")
    root.geometry("1010x810")
    root.minsize(850, 650)
    frame = ttk.Frame(root, padding=10)
    frame.pack(fill="both", expand=True)
    source = tk.StringVar()
    status = tk.StringVar(value="준비됨 — MORAI GUI에서 Sensor/Ego Network를 먼저 Load·Connect하세요.")
    vars_ = {}
    active = {"process": None, "check_process": None, "config": None,
              "document": None, "stopping": False}
    rerun_failed = tk.BooleanVar(value=False)
    result_paths = {}
    events = queue.Queue()

    def read_source(path, fresh_name=False):
        document = load_document(path)
        values = form_values(document)
        if fresh_name:
            values["campaign_name"] = default_name(path)
        active["template"] = document
        active["source_path"] = Path(path)
        active["config"] = Path(path) if is_within(path, USERS) else None
        active["document"] = document if active["config"] else None
        result_paths.clear()
        for item in table.get_children():
            table.delete(item)
        source.set(str(path))
        for key, value in values.items():
            vars_[key].set(value)
        status.set("설정 로드: {}".format(Path(path).name))

    top = ttk.LabelFrame(frame, text="설정 파일 (예제는 읽기 전용, 저장은 개인 폴더)", padding=8)
    top.pack(fill="x")
    choices = [str(p) for p in examples]
    source_box = ttk.Combobox(top, textvariable=source, values=choices, state="readonly", width=100)
    source_box.grid(row=0, column=0, sticky="ew", padx=4)
    source_box.bind("<<ComboboxSelected>>", lambda _: read_source(source.get(), fresh_name=True))

    def choose_personal():
        path = filedialog.askopenfilename(initialdir=str(USERS), filetypes=[("YAML", "*.yaml *.yml")])
        if path:
            try:
                read_source(path)
            except Exception as error:
                messagebox.showerror("설정 로드 실패", str(error))

    personal_button = ttk.Button(top, text="내 YAML 열기 / resume", command=choose_personal)
    personal_button.grid(row=0, column=1, padx=4)
    top.columnconfigure(0, weight=1)
    ttk.Label(top, text="처음 연결은 MORAI MR GUI에서 직접 합니다. 시나리오 전환만 자동입니다.", foreground="#a44").grid(row=1, column=0, columnspan=2, sticky="w", pady=5)

    notebook = ttk.Notebook(frame)
    notebook.pack(fill="x", pady=8)
    basic = ttk.Frame(notebook, padding=10)
    advanced = ttk.Frame(notebook, padding=10)
    notebook.add(basic, text="기본 설정")
    notebook.add(advanced, text="고급 / 네트워크")

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

    add_field(basic, 0, "campaign_name", "캠페인 이름")
    add_field(basic, 1, "directory", "시나리오 JSON 폴더", browse="dir")
    add_field(basic, 2, "glob", "파일 선택 패턴")
    for row, (key, label) in enumerate((("x", "목적지 map X (m)"), ("y", "목적지 map Y (m)"),
                                        ("z", "목적지 map Z (m)"), ("yaw_deg", "목적지 yaw (deg)")), 3):
        add_field(basic, row, "goal_" + key, label)
    add_field(basic, 7, "timeout_sec", "주행 제한 시간 (초)")
    add_field(basic, 8, "pose_source", "측위", choices=["localization", "maptf_morai"])
    add_field(basic, 9, "detector", "객체 탐지", choices=["od_gt", "bevfusion"])
    add_field(basic, 10, "traffic_light", "신호등 인지", boolean=True)
    add_field(basic, 11, "video", "화면 영상 녹화", boolean=True)
    add_field(basic, 12, "include_sensors", "bag에 카메라/LiDAR", boolean=True)

    add_field(advanced, 0, "topic_profile", "rosbag 토픽 JSON", browse="file")
    add_field(advanced, 1, "sensor_json", "MORAI Sensor JSON", browse="file")
    add_field(advanced, 2, "ego_network_json", "MORAI Ego Network JSON", browse="file")
    add_field(advanced, 3, "scenario_load_host", "ScenarioLoad IP")
    add_field(advanced, 4, "goal_tolerance_m", "목적지 허용 반경 (m)")
    ttk.Label(advanced, text="그 외 옵션과 시나리오별 .case.yaml override는 원본 YAML 값을 보존합니다.").grid(row=5, column=0, columnspan=3, sticky="w", pady=8)

    def collect():
        values = {key: var.get() for key, var in vars_.items()}
        document = build_document(active["template"], values)
        path = active["source_path"]
        if is_within(path, EXAMPLES) or not is_within(path, USERS):
            path = personal_path(document)
        elif path.stem != document["campaign_name"]:
            path = personal_path(document)
        return document, path

    def save():
        if active["process"] and active["process"].poll() is None:
            raise ValueError("캠페인 실행 중에는 YAML을 변경할 수 없습니다. 중단 후 새 이름으로 저장하세요")
        document, path = collect()
        save_personal(document, path)
        active.update(config=path, document=document)
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

    def start():
        if active["check_process"] and active["check_process"].poll() is None:
            messagebox.showinfo("설정 검사 중", "현재 설정 검사가 끝난 후 캠페인을 시작하세요.")
            return
        if active["process"] and active["process"].poll() is None:
            messagebox.showinfo("이미 실행 중", "한 번에 한 캠페인만 실행할 수 있습니다.")
            return
        try:
            path = save()
        except Exception as error:
            show_error("실행 준비 실패", error)
            return
        if not messagebox.askokcancel("캠페인 시작", "MORAI GUI에서 YAML의 Sensor/Ego Network JSON을 Load·Connect했나요?\n\n녹화 중 GUI를 최소화합니다. 다시 띄우면 전체 화면 영상에도 찍힐 수 있습니다."):
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
        active.update(process=process, stopping=False)
        set_running_ui(True)
        status.set("주행 검증 실행 중: {}".format(active["document"]["campaign_name"]))
        threading.Thread(target=command_reader, args=(process, "run"), daemon=True).start()
        root.after(800, lambda: root.iconify() if active["process"] and
                   active["process"].poll() is None else None)

    def stop():
        process = active["process"]
        if process and process.poll() is None and not active["stopping"]:
            if messagebox.askyesno("안전 중단", "현재 주행을 중단하고 기록을 보존할까요?\n다음 실행에서 이어갈 수 있습니다."):
                active["stopping"] = True
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                status.set("중단 요청 중 — 녹화와 ROS 스택 정리를 기다립니다")

    def open_results():
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

    controls = ttk.Frame(frame)
    controls.pack(fill="x", pady=5)
    control_buttons = {}
    for label, action in (("개인 YAML 저장", save_clicked), ("설정 검사", check),
                          ("실행 / resume", start), ("중단", stop), ("결과 폴더", open_results)):
        button = ttk.Button(controls, text=label, command=action)
        button.pack(side="left", padx=4)
        control_buttons[label] = button
    ttk.Checkbutton(controls, text="기존 FAIL도 다시 실행", variable=rerun_failed).pack(side="left", padx=12)

    def set_running_ui(running, *, allow_stop=True):
        source_box.configure(state="disabled" if running else "readonly")
        personal_button.configure(state="disabled" if running else "normal")
        for label in ("개인 YAML 저장", "설정 검사", "실행 / resume"):
            control_buttons[label].configure(state="disabled" if running else "normal")
        control_buttons["중단"].configure(state="normal" if running and allow_stop else "disabled")

    set_running_ui(False)
    ttk.Label(frame, textvariable=status, wraplength=970).pack(anchor="w", pady=4)

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
    ttk.Label(frame, text="실패 장면을 더블클릭하면 관측 수치와 의심 단계를 볼 수 있습니다. 원인 확정은 아닙니다.").pack(anchor="w")
    log = tk.Text(frame, height=9, wrap="word", state="disabled")
    log.pack(fill="both", expand=True)

    def append(line):
        log.configure(state="normal")
        log.insert("end", line + "\n")
        if int(log.index("end-1c").split(".")[0]) > 500:
            log.delete("1.0", "100.0")
        log.see("end")
        log.configure(state="disabled")

    def update_progress():
        document = active.get("document")
        if not document:
            return None
        path = campaign_output(document) / "campaign_state.json"
        try:
            state = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None
        config_path = active.get("config")
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
            status.set("진행 {}/{} | 성공 {} 실패 {} 오류 {} | 현재 {}".format(
                len(results), state.get("total", "?"), counts["PASS"], counts["FAIL"], counts["ERROR"], current))
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
                if label == "run":
                    active["process"] = None
                    set_running_ui(False)
                    root.deiconify()
                    root.lift()
                    state = update_progress()
                    status.set(completion_label(code, state))
                else:
                    active["check_process"] = None
                    set_running_ui(False)
                    status.set("설정 검사 {}".format("통과" if code == 0 else "실패 (로그 확인)"))
        update_progress()
        root.after(500, poll)

    def on_close():
        if active["check_process"] and active["check_process"].poll() is None:
            messagebox.showinfo("설정 검사 중", "설정 검사가 끝난 후 창을 닫아 주세요.")
            return
        if active["process"] and active["process"].poll() is None:
            messagebox.showinfo("실행 중", "먼저 중단 버튼을 눌러 ROS와 녹화를 정리하세요.")
            return
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    read_source(initial, fresh_name=True)
    if on_ready is not None:
        on_ready({"root": root, "fields": vars_, "actions": {
            "save": save, "check": check, "start": start, "stop": stop,
            "update_progress": update_progress}, "active": active,
            "status": status, "table": table, "log": log,
            "rerun_failed": rerun_failed})
    root.after(500, poll)
    root.mainloop()
    return 0
