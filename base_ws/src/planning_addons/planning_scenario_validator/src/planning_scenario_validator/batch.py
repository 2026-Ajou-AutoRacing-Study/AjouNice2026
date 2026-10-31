"""One persistent queue of campaign YAMLs; campaigns remain the source of truth."""
import argparse
import csv
import fcntl
import hashlib
import json
import os
import signal
import subprocess
import threading
import time
import uuid
from pathlib import Path

from .config import ROOT, EXECUTION_CONTRACT, load_campaign, resolve_path
from .executions import load_execution, prepare_execution
from .runtime import RuntimeFailure, atomic_json, now


def queue_path():
    return Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config") / "ajounice/planning_scenario_validator/batch.json"


def read_queue(path=None):
    path = Path(path or queue_path())
    if not path.exists():
        return {"schema_version": 1, "items": [], "run_directory": None}
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or value.get("schema_version") != 1 or not isinstance(value.get("items"), list):
        raise ValueError("배치 목록 파일 형식이 잘못되었습니다. 기존 파일을 보존하고 목록을 다시 준비하세요")
    if any(not isinstance(item, str) for item in value["items"]):
        raise ValueError("배치 목록에는 YAML 경로만 넣습니다")
    if value.get("run_directory") is not None and not isinstance(value["run_directory"], str):
        raise ValueError("배치 결과 폴더 경로 형식이 잘못되었습니다")
    return value


def write_queue(value, path=None):
    atomic_json(Path(path or queue_path()), value)


def batch_state(value):
    directory = value.get("run_directory")
    if not directory:
        return None
    state = json.loads((resolve_path(directory) / "state.json").read_text(encoding="utf-8"))
    if (not isinstance(state, dict) or state.get("schema_version") != 1 or
            not isinstance(state.get("items"), list) or not state["items"] or
            state.get("status") not in ("READY", "RUNNING", "DONE", "INTERRUPTED", "ERROR")):
        raise ValueError("배치 state 형식이 잘못되었습니다. 기존 결과는 보존합니다")
    for row in state["items"]:
        if (not isinstance(row, dict) or any(key not in row for key in
                ("path", "contract", "campaign", "output", "scene_count", "estimate_sec", "status")) or
                not isinstance(row.get("counts", {}), dict)):
            raise ValueError("배치 장면 상태 형식이 잘못되었습니다. 기존 결과는 보존합니다")
    index = state.get("current_index")
    if index is not None and (type(index) is not int or not 0 <= index < len(state["items"])):
        raise ValueError("배치 진행 순서가 잘못되었습니다. 기존 결과는 보존합니다")
    return state


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def input_contract(cfg):
    contract = {"yaml": digest(cfg["path"]),
            "scenarios": {str(case.source): case.source_sha256 for case in cfg["cases"]},
            "network": {key: digest(cfg["network"][key]) for key in ("sensor_json", "ego_network_json")},
            "topic_profile": (None if not cfg['recording'].get('bag_enabled', True) else
                              hashlib.sha256(json.dumps(cfg['recording']['topics']).encode()).hexdigest()
                              if 'selected_topics' in cfg['recording'] else
                              digest(resolve_path(cfg["recording"]["topic_profile"])))}
    if cfg.get('route'):
        contract['route_definition'] = digest(cfg['route']['definition_file'])
    return contract


def plan(paths, *, check_install=True):
    if not paths:
        raise ValueError("배치에 실행할 YAML을 추가하세요")
    configs = [load_campaign(path, check_install=check_install) for path in paths]
    if len({str(cfg["path"]) for cfg in configs}) != len(configs):
        raise ValueError("같은 YAML이 배치에 중복되었습니다")
    # MORAI first connection stays manual: do not cross a reconnect boundary.
    network = None
    load_target = None
    rows = []
    for cfg in configs:
        contract = input_contract(cfg)
        current = tuple(contract["network"][key] for key in ("sensor_json", "ego_network_json"))
        target = (cfg["document"].get("morai", {}).get("scenario_load_host", "127.0.0.1"),
                  cfg["document"].get("morai", {}).get("scenario_load_port", 9095), str(cfg.get("install")))
        if network is not None and (network != current or load_target != target):
            raise ValueError("MORAI Network/센서 또는 ScenarioLoad 설정이 서로 다릅니다. 무인 배치는 같은 연결 설정만 지원합니다")
        network, load_target = current, target
        rows.append({"path": str(cfg["path"]), "campaign": cfg["name"],
                     "output": str(cfg["output"]), "scene_count": len(cfg["cases"]),
                     "estimate_sec": sum(case.timeout_sec + 35 for case in cfg["cases"]),
                     "contract": contract, "status": "PENDING", "counts": {},
                     "last_error": None})
    return configs, rows


def summarize_campaign(cfg):
    """A zero exit code alone does not prove all scenes were recorded."""
    path = cfg["output"] / "campaign_state.json"
    try:
        state = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return False, {}, "실행 결과 state가 없거나 읽을 수 없습니다"
    if (state.get("execution_contract") != EXECUTION_CONTRACT or
            state.get("config_sha256") != digest(cfg["path"]) or
            state.get("config_path") != str(cfg["path"])):
        return False, {}, "이전 계약 또는 다른 YAML의 실행 결과입니다"
    results = state.get("results", {})
    counts = {label: 0 for label in ("PASS", "FAIL", "ERROR")}
    complete = True
    for case in cfg["cases"]:
        result = results.get(case.name, {})
        status = result.get("status")
        if status in counts:
            counts[status] += 1
        if (status not in ("PASS", "FAIL") or result.get("source_sha256") != case.source_sha256):
            complete = False
    return complete, counts, None if complete else "중단·미완료·시스템 오류 장면이 남았습니다"


def save_state(directory, state):
    directory = Path(directory)
    state["updated_at"] = now()
    atomic_json(directory / "state.json", state)
    path = directory / "summary.csv"
    temporary = path.with_suffix(".csv.tmp")
    with temporary.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["order", "campaign", "yaml", "status", "scenes", "pass", "fail", "error", "reason", "result_directory"])
        for index, item in enumerate(state["items"], 1):
            counts = item.get("counts", {})
            writer.writerow([index, item["campaign"], item["path"], item["status"], item["scene_count"],
                             counts.get("PASS", 0), counts.get("FAIL", 0), counts.get("ERROR", 0),
                             item.get("last_error") or "", item["output"]])
    os.replace(temporary, path)


def run_batch(path=None, *, runner=None, stop_event=None, check_install=True):
    path = Path(path or queue_path())
    path.parent.mkdir(parents=True, exist_ok=True)
    stop_event = stop_event or threading.Event()
    runner = str(runner or ROOT / "planning_validator")
    # Keep the lock inode stable even when the queue JSON is atomically replaced.
    with path.with_suffix(".lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeFailure("이 배치가 이미 실행 중입니다") from error
        queue = read_queue(path)
        state = batch_state(queue)
        if state:
            if [str(resolve_path(path)) for path in queue['items']] != [row['path'] for row in state['items']]:
                raise ValueError('배치 실행 목록이 바뀌었습니다. 이전 목록을 복원하세요')
            if state.get('snapshot_inputs'):
                configs = [load_execution(row['output'], check_install=check_install) for row in state['items']]
                _, planned = plan([cfg['path'] for cfg in configs], check_install=check_install)
                if [row['contract'] for row in planned] != [row['contract'] for row in state['items']]:
                    raise ValueError('배치 실행 사본이 바뀌었습니다. 사본을 복원하세요')
            else:
                configs, planned = plan(queue['items'], check_install=check_install)
                if [(r['path'], r['contract']) for r in state['items']] != [(r['path'], r['contract']) for r in planned]:
                    raise ValueError('이전 배치의 입력이 바뀌었습니다. 입력을 복원하세요')
            directory = resolve_path(queue["run_directory"])
        else:
            _, planned = plan(queue['items'], check_install=check_install)
            configs = []
            for row in planned:
                cfg = prepare_execution(row['path'], check_install=check_install)
                configs.append(cfg)
                row.update(output=str(cfg['output']), execution_path=str(cfg['path']), contract=input_contract(cfg))
            plan([cfg['path'] for cfg in configs], check_install=check_install)
            directory = ROOT / "planner_logs/validation_batches" / uuid.uuid4().hex[:16]
            state = {"schema_version": 1, "snapshot_inputs": True, "created_at": now(), "items": planned,
                     "current_index": None, "status": "READY"}
            queue["run_directory"] = str(directory)
            # Persist state before pointing the GUI to it.
            save_state(directory, state)
            atomic_json(directory / "queue.json", queue)
            write_queue(queue, path)
        state["status"] = "RUNNING"
        save_state(directory, state)
        try:
            for index, (cfg, item) in enumerate(zip(configs, state["items"])):
                if stop_event.is_set():
                    break
                # Validate all inputs again before each campaign, not just the active one.
                paths = [row.get('execution_path', row['path']) for row in state['items']]
                if state.get('snapshot_inputs'):
                    for row in state['items']:
                        load_execution(row['output'], check_install=check_install)
                _, current = plan(paths, check_install=check_install)
                if [r["contract"] for r in current] != [r["contract"] for r in state["items"]]:
                    raise ValueError("배치 실행 중 입력 파일이 바뀌었습니다. 다음 주행을 시작하지 않습니다")
                complete, counts, _ = summarize_campaign(cfg)
                if item["status"] == "DONE" and not complete:
                    raise ValueError("완료된 캠페인 결과가 변경되거나 사라졌습니다")
                if complete:
                    item.update(status="DONE", counts=counts, last_error=None)
                    save_state(directory, state)
                    continue
                state["current_index"] = index
                item.update(status="RUNNING", last_error=None)
                save_state(directory, state)
                print("배치 {}/{} · {}".format(index + 1, len(configs), cfg["name"]), flush=True)
                if stop_event.is_set():
                    break
                process = subprocess.Popen([runner, str(cfg["path"]), "--resume=true"],
                                           cwd=str(ROOT), start_new_session=True)
                sent = False
                while process.poll() is None:
                    if stop_event.is_set() and not sent:
                        try:
                            os.killpg(process.pid, signal.SIGINT)
                        except ProcessLookupError:
                            pass
                        sent = True
                    time.sleep(0.1)
                complete, counts, error = summarize_campaign(cfg)
                item.update(counts=counts, returncode=process.returncode)
                if stop_event.is_set():
                    item.update(status="DONE" if complete else "INTERRUPTED", last_error=None if complete else "사용자 중단")
                    break
                if process.returncode != 0 or not complete:
                    item.update(status="ERROR", last_error=error or "실행기 종료 코드 {}".format(process.returncode))
                    state["status"] = "ERROR"
                    save_state(directory, state)
                    return 1
                item.update(status="DONE", last_error=None)
                save_state(directory, state)
            state["status"] = "INTERRUPTED" if stop_event.is_set() else "DONE"
            save_state(directory, state)
            print("배치 {} · 결과 {}".format(state["status"], directory), flush=True)
            return 130 if stop_event.is_set() else 0
        except BaseException as error:
            state["status"] = "ERROR"
            state["last_error"] = str(error)
            save_state(directory, state)
            raise


def main(argv=None):
    parser = argparse.ArgumentParser(description="저장된 YAML 실행 목록을 순서대로 실행/재개")
    parser.add_argument("queue", nargs="?", default=str(queue_path()))
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        if args.check:
            _, rows = plan(read_queue(args.queue)["items"])
            print("배치 검사 통과: YAML {}개, 장면 {}개, 예상 {:.0f}분 (추정)".format(
                len(rows), sum(row["scene_count"] for row in rows), sum(row["estimate_sec"] for row in rows) / 60))
            return 0
        return run_batch(args.queue, stop_event=stop)
    except (ValueError, RuntimeFailure, OSError) as error:
        print("배치 오류: {}".format(error), flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
