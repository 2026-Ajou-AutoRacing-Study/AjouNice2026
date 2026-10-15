"""Interpreter selection only; signal decision parameters remain unchanged."""
import os
from pathlib import Path

DEFAULT_PYTHON = "~/.local/share/ajounice/traffic_light_gpu/bin/python"
MODE_LABELS = {"system": "시스템 Python", "virtualenv": "가상환경 Python"}
LABEL_MODES = {label: mode for mode, label in MODE_LABELS.items()}


def runtime_mode(stack):
    # Old YAMLs with a custom Conda interpreter continue to work unchanged.
    mode = stack.get("traffic_light_runtime")
    if mode is None:
        mode = "virtualenv" if stack.get("traffic_light_python", "/usr/bin/python3") != "/usr/bin/python3" else "system"
    if not isinstance(mode, str) or mode not in MODE_LABELS:
        raise ValueError("stack.traffic_light_runtime은 system 또는 virtualenv입니다")
    return mode


def runtime_settings(stack, root, *, check_exists=False):
    mode = runtime_mode(stack)
    device = str(stack.get("traffic_light_device", "0")).strip()
    if device != "cpu" and not (device.isascii() and device.isdecimal()):
        raise ValueError("신호등 장치는 cpu 또는 GPU 번호(예: 0)여야 합니다")
    value = "/usr/bin/python3" if mode == "system" else stack.get("traffic_light_python", DEFAULT_PYTHON)
    if not isinstance(value, str) or not value.strip():
        raise ValueError("신호등 가상환경 Python 실행 파일 경로가 필요합니다")
    python = Path(os.path.expandvars(value)).expanduser()
    python = python if python.is_absolute() else Path(root) / python
    # Do not resolve symlinks: a venv's bin/python symlink identifies its prefix.
    python = Path(os.path.abspath(python))
    if check_exists and (not python.is_file() or not os.access(python, os.X_OK)):
        raise ValueError("신호등 Python이 없거나 실행할 수 없습니다: {}. 가이드의 환경 설치부터 진행하세요".format(python))
    return str(python), device
