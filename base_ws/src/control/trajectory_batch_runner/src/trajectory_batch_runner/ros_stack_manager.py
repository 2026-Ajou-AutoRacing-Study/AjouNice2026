# 목적: batch에서 사용하는 ROS launch 프로세스의 시작과 종료 순서를 관리한다.
# 역할: persistent stack과 iteration stack을 분리하고 자신이 시작한 프로세스만 정리한다.
# 주요 흐름: roslaunch 명령 구성 -> 새 process group 실행 -> 상태 확인 -> SIGINT 기반 역순 종료.

"""trajectory batch용 ROS launch process manager."""

import os
import shutil
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

from trajectory_batch_runner.config_loader import BatchConfig


class RosStackError(RuntimeError):
    """ROS launch 프로세스를 시작하거나 종료할 수 없을 때 발생한다."""


@dataclass
class ManagedProcess:
    """runner가 직접 시작한 한 개의 roslaunch process group이다."""

    name: str
    command: List[str]
    process: subprocess.Popen


def _launch_value(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


class RosStackManager:
    """persistent stack과 현재 iteration stack의 생명주기를 관리한다."""

    def __init__(self, config: BatchConfig):
        self.config = config
        self.roslaunch = shutil.which("roslaunch")
        if self.roslaunch is None:
            raise RosStackError("PATH에서 roslaunch를 찾을 수 없습니다")
        self.persistent: List[ManagedProcess] = []
        self.iteration: List[ManagedProcess] = []

    def _resolve_argument(self, value: Any) -> Any:
        if isinstance(value, str) and value.startswith("paths."):
            return self.config.resolve_config_path(value)
        return value

    def _build_command(
        self,
        block: Dict[str, Any],
        extra_arguments: Optional[Dict[str, Any]] = None,
    ) -> List[str]:
        try:
            package = block["package"]
            launch_file = block["launch_file"]
        except (KeyError, TypeError) as error:
            raise RosStackError("launch block에 package/launch_file이 필요합니다") from error
        command = [self.roslaunch, str(package), str(launch_file)]
        arguments = dict(block.get("arguments", {}))
        if extra_arguments:
            arguments.update(extra_arguments)
        for key, raw_value in arguments.items():
            value = self._resolve_argument(raw_value)
            command.append("{}:={}".format(key, _launch_value(value)))
        return command

    def _start(
        self,
        name: str,
        block: Dict[str, Any],
        destination: List[ManagedProcess],
        extra_arguments: Optional[Dict[str, Any]] = None,
    ) -> ManagedProcess:
        command = self._build_command(block, extra_arguments)
        try:
            process = subprocess.Popen(command, start_new_session=True)
        except OSError as error:
            raise RosStackError(
                "{} 실행에 실패했습니다: {}".format(name, " ".join(command))
            ) from error
        managed = ManagedProcess(name=name, command=command, process=process)
        destination.append(managed)
        time.sleep(0.5)
        return_code = process.poll()
        if return_code is not None:
            destination.remove(managed)
            raise RosStackError(
                "{}이 준비 전에 종료됐습니다 (exit={}): {}".format(
                    name, return_code, " ".join(command)
                )
            )
        return managed

    def start_persistent_stack(self) -> None:
        """udp와 maptf_morai_cor를 batch 시작 시 한 번 실행한다."""

        if self.persistent:
            raise RosStackError("persistent stack이 이미 실행 중입니다")
        blocks = self.config.get("persistent_stack")
        try:
            self._start("udp", blocks["udp"], self.persistent)
            self._start(
                "maptf_morai_cor",
                blocks["maptf_morai_cor"],
                self.persistent,
            )
        except Exception:
            self.stop_persistent_stack()
            raise

    def start_publisher(self, trajectory_csv: str) -> None:
        block = self.config.get("iteration_stack.trajectory_publisher")
        argument_name = block.get("trajectory_argument")
        if not isinstance(argument_name, str) or not argument_name:
            raise RosStackError("trajectory_argument 설정이 필요합니다")
        self._start(
            "trajectory_publisher",
            block,
            self.iteration,
            {argument_name: trajectory_csv},
        )

    def start_validator(self) -> None:
        self._start(
            "trajectory_validator",
            self.config.get("iteration_stack.trajectory_validator"),
            self.iteration,
        )

    def start_follower(self) -> None:
        self._start(
            "trajectory_follower",
            self.config.get("iteration_stack.trajectory_follower"),
            self.iteration,
        )

    @staticmethod
    def _stop_one(managed: ManagedProcess) -> None:
        process = managed.process
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=10.0)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5.0)
                return
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                return
            process.wait(timeout=5.0)

    @classmethod
    def _stop_all(cls, processes: List[ManagedProcess]) -> None:
        while processes:
            managed = processes.pop()
            cls._stop_one(managed)

    def stop_iteration_stack(self) -> None:
        """follower, validator, publisher 순서로 현재 iteration을 종료한다."""

        self._stop_all(self.iteration)

    def stop_persistent_stack(self) -> None:
        """maptf_morai_cor와 udp를 runner가 시작한 역순으로 종료한다."""

        self._stop_all(self.persistent)

    def close(self) -> None:
        self.stop_iteration_stack()
        self.stop_persistent_stack()

    def __enter__(self) -> "RosStackManager":
        return self

    def __exit__(self, _type: Any, _value: Any, _traceback: Any) -> None:
        self.close()
