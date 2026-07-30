# 목적: ROS topic 준비 상태와 trajectory_validator 기록 완료를 확인한다.
# 역할: runner의 순서 제어에서 대기·timeout·history 판정 로직을 분리한다.
# 주요 흐름: 필수 topic 대기 -> scenario Ego pose 확인 -> 새 run 폴더 탐지 -> metadata 완료 확인.

"""trajectory batch 실행 상태 monitor."""

import json
import math
import time
from pathlib import Path
from typing import Optional, Set

from trajectory_batch_runner.config_loader import BatchConfig
from trajectory_batch_runner.scenario_builder import ScenarioResult


class ValidationMonitorError(RuntimeError):
    """topic 또는 validator 기록이 제한 시간 안에 준비되지 않을 때 발생한다."""


class ValidationMonitor:
    """ROS message와 validator history를 이용해 iteration 상태를 판정한다."""

    def __init__(self, config: BatchConfig):
        self.config = config
        self.history_root = config.resolve_config_path(
            "paths.directories.validator_history_root", expect_directory=True
        )
        self.topics = config.get("monitoring.topics")
        self.stack_ready_timeout = float(
            config.get("monitoring.timeout_sec.stack_ready")
        )
        self.scenario_ready_timeout = float(
            config.get("monitoring.timeout_sec.scenario_ready")
        )
        self.iteration_timeout = float(
            config.get("monitoring.timeout_sec.iteration_complete")
        )
        self.position_tolerance = float(
            config.get("monitoring.scenario_position_tolerance_m")
        )
        for label, value in (
            ("stack_ready", self.stack_ready_timeout),
            ("scenario_ready", self.scenario_ready_timeout),
            ("iteration_complete", self.iteration_timeout),
            ("scenario_position_tolerance_m", self.position_tolerance),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValidationMonitorError("{}은 양수여야 합니다".format(label))

    @staticmethod
    def _rospy():
        try:
            import rospy
        except ImportError as error:
            raise ValidationMonitorError("rospy를 import할 수 없습니다") from error
        return rospy

    def wait_persistent_stack(self) -> None:
        """Ego와 localization message가 모두 수신되는지 확인한다."""

        rospy = self._rospy()
        try:
            from morai_msgs.msg import EgoVehicleStatus
            from nav_msgs.msg import Odometry

            rospy.wait_for_message(
                self.topics["ego"],
                EgoVehicleStatus,
                timeout=self.stack_ready_timeout,
            )
            rospy.wait_for_message(
                self.topics["kinematic_state"],
                Odometry,
                timeout=self.stack_ready_timeout,
            )
        except Exception as error:
            raise ValidationMonitorError(
                "persistent stack topic이 준비되지 않았습니다"
            ) from error

    def wait_trajectory(self) -> None:
        """traj_gen의 trajectory message를 기다린다."""

        rospy = self._rospy()
        try:
            from autoware_planning_msgs.msg import Trajectory

            rospy.wait_for_message(
                self.topics["trajectory"],
                Trajectory,
                timeout=self.stack_ready_timeout,
            )
        except Exception as error:
            raise ValidationMonitorError(
                "trajectory topic이 준비되지 않았습니다"
            ) from error

    def wait_control_command(self) -> None:
        """traj_fol_a의 MORAI control command를 기다린다."""

        rospy = self._rospy()
        try:
            from morai_msgs.msg import CtrlCmd

            rospy.wait_for_message(
                self.topics["control_command"],
                CtrlCmd,
                timeout=self.stack_ready_timeout,
            )
        except Exception as error:
            raise ValidationMonitorError(
                "control command topic이 준비되지 않았습니다"
            ) from error

    def wait_scenario_pose(self, scenario: ScenarioResult) -> None:
        """ScenarioLoad 뒤 Ego XY 위치가 생성한 scenario 시작점인지 확인한다."""

        rospy = self._rospy()
        try:
            from morai_msgs.msg import EgoVehicleStatus
        except ImportError as error:
            raise ValidationMonitorError(
                "morai_msgs/EgoVehicleStatus를 import할 수 없습니다"
            ) from error
        deadline = time.monotonic() + self.scenario_ready_timeout
        last_distance: Optional[float] = None
        while time.monotonic() < deadline and not rospy.is_shutdown():
            remaining = max(deadline - time.monotonic(), 0.05)
            try:
                message = rospy.wait_for_message(
                    self.topics["ego"],
                    EgoVehicleStatus,
                    timeout=min(remaining, 1.0),
                )
            except Exception:
                continue
            last_distance = math.hypot(
                float(message.position.x) - scenario.morai_x,
                float(message.position.y) - scenario.morai_y,
            )
            if last_distance <= self.position_tolerance:
                return
        raise ValidationMonitorError(
            "ScenarioLoad 뒤 Ego 위치가 시작점에 적용되지 않았습니다"
            + (
                " (마지막 XY 오차 {:.3f} m)".format(last_distance)
                if last_distance is not None
                else ""
            )
        )

    def _sample_directory(self, trajectory_stem: str) -> Path:
        return self.history_root / trajectory_stem

    def snapshot_runs(self, trajectory_stem: str) -> Set[Path]:
        """validator 시작 전 현재 trajectory의 run 폴더 목록을 저장한다."""

        sample_directory = self._sample_directory(trajectory_stem)
        if not sample_directory.is_dir():
            return set()
        return {path.resolve() for path in sample_directory.iterdir() if path.is_dir()}

    def wait_new_run_directory(
        self, trajectory_stem: str, before: Set[Path]
    ) -> Path:
        """validator가 현재 iteration용 새 run 폴더를 만들 때까지 기다린다."""

        rospy = self._rospy()
        sample_directory = self._sample_directory(trajectory_stem)
        deadline = time.monotonic() + self.stack_ready_timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if sample_directory.is_dir():
                candidates = sorted(
                    (
                        path.resolve()
                        for path in sample_directory.iterdir()
                        if path.is_dir() and path.resolve() not in before
                    ),
                    key=lambda path: path.stat().st_mtime_ns,
                    reverse=True,
                )
                if candidates:
                    return candidates[0]
            time.sleep(0.1)
        raise ValidationMonitorError(
            "trajectory_validator의 새 history run 폴더가 생성되지 않았습니다"
        )

    def wait_completion(self, run_directory: Path) -> Path:
        """새 metadata가 config의 완료 조건으로 마감될 때까지 기다린다."""

        rospy = self._rospy()
        metadata_path = run_directory / "metadata.json"
        expected_status = self.config.get(
            "monitoring.validator_completion.metadata_status"
        )
        accepted_reason = self.config.get(
            "monitoring.validator_completion.accepted_end_reason"
        )
        deadline = time.monotonic() + self.iteration_timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if metadata_path.is_file():
                try:
                    with metadata_path.open(encoding="utf-8") as stream:
                        metadata = json.load(stream)
                except (OSError, json.JSONDecodeError):
                    time.sleep(0.1)
                    continue
                if metadata.get("status") == expected_status:
                    reason = metadata.get("recording", {}).get("end_reason")
                    if reason != accepted_reason:
                        raise ValidationMonitorError(
                            "validator가 {}로 종료됐습니다: {}".format(
                                reason, metadata_path
                            )
                        )
                    return metadata_path
            time.sleep(0.2)
        raise ValidationMonitorError(
            "validator 기록이 제한 시간 안에 완료되지 않았습니다: {}".format(
                run_directory
            )
        )
