#!/usr/bin/env python3
# 목적: manifest의 iteration을 MORAI에서 순서대로 실행하고 validator 기록을 완성한다.
# 역할: 재사용 모듈을 조합해 scenario 준비, UDP load, ROS stack, 완료 대기를 조율한다.
# 주요 흐름: 전체 사전 검증 -> persistent stack -> iteration node 준비 -> scenario load -> 기록 확인.

"""MORAI trajectory batch runner ROS node."""

import argparse
import sys
from dataclasses import dataclass
from typing import List

import rospy

from trajectory_batch_runner.config_loader import BatchConfig, load_batch_config
from trajectory_batch_runner.manifest_loader import ManifestRow, load_manifest
from trajectory_batch_runner.morai_udp_client import MoraiScenarioLoadClient
from trajectory_batch_runner.ros_stack_manager import RosStackManager
from trajectory_batch_runner.scenario_builder import (
    ScenarioResult,
    load_scenario,
)
from trajectory_batch_runner.trajectory_input import (
    TrajectoryStart,
    read_trajectory_start,
    trajectory_package_relative_path,
)
from trajectory_batch_runner.validation_monitor import ValidationMonitor


@dataclass(frozen=True)
class PreparedIteration:
    """실행 전에 trajectory와 scenario 검증을 끝낸 manifest 행이다."""

    row: ManifestRow
    start: TrajectoryStart
    scenario: ScenarioResult
    trajectory_package_path: str


def parse_args():
    parser = argparse.ArgumentParser(
        description="MORAI trajectory batch manifest를 iteration 순서대로 실행합니다"
    )
    parser.add_argument("--config", required=True, help="batch YAML config 경로")
    parser.add_argument(
        "--manifest",
        help="manifest 파일명 또는 경로; 생략하면 config의 manifest_csv 사용",
    )
    return parser.parse_args(rospy.myargv()[1:])


def prepare_iterations(config: BatchConfig) -> List[PreparedIteration]:
    """차량을 움직이기 전에 manifest 전체와 scenario를 검증한다."""

    rows = load_manifest(config)
    prepared = []
    for row in rows:
        start = read_trajectory_start(config, row)
        scenario = load_scenario(config, row, start)
        prepared.append(
            PreparedIteration(
                row=row,
                start=start,
                scenario=scenario,
                trajectory_package_path=trajectory_package_relative_path(
                    config, row.trajectory_path
                ),
            )
        )
    return prepared


def run_iteration(
    config: BatchConfig,
    item: PreparedIteration,
    stack: RosStackManager,
    monitor: ValidationMonitor,
    udp: MoraiScenarioLoadClient,
) -> None:
    """준비 load부터 validator 완료까지 manifest 한 행을 실행한다."""

    row = item.row
    rospy.loginfo("batch %s 시작", row.label)
    stack.stop_iteration_stack()

    stack.start_publisher(item.trajectory_package_path)
    monitor.wait_trajectory()

    # Reset Ego before the validator exists so the teleport/reconnect samples
    # never enter tracking metrics. The generated scenario starts at 0 km/h.
    udp.send(item.scenario.path, pause=False)
    monitor.wait_scenario_pose(item.scenario)
    rospy.loginfo("batch %s 초기 위치 적용 완료", row.label)

    previous_runs = monitor.snapshot_runs(row.trajectory_path.stem)
    stack.start_validator()
    run_directory = monitor.wait_new_run_directory(
        row.trajectory_path.stem, previous_runs
    )

    stack.start_follower()
    monitor.wait_control_command()

    metadata_path = monitor.wait_completion(run_directory)
    rospy.loginfo(
        "batch %s 완료: %s",
        row.label,
        metadata_path.relative_to(config.repository_root),
    )


def run(arguments) -> int:
    config = load_batch_config(arguments.config, arguments.manifest)
    prepared = prepare_iterations(config)
    rospy.loginfo(
        "batch 사전 검증 완료: experiment=%s, map=v%s (%s), "
        "manifest=%s, iterations=%d",
        config.get("batch.experiment_type"),
        config.get("map.lanelet.selected_version"),
        config.get("map.lanelet.selected_file"),
        config.manifest_path,
        len(prepared),
    )

    stack = RosStackManager(config)
    monitor = ValidationMonitor(config)
    rospy.on_shutdown(stack.close)
    try:
        with MoraiScenarioLoadClient(config) as udp:
            stack.start_persistent_stack()
            monitor.wait_persistent_stack()
            for item in prepared:
                if rospy.is_shutdown():
                    raise rospy.ROSInterruptException("ROS shutdown")
                try:
                    run_iteration(config, item, stack, monitor, udp)
                finally:
                    stack.stop_iteration_stack()
    finally:
        stack.close()
    rospy.loginfo("모든 batch iteration이 완료됐습니다")
    return 0


def main() -> int:
    arguments = parse_args()
    rospy.init_node("trajectory_batch_runner", anonymous=False)
    try:
        return run(arguments)
    except rospy.ROSInterruptException:
        return 130
    except Exception as error:
        rospy.logfatal("trajectory_batch_runner 오류: %s", error)
        return 1


if __name__ == "__main__":
    sys.exit(main())
