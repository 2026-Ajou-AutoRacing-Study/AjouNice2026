# 목적: sample trajectory CSV의 첫 pose와 시작 목표 속도를 읽는다.
# 역할: manifest 속도 검증과 MORAI scenario 시작 pose 생성에 필요한 값을 제공한다.
# 주요 흐름: CSV 첫 행 읽기 -> 필수 열 숫자 변환 -> 속도 비교 -> package 상대경로 생성.

"""sample trajectory 입력 검사."""

import csv
import math
from dataclasses import dataclass
from pathlib import Path

from trajectory_batch_runner.config_loader import BatchConfig
from trajectory_batch_runner.manifest_loader import ManifestRow


class TrajectoryInputError(ValueError):
    """trajectory CSV의 형식이나 시작 조건이 올바르지 않을 때 발생한다."""


@dataclass(frozen=True)
class TrajectoryStart:
    """trajectory 첫 데이터 행에서 읽은 시작 상태다."""

    x: float
    y: float
    z: float
    yaw_rad: float
    lanelet_id: str
    velocity_mps: float

    @property
    def speed_kph(self) -> float:
        return self.velocity_mps * 3.6


def _finite_float(row: dict, key: str, path: Path) -> float:
    value = row.get(key)
    try:
        parsed = float(value)
    except (TypeError, ValueError) as error:
        raise TrajectoryInputError(
            "{} 첫 행의 {} 값이 숫자가 아닙니다: {!r}".format(path, key, value)
        ) from error
    if not math.isfinite(parsed):
        raise TrajectoryInputError(
            "{} 첫 행의 {} 값이 유한하지 않습니다".format(path, key)
        )
    return parsed


def read_trajectory_start(
    config: BatchConfig, row: ManifestRow
) -> TrajectoryStart:
    """trajectory 첫 행을 읽고 manifest의 speed_kph와 일치하는지 확인한다."""

    columns = config.get("scenario_generation.trajectory_columns")
    required = ("x", "y", "z", "yaw", "lanelet_id", "velocity_mps")
    if not isinstance(columns, dict) or any(key not in columns for key in required):
        raise TrajectoryInputError(
            "scenario_generation.trajectory_columns 설정이 부족합니다"
        )
    with row.trajectory_path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        missing = [
            columns[key]
            for key in required
            if columns[key] not in (reader.fieldnames or [])
        ]
        if missing:
            raise TrajectoryInputError(
                "{}에 필요한 열이 없습니다: {}".format(
                    row.trajectory_path, ", ".join(missing)
                )
            )
        try:
            first = next(reader)
        except StopIteration as error:
            raise TrajectoryInputError(
                "trajectory CSV에 데이터 행이 없습니다: {}".format(
                    row.trajectory_path
                )
            ) from error

    lanelet_id = (first.get(columns["lanelet_id"]) or "").strip()
    if not lanelet_id:
        raise TrajectoryInputError(
            "{} 첫 행의 lanelet_id가 비어 있습니다".format(row.trajectory_path)
        )
    start = TrajectoryStart(
        x=_finite_float(first, columns["x"], row.trajectory_path),
        y=_finite_float(first, columns["y"], row.trajectory_path),
        z=_finite_float(first, columns["z"], row.trajectory_path),
        yaw_rad=_finite_float(first, columns["yaw"], row.trajectory_path),
        lanelet_id=lanelet_id,
        velocity_mps=_finite_float(
            first, columns["velocity_mps"], row.trajectory_path
        ),
    )
    if start.velocity_mps < 0.0:
        raise TrajectoryInputError("trajectory 시작 속도는 음수일 수 없습니다")
    tolerance = float(config.get("scenario_generation.initial_speed_tolerance_kph"))
    if (
        not math.isfinite(tolerance)
        or tolerance < 0.0
        or abs(start.speed_kph - row.speed_kph) > tolerance
    ):
        raise TrajectoryInputError(
            "{}의 시작 속도 {:.6f} km/h가 manifest speed_kph {:.6f}와 "
            "허용 오차 {:.6f} 안에서 일치하지 않습니다".format(
                row.trajectory_path,
                start.speed_kph,
                row.speed_kph,
                tolerance,
            )
        )
    return start


def trajectory_package_relative_path(
    config: BatchConfig, trajectory_path: Path
) -> str:
    """traj_gen의 trajectory_csv 인자로 넘길 package 상대경로를 만든다."""

    package_root = (
        config.repository_root / "base_ws/src/control/trajectory_generator"
    ).resolve()
    try:
        relative = trajectory_path.resolve().relative_to(package_root)
    except ValueError as error:
        raise TrajectoryInputError(
            "trajectory가 trajectory_generator package 밖에 있습니다: {}".format(
                trajectory_path
            )
        ) from error
    return relative.as_posix()
