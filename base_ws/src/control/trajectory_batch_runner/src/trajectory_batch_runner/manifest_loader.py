# 목적: 단순한 batch manifest CSV를 검증하고 iteration 목록으로 변환한다.
# 역할: runner와 scenario generator가 동일한 CSV 해석 규칙을 공유하게 한다.
# 주요 흐름: 헤더 확인 -> 행 값 변환 -> trajectory 경로 결합 -> iteration 정렬.

"""batch manifest CSV 로더."""

import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import List

from trajectory_batch_runner.config_loader import BatchConfig, ConfigError


SAFE_TYPE = re.compile(r"^[a-z0-9][a-z0-9_]*$")


class ManifestError(ValueError):
    """manifest 형식이나 값이 올바르지 않을 때 발생한다."""


@dataclass(frozen=True)
class ManifestRow:
    """manifest 한 행과 해석된 실제 파일 경로를 표현한다."""

    iteration: int
    experiment_type: str
    sub_type: str
    trajectory_file: str
    speed_kph: float
    trajectory_path: Path
    line_number: int

    @property
    def label(self) -> str:
        return "iteration {} ({}/{}, {})".format(
            self.iteration,
            self.experiment_type,
            self.sub_type,
            self.trajectory_file,
        )


def _parse_positive_integer(value: str, label: str) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError) as error:
        raise ManifestError("{}은 양의 정수여야 합니다".format(label)) from error
    if parsed <= 0 or str(parsed) != value.strip():
        raise ManifestError("{}은 양의 정수여야 합니다".format(label))
    return parsed


def _parse_positive_float(value: str, label: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as error:
        raise ManifestError("{}은 양수여야 합니다".format(label)) from error
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise ManifestError("{}은 유한한 양수여야 합니다".format(label))
    return parsed


def _require_type_token(value: str, label: str) -> str:
    token = value.strip()
    if not SAFE_TYPE.fullmatch(token):
        raise ManifestError(
            "{}은 영문 소문자, 숫자, 밑줄만 사용할 수 있습니다".format(label)
        )
    return token


def _require_suffix(value: str, suffix: str, label: str) -> str:
    cleaned = value.strip()
    if Path(cleaned).suffix.lower() != suffix:
        raise ManifestError("{}은 {} 파일이어야 합니다".format(label, suffix))
    return cleaned


def load_manifest(config: BatchConfig) -> List[ManifestRow]:
    """manifest를 읽어 검증된 iteration 오름차순 목록을 반환한다."""

    required = config.get("manifest.required_columns")
    if not isinstance(required, list) or not all(
        isinstance(value, str) for value in required
    ):
        raise ConfigError("manifest.required_columns 형식이 올바르지 않습니다")
    expected_experiment = config.get("batch.experiment_type")

    rows: List[ManifestRow] = []
    seen_iterations = set()
    with config.manifest_path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != required:
            raise ManifestError(
                "manifest 헤더가 config와 다릅니다\nexpected={}\nactual={}".format(
                    required, reader.fieldnames
                )
            )
        for line_number, raw in enumerate(reader, start=2):
            if raw is None or all(not (value or "").strip() for value in raw.values()):
                continue
            iteration = _parse_positive_integer(
                raw["iteration"], "{}행 iteration".format(line_number)
            )
            if iteration in seen_iterations:
                raise ManifestError("iteration {}이 중복됩니다".format(iteration))
            seen_iterations.add(iteration)

            experiment_type = _require_type_token(
                raw["experiment_type"], "{}행 experiment_type".format(line_number)
            )
            if experiment_type != expected_experiment:
                raise ManifestError(
                    "{}행 experiment_type={}가 config의 {}와 다릅니다".format(
                        line_number, experiment_type, expected_experiment
                    )
                )
            sub_type = _require_type_token(
                raw["sub_type"], "{}행 sub_type".format(line_number)
            )
            trajectory_file = _require_suffix(
                raw["trajectory_file"],
                ".csv",
                "{}행 trajectory_file".format(line_number),
            )
            speed_kph = _parse_positive_float(
                raw["speed_kph"], "{}행 speed_kph".format(line_number)
            )
            try:
                trajectory_path = config.resolve_child(
                    "paths.directories.trajectory_directory",
                    trajectory_file,
                    "{}행 trajectory_file".format(line_number),
                    must_exist=True,
                )
            except ConfigError as error:
                raise ManifestError(str(error)) from error
            rows.append(
                ManifestRow(
                    iteration=iteration,
                    experiment_type=experiment_type,
                    sub_type=sub_type,
                    trajectory_file=trajectory_file,
                    speed_kph=speed_kph,
                    trajectory_path=trajectory_path,
                    line_number=line_number,
                )
            )

    if not rows:
        raise ManifestError("실행할 iteration이 manifest에 없습니다")
    return sorted(rows, key=lambda row: row.iteration)
