#!/usr/bin/env python3
# 목적: manifest에 적힌 모든 iteration의 MORAI scenario JSON을 batch 전에 생성한다.
# 역할: config/manifest/trajectory/scenario 내부 모듈을 조합하는 생성 전용 진입점이다.
# 주요 흐름: 인자 읽기 -> config와 manifest 검증 -> trajectory 시작 상태 확인 -> scenario 생성.

"""batch manifest 기반 MORAI scenario JSON 생성 명령."""

import argparse
import sys

from trajectory_batch_runner.config_loader import load_batch_config
from trajectory_batch_runner.manifest_loader import load_manifest
from trajectory_batch_runner.scenario_builder import ensure_scenario
from trajectory_batch_runner.trajectory_input import read_trajectory_start


def parse_args():
    parser = argparse.ArgumentParser(
        description="batch manifest의 trajectory 시작 상태로 MORAI scenario를 생성합니다"
    )
    parser.add_argument("--config", required=True, help="batch YAML config 경로")
    parser.add_argument(
        "--manifest",
        help="manifest 파일명 또는 경로; 생략하면 config의 manifest_csv 사용",
    )
    return parser.parse_args()


def run(arguments) -> int:
    config = load_batch_config(arguments.config, arguments.manifest)
    rows = load_manifest(config)
    created = 0
    reused = 0
    for row in rows:
        start = read_trajectory_start(config, row)
        result = ensure_scenario(config, row, start)
        if result.reused_existing:
            reused += 1
            state = "확인"
        else:
            created += 1
            state = "생성"
        print(
            "[{}/{}] {} {} <- {} ({:.3f} km/h)".format(
                row.iteration,
                len(rows),
                state,
                result.path.relative_to(config.repository_root),
                row.trajectory_file,
                row.speed_kph,
            )
        )
    print("scenario 처리 완료: 생성 {}개, 기존 확인 {}개".format(created, reused))
    return 0


def main() -> int:
    try:
        return run(parse_args())
    except Exception as error:
        print("scenario_json_generator 오류: {}".format(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
