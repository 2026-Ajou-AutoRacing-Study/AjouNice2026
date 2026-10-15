#!/usr/bin/python3
"""캠페인 결과 파일을 읽는 터미널 대시보드."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from planning_scenario_validator.dashboard import main

if __name__ == "__main__":
    sys.exit(main())
