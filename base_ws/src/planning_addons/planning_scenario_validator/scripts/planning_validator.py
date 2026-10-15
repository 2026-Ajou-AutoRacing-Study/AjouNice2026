#!/usr/bin/python3
"""ROS 패키지 진입점: YAML 하나로 planning 캠페인을 실행한다."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from planning_scenario_validator.runner import main

if __name__ == "__main__":
    sys.exit(main())
