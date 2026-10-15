#!/usr/bin/python3
"""Ubuntu/Drive setup entrypoint; no ROS or third-party Python imports needed."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from planning_scenario_validator.bootstrap import main

if __name__ == '__main__':
    sys.exit(main())
