#!/usr/bin/python3
"""Read-only diagnostic entrypoint, does not install packages or launch driving."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from planning_scenario_validator.doctor import main

if __name__ == '__main__':
    sys.exit(main())
