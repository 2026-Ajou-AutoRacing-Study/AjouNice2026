#!/usr/bin/python3
"""Persistent campaign queue entrypoint; GUI and CLI use the same backend."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from planning_scenario_validator.batch import main

if __name__ == "__main__":
    sys.exit(main())
