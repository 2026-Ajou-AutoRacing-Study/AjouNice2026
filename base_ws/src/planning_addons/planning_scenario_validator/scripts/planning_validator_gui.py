#!/usr/bin/python3
"""Tkinter entry point; the existing CLI remains the only campaign engine."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from planning_scenario_validator.gui import main

if __name__ == "__main__":
    sys.exit(main())
