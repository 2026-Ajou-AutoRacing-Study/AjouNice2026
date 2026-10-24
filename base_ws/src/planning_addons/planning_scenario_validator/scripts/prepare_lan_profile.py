#!/usr/bin/python3
"""Create private LAN profiles without connecting MORAI or starting any stack."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from planning_scenario_validator.lan_profiles import main

if __name__ == '__main__':
    sys.exit(main())
