#!/usr/bin/env python3
"""Audit v81 speed limits against the competition highway exception bounds."""

# The v81 split does not change physical geometry or speed policy. Reuse the
# complete v80 audit implementation with the current map and report paths.
import audit_v80_speed_rule as audit

audit.MAP = audit.ROOT / "lanelet_map/lanelet2_map_v81.osm"
audit.REPORT = audit.ROOT / "docs/kcity_v81_speed_rule_audit.json"


if __name__ == "__main__":
    audit.main()
