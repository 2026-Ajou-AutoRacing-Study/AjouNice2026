#!/usr/bin/env python3
"""Keep competition UDP interfaces and add the proven local UDP ScenarioLoad."""
import json
import shutil
from pathlib import Path


ROOT = Path(__file__).resolve().parents[5]
BASE = ROOT / "configs/network/ego_network_competition_local.json"
SCENARIO_SOURCE = ROOT / "configs/network/ego_network_local.json"
OUTPUT = ROOT / "base_ws/src/control/planning_scenario_validator/config/generated/ego_network_validator_local.json"
MR_NETWORK_DIR = Path.home() / "MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Network/25.S4.MolitComp03"


def build():
    base = json.loads(BASE.read_text(encoding="utf-8"))
    source_network = json.loads(SCENARIO_SOURCE.read_text(encoding="utf-8"))
    entries = base["egoNetworkData"][0]["listEgoNetworkInfo"]
    source = source_network["egoNetworkData"][0]["listEgoNetworkInfo"]
    def is_scenario(entry):
        config = entry.get("config", {})
        return (entry.get("PUBSUB_TYPE") == 783
                and entry.get("netType") == 1
                and config.get("commType") == 1
                and config.get("udpConfig", {}).get("hostPort") == 9095)
    scenario = [entry for entry in source if is_scenario(entry)]
    if len(scenario) != 1:
        raise ValueError("UDP ScenarioLoad entry must be unique")
    slots = [index for index, entry in enumerate(entries) if entry.get("PUBSUB_TYPE") == 783]
    if len(slots) != 1:
        raise ValueError("competition network must have one replaceable PUBSUB_TYPE=783 slot")
    entries[slots[0]] = scenario[0]
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps(base, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if MR_NETWORK_DIR.is_dir():
        installed = MR_NETWORK_DIR / OUTPUT.name
        shutil.copy2(OUTPUT, installed)
        return installed
    return OUTPUT


if __name__ == "__main__":
    print(build())
