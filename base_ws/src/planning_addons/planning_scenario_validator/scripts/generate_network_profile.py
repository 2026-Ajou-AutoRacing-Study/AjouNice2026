#!/usr/bin/env python3
"""Keep competition local/LAN interfaces and add validator UDP ScenarioLoad."""
import json
import argparse
import ipaddress
import shutil
from pathlib import Path


ROOT = Path(__file__).resolve().parents[5]
BASE = ROOT / "configs/network/ego_network_competition_local.json"
SCENARIO_SOURCE = ROOT / "configs/network/legacy/ego_network_local.json"
OUTPUT = ROOT / "base_ws/src/planning_addons/planning_scenario_validator/config/internal/generated/ego_network_validator_local.json"
MR_NETWORK_DIR = Path.home() / "MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Network/25.S4.MolitComp03"
LAN_OUTPUT = OUTPUT.with_name('ego_network_validator_lan.json')


def build(*, lan=False, morai_ip=None, ros_ip=None, install=True):
    if (morai_ip or ros_ip) and not lan:
        raise ValueError('IP overrides require --lan')
    if bool(morai_ip) != bool(ros_ip):
        raise ValueError('--morai-ip and --ros-ip must be specified together')
    for address in (morai_ip, ros_ip):
        if address and ipaddress.ip_address(address).version != 4:
            raise ValueError('IPv4 addresses are required')
    base_path = ROOT / 'configs/network/ego_network_competition_lan.json' if lan else BASE
    scenario_path = ROOT / 'configs/network/legacy/ego_network_1_lan.json' if lan else SCENARIO_SOURCE
    output = LAN_OUTPUT if lan else OUTPUT
    base = json.loads(base_path.read_text(encoding="utf-8"))
    source_network = json.loads(scenario_path.read_text(encoding="utf-8"))
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
    if morai_ip:
        def replace_udp(value):
            if isinstance(value, dict):
                udp = value.get('udpConfig')
                if isinstance(udp, dict):
                    udp.update(hostIP=morai_ip, destinationIP=ros_ip)
                for child in value.values():
                    replace_udp(child)
            elif isinstance(value, list):
                for child in value:
                    replace_udp(child)
        replace_udp(base)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(base, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if install and MR_NETWORK_DIR.is_dir():
        installed = MR_NETWORK_DIR / output.name
        shutil.copy2(output, installed)
        return installed
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lan', action='store_true', help='competition LAN + validator ScenarioLoad')
    parser.add_argument('--morai-ip', help='MORAI PC IPv4 (with --ros-ip)')
    parser.add_argument('--ros-ip', help='ROS PC IPv4 (with --morai-ip)')
    parser.add_argument('--no-install', action='store_true', help='Generate only; do not copy into local MR')
    args = parser.parse_args()
    print(build(lan=args.lan, morai_ip=args.morai_ip, ros_ip=args.ros_ip, install=not args.no_install))
