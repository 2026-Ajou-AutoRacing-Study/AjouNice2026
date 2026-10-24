"""Prepare temporary LAN-to-LAN development copies, not competition profiles."""
import argparse
import copy
import ipaddress
import json
from pathlib import Path

import yaml

from .authoring import updated_yaml
from .network_contract import (_active_network_udp, _active_sensor_udp,
                               inspect_network_contract)


ROOT = Path(__file__).resolve().parents[6]
PACKAGE = Path('base_ws/src/planning_addons/planning_scenario_validator')


def _address(value):
    address = ipaddress.IPv4Address(value)
    if (address.is_loopback or address.is_unspecified or address.is_multicast
            or str(address) == '255.255.255.255'):
        raise ValueError('LAN PC의 실제 IPv4 주소를 사용하세요: {}'.format(value))
    return str(address)


def patch_lan_addresses(document, ego_network, sensor_config, morai_ip, ros_ip):
    """Change only active UDP addresses and YAML routing, not geometry or ports."""
    morai_ip, ros_ip = _address(morai_ip), _address(ros_ip)
    if morai_ip == ros_ip:
        raise ValueError('두 PC LAN 설정에는 서로 다른 MORAI/ROS IP가 필요합니다')
    document, ego_network, sensor_config = copy.deepcopy(
        (document, ego_network, sensor_config))
    rows = list(_active_network_udp(ego_network))
    entries = ego_network.get('egoNetworkData', [{}])[0].get('listEgoNetworkInfo', [])
    scenario = [udp for kind, key, udp in rows if kind == 783 and key == 'config']
    if (sum(entry.get('PUBSUB_TYPE') == 783 for entry in entries) != 1
            or len(scenario) != 1 or scenario[0].get('hostPort') != 9095
            or scenario[0].get('destinationPort') != 9096):
        raise ValueError('기존 YAML에 validator용 Ego Network(ScenarioLoad 9095→9096)가 '
                         '필요합니다. 시나리오 자동 로드가 없는 일반 주행용 Ego Network로 대신하지 마세요')
    sensors = list(_active_sensor_udp(sensor_config))
    if not sensors:
        raise ValueError('Sensor JSON에 활성 UDP 센서가 없습니다')
    for udp in [row[2] for row in rows] + sensors:
        udp['hostIP'], udp['destinationIP'] = morai_ip, ros_ip
    document.setdefault('morai', {})['scenario_load_host'] = morai_ip
    report = inspect_network_contract(document, ego_network, sensor_config)
    if report['errors']:
        raise ValueError('\n'.join(report['errors']))
    return document, ego_network, sensor_config, report


def _resolve(value, root):
    path = Path(value).expanduser()
    return (path if path.is_absolute() else root / path).resolve()


def _personal_yaml(path, root):
    try:
        parts = path.relative_to(root / PACKAGE / 'config/missions').parts
    except ValueError as error:
        raise ValueError('미션 workspace의 개인 YAML만 사용하세요. base/실행 사본은 수정하지 않습니다') from error
    if (len(parts) != 3 or parts[1] != 'workspace' or path.suffix not in ('.yaml', '.yml')
            or path.name in ('config_snapshot.yaml', 'source_config.yaml')):
        raise ValueError('config/missions/<미션>/workspace 안의 개인 YAML을 선택하세요')


def prepare_lan_bundle(source, morai_ip, ros_ip, *, output=None, root=ROOT):
    """Create sibling YAML+JSONs. Existing differing files are never overwritten."""
    root = Path(root).resolve()
    source = _resolve(source, root)
    _personal_yaml(source, root)
    destination = (_resolve(output, root) if output else
                   source.with_name(source.stem + '_lan_dev.yaml'))
    _personal_yaml(destination, root)
    if destination.parent != source.parent or destination == source:
        raise ValueError('출력은 원본과 같은 workspace의 다른 YAML이어야 합니다')
    text = source.read_text(encoding='utf-8')
    document = yaml.safe_load(text)
    if (not isinstance(document, dict) or document.get('schema_version') != 2
            or not isinstance(document.get('network'), dict)):
        raise ValueError('schema_version: 2인 개인 실행 YAML이 필요합니다')
    network = document['network']
    ego = json.loads(_resolve(network['ego_network_json'], root).read_text(encoding='utf-8'))
    sensor = json.loads(_resolve(network['sensor_json'], root).read_text(encoding='utf-8'))
    document, ego, sensor, report = patch_lan_addresses(
        document, ego, sensor, morai_ip, ros_ip)
    directory = destination.with_name(destination.stem + '_network')
    if directory.is_symlink() or directory.resolve().parent != destination.parent:
        raise ValueError('Network 출력 폴더는 같은 workspace 안의 실제 폴더여야 합니다')
    sensor_path, ego_path = directory / 'sensor_lan_dev.json', directory / 'ego_network_lan_dev.json'
    for key, path in (('sensor_json', sensor_path), ('ego_network_json', ego_path)):
        document['network'][key] = str(path.relative_to(root))
    payloads = {
        sensor_path: json.dumps(sensor, ensure_ascii=False, indent=2) + '\n',
        ego_path: json.dumps(ego, ensure_ascii=False, indent=2) + '\n',
        destination: updated_yaml(text, document),
    }
    # Check every destination before writing any file; rerunning unchanged is safe.
    for path, payload in payloads.items():
        if path.is_symlink() or (path.exists() and (
                not path.is_file() or path.read_bytes() != payload.encode('utf-8'))):
            raise ValueError('기존 파일을 덮어쓰지 않습니다: {}. 다른 --output 이름을 사용하세요'.format(path))
    for path, payload in payloads.items():
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open('x', encoding='utf-8') as handle:
                handle.write(payload)
    return {'yaml': destination, 'sensor': sensor_path, 'ego_network': ego_path,
            'network_contract': report}


def main(argv=None):
    parser = argparse.ArgumentParser(description='LAN-to-LAN 개발용 임시 Sensor/Ego 사본 준비 (대회용 아님, 연결/주행 없음)')
    parser.add_argument('--yaml', required=True, help='미션 workspace의 기존 개인 YAML')
    parser.add_argument('--morai-ip', required=True, help='MORAI PC IPv4')
    parser.add_argument('--ros-ip', required=True, help='ROS PC IPv4')
    parser.add_argument('--output', help='같은 workspace의 새 YAML 경로 (기본: 원본이름_lan_dev.yaml)')
    args = parser.parse_args(argv)
    try:
        result = prepare_lan_bundle(args.yaml, args.morai_ip, args.ros_ip, output=args.output)
    except (ValueError, OSError, KeyError, TypeError, IndexError, yaml.YAMLError) as error:
        print('LAN 준비 실패: {}'.format(error))
        return 2
    print('개발용 임시 사본입니다. 대회에서는 검증된 현장점검 실행 절차와 설정을 그대로 사용하세요.')
    print('새 개발 YAML: {}'.format(result['yaml']))
    print('MORAI Sensor Network에 Load: {}'.format(result['sensor']))
    print('MORAI Ego Network에 Load: {}'.format(result['ego_network']))
    print('원본/공용 파일은 그대로입니다. 생성 파일은 workspace 아래 Git 제외입니다.')
    print('두 PC라면 JSON 두 개를 MORAI PC로 옮겨 Load/Connect하세요. 자동 원격 전송은 하지 않습니다.')
    print('Scenario 폴더도 MORAI가 실제 읽는 공유/마운트 경로인지 확인하세요.')
    print('GUI에서 새 YAML을 불러온 뒤 새 실험을 시작하세요. 과거 실행 재개는 기존 IP 사본을 사용합니다.')
    for warning in result['network_contract']['warnings']:
        print('주의: ' + warning)
    print('정적 주소 정합만 확인했습니다. NIC/방화벽/실제 UDP 수신·주행 검사는 별도입니다.')
    return 0
