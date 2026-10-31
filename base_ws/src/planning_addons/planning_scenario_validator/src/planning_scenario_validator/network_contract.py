"""Static MORAI/validator UDP address consistency, without DNS or live probing.

This checks the settings that will be used, not filenames such as ``*_local``.
It neither changes profiles nor certifies NIC ownership or remote file access.
"""
import ipaddress


def _ipv4(value):
    if not isinstance(value, str):
        return None
    if value.strip().lower() == 'localhost':
        return ipaddress.IPv4Address('127.0.0.1')
    try:
        return ipaddress.IPv4Address(value.strip())
    except ipaddress.AddressValueError:
        return None


def _active_network_udp(ego_network):
    # Match the bridge and load_campaign: only the selected first Ego is used.
    for ego in ego_network.get('egoNetworkData', [])[:1]:
        for entry in ego.get('listEgoNetworkInfo', []):
            if entry.get('netType') != 1:
                continue
            # The control sender uses EgoCtrlConfig; other receivers use config.
            for key in ('config', 'EgoCtrlConfig'):
                config = entry.get(key, {})
                if config.get('commType') == 1 and isinstance(config.get('udpConfig'), dict):
                    yield entry.get('PUBSUB_TYPE'), key, config['udpConfig']


def _active_sensor_udp(sensor_config):
    for category, key in (('cameraList', 'cc'), ('Lidar3DList', 'lc'),
                          ('GPSList', 'gc'), ('IMUList', 'ic')):
        for sensor in sensor_config.get(category, []):
            config = sensor.get(key, sensor)
            if config.get('commType') == 1 and isinstance(config.get('udpConfig'), dict):
                yield config['udpConfig']


def _destinations(rows):
    return {str(row['destinationIP']).strip() for row in rows
            if isinstance(row.get('destinationIP'), str) and row['destinationIP'].strip()}


def inspect_network_contract(document, ego_network, sensor_config):
    """Return static errors/warnings; leave all supplied dictionaries unchanged.

    Different concrete IPv4 ScenarioLoad destinations are a definite mismatch.
    A wildcard bind or hostname cannot prove a mismatch without live context,
    so it receives a warning rather than an unsupported DNS-based rejection.
    """
    errors, warnings = [], []
    morai = document.get('morai', {})
    scenario_host = morai.get('scenario_load_host', '127.0.0.1')
    network_rows = list(_active_network_udp(ego_network))
    scenario_rows = [udp for kind, key, udp in network_rows if kind == 783 and key == 'config']
    scenario_wire_host = scenario_rows[0].get('hostIP') if len(scenario_rows) == 1 else None
    configured_ip, wire_ip = _ipv4(scenario_host), _ipv4(scenario_wire_host)

    if scenario_wire_host is not None:
        if ((configured_ip and configured_ip.is_unspecified)
                or (wire_ip and wire_ip.is_unspecified)):
            warnings.append('ScenarioLoad에 0.0.0.0 wildcard가 있습니다. 이 정적 검사만으로 '
                            '송신 대상과 MORAI 수신 주소의 일치를 인증하지 않습니다.')
        elif configured_ip is not None and wire_ip is not None:
            if configured_ip != wire_ip:
                errors.append(
                    'ScenarioLoad 주소 불일치: morai.scenario_load_host={} 이지만 Ego Network의 '
                    'PUBSUB_TYPE=783 UDP hostIP는 {}입니다. 개인 YAML의 scenario_load_host를 '
                    '실제 MORAI PC 주소({})로 맞추고 새 실험을 시작하세요. '
                    '기존 실행 사본은 수정하지 마세요.'.format(
                        scenario_host, scenario_wire_host, scenario_wire_host))
        else:
            warnings.append('ScenarioLoad 주소에 hostname 또는 IPv4로 확정할 수 없는 값이 있습니다 '
                            '(YAML={}, Network={}). DNS 조회 없이 정적으로 일치를 판정하지 않습니다.'.format(
                                scenario_host, scenario_wire_host))

    if configured_ip is not None and not configured_ip.is_loopback and not configured_ip.is_unspecified:
        if morai.get('scenario_install_dir', 'auto') == 'auto':
            warnings.append('원격 MORAI 주소를 사용하지만 morai.scenario_install_dir=auto입니다. '
                            'auto는 ROS PC의 로컬 폴더만 찾습니다. MORAI Scenario 폴더를 공유/마운트하고 '
                            '그 경로를 명시하세요. IP 설정만으로 원격 파일 전송은 수행되지 않습니다.')

    ego_destinations = _destinations([udp for _, _, udp in network_rows])
    sensor_destinations = _destinations(_active_sensor_udp(sensor_config))
    # Wildcard receiver binds do not imply the wrong NIC. Compare concrete sets.
    concrete_ego = {ip for ip in ego_destinations if ip != '0.0.0.0'}
    concrete_sensor = {ip for ip in sensor_destinations if ip != '0.0.0.0'}
    if concrete_ego and concrete_sensor and concrete_ego != concrete_sensor:
        warnings.append('Sensor/Ego Network의 UDP destinationIP가 다릅니다 '
                        '(Sensor={}, Ego={}). 여러 NIC를 의도한 설정인지 확인하세요. '
                        '이 정적 검사는 실제 NIC 보유·UDP 수신을 인증하지 않습니다.'.format(
                            ', '.join(sorted(concrete_sensor)), ', '.join(sorted(concrete_ego))))

    return {'errors': errors, 'warnings': warnings,
            'scenario_host': scenario_host, 'scenario_wire_host': scenario_wire_host,
            'ros_destination_ips': sorted(ego_destinations | sensor_destinations)}
