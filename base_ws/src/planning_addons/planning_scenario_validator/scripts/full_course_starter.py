"""Full-course validation inputs: sample actors, no synthetic dataset placement."""
import copy
import hashlib
import json

import yaml

from planning_scenario_validator.authoring import annotated_yaml
from planning_scenario_validator.traffic_light import DEFAULT_PYTHON


ROUTE_SOURCE = 'base_ws/src/planning_addons/competition_route_requester/config/kcity_v85_competition_route.yaml'


def build_full_course(sample, geometry, template, root, package, output, validate):
    folder = output / 'full_course/base'
    route_path = root / ROUTE_SOURCE
    route = yaml.safe_load(route_path.read_text(encoding='utf-8'))
    route_a = route['route_a']

    def pose(ref):
        return {key: route['poses'][ref][key] for key in ('x', 'y', 'z', 'yaw_deg')}

    goal = pose(route_a['goal_ref'])
    waypoints = [pose(ref) for ref in route_a['waypoint_refs']]
    scenes = [('full_baseline_00_sample', copy.deepcopy(sample), '배포 샘플 전체를 그대로 보존')]
    vehicles = copy.deepcopy(sample)
    models = (30100004, 30100007, 30200014)  # Catalog: K5 / Ioniq / Ioniq5.
    for index, actor in enumerate(vehicles['vehicleList']):
        actor['DataID'] = models[index % len(models)]
    for spawn in vehicles['spawnPointList']:
        if spawn['UNIQUEID'] == 4:
            spawn['spawnVehicleTypeID'] = 30100007
    scenes.append(('full_normal_01_vehicle_types', vehicles,
                   '직접 배치 차량 외형과 로터리 고정 차종만 변경; 고주로 MiddleSize RandomMode 유지'))
    obstacles = copy.deepcopy(sample)
    for actor in obstacles['objectList']:
        if actor['DataID'] == 40100001:
            actor['DataID'] = 40100005  # TrafficBarrel at the sample CargoBox pose.
            actor['scale'] = {'x': 1.0, 'y': 1.0, 'z': 1.0,
                              '_x': '1.000', '_y': '1.000', '_z': '1.000'}
    scenes.append(('full_normal_02_obstacle_types', obstacles,
                   '샘플 CargoBox 위치에 scale 1 TrafficBarrel; 나머지 조건 유지'))
    mixed = copy.deepcopy(vehicles)
    mixed['objectList'] = copy.deepcopy(obstacles['objectList'])
    for actor in mixed['pedestrianList']:
        if actor['DataID'] == 50100001:
            actor['DataID'] = 50100016  # Woman8, same unique ID and waypoint path.
    scenes.append(('full_normal_03_mixed_types', mixed,
                   '차량·정적 장애물·보행자 종류 변경; 배치·속도·트리거·경로 유지'))
    expected, metadata = {}, []
    for stem, scene, description in scenes:
        validate(scene, geometry)
        expected[folder / 'scenarios' / (stem + '.json')] = json.dumps(scene, ensure_ascii=False, indent=2) + '\n'
        metadata.append({'name': stem, 'description': description, 'live_verified': False})
    campaign = copy.deepcopy(template)
    campaign.update(schema_version=2, campaign_name='starter_full_course_bev')
    campaign['scenarios'] = {'directory': str((folder / 'scenarios').relative_to(root)),
                             'glob': '*.json', 'repeat': 1}
    campaign['default_case'] = {'goal': goal, 'waypoints': waypoints,
                                'timeout_sec': 600, 'arrival_mode': 'stop'}
    campaign['stack'].update(pose_source='localization', detector='bevfusion', traffic_light=True,
        bevfusion_model='base5_e24', traffic_light_runtime='virtualenv',
        traffic_light_python=DEFAULT_PYTHON, traffic_light_device='0')
    campaign['recording'].update(include_sensors=False, minimum_free_disk_gib=10)
    campaign['recording']['video'].update(enabled=True, window_layout='split', focus_planviz=False,
                                         bottom_bar_px='auto', fps=10)
    campaign['upload'] = {'enabled': False, 'rclone_remote': 'ajou_drive:planning_validator', 'profile': 'basic'}
    expected[folder / 'example.yaml'] = annotated_yaml(campaign)
    single = copy.deepcopy(campaign)
    single['campaign_name'] = 'full_course_sample_only'
    single['scenarios']['include'] = ['full_baseline_00_sample.json']
    expected[folder / 'sample_only.yaml'] = annotated_yaml(single)
    for suffix, include in (('sample', ['full_baseline_00_sample.json']),
                            ('variants', [stem + '.json' for stem, _, _ in scenes[1:]])):
        analysis = copy.deepcopy(campaign)
        analysis['campaign_name'] = 'full_course_base5_e24_' + suffix
        analysis['scenarios']['include'] = include
        analysis['stack']['bevfusion_model'] = 'base5_e24'
        analysis['recording'].update(analysis_mode=True, include_sensors=True,
            topic_profile='base_ws/src/planning_addons/planning_scenario_validator/config/internal/topic_profiles/perception_analysis.json')
        expected[folder / ('base5_e24_analysis_' + suffix + '.yaml')] = annotated_yaml(analysis)
    expected[folder / 'manifest.json'] = json.dumps({
        'schema_version': 1, 'mission': 'full_course', 'live_verified': False,
        'source_sample': 'configs/scenario/2026_molit_comp_sample_scene.json',
        'source_sample_sha256': hashlib.sha256((root / 'configs/scenario/2026_molit_comp_sample_scene.json').read_bytes()).hexdigest(),
        'route_source': ROUTE_SOURCE, 'route_source_sha256': hashlib.sha256(route_path.read_bytes()).hexdigest(),
        'route_kind': 'Route A waypoint request only; no requester preferred-primitive or A/B switching policy',
        'waypoint_refs': route_a['waypoint_refs'], 'goal_ref': route_a['goal_ref'],
        'goal_map': goal, 'cases': metadata,
        'official_distribution': 'sample variants, not confirmed competition distribution',
    }, ensure_ascii=False, indent=2) + '\n'
    start = sample['egoVehicle']['initPosition']['pos']
    expected[folder / 'README.md'] = f"""# 전체 코스 주행

미션을 따로 자르지 않고 샘플 시작점에서 주요 코스를 연속 주행합니다. 학습 데이터 생성용이 아니라 풀스택 회귀·실패 사례 확인용입니다.

## 처음 실행

1. GUI의 **내 미션 선택하기 → 전체 코스 주행**을 고릅니다.
2. 처음에는 **예제 / sample_only.yaml**을 선택하고 **베이스 → 내 작업 폴더 복사 (최초 1회 실행)**를 누릅니다.
3. localization / BEVFusion / 신호등 on을 확인하고 **YAML 저장 → 설정 검사 → 새 실험 시작**합니다.
4. 샘플 하나의 연결·배우·경로·녹화를 확인한 뒤 **예제 / example.yaml**도 복사하면 네 장면을 순서대로 실행합니다.

[example.yaml](example.yaml): 전체 4장면. [sample_only.yaml](sample_only.yaml): 샘플 1장면.
E24 인지 분석은 **[base5_e24_analysis_sample.yaml](base5_e24_analysis_sample.yaml)**로 샘플 하나부터 확인합니다. 연결·route·배우·녹화 마감을 확인한 뒤 **[base5_e24_analysis_variants.yaml](base5_e24_analysis_variants.yaml)**로 나머지 세 장면을 시험합니다. 모두 workspace로 복사한 뒤 실행하세요. example/sample_only도 E24 production을 사용합니다.
개인 파일은 `../workspace/`에 생기며 Git에서 제외됩니다. 새 YAML·JSON도 여기에 넣고 새로고침하세요.
공통 사용 순서는 저장소 `docs/planning/SCENARIO_VALIDATOR_WORKFLOW.md`를 참고합니다.

## 시작점·종료점·경로

- Ego 시작점은 샘플 JSON 그대로: MORAI x={start['x']:.6f}, y={start['y']:.6f}, z={start['z']:.6f}.
- 목적지 map: x={goal['x']:.6f}, y={goal['y']:.6f}, z={goal['z']:.6f}, yaw={goal['yaw_deg']:.6f}°.
- 기존 v85 Route A의 경유점 {len(waypoints)}개와 비순환 종료점 `{route_a['goal_ref']}`를 사용합니다.
- 시작점과 결승선이 겹치는 코스라 **출발선 재진입 직전 종료**합니다. 정확한 결승선 통과/공식 완주 판정 실험은 아닙니다.
- validator는 경유점들을 한 번의 SetWaypointRoute로 요청합니다. competition_route_requester의 선호 차선 지정·Route A/B 전환·fallback 정책까지 실행하는 구성은 아닙니다. 그 정책 시험은 별도 통합이 필요합니다.
- 주행 제한은 장면당 **600초**입니다. 짧은 개별 미션 제한시간과 달리 약 2.2km 코스·신호 대기를 고려한 첫 상한이며 공식 제한시간은 아닙니다. 준비·마감 시간은 별도입니다.

## 시나리오 구성

| JSON | 변경점 |
| --- | --- |
| full_baseline_00_sample | 배포 샘플 그대로. Runtime에서 Ego ExternalCtrl 정지 시작·Network sidecar만 준비 |
| full_normal_01_vehicle_types | 직접 배치 차량은 K5/Ioniq/Ioniq5로 변경, 로터리 spawn 차종은 Ioniq |
| full_normal_02_obstacle_types | CargoBox 위치에 scale 1 TrafficBarrel |
| full_normal_03_mixed_types | 차량·장애물 변형 + 같은 경로의 Woman8 |

샘플의 spawn 위치·시작/종료 링크, NPC 속도, 횡단 트리거, 신호·음영 영역은 유지합니다. 고주로 RandomMode MiddleSize도 유지합니다. CargoBox 원본 scale은 기준선에만 그대로 남기고 새 장애물 변형은 scale 1입니다. WoodBox는 넣지 않습니다.
배우 종류만 바꿔도 차체 크기·동역학·인지 성능이 달라질 수 있습니다. 공식 출제 차종/속도 분포를 복제했다는 뜻은 아닙니다.

## 실행 전 주의

- **전체 live 주행은 미검증**입니다. 생성 재현성과 YAML/JSON 계약 검사는 폐루프 성공 증명이 아닙니다. 먼저 샘플 한 장면에서 route 생성과 NPC 동작을 보세요.
- 기본은 localization + BEVFusion + 신호등 on, basic bag + 카메라/LiDAR 원본 녹화 제외 + 영상 on입니다. 실제 센서는 계속 켜둡니다.
- 별도 E24 분석 YAML은 AMP·카메라 병렬화 on / compile off, V/P/O 0.40/0.10/0.10이며 **센서 원본 + 영상 on**입니다. 용량이 훨씬 큽니다. 원본 센서가 없는 bag은 사후 재추론할 수 없습니다. 서버·bridge는 검증기가 함께 시작/종료하고 canonical detection publisher 중복을 거부합니다.
- 모델은 GUI에서 선택하거나 stack.bevfusion_model로 지정합니다. 항목이 없으면 E24 production입니다. E25는 명시적 복귀, E9는 별도 threshold를 선정한 비교 후보입니다. 모델·config·sensor·스택 설정 hash가 바뀌면 같은 결과에 resume하지 말고 새 실험을 시작하세요.
- 마감 후 recording_audit.json의 토픽별 실제 message_count를 확인합니다. 주행 판정과 녹화 완전성은 별개입니다. /clock·선택 기능·debug 토픽의 미발행도 0으로 남기며 존재하지 않는 데이터를 녹화됐다고 표시하지 않습니다.
- 신호등 가상환경 설치가 필요합니다. 이미 시스템 Python으로 준비했다면 YAML의 stack.traffic_light_runtime을 system으로 바꾸세요. 신호등 off 결과를 정상 풀스택 결과와 섞지 마세요.
- 4장면 모두 제한시간까지 가면 주행만 40분입니다. 초기 준비/로드/마감을 더하면 더 걸립니다. 용량은 장면 하나의 실제 bag·영상 크기로 판단하세요.
- 자동 PASS는 충돌 없음·목적지 정차·제한시간 조건입니다. 경유 구간 실제 통과·차선 침범·규정 속도·미션 수행 적절성까지 모두 판정하지 않습니다. 실패는 위치를 옮겨 없애지 말고 JSON/이벤트/bag/영상의 근거를 봅니다.

## 추가할 때

먼저 샘플 하나를 workspace/scenarios에 복사해 객체 종류 하나만 바꾸세요. 같은 시작점·경유점·목적지를 유지하면 같은 YAML의 패턴/장면 선택으로 전체 또는 일부를 시험할 수 있습니다. 속도·spawn 간격을 바꿀 때는 별도 이름으로 남기며 공식 범위라고 쓰지 마세요.

출처: `configs/scenario/2026_molit_comp_sample_scene.json`, `{ROUTE_SOURCE}`.
실제 구성과 변경 의도는 [manifest.json](manifest.json), [scenarios/](scenarios/)를 봅니다.
생성 규칙은 `scripts/full_course_starter.py`이며 `scripts/build_mission_starters.py --check`로 재현 검사합니다.
"""
    return expected
