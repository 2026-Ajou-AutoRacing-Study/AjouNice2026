"""Mission-local teammate guides rendered from the actual generated inputs.

These are authoring hints, not claims of live driving or actor validation.
"""
from pathlib import Path


PROFILES = {
    "roundabout": {
        "title": "회전교차로",
        "scope": "회전교차로 접근부터 진입·통과까지, 순환 차량을 보고 기다리거나 진행하는 판단을 봅니다.",
        "sources": ["configs/scenario/perception_benchmark/bench_roundabout_raw.json",
                    "configs/scenario/2026_molit_comp_sample_scene.json"],
        "anchor": "Ego는 기존 로터리 평가 장면의 접근 위치입니다. 목적지는 기존 전체 코스에서 시작점 이후 약 100m를 따라 선택했습니다. NPC는 샘플 spawn ID 4의 위치·시작/종료 링크를 유지했습니다.",
        "design": "00은 NPC 없는 기준선입니다. 01은 K5 한 차종, 목표 20km/h, 초기 0km/h, 5초 간격입니다. 나머지는 목표 15/20/25/30/35/40km/h와 간격 3/5/7/10/14초, 초기 속도를 조합합니다. 각 JSON 안에서는 차종 하나로 고정하고 JSON 사이에서 차종을 바꿉니다. 16–23의 MinLatBias/MaxLatBias는 ±0.35%입니다 (m 단위가 아닙니다).",
        "extensions": ["먼저 같은 차종·속도에서 spawn 간격만 바꿔, 기다림과 진입 시점이 달라지는지 보세요.",
                       "다음에는 간격을 고정하고 속도만 바꾸세요. 초기 0과 이미 움직이는 초기 속도도 나눠 보세요.",
                       "고정 차종만 바꾼 대조 장면을 만들어 큰 차체나 다른 외형에서도 동일하게 대응하는지 보세요."],
        "guards": ["순환 경로·종료 링크를 임의 도로로 바꾸거나, 한 JSON에서 여러 차종을 무작위로 섞는 것을 기본 시험으로 삼지 않습니다.",
                   "주최측은 본선 로터리 속도가 샘플과 다르고 차종은 비공개 고정이라고 답했습니다. 예제 수치와 차종은 검증용 가정입니다."],
        "observe": "NPC가 실제 순환 경로로 들어오는지, Ego가 gap을 보고 진입하는지, 오래 기다린 FAIL인지 충돌 FAIL인지 확인하세요. 무차량 기준선조차 실패하면 route·측위·제어부터 구분합니다.",
        "prompt": "샘플 spawn ID 4의 위치·순환/종료 링크는 유지하고, JSON별 고정 차종·목표속도·초기속도·spawn 간격 중 한두 조건만 바꿔 줘. 로터리 속도와 차종은 공식 확정값으로 쓰지 마.",
    },
    "highway": {
        "title": "고속주행도로",
        "scope": "고주로 합류 접근부터 합류 후 주행까지, 주변 차량과의 간격·상대속도 변화에 판단이 어떻게 대응하는지 봅니다.",
        "sources": ["configs/scenario/perception_benchmark/bench_highway_merge_raw.json",
                    "configs/scenario/2026_molit_comp_sample_scene.json"],
        "anchor": "Ego는 기존 고주로 합류 평가 장면의 접근 위치입니다. 목적지는 기존 전체 코스에서 시작점 이후 약 220m를 따라 선택했습니다. NPC는 샘플 spawn ID 6/8/10의 위치·시작/종료 링크를 유지했습니다.",
        "design": "00은 NPC 없는 기준선입니다. 01은 spawn ID 10/6/8에 각각 목표 40/50/60km/h, 초기 0km/h, 10초 간격을 줍니다. 나머지는 목표 30/40/50/60/70/80km/h를 spawn별로 순환 배정하고 간격 3/5/7/10/14초, 초기 0/10/20/목표속도를 조합합니다. 16–23의 MinLatBias/MaxLatBias는 ±0.35%입니다 (m 단위가 아닙니다). RandomMode의 MiddleSize 필터 [2]를 유지하지만 실제 spawn 차종은 실행마다 달라질 수 있습니다.",
        "extensions": ["같은 목표속도에서 spawn 간격만 바꿔 넓은 gap·보통 gap·좁은 gap을 대조하세요.",
                       "같은 간격에서 차선별 목표속도 조합을 바꿔 보세요. 특정 차선이 항상 가장 빠르다는 가정을 넣지 마세요.",
                       "초기 0으로 가속하는 장면과 초기=목표로 순항하는 장면을 구분하세요. Ego가 지나갈 때 NPC가 실제로 만나는지도 보세요."],
        "guards": ["현재 베이스의 NPC는 자동 차선 변경 off입니다. 끼어들기·Ego 100–120km/h 추월을 모두 검증하는 세트는 아닙니다. 그런 시험은 별도 경로·타이밍·목적지 검토가 필요합니다.",
                   "주최측은 본선 속도가 샘플과 다르고 RandomMode 사용은 미정, 사용한다면 MiddleSize라고 답했습니다. 30–80km/h는 공식 범위가 아닙니다."],
        "observe": "Ego가 NPC와 실제로 만났는지, 밀도 때문에 합류를 기다렸는지, 합류 후 경로가 생기는지 확인하세요. RandomMode 재실행은 동일 차종을 보장하지 않으므로 영상과 생성된 객체를 함께 봅니다.",
        "prompt": "샘플 spawn ID 6/8/10의 위치·주행/종료 링크와 MiddleSize 필터를 유지하고, 목표속도·초기속도·spawn 간격을 단계적으로 바꿔 줘. 현재 베이스에 없는 차선 변경/고속 추월을 구현됐다고 주장하지 마.",
    },
    "static_obstacle": {
        "title": "일반 정적 장애물 회피",
        "scope": "GPS 음영이 아닌 일반 정적 미션에서, 장애물 종류·좌우 위치·개수에 따라 경로가 생기는지와 회피 후 복귀를 봅니다.",
        "sources": ["configs/scenario/2026_molit_comp_sample_scene.json",
                    "configs/scenario/perception_benchmark/bench_static_obstacle_base3_assets_raw.json",
                    "base_ws/src/planning_addons/planning_scenario_validator/scripts/build_static_corridor_cases.py"],
        "anchor": "샘플 CargoBox가 있는 정적 구간을 사용합니다. Ego는 그 앞 도로상의 기존 검증 시작점이며 목적지는 장애물 이후 기존 코스의 점입니다. 관련 빌더의 코스 거리 표기는 시작 약 285m, 기본 목표 약 355m입니다.",
        "design": "00은 무장애물, 01은 샘플 CargoBox 위치·scale을 그대로 둡니다. 일반 변형은 MORAI x=-60.61 주변 ±0.65m, y=-159.25에서 객체마다 약 17m 간격으로 배치합니다. CargoBox·PE_Drum·TrafficBarrel·Concrete_Barricade01·PE_Firewall_Orange를 1–3개 조합합니다. 일부 장면에는 정지 차량, 마지막 두 장면에는 스쿠터/자전거도 포함합니다. 01을 제외한 생성 객체 scale은 1입니다.",
        "extensions": ["객체 하나로 시작해서 같은 위치의 종류만 바꾸고, 이후 같은 객체의 좌우 위치만 바꿔 보세요.",
                       "단독 배치 결과를 남긴 뒤 두 객체의 좌우 엇갈림과 종방향 간격을 바꿔 보세요. 실제 차체가 통과할 공간이 있는지도 구분하세요.",
                       "정지 차량·학습에서 드문 객체를 추가할 수 있습니다. 너무 가까운 시작 배치와 완전 봉쇄는 별도 스트레스 케이스로 표시하세요."],
        "guards": ["학습 데이터 생성처럼 아무 위치에나 무작위 배치하지 않습니다. 회피 불가능한 완전 봉쇄에서 정지하는 것은 합리적인 대응일 수 있으며, timeout을 곧바로 판단 버그라고 단정하지 않습니다.",
                   "WoodBox는 확인 전까지 제외합니다. 새 객체는 asset 지원·실제 크기를 확인합니다. Object_topic.size만으로 scale/충돌 여유가 맞다고 가정하지 않습니다."],
        "observe": "장애물 검출→track→정지/회피 궤적→제어 순으로 보세요. GT 대조도 같은 위치에서 실패하면 인지 단독 원인으로 단정하지 않습니다. 차선 침범이나 회피 후 복귀도 영상을 확인합니다.",
        "prompt": "샘플 정적 구간의 시작점·기본 목적지를 유지하고 좌우 위치·종류·개수·간격을 한두 조건씩 바꿔 줘. 단독 대조와 다객체를 함께 준비하고, 완전 봉쇄는 별도 표시해 줘. scale=1, WoodBox 제외를 유지해 줘.",
    },
    "gps_shadow": {
        "title": "GPS 음영 구간 정적 장애물 회피",
        "scope": "GPS 음영을 통과하면서 정적 장애물·정지 차량을 회피할 때 측위와 판단·제어가 함께 유지되는지 봅니다. 동적 끼어들기 미션이 아닙니다.",
        "sources": ["configs/scenario/localization_evaluation/localization_shadow_entry_clear.json",
                    "configs/scenario/hidden_mission/static_2/",
                    "configs/scenario/hidden_mission/static_3/"],
        "anchor": "기존 음영 진입 측위 평가 시작점과 기존 음영 정적 미션 YAML의 목적지를 사용합니다. 샘플 음영 영역은 유지하고 다른 미션의 spawn 교통은 제거했습니다.",
        "design": "00은 장애물 없는 음영 기준선입니다. 01–02는 지도 링크 A2256W000126의 0.35 위치 주변 CargoBox, 03–05는 같은 구간의 정지 차량입니다. 06–23은 기존 hidden_mission/static_2와 static_3의 18개 배치를 사용하며 Ego 시작점은 공통으로 맞췄습니다. 출처는 manifest의 parameters.source에 있습니다.",
        "extensions": ["동일 정적 객체를 좌우로 소폭 옮기거나 한 개/두 개 배치를 대조하세요. 음영 경계와 배우 위치를 먼저 확인하세요.",
                       "같은 JSON을 localization과 maptf_morai의 별도 YAML로 대조해 측위와 판단 원인을 분리하세요. GT 측위 PASS를 대회 측위 PASS로 집계하지 않습니다.",
                       "음영 전·중·후 위치와 steering을 함께 비교하세요. GNSS가 끊긴 뒤와 복구된 뒤의 실패를 나눠 보세요."],
        "guards": ["끼어들기·움직이는 차량을 기본 미션에 넣지 않습니다. 기존 정지 차량을 움직이게 하려면 별도 실험임을 표시합니다.",
                   "음영 영역을 줄이거나 localization의 heading/시간 보정을 바꿔 PASS시키지 않습니다. maptf_morai는 원인 분리용이고 본선 대체 측위가 아닙니다."],
        "observe": "실제 GNSS 음영이 적용되는지, 차량이 진입·통과하는지와 측위 jump·복구를 봅니다. 무장애물에서도 흔들리면 장애물 회피 문제와 별개로 기록합니다.",
        "prompt": "공식 음영 영역과 현재 시작점·목적지를 유지하고 정적 장애물/정지 차량만 변형해 줘. 끼어들기는 넣지 마. 같은 JSON의 localization/GT 측위 대조 YAML을 분리하고, 측위 파라미터는 수정하지 마.",
    },
    "dynamic_obstacle": {
        "title": "사거리 동적 장애물 대응",
        "scope": "샘플 사거리에서 횡단·교차 객체를 보고 감속/정지하고, 통과 후 다시 출발하는 판단을 봅니다. GPS 음영 미션과는 분리했습니다.",
        "sources": ["configs/scenario/2026_molit_comp_sample_scene.json"],
        "anchor": "Ego는 지도 링크 A2256W000202의 0.40 위치, 목적지는 A2256W000315의 0.42 위치에서 도로 방향을 사용했습니다. 보행자는 샘플 PedPath - 3를 쓰고, 교차 차량은 기존 도로 링크 A2256W001124 또는 A2256W000636에 둡니다.",
        "design": "00은 무배우, 01은 Man1·샘플 횡단 경로·속도 설정 10·activeDistance 15의 기준선입니다. 02–11은 보행자 모델·방향·속도 설정 3/5/7/10·activeDistance 15/20/25/30을 바꿉니다. 12–17은 같은 횡단 경로의 스쿠터/자전거, 18–23은 목표 10/20/30km/h의 교차 차량이며 마지막 두 장면에는 보행자를 함께 둡니다. 12–23의 배우 이동은 특히 실기 확인이 필요합니다.",
        "extensions": ["같은 보행자 경로에서 방향만 바꾸고, 다음에는 속도/activeDistance를 하나씩 바꿔 Ego와 실제로 만나는 시점을 확인하세요.",
                       "보행자 단독과 교차 차량 단독을 먼저 확인한 뒤 동시에 등장하는 장면을 추가하세요.",
                       "스쿠터나 다른 동적 객체도 추가할 수 있지만 경로 연결·trigger·animation을 확인하세요. 속도 필드만 채워서는 이동이 보장되지 않습니다."],
        "guards": ["pedestrian.speed, object.moveSpeed, vehicle.desiredVelocity를 같은 단위·동작으로 추측하지 않습니다. MORAI UI/문서와 실제 이동을 확인합니다. activeDistance는 시간(초)이 아니라 거리 조건입니다.",
                   "이동 배우가 멈춰 있거나 Ego가 지나간 뒤에만 움직이면 동적 대응 성공으로 집계하지 않습니다. 초기 actor/waypoint unique ID 연결과 좌표 변환을 유지합니다."],
        "observe": "객체 이동 시작·Ego 조우·정지·객체 통과·재출발을 순서대로 보세요. 미출발 FAIL이 경로 미생성인지 객체가 실제로 길을 막는지 구분하세요. 신호등 off 기준이므로 신호등 통합 시험은 별도입니다.",
        "prompt": "샘플 사거리와 PedPath - 3 또는 기존 교차 도로 링크를 기반으로 방향·속도·activeDistance를 단계적으로 바꿔 줘. 새 배우 이동은 미검증으로 표시하고 실제 조우를 확인할 수 있게 해 줘. activeDistance를 초 단위 trigger로 해석하지 마.",
    },
}


def render_guide(family, rows, campaign, geometry, root, package):
    profile = PROFILES[family]
    initials = [row[1]["egoVehicle"]["initPosition"] for row in rows]
    first = initials[0]
    # Do not silently document one anchor if future scenarios move the ego.
    for initial in initials[1:]:
        if any(initial.get(key) != first.get(key) for key in ("pos", "rot", "initLink", "initLinkRatio")):
            raise ValueError(f"{family}: 시작점이 달라졌습니다. 미션 안내의 좌표 표를 검토하세요")
    position = first["pos"]
    yaw = float(first["rot"]["yaw"])
    start_map = geometry.goal([position["x"], position["y"], position["z"], yaw])
    goal = campaign["default_case"]["goal"]
    prefix = rows[0][0].rsplit("_", 2)[0]
    case_names = [row[0] for row in rows]
    here = "base_ws/src/planning_addons/planning_scenario_validator/config/missions/" + family
    package_path = str(Path(package).relative_to(root))
    sources = "\n".join("- `" + source + "`" for source in profile["sources"])
    extensions = "\n".join(f"{i}. {item}" for i, item in enumerate(profile["extensions"], 1))
    guards = "\n".join("- " + item for item in profile["guards"])
    alternatives = "다른 좌표 후보를 임의로 제공하지 않았습니다. 합류만/순항만 등 검증 구간을 바꾸려면 지도에서 연결되는 시작·끝점을 확인하고 별도 YAML로 먼저 한 장면을 실행하세요."
    if family == 'highway':
        alternatives += ("\n\n소수 JSON을 여러 번 돌릴 때는 [random_example.yaml](random_example.yaml)을 GUI에서 선택하고 workspace로 복사하세요. "
                         "기존 high_baseline_01/high_normal_06 두 장면 × 10회 = 총 20회이며, NPC 목표 속도 50–60km/h와 path offset −10–10%를 회차마다 추출합니다. "
                         "위치·주행/종료 링크·MiddleSize 필터와 초기 속도는 원본대로 유지합니다. 대회 공식 속도 범위는 아닙니다. "
                         "repeat/seed/범위만 YAML 위쪽에서 바꾸면 되고, 랜덤을 끄려면 enabled: false입니다. "
                         "실제로 로드한 scenario.json과 추출값이 input.json/결과 JSON에 남습니다. 같은 seed/JSON/회차는 같은 파라미터지만 native RandomMode 차종과 physics까지 고정하지는 않습니다.")
    if family == "static_obstacle":
        candidate_path = Path(package) / "config/missions/static_obstacle/base/static_pair_mission_bevfusion.yaml"
        import yaml
        candidate = yaml.safe_load(candidate_path.read_text())["default_case"]
        c = candidate["goal"]
        alternatives = (f"긴 구간의 기존 후보는 `static_pair_mission_bevfusion.yaml`의 map 목적지 "
                        f"({c['x']}, {c['y']}, {c['z']}), yaw {c['yaw_deg']}°입니다. "
                        f"그 YAML의 주행 제한은 {candidate['timeout_sec']}초입니다. 기존 pair 장면용 조건이며 새 24개 전체에 검증한 목적지가 아닙니다. "
                        "끝쪽 객체까지 보고 싶을 때 별도 YAML에서 시험하세요. 지금 기본 goal과 섞어 실행하지 않습니다.")
    return f"""# {profile['title']} — 시나리오 만들 때 옆에 두는 안내

**[example.yaml](example.yaml)은 공유 원본이므로 바로 실행하지 마세요. GUI에서 선택한 뒤 반드시 `베이스 → 내 작업 폴더 복사`를 누르고, 열린 YAML 경로가 이 미션의 `workspace/`인지 확인하세요.** 이후 설정·실행·배치 추가는 개인 복사본을 사용합니다. `workspace/`의 개인 파일은 Git 제외지만 `base/`의 example 및 공유 비교 예제 YAML은 추적됩니다.

시작점·목적지는 그대로 쓰고 객체 조건부터 바꾸면 됩니다. 공통 설치/실행법은 저장소의 `docs/planning/SCENARIO_VALIDATOR_RUNBOOK.md`, 제작 규칙은 `docs/planning/SCENARIO_VALIDATOR_AUTHOR_GUIDE.md`를 봅니다. 아래 파일 경로는 모두 저장소 루트 기준입니다.

## 1. 무엇을 확인하려고 만들었나

{profile['scope']}

학습 데이터 수집이 아니라 **평범한 상황의 회귀 + 대응하지 못하는 상황 발견**이 목적입니다. 실패하는 장면도 필요한 결과입니다. PASS시키기 위해 판단 파라미터나 장애물을 임의로 바꾸지는 않습니다.

## 2. 기본 시작점과 목적지

이 미션의 24개 JSON은 같은 시작점이고 [example.yaml](example.yaml)은 공통 목적지 하나를 사용합니다. 아래 값은 생성기가 실제 JSON/YAML에서 가져옵니다. 숫자는 표시용으로 소수점 6자리까지 반올림했으므로 실행에는 원본 파일 값을 사용하세요.

| 항목 | 좌표/방향 | 수정하는 파일 |
| --- | --- | --- |
| Ego 시작점, MORAI 좌표 | x={position['x']:.6f}, y={position['y']:.6f}, z={position['z']:.6f}, yaw={yaw:.6f}° | JSON의 egoVehicle.initPosition |
| 같은 시작점, map 좌표 참고 | x={start_map['x']:.6f}, y={start_map['y']:.6f}, z={start_map['z']:.6f}, yaw={start_map['yaw_deg']:.6f}° | 위치 참고용; 입력을 추가하지 않음 |
| 목적지, planning map 좌표 | x={goal['x']:.6f}, y={goal['y']:.6f}, z={goal['z']:.6f}, yaw={goal['yaw_deg']:.6f}° | YAML의 default_case.goal |
| 도로 시작 anchor | initLink={first.get('initLink', '')}, initLinkRatio={first.get('initLinkRatio', 0):.6f} | JSON; 위치만 바꿀 때도 함께 확인 |
| 주행 제한 / 도착 | {campaign['default_case']['timeout_sec']}초 / {campaign['default_case']['arrival_mode']} | YAML의 default_case |

{profile['anchor']}

객체 종류·좌우 위치·개수·속도·spawn 간격만 바꾸고 같은 구간을 통과한다면 시작점·목적지를 유지하세요. 시작점을 바꿔도 같은 목적지까지 연결되면 같은 YAML이 가능하지만 경로·방향·주행시간을 다시 확인해야 합니다. 조건별 설정을 동시에 남겨둘 때는 별도 YAML로 구분합니다.
같은 개인 YAML은 수정·재사용해도 됩니다. GUI의 `새 실험 시작`은 timestamp 결과 폴더와 입력 사본을 만들고, `실험 기록 / 재개`는 당시 사본을 사용합니다. 결과 폴더의 입력 사본은 직접 수정하지 마세요.

{alternatives}

MORAI 객체 좌표를 goal에 그대로 복사하지 마세요. 현재 변환은 `configs/tf/morai_lanelet_static_tf.yaml`의 map_to_morai_map이며 map = R × MORAI + translation입니다. 지도/TF가 바뀌면 다시 확인합니다. 위 yaw는 지도상의 배치 방향이지 localization heading 보정 파라미터가 아닙니다.

## 3. 기존 24개를 어떻게 배치했나

기준선 2개(`{prefix}_baseline_*`) → 일반 변형 14개(`{prefix}_normal_*`) → stress 8개(`{prefix}_stress_*`)입니다. normal/stress는 설계상 분류이지 성공/실패 예측이나 대회 출제 확률이 아닙니다.

{profile['design']}

정확한 장면별 배우·파라미터·출처는 [manifest.json](manifest.json), 실제 설정은 [scenarios/](scenarios/)에서 봅니다. "baseline"도 풀스택 주행 성공을 보장하지 않습니다. 01은 샘플 조건을 참고한 단순 대조이며 원본 본선 시나리오 전체를 복제한 것은 아닙니다.

근거 파일:

{sources}
- `kcity_map/mgeo_map/link_set.json` — 도로 링크·진행 방향
- `configs/global_path/2026_molit_comp_global_path.txt` — 기존 코스
- `configs/tf/morai_lanelet_static_tf.yaml` — 좌표 변환
- `{package_path}/scripts/build_mission_starters.py` — 실제 생성 규칙

## 4. 팀원이 추가해 볼 순서

{extensions}

처음에는 기존 JSON 하나를 `../workspace/scenarios/`에 새 이름으로 복사해 한두 조건만 바꾸세요. 일반 상황을 충분히 두고 스트레스를 섞습니다. YAML은 GUI의 베이스 복사로 준비합니다. 이름·파일 패턴/장면 선택·반복·측위/탐지·신호등·녹화/공유는 기본 탭에서 수정하고 상단 파란 YAML 저장을 누릅니다. 같은 개인 파일에 저장되며 날짜 파일을 새로 만들지 않습니다. ‘여기부터 수정 X’ 아래 목적지·시간·네트워크와 회색 GUI 값은 그대로 유지하세요. 랜덤 범위·Python 환경은 VSCode에서만 수정합니다. 빈 YAML·장면별 .case.yaml은 만들지 않습니다. 경로는 YAML 옆 기준이 아니라 저장소 루트 기준입니다.

주의할 점:

{guards}

## 5. 실행 후 무엇을 보면 되나

먼저 `{case_names[0]}.json`과 `{case_names[1]}.json`을 파일 패턴 `{prefix}_baseline_*.json`으로 확인하세요. 새 배우 장면도 대표 한두 개는 직접 보고 나서 전체를 걸어둡니다.

{profile['observe']}

현재 이 24개는 **형식·생성 재현성 검증만 완료, 전체 live 주행은 미검증**입니다. 기존 정적 pair의 GT/BEV 주행 검증을 이 묶음의 성공률로 확대하지 않습니다. 무장애물 기준선·배우 동작·goal route를 실제로 확인해야 합니다.

기본은 localization + BEVFusion + 신호등 off, basic bag + 원시 센서 녹화 제외 + 영상 on입니다. 카메라/LiDAR 입력 자체는 켜둡니다. 신호등 on 시험은 별도 YAML로 분리하세요. `od_gt`는 같은 JSON의 인지 원인 분리용 대조이고, `maptf_morai`는 측위 원인 분리용 대조입니다.

현재 자동 PASS/FAIL은 충돌·목적지·시간 조건 중심입니다. 양보·차선 침범·규정 속도·배우가 실제로 만났는지는 전부 자동 판정하지 않습니다. 도착했다고 미션 전체가 올바른 것은 아니며, timeout이라고 모두 판단 버그인 것도 아닙니다. `summary.csv`와 해당 run JSON/이벤트/영상에서 관측 근거를 함께 봅니다. 분석기의 원인 설명도 확정 원인이 아니라 근거에 기반한 후보로 확인하세요.

개인 파일은 workspace에서 Git 제외됩니다. 녹화는 `planner_logs/validation/<실험 이름>_<YYMMDD_HHMMSS>/`에 남습니다. GUI 복사 버튼이 만드는 workspace README는 이 원본 안내로 연결하는 참고 문서이며 개인 JSON 수정 내용을 자동 추적하지 않습니다.

## 6. 담당 AI에게 복붙

```text
{profile['title']} 판단 검증용 개인 시나리오와 완성된 v2 YAML을 만들어 줘.
먼저 {here}/base/README.md,
같은 base의 example.yaml·manifest.json·대표 baseline/normal/stress JSON,
docs/planning/SCENARIO_VALIDATOR_AUTHOR_GUIDE.md를 읽어 줘.

{profile['prompt']}
공유 base와 planning/control/localization 코드·파라미터는 수정하지 말고
{here}/workspace/ 아래에 개인 JSON과 YAML을 만들어 줘.
기본 시작점·목적지를 우선 유지해 줘. YAML 하나는 공통 목적지·시간·스택 하나야.
조건별 설정을 함께 남길 때는 다른 YAML을 만들고, 같은 YAML 수정 후에는 새 실험을 시작해 줘.
재개는 현재 원본이 아니라 결과 폴더에 보관된 입력 사본을 사용해 줘.
일반 사례와 어려운 사례를 섞되 처음에는 한두 장면으로 시험하게 해 줘.
asset/DataID·도로 링크·좌표를 파일에서 확인하고 추측하지 마.
새 장면마다 의도·변경 조건·미검증 배우 동작을 개인 문서로 남겨 줘.
자동 PASS/FAIL과 실제 미션 적절성을 구별하고 근거 없는 원인 단정은 하지 마.
생성만 하고 실제 주행했다고 주장하지 마. 실주행에는 MORAI 연결과 별도 확인이 필요해.
```

이 안내는 생성기와 함께 관리합니다. 공유 좌표·시나리오를 바꿀 때는 생성기를 수정하고 `build_mission_starters.py --check`로 문서까지 일치하는지 검사하세요. **이 README는 agent 행동을 강제하는 AGENTS.md가 아니라 팀원용 작업 참고 문서입니다.**
"""
