# 사거리 동적 장애물 대응 — 시나리오 만들 때 옆에 두는 안내

처음에는 [example.yaml](example.yaml)을 GUI에서 열고 **베이스 → 내 작업 폴더 복사**를 누르세요. 시작점·목적지는 그대로 쓰고 객체 조건부터 바꾸면 됩니다. 공통 설치/실행법은 저장소의 `docs/planning/SCENARIO_VALIDATOR_RUNBOOK.md`, 제작 규칙은 `docs/planning/SCENARIO_VALIDATOR_AUTHOR_GUIDE.md`를 봅니다. 아래 파일 경로는 모두 저장소 루트 기준입니다.

## 1. 무엇을 확인하려고 만들었나

샘플 사거리에서 횡단·교차 객체를 보고 감속/정지하고, 통과 후 다시 출발하는 판단을 봅니다. GPS 음영 미션과는 분리했습니다.

학습 데이터 수집이 아니라 **평범한 상황의 회귀 + 대응하지 못하는 상황 발견**이 목적입니다. 실패하는 장면도 필요한 결과입니다. PASS시키기 위해 판단 파라미터나 장애물을 임의로 바꾸지는 않습니다.

## 2. 기본 시작점과 목적지

이 미션의 24개 JSON은 같은 시작점이고 [example.yaml](example.yaml)은 공통 목적지 하나를 사용합니다. 아래 값은 생성기가 실제 JSON/YAML에서 가져옵니다. 숫자는 표시용으로 소수점 6자리까지 반올림했으므로 실행에는 원본 파일 값을 사용하세요.

| 항목 | 좌표/방향 | 수정하는 파일 |
| --- | --- | --- |
| Ego 시작점, MORAI 좌표 | x=-59.892653, y=-80.531189, z=28.726135, yaw=90.423000° | JSON의 egoVehicle.initPosition |
| 같은 시작점, map 좌표 참고 | x=71.666477, y=465.380149, z=28.726135, yaw=90.423000° | 위치 참고용; 입력을 추가하지 않음 |
| 목적지, planning map 좌표 | x=70.991398, y=585.981428, z=28.387175, yaw=90.733994° | YAML의 default_case.goal |
| 도로 시작 anchor | initLink=A2256W000202, initLinkRatio=0.400000 | JSON; 위치만 바꿀 때도 함께 확인 |
| 주행 제한 / 도착 | 70초 / stop | YAML의 default_case |

Ego는 지도 링크 A2256W000202의 0.40 위치, 목적지는 A2256W000315의 0.42 위치에서 도로 방향을 사용했습니다. 보행자는 샘플 PedPath - 3를 쓰고, 교차 차량은 기존 도로 링크 A2256W001124 또는 A2256W000636에 둡니다.

객체 종류·좌우 위치·개수·속도·spawn 간격만 바꾸고 같은 구간을 통과한다면 시작점·목적지를 유지하세요. 시작점을 바꿔도 같은 목적지까지 연결되면 같은 YAML이 가능하지만 경로·방향·주행시간을 다시 확인해야 합니다. 조건별 설정을 동시에 남겨둘 때는 별도 YAML로 구분합니다.
같은 개인 YAML은 수정·재사용해도 됩니다. GUI의 `새 실험 시작`은 timestamp 결과 폴더와 입력 사본을 만들고, `실험 기록 / 재개`는 당시 사본을 사용합니다. 결과 폴더의 입력 사본은 직접 수정하지 마세요.

다른 좌표 후보를 임의로 제공하지 않았습니다. 합류만/순항만 등 검증 구간을 바꾸려면 지도에서 연결되는 시작·끝점을 확인하고 별도 YAML로 먼저 한 장면을 실행하세요.

MORAI 객체 좌표를 goal에 그대로 복사하지 마세요. 현재 변환은 `configs/tf/morai_lanelet_static_tf.yaml`의 map_to_morai_map이며 map = R × MORAI + translation입니다. 지도/TF가 바뀌면 다시 확인합니다. 위 yaw는 지도상의 배치 방향이지 localization heading 보정 파라미터가 아닙니다.

## 3. 기존 24개를 어떻게 배치했나

기준선 2개(`do_baseline_*`) → 일반 변형 14개(`do_normal_*`) → stress 8개(`do_stress_*`)입니다. normal/stress는 설계상 분류이지 성공/실패 예측이나 대회 출제 확률이 아닙니다.

00은 무배우, 01은 Man1·샘플 횡단 경로·속도 설정 10·activeDistance 15의 기준선입니다. 02–11은 보행자 모델·방향·속도 설정 3/5/7/10·activeDistance 15/20/25/30을 바꿉니다. 12–17은 같은 횡단 경로의 스쿠터/자전거, 18–23은 목표 10/20/30km/h의 교차 차량이며 마지막 두 장면에는 보행자를 함께 둡니다. 12–23의 배우 이동은 특히 실기 확인이 필요합니다.

정확한 장면별 배우·파라미터·출처는 [manifest.json](manifest.json), 실제 설정은 [scenarios/](scenarios/)에서 봅니다. "baseline"도 풀스택 주행 성공을 보장하지 않습니다. 01은 샘플 조건을 참고한 단순 대조이며 원본 본선 시나리오 전체를 복제한 것은 아닙니다.

근거 파일:

- `configs/scenario/2026_molit_comp_sample_scene.json`
- `kcity_map/mgeo_map/link_set.json` — 도로 링크·진행 방향
- `configs/global_path/2026_molit_comp_global_path.txt` — 기존 코스
- `configs/tf/morai_lanelet_static_tf.yaml` — 좌표 변환
- `base_ws/src/planning_addons/planning_scenario_validator/scripts/build_mission_starters.py` — 실제 생성 규칙

## 4. 팀원이 추가해 볼 순서

1. 같은 보행자 경로에서 방향만 바꾸고, 다음에는 속도/activeDistance를 하나씩 바꿔 Ego와 실제로 만나는 시점을 확인하세요.
2. 보행자 단독과 교차 차량 단독을 먼저 확인한 뒤 동시에 등장하는 장면을 추가하세요.
3. 스쿠터나 다른 동적 객체도 추가할 수 있지만 경로 연결·trigger·animation을 확인하세요. 속도 필드만 채워서는 이동이 보장되지 않습니다.

처음에는 기존 JSON 하나를 `../workspace/scenarios/`에 새 이름으로 복사해 한두 조건만 바꾸세요. 일반 상황을 충분히 두고 스트레스를 섞습니다. 새 YAML은 기존 example 전체를 복사하거나 GUI로 저장하세요. 빈 YAML·장면별 .case.yaml은 만들지 않습니다. 경로는 YAML 옆 기준이 아니라 저장소 루트 기준입니다.

주의할 점:

- pedestrian.speed, object.moveSpeed, vehicle.desiredVelocity를 같은 단위·동작으로 추측하지 않습니다. MORAI UI/문서와 실제 이동을 확인합니다. activeDistance는 시간(초)이 아니라 거리 조건입니다.
- 이동 배우가 멈춰 있거나 Ego가 지나간 뒤에만 움직이면 동적 대응 성공으로 집계하지 않습니다. 초기 actor/waypoint unique ID 연결과 좌표 변환을 유지합니다.

## 5. 실행 후 무엇을 보면 되나

먼저 `do_baseline_00.json`과 `do_baseline_01.json`을 파일 패턴 `do_baseline_*.json`으로 확인하세요. 새 배우 장면도 대표 한두 개는 직접 보고 나서 전체를 걸어둡니다.

객체 이동 시작·Ego 조우·정지·객체 통과·재출발을 순서대로 보세요. 미출발 FAIL이 경로 미생성인지 객체가 실제로 길을 막는지 구분하세요. 신호등 off 기준이므로 신호등 통합 시험은 별도입니다.

현재 이 24개는 **형식·생성 재현성 검증만 완료, 전체 live 주행은 미검증**입니다. 기존 정적 pair의 GT/BEV 주행 검증을 이 묶음의 성공률로 확대하지 않습니다. 무장애물 기준선·배우 동작·goal route를 실제로 확인해야 합니다.

기본은 localization + BEVFusion + 신호등 off, basic bag + 원시 센서 녹화 제외 + 영상 on입니다. 카메라/LiDAR 입력 자체는 켜둡니다. 신호등 on 시험은 별도 YAML로 분리하세요. `od_gt`는 같은 JSON의 인지 원인 분리용 대조이고, `maptf_morai`는 측위 원인 분리용 대조입니다.

현재 자동 PASS/FAIL은 충돌·목적지·시간 조건 중심입니다. 양보·차선 침범·규정 속도·배우가 실제로 만났는지는 전부 자동 판정하지 않습니다. 도착했다고 미션 전체가 올바른 것은 아니며, timeout이라고 모두 판단 버그인 것도 아닙니다. `summary.csv`와 해당 run JSON/이벤트/영상에서 관측 근거를 함께 봅니다. 분석기의 원인 설명도 확정 원인이 아니라 근거에 기반한 후보로 확인하세요.

개인 파일은 workspace에서 Git 제외됩니다. 녹화는 `planner_logs/validation/<campaign_name>/`에 남습니다. GUI 복사 버튼이 만드는 workspace README는 이 원본 안내로 연결하는 참고 문서이며 개인 JSON 수정 내용을 자동 추적하지 않습니다.

## 6. 담당 AI에게 복붙

```text
사거리 동적 장애물 대응 판단 검증용 개인 시나리오와 완성된 v2 YAML을 만들어 줘.
먼저 base_ws/src/planning_addons/planning_scenario_validator/config/missions/dynamic_obstacle/base/README.md,
같은 base의 example.yaml·manifest.json·대표 baseline/normal/stress JSON,
docs/planning/SCENARIO_VALIDATOR_AUTHOR_GUIDE.md를 읽어 줘.

샘플 사거리와 PedPath - 3 또는 기존 교차 도로 링크를 기반으로 방향·속도·activeDistance를 단계적으로 바꿔 줘. 새 배우 이동은 미검증으로 표시하고 실제 조우를 확인할 수 있게 해 줘. activeDistance를 초 단위 trigger로 해석하지 마.
공유 base와 planning/control/localization 코드·파라미터는 수정하지 말고
base_ws/src/planning_addons/planning_scenario_validator/config/missions/dynamic_obstacle/workspace/ 아래에 개인 JSON과 YAML을 만들어 줘.
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
