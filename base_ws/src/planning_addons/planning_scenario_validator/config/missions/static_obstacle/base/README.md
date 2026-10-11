# 일반 정적 장애물 회피 — 시나리오 만들 때 옆에 두는 안내

**[example.yaml](example.yaml)은 공유 원본이므로 바로 실행하지 마세요. GUI에서 선택한 뒤 반드시 `베이스 → 내 작업 폴더 복사`를 누르고, 열린 YAML 경로가 이 미션의 `workspace/`인지 확인하세요.** 이후 설정·실행·배치 추가는 개인 복사본을 사용합니다. `workspace/`의 개인 파일은 Git 제외지만 `base/`의 example 및 공유 비교 예제 YAML은 추적됩니다.

시작점·목적지는 그대로 쓰고 객체 조건부터 바꾸면 됩니다. 공통 설치/실행법은 저장소의 `docs/planning/SCENARIO_VALIDATOR_RUNBOOK.md`, 제작 규칙은 `docs/planning/SCENARIO_VALIDATOR_AUTHOR_GUIDE.md`를 봅니다. 아래 파일 경로는 모두 저장소 루트 기준입니다.

## 1. 무엇을 확인하려고 만들었나

GPS 음영이 아닌 일반 정적 미션에서, 장애물 종류·좌우 위치·개수에 따라 경로가 생기는지와 회피 후 복귀를 봅니다.

학습 데이터 수집이 아니라 **평범한 상황의 회귀 + 대응하지 못하는 상황 발견**이 목적입니다. 실패하는 장면도 필요한 결과입니다. PASS시키기 위해 판단 파라미터나 장애물을 임의로 바꾸지는 않습니다.

## 2. 기본 시작점과 목적지

이 미션의 24개 JSON은 같은 시작점이고 [example.yaml](example.yaml)은 공통 목적지 하나를 사용합니다. 아래 값은 생성기가 실제 JSON/YAML에서 가져옵니다. 숫자는 표시용으로 소수점 6자리까지 반올림했으므로 실행에는 원본 파일 값을 사용하세요.

| 항목 | 좌표/방향 | 수정하는 파일 |
| --- | --- | --- |
| Ego 시작점, MORAI 좌표 | x=-63.195000, y=-186.079000, z=28.736000, yaw=78.294000° | JSON의 egoVehicle.initPosition |
| 같은 시작점, map 좌표 참고 | x=68.364130, y=359.832338, z=28.736000, yaw=78.294000° | 위치 참고용; 입력을 추가하지 않음 |
| 목적지, planning map 좌표 | x=71.510000, y=429.608000, z=28.383000, yaw=90.033000° | YAML의 default_case.goal |
| 도로 시작 anchor | initLink=A2256W000213, initLinkRatio=0.606081 | JSON; 위치만 바꿀 때도 함께 확인 |
| 주행 제한 / 도착 | 65초 / stop | YAML의 default_case |

샘플 CargoBox가 있는 정적 구간을 사용합니다. Ego는 그 앞 도로상의 기존 검증 시작점이며 목적지는 장애물 이후 기존 코스의 점입니다. 관련 빌더의 코스 거리 표기는 시작 약 285m, 기본 목표 약 355m입니다.

객체 종류·좌우 위치·개수·속도·spawn 간격만 바꾸고 같은 구간을 통과한다면 시작점·목적지를 유지하세요. 시작점을 바꿔도 같은 목적지까지 연결되면 같은 YAML이 가능하지만 경로·방향·주행시간을 다시 확인해야 합니다. 조건별 설정을 동시에 남겨둘 때는 별도 YAML로 구분합니다.
같은 개인 YAML은 수정·재사용해도 됩니다. GUI의 `새 실험 시작`은 timestamp 결과 폴더와 입력 사본을 만들고, `실험 기록 / 재개`는 당시 사본을 사용합니다. 결과 폴더의 입력 사본은 직접 수정하지 마세요.

긴 구간의 기존 후보는 `static_pair_mission_bevfusion.yaml`의 map 목적지 (71.704, 459.605, 28.379), yaw 90.131°입니다. 그 YAML의 주행 제한은 50초입니다. 기존 pair 장면용 조건이며 새 24개 전체에 검증한 목적지가 아닙니다. 끝쪽 객체까지 보고 싶을 때 별도 YAML에서 시험하세요. 지금 기본 goal과 섞어 실행하지 않습니다.

MORAI 객체 좌표를 goal에 그대로 복사하지 마세요. 현재 변환은 `configs/tf/morai_lanelet_static_tf.yaml`의 map_to_morai_map이며 map = R × MORAI + translation입니다. 지도/TF가 바뀌면 다시 확인합니다. 위 yaw는 지도상의 배치 방향이지 localization heading 보정 파라미터가 아닙니다.

## 3. 기존 24개를 어떻게 배치했나

기준선 2개(`soa_baseline_*`) → 일반 변형 14개(`soa_normal_*`) → stress 8개(`soa_stress_*`)입니다. normal/stress는 설계상 분류이지 성공/실패 예측이나 대회 출제 확률이 아닙니다.

00은 무장애물, 01은 샘플 CargoBox 위치·scale을 그대로 둡니다. 일반 변형은 MORAI x=-60.61 주변 ±0.65m, y=-159.25에서 객체마다 약 17m 간격으로 배치합니다. CargoBox·PE_Drum·TrafficBarrel·Concrete_Barricade01·PE_Firewall_Orange를 1–3개 조합합니다. 일부 장면에는 정지 차량, 마지막 두 장면에는 스쿠터/자전거도 포함합니다. 01을 제외한 생성 객체 scale은 1입니다.

정확한 장면별 배우·파라미터·출처는 [manifest.json](manifest.json), 실제 설정은 [scenarios/](scenarios/)에서 봅니다. "baseline"도 풀스택 주행 성공을 보장하지 않습니다. 01은 샘플 조건을 참고한 단순 대조이며 원본 본선 시나리오 전체를 복제한 것은 아닙니다.

근거 파일:

- `configs/scenario/2026_molit_comp_sample_scene.json`
- `configs/scenario/perception_benchmark/bench_static_obstacle_base3_assets_raw.json`
- `base_ws/src/planning_addons/planning_scenario_validator/scripts/build_static_corridor_cases.py`
- `kcity_map/mgeo_map/link_set.json` — 도로 링크·진행 방향
- `configs/global_path/2026_molit_comp_global_path.txt` — 기존 코스
- `configs/tf/morai_lanelet_static_tf.yaml` — 좌표 변환
- `base_ws/src/planning_addons/planning_scenario_validator/scripts/build_mission_starters.py` — 실제 생성 규칙

## 4. 팀원이 추가해 볼 순서

1. 객체 하나로 시작해서 같은 위치의 종류만 바꾸고, 이후 같은 객체의 좌우 위치만 바꿔 보세요.
2. 단독 배치 결과를 남긴 뒤 두 객체의 좌우 엇갈림과 종방향 간격을 바꿔 보세요. 실제 차체가 통과할 공간이 있는지도 구분하세요.
3. 정지 차량·학습에서 드문 객체를 추가할 수 있습니다. 너무 가까운 시작 배치와 완전 봉쇄는 별도 스트레스 케이스로 표시하세요.

처음에는 기존 JSON 하나를 `../workspace/scenarios/`에 새 이름으로 복사해 한두 조건만 바꾸세요. 일반 상황을 충분히 두고 스트레스를 섞습니다. YAML은 GUI의 베이스 복사로 준비합니다. 이름·파일 패턴/장면 선택·반복·측위/탐지·신호등·녹화/공유는 기본 탭에서 수정하고 상단 파란 YAML 저장을 누릅니다. 같은 개인 파일에 저장되며 날짜 파일을 새로 만들지 않습니다. ‘여기부터 수정 X’ 아래 목적지·시간·네트워크와 회색 GUI 값은 그대로 유지하세요. 랜덤 범위·Python 환경은 VSCode에서만 수정합니다. 빈 YAML·장면별 .case.yaml은 만들지 않습니다. 경로는 YAML 옆 기준이 아니라 저장소 루트 기준입니다.

주의할 점:

- 학습 데이터 생성처럼 아무 위치에나 무작위 배치하지 않습니다. 회피 불가능한 완전 봉쇄에서 정지하는 것은 합리적인 대응일 수 있으며, timeout을 곧바로 판단 버그라고 단정하지 않습니다.
- WoodBox는 확인 전까지 제외합니다. 새 객체는 asset 지원·실제 크기를 확인합니다. Object_topic.size만으로 scale/충돌 여유가 맞다고 가정하지 않습니다.

## 5. 실행 후 무엇을 보면 되나

먼저 `soa_baseline_00.json`과 `soa_baseline_01.json`을 파일 패턴 `soa_baseline_*.json`으로 확인하세요. 새 배우 장면도 대표 한두 개는 직접 보고 나서 전체를 걸어둡니다.

장애물 검출→track→정지/회피 궤적→제어 순으로 보세요. GT 대조도 같은 위치에서 실패하면 인지 단독 원인으로 단정하지 않습니다. 차선 침범이나 회피 후 복귀도 영상을 확인합니다.

현재 이 24개는 **형식·생성 재현성 검증만 완료, 전체 live 주행은 미검증**입니다. 기존 정적 pair의 GT/BEV 주행 검증을 이 묶음의 성공률로 확대하지 않습니다. 무장애물 기준선·배우 동작·goal route를 실제로 확인해야 합니다.

기본은 localization + BEVFusion + 신호등 off, basic bag + 원시 센서 녹화 제외 + 영상 on입니다. 카메라/LiDAR 입력 자체는 켜둡니다. 신호등 on 시험은 별도 YAML로 분리하세요. `od_gt`는 같은 JSON의 인지 원인 분리용 대조이고, `maptf_morai`는 측위 원인 분리용 대조입니다.

현재 자동 PASS/FAIL은 충돌·목적지·시간 조건 중심입니다. 양보·차선 침범·규정 속도·배우가 실제로 만났는지는 전부 자동 판정하지 않습니다. 도착했다고 미션 전체가 올바른 것은 아니며, timeout이라고 모두 판단 버그인 것도 아닙니다. `summary.csv`와 해당 run JSON/이벤트/영상에서 관측 근거를 함께 봅니다. 분석기의 원인 설명도 확정 원인이 아니라 근거에 기반한 후보로 확인하세요.

개인 파일은 workspace에서 Git 제외됩니다. 녹화는 `planner_logs/validation/<실험 이름>_<YYMMDD_HHMMSS>/`에 남습니다. GUI 복사 버튼이 만드는 workspace README는 이 원본 안내로 연결하는 참고 문서이며 개인 JSON 수정 내용을 자동 추적하지 않습니다.

## 6. 담당 AI에게 복붙

```text
일반 정적 장애물 회피 판단 검증용 개인 시나리오와 완성된 v2 YAML을 만들어 줘.
먼저 base_ws/src/planning_addons/planning_scenario_validator/config/missions/static_obstacle/base/README.md,
같은 base의 example.yaml·manifest.json·대표 baseline/normal/stress JSON,
docs/planning/SCENARIO_VALIDATOR_AUTHOR_GUIDE.md를 읽어 줘.

샘플 정적 구간의 시작점·기본 목적지를 유지하고 좌우 위치·종류·개수·간격을 한두 조건씩 바꿔 줘. 단독 대조와 다객체를 함께 준비하고, 완전 봉쇄는 별도 표시해 줘. scale=1, WoodBox 제외를 유지해 줘.
공유 base와 planning/control/localization 코드·파라미터는 수정하지 말고
base_ws/src/planning_addons/planning_scenario_validator/config/missions/static_obstacle/workspace/ 아래에 개인 JSON과 YAML을 만들어 줘.
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
