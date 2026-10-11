# 고속주행도로 — 시나리오 만들 때 옆에 두는 안내

**[example.yaml](example.yaml)은 공유 원본이므로 바로 실행하지 마세요. GUI에서 선택한 뒤 반드시 `베이스 → 내 작업 폴더 복사`를 누르고, 열린 YAML 경로가 이 미션의 `workspace/`인지 확인하세요.** 이후 설정·실행·배치 추가는 개인 복사본을 사용합니다. `workspace/`의 개인 파일은 Git 제외지만 `base/`의 example 및 공유 비교 예제 YAML은 추적됩니다.

시작점·목적지는 그대로 쓰고 객체 조건부터 바꾸면 됩니다. 공통 설치/실행법은 저장소의 `docs/planning/SCENARIO_VALIDATOR_RUNBOOK.md`, 제작 규칙은 `docs/planning/SCENARIO_VALIDATOR_AUTHOR_GUIDE.md`를 봅니다. 아래 파일 경로는 모두 저장소 루트 기준입니다.

## 1. 무엇을 확인하려고 만들었나

고주로 합류 접근부터 합류 후 주행까지, 주변 차량과의 간격·상대속도 변화에 판단이 어떻게 대응하는지 봅니다.

학습 데이터 수집이 아니라 **평범한 상황의 회귀 + 대응하지 못하는 상황 발견**이 목적입니다. 실패하는 장면도 필요한 결과입니다. PASS시키기 위해 판단 파라미터나 장애물을 임의로 바꾸지는 않습니다.

## 2. 기본 시작점과 목적지

이 미션의 24개 JSON은 같은 시작점이고 [example.yaml](example.yaml)은 공통 목적지 하나를 사용합니다. 아래 값은 생성기가 실제 JSON/YAML에서 가져옵니다. 숫자는 표시용으로 소수점 6자리까지 반올림했으므로 실행에는 원본 파일 값을 사용하세요.

| 항목 | 좌표/방향 | 수정하는 파일 |
| --- | --- | --- |
| Ego 시작점, MORAI 좌표 | x=44.676904, y=324.057943, z=28.675192, yaw=-53.676000° | JSON의 egoVehicle.initPosition |
| 같은 시작점, map 좌표 참고 | x=176.236034, y=869.969281, z=28.675192, yaw=-53.676000° | 위치 참고용; 입력을 추가하지 않음 |
| 목적지, planning map 좌표 | x=197.517700, y=653.498242, z=28.264104, yaw=-86.688598° | YAML의 default_case.goal |
| 도로 시작 anchor | initLink=A2256W000846, initLinkRatio=0.468401 | JSON; 위치만 바꿀 때도 함께 확인 |
| 주행 제한 / 도착 | 90초 / stop | YAML의 default_case |

Ego는 기존 고주로 합류 평가 장면의 접근 위치입니다. 목적지는 기존 전체 코스에서 시작점 이후 약 220m를 따라 선택했습니다. NPC는 샘플 spawn ID 6/8/10의 위치·시작/종료 링크를 유지했습니다.

객체 종류·좌우 위치·개수·속도·spawn 간격만 바꾸고 같은 구간을 통과한다면 시작점·목적지를 유지하세요. 시작점을 바꿔도 같은 목적지까지 연결되면 같은 YAML이 가능하지만 경로·방향·주행시간을 다시 확인해야 합니다. 조건별 설정을 동시에 남겨둘 때는 별도 YAML로 구분합니다.
같은 개인 YAML은 수정·재사용해도 됩니다. GUI의 `새 실험 시작`은 timestamp 결과 폴더와 입력 사본을 만들고, `실험 기록 / 재개`는 당시 사본을 사용합니다. 결과 폴더의 입력 사본은 직접 수정하지 마세요.

다른 좌표 후보를 임의로 제공하지 않았습니다. 합류만/순항만 등 검증 구간을 바꾸려면 지도에서 연결되는 시작·끝점을 확인하고 별도 YAML로 먼저 한 장면을 실행하세요.

소수 JSON을 여러 번 돌릴 때는 [random_example.yaml](random_example.yaml)을 GUI에서 선택하고 workspace로 복사하세요. 기존 high_baseline_01/high_normal_06 두 장면 × 10회 = 총 20회이며, NPC 목표 속도 50–60km/h와 path offset −10–10%를 회차마다 추출합니다. 위치·주행/종료 링크·MiddleSize 필터와 초기 속도는 원본대로 유지합니다. 대회 공식 속도 범위는 아닙니다. repeat/seed/범위만 YAML 위쪽에서 바꾸면 되고, 랜덤을 끄려면 enabled: false입니다. 실제로 로드한 scenario.json과 추출값이 input.json/결과 JSON에 남습니다. 같은 seed/JSON/회차는 같은 파라미터지만 native RandomMode 차종과 physics까지 고정하지는 않습니다.

MORAI 객체 좌표를 goal에 그대로 복사하지 마세요. 현재 변환은 `configs/tf/morai_lanelet_static_tf.yaml`의 map_to_morai_map이며 map = R × MORAI + translation입니다. 지도/TF가 바뀌면 다시 확인합니다. 위 yaw는 지도상의 배치 방향이지 localization heading 보정 파라미터가 아닙니다.

## 3. 기존 24개를 어떻게 배치했나

기준선 2개(`high_baseline_*`) → 일반 변형 14개(`high_normal_*`) → stress 8개(`high_stress_*`)입니다. normal/stress는 설계상 분류이지 성공/실패 예측이나 대회 출제 확률이 아닙니다.

00은 NPC 없는 기준선입니다. 01은 spawn ID 10/6/8에 각각 목표 40/50/60km/h, 초기 0km/h, 10초 간격을 줍니다. 나머지는 목표 30/40/50/60/70/80km/h를 spawn별로 순환 배정하고 간격 3/5/7/10/14초, 초기 0/10/20/목표속도를 조합합니다. 16–23의 MinLatBias/MaxLatBias는 ±0.35%입니다 (m 단위가 아닙니다). RandomMode의 MiddleSize 필터 [2]를 유지하지만 실제 spawn 차종은 실행마다 달라질 수 있습니다.

정확한 장면별 배우·파라미터·출처는 [manifest.json](manifest.json), 실제 설정은 [scenarios/](scenarios/)에서 봅니다. "baseline"도 풀스택 주행 성공을 보장하지 않습니다. 01은 샘플 조건을 참고한 단순 대조이며 원본 본선 시나리오 전체를 복제한 것은 아닙니다.

근거 파일:

- `configs/scenario/perception_benchmark/bench_highway_merge_raw.json`
- `configs/scenario/2026_molit_comp_sample_scene.json`
- `kcity_map/mgeo_map/link_set.json` — 도로 링크·진행 방향
- `configs/global_path/2026_molit_comp_global_path.txt` — 기존 코스
- `configs/tf/morai_lanelet_static_tf.yaml` — 좌표 변환
- `base_ws/src/planning_addons/planning_scenario_validator/scripts/build_mission_starters.py` — 실제 생성 규칙

## 4. 팀원이 추가해 볼 순서

1. 같은 목표속도에서 spawn 간격만 바꿔 넓은 gap·보통 gap·좁은 gap을 대조하세요.
2. 같은 간격에서 차선별 목표속도 조합을 바꿔 보세요. 특정 차선이 항상 가장 빠르다는 가정을 넣지 마세요.
3. 초기 0으로 가속하는 장면과 초기=목표로 순항하는 장면을 구분하세요. Ego가 지나갈 때 NPC가 실제로 만나는지도 보세요.

처음에는 기존 JSON 하나를 `../workspace/scenarios/`에 새 이름으로 복사해 한두 조건만 바꾸세요. 일반 상황을 충분히 두고 스트레스를 섞습니다. GUI의 내 미션 선택하기에서 담당 미션 → 실험 YAML의 예제 / example.yaml → 베이스 복사로 준비합니다. 목록에는 해당 미션의 예제와 내 작업 YAML만 표시되며 경로를 직접 입력하지 않습니다. 새 YAML은 미션 workspace에 저장한 뒤 새로고침하세요. 이름·파일 패턴/장면 선택·반복·측위/탐지·신호등·녹화/공유는 기본 탭에서 수정하고 상단 YAML 저장을 누릅니다. 같은 개인 파일에 저장되며 날짜 파일을 새로 만들지 않습니다. ‘여기부터 수정 X’ 아래 목적지·시간·네트워크와 회색 GUI 값은 그대로 유지하세요. 랜덤 범위·Python 환경은 VSCode에서만 수정합니다. 빈 YAML·장면별 .case.yaml은 만들지 않습니다. 경로는 YAML 옆 기준이 아니라 저장소 루트 기준입니다.

주의할 점:

- 현재 베이스의 NPC는 자동 차선 변경 off입니다. 끼어들기·Ego 100–120km/h 추월을 모두 검증하는 세트는 아닙니다. 그런 시험은 별도 경로·타이밍·목적지 검토가 필요합니다.
- 주최측은 본선 속도가 샘플과 다르고 RandomMode 사용은 미정, 사용한다면 MiddleSize라고 답했습니다. 30–80km/h는 공식 범위가 아닙니다.

## 5. 실행 후 무엇을 보면 되나

먼저 `high_baseline_00.json`과 `high_baseline_01.json`을 파일 패턴 `high_baseline_*.json`으로 확인하세요. 새 배우 장면도 대표 한두 개는 직접 보고 나서 전체를 걸어둡니다.

Ego가 NPC와 실제로 만났는지, 밀도 때문에 합류를 기다렸는지, 합류 후 경로가 생기는지 확인하세요. RandomMode 재실행은 동일 차종을 보장하지 않으므로 영상과 생성된 객체를 함께 봅니다.

현재 이 24개는 **형식·생성 재현성 검증만 완료, 전체 live 주행은 미검증**입니다. 기존 정적 pair의 GT/BEV 주행 검증을 이 묶음의 성공률로 확대하지 않습니다. 무장애물 기준선·배우 동작·goal route를 실제로 확인해야 합니다.

기본은 localization + BEVFusion + 신호등 off, basic bag + 원시 센서 녹화 제외 + 영상 on입니다. 카메라/LiDAR 입력 자체는 켜둡니다. 신호등 on 시험은 별도 YAML로 분리하세요. `od_gt`는 같은 JSON의 인지 원인 분리용 대조이고, `maptf_morai`는 측위 원인 분리용 대조입니다.

현재 자동 PASS/FAIL은 충돌·목적지·시간 조건 중심입니다. 양보·차선 침범·규정 속도·배우가 실제로 만났는지는 전부 자동 판정하지 않습니다. 도착했다고 미션 전체가 올바른 것은 아니며, timeout이라고 모두 판단 버그인 것도 아닙니다. `summary.csv`와 해당 run JSON/이벤트/영상에서 관측 근거를 함께 봅니다. 분석기의 원인 설명도 확정 원인이 아니라 근거에 기반한 후보로 확인하세요.

개인 파일은 workspace에서 Git 제외됩니다. 녹화는 `planner_logs/validation/<실험 이름>_<YYMMDD_HHMMSS>/`에 남습니다. GUI 복사 버튼이 만드는 workspace README는 이 원본 안내로 연결하는 참고 문서이며 개인 JSON 수정 내용을 자동 추적하지 않습니다.

## 6. 담당 AI에게 복붙

```text
고속주행도로 판단 검증용 개인 시나리오와 완성된 v2 YAML을 만들어 줘.
먼저 base_ws/src/planning_addons/planning_scenario_validator/config/missions/highway/base/README.md,
같은 base의 example.yaml·manifest.json·대표 baseline/normal/stress JSON,
docs/planning/SCENARIO_VALIDATOR_AUTHOR_GUIDE.md를 읽어 줘.

샘플 spawn ID 6/8/10의 위치·주행/종료 링크와 MiddleSize 필터를 유지하고, 목표속도·초기속도·spawn 간격을 단계적으로 바꿔 줘. 현재 베이스에 없는 차선 변경/고속 추월을 구현됐다고 주장하지 마.
공유 base와 planning/control/localization 코드·파라미터는 수정하지 말고
base_ws/src/planning_addons/planning_scenario_validator/config/missions/highway/workspace/ 아래에 개인 JSON과 YAML을 만들어 줘.
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
