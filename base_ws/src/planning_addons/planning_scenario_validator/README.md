# Planning scenario validator

팀장님 PC의 임시 LAN-to-LAN 개발 오류는 [전용 짧은 안내](../../../../docs/planning/SCENARIO_VALIDATOR_LAN_QUICK_CHECK.md)를 봅니다. 대회에서는 검증된 현장점검 실행 절차·설정을 유지합니다.

공유·실행 브랜치는 `planning/autoware-universe-to-ros1`이다. 업데이트는 캠페인·스택 종료 → pull → planning/base 증분 빌드 → 환경 재source·노드 재실행 순서다. 초기 설치/clean을 매번 반복하지 않는다. 복붙 명령은 [팀원 안내 4-1](../../../../docs/planning/SCENARIO_VALIDATOR_TEAM_NOTION_GUIDE.md#4-1-다음부터는-pull-후-증분-빌드)에 있다.

2026-10-23 사용자 승인으로 기본 detector는 전체 30,750장 E12 최적화 production이다. 기존 팀 이미지 위 TRT overlay와 PC별 engine을 준비하고
`bash scripts/runtime/install_base5_production.sh`로 Drive 가중치 다운로드·검증·증분 빌드를 수행한다.
[production 배포 안내](../../../../docs/bevfusion/BASE5_BEVFUSION_PRODUCTION_HANDOFF.md)를 따른다.
GUI 기본값과 모델 없는 새 YAML은 E12이며 명시적인 `e24`/`e25`는 복귀 모델이다. 가속기 미준비는 같은 E12 eager로 시작한다. 기존 실행 결과는 모델·코드
계약이 다르면 강제 resume하지 않는다. 아래의 공용 이미지/E25 초기 설치는 E12 가중치 설치를 대신하지 않는다.

팀원이 만든 MORAI scenario JSON 폴더를 대상으로, 실제 localization → 인지 → tracking/prediction → planning → controller 폐루프 주행을 시나리오별로 검증한다. CLI와 얇은 설정 GUI를 제공하지만 MORAI MR의 최초 네트워크 Connect에는 MORAI GUI가 필요하다. 각 시나리오의 실시간 판정 결과를 저장하며, bag과 화면 영상은 선택한 녹화 설정에 따라 별도로 남긴다.

처음 쓰는 팀원은 [노션 공유용 첫 설치·실행 안내](../../../../docs/planning/SCENARIO_VALIDATOR_TEAM_NOTION_GUIDE.md)부터 따라 한다. Clone/브랜치 전환, ROS 빌드, 맵·공용 Docker 이미지·E25 다운로드, MORAI 연결과 GUI 실행을 한 문서에 정리했다. `install.sh --gui`는 GUI 도구 검사와 Network 생성이며 전체 설치를 대신하지 않는다.

설치 후에는 [실제 작업 순서](../../../../docs/planning/SCENARIO_VALIDATOR_WORKFLOW.md)를 따른다. [자주 쓰는 곳을 표시한 화면 5장](../../../../docs/planning/SCENARIO_VALIDATOR_GUI_HIGHLIGHTS.md)과 [자세한 사용법](../../../../docs/planning/SCENARIO_VALIDATOR_RUNBOOK.md)도 제공한다. **YAML 하나는 공통 실행 조건 하나**, 여러 YAML은 GUI `실행 목록 / 배치`에 추가해 순서대로 실행한다. 미션별 사용자 진입점은 `config/missions/<미션>/base/`와 Git 제외 `workspace/`이며, 토픽·생성 Network는 `config/internal/`에 분리했다.

Ubuntu 20.04에서 Docker·Drive 이미지·E25를 한 번에 준비하려면 `bash base_ws/src/planning_addons/planning_scenario_validator/install.sh --bevfusion`을 선택한다. 기본 `--gui`/`--check`는 계속 도구 검사이며 자동 설치하지 않는다. `--bevfusion --dry-run`은 변경 없는 계획 출력이다. 기존 Docker·가중치·.env·드라이버는 임의 교체하지 않으며 최초 본인 Google 인증·sudo·Docker 그룹 재로그인은 필요할 수 있다. ROS 빌드·맵은 별도다. 자세한 단계는 설치 가이드 5-0을 참고한다.

## GUI로 시작하기

**일상 설정은 GUI에서 수정 → YAML 저장 → 실행한다.** 파일 패턴/장면 선택·반복·측위/탐지·신호등·녹화/공유는 기본 탭에서 바꾼다. 목적지·시간·네트워크·Python 환경은 회색 읽기 전용이다.

1. **내 미션 선택하기 → 담당 미션 → 예제 / example.yaml → 베이스 → 내 작업 폴더 복사**. 옆 YAML 목록은 선택한 미션의 예제·내 작업 파일만 표시한다. 공유 예제를 직접 실행하지 않는다.
2. 같은 미션 `workspace/`에 주석 있는 개인 YAML·JSON이 준비된다. 기존 개인 파일은 덮어쓰지 않는다.
3. 기본 탭에서 일상 옵션을 바꾸고 상단 왼쪽 **YAML 저장**을 누른다. 현재 개인 파일에 저장하며 고정값·주석을 유지한다. 파일 이름을 새로 만들지 않는다. **‘여기부터 수정 X’ 아래 공통 목적지·시간·네트워크는 유지한다.** Ego 시작점은 JSON에 있다.
4. 자유 경로 입력·붙여넣기·내 YAML 파일 선택창은 없다. 새 YAML은 해당 미션 workspace에 저장하고 **불러오기/새로고침 → 실험 YAML 선택**으로 연다. 현재 파일을 VSCode에서 저장한 경우도 새로고침으로 표시를 갱신한다. 외부 경로·다른 미션 symlink·보조 파일·실행 사본은 목록에 표시하지 않는다. 기존 users/외부 파일과 결과는 삭제하지 않으며 CLI·과거 기록 재개는 유지한다.
5. **YAML 저장 → 오른쪽 보조 기능의 설정 검사 → 새 실험 시작**. 상단 왼쪽에는 저장·불러오기/새로고침·시작만 두며, 기록·결과 폴더도 보조 기능에서 연다. 중단·모니터 모드는 오른쪽에 항상 표시한다. 검사·새 실행 전에도 GUI 변경분을 저장한다. 수동 풀스택은 종료하고 MORAI만 연결해 둔다. 설정 검사는 주행 성공 보장이 아니다.

```bash
cd ~/AjouNice2026
./planning_validator_gui
# alias 환경이면 pvgui
```

`workspace/` 개인 파일은 Git 제외, 공유 `base/` example·비교 YAML은 추적한다. 파일 삭제·이름 변경은 `내 작업 폴더 열기`를 사용한다. 마지막에 연 실존 YAML 경로만 `~/.config/ajounice/planning_scenario_validator/gui_settings.json`에 기억한다. 폼 편집값을 따로 보관하지 않는다.

### 고주로 소수 시나리오 반복

`highway/base/random_example.yaml`을 선택하고 workspace로 복사한다. 기존 샘플 spawn/경로·MiddleSize를 유지한 JSON 2개 × 10회 = 20회다. 첫 시험 범위는 NPC 목표 50–60km/h, path offset −10–10%이며 대회 공식 범위가 아니다.

`scenarios.repeat`는 JSON 하나당 횟수다. `scenarios.randomization.enabled: false`이면 원본 그대로 반복, true이면 속도·offset을 매회 추출한다. 원본 JSON을 수정하지 않고 실제 로드한 `scenario.json`, 회차·seed·추출값을 각 `input.json`/결과 JSON에 남긴다. 같은 seed/JSON/회차의 파라미터는 재시도 때도 같지만 MORAI native RandomMode 차종·physics까지 고정하지는 않는다. PASS/FAIL·bag·영상은 회차별로 구분한다.

GUI의 **장면 선택**은 검색·일괄 체크를 지원한다. 파일 패턴을 직접 바꾸면 기존 `include` 목록을 해제하고 패턴으로 실행한다. 반복은 GUI에서 수정·저장하며 랜덤 seed/범위는 YAML에서만 바꾼다. 목적지 등 회색 필드를 프로그램으로 조작해도 GUI 저장 대상에는 포함되지 않는다. VSCode의 다른 항목 변경은 보존하고 같은 항목의 충돌은 재불러오도록 거절한다.

현재 팀 production의 일반3초·고주로native10초를 검증기에서도 대조하려면 새 개인 YAML에
다음을 명시한다. 검증기에서 생략한 값은 과거 호환 기본3초·mixed OFF·append8초를 유지하며,
기존 실행 snapshot을 자동 변경하지 않는다. 명시적 native는10.0초까지, append는기존9.9초까지
허용한다. ROS1 predictor의101-pose 지원이며 ROS2 bounded-array 호환 인증은 아니다.

```yaml
stack:
  prediction_horizon: 3.0
  prediction_mixed_horizon_enabled: true
  prediction_highway_horizon: 10.0
  prediction_highway_mode: native
```

설정 검증기 업데이트 뒤 기존 GUI는 다시 열어야 한다. 이 선택은 예측 정확도·LC 안전성 인증이
아니며, 실제 전달값은 실행의 tracking 명령과 `/map_based_prediction` 파라미터를 확인한다.

### planning 빠른 테스트

개인 YAML의 JSON 폴더에서 1–2개를 선택하고 GT/BEVFusion·GT/측위·신호등을 대조한다. **선택한 JSON 빠른 실행**은 원본을 바꾸지 않고 별도 임시 YAML/입력 사본으로 1회씩 실행한다. 랜덤 파라미터·Drive 업로드는 off, 목적지·시간·네트워크·녹화는 현재 저장된 YAML을 사용한다. 검색·Ctrl/Shift 선택·환경 검사·ROS 수신 검사·연결 JSON 경로 복사만 이 탭에 둔다.

### 설정 파일과 실행 기록

개인 YAML은 계속 수정·재사용한다. **새 실험 시작**은 `planner_logs/validation/<이름>_<YYMMDD_HHMMSS>/`에 당시 YAML·JSON·Network·토픽 목록 사본/hash를 저장한다. 같은 YAML을 내일 다시 사용해도 된다. **실험 기록 / 재개 → 선택 실험 재개**는 현재 원본이 아닌 당시 입력 사본을 사용하고 완료한 PASS/FAIL 회차를 건너뛴다. 사본을 직접 수정하면 거절한다. ROS 코드·맵·모델 바이너리는 복제하지 않는다.

`모니터 모드`·반반 배치·화면 녹화·실시간 실패 근거·배치·Drive 공유 기능은 유지한다. 모니터 전환 자체가 YAML의 화면 배치 값을 바꾸지는 않는다. `split` 실행은 하단 모니터 바로 전환하며 전체 설정 보기로 돌아온다. 영상은 전체 화면이므로 다른 앱으로 가리거나 화면을 잠그지 않는다.

YAML 하나의 목적지·시간·도착 방식·스택은 모든 JSON에 공통 적용한다. 다른 목적지면 별도 YAML이다. 현재 형식은 `schema_version: 2`; 이전 v1은 확인만 가능하며 새 주석 예제로 옮겨야 한다. 장면별 .case.yaml은 실행 설정으로 사용하지 않는다.

## 여러 YAML 배치

개인 YAML을 먼저 저장한 뒤 `실행 목록 / 배치`에서 YAML 추가 → `전체 검사` → `새 배치 시작`을 사용한다. 제거·↑/↓ 정렬은 `목록 관리` 메뉴에 있다. 배치 안에서 목적지·스택은 달라도 되지만 Sensor/Ego Network 및 ScenarioLoad 대상은 같아야 한다. 같은 YAML의 중복과 같은 ROS master의 중복 validator 실행을 거절한다. 목록은 `~/.config/ajounice/planning_scenario_validator/batch.json`에 자동 저장하며 Git에 포함되지 않는다.

주행 FAIL은 다음 YAML을 진행하고 시스템 오류는 멈춘다. B 실행 중 중단한 A/B/C 목록을 `배치 재개`하면 완료된 A와 완료된 장면은 건너뛴다. 원본 설정은 수정해도 되지만 실행한 목록과 입력 사본은 고정된다. `목록 초기화`는 결과를 지우지 않으며 이전 결과의 `queue.json`을 `이전 목록 열기`로 복원한다. 같은 YAML 목록을 처음부터 다시 시험하려면 `새 배치 시작`을 누른다. 모니터에는 배치 i/n과 현재 장면 진행이 표시된다.

배치 결과는 `planner_logs/validation_batches/<id>/{state.json,queue.json,summary.csv}`다. GUI의 `배치 CSV`·`배치 결과 폴더`로 연다. CSV는 UTF-8 BOM이며 스프레드시트 가져오기를 지원한다. 각 실험의 bag·MP4·JSON은 `planner_logs/validation/<실험 이름>_<YYMMDD_HHMMSS>/`에 남는다. 실험별 Drive 공유는 기존 옵션을 유지하며, 배치 CSV 자체는 자동 업로드하지 않는다.

`split`은 영상 자체를 자르는 기능이 아니다. `recording.video.window_layout: split`이면 시나리오마다 Dock·Panel을 제외한 X11 작업 영역에 RViz를 왼쪽 위, GUI 모니터 바를 왼쪽 아래, MORAI를 오른쪽 **전체 높이**로 배치한 뒤 X11 화면 **전체**를 녹화한다. `bottom_bar_px: auto`는 작업 영역 높이의 약 17%(170~275 px)를 왼쪽 RViz 아래에 남긴다. PC에 맞춰 고정 높이 `180`(100~400 px 입력)이나 비율 `"18%"`(10~30%)로 바꿀 수 있다. GUI 내용이 잘리지 않도록 실제 예약 높이는 최소 140 px로 제한한다. GUI와 RViz는 같은 설정값을 사용하며, GUI 창은 테두리를 포함한 실제 바깥 크기로 배치를 시도한다. MORAI 창을 못 찾거나 창이 크기 조절을 거부하면 경고만 기록하고 녹화·주행은 계속하므로 직접 배치하면 된다. 기존 예제의 `window_layout: manual`(기본값)과 `focus_planviz: true`는 기존 RViz 최대화 방식 그대로다. GUI 없이 CLI로 split을 실행하면 왼쪽 아래 공간은 비어 있으므로 필요에 따라 직접 창을 채운다.

첫 장면에서는 RViz 창이 생성된 뒤 저장된 창 크기를 늦게 복원할 수 있어, 약 6초간 초기화를 기다린 다음 반반 배치하고 1.5초간 유지 여부를 확인한다. 이후 장면에서는 이 대기를 반복하지 않는다. 실제 창 크기가 요청값에서 16px 이상 벗어나면 성공으로 처리하지 않고 수동 배치 경고를 남긴다. `split`인데 창이 원래 크기로 되돌아간다면 먼저 **실행 중이던 GUI를 닫고 다시 실행**하여 새 배치 코드를 적용한다.

## 신호등 실행 환경 선택

GUI 기본 설정은 신호등 on/off만 선택하며, **신호등 환경** 탭에서 시스템 Python / 가상환경 Python·경로·GPU/CPU 장치를 설정한다. 기본은 기존 `/usr/bin/python3`이며, 새 팀원은 `bash scripts/runtime/setup_traffic_light_env.sh`로 공용 환경을 준비한 뒤 가상환경을 선택한다. 설치는 시스템 Python/기존 Conda를 변경하지 않고 홈 폴더 아래 별도 환경을 만든다. 기본 경로는 `~/.local/share/ajounice/traffic_light_gpu/bin/python`, 장치는 `0`이다. CPU 시험은 `cpu`로 바꾼다. 신호등 모델은 Git에 포함돼 있으며 BEVFusion Docker와 별개다.

YAML 키는 `stack.traffic_light`, `traffic_light_runtime: system|virtualenv`, `traffic_light_python`, `traffic_light_device`이다. 기존 YAML의 `traffic_light_python`만 지정한 형식도 호환한다. off에서는 환경 설치가 필요 없고, on이면 실행 직전 모델/GPU/ROS import를 검사하여 실패 시 주행을 시작하지 않는다. 설정 검사 버튼은 실제 GPU 추론까지 수행하지 않는다. threshold/스무딩/지도 매칭 정책은 바꾸지 않는다. 최초 설치 및 시스템 환경 검사 명령은 [팀원 가이드](../../../../docs/planning/SCENARIO_VALIDATOR_TEAM_NOTION_GUIDE.md#7-1-신호등도-켜려면--팀원은-공용-가상환경-추천)를 참고한다.

## 5분 실행

1. 아래 `install.sh`를 한 번 실행한다. 그러면 `ego_network_validator_local.json`이 대회 MR의 `SaveFile/Network/25.S4.MolitComp03/`에도 설치된다. MORAI MR에서 K-City 맵을 열고, YAML에 적힌 Sensor JSON과 이 Ego Network JSON을 각각 로드해 Connect한다. MORAI GUI의 최초 Connect는 사용자가 한 번 한다. 시나리오 전환은 실행기가 자동으로 한다. 평소의 `ego_network_competition_local.json`을 대신 로드하면 ScenarioLoad가 없어 자동 전환이 실패한다.
2. 저장소 루트에서 실행한다.

```bash
cd ~/AjouNice2026
bash base_ws/src/planning_addons/planning_scenario_validator/install.sh
/usr/bin/python3 base_ws/src/planning_addons/planning_scenario_validator/scripts/build_static_corridor_cases.py
mkdir -p "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER"
cp base_ws/src/planning_addons/planning_scenario_validator/config/missions/static_obstacle/base/static_mission_gt.yaml \
   "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER/campaign.yaml"
./planning_validator "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER/campaign.yaml" --dry-run
./planning_validator "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER/campaign.yaml"
```

다른 터미널의 대시보드:

```bash
./planning_validator_dashboard planner_logs/validation/<실행ID>
```

Ctrl+C 후에는 `./planning_validator --execution planner_logs/validation/<실행ID>`로 재개한다. 같은 YAML 명령을 반복하면 새 실험을 처음부터 시작한다. 재개 때 실패 시나리오도 다시 보려면 `--rerun-failed`를 붙인다. 개별 오류는 `error/`에 보존한다. `--check`/`--dry-run`은 새 결과 폴더를 만들지 않는다.

BEVFusion까지 켜는 시험은 별도 예제를 복사해서 사용한다. 먼저 무장애물 연결 확인용 `bevfusion_clear_smoke.yaml`을 권장한다. **일반 정적 장애물 구간**은 아래의 `static_mission_bevfusion.yaml`을 사용한다. `static_obstacle_bevfusion.yaml`과 `static_obstacle_gt.yaml`은 이름과 달리 GPS 음영 구간 안의 hidden mission regression이므로 이번 일반 구간과 혼동하지 않는다. MORAI GUI의 Sensor는 예제에 적힌 `Sensor_competition_local.json`, Ego Network는 validator 전용 JSON으로 연결해야 한다. BEVFusion checkpoint·Docker image도 `scripts/runtime/run_bevfusion.sh`의 계약대로 준비해야 한다.

```bash
cp base_ws/src/planning_addons/planning_scenario_validator/config/missions/static_obstacle/base/bevfusion_clear_smoke.yaml \
   "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER/bevfusion_campaign.yaml"
./planning_validator "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER/bevfusion_campaign.yaml" --dry-run
./planning_validator "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER/bevfusion_campaign.yaml"
```

배포 예제의 공통 녹화 설정은 `basic.json` + `include_sensors: false` + 화면 영상 `enabled: true`다. 음영 구간 BEVFusion 예제도 같은 설정을 사용한다. 원시 센서까지 bag에 남겨 인지 문제를 재현해야 할 때만 `include_sensors: true`로 변경한다. 이 옵션은 **bag 녹화 토픽만** 바꾸며 BEVFusion 추론 입력은 그대로 유지된다. `FAIL/TIMEOUT` 같은 실제 주행 실패와 `ERROR/SYSTEM_ERROR` 같은 자동화·환경 오류는 별도로 분류한다.

일반 정적 장애물 구간의 두 CargoBox 변형을 생성·비교하려면 다음을 실행한다. 두 YAML은 **같은 scenario와 sensor/goal**을 사용하고 detector만 다르다. 원본 샘플의 CargoBox scale `2×3×2`와 비교용 native `1×1×1`을 포함하며, GPS 음영과 움직이는 NPC는 제거한다. 시나리오는 Git에 추적하지 않는 `planner_logs/validation_inputs/static_corridor/`에 생성된다.

```bash
/usr/bin/python3 base_ws/src/planning_addons/planning_scenario_validator/scripts/build_static_corridor_cases.py
cp base_ws/src/planning_addons/planning_scenario_validator/config/missions/static_obstacle/base/static_mission_bevfusion.yaml \
   "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER/static_mission_bevfusion.yaml"
./planning_validator "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER/static_mission_bevfusion.yaml" --dry-run
./planning_validator "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER/static_mission_bevfusion.yaml"
# GT 비교도 static_mission_gt.yaml을 같은 방식으로 복사하되 campaign_name을 다르게 둡니다.
```

빌더의 `soa_corridor_*`는 Base3 **OD 벤치마크용 합성 배치**다. 단일 CargoBox도 planning에서 정지하거나 충돌할 수 있으므로 이를 공식 샘플의 회피 가능 장면으로 취급하지 않는다. 일반 정적 장애물 구간의 기본 검증은 `soa_official_*`로 시작한다.

다른 장애물 종류와 배치에 대한 **비공식 스트레스 테스트**는 `static_catalog_gt.yaml`과 `static_catalog_bevfusion.yaml`로 실행한다. 두 설정은 같은 `soa_catalog_*` 8개 장면(PE_Drum, TrafficBarrel, Concrete_Barricade01, PE_Firewall_Orange 각각 2개 위치)을 고른다. `sample_pose`는 배포 샘플 CargoBox의 위치이며, `pathward_080`은 무장애물 주행 경로 쪽으로 x를 0.8m 이동시킨 위치다. 다른 asset을 그 위치에 놓은 것이므로 대회 출제 장면·공식 성공률로 해석하지 않는다. GT 실행에서 배치 자체가 회피 가능한지 먼저 보고, 동일 장면을 BEVFusion으로 비교한다.

```bash
/usr/bin/python3 base_ws/src/planning_addons/planning_scenario_validator/scripts/build_static_corridor_cases.py
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/missions/static_obstacle/base/static_catalog_gt.yaml --dry-run
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/missions/static_obstacle/base/static_catalog_gt.yaml
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/missions/static_obstacle/base/static_catalog_bevfusion.yaml
```

두 장애물을 연속으로 피해 가는 **비공식 지그재그 스트레스 테스트**는 `static_pair_mission_gt.yaml`/`static_pair_mission_bevfusion.yaml` 예제를 사용한다. PE_Drum과 TrafficBarrel을 약 17m 간격으로 배치한 한 장면(`soa_pair_05*`)을 선택하며, 두 물체가 각각 단독으로도 통과한 배치다. GT와 BEVFusion을 같은 장면·goal로 비교한다. 빌더가 함께 만드는 `soa_pair_01`~`04`와 `soa_pair_control_*`은 실패 원인을 찾기 위해 사용한 실험 장면이며 기본 실행 예제에는 포함하지 않았다. 여러 장애물 결과를 단일 물체 성공률에 합산하지 않는다.

```bash
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/missions/static_obstacle/base/static_pair_mission_gt.yaml
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/missions/static_obstacle/base/static_pair_mission_bevfusion.yaml
```

두 명령은 동시에 실행하지 않는다. 각각 스택을 자체 실행·종료하며, 첫 명령이 끝난 뒤 두 번째를 실행한다.

**실패를 찾는 본 검증**은 통과 장면만 고르지 않는다. `static_pair_stress_gt.yaml`과 `static_pair_stress_bevfusion.yaml`은 `soa_pair_01`~`05`를 모두 실행한다. 이전에 GT 기준으로 미출발·중도 정지가 재현된 장면도 분모에 포함해, 조건을 바꾸었을 때 실패가 사라지거나 새로 생기는지 본다. 한 캠페인이 끝난 다음 다른 캠페인을 실행한다.

```bash
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/missions/static_obstacle/base/static_pair_stress_gt.yaml
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/missions/static_obstacle/base/static_pair_stress_bevfusion.yaml
```

## 팀원이 해야 할 일

시나리오 폴더에는 MORAI 시나리오 `.json`만 넣고, 실행 설정은 workspace의 캠페인 YAML 하나에 둔다. 목적지나 제한시간·스택이 달라지면 YAML을 복사하고 `campaign_name`을 바꾼다. [시나리오·YAML 제작 가이드](../../../../docs/planning/SCENARIO_VALIDATOR_AUTHOR_GUIDE.md)를 참고한다.

같은 폴더의 일부 장면은 YAML의 `scenarios.include: [파일명.json, ...]`로 지정한다. include는 glob보다 우선한다. 전체 실행은 include 삭제 + glob 지정이다. GUI에서 1–2개를 선택하는 대조는 planning 빠른 테스트 탭을 사용하며 원본 YAML을 수정하지 않는다. 빈 목록·없는 파일·폴더 밖 경로·중복은 오류이며 전체로 자동 대체하지 않는다. 새 파일 자동 포함은 패턴 모드에서만 이루어진다. 선택한 장면만 실행 사본에 복제되므로 배치와 중단·재개도 동일한 입력 계약을 사용한다. 시나리오 데이터나 판단·제어 로직을 변경하는 기능은 아니다.

```text
workspace/
  narrow_gap.yaml       # 공통 목적지·시간·스택
  narrow_gap_long.yaml  # 다른 조건의 별도 캠페인
  scenarios/
    soa_narrow_gap_01.json
    soa_narrow_gap_02.json
```

```yaml
# 전체 캠페인 YAML에서 이 부분만 공통 주행 조건입니다.
default_case:
  goal: {x: -13.382585757, y: 93.782754520, z: 28.489, yaw_deg: 60.815}
  waypoints: []
  timeout_sec: 75
  arrival_mode: stop
```

`scenario_install_dir: auto`는 `~/MoraiLauncher_Lin/`의 대회 MR을 선택한다. 팀원 PC에서 경로가 다르면 해당 값을 직접 지정한다. 실행기는 원본 JSON을 수정하지 않고 MR SaveFile에 ExternalCtrl/정지 시작 사본과 `_MN.json`을 설치한다.

목적지·경유점은 MORAI scenario JSON의 로컬 좌표가 아닌 planning의 `map` 좌표로 입력한다. 일반 정적 구간 예제 goal은 v85 지도에서 장애물 약 25m 뒤의 차로 중심이다. 모든 장면이 그 목적지로 주행 가능하도록 묶는다. 다른 구간에서 끝내려면 별도 이름의 캠페인 YAML을 만든다.

## 녹화 및 판정

2026-10-14: 기본 녹화에 E24/E25 status·latency 및 `tracking/measurement_objects`를 포함하며 원본 센서 기본값은 계속 OFF다. **지연 분석 (센서 제외)**는 planner processing-time을 함께 수집한다. 모든 bag ON 모드는 마감 후 실제 메시지 수를 검사하고 주행 PASS/FAIL과 녹화 경고를 별도로 저장·표시한다. 구 실행에 복사된 목록은 자동 덮어쓰지 않는다. [상세 계약·구 bag 검사](../../../../docs/planning/SCENARIO_VALIDATOR_RUNBOOK.md#기본-녹화지연-분석-계약-2026-10-14)를 따른다.

2026-10-13 (`dataset/base5`): GUI의 **rosbag 녹화**에서 `끄기 / 경량 / 기본 / 인지 분석 / 직접 선택`을 고른다. **화면 영상 녹화**는 별도이다. `토픽 선택…`은 검색·그룹·일괄 선택을 제공하며 선택을 YAML에 저장한다. bag OFF는 rosbag 프로세스 자체를 만들지 않지만 실시간 판정·JSON·CSV를 유지한다. 기존 YAML의 `bag_enabled` 생략은 ON이다. 변경은 **YAML 저장 → 새 실험 시작**에 적용되고, 재개는 보관된 설정을 사용한다. 상세 사용법은 [짧은 런북](../../../../docs/planning/SCENARIO_VALIDATOR_RUNBOOK.md#rosbag-onoff토픽-선택-2026-10-13)을 참고한다.

- `basic.json`: camera/LiDAR 제외, planning/인지/제어/GT 핵심 토픽
- `lightweight.json`: 위치, trajectory, command, collision 중심
- `full.json`: 알려진 센서·planning 토픽 목록. 실제 수신하지 않는 항목은 rosbag에 들어가지 않는다.
- 영상은 X11 전체 화면을 시나리오마다 ffmpeg로 녹화한다. `recording.video.display`는 MORAI와 Planviz가 뜬 X11 화면(보통 `:0.0`)과 같아야 한다. `window_layout: split`은 RViz 최대화를 하지 않고 반반 배치한다. 기존 `window_layout: manual`의 `focus_planviz: true`는 녹화 직전 Planviz를 전면·최대화한다. 다른 창에 포커스를 주면 전체 화면 영상에도 그 창이 찍힌다.
- 충돌은 `/CollisionData`의 객체 ID를 실시간으로 읽는다. 한 시나리오에서 같은 ID의 접촉 메시지가 끊겼다 재등장해도 1개 객체로만 센다. goal 반경, 속도 및 유지 시간으로 성공을 판단한다. 지정 시간 미도달은 TIMEOUT이다.
- `result.json`은 개별 증거용, `summary.csv`는 사람에게 보여주기 위한 전체 표, `events.jsonl`은 시간 순서 감사 기록이다. 실패 분석기는 녹화 재생 없이 실시간 구독값과 해당 장면의 스택 로그만 사용한다. `result.json`의 `failure_analysis`에 증상 분류, 수치 근거, 로그 징후, 의심 단계와 신뢰도를 기록한다. 탐지 토픽 수신 횟수와 실제 객체가 포함된 메시지 수를 별도로 세므로, **토픽이 살아 있음**과 **객체가 검출됨**을 혼동하지 않는다. `summary.csv`에는 `failure_category`, `suspected_stage`, `evidence_summary`가 추가된다. 이는 코드 수준의 근본 원인 확정이 아니다.
- 켜둔 녹화는 ScenarioLoad 전에 시작한다. Route 요청이 거부돼도 선택한 bag·MP4 및 이벤트가 남는다. bag·영상이 모두 OFF여도 이벤트·JSON·CSV 판정 결과는 남는다. CSV의 `bag_enabled`, `video_enabled`로 의도적인 녹화 OFF를 구분한다. `elapsed_sec`은 주행 판정 시간이고, 새 결과의 `wall_elapsed_sec`은 준비와 정리를 포함한 전체 실행 시간이다.

예제의 주행 제한은 카탈로그 35초, 일반 정적 장애물·2객체·무장애물 50초, GPS 음영 구간 75초다. 미션별 스타터는 미션에 따라 65~90초다. 이 값은 **controller 준비 후 실제 주행**에만 적용되며, ScenarioLoad·측위·경로 생성 대기에는 적용되지 않는다. 실제 제한시간은 항상 캠페인 YAML의 `default_case.timeout_sec`다. 더 긴 실험은 새 이름의 YAML에서 공통 값을 늘린다. `build_static_corridor_cases.py`의 `metadata/manifest.json`은 제작 참고 자료이고 실행 조건을 덮어쓰지 않는다. 기존 폴더에 남은 `.case.yaml`은 경고 후 무시한다.

영상 구간 최적화는 후순위다. 현재처럼 ScenarioLoad 전부터 영상을 녹화하면 궤적 미생성·스택 시작 실패도 영상으로 확인할 수 있다. 실제 센서 제외 성공 녹화에서 bag은 약 50 MiB/분, MP4는 약 10 MiB/분이어서, 영상 앞부분만 잘라도 전체 저장 용량은 크게 줄지 않는다. 향후 영상 on/off 시 MORAI·BEVFusion 주기와 CPU 부하를 비교하고, 필요할 때만 궤적 준비 후 시작을 실험한다. 이 방식에서는 궤적 미생성 실패의 스크린샷을 별도로 남겨야 한다. 결과 문구를 끝 영상에 넣으려면 현재의 `recorder.stop()` 이후 GUI 결과 갱신 순서를 바꾸고, 실제 표시를 확인한 뒤 약 1초 녹화하는 기능이 필요하다. **이 항목들은 아직 구현하지 않았다.**

출력 예:

```text
planner_logs/validation/soa_static_mission_gt_example/
  campaign_state.json
  summary.csv
  stack_logs/
  success/soa_.../
    planbag_soa_..._P_20261002_123456.bag
    planbag_soa_..._P_20261002_123456.mp4
    planbag_soa_..._P_20261002_123456.json
    events.jsonl
  failure/...
  error/...
  running/...
```

Google Drive 공유는 캠페인 YAML의 `upload.enabled`, `upload.profile`, `upload.rclone_remote`로 설정한다. GUI 기본 탭에서 업로드 on/off·공유 모드를 수정하고 YAML 저장을 누른다. remote 경로는 YAML에서 수정한다. **업로드는 기본적으로 꺼져 있다.** 경로 입력칸과 새 예제의 기본값은 `ajou_drive:planning_validator`다. 이는 맵 다운로드/업로드 스크립트와 같은 공유 `AjouNice2026` 폴더 ID를 사용하므로, 실제 위치는 `AjouNice2026/planning_validator/<campaign_name>/...`이다. `AjouNice2026`을 경로에 한 번 더 쓰지 않는다. 공유 폴더에 `planning_validator/`가 이미 존재하는 것은 읽기 전용으로 확인했다. 각 PC에서 `ajou_drive` rclone remote를 미리 설정해야 하며, remote 이름이 다르면 `rclone_remote`를 수정하고 `upload.drive_root_folder_id`에 공유 폴더 ID를 명시한다. 기존에 빈 경로로 저장된 개인 YAML은 업로드를 켜기 전까지 자동으로 수정하지 않아 resume 해시를 보존한다. 시나리오 종료 직후 `success/failure/error/<장면 폴더>` 구분을 유지하여 업로드하고, 캠페인 루트의 `summary.csv`, `campaign_config.yaml`, `campaign_state.json`도 갱신해 올린다. `summary.csv`의 `result_path`는 공유 폴더 기준 상대경로다.

| `upload.profile` | 장면별 공유 파일 |
| --- | --- |
| `basic` | 결과 JSON·이벤트·시나리오·로그 등 작은 분석 자료; bag·MP4 제외 |
| `bag` | 기본 공유 + `.bag` |
| `video` | 기본 공유 + `.mp4` |
| `full` | 기본 공유 + `.bag` + `.mp4` |

로컬 bag과 영상은 어느 모드에서나 보존된다. Drive 업로드 오류는 주행 PASS/FAIL과 별개로 `campaign_state.json`의 `upload_errors`와 GUI에 기록된다. 원본 YAML의 업로드 설정을 바꾸면 다음 새 실험에 적용하며, 기존 실험 재개는 당시 사본의 업로드 설정을 유지한다. 특정 과거 bag만 재주행 없이 추가 업로드하는 기능은 아직 없다.

## 운영 계약

고속 구간에서 대회 Route B의 선호 차선을 사용하되 기존 검증 목적지를 유지하려면
개인 YAML에 다음을 지정한다. 현재 작업용 설정은 `highway/workspace/highway_125_route_b.yaml`이다.

```yaml
route:
  name: route_b
  definition_file: base_ws/src/planning_addons/competition_route_requester/config/kcity_v87_competition_route.yaml
```

실행기는 현재 위치부터 `default_case.goal`까지의 B 구간만 선택하고, 해당 차로의 중심점을
경유점으로 요청한다. 최종 route의 모든 선호 차선이 B 목록과 순서대로 일치하고 새 궤적이
준비되어야 주행을 시작한다. 후보 차선이 없거나 순서가 다르면 기존 차선을 그대로 사용하는
대신 실행 오류로 중단한다. `route_profile.json`과 결과 JSON에 요청·확인한 차선 목록을 남기며,
경로 정의 YAML도 실행 사본에 고정한다. 이 설정이 없는 다른 미션과 과거 실행 사본의 동작은 유지한다.

- 실행 전 ROS graph에 기존 `/ctrl_cmd`, `/planning/trajectory`, localization, detector publisher가 있으면 중복 실행을 막는다. 기존 수동 스택을 종료한 뒤 사용한다.
- custom goal은 `/planning/set_waypoint_route`로 보낸다. 대회 Route A/B requester는 실행하지 않는다.
- `pose_source: localization`은 MR `/Competition_topic` + -1.347° 보정을 사용한다. `maptf_morai`는 `/Ego_topic`을 사용한다.
- `detector: od_gt`는 `/Object_topic`이 필요하다. `bevfusion`은 카메라 4대·LiDAR가 있는 Sensor JSON과 기존 BEVFusion image/checkpoint가 필요하다.
- BEVFusion 실행기에는 YAML의 Sensor JSON을 그대로 전달한다. 모델 준비뿐 아니라 실제 detection 토픽의 새 메시지가 확인돼야 route/controller 단계로 넘어간다.
- 신호등 off는 신호 인지 노드와 planner 소비 모듈을 함께 끈다.
- 실행기에서 최초 MORAI GUI Sensor/Ego Network Connect는 자동화하지 않는다. topic 준비 검사를 통해 연결 누락을 찾는다.
- `install.sh`는 기존 대회용 Ego UDP 설정의 `PUBSUB_TYPE=783` 슬롯을 검증된 UDP ScenarioLoad(9095)로 바꾼 전용 JSON을 생성한다. 이 슬롯의 기존 `/GetTrafficLightStatus` UDP 요청 기능은 전용 프로필에서 사용하지 않으며, 카메라 기반 신호등 인지 노드와는 별개다. 기존 `ego_network_competition_local.json`은 수정하지 않는다. GUI에서 전용 JSON을 로드해야 한다.
- 현재 영상은 Planviz 창 단독 캡처가 아니라 전체 X11 화면이다. 다른 창이 덮이면 영상에도 찍힌다.
- 새 ROS 패키지로 `roslaunch planning_scenario_validator validator.launch config:=...`를 쓰려면 workspace를 한 번 빌드해야 한다. 저장소 루트 `./planning_validator`는 별도 패키지 빌드 없이 실행할 수 있다.

개발 상태와 남은 실차 검증은 [설계·검증 기록](../../../../docs/planning/scenario_validator_automation.md)을 참고한다.
