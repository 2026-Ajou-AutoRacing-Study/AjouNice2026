# Planning scenario validator

팀원이 만든 MORAI scenario JSON 폴더를 대상으로, 실제 localization → 인지 → tracking/prediction → planning → controller 폐루프 주행을 시나리오별로 검증한다. CLI와 얇은 설정 GUI를 제공하지만 MORAI MR의 최초 네트워크 Connect에는 MORAI GUI가 필요하다. 각 시나리오는 bag, 화면 영상, 실시간 판정 결과로 따로 저장한다.

## GUI로 시작하기

각자 PC의 `~/AjouNice2026` 저장소 루트에서 `bash base_ws/src/planning_addons/planning_scenario_validator/install.sh`를 한 번 실행한다. 처음 시나리오 예제를 만들 때는 `/usr/bin/python3 base_ws/src/planning_addons/planning_scenario_validator/scripts/build_static_corridor_cases.py`를 실행한다. YAML의 저장소 상대경로는 실행 시 해당 PC의 저장소 경로로 해석되며 `/home/luv`를 요구하지 않는다. MORAI MR GUI에서 사용할 Sensor JSON과 생성된 validator 전용 Ego Network JSON을 Load·Connect한다. 이 단계는 GUI가 대신하지 않는다.

```bash
cd ~/AjouNice2026
./planning_validator_gui
# docs/common_alias.txt를 source했다면 pvgui
```

GUI는 마지막으로 열거나 저장한 YAML의 경로를 `~/.config/ajounice/planning_scenario_validator/gui_settings.json`에 기억한다 (`XDG_CONFIG_HOME` 설정 시 해당 경로 사용). 재실행하면 그 파일의 저장된 설정을 그대로 불러온다. 파일이 삭제되었거나 읽을 수 없으면 기본 예제로 돌아간다. 저장하지 않은 화면 수정값과 창 위치는 복원하지 않는다.

`모니터 모드`를 누르면 GUI가 RViz와 같은 폭으로 화면 왼쪽 아래에 놓인다. 바 높이는 기존 설정을 유지하며 `중단`, `결과 폴더`, `전체 설정 보기` 버튼이 남는다. 긴 상태 문구가 있어도 버튼은 위쪽에 표시한다. 최대화된 설정 창에서도 모니터 바로 전환할 수 있고, `전체 설정 보기`를 누르면 이전 창 상태로 돌아간다. 실행 전 이 버튼을 누르면 영상 설정을 `split`으로 바꾸며, 기존 `manual` YAML에서 바로 실행할 때도 RViz 전체화면 동작을 안내하고 `split` 변경 여부를 묻는다. 이미 실행 중인 캠페인은 YAML이 고정되어 있으므로 화면 배치를 지속적으로 바꾸려면 안전 중단 후 새 캠페인 이름으로 다시 실행해야 한다.

처음에는 통과 확인된 한 장면의 `static_pair_mission_gt_split.yaml` 예제가 선택된다. RViz 왼쪽·MORAI 오른쪽·GUI 하단 모니터 바의 전체 화면 영상용 설정이다. 실패 장면을 찾는 5개 장면 스트레스 예제는 목록의 `static_pair_stress_gt.yaml`에서 선택한다. 시나리오 폴더·glob·목적지(map 좌표)·제한 시간·측위·탐지기·녹화 옵션을 확인한 뒤 `설정 검사` → `실행 / resume`을 누른다. 예제 파일은 바뀌지 않고 `config/users/<리눅스 사용자>/<campaign_name>.yaml`에 개인 설정이 생성된다. 폴더 안의 시나리오별 `.case.yaml` 목적지/시간 override와 GUI에 보이지 않는 고급 YAML 옵션도 보존된다. 직접 저장한 YAML은 `내 YAML 열기 / resume`으로 다시 열어 이어서 실행한다. 한 번 실행된 캠페인의 설정을 바꾸면 기존 기록 보호를 위해 새 캠페인 이름으로 복사할지 GUI가 묻는다. **그대로 이어 실행할 때는 기존 개인 YAML을 열고 값을 바꾸지 않는다.**

GUI는 기존 `./planning_validator`만 호출한다. 실행 중 완료 장면의 PASS/FAIL/ERROR와 현재 단계, 주행 시간·목적지 거리·속도·충돌 횟수를 실시간으로 표시한다. `split` 영상 설정으로 실행하면 설정·결과 표는 접고 GUI를 화면 하단의 얇은 모니터 바로 자동 전환한다. `전체 설정 보기`로 언제든 펼칠 수 있지만 펼친 창도 영상에 찍힌다. 실패 장면을 더블클릭하면 근거 수치와 조사 우선 단계를 볼 수 있다. 별도 대시보드는 필수가 아니다. `중단`은 결과를 보존하고 다음 실행에서 resume한다. 기본 resume는 완료된 PASS/FAIL을 건너뛴다. 같은 설정으로 실패 장면을 다시 돌리려면 `기존 FAIL도 다시 실행`을 체크한다. **실행기의 종료 코드 0은 전체 시나리오 PASS를 의미하지 않는다**. GUI의 성공/주행 실패/시스템 오류 건수와 `summary.csv`를 확인한다. GUI의 `중단` 버튼이나 **GUI를 실행한 터미널에서 Ctrl+C**로 안전 중단할 수 있다. 영상에는 전면의 다른 앱이 찍힐 수 있으므로 녹화 중 데스크톱 조작을 피한다. `설정 검사`가 통과해도 MORAI 연결·BEVFusion checkpoint·주행 성공까지 보증하지는 않는다. GUI가 필요 없는 서버/터미널에서는 아래 CLI를 그대로 사용한다.

`split`은 영상 자체를 자르는 기능이 아니다. `recording.video.window_layout: split`이면 시나리오마다 Dock·Panel을 제외한 X11 작업 영역에 RViz를 왼쪽 위, GUI 모니터 바를 왼쪽 아래, MORAI를 오른쪽 **전체 높이**로 배치한 뒤 X11 화면 **전체**를 녹화한다. `bottom_bar_px: auto`는 작업 영역 높이의 약 17%(170~275 px)를 왼쪽 RViz 아래에 남긴다. PC에 맞춰 고정 높이 `180`(100~400 px 입력)이나 비율 `"18%"`(10~30%)로 바꿀 수 있다. GUI 내용이 잘리지 않도록 실제 예약 높이는 최소 140 px로 제한한다. GUI와 RViz는 같은 설정값을 사용하며, GUI 창은 테두리를 포함한 실제 바깥 크기로 배치를 시도한다. MORAI 창을 못 찾거나 창이 크기 조절을 거부하면 경고만 기록하고 녹화·주행은 계속하므로 직접 배치하면 된다. 기존 예제의 `window_layout: manual`(기본값)과 `focus_planviz: true`는 기존 RViz 최대화 방식 그대로다. GUI 없이 CLI로 split을 실행하면 왼쪽 아래 공간은 비어 있으므로 필요에 따라 직접 창을 채운다.

첫 장면에서는 RViz 창이 생성된 뒤 저장된 창 크기를 늦게 복원할 수 있어, 약 6초간 초기화를 기다린 다음 반반 배치하고 1.5초간 유지 여부를 확인한다. 이후 장면에서는 이 대기를 반복하지 않는다. 실제 창 크기가 요청값에서 16px 이상 벗어나면 성공으로 처리하지 않고 수동 배치 경고를 남긴다. `split`인데 창이 원래 크기로 되돌아간다면 먼저 **실행 중이던 GUI를 닫고 다시 실행**하여 새 배치 코드를 적용한다.

## 5분 실행

1. 아래 `install.sh`를 한 번 실행한다. 그러면 `ego_network_validator_local.json`이 대회 MR의 `SaveFile/Network/25.S4.MolitComp03/`에도 설치된다. MORAI MR에서 K-City 맵을 열고, YAML에 적힌 Sensor JSON과 이 Ego Network JSON을 각각 로드해 Connect한다. MORAI GUI의 최초 Connect는 사용자가 한 번 한다. 시나리오 전환은 실행기가 자동으로 한다. 평소의 `ego_network_competition_local.json`을 대신 로드하면 ScenarioLoad가 없어 자동 전환이 실패한다.
2. 저장소 루트에서 실행한다.

```bash
cd ~/AjouNice2026
bash base_ws/src/planning_addons/planning_scenario_validator/install.sh
/usr/bin/python3 base_ws/src/planning_addons/planning_scenario_validator/scripts/build_static_corridor_cases.py
mkdir -p "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER"
cp base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_mission_gt.yaml \
   "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER/campaign.yaml"
./planning_validator "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER/campaign.yaml" --dry-run
./planning_validator "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER/campaign.yaml"
```

다른 터미널의 대시보드:

```bash
./planning_validator_dashboard planner_logs/validation/soa_static_mission_gt_example
```

Ctrl+C 후 같은 명령을 다시 실행하면 완료된 시나리오를 건너뛰고 남은 시나리오를 진행한다. 실패 시나리오를 다시 보려면 `--rerun-failed`를 붙인다. 개별 오류가 났다면 `error/`에 증거를 남기고 다음 실행에서 재시도한다.

BEVFusion까지 켜는 시험은 별도 예제를 복사해서 사용한다. 먼저 무장애물 연결 확인용 `bevfusion_clear_smoke.yaml`을 권장한다. **일반 정적 장애물 구간**은 아래의 `static_mission_bevfusion.yaml`을 사용한다. `static_obstacle_bevfusion.yaml`과 `static_obstacle_gt.yaml`은 이름과 달리 GPS 음영 구간 안의 hidden mission regression이므로 이번 일반 구간과 혼동하지 않는다. MORAI GUI의 Sensor는 예제에 적힌 `Sensor_competition_local.json`, Ego Network는 validator 전용 JSON으로 연결해야 한다. BEVFusion checkpoint·Docker image도 `scripts/runtime/run_bevfusion.sh`의 계약대로 준비해야 한다.

```bash
cp base_ws/src/planning_addons/planning_scenario_validator/config/examples/bevfusion_clear_smoke.yaml \
   "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER/bevfusion_campaign.yaml"
./planning_validator "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER/bevfusion_campaign.yaml" --dry-run
./planning_validator "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER/bevfusion_campaign.yaml"
```

배포 예제의 공통 녹화 설정은 `basic.json` + `include_sensors: false` + 화면 영상 `enabled: true`다. 음영 구간 BEVFusion 예제도 같은 설정을 사용한다. 원시 센서까지 bag에 남겨 인지 문제를 재현해야 할 때만 `include_sensors: true`로 변경한다. 이 옵션은 **bag 녹화 토픽만** 바꾸며 BEVFusion 추론 입력은 그대로 유지된다. `FAIL/TIMEOUT` 같은 실제 주행 실패와 `ERROR/SYSTEM_ERROR` 같은 자동화·환경 오류는 별도로 분류한다.

일반 정적 장애물 구간의 두 CargoBox 변형을 생성·비교하려면 다음을 실행한다. 두 YAML은 **같은 scenario와 sensor/goal**을 사용하고 detector만 다르다. 원본 샘플의 CargoBox scale `2×3×2`와 비교용 native `1×1×1`을 포함하며, GPS 음영과 움직이는 NPC는 제거한다. 시나리오는 Git에 추적하지 않는 `planner_logs/validation_inputs/static_corridor/`에 생성된다.

```bash
/usr/bin/python3 base_ws/src/planning_addons/planning_scenario_validator/scripts/build_static_corridor_cases.py
cp base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_mission_bevfusion.yaml \
   "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER/static_mission_bevfusion.yaml"
./planning_validator "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER/static_mission_bevfusion.yaml" --dry-run
./planning_validator "base_ws/src/planning_addons/planning_scenario_validator/config/users/$USER/static_mission_bevfusion.yaml"
# GT 비교도 static_mission_gt.yaml을 같은 방식으로 복사하되 campaign_name을 다르게 둡니다.
```

빌더의 `soa_corridor_*`는 Base3 **OD 벤치마크용 합성 배치**다. 단일 CargoBox도 planning에서 정지하거나 충돌할 수 있으므로 이를 공식 샘플의 회피 가능 장면으로 취급하지 않는다. 일반 정적 장애물 구간의 기본 검증은 `soa_official_*`로 시작한다.

다른 장애물 종류와 배치에 대한 **비공식 스트레스 테스트**는 `static_catalog_gt.yaml`과 `static_catalog_bevfusion.yaml`로 실행한다. 두 설정은 같은 `soa_catalog_*` 8개 장면(PE_Drum, TrafficBarrel, Concrete_Barricade01, PE_Firewall_Orange 각각 2개 위치)을 고른다. `sample_pose`는 배포 샘플 CargoBox의 위치이며, `pathward_080`은 무장애물 주행 경로 쪽으로 x를 0.8m 이동시킨 위치다. 다른 asset을 그 위치에 놓은 것이므로 대회 출제 장면·공식 성공률로 해석하지 않는다. GT 실행에서 배치 자체가 회피 가능한지 먼저 보고, 동일 장면을 BEVFusion으로 비교한다.

```bash
/usr/bin/python3 base_ws/src/planning_addons/planning_scenario_validator/scripts/build_static_corridor_cases.py
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_catalog_gt.yaml --dry-run
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_catalog_gt.yaml
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_catalog_bevfusion.yaml
```

두 장애물을 연속으로 피해 가는 **비공식 지그재그 스트레스 테스트**는 `static_pair_mission_gt.yaml`/`static_pair_mission_bevfusion.yaml` 예제를 사용한다. PE_Drum과 TrafficBarrel을 약 17m 간격으로 배치한 한 장면(`soa_pair_05*`)을 선택하며, 두 물체가 각각 단독으로도 통과한 배치다. GT와 BEVFusion을 같은 장면·goal로 비교한다. 빌더가 함께 만드는 `soa_pair_01`~`04`와 `soa_pair_control_*`은 실패 원인을 찾기 위해 사용한 실험 장면이며 기본 실행 예제에는 포함하지 않았다. 여러 장애물 결과를 단일 물체 성공률에 합산하지 않는다.

```bash
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_pair_mission_gt.yaml
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_pair_mission_bevfusion.yaml
```

두 명령은 동시에 실행하지 않는다. 각각 스택을 자체 실행·종료하며, 첫 명령이 끝난 뒤 두 번째를 실행한다.

**실패를 찾는 본 검증**은 통과 장면만 고르지 않는다. `static_pair_stress_gt.yaml`과 `static_pair_stress_bevfusion.yaml`은 `soa_pair_01`~`05`를 모두 실행한다. 이전에 GT 기준으로 미출발·중도 정지가 재현된 장면도 분모에 포함해, 조건을 바꾸었을 때 실패가 사라지거나 새로 생기는지 본다. 한 캠페인이 끝난 다음 다른 캠페인을 실행한다.

```bash
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_pair_stress_gt.yaml
./planning_validator base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_pair_stress_bevfusion.yaml
```

## 팀원이 해야 할 일

시나리오 폴더에 `.json`을 넣는다. 공통 goal을 쓴다면 YAML의 `default_case.goal`을 한 번만 설정한다. 목적지가 다를 때만 같은 이름의 `.case.yaml`을 추가한다.

```text
scenarios/
  soa_narrow_gap.json
  soa_narrow_gap.case.yaml
```

```yaml
description: 좁은 S자 통과
goal: {x: -13.382585757, y: 93.782754520, z: 28.489, yaw_deg: 60.815}
waypoints: []
timeout_sec: 75
arrival_mode: stop
```

`scenario_install_dir: auto`는 `~/MoraiLauncher_Lin/`의 대회 MR을 선택한다. 팀원 PC에서 경로가 다르면 해당 값을 직접 지정한다. 실행기는 원본 JSON을 수정하지 않고 MR SaveFile에 ExternalCtrl/정지 시작 사본과 `_MN.json`을 설치한다.

목적지·경유점은 MORAI scenario JSON의 로컬 좌표가 아닌 planning의 `map` 좌표로 입력한다. 일반 정적 구간 예제 goal은 v85 지도에서 장애물 약 25m 뒤의 차로 중심이다. 장면마다 다른 지점에서 끝내려면 `.case.yaml`로 지도 좌표 목적지를 지정한다.

## 녹화 및 판정

- `basic.json`: camera/LiDAR 제외, planning/인지/제어/GT 핵심 토픽
- `lightweight.json`: 위치, trajectory, command, collision 중심
- `full.json`: 알려진 센서·planning 토픽 목록. 실제 수신하지 않는 항목은 rosbag에 들어가지 않는다.
- 영상은 X11 전체 화면을 시나리오마다 ffmpeg로 녹화한다. `recording.video.display`는 MORAI와 Planviz가 뜬 X11 화면(보통 `:0.0`)과 같아야 한다. `window_layout: split`은 RViz 최대화를 하지 않고 반반 배치한다. 기존 `window_layout: manual`의 `focus_planviz: true`는 녹화 직전 Planviz를 전면·최대화한다. 다른 창에 포커스를 주면 전체 화면 영상에도 그 창이 찍힌다.
- 충돌은 `/CollisionData`의 객체 ID를 실시간으로 읽는다. 한 시나리오에서 같은 ID의 접촉 메시지가 끊겼다 재등장해도 1개 객체로만 센다. goal 반경, 속도 및 유지 시간으로 성공을 판단한다. 지정 시간 미도달은 TIMEOUT이다.
- `result.json`은 개별 증거용, `summary.csv`는 사람에게 보여주기 위한 전체 표, `events.jsonl`은 시간 순서 감사 기록이다. 실패 분석기는 녹화 재생 없이 실시간 구독값과 해당 장면의 스택 로그만 사용한다. `result.json`의 `failure_analysis`에 증상 분류, 수치 근거, 로그 징후, 의심 단계와 신뢰도를 기록한다. 탐지 토픽 수신 횟수와 실제 객체가 포함된 메시지 수를 별도로 세므로, **토픽이 살아 있음**과 **객체가 검출됨**을 혼동하지 않는다. `summary.csv`에는 `failure_category`, `suspected_stage`, `evidence_summary`가 추가된다. 이는 코드 수준의 근본 원인 확정이 아니다.
- 녹화는 ScenarioLoad 전에 시작한다. Route 요청이 거부돼도 bag·MP4·이벤트가 남는다. `elapsed_sec`은 주행 판정 시간이고, 새 결과의 `wall_elapsed_sec`은 준비와 정리를 포함한 전체 실행 시간이다.

예제의 주행 제한은 카탈로그 35초, 일반 정적 장애물·2객체·무장애물 50초, GPS 음영 구간 75초로 짧게 잡았다. 이 값은 **controller 준비 후 실제 주행**에만 적용되며, ScenarioLoad·측위·경로 생성 대기에는 적용되지 않는다. JSON과 같은 이름의 `.case.yaml`에 `timeout_sec`이 있으면 예제 YAML의 `default_case.timeout_sec`보다 우선한다. `build_static_corridor_cases.py`가 만드는 sidecar도 함께 단축했으므로 기존 생성 폴더는 빌더를 다시 실행해야 새 값이 적용된다. 더 긴 실험이 필요하면 해당 `.case.yaml`만 늘린다. 기존 캠페인 결과와 새 시간제한 결과를 섞지 않도록 새 `campaign_name`으로 실행한다.

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

Google Drive 공유는 캠페인 YAML의 `upload.enabled`, `upload.profile`, `upload.rclone_remote`로 설정한다. GUI에서는 기본 설정 탭의 **Drive 자동 업로드·공유 모드**와 고급 탭의 **Drive rclone 경로**를 사용한다. **업로드는 기본적으로 꺼져 있다.** 경로 입력칸과 새 예제의 기본값은 `ajou_drive:planning_validator`다. 이는 맵 다운로드/업로드 스크립트와 같은 공유 `AjouNice2026` 폴더 ID를 사용하므로, 실제 위치는 `AjouNice2026/planning_validator/<campaign_name>/...`이다. `AjouNice2026`을 경로에 한 번 더 쓰지 않는다. 공유 폴더에 `planning_validator/`가 이미 존재하는 것은 읽기 전용으로 확인했다. 각 PC에서 `ajou_drive` rclone remote를 미리 설정해야 하며, remote 이름이 다르면 `rclone_remote`를 수정하고 `upload.drive_root_folder_id`에 공유 폴더 ID를 명시한다. 기존에 빈 경로로 저장된 개인 YAML은 업로드를 켜기 전까지 자동으로 수정하지 않아 resume 해시를 보존한다. 시나리오 종료 직후 `success/failure/error/<장면 폴더>` 구분을 유지하여 업로드하고, 캠페인 루트의 `summary.csv`, `campaign_config.yaml`, `campaign_state.json`도 갱신해 올린다. `summary.csv`의 `result_path`는 공유 폴더 기준 상대경로다.

| `upload.profile` | 장면별 공유 파일 |
| --- | --- |
| `basic` | 결과 JSON·이벤트·시나리오·로그 등 작은 분석 자료; bag·MP4 제외 |
| `bag` | 기본 공유 + `.bag` |
| `video` | 기본 공유 + `.mp4` |
| `full` | 기본 공유 + `.bag` + `.mp4` |

로컬 bag과 영상은 어느 모드에서나 보존된다. Drive 업로드 오류는 주행 PASS/FAIL과 별개로 `campaign_state.json`의 `upload_errors`와 GUI에 기록된다. 이미 실행한 캠페인의 업로드 설정을 바꾸면 resume 해시가 달라지므로 새 캠페인 이름을 사용한다. 특정 과거 bag만 재주행 없이 추가 업로드하는 기능은 아직 없다.

## 운영 계약

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
