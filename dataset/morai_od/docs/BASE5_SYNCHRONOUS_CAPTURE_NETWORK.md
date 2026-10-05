# Base5 MR 동기 캡처 Network — PREPARED

2026-10-06: [v2 CURRENT](BASE5_V2_CURRENT.md)의 v18 동적 거리 수정·live 복구 기록이 최신이다.
**본 수집은 보류다. 아래 v16 명령은 검증된 이전 smoke 세션 예시이며10K 시작 승인이 아니다.**

v18은 같은 one-load/seven-interface network를 사용한다. pause·선택적 actor-only load를
도입하지 않는다. 새 AI presence 계약으로 이전 output resume은 거부될 수 있으므로 새
output을 사용한다. Dashboard는 새 세션 output을 `--capture-root`로 지정하면 된다.
아래12-scene 성공은 이전 asset-review 이력이며 새10K 배치 성공을 뜻하지 않는다.

## v2 캡처·실시간 대시보드

터미널1/2는 아래 `ajou`, `rc`, rosbridge 순서를 따른다. SaveSensorData 수집 자체에는
UDP sensor receiver·localization·BEVFusion·planning·RViz가 필요하지 않다.
MR Network GUI에서 전용 JSON을 다시 연결하고 Capture Mode/Instance 저장을 확인한다.
현재 runner 기본은 `ego_network_mr_base5_savesensordata_collision_local.json`이다.
기존6-interface 프로필은 이력/일반 캡처용이며 새 안전 gate를 통과하지 못한다.

### 현재 v16 수집·중단·재개

대표16장 완료:13후보/3검토/0실행실패. 동적 거리2장·Dog2 가시성1장은 검토로 보존한다.
근접 회피8장은 모두 후보이며 full10K 검증이나 detector 성능 향상을 뜻하지 않는다.

```bash
ajou
ROOT="$HOME/morai_cap/base5_v2_261006"
python3 dataset/morai_od/scripts/run_base5_capture.py \
  --manifest "$ROOT/safety_pilot_v16/manifest.json" \
  --output "$ROOT/safety_capture_v16" --limit 16
```

위 명령은 완료된16장을 다시 캡처하지 않는다. 중단 후에도 **동일 manifest/output**으로
재실행한다. 완료 후보와 검토 장면을 건너뛰며 검토 장면을 자동 승인하지 않는다.
manifest/Network/sensor 계약 불일치는 중단한다. 완료 sensor/provenance 누락도 중단한다.
모두 완료된 세션은 ROS 없이 검증·skip하므로 MR 로드/제어 명령도 보내지 않는다.
실패/미기록 캡처 폴더는 `incomplete_attempts/`에 이동·보존한 뒤 새로 캡처한다.
MR SensorData에 남아 있는 파일은 사용자 소유일 수 있으므로 자동 삭제하지 않는다.
이번 시험에서는 raw 이동 후 강제 중단→재개→완료 세션 재실행을 모두 검증했고,
완료 파일의 hash/mtime과 ledger가 불변임을 확인했다.
마지막 ledger 행의 불완전 append는 원본 backup 후 복구하며 중간 행 손상은 중단한다.
동일 output runner 동시 실행은 lock으로 거부한다. `STOP` 파일로 멈춘 경우 재개 전에
해당 세션의 STOP 파일만 제거한다. 출력 경로를 바꾸면 새 수집이지 resume이 아니다.
실제 캡처 종료/중단 시에는 brake와 bounded Park 요청을 보낸다. Park 응답 미확인은
경고로 남으며 무한 대기하지 않는다. 다음 캡처 SceneLoad 뒤에는 다시 Drive를 확인한다.

브라우저 모니터:

```bash
ajou
python3 dataset/morai_od/scripts/base5_capture_dashboard.py \
  --capture-root "$HOME/morai_cap/base5_v2_261006/safety_capture_v16"
```

`http://127.0.0.1:8075`. 다른 dashboard가 이 포트를 사용 중이면 기존 서버를 종료하거나
스크립트의 port 옵션을 확인한다. raw/provenance와 최종 `smoke_report.json`은 위 output에 있다.
평균6.079초/장·최종 raw 약77.83MiB다.16장 선택 표본의 단순10K 외삽은 약16.9시간이며
통신 retry·검토 보충·환경 부하는 별도다. 본 수집은 아직 실행하지 않았다.

대시보드 터미널:

```bash
ajou
python3 dataset/morai_od/scripts/base5_capture_dashboard.py \
  --capture-root "$HOME/morai_cap/base5_v2_261006/capture_pilot_retry"
```

브라우저 `http://127.0.0.1:8075`에서2초마다 완료/후보/검토/실패 및 거리·방향을 본다.
ROS와 무관한 read-only 서버다. 현재는 complete/48장이며41후보/7검토의 원래 pilot
결과를 표시한다. 수정 후 개별 재시험은 CURRENT의 별도 경로를 따른다.
ETA는 캡처 표본 전에는 의미가 없다. 오래된 capture1 status는 사용하지 않는다.

**본 수집 전 필요한 수정·연결 확인 후** v15 회피 대표8장 pilot만 실행하는 예시:

```bash
ajou
ROOT="$HOME/morai_cap/base5_v2_261006"
python3 dataset/morai_od/scripts/select_base5_avoidance_pilot.py \
  --manifest "$ROOT/scenarios_10k_v15/manifest.json" \
  --output "$ROOT/avoidance_selection_new.json"
INDICES="$(python3 -c 'import json,os; p=os.path.expanduser("~/morai_cap/base5_v2_261006/avoidance_selection_new.json"); print(",".join(map(str,json.load(open(p))["indices"])))')"
python3 dataset/morai_od/scripts/run_base5_capture.py \
  --manifest "$ROOT/scenarios_10k_v15/manifest.json" \
  --output "$ROOT/avoidance_pilot_new" --indices "$INDICES" --limit 8
python3 dataset/morai_od/scripts/audit_base5_avoidance_capture.py \
  --capture-root "$ROOT/avoidance_pilot_new" \
  --output "$ROOT/avoidance_pilot_new/geometry_audit.json"
```

출력 경로는 새 이름을 사용한다. selector는 기존 파일을 덮어쓰지 않는다. 회피 장면은
지면/collider live evidence가 없으면 review로 남고 학습 변환에 자동 포함되지 않는다.
`prepare_base5_avoidance_gap_probe.py`는 누락 scene의 객체를 유지한 채 Ego만 외측으로
이동하는 대조 시험용이며 본 training deck을 바꾸거나 초근접 quota를 자동 완화하지 않는다.
현재 v15 캡처/대조 결과와 남은 geometry 보정은 CURRENT를 따른다.

MR 경로가 다르면 `--scenario-dir`과 `--sensor-dir`를 지정한다.
런타임 scene/EgoNetwork는 `zz_b5_<session>_...`로 한 장씩 설치·회수한다.
기존 사용자 시나리오를 지우거나10K 전체를 MR GUI 폴더에 넣지 않는다.
통신 stale/pose 미확인/서비스 timeout이면 즉시 중단한다. Ctrl+C 또는 output의
`STOP` 파일로 종료하며 ledger/raw는 보존한다. 재시작은 같은 manifest/hash로만 한다.

이번 검증에서는 SceneLoad 뒤 모든 rosbridge client가 끊겼다. network=false도 실패해
현재 runner는 network=true를 사용한다. 모델/GPU 문제나 JSON 누락으로 단정하지 않는다.
Ego/Object 신규 메시지와3개 명령 subscriber가 복구되기 전에는 로드하지 않는다.
전체10K 수집용 `--approve-bulk-capture`는 아직 사용하지 않는다.

캡처 통과 후 변환 예시(본 학습 실행 명령이 아님):

```bash
ajou
python3 dataset/morai_od/scripts/build_base5_bevfusion_infos.py \
  --ledger "$HOME/morai_cap/base5_v2_261006/pilot_reaudit_v2/ledger.jsonl" \
  --output artifacts/datasets/base5_v2_live_pilot \
  --sensor-config configs/sensors/Sensor_competition_local.json \
  --policy lidar1_camera0
```

이 policy는 비교 후보다. `lidar5`, `lidar1`도 별도 output으로 비교한다. Review scene과
관측 불가능한 primary는 자동 background로 변환하지 않는다. Raw는 수정하지 않는다.

2026-10-05. JSON 정적 검증/설치 및 실제12-scene 캡처 연결 통과.
결과와 제한은 [MR asset smoke](261005_base5_mr_asset_review_smoke.md)를 따른다.

## 파일과 용도

### CollisionData 연결 확인용 별도 프로필 — PREPARED

`configs/network/ego_network_mr_base5_savesensordata_collision_local.json`은 기존6개
연결에 ROS `/CollisionData`만 추가한7-interface 프로필이다. MR Network 폴더에
같은 이름으로 설치했다. 후속 대조 시험에서 실제 ROS CollisionData 연속 수신을 확인했다.

MR Scenario 폴더에는 다음2개만 설치했다. 둘 다 정지 배치 대조 장면이며 의도적
충돌 시험이나 물리 collider 안전성 인증 장면이 아니다.

- `1_b5_collision_check_barrier_baseline.json`
- `2_b5_collision_check_truck_baseline.json`

각각 `EgoNetwork/<scenario_name>_MN.json`도 새7-interface 프로필로 설치했다.
GUI에서 첫 시나리오를 로드하고 새 Network를 로드·연결한 뒤 수신을 재확인한다.
토픽이 없거나 메시지가 없다는 이유만으로 비충돌을 판정하지 않는다.
후속 runner는 새7-interface 프로필을 sidecar와 provenance에 사용하고 CollisionData를
기록한다. 양쪽 배열에서 Ego UID0 외 접촉 상대를 검사한다. 연결 stale이면 중단하고,
로드 이후 접촉 이력이 있으면 review로 남긴다. Capture pause 동안2초 초과 관측 공백도
자동 무충돌로 승인하지 않는다.

2026-10-06 정지 배치 대조 evidence:
`/home/luv/morai_cap/base5_v2_261006/collision_load_probe_023706/`.
시나리오4개, 실행 probe, summary/전체 telemetry JSON, 경량 rosbag, MR Player.log를 보존했다.
기존 바리케이드 baseline → Ego만 객체 쪽으로0.6m 이동 →1.1m 이동 →baseline 복원 순서다.
각 장면5초 관측에서 baseline과 복원은 대상UID8500이 모든 Object 메시지에 존재했다.
두 inward 배치에서는 대상이 전부 누락됐으며 각각 `cannot be placed here`가 로그에 남았다.
네 장면 모두 CollisionData는 `collision_object=[Ego UID0]`, `collision_objecta=[]`였다.
따라서 이 ROS 프로필에서 Ego만 있는 메시지는 실재하는 충돌 상대의 증거가 아니며,
배열 비어 있지 않음만으로 collision을 판단하면 안 된다. 반대로 상대가 없다는 것만으로
배치 안전성을 통과시키면 로드 거부로 사라진 객체를 정상 배경으로 잘못 수집한다.
대상UID 존재 + annotation 존재 + 신규 CollisionData 관측을 별도 gate로 유지해야 한다.
이 실험은 정지 로드 거부를 확인했다. 실제 주행 접촉 positive control은 아래 후속을 따른다.

### 1km/h 접근·정지 후 충돌 대조 — EVIDENCE

후속 raw: `/home/luv/morai_cap/base5_v2_261006/collision_contact_probe_024422/`.
앞선 정지 세분화: `collision_contact_probe_024344/` (정지4조건 완료 뒤 probe의
scenario key 오류로 이동 시험 전 중단·baseline 복원; 완료 주행 근거로 사용하지 않는다).
정지 Ego를 baseline보다 객체 쪽으로0.1m/0.2m 이동하면 대상은 존재했고 상대 충돌은
없었다.0.3m 이동은 대상이 누락됐다. 주행 전6m 뒤쪽 배치에서도 대상은 존재했고 충돌은 없었다.
외부 제어/Drive를 확인하고 `CtrlCmd(longlCmdType=2, velocity=1)`로 두 번 직진했다.
velocity 명령의 단위는 km/h다. 각각 CollisionData에 `[0,8500]`이8/7메시지 나타났고,
충돌 감지 즉시 brake 후5초 관측에서는251/251메시지가 Ego만 포함했다.
대상은 주행과 정지 후 모든 Object 메시지에 계속 존재했다. 새로운 header의 연속 수신이므로
latched 마지막 메시지나 topic 미연결이 아니다. 최종 baseline 복원·임시 파일 회수 완료.

충돌 이후 정지에서 상대가 사라진 것은 확인했으나, solver가 접촉을 분리했을 가능성도 있어
"정적 collider 충돌은 발행하지 않는다" 또는 "1km/h 임계값에서만 발행한다"는 결론은 아니다.
로드 시 객체 존재, annotation 존재와 더불어 **로드 이후 캡처까지 non-Ego 충돌 이력**을
봐야 한다. 캡처 순간 한 메시지만 읽어 충돌이 없다고 판정하지 않는다.
raw summary의 `speed_kph_min/max` 키는 Ego 메시지의 수치에 단위를 잘못 명명한 probe
이력이다. 이를 실제 km/h 근거로 쓰지 않으며 이동량/시간 또는 확인된 단위로 재계산한다.
이번 정지/이동 시험은 하나의 concrete-barrier asset 근거이며 전 asset 안전성 인증은 아니다.

- 현재 Git 원본: `configs/network/ego_network_mr_base5_savesensordata_collision_local.json`
- MR 로드 위치: `$HOME/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Network/25.S4.MolitComp03/ego_network_mr_base5_savesensordata_collision_local.json`
- MR 개인 PC의 ScenarioLoad + 정지 Ego + SaveSensorData/Instance annotation 검증용이다.
  대회 LAN용 또는 full-stack 주행용 Network가 아니다.
- 초기 `ego_network_mr_base5_savesensordata_local.json`은 검증된6-interface 기반이다.
  현재 collision 프로필은 여기에 실제 수신을 검증한 ROS CollisionData1개를 추가했다.

| 연결 | 목적 |
|---|---|
| /Ego_topic | Ego pose와 정지/드리프트 확인 |
| /Object_topic | actor UID/상태 대조, 캡처 annotation과 GT 확인 |
| /ScenarioLoad | 에이전트의 소량 시나리오 로드 |
| /ctrl_cmd | 정지/제어 요청 |
| /Service_MoraiEventCmd | 제어 모드·기어 등 runner의 기존 서비스 |
| /SaveSensorData | 동기 센서 파일 캡처 요청 |
| /CollisionData | 로드 이후 캡처까지의 물리 접촉 이력 |

Ego UNIQUEID=0만 포함한다. Simulator UNIQUEID=-100, SyncMode 서비스, /tf,
신호 상태, Competition/기타 상태 연결은 포함하지 않는다. Ego/Object ROS 발행 설정은
기존50Hz, rosbridge는 `ws://127.0.0.1:9090`이다. JSON의 비활성 UDP/기타 config 블록은
MORAI schema 보존용이며 이 프로필이 UDP Ego를 추가 연결한다는 뜻이 아니다.

## 현재 설정 비교와 연결 실패 해석

확인 당시 MR `NetworkInfo_2023_Hyundai_Ioniq5.json`에는 Ego15개 interface와 Simulator
항목이 있었고, SaveSensorData도 이미 정의되어 있었다. 따라서 앞선 subscriber0 결과는
필드 누락의 증거가 아니라 실제 subscription 미연결의 증거다. 새 파일명만으로 이
문제가 해결됐다고 주장하지 않는다. GUI 로드/연결 후 다시 검사해야 한다.

## 터미널 실행 순서 — SaveSensorData 수집 전용

이 절은 Base5 동기 파일 캡처용이다. 전체 UDP 속도 시험이나 Base3/Base4 연속
rosbag 수집 명령과 섞지 않는다. 각 터미널은 `ajou`로 MR workspace를 준비한다.

### 터미널 1 — 필수 ROS master

```bash
ajou
rc
```

### 터미널 2 — 필수 ROS bridge

```bash
ajou
rosparam set /use_sim_time false
roslaunch rosbridge_server rosbridge_websocket.launch
```

MR에서는 `ego_network_mr_base5_savesensordata_collision_local.json`을 로드·연결한다.
센서 설정은 `configs/sensors/Sensor_competition_local.json`을 기준으로 유지하고,
Capture Mode와 LiDAR Instance annotation 저장을 활성화한다.
Ego/Object, 제어, ScenarioLoad, SaveSensorData는 위 Network의 ROS 인터페이스를 사용한다.

### 터미널 3 — 실시간 센서 토픽도 필요할 때만 추가

```bash
ajou
roslaunch sensor_interface morai_udp_interface.launch \
  sensor_config:="$AJOUNICE_ROOT/configs/sensors/Sensor_competition_local.json" \
  enable_ego_network:=false \
  enable_control:=false
```

SaveSensorData는 MR이 센서 파일과 annotation을 직접 저장한다. 이 UDP receiver가
받은 ROS 센서 토픽을 저장하는 것이 아니므로, **동기 파일 캡처 자체에는 터미널 3이
필수가 아니다.** 다만 sensor topic 검사, rosbag 병행, timing sidecar가 필요한 runner
설정에는 센서 UDP 수신이 필요하다. 센서 수신 없이 실행할 때는 해당 runner의
`--no-timing-sidecar` 등 요구사항을 확인하고, annotation completeness gate는 유지한다.
Capture Mode에서는 실시간 스트림이 중단될 수 있으므로 토픽 수신을 자동 보장하지 않는다.

### 연결 후 — 한 장 검증을 통과한 다음 수집

Ego/Object의 실제 신규 메시지, ScenarioLoad/SaveSensorData 연결을 확인한 뒤
한 장을 캡처한다. Camera4/LiDAR/Instance annotation과 대상 UID가 저장된 경우에만
준비한 manifest로 확대한다. 이 문서의 연결 완료가 10K 본 수집 승인을 뜻하지 않는다.

### 혼동 금지

- `udp enable_sensors:=false enable_lidar:=false`는 **Ego/Object·제어만 UDP**로
  실행한다는 뜻이다. ROS 수신 모드로 전환하는 옵션이 아니다.
- 위 명령과 sensor-only UDP를 함께 켜면 전체 UDP를 두 프로세스 역할로 나눈 것이다.
  Base5의 ROS Ego/Object 연결과 동시에 켜지 않는다.
- 전체 UDP 보행자 속도 시험은 별도 진단이다. 그 시험이 Base5 수집 Network를
  전체 UDP로 변경했다는 뜻이 아니다.
- Base5 학습 BBox GT는 동기 LiDAR Instance annotation이다. 보조 `/Object_topic`
  velocity는 별도 평가·분석용이며 single-frame OD의 속도 학습 label로 사용하지 않는다.

## 로드 후 검증

1. 기존 rosbridge와 ROS master는 유지한다. 같은 Ego/Object를 UDP와 ROS 양쪽에서
   중복 발행하지 않는다. UDP를 유지한다면 sensor-only로 둔다.
2. MR Network GUI에서 위 새 파일을 로드하고 연결한다. Capture Mode와 LiDAR Instance
   annotation 출력도 켜 둔다. Network JSON은 센서 annotation 옵션을 설정하지 않는다.
3. Ego/Object의 신규 메시지, ScenarioLoad subscriber, SaveSensorData subscriber를
   확인한다. ROS capture publisher를 임시 생성해야 SaveSensorData topic이 보일 수 있다.
4. 연결 실패 시 중단한다. 연결 통과 후 단일 scene/capture에서 Camera4/LiDAR/Instance
   annotation/UID가 저장되는지 검증한다. 이후에만 소량 deck을 진행한다.

센서 JSON, production alias, 초기 위치/heading 및 기존 Network 파일은 변경하지 않았다.
ScenarioLoad 시 EgoNetwork sidecar가 Network를 다시 바꿀 수 있으므로 신규 캡처 시나리오
설치에는 이 파일을 `--network-template`로 사용한다. 단순 GUI 연결 검증을 위해 모든
기존 시나리오 sidecar를 덮어쓰지 않는다.
## v17 검증 주의 — 본 수집 보류

2026-10-06: 최신 후보10K는 offline 검사를 통과했지만 동적 timing3건은 아직 미해결이다.
pause 재개/선택적 actor-only load를 시험하다 서비스 timeout·연결 freshness 실패로 중단했다.
이 두 방식은 기본 runner에 적용하지 않았다. MR 재연결 후 정상 one-load 소량 검증부터
재확인한다. 지금 본 수집 명령을 실행하지 않는다. 상세 상태·증거는
[BASE5_V2_CURRENT](BASE5_V2_CURRENT.md)의 v17 항목을 따른다.
