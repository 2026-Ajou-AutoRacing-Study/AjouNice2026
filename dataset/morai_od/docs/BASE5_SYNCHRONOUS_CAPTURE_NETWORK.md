# Base5 MR 동기 캡처 Network — PREPARED

## 현재 진입점 — Sunny11시 직접5K / 2026-10-07

[CURRENT](BASE5_V2_CURRENT.md)와 [직접5K 실행·정지·재개](261007_base5_5k_capture_runbook.md)를
우선한다. 아래 v43/v40/v39 및 모든 구10K 명령은 당시 이력이다.
guarded bridge·7-interface·매scene network reload를 유지한다.
이번 수집 후보는 v105 직접5K이며 사용자 최종 확인 전 본수집·학습을 시작하지 않는다.
현재 bridge가 연결돼 있으면 아래 과거 재시작 지시 때문에 임의로 끊지 않는다.
통신/Park 미확인은 halt다. 확인된 접촉은 실패/부족분으로 분리하며,
안전 Park·새 장면 확인 후에만 진행한다. 구 halted session은 강제 resume하지 않는다.

## v40 당시 — guard 적용 bridge 재시작 필요

[서비스 응답 혼선 재현·재연결 명령](261006_base5_eventcmd_rosbridge_v40.md)이우선한다.
새수집은28/32quota까지통과후최종Park미확인으로중단했다.
7-interface network는그대로유지한다.기존bridge종료후`run_base5_rosbridge.py`실행,
MR Disconnect/Connect 후소량live검증이필요하다.설치ROS파일/production alias는수정하지않았다.

## v39 당시 — 재연결 이후 안전 batch·quota pilot

[최신 근거·실행·모니터링](261006_base5_safe_batch_quota_v39.md)을 우선한다.
network 재사용은 계속 금지하며7-interface/scene별network reload를 유지한다.
새6장 pause/resume와chunk4 첫12셀 보완/완료resume를 검증했다.
boundary Park 간헐timeout은 남았으므로 실패 세션을 정상finalPark만으로 재개하지 않는다.
`run_base5_batch_campaign.py`는max64셀 pilot이고 본10K명령이 아니다.
후속검증합계20/32quota에서 최종Park미확인으로중단했다. 재연결/정지/Park를다시확인하고
실패출력이아닌새output에서남은12셀을검증해야한다. 현재 무인본수집 시작금지다.

## v38 — 네트워크 재사용 금지 / 재연결 필요

[최신실험·복구](261006_base5_ros_batch_network_evidence_v38.md)가 아래 이력보다 우선한다.
ROS 프로세스 공유는 소량 통과했지만 network flag=false는 두 번째 장면에서 전체client가 끊겼다.
--batch-reuse-network 금지, scene별 network reload 유지. 종료Park 미확인이므로 새 로드도 중단했다.
원래 configs/network/ego_network_mr_base5_savesensordata_collision_local.json을 MR에서 Load/Connect하고,
Ego 정지/Park·필수topic·사전Park 응답을 확인한 뒤 새output의 소량검증만 재개한다.
실패 network_reuse_v38 output은 resume하거나 계약을 수정하지 않는다.
--shared-ros-batch는 max16 실험후보이며 새 guard/stop-resume live는 아직 미검증, 본수집용 기본이 아니다.

## v37 — 조건부 준비 대기 / 구세션 보호

최신 비용/실행 계약은 [v37 계측·소량A/B](261006_base5_capture_stage_optimization_v37.md)를 따른다.
실제 publisher 연결과 신선한 Ego/Object/CollisionData,0.3초 정지 증거가 확보되면
preflight 고정 대기를 단축한다. 증거미확보3초 timeout은 로드 전 중단이다.
아래 과거 세션 명령은 당시 검증 이력이다. 코드/안전 hash가 바뀐 v36 및 v37 baseline을
현재 코드로 resume하거나 계약파일을 수동 수정하지 않는다. raw/ledger는 보존하고 읽기전용 report를 사용한다.
현재 코드와 동일 hash의 새 세션만 기존 resume/STOP/quota 규칙으로 재개한다.

## v31 — quality campaign resume/quota 보충

최신 진행 상태는 [CURRENT](BASE5_V2_CURRENT.md)와 [EVIDENCE](261006_base5_campaign_recovery_evidence.md)를 읽는다.
아래는 소량 검증용이다. 본10K 시작 명령이 아니다. 기존 Capture Mode/CollisionData 연결을 유지한다.

```bash
ajou
ROOT="$HOME/morai_cap/base5_v2_261006"
python3 dataset/morai_od/scripts/run_base5_capture_campaign.py \
  --manifest "$ROOT/appearance_integration_deck_v28/manifest.json" \
  --indices 3,6 --output "$ROOT/campaign_final_v31" --resume
```

이미 완료한 위 output은 재촬영하지 않고 무결성 확인만 한다. 새 검증은 다른 output으로 실행한다.
`--stop-after-cells 1`은 안전 종료된 첫 셀 후 멈추는 resume 검증용 옵션이다.
campaign·개별 child lock이 중복 실행을 막는다. 계약 변경/unsafe child는 새 output 및 안전 확인이 필요하다.

```bash
ajou
python3 dataset/morai_od/scripts/base5_capture_dashboard.py \
  --capture-root "$HOME/morai_cap/base5_v2_261006/campaign_final_v31" --port 8075
```

브라우저 `http://127.0.0.1:8075`. child 대기 중에도 상태가 갱신된다.
안전 중단은 해당 campaign 폴더에 `STOP` 파일을 만든다. 재개 전 STOP을 제거하고 `--resume`을 사용한다.
강제 kill/미확인 Park은 자동 재개하지 않는다. 먼저 실차 상태/통신/브레이크를 확인한다.

재시도 소진 시 `quota.json`의 부족 셀과 `deficit_manifest.json`이 생성된다.
보충 필요성을 확인한 뒤 이 manifest로 **새 output**의 bounded campaign을 시작한다.
같은 output의 resume은 소진된 실패를 무한 반복하지 않는다.
변환/학습에는 `selected_ledger.jsonl`만 전달한다. 빈 지면/실패/review raw를 임의로 합치지 않는다.
64셀 초과의 명시적 bulk 플래그는 준비돼 있지만 현재는 본 수집을 시작하지 않는다.

2026-10-06: [v2 CURRENT](BASE5_V2_CURRENT.md)의 v30 입력Z 대조가 최신이다.
v29의Park2회timeout 중단 후 v30 사전Park 정상 확인으로 소량실험을 재개했고,
3세션 종료Park을 확인했다. 간헐첫timeout/재시도 성공은 남아 있으며 장시간 인증은 아니다.
추가 세션도 사전통신/종료Park gate를 유지한다. v29 안전실패 기록은 수정하지 않았다.
아래 v23 분산검증 이력과 맨 아래 yaw 검사 명령을 구분한다.
**본 수집은 보류다. 아래 v16 명령은 검증된 이전 smoke 세션 예시이며10K 시작 승인이 아니다.**

v18은 같은 one-load/seven-interface network를 사용한다. pause·선택적 actor-only load를
도입하지 않는다. 새 AI presence 계약으로 이전 output resume은 거부될 수 있으므로 새
output을 사용한다. Dashboard는 새 세션 output을 `--capture-root`로 지정하면 된다.
아래12-scene 성공은 이전 asset-review 이력이며 새10K 배치 성공을 뜻하지 않는다.

## v23 — 준비·모니터링·최종 완료 resume

새 분산48개를 준비할 때는 빈output을 사용한다. 아래 준비는 MR을 로드하지 않으며
10K 생성도 아니다. 원본spec을 참고하되 일방향 appearance profile로 다시 배치한다.

```bash
ajou
ROOT="$HOME/morai_cap/base5_v2_261006"
python3 dataset/morai_od/scripts/prepare_base5_readiness_pilot.py \
  --manifest "$ROOT/scenarios_10k_v18d/manifest.json" \
  --output "$ROOT/readiness_next_pilot"
```

실제 캡처는 연결/CollisionData·Capture Mode 확인 후,사용자 승인된 새output에서만 한다.
현재 완료한 세션의 **read-only resume** 확인은 다음이다. 기존1개를 재촬영하지 않는다.

```bash
ajou
ROOT="$HOME/morai_cap/base5_v2_261006"
python3 dataset/morai_od/scripts/run_base5_capture.py \
  --manifest "$ROOT/readiness_retry_pilot_v22/manifest.json" \
  --output "$ROOT/all_actor_capture_v23" --indices 1 --limit 1
```

최신 session schema5는 모든 moving actor의 캡처 직전 움직임과 전후 endpoint를
검사한다. 이전schema3/4 세션은 현runner로 resume하지 않는다. 계약SHA/schema를
수동 변경해 우회하지 않는다. 아래v21/v16 명령은 당시 실행 이력이다.

모니터링은 ROS 없이 가능하다. 이미8075가 떠 있으면 두 번째 서버를 띄우지 않는다.

```bash
ajou
python3 dataset/morai_od/scripts/base5_capture_dashboard.py \
  --capture-root "$HOME/morai_cap/base5_v2_261006/readiness_capture_v22" --port 8075
```

이 화면은 최초48회의46후보/1검토/1실패를 숨기지 않는다. 재배치 포함 최종48조건은
`$ROOT/readiness_selected_report_v23.json` 및 `readiness_selected_v23.provenance.json`을 본다.
선택본 PKL은 `artifacts/datasets/base5_readiness_selected_v23/`다. 데이터/산출물은 Git 제외다.
GT 기본은 lidar1_camera0,scale1·geometric Instance·component 보존·7D/no velocity다.
원거리 미관측/이동 미확인/끝점 도달을 승인하거나 empty negative로 바꾸지 않는다.
종료Park이 확인된 세션만 `select_base5_ready_captures.py`로 통합한다.

## v21 일방향 소량 재검증 명령 — 과거 실행 이력

다음은 당시 **이미 완료한4장을 read-only resume 확인**했던 명령이다.
현재schema5 runner로는 계약 불일치로 거부하므로 재실행하지 않는다.

```bash
ajou
ROOT="$HOME/morai_cap/base5_v2_261006"
python3 dataset/morai_od/scripts/run_base5_capture.py \
  --manifest "$ROOT/one_way_pilot_v21c/manifest.json" \
  --output "$ROOT/one_way_capture_v21c" --indices 12,14,17,18 --limit 4
```

새 offline19개 pilot을 준비하려면 반드시 새output을 지정한다(실제로드 없음):

```bash
ajou
ROOT="$HOME/morai_cap/base5_v2_261006"
python3 dataset/morai_od/scripts/prepare_base5_safety_pilot.py \
  --manifest "$ROOT/scenarios_10k_v18d/manifest.json" \
  --output "$ROOT/one_way_next_pilot" --motion-policy one-way-slow-v1 \
  --extra-runtime-coverage --placement-attempts 1024 --one-way-setup-budget-sec 6
```

기존source의quota/asset spec을 참고해 새 일방향 payload를 만들며 원본은 변경하지 않는다.
1024는offline 배치 탐색 한도이지 MR 로드 횟수가 아니다. actual 거리/충돌 gate는 유지한다.
`one_way_setup_budget_sec`는 속도 제한을 계산하는 명목여유로, 실제 캡처 대기시간이나
통신 timeout을6초로 바꾸는 파라미터가 아니다. 늦어서 이미 멈췄으면 motion check 실패다.

## v2 캡처·실시간 대시보드

터미널1/2는 아래 `ajou`, `rc`, rosbridge 순서를 따른다. SaveSensorData 수집 자체에는
UDP sensor receiver·localization·BEVFusion·planning·RViz가 필요하지 않다.
MR Network GUI에서 전용 JSON을 다시 연결하고 Capture Mode/Instance 저장을 확인한다.
현재 runner 기본은 `ego_network_mr_base5_savesensordata_collision_local.json`이다.
기존6-interface 프로필은 이력/일반 캡처용이며 새 안전 gate를 통과하지 못한다.

### v19 보충·정책 비교·dashboard

v19 live는 Park 응답 미확인으로 중단했다. v20 gear-only 종료는3회 첫 응답 통과했다.
다음 명령의 기본은 **준비만**이며
MR을 로드하지 않는다. 기존 `replenishment_v19`는 중단 증거이므로 덮어쓰거나 자동 재개하지 않는다.
MR 제어/기어를 확인한 뒤 별도 소량 live를 검증해야 한다. 10K 시작 명령이 아니다.

```bash
ajou
ROOT="$HOME/morai_cap/base5_v2_261006"
python3 dataset/morai_od/scripts/base5_replenishment.py \
  --source-manifest "$ROOT/scenarios_10k_v18d/manifest.json" \
  --source-ledger "$ROOT/shuttle_capture_v18d/ledger.jsonl" \
  --output "$ROOT/replenishment_next_pilot" --attempts 3
```

안전 확인 후 명시적으로 `--run`을 추가해야 bounded live가 시작된다. 준비만은 동작 검증이 아니다.
원본review는 삭제하지 않으며 안전종료까지 성공한 보충만 `selected_ledger.jsonl`에 기록한다.
child 종료Park 실패/통신 실패 시 즉시 중단하며 해당 output의 자동 재개를 거부한다.
`STOP` 파일은 다음 attempt 시작 전 검사한다. 현재 attempt의 cleanup을 생략하지 않는다.

현재 중단 결과를 확인하는 read-only dashboard:

```bash
ajou
python3 dataset/morai_od/scripts/base5_capture_dashboard.py \
  --capture-root "$HOME/morai_cap/base5_v2_261006/replenishment_v19" --port 8075
```

브라우저 `http://127.0.0.1:8075`. 개별 child 종료gate까지 보려면 capture-root를
`replenishment_v19/attempts/b5v2_00024_r01`로 바꾼다. `halted_exit_safety`는 정상완료가 아니다.

GT정책 비교는 ROS/MR 없이 수행한다:

```bash
ajou
ROOT="$HOME/morai_cap/base5_v2_261006"
python3 dataset/morai_od/scripts/report_base5_gt_policies.py --ledgers \
  "$ROOT/shuttle_capture_v18d/ledger.jsonl" \
  "$ROOT/shuttle_extra_capture_v18/ledger.jsonl" \
  "$ROOT/ai_presence_capture_v18/ledger.jsonl" \
  --output "$ROOT/gt_policy_comparison_v19.json"
```

### v16 수집·중단·재개 이력

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
실제 캡처 종료/중단 시에는 brake와 bounded Park 요청을 보낸다. **현재 v19부터**
Park 응답 미확인은 경고만이 아니라 종료실패 상태로 남으며 자동 후속로드를 중단한다.
요청당3초·2회 동안 제동을 유지하고 `shutdown_safety.json`에 결과를 기록한다.
v20 종료 요청은 option2/gear1이며 제어모드를 재설정하지 않는다. 로드 직후 mode/Drive
확인은 option3 그대로다. 완료된 unsafe output을 새 결과로 덮어쓰지 않는다.

정지 상태에서만 한 번 요청하는 독립 진단(캡처/로드 없음):

```bash
ajou
python3 dataset/morai_od/scripts/probe_base5_park.py --option 2 \
  --output "$HOME/morai_cap/base5_v2_261006/park_manual_check.json"
```

증거 파일이 이미 있으면 덮어쓰지 않는다. timeout이면 반복해서 쌓지 말고 MR 연결·제어를
확인한다. 이 명령으로 모든 런처의 안정성이 검증되는 것은 아니다.

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
## yaw별 범위 검사 재현 / v24

현재 네트워크·Capture Mode·Instance 설정은 위의 Base5 동기 캡처와 동일하다.
`prepare_base5_yaw_probe.py`는 기존 source manifest의 실제 도로 배치6개를 사용하므로
source가 없는 fresh clone에서 단독 실행되는 범용 생성 명령은 아니다.
NPC payload를 그대로 두고 Ego yaw만 바꾸며, MR 폴더에는 runner가 현재1장만 설치/정리한다.

```bash
ajou
python3 dataset/morai_od/scripts/prepare_base5_yaw_probe.py \
  --manifest /home/luv/morai_cap/base5_v2_261006/scenarios_10k_v18d/manifest.json \
  --output /home/luv/morai_cap/base5_v2_261006/yaw_probe_new

python3 dataset/morai_od/scripts/audit_base5_scenarios.py \
  --manifest /home/luv/morai_cap/base5_v2_261006/yaw_probe_new/manifest.json \
  --output /home/luv/morai_cap/base5_v2_261006/yaw_probe_new/offline_audit.json
```

ROI 경계 진단용이므로 offline ROI 오류2건을 의도적으로 포함한다.
일반 생성 audit의 합격과 구분하고 일반 학습 quota에 섞지 않는다.
`report_base5_yaw_probe.py`가 기대한2건 외에 오류가 있는지 명시한다.

v24 실행의 raw/ledger/metadata를 보존하고 재현에는 새 output을 사용한다.
당시 마지막 Park 응답을 확인하지 못해 중단했다. v25는 사전Park probe 후 아래2조건을
완료했다. 아래는 **완료 세션의 read-only resume 예시**이며 새 캡처 명령이 아니다.

```bash
ajou
python3 dataset/morai_od/scripts/run_base5_capture.py \
  --manifest /home/luv/morai_cap/base5_v2_261006/yaw_remaining_manifest_v24.json \
  --output /home/luv/morai_cap/base5_v2_261006/yaw_remaining_capture_v25 \
  --limit 2
```

집계는 read-only이며 MR 통신이 필요 없다. 여러 ledger를 입력하면 처음 확인된 유효
고정NPC 조건을 채택하고, 이동/미저장 시도도 제외 이유로 남긴다.
새 remaining ledger는 완료 후 추가한다. raw를 덮어쓰거나 실패를 삭제하지 않는다.

```bash
ajou
python3 dataset/morai_od/scripts/report_base5_yaw_probe.py \
  --manifest /home/luv/morai_cap/base5_v2_261006/yaw_probe_v24/manifest.json \
  --ledger /home/luv/morai_cap/base5_v2_261006/yaw_capture_v24/ledger.jsonl \
  --ledger /home/luv/morai_cap/base5_v2_261006/yaw_retry_capture_v24/ledger.jsonl \
  --ledger /home/luv/morai_cap/base5_v2_261006/yaw_remaining_capture_v25/ledger.jsonl \
  --readiness-ledger /home/luv/morai_cap/base5_v2_261006/readiness_selected_v23.jsonl \
  --output /home/luv/morai_cap/base5_v2_261006/yaw_report_v25.json
```

도로 의미의 전후/좌우는 road-frame metadata, 학습/추론 범위는 실제 LiDAR-frame으로 판단한다.
GT를 도로 방향으로 돌려 범위 안으로 취급하지 않는다. 검증 중 detector·planner·GPU 추론을 켜지 않는다.

## 동적 외관·뒤쪽 출발 진단 / v26

기존 네트워크·Capture Mode를 유지하며 full stack은 필요 없다. 같은 ordinary pedestrian의
완료 raw가 있어야 한다. 아래경로는 로컬 증거이며freshclone용 데이터 제공 명령은 아니다.
목표위치까지 움직인 뒤 저장하므로 저속화를 위해좁은cell 전체에 경로를 억지로 넣지 않는다.
target 도착확인/전체경로 충돌 proxy/실제CollisionData/endpoint/가시성 gate를 유지한다.

```bash
ajou
python3 dataset/morai_od/scripts/prepare_base5_motion_appearance_probe.py \
  --capture "$HOME/morai_cap/base5_v2_261006/readiness_capture_v22/captures/b5v2_00039" \
  --output "$HOME/morai_cap/base5_v2_261006/motion_probe_next"
```

위 명령은 새12개 scenario를 준비할 뿐 MR에 로드하지 않는다. 새output만 허용한다.
실제v26 캡처는 완료됐으며 아래는 완료12개를다시촬영하지 않는 read-only resume 확인이다.
실패2회 기록은ledger에보존돼있다. 완료한원본scenario나session계약을 바꿔재개하지 않는다.

```bash
ajou
python3 dataset/morai_od/scripts/run_base5_capture.py \
  --manifest "$HOME/morai_cap/base5_v2_261006/motion_probe_v26/manifest.json" \
  --output "$HOME/morai_cap/base5_v2_261006/motion_capture_v26" --limit 12
```

원본RGB crop·속도/도착/캡처시각의 read-only 보고서 생성은 새output을사용한다.
`appearance_contact_sheet.png`는보기용 확대이며 새로운센서해상도 데이터가 아니다.

```bash
ajou
python3 dataset/morai_od/scripts/report_base5_motion_appearance_probe.py \
  --capture-root "$HOME/morai_cap/base5_v2_261006/motion_capture_v26" \
  --output "$HOME/morai_cap/base5_v2_261006/motion_report_next"
```

normal Base5 generator의자연속도 기본전환은아직아니다. 현재walking/running tag는속도분기이고,
모든자산에서 explicit animation Walk/Run 전환을확인한것은아니다.

## 자산별 외관 비교 / v27

동적 z gate도 기본 적용한다. 갑작스러운 z변화/검증된 높이 reference 불일치는
합격 후보에서 제외하고 raw 증거는 보존한다. objectList 이동은 높이 reference가
없어도review이며, 안정적인 z만으로 공중 배치를 합격시키지 않는다. 경사면의 임의
고정 z reference로 대체하지 않는다. 현재 진단은 `motion_probe.height_calibration.static_z`
실측값을 사용하며, 일반 생성기에는 위치·자산별 지면 reference 검증이 추가로 필요하다.

기존 ROS 동기캡처 network/Capture Mode를 그대로 사용했다. 다른 UDP/full stack을
동시에 켜지 않는다. 대표13종52조건의 완료 기록과 높이 matched 재촬영 근거는
[EVIDENCE](261006_base5_asset_motion_appearance_evidence.md)와 CURRENT를 읽는다.
현재 preparer는 진단 전용이며 일반10K 수집 명령이 아니다.

새 진단 deck 준비는 MR에 로드하지 않으며 존재하지 않는 output만 허용한다.
reference는 로컬 raw이므로 fresh clone만으로 아래 경로가 생기지는 않는다.

```bash
ajou
python3 dataset/morai_od/scripts/prepare_base5_asset_motion_probe.py \
  --reference "$HOME/morai_cap/base5_v2_261006/readiness_capture_v22/captures/b5v2_00039" \
  --output "$HOME/morai_cap/base5_v2_261006/asset_motion_probe_next"
```

수집은 해당 manifest와 새 output을 `run_base5_capture.py --limit 52`에 전달한다.
이미 완료한 session에 다른 hash의 deck을 덮어써 resume하지 않는다.
높이 비교용 `--height-reference-report`는 실제 정지/이동 report에서 계산한 waypoint
z offset으로 입력 시나리오만 바꾸는 진단 옵션이다. 일반도로 지면 보정/GT 변환으로 쓰지 않는다.

최종 선택 보고서를 read-only로 다시 만들 때는 아래처럼 run 순서를 명시한다.
원래 실패와 교체된 raw도 보고서에 보존되며 원본 파일을 바꾸지 않는다.

```bash
ajou
python3 dataset/morai_od/scripts/report_base5_asset_motion_probe.py \
  --capture-root "$HOME/morai_cap/base5_v2_261006/asset_motion_capture_v27" \
  --additional-capture-root "$HOME/morai_cap/base5_v2_261006/asset_motion_capture_v27_corrected" \
  --additional-capture-root "$HOME/morai_cap/base5_v2_261006/asset_motion_capture_v27_height" \
  --output "$HOME/morai_cap/base5_v2_261006/asset_motion_report_next"
```

각 `*_comparison.png`는 정지2/이동2 원본 RGB crop 확대다. `*_lidar_xz.png`는
component 로컬 관측점 그림이며 mesh 재구성이나 detector 평가가 아니다.

## v28 — 일반 생성기 정책/근접·혼합 및 입력 Z 재촬영

네트워크는 위 MR seven-interface 설정 그대로다. 추가 topic/UDP는 필요 없다.
현재 기본은 `natural-target-v1`이며 오래된 one-way-slow deck을 새 기본 검증으로 사용하지 않는다.
본10K는 승인하지 않았다. 아래는 **소량 새 세션 예시**다. 이미 완료한 v28 output은 덮어쓰지 않는다.

```bash
ajou
python3 dataset/morai_od/scripts/prepare_base5_appearance_pilot.py \
  --output "$HOME/morai_cap/base5_appearance_next/deck"
python3 dataset/morai_od/scripts/audit_base5_scenarios.py \
  --manifest "$HOME/morai_cap/base5_appearance_next/deck/manifest.json" \
  --output "$HOME/morai_cap/base5_appearance_next/offline_audit.json"
python3 dataset/morai_od/scripts/run_base5_capture.py \
  --manifest "$HOME/morai_cap/base5_appearance_next/deck/manifest.json" \
  --output "$HOME/morai_cap/base5_appearance_next/capture" --limit 9
```

기존 dashboard는 새 capture output을 `--capture-root`로 지정한다.
통신/종료Park 실패 시 중단한다. 지면 미확인·below-ground·visibility 누락을 성공으로 취급하지 않는다.

확실한 above-ground만 입력 Z 수정 deck으로 분리할 수 있다. 다른 위치에 같은 offset을
복사하지 않는다. 보정 뒤 반드시 새 캡처+지면/Z/CollisionData 검사를 통과해야 한다.

```bash
ajou
python3 dataset/morai_od/scripts/prepare_base5_ground_retry.py \
  --manifest "$HOME/morai_cap/base5_appearance_next/deck/manifest.json" \
  --captures "$HOME/morai_cap/base5_appearance_next/capture" \
  --output "$HOME/morai_cap/base5_appearance_next/ground_retry_deck"
python3 dataset/morai_od/scripts/run_base5_capture.py \
  --manifest "$HOME/morai_cap/base5_appearance_next/ground_retry_deck/manifest.json" \
  --output "$HOME/morai_cap/base5_appearance_next/ground_retry_capture" --limit 9
```

retry deck count가 0이면 실행하지 않는다. 실행 limit는 실제 deck count에 맞춘다.
새 세션은 v6 안전 계약에 감사/runner 코드 hash도 기록한다. 검증 코드가 바뀌면 기존 세션의
resume을 거부한다. 기존 v28 촬영의 v5 계약을 수동 수정하지 말고 읽기 전용 report로 재검사한다.
이는 universal terrain solver가 아니며 가까운 sparse ground 때문에 review가 많을 수 있다.
원본 ledger/GT를 수정하지 않는 최종 코드 재검사:

```bash
ajou
python3 dataset/morai_od/scripts/report_base5_ground_integration.py \
  --capture-root "$HOME/morai_cap/base5_appearance_next/capture" \
  --output "$HOME/morai_cap/base5_appearance_next/ground_report.json"
```

결과/제한은 [v28 EVIDENCE](261006_base5_appearance_ground_integration_evidence.md)를 참고한다.

## v29 — 소량 자동 품질 보완 루프

지면 감사는 zero XYZ 점/Ego footprint를 제외한다. raw point/GT를 수정하지 않는다.
독립 지면은 캡처용 Ego를 route상12m 뒤로 옮긴 **calibration-only** 파일이며 학습에서 제외한다.
MR seven-interface network는 그대로다. 현재 서비스 문제가 있으므로 아래는 정상 재연결 후 예시다.

```bash
ajou
python3 dataset/morai_od/scripts/probe_base5_park.py \
  --output "$HOME/morai_cap/base5_quality_next_park.json"
```

Park확인실패시loop를실행하지않는다. 기존코드의간헐첫timeout과반복실패를구분한다.

```bash
ajou
python3 dataset/morai_od/scripts/run_base5_quality_loop.py \
  --manifest "$HOME/morai_cap/base5_v2_261006/appearance_integration_deck_v28/manifest.json" \
  --indices 3,6,8 \
  --output "$HOME/morai_cap/base5_quality_next"
```

최대4cell·cell당최대2retry다. 위indices는v29에서중단/미실행한조건이다.
sourcepath는로컬pilot예시이며10K수집명령이아니다.
STOP요청은새터미널에서 `touch "$HOME/morai_cap/base5_quality_next/STOP"`이다.
기존 dashboard에 `--capture-root "$HOME/morai_cap/base5_quality_next"`를지정한다.
campaign은기존output을덮어쓰지않으며,실패시child로그/shutdown/원본을보존한다.
각child의안전계약에코드hash가있어코드변경뒤기존childresume은거부된다.

안전하게끝난partial selection만변환한다. stage=halted라고실패raw를추가하지않는다.

```bash
ajou
python3 dataset/morai_od/scripts/build_base5_bevfusion_infos.py \
  --ledger "$HOME/morai_cap/base5_quality_next/selected_ledger.jsonl" \
  --output "$AJOUNICE_ROOT/artifacts/datasets/base5_quality_next_infos" \
  --sensor-config "$AJOUNICE_ROOT/configs/sensors/Sensor_competition_local.json" \
  --point-cloud-range -100 -30 -3 100 30 2
```

미실행 조건·실제 근거는 [v29 EVIDENCE](261006_base5_ground_quality_loop_evidence.md)를 읽는다.
