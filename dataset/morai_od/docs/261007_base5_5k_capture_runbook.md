# Base5 Sunny13시 직접 5K — 현재 운영 런북

2026-10-08 / v117 endpoint-linear + v115 static advisory.
Base5-only single-frame OD 수집이다. E25 추론·temporal 학습·rosbag 녹화 명령이 아니다.
**본5K는 사용자 승인 후 시작한다.** 아래 명령은 준비된 새manifest/output용이며 이번 검증에서 실행하지 않았다.

## 실행 계약

- 새5K offline 생성 완료: 정적4846/실제 이동154,122종 primary 최소4회.
- 이동 외관은 신호등 사거리 두 횡단보도에 한정한다. 범용/정적 미션은 같은 자산을 정지 배치한다.
- 수동 저장 높이를 초기 후보로 검증한 endpoint lookup의 domain 안에서 두 waypoint를 생성한다.
  중간 Z는 두 끝점의 선형 보간이며 자동 지면 추종이 아니다.
  미측정 구간은 같은 셀의 초기 생성 재시도이며 경로를 무제한 flat 외삽하지 않는다.
- all-editor mover만 terrain/static-reference 준비를 생략한다. 원본 capture는 계속 검사한다.
- 정적/검증된 이동 경로 모두 plane 부족·작은 높이 gap은 warning이다.
  신뢰 plane ±40cm 초과·심한 Z step·CollisionData 접촉·정지 실패/통신 이상은 제외/중단 정책을 유지한다.
- fresh transport의 all-static actor 누락은 원본 annotation으로 review하며 같은-quota 재배치/부족분으로 처리한다.
  이동 primary 또는 stale topic은 이 예외로 넘어가지 않는다.
- 원본 point/GT·실패 근거를 보존한다. 배치 수는 실제 visible-positive 수가 아니다.
- 이동 primary의 걷기/달리기·사람/희귀 그룹별 양방향 차이≤1, 재시도에서도 방향을 고정한다.
  [CURRENT](BASE5_V2_CURRENT.md)와 [근거/한계](261008_base5_intersection_ground_evidence.md)를 먼저 확인한다.
- 구 v113/v114/v115/v116 pilot output을 현재 코드의 새 본수집에 resume하지 않는다.
  수집 시작 후 코드·config·manifest·scenario·lookup/source를 바꾸지 않는다.

v117 대표18셀18승인/21capture·184.685s, partial/완료resume와CPU변환 PASS다.
정적근접누락3review와일시적service응답오류2회는보존·복구했다. 세부사항은 EVIDENCE를 따른다.
새두점경로112개승인용검증은높이/운영scope이며 전체5K성공률 인증이 아니다.
전체소요는현재근거를조건부로합치면약10시간안팎이지만보장하지않고누적rate로갱신한다.

## 터미널 1 — master

```bash
ajou
rc
```

기존 master/bridge가 살아 있으면 중복 실행하지 않는다. 터미널들이 같은 ROS_MASTER_URI를 사용해야 한다.

## 터미널 2 — guarded bridge

```bash
ajou
python3 dataset/morai_od/scripts/run_base5_rosbridge.py
```

MR Capture Mode를 켜고 대회 센서 설정·Sunny13시를 UI에서 확인한다.
Ego Network Load→Connect:

```text
$HOME/AjouNice2026/configs/network/ego_network_mr_base5_savesensordata_collision_local.json
```

Ego/Object/CollisionData·ScenarioLoad·SaveSensorData·제어 서비스의7-interface profile이다.
**UDP receiver, sensing, localization, BEVFusion, planning, control을 별도로 실행하지 않는다.**
수집기가 Ego 정지/Park와 소량 임시 scenario 설치·회수를 맡는다.
5K 전체파일을 MR GUI 폴더로 복사하지 않는다. 같은 master에서 live 수집기를 동시에 두 개 돌리지 않는다.

## 터미널 3 — 승인 후 본수집 / 새 output

```bash
ajou
python3 -u dataset/morai_od/scripts/run_base5_batch_campaign.py \
  --manifest "$HOME/morai_cap/base5_v2_261008/endpoint_linear_final_5k_v117/scenarios_5k/manifest.json" \
  --output "$HOME/morai_cap/base5_v2_261008/collection_5k_endpoint_linear_v117" \
  --limit 5000 \
  --chunk-size 4 \
  --approve-bulk-capture \
  --resume
```

- limit: 이번 campaign의 설계 셀 수. 이manifest는 이동/정적이 섞여 있으므로 limit4846으로 정적만 수집되지 않는다.
- chunk-size: 공유 ROS child당 scene 수. 네트워크는 각scene reload하며 boundary/final Park를 확인한다.
- approve-bulk-capture: 64셀 초과를 승인한 실행용. 작은 smoke에서는 사용하지 않는다.
- resume: 기존 안전한 성공분을 검증 후 재사용. 첫 실행에도 가능하다. limit/chunk/계약을 바꿔 resume하지 않는다.
- 통신/정지/Park 불확실의 halted 상태는 자동 강제resume 대상이 아니다. 근거를 확인하고 안전 복구/새output을 결정한다.

## 터미널 4 — 대시보드

```bash
ajou
python3 dataset/morai_od/scripts/base5_capture_dashboard.py \
  --capture-root "$HOME/morai_cap/base5_v2_261008/collection_5k_endpoint_linear_v117" \
  --port 8075
```

브라우저 http://127.0.0.1:8075.
승인/부족분·누적 시간·승인 rate·디스크·현재 child/recovery를 확인한다.
finished는 승인 수가 아니다. output 생성 전에는 not_started다.
waiting_stationary_ego는 정상 정지 검사 단계다. 준비 중 ETA0만으로 멈춤을 단정하지 않는다.

## 일시정지·재개

현재 chunk와 안전 종료를 마친 뒤 pause:

```bash
ajou
touch "$HOME/morai_cap/base5_v2_261008/collection_5k_endpoint_linear_v117/PAUSE"
```

paused 확인 후 파일을 제거하고 터미널3의 **동일한 명령**으로 재개한다.

```bash
ajou
rm "$HOME/morai_cap/base5_v2_261008/collection_5k_endpoint_linear_v117/PAUSE"
```

급히 중단하려면 부모터미널 Ctrl-C/STOP 경로를 사용한다. 종료 Park 확인 전 다른 수집기를 켜지 않는다.
부족분은 deficit_manifest/ledger에 남는다. 끝난 뒤 부족 셀의 같은 자산·family/phase·거리 quota로
별도 bounded 보충을 설계한다. 잘 나온 장면만 골라 전체quota 완료로 표시하지 않는다.

## 최종 감사·CPU 변환

```bash
ajou
python3 dataset/morai_od/scripts/report_base5_final_coverage.py \
  --manifest "$HOME/morai_cap/base5_v2_261008/endpoint_linear_final_5k_v117/scenarios_5k/manifest.json" \
  --ledger "$HOME/morai_cap/base5_v2_261008/collection_5k_endpoint_linear_v117/selected_ledger.jsonl" \
  --output "$HOME/morai_cap/base5_v2_261008/collection_5k_endpoint_linear_v117/effective_coverage.json" \
  --require-effective-complete
```

전체승인 장수와122종 실제 visible-primary 최소4회는 별개다. 최소4회는 학습 충분성 보장이 아니다.
GT는 raw Instance component 유지, LiDAR≥1점 또는 실제 camera Instance visibility로 positive를 정한다.
안 보이는 객체를 정답으로 넣지 않는다. LiDAR/Ego ROI는[-100,-30,-3,100,30,2]다.
accepted·review없음·calibration아님만 전달한다.

```bash
ajou
python3 dataset/morai_od/scripts/build_base5_bevfusion_infos.py \
  --ledger "$HOME/morai_cap/base5_v2_261008/collection_5k_endpoint_linear_v117/selected_ledger.jsonl" \
  --output "$HOME/morai_cap/base5_v2_261008/base5_5k_infos_endpoint_linear_v117" \
  --sensor-config configs/sensors/Sensor_competition_local.json
```

다른환경은 별도5K campaign/output이다. 이번현재환경은Sunny13시이며 과거11시표기를 재사용하지 않는다.
새manifest 재생성은 freeze_base5_intersection_release.py --endpoint-linear의 새output으로만 수행한다.
구v113 명령/4171+829/45.7h등은 Git 이력·CURRENT의 historical evidence에 보존하며 현재실행값이 아니다.
