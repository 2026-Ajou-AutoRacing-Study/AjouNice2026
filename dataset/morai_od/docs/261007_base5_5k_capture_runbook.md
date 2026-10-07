# Base5 Sunny13시 직접 5K — 현재 운영 런북

2026-10-07. 과거 v45/10K 명령 대신 이 문서를 사용한다.
본수집은 사용자 승인 후 진행하며, 이미 진행 중인 수집은 기존 명령으로 이어간다.
E25 추론·temporal 학습·rosbag 녹화용 런북이 아니다.

2026-10-07 [환경 정정](261007_base5_sunny13_environment_correction.md): 실제 MR 기본/수집은
사용자 확인 Sunny 13시다. v108의 미시작 기록은 당시 사전 보고이며, 현재 수집 재시작 지시가 아니다.
**기존 수집/resume 명령·manifest·output은 그대로 둔다. 아래 새 경로는 신규 수집용이다.**

v113은 v112의122종/동적 미션 차량12셀을 유지하고 최종 객체 XY의 초기 높이를 보완한 새 버전이다.
생성기 코드가 바뀌었으므로 구 v105/v109 output에는 새 코드로 resume하지 않는다.
구 수집을 이어갈 때는 원래 commit `08bf6816`의 코드/manifest/output을 사용한다.
구 v111 재개는 원래 commit `b0997088`을 사용한다.
구 v112 재현은 원래 commit `c67f62c3`을 사용한다.
아래 명령은 새 v113 output 전용이며 대표 smoke 결과와 사용자 승인을 확인한 뒤 시작한다.
같은 위치 Dog2/정상 근접 보완과 혼합 raw 재검사는 통과했다. 새 frozen 대표4셀은3승인/1부족분이다.
별도 Ego drift halt/roadside animated calibration 부족도 남는다. 부족분을 숨기거나 전체 자산 live 인증으로 해석하지 않는다.

## 터미널 순서

각 터미널에서 먼저 `ajou`를 실행한다. 같은 master/bridge가 이미 살아 있으면
중복 실행하지 않는다. 저장소 경로가 다르면 `ajou` 대신 해당 저장소와 ROS 환경을 사용한다.

### 터미널 1 — ROS master

```bash
ajou
rc
```

### 터미널 2 — guarded bridge

```bash
ajou
python3 dataset/morai_od/scripts/run_base5_rosbridge.py
```

기본 localhost9090. 일반 rosbridge launch로 바꾸지 않는다.
MR에서 Capture Mode를 켜고 센서 설정·Sunny13시를 실제 확인한다.
Ego Network의 Load → Connect에 다음 profile을 사용한다.

```text
$HOME/AjouNice2026/configs/network/ego_network_mr_base5_savesensordata_collision_local.json
```

7-interface의 Ego/Object/CollisionData·ScenarioLoad·SaveSensorData·제어 서비스를 유지한다.
UDP receiver, sensing, localization, BEVFusion, planning, control은 별도로 켜지 않는다.
수집기가 정지 Ego·Park와 시나리오 주입을 관리한다.5K 파일을 MR GUI 폴더에 복사하지 않는다.
필요한 소량 임시 파일만 설치/회수해 GUI 과부하를 막는다.
다른 output의 수집기라도 같은 ROS master에 두 개를 동시에 실행하지 않는다. live lock이 차단한다.

### 터미널 3 — 승인 후 수집 / 같은 명령으로 재개

새 manifest/runtime은 최종 XY 높이 보완 v113이다. 구 pilot output에 다른 계약으로 resume하지 않는다.
이전 v107 소량표본의 전체5K 외삽은45.7h였으며 새 v113의 확정 ETA가 아니다.
정적 미션 자산 분포가 바뀌었고 동적 준비비용·부족분도 남는다. 실제 누적 속도를 사용한다.
아래전체명령은사용자승인후에만실행한다. 현재자동으로시작하지않는다.

```bash
ajou
python3 -u dataset/morai_od/scripts/run_base5_batch_campaign.py \
  --manifest "$HOME/morai_cap/base5_v2_261007/final_5k_final_xy_ground_v113_final/manifest.json" \
  --output "$HOME/morai_cap/base5_v2_261007/collection_5k_final_xy_ground_v113" \
  --limit 5000 \
  --chunk-size 4 \
  --approve-bulk-capture \
  --resume
```

`--limit`은 목표 scene 수, `--chunk-size`는 공유 ROS child당 개수다.
64셀 초과는 `--approve-bulk-capture`가 있어야 한다. `--resume`은 첫 실행에도 사용 가능하다.
완료 성공분은 검증 후 재사용하며 실패분을 몰래 성공으로 세지 않는다.
수집 중 코드·config·manifest·scenario를 수정하지 않는다. 변경 후에는 새 output이 필요하다.

### 터미널 4 — 대시보드

```bash
ajou
python3 dataset/morai_od/scripts/base5_capture_dashboard.py \
  --capture-root "$HOME/morai_cap/base5_v2_261007/collection_5k_final_xy_ground_v113" \
  --port 8075
```

브라우저 `http://127.0.0.1:8075`. 승인/부족분·누적 시간·승인 속도·디스크 여유,
현재 recovery 및 실제 capture stage를 확인한다. 아직 output이 없어도 not_started를 표시한다.
화면의 finished는 승인 수가 아니다. 부모 eta가 준비 중0으로 표시돼도 멈춤으로 단정하지 않는다.

## 정지·resume·부족분

현재 chunk와 Park 검사를 마친 뒤 일시정지:

```bash
ajou
touch "$HOME/morai_cap/base5_v2_261007/collection_5k_final_xy_ground_v113/PAUSE"
```

paused 확인 후 PAUSE를 제거하고 터미널3과 동일한 명령으로 재개:

```bash
ajou
rm "$HOME/morai_cap/base5_v2_261007/collection_5k_final_xy_ground_v113/PAUSE"
```

긴급 STOP은 해당 output의 `STOP` 파일이다. 종료 중 Park가 확인되기 전 다음 수집을 켜지 않는다.
`halted`는 자동 resume하지 않는다. 로그·shutdown_safety를 확인하고 안전 복구 후 새 output이다.
확인된 CollisionData 접촉은 부족분으로 분리하고 새 장면의 안전/Park가 확인된 경우만 진행한다.
통신 단절·Park/pose 미확인은 전체 halt다. 정적 secondary 누락은 review이며 성공이 아니다.
정적 QA child당 실측 높이 재촬영1회·배치 대체1회·같은 UID/payload의 근접 witness1회,
총 witness 최대8회다. 반복 한도 초과는 deficit로 남기며 raw GT/ground gate를 완화하지 않는다.
바깥 quota recovery는 기본2round라 전체 비용과 한 child 비용을 혼동하지 않는다.
지도 roadside/unsupported를 지면 증거로 승격하거나 plane 미확인만으로 Z를 낮추지 않는다.

complete_with_deficit의 resume은 검증 전용이며 새 촬영으로 부족분을 채우지 않는다.
`deficit_manifest.json`을 확인해 같은 asset/family/distance/direction quota로 별도 보충 campaign을 만든다.
리포트의 effective coverage와 최종 selected_ledger를 합쳐 검사한 뒤 학습 전달본을 만든다.
잘 나온 장면만 선별해 희귀/근접 quota를 없애지 않는다.

## 최종 감사·CPU GT 변환

```bash
ajou
python3 dataset/morai_od/scripts/report_base5_final_coverage.py \
  --manifest "$HOME/morai_cap/base5_v2_261007/final_5k_final_xy_ground_v113_final/manifest.json" \
  --ledger "$HOME/morai_cap/base5_v2_261007/collection_5k_final_xy_ground_v113/selected_ledger.jsonl" \
  --output "$HOME/morai_cap/base5_v2_261007/collection_5k_final_xy_ground_v113/effective_coverage.json" \
  --require-effective-complete
```

전체 승인 장수와122종 실제 visible-primary 최소4장은 별개 검사다.
4장은 학습 충분성 보장이 아니라 누락 방지 하한이다. 배치 수와 실제 positive parent 수도 분리한다.
raw Instance component GT는 보존하고, LiDAR≥1점 또는 실제 camera Instance visibility 근거를 사용한다.
아예 안 보이는 객체는 positive로 넣지 않는다. LiDAR/Ego ROI는[-100,-30,-3,100,30,2]다.
학습 전달본에는 accepted·review없음·calibration아님만 편입한다.

```bash
ajou
python3 dataset/morai_od/scripts/build_base5_bevfusion_infos.py \
  --ledger "$HOME/morai_cap/base5_v2_261007/collection_5k_final_xy_ground_v113/selected_ledger.jsonl" \
  --output "$HOME/morai_cap/base5_v2_261007/base5_5k_infos_final_xy_ground_v113" \
  --sensor-config configs/sensors/Sensor_competition_local.json
```

최종 원본 캡처/raw/실패·calibration 근거는 재검증을 위해 남긴다.
Sunny13시와 다른 환경을 같은 output의 resume에 섞지 않는다. 나머지5환경은 별도5K campaign이다.

## 정적만 먼저 수집하기로 승인한 경우

정적4,171장과실제이동829장은ordered manifest에서이미분리했다.
정적에만한정할경우아래별도output을사용한다. 이output을나중에limit5000으로resume하지않는다.
이것은전체5K를완료한것이아니며실제이동829quota를없애는것도아니다.

```bash
ajou
python3 -u dataset/morai_od/scripts/run_base5_batch_campaign.py \
  --manifest "$HOME/morai_cap/base5_v2_261007/final_5k_final_xy_ground_v113_final/manifest.json" \
  --output "$HOME/morai_cap/base5_v2_261007/collection_static4171_final_xy_ground_v113" \
  --limit 4171 --chunk-size 4 --approve-bulk-capture --resume
```

대시보드도해당static output을가리킨다. 기존정적4160표본의조건부외삽13.4h는새4171분포의확정예상이아니다.
동적은추가준비최적화/한계검토후별도campaign으로진행하고마지막에전체quota감사한다.
