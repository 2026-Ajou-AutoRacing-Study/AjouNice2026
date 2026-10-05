# Base5 독립 10K v2 — CURRENT / 10K 생성·소량 live 검증

2026-10-06. 이 문서를 Base5 생성·캡처·변환의 최신 진입점으로 읽는다.
기존 v1/E25 보강 실험과 구분한다. **시나리오 10,000개 생성은 완료했지만,
소량 live는 아래처럼 통과/검토로 나뉘었다. 본 수집과 본 학습은 시작하지 않았다.**

## 목적과 구현 범위

- Base3와 섞거나 E25에서 시작하는 것을 전제로 하지 않는 독립 single-frame OD corpus다.
- 1 scene = 1 동기 캡처. Ego는 정지시키고 동적 자산은 움직이는 외관을 캡처한다.
- Temporal input, detector velocity target, GT object-motion input은 없다.
- MR만 사용하고 scale=1이다. 열린 G80는 기존 지면 실패 때문에 보류하며 일반 G80,
  WoodBox, MR 복구 자산은 포함한다. Tree/Fire/맨홀/Speed_Bump/일반 표지판은 positive가 아니다.
- Production E25, 0.40/0.15/0.10, tracker, planning/control/localization은 변경하지 않았다.
  새 모델 threshold는 학습 후 dev/held-out 평가로 정한다.

## 생성된 할당량

| Primary family | 범용 | 미션 조건부 | 합계 |
|---|---:|---:|---:|
| 고주로 차량 | 1,800 | 1,200 | 3,000 |
| 로터리 차량 | 200 | 500 | 700 |
| 기타 도로·정적 차량 | 200 | 100 | 300 |
| 실체 장애물 | 1,800 | 1,200 | 3,000 |
| 보행자·동적/복합 | 2,000 | 500 | 2,500 |
| 빈 배경 | 0 | 0 | 500 |
| 합계 | 6,000 | 3,500 | 10,000 |

이는 primary 장면 quota다. 보조 객체가 함께 있어 class별 박스 수와 다르다.
보행자 관련은 일반 사람1,875 / dummy125 / animal125 / 사람+개125 /
유모차125 / 탑승자125다. 풀은 일반 차량43, 정적 OBJ 차량16, 실체 장애물19,
사람25, dummy3, animal3, 사람+개1, 유모차3, 탑승자9를 사용한다.
기타 차량300장은 정적 OBJ150 / 일반 차량150이다.

고주로는 합류600 / 본선1,500 / 병목 이후600 / 출구300이며 primary 전방1,500 /
후방1,500이다. NPC는 실제 MGeo 경로 위치·yaw를 사용한다. 임의 NPC 횡 offset은 없다.
Ego yaw는 도로 대비 절대0–3° 60%, 3–10° 30%, 10–15° 10%, 부호는 랜덤이다.
Ego 횡 offset은 0/±0.15/±0.30 m이며 좁은 구간은 ±0.15 m와 lane-envelope 검사로 제한한다.

### 거리 — geometric center의 Ego-frame XY 거리

| 대상/방향 | 거리 bin과 비중 |
|---|---|
| 고주로 전/후 | 6–10:10%, 10–30:25%, 30–45:20%, 45–60:15%, 60–80:15%, 80–99:15% |
| 로터리 | 5–10:25%, 10–20:40%, 20–30:25%, 30–40:10% |
| 기타 차량 전/후 | 6–15:35%, 15–30:40%, 30–60:25% |
| 기타 차량 측방 | 3–6:25%, 6–15:50%, 15–29:25% |
| 보행자 관련 전/후 | 3–5:25%, 5–10:20%, 10–15:20%, 15–30:25%, 30–50:10% |
| 보행자 관련 측방 | 3–5:25%, 5–10:20%, 10–15:20%, 15–29:35% |
| 장애물 전방 | 3–10:40%, 10–30:35%, 30–60:20%, 60–99:5% |
| 장애물 측방 | 2–5:50%, 5–15:35%, 15–30:15% |
| 장애물 후방 | 3–10:60%, 10–30:35%, 30–60:5% |

장애물 방향은 전방40% / 좌20% / 우20% / 후20%다. 중심 거리와 표면 간격을
혼동하지 않는다. 초근접 측방은 추정 표면 clearance 0.3 m 이상으로 배치한다.
현재 ±30 m lateral ROI에서 측방30–50/60 m는 불가능하므로 위처럼 수정했다.
고주로 근접 대형차는 비충돌 가능한 실제 인접 차선/위치를 찾으며 불가능한 cell은
다른 방향으로 몰래 바꾸지 않는다. 512회 재시도 후 실패하면 생성 자체를 중단한다.

## GT·class 정책: raw 보존, positive 후보 비교

GT는 동기 LiDAR Instance의 center/rotation/size를 직접 사용한다. /Object_topic의
차축→중심 offset이나 -1.347°를 annotation에 다시 적용하지 않는다.
같은 부모 UID의 사람·개·유모차·아기·자전거 GT를 독립 component로 보존한다.

원본 point/annotation을 삭제하지 않고 `lidar5`, `lidar1`, `lidar1_camera0`를 각각 계산한다.
Camera-only 후보는 실제 Instance pixel, model bottom-crop 내 가시성, 최소16 pixel 및
4×4 px extent가 필요하다. 같은 부모·같은 class 다중 component는 카메라 귀속이
애매하므로 zero-point positive로 자동 승인하지 않는다. **최종 GT 정책은 미확정이다.**
역사적12장 fixture에는 zero-point camera-only 사례가 없었지만 새48장에서는3개를
관측했다. 작은 pilot이므로 아직 이 정책의 학습 성능 우월성을 검증한 것은 아니다.

움직이는 동물을 UNKNOWN 강제정지로 묶는 한계 때문에 별도 **4-class 연구 후보**
Vehicle/Pedestrian/Obstacle/Animal을 연결했다. 이전3-class 합의에서 바뀐 실험안이며
production 변경이나 최종 학습 ontology 확정이 아니다. Animal/bicycle의 tracker·planning
runtime mapping 및 기존 class 회귀 검증은 본 학습/승격 전에 별도로 결정해야 한다.
아기는 Pedestrian, 유모차는 Obstacle이며 component 원본 종류도 남긴다.

## 모델 wiring 검증과 production 경계

후보 config: `perception/detection3d/bevfusion/configs/bevfusion_base5_independent_100m.py`.
이미지320×896, positive ROI x±100/y±30 m, batch1, random init, 7D box다.
내부 연산 grid는 stride 정합 때문에 y±32 m로 padding한다. 이것을 positive 범위 확대로
해석하지 않는다. E25를 load하지 않으며 이미지 backbone도 pretrained 없이 smoke했다.

직사각형 BEV에서 기존 정사각형 grid가 숨기던 두 문제를 실제 loss pass로 확인/수정했다.
카메라 BEV XY→YX 전치 및 TransFusion position grid의 flatten/XY 순서를 별도 opt-in
모듈로 고쳤다. SECOND/FPN 정합을 위해 Y600 대신640 voxel을 쓴다.
기존 production 모듈/config는 건드리지 않았다.

5060 Ti 단일 batch의 finite loss와 전체 gradient finite 검사를 통과했다.
Peak allocated3.723 GiB / reserved4.715 GiB였다. 34.684초는 모델 초기화+역전파 smoke
시간이며 학습 throughput/추론 latency/모델 정확도 측정이 아니다. 기존 MR을 끄지 않았다.

## 증거 경로와 판정

로컬 출력 root: `/home/luv/morai_cap/base5_v2_261006/`.

| 증거 | 결과 |
|---|---|
| `scenarios_10k_v10/manifest.json`, `offline_audit.json` | dummy 정지 수정 후10,000 unique payload, quota/거리/추정 footprint 검사 errors0 |
| 동일 manifest split | train8,609 / val1,391, link-group hash |
| 생성 resample | 평균0.594회, 최대76회 |
| `historical_component_replay_v3/` | 기존12장 재파싱,17component, G80 2장 제외하여10장 후보 |
| Repo `artifacts/datasets/base5_v2_pilot/gpu_forward_smoke_v2.json`, `gpu_backward_smoke.json` | loss/gradient finite; 새 v2 수집 정확도 결과가 아님 |
| 동일 Repo 경로 `gpu_predict_smoke.json` | random-init 단일 batch decode14개, finite/4-class label 검사 통과; 정확도 아님 |
| `capture1/`, `capture2/` | 첫 load 뒤 서비스 응답/연결 실패, 새 v2 accepted0 |
| `capture_pilot_retry/`, `pilot_reaudit_v2/summary.json` | v9 대표48장 실제 캡처,41장 후보/7장 검토,원본/이전 audit 보존 |
| `dummy_fix_live/` | v10의 정지 dummy 재캡처1장 통과 |
| `dynamic_timing_fast/` | 거리 실패5장 재캡처,1장 통과/4장 거리 검토 유지 |
| Repo `artifacts/datasets/base5_v2_live_lidar1_camera0/gpu_live_predict_smoke.json` | 새 live 데이터41장 로딩 및 단일 GPU loss/decode finite |

Live에서는 첫 ScenarioLoad 뒤 MR rosbridge 연결이 모두 끊겼다. network=false 재시도도
pose 확인에 실패했다. Player.log의 ScenarioLoad 완료/인터페이스 disconnect와 ROS 미수신을
확인했지만 단일 근본 원인은 확정하지 않았다. 이후 연결이 복구되어 pilot을 재개했다.
Network 재로드 직후 service duplicate overwrite 로그도 확인했다.5초 timeout,
재광고 안정화 대기 및 idempotent mode/gear 요청1회 재시도로 연결 문제를 완화했다.
검증 응답을 생략하지 않으며 최종 통신 실패는 중단한다. MR 재시작은 하지 않았다.

v10 전체는 **OFFLINE_PASS이지 전수 LIVE_PASS가 아니다.** 43개 asset은 아직 캡처 기반 geometry
reference가 없어 보수적 크기를 사용한다. 지면/애니메이션/실제 mesh·collider, 근접 partial-view,
원거리 가시성은 이 검사로 인증하지 못한다. 인접 link 사이 시야 중복도 가능하므로
현재 train/val split을 최종 generalization test로 주장하지 않는다.

## 다음 순서

1. [동기 캡처 런북](BASE5_SYNCHRONOUS_CAPTURE_NETWORK.md)대로 연결/timeout gate를 유지한다.
2. 완료된48장 및 수정 후 재캡처: 파일 완결, 정지/동작, UID/component, 실제 GT footprint·거리·visibility 검사.
   새 캡처는 기록한 LiDAR→Ego calibration으로 GT center를 옮겨 거리 bin을 확인한다
   (0.5 m 허용 오차). Calibration 없는 과거 fixture는 LiDAR 거리/2.5 m의 제한적 검사다.
3. 실패 quota를 수정/보충하고 annotation 지면 검사 및 미검증 asset을 확인한다.
   동적 미션은 앞/뒤 횡단보도·도로 횡단의 명시적 geometry quota가 아직 부족하다.
   현재는 validator anchor 주위 독립 배치와8m actor 이동 segment 검사다.
4. Zero-point camera 사례로 positive 정책을 비교하고4-class/runtime 여부를 확정한다.
5. 본 수집 승인 후10,000 **accepted** frame을 모은다. 이번 생성10K가 accepted10K를 보장하지 않는다.
6. Base5-only from-scratch 학습 → 동일 mission held-out bag의 Detection/Tracking/Map-CV.

### Live에서 발견한 수정과 GT 정책 비교

- SUV/Sedan 등 차량 annotation subtype을 Vehicle로 정규화했다. 미지 class는 조용히
  Obstacle로 처리하지 않는다. 실패한 캡처도 raw/provenance를 남긴다.
- v9의 dummy는 `static`을 `standing`과 다르게 처리한 분기 때문에 speed8로 움직였다.
  v10에서 speed0/non-waypoint로 고쳤고 실제 재캡처가 통과했다.
- 다른5개 근접 동적 scene은 대기 중 이동으로 거리 bin을 벗어났다. 실제 GT를 원하는
  위치로 조작하거나 허용 오차를 늘리지 않고 capture timing을 바꿔 재검증했다.
  고정1.2초 대기 대신0.05 m 실제 이동을 확인한 즉시 캡처하고 제어 후 대기를 줄여
  1/5장이 통과했다. 평균 약4.864초/장이지만 나머지4장은 여전히 실패한다.
  **본 수집 gate 미통과**다. 이후에는 actor 시작 위치와 예상 capture 시점의 목표 위치를
  구분하고, 시작/종료 footprint 비충돌을 유지하면서 짧은 near bin의 phase를 맞춰야 한다.
  이벤트 재연결 지연까지 고려하여 예상시점만으로 성공 처리하지 않고 실제 GT bin을 확인한다.
- 후방80–99m 차량1장은 annotation/센서 evidence 누락으로 검토다. 이를 empty negative로
  쓰지 않고 가시성이 확보되는 별도 배치로 보충해야 한다.
- Camera pixel gate는 원본 pixel이 아니라 실제896-wide resize 후320 bottom-crop에서
  계산하도록 고쳤고 모든48장을 원본을 보존한 별도 audit 버전으로 다시 검사했다.

| 변환 후보 | 유지 scene | 유지 GT | 내용 |
|---|---:|---:|---|
| LiDAR≥5 | 33 | 42 | pilot Animal positive0, sparse 외관 누락 |
| LiDAR≥1 | 40 | 52 | Animal1, camera-only component 제외 |
| LiDAR≥1 OR camera-visible | 41 | 55 | Animal3, 실제 zero-point camera 사례 보존 |

이 비교는 accepted 후보의 primary가 해당 policy에도 보이는 경우만 변환한다.
전체48장의 raw GT는66component다. 이 pilot 선택은 train41/val0이므로 validation accuracy
목적으로 사용할 수 없다. 다음 pilot selector는 split도 coverage tag에 포함한다.

실측48장 raw는 약221 MiB, 평균5.576초/장·p50 5.381초였다. 단순10K 외삽은
약45 GiB/15.5시간이며 실패 보충·추가 metadata·환경 부하를 포함하지 않는다.
따라서 현재 single-frame load 경로에서10K 본 수집이2–3시간이라고 예상하면 안 된다.

전체 수집시간/용량을 pilot 외삽만으로 보장하지 않는다. 10K 파일을 MR 시나리오 GUI에
한꺼번에 넣지 않는다. Runner가 소량 runtime 파일을 설치하고 자신이 만든 파일만 회수한다.
