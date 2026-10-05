# 2026-10-04 인지 거리·근접 검출·차량 승격 pilot

## 현재 결론과 적용 범위

목표는 전방 차량/장애물의 **빠른 존재 전달**, 후방 차량/로터리의 **위치·속도·궤적 안정성**이다.
현재 E25의 실패를 BEVFusion 전체나 향후 학습 모델의 한계로 일반화하지 않는다.
대회 RTX 4090은 아직 측정하지 않았고, 이번 GPU 시험은 RTX 5060 Ti에서 수행했다.
Production E25, threshold `0.40/0.15/0.10`, 설치된 tracker, planning/control/localization은 변경하지 않았다.

## 1. Tracker: 확정 UNKNOWN의 빠른 차량 승격 후보

기존 class score smoothing은 alpha=0.2다. UNKNOWN score가 1인 경우 CAR 관측을
반복해도 대표 class가 바뀌는 데 여러 관측이 필요하다. 검출 자체가 없거나 신규
track이 아직 tentative인 문제와는 별개다.

새 opt-in 정책은 다음 조건을 모두 만족한 **동일 track의 정상 association 이후**에만 적용한다.

- 이미 confirmed이며 대표 class가 UNKNOWN이다.
- 새 관측은 CAR/TRUCK이며 confidence가 유한하고 `>=0.60`이다.
- 차량 score를 관측 confidence로, UNKNOWN 잔여 score를 `1-confidence`로 설정한다.
- ID, association gate, 신규 track 확인 조건, coasting은 바꾸지 않는다.

`tracking_prediction.launch` 인자는
`tracker_fast_confirmed_vehicle_promotion:=true`,
`tracker_vehicle_promotion_min_confidence:=0.60`이다. 기본은 **false**다.
Replay wrapper에서는 `TRACKER_FAST_CONFIRMED_VEHICLE_PROMOTION=true`로 시험한다.
약한 관측/미확정 track은 기존 동작을 유지한다. 차량으로 영구 고정하는 정책은 아니며,
다음 class 관측에서는 기존 smoothing이 다시 적용된다. 반복적인 CAR↔UNKNOWN 진동과
잘못된 승격은 추가 회귀 검증 대상이다.

검증 결과:

- Tracker lifecycle/승격 단위 검증 8개 통과. 강한 CAR 즉시 승격·동일 ID,
  약한 관측/옵션 OFF·tentative 확인 조건을 검증했다.
- H03 고속 bag의 저장 detection을 ROS pipeline에서 OFF/ON으로 재생했다.
  동일 평가에서 TP/FP/FN 및 ID switch(0)는 같고, speed MAE는
  `0.768→0.772 m/s`, FDE@3s는 `1.5759→1.5757 m`로 개선 근거가 없다.
- 이 A/B summary는 unresolved GT 전체 프레임 제외를 적용한 최신 공식 subset과
  다르므로, 이전 보고서의 F1/AVE와 직접 비교하지 않는다.
- 기존 고주로 7-bag/로터리 3-bag 감사에서 첫 공간 매칭 track이 UNKNOWN인
  Vehicle segment는 0개였다. 따라서 이 bag들만으로 승격 개선 효과를 입증할 수 없다.
  새로운 원거리 UNKNOWN→CAR 사례를 확보한 뒤 production 승격을 판단한다.

첫 검출→첫 track header 차이를 전체 pipeline latency로 해석하면 안 된다.
두 출력의 샘플링/공간 매칭 시점이 달라 음수나 긴 차이도 발생하며,
bag 시작 이전 track, scene 진입·이탈 및 재등장도 분리해야 한다.
신규 track confirmation을 무조건 건너뛰는 기능은 이번에 구현하지 않았다.

## 2. 근접 장애물: 검출 단계의 추가 데이터가 필요하다

새 팀원 16-bag의 저장 detection/track과 보정된 geometric GT를 감사했다.
카탈로그 미식별 객체 근처 프레임은 기존 보수적 whole-frame 정책으로 제외했다.
아래는 AP가 아니라 **GT 객체-프레임 관측 합계**다. 긴 실패 구간의 반복 관측도 포함한다.

| 장애물 GT 거리 | 관측 수 | 같은 class가 2m 안에서 매칭됨 | 같은 class 후보가 5m 안에도 없음 |
|---|---:|---:|---:|
| 0–5m | 400 | 123 (30.8%) | 267 (66.8%) |
| 5–10m | 349 | 343 (98.3%) | 4 (1.1%) |
| 10–20m | 743 | 717 (96.5%) | 19 (2.6%) |
| 20–30m | 775 | 454 (58.6%) | 308 (39.7%) |

나머지는 다른 class나 association 미대응 후보이며, 위 두 열의 합이 전체는 아니다.
0–5m 전방 관측 177건 중 159건, 측면 169건 중 99건에서 주변 5m에 같은 class 후보가 없었다.
후방 54건에서는 9건이었다. 시점별 독립 성공률이나 인과관계를 입증하는 수치는 아니다.
근접 시 이미지 시야를 벗어남, 부분적인 LiDAR 형상, asset/거리 학습 분포 부족 등의
가설을 raw replay와 추가 수집으로 구분한다. Tracker만으로 해결했다고 하지 않는다.

10m 미만의 같은 class 후보 없음 관측은 CargoBox 48, PE_Drum 39,
TrafficBarrel 26, Concrete_Barricade01 136, PE_Firewall_Orange 22건이었다.
우선 수집 asset 선택에 사용한다. `concrete-barrier`는 137건 모두 2m 이내에서
같은 class로 대응했으나, 다른 asset과 단순한 난이도 순위로 비교하지 않는다.

수집 후보는 scale=1, 0.5/1/2/3/5/8m, 정면·사선·측면·후방,
좌우 통과 방향, 부분 시야 밖, 얇은 방향, 복수 장애물 조합이다.
Raw/GT를 보존하고 point 수가 0인 관측을 일률적으로 지우지 않으며 camera-visible 조건도 검토한다.

## 3. 고해상도: config와 실시간 전처리 불일치 해소

실시간 전처리가 `256×704 / resize=0.55` 고정이어서 모델 config만 바꾸면
이미지와 투영 조건이 일치하지 않을 수 있었다.
현재는 deterministic test pipeline의 final_dim/resize/reference_width/range를 읽고,
view-transform image/feature 크기와 다르면 시작 시 거부한다.
기존 E25 config에서는 종전 값을 유지한다.

후보 config:
`perception/detection3d/bevfusion/configs/bevfusion_base5_highres_320x896_pilot.py`

- image `320×896`, resize `0.70`, view feature `40×112`, batch 1.
- 첫 `320×880` 시험은 Swin padding과 depth feature 폭 불일치로 forward가 실패했다.
  32의 배수로 수정하고, 동일한 잘못된 설정은 전처리 검증에서 거부한다.
- E25 학습 가중치를 읽고 해상도 의존 frustum buffer는 새 shape로 생성한다.
  forward 통과는 고해상도 재학습 모델의 정확도 평가가 아니다.
- resize/crop도 달라지므로 순수 픽셀 수만의 A/B는 아니다. 재학습과 동일 변환으로 평가한다.

RTX 5060 Ti, 기존 1 sample·19,200 raw points, 4 cameras, batch 1 AMP,
warmup 후 12회. JPEG decode·전처리·forward·postprocess service 시간:

| 입력 | mean | p50 | p95 | peak allocated VRAM |
|---|---:|---:|---:|---:|
| 256×704 | 100.5ms | 100.7ms | 106.0ms | 10,854MiB |
| 320×896 | 134.6ms | 134.9ms | 139.9ms | 10,889MiB |

ROS 동기화·통신·tracker·planner까지의 end-to-end 값이 아니다. 1 sample이므로
scene 변화, 학습 memory, MORAI 동시 실행, 4090에서의 속도를 보장하지 않는다.
고해상도 실행은 가능하지만 정확도 개선은 미확인이다. 4090에서 같은 조건을 측정해야 한다.

## 4. 원거리: 입력·출력 범위와 존재 검출을 나눠 검증

현 모델의 LiDAR/BEV/출력은 ±30m 계약이다. 해상도만 높인다고 40m 이상의 출력이
가능해지지 않는다. 이는 **현재 config의 제약**이지 재학습 후의 상한이 아니다.
VLP-16 설정 fireDistance 100m도 실측 검출 보장 거리는 아니다.
2026-10-05 [MR live 거리 시험](261005_MR_VLP16_100M_RANGE_EVIDENCE.md)에서
fireDistance150을 저장해도 실제 UDP 출력은 약100m로 제한됨을 관측했다.
현재 MR/VLP16 수집은100m 상한으로 설계하고, JSON 변경만으로150m 확장을 가정하지 않는다.
전방 150m 등은 검증 목표이며, 100m 이상에서는 camera 주도 등의 별도 설계도 검토한다.

다음 수집·검증표를 MR 연결 후 구성한다. 이번에는 MORAI OFF여서 수집하지 않았다.

- 전방/후방 각각 20/30/40/60/80/100m.
- 120/150/200m는 camera 가시성을 먼저 확인하고 LiDAR가 도달한다고 가정하지 않는다.
- 거리별 camera 픽셀 점유, LiDAR 점 수, GT 가시성, 첫 detection/track/prediction을 저장한다.
- 중형차 중심으로 여러 차량·가림·인접 차선·갓길·합류를 포함한다.
  본선 NPC 속도가 샘플의 고정값과 같다고 가정하지 않는다.
- 상대 접근 속도/자차 속도를 바꾼다. 전방은 존재 검출 Recall과 첫 전달 거리를 주 지표로 삼는다.
  후방/로터리는 여기에 속도·ID·Map-CV 평가를 추가한다.
- ROI 확대는 voxel 수/BEV 해상도/head 출력/GT filter/평가 범위를 함께 설계한다.
  공간 범위만 넓힌 뒤 현재 가중치의 실패를 모델의 상한이라고 해석하지 않는다.
- production topic/type을 유지한다. 이번에는 별도 존재-only 출력 추가나 planner 수신을
  바꾸지 않고, 기존 계약에서 빠르게 전달할 조건을 검증한다.

## 재현과 잔여 작업

### 후속: ±60m longitudinal ROI wiring smoke

`bevfusion_base5_range60_pilot.py`를 추가했다. 전후방 ±60m, 좌우 ±30m이며
voxel 0.075m와 output cell 0.6m를 유지한다. Voxel range, sparse shape,
view-transform bounds, head train/test grid, bbox coder, input/GT filter를 함께 바꿨다.
Grid는 `[1600,800,41]`, image-view grid는 `[400,200,1]`이다.
Production alias/config는 이 후보를 선택하지 않는다.

중요한 checkpoint 위험도 방어했다. `view_transform.dx/bx/nx`는 ROI가 달라도
tensor shape가 같아서 E25 로딩 시 과거 값이 조용히 새 config를 덮을 수 있다.
Runtime은 checkpoint 로딩 후 config 기하를 복원하고, 새 range 학습 config는
`MoraiConfigViewGeometryHook`으로 train/val/test 시작 시 동일 복원을 수행한다.
학습된 image/depth 가중치를 삭제하거나 임의로 변경하는 기능은 아니다.
기존 LR hook을 유지한 채 추가 hook을 등록하고 checkpoint 로딩 이후 호출 순서도 확인했다.

RTX 5060 Ti에서 기존 sample의 512-point 축소 입력으로 baseline과 ±60m 후보
forward가 모두 통과했다. 별도 geometry 확인 run에서도
`nx=[400,200,1]`, `bx≈[-59.85,-29.85,0]`를 확인했다.
MORAI가 실제 실행 중이어서 그대로 두고 짧은 smoke만 수행했다.
내부 synthetic warmup은 그대로 존재하므로 peak VRAM은 단순 512-point 입력만의
memory가 아니다. 이 결과를 원거리 검출률·실주행 latency·4090 처리량으로 쓰지 않는다.

현재 남는 경계:

- image depth bin은 E25 호환 `[1,60,0.5]`를 유지한다. 축방향 ±60m ROI 모서리의
  camera evidence나 60m 이상 가시성을 보장하지 않는다. depth bin 확대는
  depth-head 채널/가중치 초기화와 memory까지 고려한 별도 재학습 실험이다.
- 기존 sample과 checkpoint로 wiring만 시험했으며, 30–60m 실제 차량 검출 성능은 미측정이다.
- train pipeline의 범위를 넓혀도 이미 ±30m로 잘라낸 dataset GT는 복구되지 않는다.
  raw bag/metadata에서 확장 GT와 point를 재변환하거나 새로 수집해야 한다.
- 기존 runtime/evaluator의 30m 출력·평가 필터도 별도로 점검해야 한다.
  모델 ROI만 바꾼 뒤 planning까지 60m 객체가 전달된다고 주장하지 않는다.
- full input의 forward/backward, 고해상도+확장 ROI 동시 조합, 4090 검증은 남아 있다.

추가 Docker 테스트 3개(기하 복원 2개, ±45m point 보존/횡범위 유지 1개)를 포함해
Docker suite 13개를 통과했다. 학습 hook 등록 확인은 완료했으나 실제 재학습은 아직 안 했다.

축소 smoke 재현(팀 Docker 내부):

```bash
python perception/detection3d/bevfusion/scripts/benchmark_resolution_pilot.py \
  --configs bevfusion_base5_range60_pilot.py \
  --max-points 512 --warmup-iterations 0 --iterations 3 \
  --output artifacts/evaluation/bevfusion/range60_pilot_261004/geometry_verified_smoke.json
```

Repository/`morai_cap` 및 최신 `docker/bevfusion/app` mount와 repository PYTHONPATH가
필요하다. 이 명령은 학습이나 production 서버를 시작하지 않는다.

검증: tracker 8개, 평가기 23개, Docker 전처리/프로토콜/캘리브레이션/TCP 10개
단위 테스트 통과. Python/launch XML/shell syntax와 diff whitespace 검증도 통과했다.
Tracker는 실험용 devel target만 빌드했으며 install_aw149 실행본은 교체하지 않았다.
테스트 통과가 전체 주행 회귀나 원거리 개선을 입증하는 것은 아니다.

Artifacts (Git ignored):

- `artifacts/evaluation/bevfusion/acquisition_nearfield_261004/e25_audit.json`
- `artifacts/evaluation/bevfusion/vehicle_promotion_261004/H03_{off,on}/`
- `artifacts/evaluation/bevfusion/resolution_pilot_261004/benchmark.json`
- `artifacts/evaluation/bevfusion/range60_pilot_261004/geometry_verified_smoke.json`

감사는 `perception/detection3d/bevfusion/scripts/audit_acquisition_and_nearfield.py`,
GPU 시험은 `benchmark_resolution_pilot.py`다. 동일 ID 3회는 짧은 연속 출력 확인이며
속도 안정성 보장이 아니다. 출력-GT association은 내부 tracker association의 대리 지표다.

우선순서는 원거리 가시성·ROI 설계와 근접 수집 → 짧은 재학습 → 같은 held-out bag의
존재 검출/Detection/Tracking/Map-CV 평가 → 4090 end-to-end 속도 확인이다.
현재 가중치나 installed tracker를 임의로 바꾸지 않는다.
