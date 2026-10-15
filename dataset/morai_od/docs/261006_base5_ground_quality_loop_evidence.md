# Base5 근접 지면 검사 수정·자동 보완 루프 — EVIDENCE v29

2026-10-06. Production E25/threshold/tracker/planning/localization은 그대로다.
10K 생성·본 수집·학습·GPU 추론/속도 측정은 하지 않았다.

## 먼저 정정할 내용

v28의 근접 장면 일부가 지면 아래라고 나온 것은 실제 배치 문제로 확정할 수 없었다.
같은 위치에서 객체를 비우고 Ego를 경로상 12m 뒤로 배치하여 지면을 따로 촬영하니
Woman2, Dog2, Ped_Pet1의 실제 annotation 하단은 주변 지면과 대체로 맞았다.
이후 원본 point cloud를 조사했다.

- Dog2 근접 원본의 Ego footprint 안쪽 점은 10,010개였고 모두 raw XYZ=(0,0,0)이었다.
  extrinsic 적용 후 전부 Ego z=1.388m의 같은 위치가 된다.
- 이 무효 좌표가 로컬 평면 적합을 왜곡해 약0.85m below-ground 오판을 만들었다.
  공식 MORAI no-return 인코딩 사양을 확인한 것은 아니며, 실제 ground return으로
  사용할 수 없는 zero-XYZ 데이터라는 관측이다.
- 지면 적합/가시성 point-count에서 zero XYZ를 제외했다. Ego footprint 자체도
  ground fit에서 제외한다. raw 센서 파일 및 학습 입력 cloud는 수정하지 않았다.
- 기존 정규17장 최종 코드 재검사: **13 accepted / 4 review**.
  v28의6/17 및 근접13장 중2장 통과 결과는 당시 검사기의 이력이다.
  현재 근접·혼합13장은 **9 통과/4 review**이며 남은 것은 지면 미확인2장과
  runtime/annotation 누락2장이다. 이유는 중복될 수 있다.
- v27 개 표본의 실제 +25–29cm 공중 gap은 별개다. 해당4조건의 입력Z 수정 및
  재촬영 효과는 유지되며 이번 오판 정정으로 그 문제까지 없었다는 뜻은 아니다.

## 보강한 자동 절차

`run_base5_quality_loop.py`는 최대4개 cell의 pilot 전용 orchestration이다.

1. 일반 generator가 만든 scene을 캡처하고 모든 기존 품질 gate를 적용한다.
2. above-ground만 실패했다면 검증된 입력 Z 하향 수정 deck을 만들어 재캡처한다.
3. 가까워서 지면 측정이 안 되거나 below-ground 의심이면 객체를 비우고 Ego를
   같은 경로에서12m 뒤로 옮겨 지면을 촬영한다. ordinary route와 일치하지 않는
   source는 임의 경로로 보내지 않고 거부한다.
4. 독립 raw 지면을 실제 Ego pose와 MR heading−1.347°로 world frame에 옮긴다.
   캡처 위치 주변 평면의 충분한 점/2D 지지/RMS/경사 조건 및 hull 외삽≤0.75m가 필요하다.
   지면 reference에는 source scene/LiDAR hash를 남긴다.
5. 원래 객체 배치를 다시 캡처한다. 로컬 평면과 독립 평면이 모두 있으면 서로
   10cm 이내로 일치해야 한다. 누락/충돌/visibility gate를 reference로 면제하지 않는다.
6. 여전히 안 되면 같은 primary asset/거리/방향/미션/정지·동적 요청/split의 새
   placement를 생성해 재검증한다. cell당 재시도는 최대2회다.

above-ground 입력 수정은 원래0.75m 이내 하향 한정이다. below-ground 의심만으로
무작정 위로 올리지 않는다. 지면을 못 구하면 통과시키지 않는다. raw GT를 사후 이동시키지 않는다.
20m 밖 및 이동 경로 전체의 지면 접촉을 인증하는 구현이 아니다.
이 단계는 **관측 기반 보완/재촬영 루프**이며 모든 장면의 최초 입력 Z를 정답으로
예측하는 universal terrain solver를 완료한 것은 아니다.

## 실제 수행 결과와 안전 중단

- 수동 분리 단계: empty ground3장 + 원래 배치 재캡처3장.
  Woman2/Dog2/Ped_Pet1 primary 거리3–5m와 차량·장애물 secondary의 혼합 장면이다.
  3장 모두 height/motion/Z step/visibility/CollisionData 검사 통과, 종료Park 확인.
  primary 하단–독립 지면 gap은 각각 +6.6cm, +0.7cm, −5.3cm다.
  입력 Z를 바꾸지 않아도 맞았다. 이전 below-ground 오판의 독립 대조 근거다.
- 자동 루프는 원래9-cell deck의 indices1,3,6,8을 대상으로 시작했다.
  첫 Dog1 혼합 scene은 initial ground review → empty ground capture →
  independent-reference 재캡처 및 종료Park까지 통과했다.
- 두 번째 Deer1 혼합 scene도 같은 보완 후 **raw 캡처 품질은 통과**했으나 종료Park
  서비스 응답이2번 연속 timeout났다. 자동 루프는 즉시 `halted`가 됐다.
  나머지 BabyStroller3/Scooter1 두 cell은 로드하지 않았다.
- 이번 실제 저장12장 = 지면 calibration5장 + 객체 장면7장이다.
  품질/안전 종료 모두 통과한 객체4장만 학습 변환 smoke에 선택했다.
  두 번째 Deer1의 raw는 보존했지만 unsafe-session 때문에 선택하지 않았다.
- 초기에 Park probe1회 timeout, 정지 상태 재확인1회 성공이었다. 모든 raw를 삭제하거나
  shutdown_safety.json을 성공으로 덮어쓰지 않았다. 현장 서비스 지연 해결을 주장하지 않는다.

loop에는 STOP 전달, child timeout/비정상 종료 후 후속로드 차단,
안전하게 끝난 partial selection 보존, dashboard 호환 status.json을 구현했다.
이 실행의 report는 이전 코드 형태라 partial selection은 report 안에만 있고,
실제 변환용 선택은 아래 독립 selector로 만들었다. 최신 코드는 halted 시에도
안전한 selected_ledger.jsonl을 저장한다. campaign 자체 resume은 새 output을 요구하며,
기존 일반 capture runner의 동일-code child resume 기능과 구분한다.

## 학습용 변환 smoke

`build_base5_bevfusion_infos.py`는 calibration_only 장면을 무조건 제외한다.
empty 지면 촬영을 학습 negative로 섞지 않는다.
LiDAR frame center box ROI `[-100,-30,-3,100,30,2]`를 명시적으로 적용하며
메타데이터에도 기록한다. 바깥 GT는 원본에서 삭제하지 않고 변환에서 제외한다.
기존 generator의 Ego-frame 거리 quota와 학습 LiDAR-frame ROI를 구분한다.

- 안전 종료4장 → train4 / val0 / test0. 성능 평가용 split이 아니라 wiring 검사다.
- GT8개: Vehicle1, Pedestrian2, Obstacle2, Animal3.
- policy `lidar1_camera0`, Animal camera-only2개 보존.
- RGB4개/프레임, LiDAR XYZI/finite, class index/7D box/캘리브레이션 CPU 검사 PASS.
- 변환8개 box와 raw audit annotation 차이 **0**.
- camera↔LiDAR extrinsic inverse 오차 최대약2.22e−16.
- 학습 input cloud에서 zero point를 없애거나 detector 전처리를 바꾼 결과는 아니다.
  실제 MMDet3D train-forward나 모델 성능 개선을 확인한 실험은 아니다.
- Base5 unit tests76개 통과. zero-placeholder regression, Ego 제외,
  blind-spot reference, unsafe child 후 중단, calibration 학습 제외, ROI 경계 검증 포함.

## 산출물

`/home/luv/morai_cap/base5_v2_261006/` 아래:

- `empty_ground_deck_v29/`, `empty_ground_capture_v29/`: 독립 지면3조건.
- `probed_ground_deck_v29/`, `probed_ground_capture_v29/`: 원래 객체 배치3조건.
- `quality_loop_v29/report.json`: 부분 성공/안전중단 및 child 원본.
- `quality_loop_v29/b5appearance28_03/retry_0/shutdown_safety.json`: Park 실패 근거.
- `ground_ready_v29/selected_ledger.jsonl` 및 `readiness.json`: 안전 선택4장/CPU 검사.
- `appearance_ground_report_v29_corrected.json`: v28 raw17장 재검사, 원본 ledger 불변.

PKL: `artifacts/datasets/base5_ground_ready_v29_infos/`.
다음은 사용자 재연결/서비스 정상 확인 후 미실행2조건 및 중단조건을 소량 마무리하는 것이다.
MR 종료 전 GPU latency를 측정하지 않으며 본10K는 여전히 미시작이다.
