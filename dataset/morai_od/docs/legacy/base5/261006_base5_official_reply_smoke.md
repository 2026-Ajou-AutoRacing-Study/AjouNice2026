# Base5 v33 — 공식 답변 코드 반영 / 12-cell live smoke 완료

2026-10-06. 최초 준비 때는 MR/ROS가 OFF였다. 사용자 재연결 후 아래 live smoke까지
완료했다. 소량 수집 파이프라인 검증이며 본10K 생성/수집/학습/모델 추론은 하지 않았다.

## 최종 live 결과

원본: `/home/luv/morai_cap/base5_v2_261006/official_reply_capture_v33/`.
`quota.json`: target12/accepted12/deficit0/pending0, `status.json`: complete.

- 12셀 모두 최초 촬영에서 통과. ground/visibility 재촬영 및 quota 보충은 이번에는
  필요하지 않았다. 새 코드에서 이 경로의 live 재발을 검증했다고 주장하지 않는다.
  이전 v31 live 보완 기록과 이번 회귀 테스트93개 통과를 별도로 유지한다.
- 첫 셀 종료 후 paused→동일 campaign resume→나머지11셀 완료.
  완료 후 read-only resume도 통과했고, 완료 셀 재촬영·quota 중복은 없었다.
- 12 child 모두 CollisionData 비충돌, 종료Park 첫 요청 성공(약4–19ms).
  첫 사전Park은 Ego 잔류 속도가 0.02 기준보다 커 기어 요청 없이 거부됐다.
  브레이크로 정지시킨 뒤 별도 `official_reply_park_v33b.json`에서 Park 응답을 확인했다.
  이 결과를 간헐 서비스 timeout의 근본 해결/장시간 무고장 인증으로 해석하지 않는다.
- Woman2·Dog1/2·Deer1·Ped_Pet1 동적5셀은 실제 이동과 capture-window Z step0m를
  확인했다. 근접 primary의 ground support gate도 통과했다. 초기 activation 전후 전체
  경로나 모든 자산의 지면 문제를 해결했다는 뜻은 아니다.
- 앞차 실제 LiDAR-frame 거리96.86m: LiDAR0점, CAMERA_1 Instance102 native/
  model-crop46픽셀. 뒤차95.83m: LiDAR4점, CAMERA_4 115/59픽셀.
  기존 lidar5 정책이면 이 두 GT는 탈락하며 현재 lidar1_camera0 정책은 보존한다.
  이는 실제 픽셀 존재 근거이지 learned detector 정확도 근거가 아니다.
- TrafficBarrel 측방 actual annotation gap0.36380m, CollisionData 비충돌,
  ground gap 약0.01768m. LiDAR0점이지만 CAMERA_2 픽셀로 보존됐다.
- Dog2 및 Ped_Pet1의 animal component도 camera-visible/0-LiDAR-point 근거로 유지.
- ground 검사 범위는20m다. 원거리 차량과 그 밖의 secondary는
  outside_local_ground_probe_range로 지면 인증 대상이 아니다. review0을 모든 자산의
  mesh/collider 접촉 인증으로 인용하지 않는다.

CPU 변환: `artifacts/datasets/base5_official_reply_smoke_v33/`.
train11/val1/test0, GT30개(vehicle6/pedestrian11/obstacle9/animal4).
센서 근거 부족 component3개는 제외(obstacle1/pedestrian2), 원본은 보존했다.
7D finite box·LiDAR/camera path 검사 통과, raw audit 대비 변환GT 최대차이0,
velocity target 없음, ROI LiDAR-frame ±100m×±30m.
이는 학습 입력 wiring이며 정확도 평가/최종 모델 선정이 아니다.
캠페인 폴더 약66.4MiB. 10K 시간/용량 예측의 대표 throughput으로 단정하지 않는다.

## 반영

- 열린 G80(40100034)은 catalog에서 competition_excluded. 두 pool loader에도 ID 방어 규칙.
  문 닫힌 2021_Hyundai_G80은 positive 유지. 로컬 런처의 목록 삭제 완료를 뜻하지 않는다.
- vehicleList 차량과 objectList의 semantic vehicle/obstacle은 외관 수집에서 항상 정지.
  rigid 자산 이동 스트레스 시나리오를 학습 분포로 사용하지 않는다.
- 관측된 사람/동물의 동적 외관과 일반 사람/Animal의 동적 후보는 유지한다.
  AI 이동 미검증은 정지, fixed-pose 관측 자산은 정지 우선이다.
- 미검증 복합 자산은 정지 캡처 후보로 두되 fixed-pose 인증으로 해석하지 않는다.
  후속 자산별 외관 시험으로만 동적 정책을 확대한다. component GT/ground/Z/collision gate 유지.
- detector/tracker/planning/localization, threshold, 속도 평가 기본값은 변경하지 않았다.

## 오프라인 smoke

최종 준비: `/home/luv/morai_cap/base5_v2_261006/official_reply_smoke_v33d/manifest.json`.
`offline_audit.json`: OFFLINE_PASS, 12 unique payload, errors 0.

- 일반 생성기의 근접·혼합9개: Woman2, Dog1/2, Deer1, Ped_Pet1,
  BabyStroller2/3, Bike3, Scooter1. primary와 secondary를 함께 확인한다.
- 전방·후방 80–99m Ioniq 각1개.
- Ego가 오른쪽으로 회피하여 객체가 왼쪽에 남는 alongside1개.
  계획상 footprint gap 0.31972m. 실제 collider/annotation gap 인증이 아니다.
- 위 준비 당시 원거리 관측점/픽셀, 근접 지면과 동작중 높이, CollisionData는 live 미확인이었다.
  geometry reference 없는 일부 component는 raw/독립 ground 보완에서 통과해야 선택된다.

초기 v33은 추가 alongside의 side 의미를 잘못 지정해 offline audit에서 거부됐다.
avoidance_side는 Ego의 회피 방향이며 객체의 관측 방향 role과 반대다.
v33b/c는 준비 실패/불완전 manifest로 사용 금지. 원본 이력은 보존했고 v33d만 최종 선택한다.
새 smoke 회귀 테스트가 12장·원거리 앞뒤·G80 제외·side audit를 검사한다.

## 실행·resume 이력

네트워크는 `BASE5_SYNCHRONOUS_CAPTURE_NETWORK.md`의 master→ROS bridge→
MR seven-interface 연결, Capture Mode/Instance 저장 계약 그대로다. 센서 UDP나 풀스택 불필요.
새 output에서 사전 Park 통과 후 첫 셀 stop/resume, 남은 셀 quality/capture audit를 한다.
통신 stale/CollisionData 접촉/미확인 Park은 후속 로드를 중단한다.

```bash
ajou
python3 dataset/morai_od/scripts/probe_base5_park.py \
  --output "$HOME/morai_cap/base5_v2_261006/official_reply_park_v33.json"
```

PASS일 때만 이어서:

```bash
ajou
ROOT="$HOME/morai_cap/base5_v2_261006"
python3 dataset/morai_od/scripts/run_base5_capture_campaign.py \
  --manifest "$ROOT/official_reply_smoke_v33d/manifest.json" \
  --output "$ROOT/official_reply_capture_v33" --stop-after-cells 1
```

첫 셀의 안전 종료를 확인한 뒤 동일 manifest/output에 `--resume`을 사용한다.
사전 Park 증거 파일이 이미 있으면 덮어쓰지 않고 새 이름을 사용한다.
live 후 raw annotation/ground/visibility/Z, ledger·quota·deficit 및 CPU 변환을 검사한다.
위 첫 셀 명령 뒤 동일 명령에 `--stop-after-cells 1` 대신 `--resume`을 넣어 완료했다.
현재 완료 output에 같은 계약으로 resume하면 무결성만 확인하고 촬영하지 않는다.
새 실험은 다른 output을 사용한다. 10K 준비 승인은 별도다.

## 다음

공식 제외/동작 정책을 유지하며 최종10K 거리·방향·자산·혼합 quota를 오프라인 감사한다.
user가 승인하기 전 10K 시나리오 생성과 본 캡처를 시작하지 않는다.
