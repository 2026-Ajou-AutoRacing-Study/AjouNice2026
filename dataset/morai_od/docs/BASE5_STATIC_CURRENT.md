# Base5-Static 생성기 — CURRENT PREPARED

2026-10-05. **생성기·새 카탈로그 연결 완료, MR 자산12-scene 동기 smoke 완료. 본 수집 미실행.**
Production E25, 학습 ROI, tracker 및 localization 계약은 변경하지 않았다.

MR 동기 캡처 연결은 [Base5 전용 Network](BASE5_SYNCHRONOUS_CAPTURE_NETWORK.md)를
사용한다. JSON 준비/설치와 실제 subscriber 연결·캡처 성공은 별도 검증이다.

자산 정책: [MR 카탈로그 현재 상태](MR_ASSET_COLLECTION_CURRENT.md)를 따른다.
아래 생성기는 새 versioned catalog의59 Vehicle/28 Pedestrian/19 Obstacle 후보를
사용한다. [12-scene smoke](261005_base5_mr_asset_review_smoke.md)에서 G80 지면 실패와
복합객체의 동일UID 다중GT를 확인했다. 전체pool·초근접·partial-view 검증 완료는 아니다.
일반 표지판은 도로변 전용이며 현재 positive pool에는 넣지 않는다.

동물/아기/복합 자산은 위 MR CURRENT의 최신 학습 정책을 따른다: 3-class 유지,
원본 세부 종류 및 component GT 보존, 소량 포함 후 평가. 이는 정책 합의이며
component-preserving 학습 변환 및 positive pool 편입은 아직 구현하지 않았다.
자전거 runtime 매핑과 동물의 강제정지 정책 한계는 별도 해결 대상이다.

센서 거리 경계: [MR/VLP16 live 증거](../../../docs/bevfusion/261005_MR_VLP16_100M_RANGE_EVIDENCE.md).
fireDistance150을 저장해도 실제 출력은 약100m로 제한됐다. Base5의20–60m 차량
목표는 유지하며100m 밖 LiDAR return을 전제로 대량 수집하지 않는다.

## 목적과 규모

SaveSensorData의 LiDAR Instance geometric GT를 직접 사용하는 독립 single-frame OD
보강이다. Base3 공용 좌표·MGeo·MR template 로더와 Base4 상대 배치/횡 offset/도로 구간
아이디어를 재사용한다. Temporal/velocity 성능 검증용 데이터가 아니다.

`../configs/base5_static_v1.json`에서 교체 가능한 초기 할당량을 관리한다.

| 묶음 | 목표 프레임 | 요청 pivot 거리 |
|---|---:|---|
| near_obstacle | 4,000 | 3.2–5 / 5–10 / 10–20 m |
| far_vehicle | 4,000 | 20–30 / 30–45 / 45–60 m |
| pedestrian | 1,500 | 3.2–10 / 10–20 / 20–30 m |
| background | 500 | 명시적 빈 배경 |

**1 scene = 1 capture**, 총 10,000개의 독립 배치 목표다. 정지한 동일 장면을 3번
찍어 10k로 부풀리지 않는다. 따라서 과거 3,334 scene × 3 capture의 Base3보다 로드
횟수가 많다. 수집 시간은 live 측정 전 확정하지 않는다.

위 거리는 geometric GT center/표면 간격이 아니다. 특히 **0–2m geometric GT 공백을
이 설정만으로 해결했다는 뜻은 아니다.** 첫 live에서 asset별 center·크기·차량과의
비충돌 간격을 측정하여 초근접 별도 배치를 정해야 한다. 현재 기본값은 이를 무작정
겹치도록 배치하지 않는 초기 검증용이다. Partial-view·ground-support도 미검증이다.

## 구현된 계약

- MR runtime eligibility/template 사용. WoodBox 및 MR 복구 자산을 과거 MRP quarantine으로
  제외하지 않는다. Obstacle positive는 ontology_class=Obstacle만 사용한다.
- Ego 초기속도 0, 정적 차량·정적 보행자, 장애물 scale=1. Spawn traffic은 비운다.
- 방향·거리 bin·yaw·asset·offset은 서로 다른 seed의 균형 deck으로 섞는다.
- Ego offset은 씬마다 고정 ±0.15/0.30m, merge/taper에서는 최대 ±0.15m.
- 원거리 차량은 실제 MGeo 고주로 3개 경로의 앞/뒤 및 인접 차선 위에 둔다.
  요청한 이웃 차선이 없는 edge lane은 앞/뒤로 fallback하고 실제 role을 기록한다.
  Arc 거리와 XY 거리의 차이를 보존하며 짧은 경로/경로 밖 배치를 거부한다.
- 나머지 family는 Base3의 general/roundabout/merge/exit/taper/post-taper topology resolver를
  사용한다. Actor z는 road anchor의 z를 사용하지만 옆 지면의 높이까지 검증한 것은 아니다.
- Split은 link + world 50m cell 그룹을 hash하여 train/val 85/15로 배정한다.
  같은 그룹의 asset/yaw 변형을 섞지 않는다. 인접 cell이 완전히 독립 test라는 뜻은 아니며,
  최종 성능은 별도 실주행 held-out rosbag으로 평가한다.
- 개별 scene seed, 요청/배치 거리, 실제 role, topology, asset UID, split group 및 source hash를
  manifest/metadata에 기록한다. 같은 config/start/count는 같은 배치를 재현한다.
- **한 번에 최대64 scene**만 생성. 기존 nonempty output overwrite 금지.
  생성기는 설치/ScenarioLoad/녹화/제어 명령을 호출하지 않는다.

## 실행 — 아직 수집 명령 아님

저장하지 않고 10k 설계의 할당량만 확인:

```bash
cd ~/AjouNice2026
python3 dataset/morai_od/scripts/generate_base5_static.py --plan-only
```

live smoke 준비 시 새 디렉터리에 소량 생성:

```bash
python3 dataset/morai_od/scripts/generate_base5_static.py \
  --output "$HOME/morai_cap/base5_static_smoke/scenarios" \
  --start 0 --count 12
```

MR 위치가 다르면 `--runtime-asset-dir`, 지도 경로가 다르면 `--mgeo-dir`을 명시한다.
생성기 config와 코드만 Git에 둔다. 대량 scenario/capture는 Git 밖 새 경로에 둔다.
다음 배치는 `--start 12`처럼 같은 config/seed의 이어지는 범위를 사용한다.

## 본 수집 전 남은 gate

1. MR 동기 capture에서 4-camera + LiDAR + **Instance annotation**이 모두 저장되는지 확인.
   최근 일부 mode에서 annotation이 없었으므로 파일 저장만으로 성공 처리하지 않는다.
2. 해당 asset UID의 annotation, geometric center/size 및 정상 센서 상태 확인.
3. 충돌·Ego drift·지면 아래/공중 배치·차선 밖 offset 확인. 실패 scene은 제외/재배치.
4. Annotation 기준 실제 거리/방향/부분 가림과 accepted frame 할당량 감사.
   현재 manifest의 required_live_audits는 요구 사항이지 이미 구현된 자동 gate가 아니다.
5. Sparse GT는 기존 정책을 무조건 제거하지 않고 camera-visible 검토를 분리한다.
   GT 보정을 이중 적용하거나 미식별 객체를 조용히 background로 만들지 않는다.
6. 30m+ raw는 보존한다. 학습·출력·평가 ROI 확장을 함께 완료하기 전에는 현 ±30m
   E25 recipe로 모든 원거리 데이터를 학습했다고 주장하지 않는다.
7. 통과 후 batch 설치/회수·resume·audit 기반 부족 구간 보충 runner를 연결한다.
   오래된 Base3 10k shell은 MRP 경로/환경/manifest가 달라 그대로 실행하지 않는다.

## 검증 기록

- 10k 설계 plan-only 실행: scenario 생성0.
- `/tmp/base5_static_offline_smoke_261005`에 12 scene 생성, MR 설치/로드0.
- 5개 단위/integration 테스트 통과. TemporaryDirectory 안 64-scene 생성으로
  네 family, 정지 계약, scale1, manifest, overwrite 거부를 확인했다. 테스트 종료 시 삭제된다.
- 기존 Base5 관련 테스트도 별도로 회귀 확인한다. 오프라인 테스트는 live 성공의 근거가 아니다.

주요 코드: `../scripts/generate_base5_static.py`.
테스트: `../tests/test_base5_static.py`.
