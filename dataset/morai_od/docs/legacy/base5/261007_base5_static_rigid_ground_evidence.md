# Base5 원거리 정적 지면 검증·원본 재검사 — EVIDENCE / v72

2026-10-07. 독립 Base5 single-frame 수집 QA이며 모델/production/localization 변경이 아니다.
본10K 미재개. GT·raw point/image는 변경하지 않는다.

## 원거리 불일치 분리

동일 Kona의 가까운 검증 평면과 원거리 annotation을 비교했다.

- yaw-only Ego world 변환: annotation-ground gap +0.173318m.
- 같은 rigid annotation의 전체 상대 회전으로 평면 연결: −0.058405m.
- 차이 약0.231723m. 가까운 직접 검사 gap−0.0584m와 새 결과가 일치한다.
- 상대 회전에서 roll 약−0.124°, pitch 약−0.402°가 나타났다.

이는 **비교 좌표 변환이 gap을 바꾸는 직접적인 근거**다. 절대 Ego roll/pitch를 별도 센서로
측정한 결과는 아니며 모든 MORAI 부유 현상의 원인을 확정하는 결과도 아니다.
해당 정적 배치는 실제 위치/heading이 같았으므로 원거리 yaw-only 결과만으로 차량이
떠 있다고 판정하고 입력 Z를 낮추면 안 된다. 그 처리는 v66부터 계속 금지한다.

## 새 검증 계약

`base5_static_ground_reference.py`는 가까운 지면 검사가 통과한 **동일 정적 배치**의
평면을 같은 rigid annotation의 전체 상대 R/T로 연결한다. current local plane이
부족한 경우에만 사용하며, local plane이 실제 above/below-ground라고 판정한 결과를
reference로 덮어쓰지 않는다.

필수 조건:

- calibration-only 근접 캡처 accepted/ground verified/최종 Park 확인.
- current/reference의 actor JSON payload SHA 동일: DataID/위치/회전/scale 등 변경 불가.
- 실제 Object_topic XYZ 차이≤1cm, heading≤0.05°, size≤1mm, 메시지 speed≤0.05.
  reference와 current의 before/after 모두 검사한다.
- 단일 rigid component, Instance size≤1mm 일치. 동적/복합 annotation에는 적용하지 않는다.
- annotation 상대 회전과 Ego heading 차이≤0.2°, Ego 수평 위치 차이≤25cm, 상대 tilt≤3°.
  잘못된 OBB 축/180°flip 또는 다른 도로에 옮겨진 객체는 거부한다.
- 평면 support가 전체 footprint를 지지하고 기존 annotation gap −30~+15cm 범위여야 한다.
- reference metadata/audit/telemetry/scenario/LiDAR/CollisionData/Park 파일 hash를 확인한다.
  관측 pose/view mismatch는 quality review, 손상된 hash/배치 계약은 안전 실패다.

이 검증은 동일한 정적 객체가 동일한 실제 pose·형상을 유지한다는 조건부 QA다.
Object_topic에 물리 roll/pitch가 없으므로 동적 actor나 애니메이션, 복합 객체, 다른 위치,
다른 scenario scale에 확대하지 않는다. 실제 mesh/collider 접촉의 직접 증명도 아니다.

## 캡처 비용 감소: 원거리 raw 재검사

처음 원거리 캡처가 오직 `actor_ground_plane_unverified`로 review였을 때만 재사용한다.
근접 witness 확보 후 원거리 촬영을 반복하지 않고, 별도 검증 view를 만든다.

- 원본 metadata/audit/실패 기록을 덮어쓰지 않는다.
- 센서·telemetry/scenario는 hardlink로 같은 bytes를 참조한다. **양쪽 raw 파일을 편집하지 않는다.**
- 별도 metadata/audit/reaudit_result에 새 proof와 원본 checksum을 보존한다.
- 입력 Z/GT 변경 없이 원본 before/after 실제 pose를 reference와 대조한다.
- 다른 quality 실패, 동적, calibration-only frame은 학습 quota로 승격하지 않는다.
- 완료 view 재사용 시 source/reference/raw hash와 재검사 결과를 확인한다.
  캠페인 resume에서도 완료된 child를 새 MR 명령 없이 검증·재사용한다.

## 실제 결과

근거 루트 `/home/luv/morai_cap/base5_v2_261007`는 Git 비추적이다.

| 세션 | 결과 |
| --- | --- |
| static_rigid_live_v67 | 약72m Kona 통과, raw GT/input Z 미수정, 종료 Park |
| static_rigid_campaign_v68 | Kona/Porter2/Scooter 혼합/TrafficBarrel4조건 모두 통과; Barrel은 동일quota 대체 |
| static_distance_live_v69 | Sorento44.11m, Mixer96.89m, 후방MPV88.11m, P54053.39m 모두 통과 |
| static_distance_timed_v70 | 같은4조건 모두 통과, 부모 전체78.617초/실제12촬영 |
| static_distance_reaudit_v71 | 같은4조건 모두 통과, 부모 전체53.596초/실제8촬영; 약31.83% 감소 |
| static_reaudit_resume_v72 | 최종코드1조건13.503초; 완료resume0.104초/추가촬영0/지면witness·Park 파일 hash 불변 |

v70 최종 gap: Sorento−5.40cm, Mixer−11.45cm, MPV−5.21cm, P540−8.97cm.
근접 plane이 원거리 frame에 직접 있었던 것처럼 가장하지 않고 reference provenance를 남긴다.

v71 원본 metadata/audit4/4 불변 및 LiDAR hardlink4/4 확인.
최종 v72에서는 raw/telemetry/scenario 등33파일의 기록 hash와 원본/검증 view 양쪽을 대조했다.
CPU 변환은 v68 정상4장/GT6개, v71 정상4장/Vehicle GT4개, v72 정상1장/Vehicle GT1개다.
별도 근접 calibration은 포함하지 않았다. Base5 단위 회귀179 tests PASS.

## 시간 추정·미해결 범위

위4조건은 **모두 sparse 원거리 지면**이므로 53.6/4초를 전체10K에 곱하지 않는다.
직접 검사 통과는 추가 캡처0회, reference 필요한 rigid 정적 배치는 근접 캡처1회 추가다.
원본 재검사로 추가 원거리 촬영1회를 없앴다. 부모 전체 시간에는 Python 시작/준비/QA도 포함한다.

이전18.7시간은 아직 새 정책의 확정 ETA가 아니다. 전체 family·거리·혼합에서 reference 요구율,
dynamic terrain 준비, retry/quota 보충을 측정한 뒤 다시 계산해야 한다.
미확인53자산의 초기 높이/복합 객체/secondary가 먼 혼합 배치/동적 전체경로/장시간 안정성은
이번4거리 표본으로 해결됐다고 간주하지 않는다. 자동 rigid fallback은 현재 primary가 문제인
정적 장면에 한정한다. 다음은 이 미해결 coverage의 대표 표본과 campaign pause/resume 검증이다.
