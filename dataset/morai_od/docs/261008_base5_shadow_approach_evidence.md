# Base5 음영 구간 사전 진입 관측 — v122 EVIDENCE

2026-10-08. Base5-only Sunny13시 직접5K의 생성·센서 취득 검증이다.
본5K 수집/학습/모델 추론/실제 회피 제어는 실행하지 않았다.
Production E25/threshold/tracker/planning/control/localization은 변경하지 않았다.

## 변경 이유와 v121 기준

사용자가 제안한 중앙부 집중·오른쪽 주행 차로 위주·단순한 소량 이동에 더해,
밝은 구간의 Ego가 음영 내부 객체를 먼저 보고 진입하는 관측을 별도 quota로 만든다.
객체 위치와 Ego 관측 위치는 별도다. 중앙부 객체만 늘려도 바깥 접근 관측이 확보되지는 않는다.
이는 학습 분포에 대한 설계 사전확률이지 주최 측이 확인한 출제 확률이 아니다.

v121 실제 metadata/도로 선분 투영 검사: 입구120 중 Ego가 volume 입구 밖인68,
바깥 Ego 앞에 내부 actor가 있는60, 내부 앞쪽30%에 actor가 있는41이었다.
입구 Ego 범위는−15~+10m라서30–50m 전 접근은 없었다. 배치 숫자이지 검출 성공률이 아니다.

## 현재500셀

| 관측 단계 | Ego의 volume 입구/출구 기준 위치 | 전체 | 양성 |
|---|---|---:|---:|
| 바깥 먼 접근 |입구30–50m 전|60|58|
| 바깥 가까운 접근 |입구10–30m 전|60|58|
| 입구 직전 |입구0.5–10m 전|30|29|
| 진입 직후 |입구0–12m 뒤|90|86|
| 내부 |입구15m 뒤~출구15m 전|190|182|
| 출구 전후 |출구12m 전~15m 뒤|70|67|
| 합계 | |500|480|

내부190에는 v121의 실제 이동 제안24셀이 포함되며 파일/metadata 참조까지 그대로 보존했다.
원래 독립 terrain 준비·실제 이동/Z/충돌 gate도 유지한다. 기존 running1셀 ground deficit를
이번 정적20셀 PASS로 해결했다고 해석하지 않는다. 음영476정적/24이동, 전체4822정적/178이동이다.

- 새 바깥 접근150셀 중145양성은 모두 **Ego 밖/주 객체 안/전방**을 joint constraint로 강제한다.
  차량 primary54(일반49/정적 차량5), 장애물37/사람36/탑승자6/유모차5/반려동물 동반3/dummy2/동물2다.
- 대부분 접근 객체는 volume 길이10–40%에 배치한다. 먼 거리 bin과 충돌하는 드문 입구 관측4셀은
  60–90%를 허용한다. 내부 관측은20–80% 중심, 일부 가장자리 관측도 유지한다.
  실제 정적 주 객체456 중294가20–80%에 놓였다. 중앙부만 남겨 입구 대상 학습을 없애지는 않는다.
- 정적 주 객체는 route strip0m365/옆 strip3.5m91(80/20%)다. 이 구간의 실제 주행 차로를
  오른쪽 기준으로 쓰며 반대쪽/비스듬한 차량 모습도 남긴다. 공식 spawning 비율이 아니다.
- 다중 actor 중93셀은 좌우 엇갈린 strip 배치다. 같은 strip 다중 배치도 유지한다.
  footprint 분리·실제 도로 지지·native annotation 간격·CollisionData는 별도 검사한다.
- primary 거리3–10/10–30/30–60/60–100m의40/240/160/40셀은 그대로다.
  Ego 위치/대상 위치/거리 bin의 교집합을 먼저 선택하고 실제 radial 거리와 Ego-frame ROI를 다시 검사한다.
  중앙 객체+바깥 Ego의 합산 거리가100m 밖이면 재생성하며 GT ROI를 도로 접선 기준으로 바꾸지 않는다.
- Ego는 정지/Park snapshot이다. 진입 직후/근접에는 도로 지지가 되는 옆 차로 관측도 포함하지만
  upstream 전체가 두 차로라고 가정하지 않는다. 실제 planner/controller의 회피 경로를 재생한 결과는 아니다.

## 보존·구현

source: `~/morai_cap/base5_v2_261008/shadow_release_5k_v121/scenarios_5k/manifest.json`.
current: `~/morai_cap/base5_v2_261008/shadow_approach_5k_v122_b/scenarios_5k/manifest.json`.
SHA256: `8e9715f4fd7b2ba24970f51be35da97e9f07d0d2bbe2b9ad19e74a5247d579b9`.

476개 음영 정적 셀만 재생성했다. 기존4500개와 음영 이동24개, 총4524 entry는 원본과 완전히 같다.
1750개 기존 미션·로터리 범용100·범용/차량 초근접·122종 전체 primary 최소4회는 유지한다.
음영122종 primary 최소1회,5000고유payload/전수 OFFLINE_PASS/오류0다.
기존 family 장부와 의미별 shadow overlay를 구분하며 family 비율을 다시 늘리지 않았다.

- `revise_base5_shadow_approach.py`: fresh output 전용 정적476개 교체/원본 검증/frozen spec override/pilot 사전 선정.
- `base5_shadow_observation.py`: 단계별 Ego·내부 target joint placement, 실제 최종 XY의 연속 도로 투영 검증.
- `audit_base5_scenarios.py`: 단계·내부 target 계약을 독립 재검사한다.
- `base5_capture_audit.py`: 새 사전 접근 quota는 native positive primary가 실제 전방/ROI 안에 있어야 한다.
  CAMERA_1 실제 Instance pixel 여부도 별도 기록한다. LiDAR-only/camera-only의 기존 GT 정책은 유지한다.
  센서에 안 보이는 객체를 배치되었다는 이유로 positive/성공으로 세지 않는다.
- 같은-quota 재배치가 기존 spec을 재사용하므로 새 stage/내부 band/주 차로도 유지한다.
  모든 기존 runtime fingerprint가 shadow module/generator/audit hash를 포함한다. 구output resume 금지다.

## 실험-검증-수정 루프

1. 첫 생성본 `shadow_approach_5k_v122/`은 upstream 옆 차로 Ego의 footprint 도로 지지 실패로 중단됐다.
   실패 proposal/report는 보존했다.−50~−30m 구간 전체를 동일한 두 차로로 가정하지 않도록 수정했다.
   gate 완화 없이 바깥 Ego를 실제 route 위에 놓고 새 `_b` output으로 재생성했다.
2. 새5000개 전수 offline/Park/도로 높이/기존 미션 검사와 별도4524보존/접근 quota 검사 모두 PASS다.
3. live 시작 전3초 읽기에서 Ego/Object/CollisionData 약50Hz, Ego 정지 및 non-Ego contact 없음 확인.
   이미 연결된 guarded rosbridge를 사용하며 별도 UDP/BEVFusion/planning은 실행하지 않았다.
4. 사전 선정20셀은6관측 단계·차량/장애물·좌우 엇갈림·좌측 주 객체·근접·복합·빈 배경을 포함한다.
   실제23회 capture/최종20승인/부족0, active wall161.432초였다.
5. 원본2셀은 `primary_no_sensor_evidence`/새 forward visibility gate 실패,
   원본1셀은 `static_ground_severe_below`였다. 각각 bounded same-primary/spec/split replacement로 보충했다.
   실패 raw/audit는 그대로 남겼으며 실패를 성공으로 바꾸지 않았다.

## 실제 결과와 경계

- 최종20/20, 실제 peer contact 없음 및 각 boundary/final Park 확인.
- 바깥 접근 양성9/9: 내부 planned target/실제 native 전방 positive/실제 CAMERA_1 Instance 가시성 확인.
  native 이미지에서도 밝은 접근 도로와 어두운 터널 내부의 차량을 확인했다.
- CPU 변환20프레임(train12/val8)/GT29(vehicle13/obstacle12/pedestrian4),raw 보존.
  unsupported component6개는 기존 가시성/ROI 정책으로 제외하며 원본 annotation은 남긴다.
- 완료 resume의791raw 파일 SHA/inode/mtime/size·8event log·선택ledger·누적wall 불변/추가capture0을 별도 검사했다.
- 전체 Base5 회귀341개 PASS(72.644초), 새 단계/좌표 변조/가시성/camera-only 검사7개 포함이다.
  최종 실행 로그는 current root의 `tests.log`에 보존한다. `git diff --check`도 PASS다.

대표 전방 이미지:
`live_static/chunk_0000/capture/captures/b5v2_00139/CAMERA_1/26-10-08-12-48-07-649266_Intensity.png`.
Ego 입구31.19m 전, primary 내부38.14%/거리약68m의 차량 관측이다.
이것이 학습 후 detector나 planner가 해당 차량을 성공적으로 처리했다는 뜻은 아니다.

current root의 주요 산출물:
`revision_report.json`, `offline_audit.json`, `planned_validation.json`, `pilot_manifest.json`,
`live_validation.json`, `live_static/selected_ledger.jsonl`, `pilot_infos/conversion_audit.json`,
`complete_resume_verification.json`.

GT geometric center/size/component·LiDAR≥1점 OR 실제 camera visibility·4class·±100m/횡±30m는 유지한다.
새 장면을 생성·취득해 학습에 넣을 수 있는 구현/대표 검증이지5K 전체 성공률·122종 전체 노출·
물리적인 모든 접지·4090 실시간성·사전 회피 제어 성능 인증이 아니다.
기존10–13h 운영 예산은 그대로 조건부이며 첫100–200셀 실측으로 갱신한다.
본5K는 사용자 승인 후 런북의 **새 v122 output**에서 시작한다. 학습이나 본수집은 미시작이다.
