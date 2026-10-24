# Base5 음영 구간 관측 보강 — EVIDENCE

2026-10-08. 독립 Base5 Sunny13시5K의 터널 내부·입구·출구 관측을 보강한다.
E25/threshold/tracker/planning/control/localization은 수정하지 않는다.
본5K 수집·모델 학습·추론은 실행하지 않는다. 아래 live 결과는 센서·GT 취득 검증이지 탐지 성능이 아니다.

## 공식 근거와 Base3 비교

사용자 전달10/06 MORAI 답변은 고주로 정차 차량/정적 장애물 배치 없음, 터널은 대회 F2 전체
객체 범위 고려, 동적 객체에 정지/재출발 허용이다. Vehicle/Spawn의 Activation Distance는 없고,
MGeo 역주행·Spawn waypoint는 불가능하나 개별 NPC waypoint로 차로 침범은 가능하다.
기존 정적 장애물 자산을 억지로 움직이지 않으며 G80 열린 문 공식 제외는 유지한다.

Base3 초기 route-wide 설계에는 gps_shadow_obstacle6/gps_shadow_vehicle_interaction7 clip이 있었다.
최종 class-centric Base3의 감사표에는 별도 음영 quota가 없고 route_general 표본에 포함될 수 있었다.
두 기록을 같은 최종 수량으로 해석하지 않는다. 이번에는 어두운 배경의 관측 quota를 명시한다.
Base3의 그늘 구간 참조 위치와 MGeo 경로 아이디어는 참고하되 Base3 데이터를 결합하지 않는다.

## 5K 안의500장 overlay

기존 범용 장면480장/빈 배경20장의 관측 위치를 교체한다. 추가500장이 아니다.
로터리 미션250장과 범용100장, 고주로 미션600장/근접66장, 정적 회피650장,
사거리 동적 관측250장 및 기존 범용 초근접 셀은 보존한다. 나머지4500개의 manifest entry와
source scene/metadata 참조를 그대로 유지한다. 고주로 범용에서는162장을 터널 관측으로 이동한다.
family는 기존 quota 장부의 이름이며 shadow_phase가 별도의 위치 관측 분류다.

| 음영 관측 | 장면 수 |
|---|---:|
| 입구·명암 전환 |120|
| 내부 |260|
| 출구·명암 전환 |120|
| 합계 |500|

480positive의 primary 중심 거리:3–10m40장/10–30m240장/30–60m160장/60–100m40장.
ROI는 LiDAR/Ego[-100,-30,-3,100,30,2]다. 작은 움직이는 대상은 가까운10–30m 관측을 우선한다.
20장은 무객체 배경이다. 나머지는1개/2개/3개 배치를 섞어 실제 있을 법한 음영 상황을 관측한다.
기존 범용 혼합 장면을 전부 단일 객체로 변경하지 않는다.

학습 대상122종 각각 최소1번을 음영 primary에 포함한다. 기존 전체5K primary 최소4회는 유지한다.
희귀 정적 차량2종은 원래 근접 셀을 제거하지 않고 높은 빈도의 다른 donor를 교체해 음영 관측을 추가한다.
일반 차량·정적 차량·실체 장애물·사람·dummy·동물·반려동물 동반·유모차·탑승자 모두 포함한다.
원본 Instance component의 geometric center/size/class를 쓰며 raw나 GT에 인위적인 높이/위치 보정은 하지 않는다.

## 배치·이동과 높이

샘플 shadedArea volume과 실제 대회 MGeo route가 만나는 약97m 구간을 anchor로 쓴다.
그 volume이 카메라의 정확한 어둠 경계라는 뜻은 아니다. 실제 CAMERA_1 이미지 밝기도 따로 검사한다.
Ego는 입구/내부/출구의 경로 위에 놓고 yaw±3°/기존 작은 횡offset 및 전체 차체 도로 지지를 검사한다.
좌우 근접은 동일 도로의 두 차로 관측으로 생성하며 실제 map road_strip/near_centerline 근거가 없는
좌표는 버린다. 내부 primary가 volume 경로 범위를 벗어나면 같은 quota로 초기 재생성한다.
차량은 Park 정지 외관이고 방향은 순방향/반대 방향/비스듬한 차로 침범 자세를 섞는다.
이는 실제 NPC 끼어들기 제어·traffic interaction·MGeo 역주행 재현이 아니다.

사람·동물·반려동물 동반은 실제 이동 외관을 소량 캡처한다. AI motion 미검증 actor는 standing을 유지한다.
유모차/탑승자의 현재 고정 외관 정책과 rigid 정적 자산의 정지 정책은 그대로다.
사거리 수동 높이 lookup을 터널에 재사용하지 않는다. 초기 MGeo/자산 pivot 후보 뒤에 독립 empty
terrain 및 정지 annotation reference로 실제 지면/경로 높이를 준비하고 실제 움직임·Z·접촉을 검사한다.
터널 이동은 준비된 .5m terrain sample 경로를 사용할 수 있다. 사거리의 두 waypoint 정책은 변경하지 않는다.

## 실험-검증-보완 이력

1. 첫500장 생성에서 오른쪽 도로 밖 후보가 같은 셀에서 계속 실패했다. 두 차로의 위치를 구분하고,
   우측 관측은 Ego를 실제 옆 차로에 놓도록 보완했다. 도로 지지 gate를 완화하지 않았다.
2. 초기 정적 대표23장면은23/23승인,134.915초였다. 실제 내부 영상이 어두웠다.
3. 내부 대상이 volume 밖에 놓이는 일부 장면을 막고 source spec override를 config에 동결했다.
   후속 정적23셀은 bounded 대체 포함23승인/149.924초였다. 원거리 작은 Bollard가 가시성 실패한
   원본은 보존하고 같은 자산·거리·방향·split으로 보충했다.
4. 첫 실제 이동 pilot에서 지면 평면 지지가 부족했다.4셀의 초기/대체를 수행했으나4부족분이어서
   안전한 chunk 경계에서 pause했다. 이를 동적 부유나 성공으로 보고하지 않는다.
5. 기존 가까운 ground witness는 대상 뒤6m에서 찍었다. 실제 empty cloud에서 대상 주변의 지면
   점이 부족했다. 터널 mover의 추가 witness만10–12m로 뒤로 옮겼고 동일 plane/inlier/hull 검사를
   유지했다. 움직이는 작은 대상의 거리도30–60m에서10–30m로 줄였다.
6. 이 단계에서 개·사슴·사람의 실제 이동이 통과했다. 이동 경로를 도로 중앙으로 옮겼고,
   AI 이동 미검증 actor를 실제 이동 quota에서 제거했다. AI 자산 자체는 정지 학습 목록에 남는다.
   Dog1/Dog2/Deer1 및 반려동물 동반을 이동 pilot에 명시했다. 최종 생성본은 실제 moving24인지 별도로 검사한다.
7. v121 정적23셀은23승인/부족분0,186.118초였다. 이동7셀은4승인/3부족분,298.741초였다.
   Dog1/Dog2/Deer1/Woman7이 통과했고 Ped_Pet1/달리기/반대 방향 사람 셀의 실패 근거는 보존했다.
   `moving_initial_validation.json`은 사전 선정7셀 중3누락을 명시적으로 FAIL로 표시한다.
8. 대상 확인만으로는 긴 경로 출발부의 약3m에 평면 지지가 부족했다. 조건부 departure witness를 추가했지만
   12m 확인 위치는 일부 중간 경로를 자기 차량 bonnet/센서 근접 사각에 남겼다. source cell의 targetXY/
   자산/거리/속도/방향 요청/GT를 바꾸지 않고 departure witness를16m와12m의 상보 관측으로 보완한다.
   각각의 증거로 경로가 확인되면 그 이후 추가 촬영을 하지 않는다. 최대8개near/전체9개terrain 상한은 유지한다.
   처음 부족했던3셀만 새output에서 재시험했다. 16m 단독은 Boy1을 통과시켰지만 두 running셀은 남았다.
   보존된16m/12m witness를 합치니 Ped_Pet1 전체 경로가 같은 plane/hull 검사에서 offline 통과했다.
   이 보완을 fresh runtime 자동 경로에 연결해 나머지2셀을 실제 캡처했다. Ped_Pet1 running은 통과,
   Woman3 running은 같은-quota 재배치 후에도 지면 근거 부족으로 남았다. 희소 지면을 근거 없이 통과시키지 않는다.

## 정적 검증과 실제 어둠

최신 생성본 `~/morai_cap/base5_v2_261008/shadow_release_5k_v121/scenarios_5k/manifest.json`은
5000고유payload/오류0 OFFLINE_PASS다. 보존4500entry와 기존1750mission entry 모두 원본과 동일하다.
음영 한/두/세 객체208/185/87·빈 배경20, 전체정적4822/이동178이다.

`live_static` 사전 선정23셀은 입구6/내부10/출구7, raw positive GT45개
(vehicle16/pedestrian13/obstacle14/animal2)다. camera/LiDAR 가시성이 없는3component는 보존하되 GT에서 제외했다.
측정 가능한11ground gap은−6.25cm~+3.35cm였다. 미측정 나머지를 완전 접지로 인증하지 않는다.
front camera 도로 ROI 평균 밝기의 phase별 중앙값은입구29.97/내부23.30/출구48.38(8bit)이다.
actual native 내부 영상의 지붕/도로가 어두웠다. 대상122종 각각의 모든 시야·노출을 검증했다는 뜻은 아니다.
이미지 예시:
`live_static/chunk_0002/capture/captures/b5v2_00174/CAMERA_1/26-10-08-12-04-11-618100_Intensity.png`.

CPU 변환23프레임/45GT PASS. static 완료resume913raw파일·moving 초기 완료resume1020raw파일의
SHA/inode/mtime/size·선택ledger·누적시간은 불변, 추가capture0이었다. 이 검증은 해당runtime 시점에 했으며,
그 뒤 departure witness/fingerprint를 보강했으므로 이 개발output에 새코드로 다시resume하지 않는다.
새runtime으로 본수집은 별도 fresh output을 사용한다. 기존실패/정적/이동calibration은 전부 보존한다.
batch/quality/session fingerprint에 새 shadow module도 포함해 다른 생성 로직으로 partial resume하는 것을 막는다.

## 운영 범위와 한계

터널을 모든 객체가 균등 출제되는 것으로 가정하지 않는다. 이500셀은 영상·기하학적 객체 탐지 보강이다.
정지 차량/복합 fixed-pose는 motion policy상 외관 snapshot이며 차량 실제 끼어들기 제어는 구현하지 않는다.
이동 경로 지면 후보가 독립 확인되지 않으면 bounded same-quota replacement 또는 부족분이다.
불확실 지면을 강제로 허용하거나 GT를 생성하지 않는다. calibration-only 빈 장면은 학습negative에 넣지 않는다.
Ego Park/실제 정지/peer CollisionData/visibility/심각Z 검사는 계속 유지한다.

기존10–12h 근거에24개의 음영 이동셀 조건부terrain 준비 비용이 추가된다.
추가여유0.5–1h를 두어 전체 운영 예산은10–13h로 본다. 성공률/전체ETA 보장은 아니다.
첫100–200본수집의 누적rate로 갱신한다. 본5K/학습/추론은 미시작이다.
최종 이동 재검증·새runtime smoke/resume 결과는 아래 완료 절을 따른다.

## 최종 완료 결과 — 전수 합격과 구분

사전 선정32셀 중31셀 승인이다. 정적23+새runtime 정적2=25/25, 이동6/7이다.
동물3종/걷는Woman7/반대 방향Boy1/달리는Ped_Pet1의 실제 이동을 확인했다.
이동 시 관측 연속Z step 최대1.501mm, 독립 ground support 모두 확인, peer collision0 및 종료Park다.
Ped_Pet1의 사람/동물component는 별도 GT로 보존했다. 실제 이동 방향은 측정 지면에 맞춘 safe
candidate로 바뀔 수 있으며, 원래 requested targetXY/자산/거리/속도는 보존한다.

누락은 `b5v2_00008` Woman3 running 한 셀이다. 원본/동일-quota 대체 모두
`motion_ground_preparation_unavailable`이었다. floating이 확인됐다는 뜻이 아니다.
기본collector는 이런 셀을 bounded 대체 후 부족분으로 남기며 이후 셀로 진행한다.
통신/종료Park 불확실까지 일반 부족분으로 넘기지는 않는다. 본5K에서 모든178mover가 성공한다고 보장하지 않는다.

`final_validation.json`은31승인을 검사했지만32번째 셀이 없으므로 **FAIL/1누락**이다.
이를32/32 PASS 또는 최종5K 수집 완료로 표현하지 않는다. 고정 전체 계획은5000/122종/거리·근접 quota를
만족하며 일반정적/사거리 이동의 기존 근거도 유지한다. 남은 한 셀은 수집 후 같은-quota 보충 대상이다.
자동 부족분 처리/원본검사/시간 예산을 포함한 제한적 운영 구현은 완료했고 본수집 자동승인은 하지 않는다.

실제31프레임은 CPU infos로 모두 변환됐다. GT54개: vehicle17/pedestrian16/obstacle15/animal6.
이는 탐지 모델 성능이 아니라 accepted raw annotation/visibility/ROI 변환 검증이다.
최종 runtime 완료resume는 moving411+static70=481raw파일 불변/추가capture0,
334 Base5 단위·회귀 tests PASS, git diff whitespace 검사 PASS다.
초기 프로토타입의913/1020/601raw resume 확인은 해당runtime별 이력이며 새code로 재사용하지 않는다.

산출물 root: `~/morai_cap/base5_v2_261008/shadow_release_5k_v121/`.

- `scenarios_5k/manifest.json`, `config.json`, `revision_report.json`, `offline_audit.json`
- `static_validation.json`, `moving_initial_validation.json`, `final_validation.json`
- 승인ledger: `live_static`, `live_moving`, `live_moving_departure16`, `live_moving_final`, `live_runtime_static`
  각각의 `selected_ledger.jsonl`. 겹치는source cell을 다른prototype에서 중복선택하지 않는다.
- CPU: `static_infos`, `live_moving_infos`, `live_moving_departure16_infos`, `live_moving_final_infos`, `live_runtime_static_infos`
- resume: `static_resume.json`, `moving_initial_resume.json`, `moving_departure16_resume.json`,
  `moving_final_resume.json`, `runtime_static_resume.json`

generated raw/manifest는Git에추가하지 않는다. MR GUI에는 pilot 임시 파일만 설치/회수했으며5K전체를 복사하지 않았다.
다음은 사용자 승인 뒤 런북의 **새** `collection_5k_shadow_v121`에서 본수집과 실제 부족분 감사다.
