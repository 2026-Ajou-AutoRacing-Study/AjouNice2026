# Base5 로터리 실제 경로 관측·정적 NPC Park — v119 EVIDENCE

2026-10-08. 범위: 독립 Base5 5K 중 로터리 **미션250셀 geometry**를 재설계/생성/실제 MR 검증한다.
로터리 범용100셀·다른4750셀의 geometry는 보존한다. 후속 실험에서 초기 D 상태의 정지 NPC 이동을
확인해 정적 vehicleList actor의 기어 상태를 공통 Park로 갱신한다. E25/threshold/tracker/
planning/control/localization은 변경하지 않는다.
본5K 수집과 모델 학습/추론은 실행하지 않는다. 생성기 운영 검증과 모델 성능을 구분한다.

## 공식 답변과 설계의 경계

사용자가 전달한 MORAI 답변은 지정 경로·차선변경 비활성화·회전 궤적은 샘플과 같은 방식,
생성 주기는 고정이나 샘플과 다를 수 있음, NPC/Spawn별 목표 속도도 다를 수 있음,
차량 크기/생성주기에 따라 동시 대수가 달라짐, Ego와 상호작용해 감속·정차 가능이라는 내용이다.
별도 기존 답변에서 로터리 차종은 고정이지만 비공개다. 답변 수신일은 여기서 새로 추정하지 않는다.

따라서 궤적은 샘플로 제약하지만 본선의 속도·간격·차종을 샘플20km/h·5초·K5로 고정하지 않는다.
등속 traffic simulator/실제 끼어들기 controller를 구현하는 대신 가능한 상대 위치의 정적 geometry 관측을 만든다.
이는 temporal/velocity-head 학습이 아닌 single-frame OD 목적에 맞는 선택이다.

### 개발 bag으로 확인한 것

`rosbags/eval/perception/260923_teammate_manual/roundabout/260923_163034_R01_development/`와
`rosbags/raw_sensor_w_cam/sensor_cam_raw_roundabout1_roundabout_only.bag`만 읽었다.
`evaluation_locked` 입력은 참고 도구에서 거부한다. 평가 전용 bag 결과로 간격 quota를 튜닝하지 않았다.

- 새 R01 Ego arc 관측886.58–939.76m, 횡 p10/p50/p90≈−0.530/−0.001/+0.042m,
  yaw offset p10/p50/p90≈−4.30/−0.003/+5.37°다. 새로운±6°는 대표 관측 폭이며
  모든 주행 yaw의 최대치/실제 제어 오차 전체를 포함한다는 뜻은 아니다.
- 두 bag의 NPC 인접 pivot arc 간격 중앙값≈28.5m, 관측 대수 중앙값3/최대4,
  샘플 순항 속도 중앙값≈20.6km/h다. 본선의 보장 수치가 아니다.
- 구 roundabout1의 Ego는 약arc879.99m에 정지해 있었으므로 실제 Ego 통과 경로 근거로 쓰지 않았다.
- 해당 구 bag의 literal message 속도는 km/h, 새 R01은 m/s로 명시해 참고 도구를 실행했다.
  정규화 후 위치 차분/message 속도 비율 중앙값은 각각0.994/0.996이다.
  초기 자동 m/s 가정 보고서는 역사로 보존하고 현재 `development_reference_units_checked.json`만 참조한다.
  이는 두 특정 bag의 참고 조사이며 현재 UDP parser/기존 평가 속도 GT 정책을 변경한 것이 아니다.

## 구현

### Ego 단계별 관측

| 관측 단계 | 셀 수 | Ego 경로 arc (m) | 목적 |
|---|---:|---:|---|
| 접근 |20|870–883|진입 이전 회전 차량 조기 관측|
| 진입 대기 |30|883–887|양보 위치에서 가까워지는 회전 차량|
| 진입 |60|887–900|합류 경로와 NPC 경로가 만나는 시점|
| 원형 구간 |90|900–913|앞뒤 차량 사이에 Ego가 들어간 밀집 관측|
| 탈출 |50|913–934|앞·뒤·측후방에 남는 회전 차량|
| 합계 |250| | |

각 구간을 관측 셀 수만큼 나눠 셀 내부에서만 위치를 샘플링한다. 재시도도 같은 진행 구간에 머문다.
Ego XY는 실제 MGeo 미션 경로 기준이며 횡±0.30m/yaw±6°만 허용한다.
기존 final footprint road support·MGeo 높이·실제 정지/Park 검사를 유지한다.
기존 Base3 구간 이름의 circulation910–950에는 실제 탈출 이후가 섞여 있어 새 미션에서는 사용하지 않는다.
Ego metadata Z는 지면 높이이며 기존 serializer의 차량 기준점+0.34m와 구분한다.

### NPC 배치

샘플 spawn UID4의 경로 `A2256W000144→133→134→135→102`를 사용한다.
시작link point26/끝link 삭제point9로 실제 배열을 자르고 약92.8077m의 열린 경로를 만든다.
과거 temporal 실험의848/845/843 연장 경로는 **미션 NPC 배치에는 사용하지 않는다.**
범용 로터리100셀/과거 Base3·Base4의 경로 상수를 전역 수정하지 않았다.

- NPC pivot은 경로 연속 보간 위치, yaw는 해당 접선이며 별도 XY/yaw jitter가 없다.
- 자산별 geometric center offset/size로 배치 충돌 간격을 계산한다. `/Object_topic.size`를 학습 GT로 삼지 않는다.
- 원형 주행은 앞뒤 각1대가 필수이며 primary 방향도 균형 배정한다.
- 원형 주행의 앞뒤 경계 간격은 밀집0.5–2.5m27셀/보통4–10m36셀/여유10–22m27셀이다.
- 접근/진입/탈출도 밀집/보통/여유30/40/30%를 배정하되 교차 경로 특성상 같은 실제 gap 범위를
  강제하지 않는다. 이들 단계의 density는 배치 목표이지 인증된 gap 구간이라는 뜻이 아니다.
- Ego–NPC와 NPC–NPC 배치 OBB 최소 gap은0.5m다. 실제 물리 collider 검증은 이후 CollisionData로 따로 한다.
- 1대30셀/2대155셀/3대65셀, 총535대 배치다. 범용 혼합 예산으로 불필요하게 채우지 않는다.
- 일반 차량43종을 primary5–6회씩 순환한다. 대형차를 포함한 외형 일반화용이며 본선43종 출제 주장이 아니다.

### 실제 캡처 GT·가시성·재시도

원본 LiDAR Instance center/rotation/size를 그대로 읽고 raw를 보존한다.
기존 LiDAR≥1point 또는 실제 camera Instance crop visibility를 positive로 사용한다.
진입의 첫 두 NPC가 positive가 아니거나 원형 구간의 앞뒤 두 NPC가 실제로 안 보이면 review다.
원형 주행 첫 두 NPC의 실제 annotation gap이 밀집도 목표 범위에서±0.30m 이상 벗어나도 review다.
annotation 배열 순서가 아니라 parent UID로 매칭한다.
`roundabout_visibility` 근거와 실패 이유를 audit에 저장한다. 없는 GT를 임의 생성하지 않는다.

기존 bounded replacement는 primary 자산/phase/density/진행 셀/primary 방향/split을 유지한다.
대체 경로는 단위검사로 확인했고 부족분을 무한 재시도하거나 성공으로 세지 않는다.
정적 v115 advisory 높이 정책·CollisionData·정지/Park·통신 안전 계약은 완화하지 않는다.

## 동결 release와 전수 검사

root: `~/morai_cap/base5_v2_261008/roundabout_release_5k_v119/`

- `config.json`, `scenarios_5k/manifest.json`: 현재 계획/5000셀.
- `revision_report.json`: geometry 변경250/보존4750, 정적 차량 control-only 갱신2314,
  기존 파일 참조까지 보존2436, 원본/새scenario hash·초기 생성 재시도 근거.
- `offline_audit.json`: OFFLINE_PASS/5000고유payload/오류0, train4380/val620.
- `planned_coverage.json`:122종 primary 최소4, planned quota 부족0.
- `planned_roundabout_validation.json`: phase/density/개수와 source 보존 검사.
- `model_edge_pilot_manifest.json`: 결과를 보기 전에 선택한51셀. 로터리47셀은43종 primary·
  4phase×3density·경로 양 끝·최소 간격, 나머지4셀은 공통 Park 초기화의 다른 패밀리 통합 검증이다.

source는 `endpoint_linear_final_5k_v117/scenarios_5k/manifest.json`,
SHA256 `b6cc37084afd568b5fa95308c3a99f9e156442f1ee35fc43fcdf573385a4bd9b`다.
새release의 나머지4750 geometry는 그대로다. vehicleList를 가진2314셀의 기어 상태와 실제 위치 검사
metadata만 별도 사본으로 갱신했고 나머지2436셀은 기존파일 경로를 참조한다. source를 삭제하면 안 된다.
기본 sunny13/호환sunny11 config는 덮어쓰지 않고 별도 plan을 release config에 통합했다.
재생성은 런북의 `revise_base5_roundabout_mission.py` 명령을 따른다.

236/250셀은 첫 시도에 생성됐다. 나머지14셀은 같은 관측 셀의 도로 footprint 지원 재시도로 생성했고,
최대29회 재시도/도로 지지 거부81건을 기록했다. runtime 촬영81회 실패라는 뜻이 아니다.
전250셀에서 사용된 최소 **계획상** annotation 경계 gap은0.513744m다.

## 실제 MR 검증 이력과 실패 원인 분리

첫 설계 v118b: 4셀 종료 후 동일 계약으로15셀까지 resume해15/15승인,35/35차량 positive,
wall89.786s였다. 완료resume는545raw파일 hash/inode/mtime/size·선택ledger·누적시간 불변,
추가 capture0을 확인했다. CPU15프레임/35Vehicle GT 변환도 통과했다.
raw: `~/morai_cap/base5_v2_261008/roundabout_live15_v118b/`.

후속 v118의47셀은 당시 검사에서47승인/105차량 positive였다. 그러나 계획 pivot과 캡처 전후 실제 위치,
실제 앞뒤 gap을 추가 대조하니 **3/47셀은 정지 차량이 움직인 실패**였다. 초기 PASS를 최종 증거로 쓰지 않는다.
해당 raw/원 ledger는 수정하지 않았고 `roundabout_release_5k_v118/retrospective_strict_validation.json`에
FAIL3셀과 두 원형 주행 gap 불일치를 따로 기록했다. 최대 관측 pivot 오차는8.653m였다.
3셀은 `b5v2_00320`, `b5v2_02967`, `b5v2_00801`이며 GV80 primary/secondary가 포함됐다.

### 같은 위치/동일 객체의4조건 A/B

초기 link/constant/desired/current/viewCurrentVelocity가 모두0인데도 차량이 움직였다.
initPosition.gear / vehicleSaveData.inputData[6] / gearbox.gearMode가 D(4)였다.
`roundabout_static_probe4_v119`에서 같은 geometry의4조건을 실제 MR로 비교했다.

| 초기 상태 | 최대 pivot XY 오차 | 판정 |
|---|---:|---|
| 기존 D |5.2957m|정지 snapshot 실패|
| 기어3곳만 Park |0.00548m|정지|
| D 유지, 저장 바퀴/엔진 회전 상태만0 |5.4010m|정지 snapshot 실패|
| Park + 저장 회전 상태0 |0.00545m|정지|

4/4는 기존 가시성 QA를 통과했지만 그중 D의2개는 정지 상태 인증이 아니다.
따라서 저장 회전 상태 초기화가 원인 해결이라고 주장하지 않는다. 최소 수정인 **Park 초기화**를 채택한다.
물리 엔진 내부에서 D가 왜 잔류 구동을 만들었는지는 이 A/B만으로 확정하지 못했다.

`static_vehicle_policy=park-static-snapshot-v119`를 명시한 정적 vehicleList NPC만 적용한다.
위치·회전·자산·학습 GT는 바꾸지 않고 세 기어 상태를 일치시킨다. 실제 이동 actor는 이 마이그레이션에서 거부한다.
캡처 전후 이미 수집하던 Object snapshot 두 개 이상에서 계획 pivot XY 대비25cm 초과 이동/정보 누락은 review다.
이 검사는 velocity 단위에 의존하지 않으며 추가 대기/센서 캡처/height witness를 만들지 않는다.

### 최종 v119 실제 검증

현재51셀은 원래 사전 선정한47로터리+4통합 셀이다. 원본 v118 output을 새 runtime으로 resume하지 않는다.
`roundabout_model_edge_live51_v119`에서4셀 후 일시정지→동일 계약 재개를 확인했다.
20셀 durable 승인 뒤 chunk0005의 세 번째 load에서 pose 확인/토픽 freshness 타임아웃이 발생했고,
그 child의 마지막 Park는 미확인이라 halted를 보존했다. 해당 child의 추가2캡처는 최종 ledger에서 제외했다.
통신 오류를 가시성/배치 성공으로 처리하거나 halted 상태를 강제로 complete로 바꾸지 않았다.

이후 별도 `post_timeout_park.json`에서 실제 정지·Park 응답을 확인했고 Ego/Object/CollisionData가
각각 약50.1Hz로 복구됐다. 안전한 앞20셀만 유지한 뒤 **같은 원래 payload/ID의 나머지31셀**을
`remaining31_pilot_manifest.json` / 새 `roundabout_remaining31_live_v119`에서 검증했다.
`remaining_pilot` 도구는 원 payload hash·캡처 collision/boundary Park·child 종료 Park가 모두 맞아야
기존 성공을 제외 목록에 넣는다. 실패 raw와 halted 원본은 삭제하지 않는다.
최종 성공 선택은 별도 `validated_pilot51_ledger.jsonl`에 원래51셀 순서로 합쳤다.
원래 halted campaign/ledger를 완료 상태로 고쳐 쓰지 않았으며 중복/미완료 셀의 합산은 거부한다.

### 확정 운영 결과

| 검사 | 결과 |
|---|---|
| 최종 사전 선정 셀 |51/51선택, 로터리47+다른 패밀리4|
| 로터리 primary 차량 종류 |43/43종|
| 로터리 배치→실제 visible positive |105/105차량|
| 관측 단계 |접근12/진입11/원형15/탈출9|
| 원형 앞뒤 동시 positive·간격 quota |15/15통과|
| 최종 선택의 CollisionData 무접촉·boundary Park |51/51확인|
| 정지 NPC 최대 실제 pivot XY 오차 |로터리1.64cm / 다른 패밀리0.61cm|
| 원본 annotation 최소 Ego 경계 간격 |0.45385m|
| 로터리 측정 가능한66actor의 annotation 하단↔plane 최대 절대차 |13.92cm|
| 로터리 ground warning |39actor, 전부 plane 미확인; 부유 확정으로 해석하지 않음|
| CPU infos 변환 |51train frame /113Vehicle+2Pedestrian GT, 제외0|
| 완료31셀 resume |1111raw파일·8event log·ledger·누적시간 불변, 추가capture0|
| Base5 단위/회귀 검사 |318tests PASS|

`roundabout_remaining31_live_v119`는31/31승인·review0·failed0, wall183.281s였다.
이는 앞선 통신 중단을 제외한 보충31셀의 약5.91s/셀이며 전체5K/다른 동적 패밀리 ETA가 아니다.
초기 세션은 wall144.813s에서 중단됐으므로 그 세션을 무실패/완료 속도로 외삽하지 않는다.
통신 타임아웃의 내부 원인은 확정하지 않았고 새 세션에서는 같은 장면을 포함해 재현되지 않았다.
다시 발생하면 existing freshness/Park safety halt 계약을 유지한다.

산출물(root `roundabout_release_5k_v119/`):
- `roundabout_validation.json`: 전체5K 보존/계획 + 두 세션의 실제47로터리/4통합 근거.
- `combined_pilot_validation.json`, `validated_pilot51_ledger.jsonl`: 안전한51셀 선택, 원본 ledger 불변.
- `resume_verified.json`: 보충31셀 완료resume 검증.
- `roundabout_views.png`, `roundabout_views_cameras.png`:250Ego 관측 분포 및 실제 밀집 앞뒤 GT/4camera.
- `post_timeout_park.json`, `post_timeout_validation.json`: 통신 중단 후 정지/성공분 검증 근거.

raw는 앞20셀의 `roundabout_model_edge_live51_v119/`와 뒤31셀의 `roundabout_remaining31_live_v119/`를
함께 보존한다. CPU 변환은 `~/morai_cap/base5_v2_261008/roundabout_model_edge_infos_v119/`다.
이 pilot의51셀은 모두train이므로 성능 평가셋으로 제시하지 않는다. full5K train4380/val620은 별도다.

## 한계와 다음

- 이 검증은 수집 가능한 geometry/GT/가시성/운영 성공이지 모델 AP·tracking·planning 진입 성공 인증이 아니다.
- static Ego/NPC snapshot이며 본선 speed/spawn/interaction 분포를 재현한 traffic rollout이 아니다.
- 기하학적 annotation gap과 물리 collider는 다르다. raw annotation과 CollisionData 근거를 모두 보존한다.
- plane 미확인은 warning으로 남을 수 있으며 전지형/전자산 완전 접지를 증명하지 않는다.
- 변경한250셀의 link-disjoint 정책을 유지한다. 로터리 미션 링크는 현재 train이며 전체val620이
  로터리 일반화 시험을 대신하지 않는다. held-out 실제 주행 bag으로 학습 이후 평가해야 한다.
- 본선 차종/속도/생성 주기 공개 전 특정 숫자로 간격을 고정하지 않는다.
- 최종5K는 별도 사용자 승인 후 새 `collection_5k_roundabout_v119` output에서 시작한다.
