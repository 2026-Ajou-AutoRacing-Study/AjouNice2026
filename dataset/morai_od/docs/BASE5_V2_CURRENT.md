# Base5 독립 5K — CURRENT

2026-10-09 / v132 정지 자유 yaw·실측 배치 형상·음영 이동 방향 균형. v125/v124 운영·v123 구역별 구성 유지.
**Sunny13시 직접5K**, Base3 보강이 아닌 Base5-only single-frame OD 학습용이다.

## 최신 상태 — v132

2026-10-09 [학습split·가시성 해석·temporal 사용 결정](261009_base5_training_split_visibility_and_temporal_decisions.md):
현재30K 계획은유지하고완료후필요조건만보강한다. 생성train4255/val745·6환경동일split,
반복사용bag은dev-val/새독립주행은test다. 1771장 풍부한단서76.1%는분석proxy이며학습필터가아니다.
BabyStroller3의3/21은7parent×3component와camera대응모호성에따른보수적집계다.
모호한camera/LiDAR0 component 제외한계는남는다. single-frame/velocity off·본학습미시작을유지한다.

2026-10-09 Sunny13 본5K는 사용자가 시작해 수집 중이다. 이번 배치는 그대로 유지한다.
[현재 수집 스냅샷·라벨 결정·시간 개선 이력·종료 후 반복 비용 실험](261009_base5_collection_cost_and_next_experiments.md)이
아래 준비 당시의 미시작 표기보다 우선한다. 다음 환경 전에 상주 worker/준비 대기 중첩을 검증할 계획이며
이번 수집 중 runtime·manifest·설정·실행 계약을 바꾸지 않는다. 최적화는 아직 미구현이다.

현재 release는 `~/morai_cap/base5_v2_261008/final_appearance_5k_v132_release`다.
Sunny13 입력 `environment_suite/sunny13/manifest.json`, 새output `collection_5k_final_v132_sunny13`.
정지 실체·사람·동물·복합의 yaw를360° 연속 각도로, OBJ 자유70/도로30·일반 정적 차량 도로70/자유30으로
보완했다. 실제 이동·고주로/로터리 NPC·횡단 맥락은 경로 yaw를 유지한다. 음영24이동은 walk/run별 접근6/이탈6이다.
41종 승인 정지 native component의 parent envelope는 배치 간격·budget용이며 학습GT를 바꾸지 않는다.
범용 requested walking/running→actual standing에서도 적용되도록 수정했다. police1은 미검증 pivot fallback 유지다.
실제 누락 대조를 근거로 정적 alongside의 반려동물 동반 primary3셀만gap0.8–1.0m로 보완했다.
원래 bin과 yaw/좌우/자산 quota는 보존하며 다른 근접 조건은 유지한다.
2810셀 재생성/2190payload 보존,122종 primary 횟수·최소5·기존 family/거리/혼합·4822정적→178이동·6환경30K 유지.
계획 parent노출10738/64428은 **2026-10-09 v132 입력 한정**이다. 과거10079/10081표는 이력으로 보존한다.
전수5K/6환경 검사와 실제 MR·CPU/resume 최종 근거는
[v132 검증 및12카테고리 계약](261009_base5_final_appearance_v132.md)을 따른다.
대표55/55·CPU55/107GT·완료resume추가0/raw1858불변·45session Park·416tests PASS다.
가중 시간외삽9.635h/5K·운영10–13h, 저장외삽32.572GiB/5K·195.429GiB/30K다.
후처리·학습·복제공간은별도이며본수집누적rate/디스크로갱신한다.
구Sunny13763승인 및 v129/v130 raw/manifest를 보존하고 새runtime으로 강제resume하지 않는다.
본5K는 현재 수집 중이며 production E25/threshold/tracker/planning/localization은 미변경이다.

## 이전 상태 — v129 / 사용자 중단 후 새 입력

아래 v123/v125 '현재/본수집 미시작' 문구는 당시 이력이다. 사용자가 구Sunny13 수집을 중단했고
finished780/승인763/review17을 원본과 약3.3GiB tar.zst로 보존했다. 새 코드로 구output을 resume하지 않는다.
현재 release는 `~/morai_cap/base5_v2_261008/yaw_5k_v129_release`이며,
본수집 입력은 `environment_suite/sunny13/manifest.json`, 새output은 `collection_5k_yaw_v129_sunny13`다.
새5K는 생성·검증만 마쳤으며 사용자 재시작 전이다.

- 범용 정지pedestrian/AI 및 정적회피 사람·동물·복합 계열의 외관yaw:70%네축±15°/30%360°.
  기존 범용 실체장애물의 자유각도/길쭉한장애물bias는 유지하며 걷기·달리기·NPC경로yaw는 변경하지 않는다.
  yaw를 먼저 뽑고 회피 경계간격을 계산한다. 같은quota 재시도에서mode는 유지하고angle은 재표본화한다.
- 일반사람 정적미션alongside33셀만gap0.6–0.8m. Man2 기존gap0.30/0.45 누락·0.60/0.80 승인 대조를
  근거로 한 제한적 생성prior다. 내부collider 크기확인이나전체사람물리인증은 아니다.
- 변경1082셀/3918payload불변, 보호된 고주로/로터리/음영/이동payload 불변.
  5K 패밀리구성·122종primary횟수·정적4822→이동178·6환경30K계약은 유지한다.
  새계획parent노출10081/60486. [v129 스냅샷 및 전면검토](261008_base5_static_yaw_and_randomization_review.md)가
  아래v123의10079/60474표보다 우선하며 구표는 당시이력으로 보존한다.
- 전수5000 OFFLINE_PASS/6환경hash·split·정적우선 검증, 실제MR35/35 승인/부족0,
  CPU35프레임/91GT다. 초기실패/재배치/raw를보존하며5K전수성공/학습성능향상을주장하지않는다.
- 완료resume추가capture0/raw886파일불변·ledger/누적시간불변·10session Park·403tests PASS다.
  8075는새output의not_started/5000/6환경30000을표시하며수집폴더는아직없다.
- 실측외삽9.326h/5K, 운영10–13h/5K·60–78h/30K 유지. raw/QA저장외삽26.81GiB/160.86GiB,
  변환·복제·구백업은별도다. 누적rate와dashboard 저장계측으로갱신한다.
- production E25/threshold/tracker/planning/localization은미변경이다.

[변경·실험·한계](261008_base5_static_yaw_and_randomization_review.md),
[현재 런북](261007_base5_5k_capture_runbook.md)을 따른다. 이후 항목은 revision별 이력이다.

## 이전 준비 이력

v113의 5K 생성·전수 검사와 대표 live/resume/CPU 검증 이력은 보존한다.
새 v121 5K는 v119의 보호된 미션/근접을 유지하고 범용500셀을 음영 관측으로 재배정한다.
본수집 승인 전에는 실행하지 않는다.
사용자가 현재 MR 기본값과 수집 환경을 Sunny 13시로 확인했다.
아래 미시작/승인 대기 기록은 v108 사전 보고 시점이며 현재 수집 진행률을 뜻하지 않는다.
이번 환경 정정에서는 기존 수집을 중단·재시작하거나 본학습을 수행하지 않았다.

## 2026-10-08 대시보드 보강 / Sunny13 5K 본수집 준비

사용자가 50K가 아니라 이번 Sunny13 5K, 이후 기존6환경30K 유지임을 확인했다.
배치 입력 v123·캡처 runtime v125는 그대로다. 전수5000고유payload/오류0 및6환경manifest hash/
동일geometry/group/split/분리output을 확인하고, frozen입력/지도/원래자산목록/외부reference51파일/
v125코드 백업을 만들었다. 본수집 output6개 모두 미생성이며수집·학습은 시작하지 않았다.

읽기전용8075에 그래프와계획분모를보강했다. 누적승인/최근10분rate·패밀리·거리/방향·
부족사유·정적/이동검사순서·6환경진행을분리한다. journal은capture밖cache에만저장하며
UI/대시보드코드는캡처hash에포함되지않는다. 구v124output새runtime resume금지는유지한다.
예산10–13h/5K·60–78h/30K는소량표본외삽이며첫100–200셀rate로갱신한다.
[준비·검증·운영 경계](261008_base5_dashboard_and_launch_ready.md),
[실행 명령](261007_base5_5k_capture_runbook.md)을 따른다.

## 2026-10-08 13:05 KST 가벼운 객체 이동 진단 기록 — runtime v125 유지

사용자 요청으로 기존 bounded `object_position_samples`에 원문 velocity/heading과
source header 시각/seq를 추가했다. 요청 motion은 기존 runtime_actors에, 실제 변위 witness는
기존 motion_capture_actors에 유지한다. 단위/좌표계/heading/pivot 보정이나 속도 보간은 하지 않는다.
이는 **참고용 메타데이터이지 동기 속도 GT가 아니다.** Base5 4class/7D/velocity off는 그대로이며
속도·tracking·prediction 평가는 별도 실주행 rosbag으로 수행한다.

새 MR 정지 차량1+달리기 사람1 승인2/2, CPU2프레임/2GT, 완료resume raw68파일·event2로그
불변/추가capture0,375tests PASS다. trace를 위한 새 topic/추가 wait/촬영은 없다.
두 표본의 추가 JSON은 약48/95KiB이며 전체5K 비용 인증은 아니다.
v123의5000 배치/6환경/GT/거리/quota/노출 표 및 v124 보충/안전 gate는 변경하지 않았다.
본5K는 미시작이며 현재런북의fresh output을 쓴다. 구 v124 output을 새 코드로 resume하지 않는다.
아래 v124의33/33은 당시 코드의 근거이며 v125에서33셀을 다시 찍었다는 뜻이 아니다.
[구현·검증·저장 계약](261008_base5_motion_metadata_v125.md)을 참고한다.

## 2026-10-08 12:14 KST 본수집 직전 최종 확인 — v124 이력

v124 runtime/v123 입력으로 준비를 마쳤으며 **다음은 사용자 승인 후 Sunny13 본5K 시작**이다.
본수집은 아직 시작하지 않았다. 현재 코드로 전수5000 OFFLINE_PASS/오류0,
6환경 hash/entry/group/split/새output을 확인했다. 계획 노출10079/60474도 기존 동결 표와 같다.
새 소량 MR33/33(실제34capture/누락review1→대체1)·CPU33/72GT/오류0,
10session Park·완료resume1182raw불변/추가capture0, 기존보충2승인재개/추가capture0,
370tests PASS다. localhost8075는 현재 본수집 root의 `수집 전`을 표시한다.
가중 실측9.252h/5K, 운영예산10–13h/6환경60–78h를 유지한다. 여유약247.8GiB다.
전체5K 성공·음영24mover 전수인증/GT 실제coverage 완료는 아니다.
[최종 확인·산출물·운영 경계](261008_base5_prebulk_v124_final.md)를 먼저 읽는다.
캡처runtime/구성/GT/production은 바꾸지 않았고 대시보드의 미시작·중단 표시만 보완했다.

## 2026-10-08 완료 후 부족분 자동 보충 — runtime v124 우선

배치 입력은 아래 v123 release 그대로다. 5K 구성/6환경/GT/거리/계획 노출 표를 변경하지 않았다.
본5K는 아직 시작하지 않았다. 이번 변경은 완료된 campaign의 부족분을 명시적으로 보충하는 운영 기능이다.

- 본수집 완료 후 동일 명령에 `--resume --replenish`를 사용한다. 미완료/안전 halt는 먼저 보충하지 않는다.
- 기존 승인 셀/raw와 원본 manifest는 보존한다. 실패한 원래 셀만 동일 primary/spec/거리/방향/
  phase/motion 요청/split/group/config로 새 배치를 만들며 새 raw에서도 기존 gate를 통과해야 한다.
- 기본 보충2회차(셀당 누적 상한), 최대3회차를 명시적으로 선택한다. 같은 명령을 반복해 상한을 초기화하지 않는다.
  각 회차 내부에도 기존 bounded quality recovery가 있으므로 회차 수가 실제 촬영 횟수는 아니다.
- `selected_ledger.jsonl`에 기존 승인+보충 승인을 원래 셀당1개로 통합한다. quota/부족 manifest/
  dashboard도 갱신한다. `replenishment/state.json`과 회차별 raw/실패 근거는 보존한다.
- 보충의 안전한 PAUSE→같은 보충 명령 재개를 지원한다. 보충 이후 ordinary `--resume`은 검증만 한다.
  완료 ledger가 보충 전 상태로 돌아가지 않으며, 완료 child/성공 셀을 다시 촬영하지 않는다.
- CollisionData 접촉의 안전 종료 근거가 있는 실패는 부족분이다. 통신/Park/정지 불확실은 여전히 halt다.
  모든 부족분의 자동 해결/5000장 보장은 아니다. 상한 후 남은 부족분은 보고하고 따로 판단한다.
- runtime/hash가 바뀌었으므로 구 v123 pilot output은 새 코드로 resume하지 않는다. 과거 근거는 보존하고
  본수집은 v124 코드에서 fresh output으로 시작한다. 이미 수집을 시작했다면 원래 revision을 유지한다.
- 기존 `base5_replenishment.py`는 소량 visibility-only 독립 pilot 도구로 보존한다.
  현재 5K 통합 보충 구현은 `base5_campaign_replenishment.py`와 batch CLI다.

[실제 MR·보존·CPU·한계 근거](261008_base5_completed_replenishment_evidence.md),
[실행 명령](261007_base5_5k_capture_runbook.md#완료-후-부족분-보충--v124)을 따른다.

## 2026-10-08 구역별 5K + 동일 구성 6환경 — v123 배치 유지

현재 release는 `~/morai_cap/base5_v2_261008/regions_5k_v123_release/`다.
본수집 입력은 `environment_suite/sunny13/manifest.json`, 정적4822셀 우선/이동178셀 후순이다.
아래 v122 이하 경로·당시 비율은 이력이며 새 runtime으로 구 output을 resume하지 않는다.
객체군별5K/30K 계획 노출 표는 [v123 동결 스냅샷](261008_base5_region_release_evidence.md#객체군별-계획-노출--v123-동결-스냅샷)에 둔다.
기준은2026-10-08 11:05:24 KST/커밋`c5ff1baf`/manifest hash `9e1e3816…`다.
이후구성이변경되면새스냅샷을계산하고이참조를갱신한다. 구버전계획치를현재값이나실제취득GT수로인용하지않는다.

| 배치 카테고리 | 범용 | 미션 | 합계 |
|---|---:|---:|---:|
| 고주로 |740|600|1340|
| 로터리 |0|350|350|
| 기타 차량 |80|50|130|
| 장애물 |770|600|1370|
| 사람·동물·복합 |830|250|1080|
| 독립 음영 관측 |—|—|500|
| 빈 배경 |230|0|230|
| 전체 | | |5000|

- 고주로·로터리의 모든 신규 actor는 일반 차량43종만 사용한다. 범용 보조 객체에도 적용한다.
  이는 생성 prior이지 실주행 클래스 마스킹/GT 삭제가 아니다. 비차량 범용 Ego는 그 구역과 음영 밖이다.
  actor 실제 XY도 MGeo corridor proxy로 따로 검사한다. 공식 출제 구역의 완전한 polygon 인증은 아니다.
- 로터리 범용100셀을 미션으로 흡수해 접근70(이동관측30/대기40), 진입80, 원형130, 탈출70이다.
  NPC 공식5link 경로/앞뒤 밀집·Ego road support·정적 Park 계약을 유지한다.
- 범용 positive2420셀의 단일/일반/밀집30/50/20 크기 예산과 최대8개를 유지한다.
  사람 계열830셀의 front/left/right/rear50/20/20/10 유지, 다른 허용 구역에서는 혼합이 가능하다.
  혼합 사람 계열 중270셀은 보조 실체 장애물 최소1개를 조건으로 넣어 제거된 장애물 노출을 보충한다.
  모든 사람 장면에 장애물을 강제하지 않으며 주 객체·희귀 자산을 교체하지 않는다.
- 거리 deck 및 경계 간격 기반 근접 quota를 유지한다. 긴 버스의 불가능한 전방6–10m ring2개는
  같은 family/front의 작은 차량 ring과 교환해 자산·거리 총량 모두 보존한다.
- 음영500개 scenario geometry는 v122와 동일하다. family를 독립 `shadow`로 명시해 고주로 장부와 섞지 않는다.
  코드상 mission=false는 기존 이동 adapter 호환용이며 일반 고주로 제한의 대상이 아니다.
- 122종을 primary로 최소5회 배치한다. 계획한 parent 노출은5K당10079개이며 실제 positive GT 개수와 다르다.
  복합 객체는 여러 native component를 만들고 보이지 않는 객체는 유지 조건을 만족하지 않을 수 있다.
- 첫 live검증의 고주로 완전가림 부족분을 분석해 generic highway에 초기 footprint 각도 proxy를 추가했다.
  주 차량보다 완전히 가까운 보조 차량이 주 차량 수평각의95% 이상을 덮으면 초기 재생성한다.
  부분 가림은 유지하며 실제3D가시성/여러 blocker의 합성 가림은 여전히 native 센서검사 대상이다.
- sunny/foggy ×11/13/15의6manifest를 준비했다. 첫 Sunny13의5K+다른5환경25K=30K 관측이다.
  독립 geometry는5000개이며 모든 날씨 변형이 같은 `scene_group_id`/split을 유지한다.
  변환 시 `--environment-tag`로 token만 구분하고 merge에서 split leakage/중복을 거부한다.
  manifest는 MR 날씨/시간을 적용하지 않는다. 환경별 UI 확인·새 output이 필수다.
- 수집/GT 정책은 native Instance/4class/7D/velocity off/ROI±100m·횡±30m 그대로다.
  정지/Park·CollisionData·심한 높이 이상·가시성·bounded 대체/부족분·resume 안전 계약도 유지한다.

실제 검증 결과·비용·실패 근거는 [v123 EVIDENCE](261008_base5_region_release_evidence.md),
수집/대시보드/환경별 변환 명령은 [운영 런북](261007_base5_5k_capture_runbook.md)을 따른다.
최종live33/33(실제34capture/부족분0), CPU33/72GT·오류0, 완료resume1182raw불변/추가capture0,
355tests PASS다. 실측5K외삽9.24h, 운영예산10–13h/6환경60–78h이며누적rate로갱신한다.
본5K 자동 시작/모델 학습/production 변경은 이번 준비 작업에 포함하지 않는다.

## 2026-10-08 음영 밖→내부 사전 관측 — v122 이력

현재 `~/morai_cap/base5_v2_261008/shadow_approach_5k_v122_b/scenarios_5k/manifest.json`이다.
아래 v121 입구120/내부260/출구120 및 해당 명령은 이력이다.
음영500셀을 바깥 먼 접근60/가까운 접근60/입구 직전30/진입 직후90/내부190/출구70으로 나눈다.
바깥 Ego는 입구0.5–50m 전이며145positive 모두 내부 전방 primary를 joint placement로 확보한다.
일반/정적 차량54primary도 포함된다. Ego/대상 거리 합산이100m를 넘지 않도록 실제 ROI/bin을 검사한다.
정적 primary 차로80/20%, 좌우 엇갈림93셀·내부20–80% 집중(정적294/456)과 입구 대상 예외를 함께 둔다.
이는 soft prior이며 공식 출제 확률/실제 회피 제어 결과가 아니다.
476정적 셀만 바뀌고 기존4500+음영24mover=4524entry 불변이다. 전체5K/122종 최소4/178이동은 유지한다.
전수 OFFLINE_PASS/오류0·사전 선정live20/20(23capture, 실패3원본 보존), 바깥 양성9/9 front camera 확인,
CPU20/29GT·완료resume 추가capture0이다. 기존 running1셀 ground deficit는 이번 정적검증으로 해결하지 않았다.
현재 증거/한계: [v122 EVIDENCE](261008_base5_shadow_approach_evidence.md).
새 runtime으로 구 v121 output을 resume하지 않는다. 본5K/학습은 미시작, 사용자 승인 후 새 output이다.

## 2026-10-08 음영 구간 관측 — v121 이력 / 이동 준비 계약 유지

사용자 전달10/06 공식 답변: 고주로에는 정차 차량/정적 장애물을 배치하지 않지만 터널은
현재 F2 객체 전체가 등장할 수 있다. 동적 객체를 기다렸다 재출발하는 것도 페널티가 없다.
Vehicle/Spawn에는 activation distance가 없으며 MGeo 역주행은 불가능하다.
개별 NPC waypoint 차로 침범은 가능하다. 기존 rigid 정적 자산 이동/G80 열린 문 제외 정책은 유지한다.

전체5000 중 범용positive480/배경20을 입구120/내부260/출구120으로 교체한다.
기존1750미션·로터리 범용100·초근접 quota는 보존하고 나머지4500 entry/source 참조는 불변이다.
별도 공간 overlay이므로 기존 family 장부의 총량은 유지된다. 실질 구성은 범용positive2520/
범용배경230/기존미션1750/음영positive480/음영배경20이다. Base3 데이터를 섞지 않는다.
음영 primary122종 최소1, 전체122종 최소4를 유지한다. 거리3–10/10–30/30–60/60–100m는
40/240/160/40 positive셀이다. 한/두/세 객체208/185/87셀과 빈 배경20셀이다.

샘플 shadedArea와 실제 MGeo 경로의 교집합 약97m를 관측 anchor로 쓴다.
내부 primary가 범위를 벗어나면 초기 재생성한다. 실제 어둠은 native 카메라에서도 따로 확인한다.
차량은 정지 Park 외관 snapshot으로 반대 방향/비스듬한 차로 침범 자세도 포함한다.
실제 NPC 끼어들기 제어 성공이나 traffic rollout을 검증했다는 뜻은 아니다.
사람·동물·반려동물 동반24셀은 실제 이동 외관이며 나머지 음영476셀은 정지 배치다.
전체 정적4822/이동178(기존 사거리154+음영24)이다. AI 보행자는 카탈로그에서 제외하지 않고
미검증 이동 quota에만 사용하지 않는다. Dog1/Dog2/Deer1 모두 실제 이동 시험 대상이다.

사거리 수동 높이 lookup/두 waypoint 인증을 터널에 재사용하지 않는다. 음영 mover만 독립
empty terrain+static reference+10–12m target witness로 경로를 준비한다. 부족하면 조건부departure
16m/12m 상보 관측을 추가하고 매 단계 전체 경로가 확인되면 멈춘다. 도로 중앙에 배치하고
짧은 접근 방향 이동을 사용하며 plane/hull/실제 이동/Z/CollisionData gate를 유지한다.
정적476셀에는 추가 지면 촬영을 하지 않는다. GT/raw는 변경하지 않고 동일 Instance component를 쓴다.
기존 ROI±100m/횡±30m·lidar1_camera0·4class·Ego 정지/Park/실제 무접촉 계약은 유지한다.

현재 생성본: `~/morai_cap/base5_v2_261008/shadow_release_5k_v121/scenarios_5k/manifest.json`.
전수5000고유payload OFFLINE_PASS. 새runtime으로 구pilot/output resume하지 않는다.
실제 센서/CPU/resume 검증 결과와 남은 한계는
[음영 EVIDENCE](261008_base5_shadow_observation_evidence.md)를 따른다. 본5K는 아직 미시작/승인 대기다.
실제 정적25/25·이동6/7 승인, CPU31프레임/54GT 및 최종runtime resume481raw불변/추가capture0,
334tests PASS다. Woman3 running1셀은 지면 근거 부족으로 남았으며 최종32셀검증은1누락 FAIL이다.
floating이나 모델 탐지 실패로 단정하지 않는다. collector는 bounded 대체 후 부족분/다음셀로 처리한다.
따라서 모든동적배치 해결/5K자동승인으로 해석하지 않는다. 부족분은 본수집 뒤 같은-quota 보충 대상이다.
음영 추가 준비를 포함한 운영 예산은10–13h다. 첫100–200셀 실측으로 갱신하며 성공률/ETA 보장이 아니다.

## 2026-10-08 로터리 미션 실제 경로 관측 + 정적 NPC Park — v119 유지

사용자가 제공한 MORAI 답변: 본선의 지정 경로·차선변경 비활성화·회전 궤적은 샘플과 같은 방식이다.
생성 주기·속도·차량 수는 미공개/샘플과 다를 수 있으며 Ego와 상호작용해 감속·정차할 수 있다.
따라서 고정5초×20km/h로 본선 간격을 역산하지 않는다. 평가 locked bag은 생성기 설계에 사용하지 않았다.

로터리 미션250셀은 접근20/진입대기30/진입60/원형 주행90/탈출50으로 배정한다.
Ego는 실제 MGeo 미션 경로를 촘촘히 나눈 구간 안에서 관측한다. NPC는 샘플 spawn UID4의
`144→133→134→135→102` 경로 시작 point26부터 삭제 point9까지 약92.81m 안에만 둔다.
과거 실험용848/845/843 연장 경로를 미션 NPC 경로로 사용하지 않는다.
NPC XY/yaw 임의 jitter는 없고 Ego만 road-support 안에서 횡±0.30m/yaw±6°를 허용한다.

원형 주행90셀은 반드시 Ego 앞뒤 NPC 두 대를 넣는다. 경계 간격 밀집0.5–2.5m/보통4–10m/
여유10–22m를27/36/27셀로 배정한다. 이는 설계상 OBB 간격이며 물리 collider 간격이 아니다.
접근·진입·탈출의 밀집 등급은 교차 경로상 배치 목표이며 동일 간격을 보장하지 않는다.
실제 raw Instance positive에서 진입·원형 주행의 두 차량이 안 보이거나 원형 주행의 앞뒤 조건이
사라지면 review/기존 bounded same-quota replacement로 처리한다. 실제 간격도 목표 범위±0.30m로 검사한다.
GT를 만들어서 채우지 않는다.

전체 배치 수는1대30셀/2대155셀/3대65셀(합계535대)이며 일반 차량43종을 primary5–6회씩 순환한다.
본선 로터리 차종은 비공개이므로 K5 한 종류로 고정하지 않는다. 이는 본선에43종 모두 나온다는 뜻이 아니다.
Base5 전체 비율·정적4846/이동154·122종 primary 최소4 및 나머지4750셀의 geometry는 보존한다.
로터리 범용100셀도 배치 위치/거리/자산은 유지한다.

실제 검증에서 GV80이 초기 속도0/D 기어인데도 약5m 움직였다. 같은 위치 A/B에서 저장된
바퀴/엔진 회전 초기값만 제거해도 해결되지 않았고 Park 기어 초기화만으로 pivot 이동이 약5mm로 줄었다.
모든 정적 vehicleList NPC의 initPosition/inputData/gearbox 세 기어 값을 Park로 일치시켰다.
나머지4750 중2314셀은 control-only 갱신,2436셀은 파일 참조까지 동일하다. 이동 actor는 Park로 바꾸지 않는다.
캡처 전후 기존 Object snapshot에서 계획 pivot XY 대비25cm 초과 이동/정보 누락은 review다.
추가 대기나 촬영은 없다. 과거 v118의 초기47승인 중3셀은 이 엄격한 후속 검사에서 실패했으므로
원본/실패 근거만 보존하고 최종 검증본으로 사용하지 않는다.

v119 당시 release: `~/morai_cap/base5_v2_261008/roundabout_release_5k_v119/scenarios_5k/manifest.json`.
`revise_base5_roundabout_mission.py`가 v117 동결본에서250셀 geometry/정적 NPC 기어를 fresh output에 갱신한다.
기본 sunny13/호환sunny11 config는 과거 재현을 위해 덮어쓰지 않으며 v118 경로 plan/v119 Park config는 release에 포함된다.
옵션없는 기본 생성기를 바로 실행해서 현재 release와 같은 계획을 만든다고 해석하지 않는다.
대표 실제 검증·CPU 변환·resume·한계는 [로터리 EVIDENCE](261008_base5_roundabout_route_mission_evidence.md)를 따른다.
최종 사전 선정51셀(로터리47+통합4)을 실제 캡처/검사했고 로터리43종·105차량 모두 positive,
원형 앞뒤15/15·최종51셀 무접촉/Park를 확인했다. 최대 정지 NPC pivot 오차1.64cm다.
통신 timeout 한 번의 halted 원본을 보존하고 안전한20셀+새output31셀로 원래 검증 목록을 완성했다.
CPU51프레임/115GT, 완료resume1111raw불변·추가capture0,318tests PASS다. 실패 없던 세션으로 포장하지 않는다.
구 v117/v118 output을 현재 runtime으로 resume하지 않는다. 본5K는 새output/승인 후 시작한다.
생성기/QA 보강이며 모델 학습·탐지성능·합류 제어 성공을 검증한 실험은 아니다.

## 2026-10-08 출발→도착 2점·양방향 균형 — v117 이동 계약 유지

사용자 결정에 따라 실제 이동 경로는 정확히 두 waypoint로 생성한다.
원본 샘플 Man1도 두 점이며 과거 bag의 Z가 끝점 선형 보간과 일치했지만,
본선 모든 경로나 자동 지면 추종을 증명한 것은 아니다. 촘촘한 지면 추종 경로는 기본값에서 제거했다.
수동 저장 높이를 초기 끝점 후보에 쓰고, 중간 Z는 두 끝점의 직선으로 결정한다.

짧던 CW000002 사람 경로는 같은 구역 Ped_Pet1의 저장 끝점과 겹치는 구간 높이 차이로
후보를 만들었다. 다른 자산 높이를 그대로 정답으로 인증하지 않고 28종×두 구역×양방향을 재검증했다.
112개 승인용 캡처 PASS, 실제127회 시도 중 Dog1의 LiDAR 3D annotation 누락1건은 review로 보존했다.
해당 Dog1은 같은 속도/경로의 더 가까운 관측 위치에서 재검증했다. GT를 임의 생성하거나 실패를 합격 처리하지 않았다.
측정 가능한 annotation 하단↔local-plane 최대절대 gap7.40cm, 최대 연속Z step0.141mm다.
이는 시험 경로의 운영 QA이며 모든 지형의 물리적 접지 인증은 아니다.

미션 실제 이동 primary의 방향을 횡단보도×걷기/달리기×사람/희귀 그룹별로 먼저 고정한다.
같은-quota 재시도에서도 방향을 바꾸지 않는다. 일반 사람 primary 계획 수는 다음과 같다.

| 구역 | 걷기 +/− | 달리기 +/− |
|---|---:|---:|
| 첫 CW000002 |20/20|15/14|
| 뒤 CW000000 |19/19|13/13|

+/−는 저장 lookup 횡단축 기준이며 Ego 전후방 의미가 아니다. 홀수 그룹은1개 차이까지 허용한다.
154개 이동 장면 모두 두 waypoint를 사용한다. 정적4846/전체5000/122종 primary 최소4,
패밀리·거리·근접·관측 phase 계약은 유지했다. 구 v116 방향 편향 수치는 이력이다.

최신 경로(root `~/morai_cap/base5_v2_261008/`):
- `endpoint_linear_release56_v117/height_lookup_live.json`
- `endpoint_linear_final_5k_v117/scenarios_5k/manifest.json`
- `endpoint_linear_final_5k_v117/pilot_manifest.json` (대표18셀)

일반 생성기 대표18셀18승인/부족분0(실제21capture/184.685s), partial/완료resume PASS다.
정적 근접 Man1 누락3review는 같은-quota bounded replacement로 보충했고 raw를 보존했다.
CPU18프레임/28GT 변환,300tests PASS다. 자세한 오류·제약은 [v117 근거](261008_base5_intersection_ground_evidence.md)를 따른다.
구 v116 manifest/raw는 보존하되 새 코드로 기존 output을 resume하지 않는다.
본5K 수집/학습은 미시작이며 사용자 승인 뒤 새 output을 사용한다.

### 본수집 전 최종 재확인 — 2026-10-08

판정: **현재 수집 범위에서 높이 보완과 제외 경로는 구현·검증됐고, 추가 높이 실험은 시작 조건이 아니다.**
모든 지형에서 완전 접지를 보장하는 수정은 아니다. 작은 차이/불확실 지면은 warning,
신뢰 가능한 지면 대비±40cm 초과·수신 간격≤0.2초에서 Z step10cm 초과·실제 접촉은 실패로 남긴다.
이미 취득한 원본으로 검사하며 지면 검사를 위한 추가 캡처는 하지 않는다.

- 새5K 전수 재감사: OFFLINE_PASS/5000고유payload/오류0.
- runtime 분류: 정적4846개 전부 lightweight-static,
  이동154장면의184mover 전부 LIVE lookup/정확히2waypoint/수동경로 QA 적용.
  이release에서 구 고비용 독립지면 준비로 떨어지는 mover0이다.
- 5000개 source scenario/metadata와25개 runtime hash로 본수집 계약을 read-only 구성했다.
  config canonical-content hash·generator·lookup/source/report·직전 live runtime hash 일치.
- 기존 높이/지면 회귀78개와 endpoint7개, 총85개 재실행 PASS.
  synthetic gap+7.4/+30cm는 warning,±41cm는 severe review로 분류됨을 재확인했다.
  이 synthetic 확인은 추가 MORAI 물리실험이 아니다.
- 현재 ROS6초 읽기: Ego50.16/Object49.92/Collision50.24Hz,
  non-Ego contact UID없음, Ego 속도≈0. EventCmd service와ScenarioLoad/SaveSensorData/ctrl subscriber 존재.
  service 응답·센서 저장 자체는 직전18셀 live에서 확인했으며 이번 read-only 검사에서는 호출하지 않았다.
- 디스크 여유약252GiB. 직전21시도/18승인 raw비용의 단순5K외삽약29.7GiB는 조건부이며
  추가 실패·변환본까지 고려해도 현재 여유로 시작을 막을 근거는 없다.
- 본수집 output은 아직 없으며 수집기 프로세스도 미실행이다.
  남아 있는8081 dashboard는 과거v45 경로다. 새 수집은 런북의새경로/8075 dashboard를 사용한다.

남은 시작 절차는 MR UI에서Capture Mode·Sunny13시·대회센서 설정 유지 확인 후 사용자 본수집 승인이다.
이UI조건을 read-only ROS검사로 자동 인증했다고 해석하지 않는다.
첫100–200셀의 실제 누적rate/부족분을 확인하고, 완료 뒤122종 actual visible-primary coverage를 감사한다.
후자의 실패분 보충·coverage는 본수집 후 할 일이지 모든5K를 미리 live촬영해야 한다는 뜻이 아니다.
예상은 정적약9.37h+동적22–26분=약9.73–9.81h, 운영 여유10–12h다.
구정적40셀의family별비용과현재동적실측을결합한조건부예상이며장기rate보장은아니다.

## 2026-10-08 수동 waypoint 높이 보간 — v116 이력

사용자 보정본10개/49waypoint를 원본·EgoNetwork와 함께 hash 검증/백업하고 MR에서 해당 파일만 정리했다.
`2_MN.json`은 없어 missing으로 기록했다. 실제 수동 XY를 횡단보도 축에 투영해 자산별 경로 높이를 보간한다.
world actor Z와 내부 waypoint Y를 구분한다. 길이 밖·실제 경로에서1m 이상은 외삽하지 않는다.
단일 선의 지면 mesh 인증이나 다른 구역 공통 ground로 해석하지 않는다.

5자산×2횡단보도×양방향20회, running4회, 평행±0.5m8회, 근접8회와
다른 일반 사람23종×두 횡단보도×양방향92회, 총132회 모두 PASS다.
실제 Object Z와 경로 높이 최대차3.415mm, 원본 annotation 하단/측정 지면 최대절대 gap8.583cm다.
이는 Object가 지정 경로를 따르고 관측된 바닥과 대체로 정합한 근거이지 모든 지형의 접촉 인증은 아니다.
평행 띠는 최대1m로 제한하고 실제 추가 live 검증은±0.5m 대표 사람 경로다. 원본 point·GT는 보존했다.
추가 empty-terrain/static-reference/height recapture0이며 raw 자체 ground 검사만 사용한다.
근거 부족/작은 오차는 warning, 신뢰 local-plane ±40cm 초과·Z 불연속·domain 밖은 review다.
Ego road support/실제 정지·CollisionData·실제 움직임/GT/가시성 검사는 유지한다.

자산 profile의 pending lookup은 calibration_only에서만 허용한다. 실제 양방향 근거와 source hash가
있는 profile만 LIVE_VALIDATED로 발행한다. 다른 사람23종도 각각 양방향 ground/Z/GT/가시성 검사를 통과했다.
이 검사는23종 모두의 이동/정지 외관 차이를 비교한 실험은 아니다.
신규 generator opt-in `dynamic_region=intersection-only-v116`은 범용 이동만 standing으로 바꾸며
자산·family/거리 quota는 유지한다. 새 release의 `require_covered_moving_paths=true`는 lookup 미포함/범위 밖을
초기 생성에서 같은 설계 셀로 다시 시도한다. capture 후 고비용 terrain 준비로 넘어가지 않는다.
legacy/옵션 없는 config는 기존 엄격한 준비를 유지한다. all-editor movers만 quality loop·공유ROS batch에서
terrain/static-reference 준비를 생략하며 원본 검사·실제 이동·충돌·Ego 정지는 유지한다.

새 전체5K OFFLINE 생성: 정적4846/실제 이동154장면,122종 primary 최소4회다.
이동은 신호등 사거리 관측250장면 중154개이며 일반 차량·고정 외관/standing 관측도 이250개에 포함된다.
이동 가능한 수집 대상은 일반 사람24+동물3+반려동물 동반1=28종이다. AI police1/NCAP는 standing이다.
일반 mixed/정적 미션의 자산을 제거하지 않았고 family 총량·혼합/근접 quota·Sunny13시는 유지했다.
첫 횡단보도 사람 수동 경로의+방향 끝이 짧아 running 역방향 사람 배치가 불가능하다.
두 횡단보도 전체는 양방향을 포함하지만 방향 완전 균형을 주장하지 않는다. 경로 밖 높이 외삽으로 채우지 않는다.

고정 경로:
`~/morai_cap/base5_v2_261008/editor_ground_release56_v116/height_lookup_live.json`(28종×두 구역=56profile),
`~/morai_cap/base5_v2_261008/editor_ground_final_5k_v116/scenarios_5k/manifest.json`,
같은 상위 폴더의 `offline_audit.json`/`pilot_manifest.json`/`config.json`.
기존 default config·구manifest/output은 덮어쓰지 않았다. 본5K 수집/학습은 미시작이다.
대표 일반 생성기 batch·resume 결과는 [v116 근거](261008_base5_intersection_ground_evidence.md)를 따른다.
대표12셀12승인/15capture·실패 재배치 포함134.079s, partial/완료resume PASS, 추가촬영0이다.
CPU변환12프레임(train7/val5)·21GT, regression293개 PASS다.
fresh transport의 정적 초근접 Man1 로드 누락은 원본review로 분리하고 같은quota 재배치로 보충했다.
이동누락/통신이상은 완화하지 않았다. 기존fail raw와 원본annotation은 보존했다.
정적4171/이동829 및 이전 ETA는 과거manifest 수량이다.

## 2026-10-08 이동 캡처 사거리 제한·수동 높이 보정 — 보정 전 설계 이력

사용자 방향: 이동 외관이 필요한 사람/동물/복합체만 우선 신호등 사거리 두 횡단보도에서 이동 캡처한다.
범용/정적 회피에서는 정지 배치하며, NPC 및 검증된 rigid 외관도 정지 캡처 우선이다.
GPS 음영 저격은 조명/배경을 고려해 후속 별도 설계한다. 공식 출제 구역 제한으로 해석하지 않는다.
현재 런타임/전체5K manifest 변경은 아직 없고 v115 동적 엄격 검사를 유지한다.
Woman2/Dog1/Dog2/Deer1/Ped_Pet1×두 횡단보도=10개 수동 보정용 constant-Z 경로를
MR에 `0_<번호>_<자산>_<횡단보도>.json`+EgoNetwork sidecar로 설치했다. 실제 시나리오 로드는 하지 않았다.
사용자가 UI로 출발/전체 waypoint 접지 후 `1.json`…`10.json`으로 저장하면 원본과 함께 backup하고
자산×경로별 bounded 높이 보간표를 구축·live 검증한다. 아직 lookup 합격/시간 단축은 주장하지 않는다.
다른 pivot에 무조건 공유하거나 경로 밖으로 외삽하지 않는다. 과거4171/829 수량·ETA는 이전 manifest 이력이다.
[수동 작업·정책·후속 검증](261008_base5_intersection_manual_ground_plan.md)이 다음 단계다.

## 정적 지면 QA 경량화와 실제 비용 — v115 유지 / 과거40셀 비용

사용자 승인에 따라 정적 장면은 이미 취득한 LiDAR의 local ground 검사만 사용한다.
`static-single-capture-advisory-v115`가 명시된 장면에만 적용하며, 모든 actor가 static/standing이어야 한다.
plane 미확인·중간 정도 annotation 높이 오차는 warning으로 보존한다. 접지 인증으로 표시하지 않는다.
신뢰할 수 있는 local plane 대비 annotation 하단이 +0.40m 초과/−0.40m 미만이면 review다.
정적 지면 확인만을 위한 Ego 근접 이동·empty terrain·높이 재촬영은 하지 않는다.
심각한 높이 실패는 부족분/기존 bounded same-quota replacement로 처리한다.
가시성·GT 누락·Ego 최종 road footprint/정지·CollisionData·raw/provenance 계약은 유지한다.
v116 editor-path proof가 없는 이동 actor/calibration은 기존 엄격한 독립 ground/Z 경로를 유지한다.
v116 검증된 경로는 위 수동 높이 정책으로 대체한다.
원본 센서/GT는 변경하지 않고 core ground 결과와 warning/최종 review를 따로 저장한다.

고정seed261007115의 family-stratified 신규40장면: 39승인/1부족분,
실제43촬영(재배치3회 포함), parent wall271.322초, 승인당6.957초.
부족분은 고주로 후방6–10m 혼합의 `population_visible_actor_deficit`이며 실패를 성공으로 세지 않았다.
정적 ground 추가촬영0, 이동/terrain calibration 촬영0, 안전 미확인 session0이다.
plane 미확인36건은 actor×capture warning 수이며 36장 또는 부유36건이라는 뜻이 아니다.
기존 실제 Boy1 부유+57.812cm 근거는 새 정책에서도 severe review로 남았다.
완료resume: sensor935파일의 hash/inode/mtime·선택ledger·누적wall 불변, 추가촬영0.
CPU 변환39장(train32/val7), Vehicle37/Pedestrian9/Obstacle22/Animal4=72GT; raw 보존.
Base5 단위검사272개PASS다. production 모델/추론/threshold·planning/control/localization은 미변경이다.

정적4171장 조건부 외삽은 family 가중8.115h(단순8.060h)다.
이전 v107 정적10장 표본의13.4h보다 짧지만 표본/생성기/QA 계약이 다르므로 순수 인과 A/B 개선율은 아니다.
운영 예상은 여유 포함 약8–10h로 보며 실제 bulk 누적rate로 갱신한다. 시간 보장이 아니다.
이 값은 이동829장을 포함한 전체5K 시간이 아니다. 동적 준비비용은 이번에 새로 측정하지 않았다.
근거: `~/morai_cap/base5_v2_261007/static_light_v115/`의 `static_cost.json`,
`pilot40_offline_audit.json`, `complete_resume_verification.json`, `collection40/`.
[세부 근거](261007_base5_static_height_evidence.md#정적-qa-경량화와-40장면-실측--v115).
다음은 새 v115 전체5K manifest/전수 audit 준비 및 사용자 본수집 승인이다. 본수집/학습은 미시작이다.

## Ego 도로 지지와 실제 정지 — v114 유지

기존 `b5v2_01502`(manifest index1258)의 실패를 재현했다. 큰 회피 객체의 가상 우회 시점에서
Ego가 도로 anchor 기준 옆으로 약7.8m 이동했지만 원래 anchor의 Z/link를 계속 사용했다.
실제 최종 XY는 가까운 물리 MGeo road centerline에서 약4.45m 떨어진 roadside estimate였다.
객체를 제거하거나 초기 기어만 Park로 바꿔도 수평 이동17–29cm·Z 변동45–62cm가 남았다.
같은 yaw에서 Ego를 도로 기준점으로 옮기면 객체 유무 모두 안정됐다.
도로 밖 배치에 따른 물리적 지지 불안정이 확인됐으며, 상세 지형/서스펜션 기전까지 확정하지는 않았다.

- 생성기: 최종 Ego rear-axle XY와 회전된 footprint의 도로 지지를 검사한다.
  `roadside_estimate`는 passive actor 높이 후보로는 남지만 Ego의 도로 지지로 인정하지 않는다.
- 정적 미션의 가상 우회 Ego가 벗어나면 Ego와 primary를 같은 XY만큼 함께 옮겨
  상대 위치·yaw·BBox 간격을 유지한다. 지지되지 않는 yaw는 같은 자산/phase/quota로 bounded retry한다.
  최종 Ego XY에서 높이를 다시 구하고 actor 높이도 기존 최종 XY 정책으로 독립 초기화한다.
  실제 controller가 주행한 회피 궤적이나 객체 접지 인증으로 해석하지 않는다.
- 수집기: 실제 scenario의 Ego XY/yaw를 로드 전에 재검사한다.
  Park ACK 이후0.3초 실제 정지 witness를 확인한다(XY/Z span와 XY/수직 속도 각각<0.02).
  최대4초만 기다리고 캡처 직전에 재확인한다. 드리프트8cm·CollisionData 계약은 완화하지 않는다.
- 실패 좌표·속도·seq/source stamp·서비스 응답·원본 hash를 저장한다. 통신/정지 불확실은 halt다.
  CollisionData 접촉은 기존 physical-collision 복구 분류를 유지한다.
- 세션 계약v7은 도로/경계 지도와 관련 runtime hash도 고정한다. 기존 raw/manifest/output은 보존한다.
  새 코드로 구 output에 resume하지 않는다. v113 재현 revision은 `10578aa8`이다.

기존 5K의 설계 셀을 파일 저장/본수집 없이 전수 재구성한 결과5000/5000 생성 가능,
기존family와122종 primary 최소4회를 유지했다. 회피 쌍390개를 도로 지지 위치로 옮겼다.
새 5K 동결 manifest를 생성한 결과나 전체 지형/가시성 live 인증은 아니다.
실제 A/B·수정본 캡처·한계는
[Ego 드리프트 근거](261007_base5_static_height_evidence.md#ego-드리프트-원인-분리와-보완--v114)를 따른다.
다음 실행 기준은 위 v115다. 본수집·학습은 이번 작업에서 시작하지 않았다.

## 최종 객체 XY 기준 초기 높이 — v113 이력

일부 좌우/혼합 배치에서 XY를 옮기고도 이전 anchor Z를 사용하던 경로를 보완했다.
정적/standing actor는 최종 XY에서 실제 MGeo 도로 선분 XYZ를 보간하고,
그 link에 연결된 차선 경계가 있을 때만 횡경사를 보간한 뒤 자산 pivot 높이를 더한다.
경로용 `lazy_init`/lane-change 가상 link 208개는 지면 근거에서 제외한다.
전역 삼각분할·일괄 +Z 여유·raw cloud/annotation 보정은 하지 않는다.
지도는 지형 mesh/collider가 아니며 roadside/unsupported의 실제 접지는 계속 검사한다.
F2 저장 pivot의 동일 계산 방식 초기 후보는 81/122종이며 접지 인증 수가 아니다.

새 동결 manifest:
`~/morai_cap/base5_v2_261007/final_5k_final_xy_ground_v113_final/manifest.json`.
SHA256 `86f2d7e7c8dd2c319623ecb8343ac203caa951c60049727f98b1594ab92bd758`.
5000 unique / OFFLINE_PASS / 오류0, 122종 planned-primary 최소4, train4382/val618,
정적4171/이동829를 유지했다. v112의 Ego/객체 XY·yaw·spec·split·이동 경로는 모두 보존했다.
이동 actor 1271개는 변경하지 않았고 정적 actor 9371개 중7671개의 입력 Z가 바뀌었다.
절대 변화 p50 약0.002cm/p95 7.763cm/최대1.207m다. 최대치는 roadside 추정이며 검증된 높이가 아니다.
지도 confidence는 road strip2067/near centerline5187/roadside1138/unsupported979다.
이 confidence를 센서 ground 검사 성공으로 사용하지 않는다.

실제 동일 위치 Dog2 A/B: annotation 하단–지면 +31.274cm → −10.089cm,
old review → new accepted, 입력 Z/실제 캡처를 바꿨으며 GT를 사후 이동시키지 않았다.
정상 근접 Ped_Pet1/ElectricScooter2 old/new 모두 통과했다.
혼합4actor는 원본1회 + Girl1 근접 witness1회 + 원본 raw 재검사로 통과(13.567초), 재촬영0이다.
새 frozen 목록 대표4셀은3승인/1부족분(66.306초); 부족분은 근접 혼합의
`static_ground_reference_pose_or_view_mismatch`이며 primary 자체 ground는 통과했다.
별도 최초 Mixer scene은 캡처 전 `Stationary Ego drift exceeded`로 halt/Park됐고,
최대높이변화 animated 혼합은 지형 witness 확보 불가로 부족분이다. 실패를 숨기지 않는다.
CPU 승인분: 혼합1장 GT4(Obstacle3/Pedestrian1), frozen 차량3장 GT4, raw 보존.
261 Base5 tests PASS. 완료 대표 campaign resume 추가촬영0/3승인1부족분 유지.

정적 QA 비용 제한은 child quality-loop마다 measured-height 재촬영1회·배치 대체1회,
지면 witness 최대8회이며 동일 UID+동일 scenario payload에는1회만 허용한다.
8회는 actor 최대8개에 대한 상한이지 매 장면8회를 찍는다는 뜻이 아니다.
실측 above-ground일 때만 높이 재촬영하며 증거 부족만으로 Z를 낮추지 않는다.
한도 초과는 deficit로 보존한다. 바깥 quota campaign은 기존 최대2round이므로
전체 비용 한도가 한 child 한도와 같다고 해석하지 않는다.
서로 다른 output에서도 같은 ROS master를 동시에 제어할 수 없도록 live lock을 추가했다.
원래 output에는 새 runtime으로 resume하지 않는다. v112 재현은 commit `c67f62c3`이다.
본수집/학습은 시작하지 않았다. 5K 전체 live 인증·고정 ETA는 주장하지 않는다.
세부 실험/실패 원인은 [초기 높이 근거](261007_base5_static_height_evidence.md#최종-xy-기준-보완--v113)를 따른다.

## 동적 장애물 미션 차량 소량 보강 — v112 이력

기존250개 동적 미션 슬롯 중 일반 사람12개를 차량12개로 교체한다.
미션의4.8%,전체5K의0.24%이며 동물/반려동물 동반/유모차/탑승자/dummy 희귀 quota는 유지한다.
차량 단계는 접근6/횡단3/양보2/통과후1이다. 길이6m 이하 일반 차량 pool에서12종을 고르게 선택했다.
Porter2 등 소형 상용차도 포함하며 MiddleSize 출제 한정으로 해석하지 않는다.
대형 차량은 다른 패밀리/정적 미션에서 유지한다. 여기서43종 모두를 찍었다는 뜻은 아니다.

차량은 Single-frame 외관 학습이므로 정지 캡처한다. 실제 차량 이동/trigger/속도 GT 인증은 아니다.
CW000000/CW000002 주변12m 안의 횡단 방향 MGeo 도로 link XYZ/yaw를 사용하며,
횡단보도 보행 축 위에 임의 차량 yaw를 넣지 않는다. road witness를 전수 audit한다.
차량 양보 관측만8–20m로 조정했다. 기존 사람용5–10m와 일부 큰 차종은
거리·비충돌 조건을 동시에 충족하지 못했다. 보행자 등 기존 거리 정책은 유지한다.
metadata/manifest에 같은 거리 정책을 기록하고 같은 primary 자산으로 bounded retry한다.

새 manifest:`~/morai_cap/base5_v2_261007/final_5k_dynamic_vehicle_v112_ready/manifest.json`.
기존v111의4988개 scene payload/hash/metadata는 그대로 재사용하고12개만 재생성한다.
정적 우선 순서는4171→실제이동829이며 기존4160/840 limit를 재사용하지 않는다.
family/미션 총량5000/250, GT/가시성/접지/CollisionData 계약은 유지한다.
기존 manifest/output을 덮어쓰지 않았으며 구output에는 새runtime으로 resume하지 않는다.
v111 재개는 원래 commit `b0997088`, v105/v109는 원래 `08bf6816`을 사용한다.

검증:5000 unique/OFFLINE_PASS/오류0,train4382/val618/test0,
122종 planned-primary 최소4장 유지. 새4개 단위검사를 포함한 결과와 실제 최종 smoke는 아래 후속 기록에 갱신한다.
초기 대표 live `dynamic_vehicle_v112_smoke4`는 접근/횡단/양보/통과후4/4,
32.924초였다. ground/contact gate·종료Park을 유지했다. CPU4frame/Vehicle5GT,제외0.
이 초기 manifest의 entry 양보 거리에 구5–10m가 남아있어 spec의8–20m와 달랐다.
scenario/실제 검사 정책은8–20m였고 entry도 일치하도록 수정해 최종 manifest를 새로 만들었다.
전체 자산/새 지형 live 인증이나 모델 성능 결과는 아니다. 본수집·학습은 시작하지 않았다.

최종 `dynamic_vehicle_v112_final_smoke4`도4/4,32.983초였다. 양보entry/spec8–20m 일치,
4장 실제CollisionData no-contact·ground gate 및 종료Park을 확인했다.
CPU4frame/Vehicle5GT·제외0·raw보존이다. 신규12종을 모두 live 인증한 것은 아니다.
단위검사250개 PASS. 완료smoke의동일명령resume도추가촬영없이완료됐다.
manifest SHA256:`6b675018cad62652c385facb5583682e91a4512fb860e3720659217535be821c`.
후속 정적 자산 상향/하향 배치 반복시험은 별도 계획이다. 자동 낙하·침투 복구를
모든 자산에 가정하지 않으며 현재 접지 gate와 입력 Z 보완 정책은 유지한다.

## 정적 장애물 미션의 전체 자산 대응 — v111

정적 장애물 구간은 Obstacle 클래스만 나오는 구간이 아니다. 공식 출제 범위에 맞춰
일반 차량 43종을 정지시킨 경우, 정지 사람·동물·유모차·탑승자·반려동물 동반 사람도 포함한다.
정적 미션에서는 보조 actor까지 속도 0, waypoint 이동 없음이다. 원본 Instance의
Vehicle/Pedestrian/Obstacle/Animal 및 복합 component 분류를 유지하며 모두 Obstacle로 바꾸지 않는다.

| 정적 미션 주 객체 그룹 | 장면 |
|---|---:|
| 실체 장애물 19종 |224|
| 일반 차량 43종의 정지 배치 |180|
| 정적 차량 에셋 16종 |66|
| 일반 사람 25종 |90|
| dummy 3종 |12|
| 동물 3종 |18|
| 반려동물 동반 사람 1종 |12|
| 유모차 3종 |18|
| 탑승자·이륜차 복합 9종 |30|
| 합계 |650|

기존 obstacle 미션 600과 other_vehicle 미션 50의 슬롯 안에서 변경한다.
위 그룹은 GT 클래스별 총량이 아니라 **주 객체 배치 quota**다. family는 장면 맥락이다.
전체 5K 중 650장면만 새로 생성하고 기존 4350 scene row/payload는 그대로 재사용했다.
회피 approach/entry/alongside/clearing, 좌우·경계 간격·1–3 actor 정책을 유지한다.
122종 모두 정적 미션 primary 최소 3장, dataset 전체 planned-primary 최소 4장이다.
새 split은 train4381/val619/test0이며 실제 가시성·접지는 별도 capture gate를 통과해야 한다.

v111 보존 manifest: `~/morai_cap/base5_v2_261007/final_5k_static_all_v111_ready/manifest.json`.
전수 OFFLINE_PASS/5000 unique, planned coverage PASS다. 대표 live 결과는 아래 후속 기록을 따른다.
기존 v105/v109 output·캡처·contract는 보존한다. 생성기 코드 변경으로 **새 runtime을
기존 output에 resume하지 않는다**. 기존 수집을 재개할 때는 원래 revision을 사용한다.
본 수집은 자동 시작하지 않았다. 이전 비용 표본의 13.4h/45.7h는 새 분포의 확정 ETA가 아니다.

### v111 대표 검증 결과

- `static_mission_all_v111_smoke9`:9장면 중7승인/2부족,145.136초.
  Scooter_01/K5/정적Grandeur/Woman5/NCAP_EPTa/Dog2/BabyStroller2가 통과했다.
  Ped_Pet1·ElectricScooter2의 접근 장면은 bounded 재배치 후에도 지면 관측 근거가 부족했다.
  부유가 확정된 것이 아니며, 해당 장면을 성공으로 세지 않았다.
- `static_mission_composite_close_v111`:같은 두 자산의 별도 alongside 장면2/2,
  11.291초. 원래 접근 장면을 복구한 것으로 합산하지 않는다.
- 승인9장 모두 정지 설정·ground gate·실제 CollisionData no-contact 근거를 유지했다.
  122종 전체 live 인증은 아니며, 모든 새로운 접근/근접 지형이 해결됐다는 뜻도 아니다.
- 첫7승인 CPU 변환:7frame/8GT(Vehicle2/Pedestrian3/Obstacle2/Animal1).
  가시성 근거가 부족한 Pedestrian component1개 제외,raw 보존. 정적 미션이라고 GT를 Obstacle로 덮어쓰지 않았다.
- 새manifest SHA256:`499ba5663ef824d3ebbdb2a5c25a132fa303a12baacbe8c361ee833f5599ab56`.
- 단위검사246개 PASS. 완료 smoke9 resume에서 selected ledger SHA와 누적145.136초가
  불변이며 추가 촬영0이었다. 전체 audit 및 신규/기존 output 구분을 확인했다.
  Production detector/tracker/학습은 미변경이다.

## 읽을 문서

1. 이 CURRENT: 현재 계약·실행 경계.
2. [5K 구성/진행표](261007_base5_direct_5k_current_plan.md).
3. [현재 운영 런북](261007_base5_5k_capture_runbook.md): rc부터 resume/대시보드/변환까지.
4. [v105 근거](261007_base5_final_gates_v105_evidence.md): 실제 실패·보완·결과·비용·한계.
5. [Sunny13시 정정](261007_base5_sunny13_environment_correction.md): 실제 환경과
   잘못된 11시 선언 구분, 기존 resume 보존 및 새 header 경로.

[v100 이전 누적 이력](BASE5_V2_HISTORY_THROUGH_V100.md)은 과거 분석이 필요할 때만 읽는다.
v45/10K 명령·18.7h 등 과거 ETA를 현재 명령/예상으로 사용하지 않는다.
구 halted 수집32장·과거10K manifest·실패/pilot/raw는 보존했다.

## 구성

| 패밀리 | 범용 | 미션 | 합계 |
|---|---:|---:|---:|
| 고주로 차량 |900|600|1,500|
| 로터리 차량 |100|250|350|
| 기타·정적 차량 |100|50|150|
| 실체 장애물 |900|600|1,500|
| 사람·동물·복합 |1,000|250|1,250|
| 빈 배경 |250|0|250|
| 합계 |3,250|1,750|5,000|

범용positive3K는 single900/regular1,500/dense600(30/50/20%). 크기 비용2/3/6/8,
regular10–14점/dense16–22점/최대8actor. 배경250장은 별도다.
미션은 상황별1–3개이며 범용 점수로 억지 증원하지 않는다.
복합 actor의 component 수와 배치/visible-positive parent 수를 구분한다.
범용혼합은 최소2개·배치 parent 절반 이상 positive여야 승인한다.

122종 pool 유지: vehicle43/static vehicle16/obstacle19/person25/dummy3/animal3/
pet-person1/stroller3/rider9. 열린 G80은 공식 출제 제외, 문 닫힌 G80은 유지한다.
WoodBox/CargoBox 및 MR 복구 자산을 포함한다. 배경 나무/시설은 신규 positive quota가 아니다.
종류당 primary 최소4장 계획을 확인했으며 실제 accepted-visible primary4장도 별도로 감사한다.
이 하한은 학습 충분성 보장이 아니다.

## 배치·동작

- Ego는 장면마다 도로 anchor+기존 yaw/횡 offset, 정지·Park·1capture다.
  NPC는 실제 경로 position/yaw이며 임의 횡/yaw jitter를 주지 않는다.
- 고주로 범용은 샘플 despawn 이후를 포함한 허용 도로를 유지한다.
  미션 NPC만 샘플 spawn/despawn corridor로 제한한다.
- 범용 근접175셀, 차량 미션 근접133셀(고주로66/로터리67).
  BBox 경계 간격과 actual annotation 간격을 구분한다. collider 인증은 CollisionData로 별도 검사한다.
- 정적 회피는 approach/entry/alongside/clearing·좌우 관측이며 alongside240obstacle/
  20static-vehicle 셀이다. unknown geometry는 보수적 간격을 사용한다.
- 동적 미션250셀은 MGeo CW000000/CW000002를 기준으로 접근113/횡단62/양보50/통과후25다.
  과거 starter 기준 Ego −35~+55m 범위를 미션 분포로 사용하지 않는다.
  모든 대회 진입 경로를 구현·인증했다는 뜻은 아니다.
- 현재 v117은 사거리 미션의 관절 외관 자산만 natural-target 실제 이동한다.
  범용/정적 미션의 사람·동물도 정지이며 rigid/fixed-pose 자산은 계속 정지다.
  이는 속도 회귀/temporal 학습을 위한 정책이 아니다. 범용 동시 이동은 구v105 계약이다.
- 현재184mover 모두 저장높이 기반 LIVE endpoint lookup으로 초기화한다.
  empty LiDAR+static reference의고비용준비는적용하지않고원본ground/Z/실제이동/contact를검사한다.
  editor-proof가없는legacy/opt-out모드는기존독립지면준비를유지한다. 두모드를혼동하지않는다.

## GT·안전·실패

- raw LiDAR Instance geometric center/size/rotation/component를 보존한다.
  Object_topic pivot/catalog 보정을 sensor GT에 다시 적용하지 않는다.
- `lidar1_camera0`: LiDAR≥1점 또는 실제 camera Instance visibility 근거.
  0점 camera-visible을 일괄 제거하지 않고, 안 보이는 객체를 무조건 positive로 넣지 않는다.
  현재320×896 bottom crop에서Instance≥16px·최소extent4px다. 입력crop을바꾸면
  raw Instance에서가시성을재감사하며그변경만으로재수집할필요는없다.
- ROI는 LiDAR/Ego[-100,-30,-3,100,30,2]. 도로 접선은 배치용이며 GT/모델 ROI 기준이 아니다.
- Ego pose·신선한 ROS·Park·CollisionData·파일완결·접지·이동/Z·가시성 gate는 유지한다.
  근접 blind spot/낮고 작은 물체를 학습용 가짜 GT로 메우지 않는다.
- fresh transport·Park/Ego/contact가확인된명시적lightweight-all-static의actor누락은
  primary를포함해원본진단review→동일quota재배치/부족분이다. 누락 자체는 성공이 아니다.
  missing mover/legacy-primary·통신/Park 미확인은 계속 halt다.
- 확인된 접촉은 실패/부족분. 안전 Park·새 scene 상태가 확인된 경우만 다음으로 진행한다.
  unresolved quality는 bounded retry 후 부족분, calibration/review/failed는 학습에서 제외한다.
- safe paused는 같은 코드/manifest hash로 resume. halted는 강제 resume 금지다.
  완료 resume는 증거 검증 전용·추가 촬영0이며 부족분은 별도 보충 campaign이다.

## v105–v109 준비 산출물 이력 / 환경 정정과 기존 수집 보존

- 원본 `~/morai_cap/base5_v2_261007/final_5k_v105/manifest.json`.
- 기존resume 원본 `~/morai_cap/base5_v2_261007/final_5k_ordered_v105/manifest.json`.
  원본 SHA256: `407cc518863d496b770a87d9cd75c3ecd7989673449c1e7f854bd3fc90b8bb81`.
- 신규 수집용13시 정정본:
  `~/morai_cap/base5_v2_261007/final_5k_ordered_sunny13_v109/manifest.json`.
  SHA256:`1b8a6ced7d2af06631e1cbd2897a4464adc897ef833023f6f4f373e5c0d04523`.
- v109 당시 config:`base5_independent_5k_sunny13.json`의 아래 SHA는 당시 버전이다.
  현재 v111은 정적 미션 policy가 추가돼 이 SHA와 다르다.
  SHA256:`d58e15791a32005655da1e343e109a718646b0859bcd4fdb2918d85e03d35948`.
  기존 generator 기본 경로의 `sunny11` 파일은 동일한 13시 내용의 호환 alias다.
- 5000 scene row/geometry/metadata/split은 정정 전후 불변이다.
  이미 수집 중인 명령은 원본 manifest/output 그대로 resume하고 정정 기록을 함께 유지한다.
  정정본은 새 output에서만 사용하며 중간에 갈아끼우지 않는다.
- 5,000 unique payload, train4,387/val613/test0, OFFLINE_PASS/오류0.
- 순서만 정적4,160→실제 이동840으로 변경, 모든5000payload/quota/split 보존.
- 122종 planned coverage PASS/min4. 전체 sensor effective coverage는 아직 NOT_CAPTURED다.
- 최종 통합 `final_pilot_v105`는 별도 검증이다. 최신 수치·resume·CPU·시간은
  [근거 문서](261007_base5_final_gates_v105_evidence.md)의 동결 결과를 따른다.

## v105–v108 결론·잔여 경계 이력 — 현재 시작 판단은 위 v117

- 차량 근접7/7, 최종 통합18셀 중15승인(정적12/12·실제이동3/6), CPU57GT다.
  후속 독립 지면 보완은 이전 실패3셀 중1셀을 추가 복구했다. 전체 성공률 추정이 아니다.
- 고정seed 비용 표본은12중11승인(정적10/10·실제이동1/2),254.250초다.
  정적11.60초/승인,동적138.22초/승인으로 준비·실패·QA 비용을 포함한다.
  계획4160/840으로 단순 외삽하면정적부분13.4h/전체45.7h·원본과QA약75GiB다.
  동적표본2개뿐이고부족분보충·지형차이에민감하므로약속된ETA가아니다.
- **배치·GT·안전·resume 경로 준비와 모든 동적 장면의 접지 해결은 다르다.**
  일부지형은독립support/참조불일치로bounded retry 후부족분이다.
  정적먼저4160→이동840순서는준비했지만,전체5K를빠른야간수집이라고권하지않는다.
  사용자보고후정적우선진행또는동적준비비용추가최적화범위를정한다.
- 신규coverage의`--require-effective-complete`는현재소량15장에대해의도대로거부한다.
  아직전체122종의actual-positive4장씩이충족된dataset은아니다.
- 원본/campaign hash는버전별이다. v108로수정한코드를구v105/v107 output의resume에섞지않는다.
- 최종v108 fresh회귀4/4·4class10GT·완료resume추가촬영0,단위검사238개PASS다.
  이는남은unsupported동적조건을모두해결했다는의미는아니다.

Production E25·threshold0.40/0.15/0.10·tracker·planning/control/localization은 미변경이다.
새 모델 threshold는 학습 후 동일 held-out mission rosbag의 Detection/Tracking/Map-CV로 결정한다.
오프라인 split은 neighboring-view leakage가 남아 최종 일반화 성능의 근거가 아니다.
전체5K 물리 검증/모델 성능/4090 full-stack 실시간성은 이번 소량 수집 검증으로 인증하지 않는다.
