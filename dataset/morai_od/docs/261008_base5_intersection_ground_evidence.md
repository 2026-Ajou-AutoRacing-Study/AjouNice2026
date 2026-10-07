# Base5 수동 사거리 높이 lookup — EVIDENCE / v117

## 2026-10-08 출발→도착 2점·보행자 반대 방향 — 최신 v117

사용자 요청: 두 끝점의 선형 이동을 기본값으로 채택하고, 반대 방향 사람을 충분히 포함한다.
촘촘한 v116 waypoint·raw·manifest를 덮어쓰지 않고 새 opt-in 모드/새 lookup/새5K를 만들었다.
production detector/tracker/planning/control/localization 변경, 본5K 수집, 모델 학습은 없다.

### 샘플 근거와 해석 경계

`~/Downloads/original_2026_molit_comp_sample_scene.json`과 저장된 sample의 Man1 경로는 같다.
Man1(DataID50100001/UID3), PedPath - 3, 뒤 횡단보도 CW000000의 두 점은
(-56.7439994812,-11.6140003204,28.561000824) →
(-70.5690002441,-11.9289999008,28.433000565)이다. XY13.8286m, Z차−0.1280m다.
기존 `planner_logs/analysis/pedestrian_udp_velocity_261005/zz_ped_vel_udp_261005_1.bag`의
실제 Z는 끝점 선형 보간과 최대약6.55e−6m 차이였고 속도2.7804m/s였다.
두 waypoint/선형 Z의 근거이지, 원본 끝점의 수동 편집 방식·정확한 접지·자동 terrain following
또는 본선 모든 actor의 경로 토폴로지를 증명한 것은 아니다.
샘플 trigger15m와 이번 즉시 이동/준비시간 보상도 구분한다. 이동 외관을 배우는 정지 Ego OD 수집이다.

### 초기 높이·방향 구현

- `endpoint-linear-v117`: 실제 actor 경로와 profile 높이 knot를 정확히2개로 고정한다.
  target Z도 같은 두 끝점의 직선이며 중간 지면을 따라 굴곡진 경로로 만들지 않는다.
  raw point/Instance annotation을 수정해 정합 오차를 숨기지 않는다.
- 끝점 후보 domain은 횡단축 s=−11.9…+12m다. 기록 범위 안의 수동 높이를 사용한다.
  짧던 CW000002 사람24종의 상단 후보는 같은 구역 Ped_Pet1의 저장 높이를 사용하되,
  겹치는 s=−6/−3/0m의 차이 중앙값으로 pivot 차이를 보완한다. 비교 차이가5cm를 넘으면 거부한다.
  이것은 cross-asset 후보이며 실제 자산별 양방향 QA를 새로 통과한 뒤에만 LIVE로 발행한다.
  모든 도로/모든 평행 경로에 donor 높이를 확장하지 않는다.
- 실제 primary의 crossing×walk/run×subfamily별 방향을 생성 전에 배정한다.
  홀수 subgroup와 전체 walk/run aggregate 모두 양방향 차이≤1이다.
  재시도/replacement에서 방향을 바꿔 어려운 반대 방향을 회피하지 않는다.
  실제 raw 끝점 방향과 metadata sign 일치도 전수 audit한다.
- 기존 dense 모드는 opt-out 이력으로 유지한다. source/profile/report hash와 준비 scope 검사는 유지한다.

### 높이·이동 live QA

일반 사람24종+Dog1/Dog2/Deer1/Ped_Pet1=28종×두 횡단보도×양방향이다.
사람은 running2.2m/s로 긴 준비 거리 조건을 검증했고 동물/Pet은 기존걷기속도를 유지했다.

| 항목 | 결과 |
|---|---:|
| 양방향 승인용 고유 profile/direction 캡처 |112/112 승인|
| LIVE profile |56/56|
| 이전 진단까지 실제 총 시도 |127|
| 이전 review (합격 근거로 미사용) |1|
| 전체 session wall (실패·중복 포함) |1069.382s|
| 실제 시도당 wall |8.420s|
| 최대 Object Z↔경로 선형 Z 차이 |0.004735mm|
| 최대 연속 Z step |0.141144mm|
| 측정 가능한 annotation 하단↔local plane 절대 gap |7.399cm|

처음 Ego20m에서 `b5ed117_231_r` Dog1역방향1건이 review였다.
Camera image/BBox2D/BBox3D에는 개가 있었지만 **LiDAR Instance.txt의 3D GT가 비어 있었다.**
실제 actor 이동/지정높이/CollisionData 검사는 정상이며 심한 부유의 근거는 아니다.
현재 converter는 LiDAR3D annotation에서 box를 구성하므로 이 raw를 camera-only 합격으로 꾸미지 않았다.
중단 session의14승인+1review를 별도 진단 근거로 보존하고 승인집계에 중복 포함하지 않는다.
Ego 관측 거리12m의 새 output에서 같은 경로/속도 Dog1 및 나머지 asset을 재검증해 GT 포함 PASS했다.
report의 `prior_diagnostic_attempts`/`used_for_profile_approval=false`에 초기 실패 비용을 남겼다.

추가 empty-terrain/static-reference/높이 재촬영0, 모든 선택 session의 안전 종료 Park 확인이다.
입력 경로를 따르는 Object Z만으로 접지를 인증하지 않으며, 측정 불가 plane은 기존 warning이다.
기존 심각한±40cm gap·Z step·실제 충돌·Ego 정지·transport 검사는 유지한다.

### 새5K 방향 계획과 전수 검증

5000고유payload OFFLINE_PASS, primary122종 최소4, 정적4846/실제 이동154장면이다.
기존 family·미션·거리·근접·관측단계(접근73/횡단37/양보25/통과후19)는 유지했다.
전체 이동 장면의 모든 mover는 정확히2waypoint이며 source/report proof를 검증했다.
이동 scene154중1개는 primary standing/secondary moving이므로 이동 primary 집계는153개다.

| 구역 | 일반 사람 걷기 +/− | 일반 사람 달리기 +/− | 희귀 포함 primary 걷기 +/− | 희귀 포함 primary 달리기 +/− |
|---|---:|---:|---:|---:|
| 첫 CW000002 |20/20|15/14|26/25|16/16|
| 뒤 CW000000 |19/19|13/13|21/21|14/14|

+/−는 lookup 횡단축 기준이며 Ego 전후방 의미가 아니다.
사람/animal/Pet subgroup도 각각 차이≤1이다. 방향뿐 아니라 걷기/달리기별 반대 방향이 존재한다.
이는 계획 장수이며 실제 최종 visible-positive 비율은 본수집 ledger로 다시 검사한다.

최신 root `~/morai_cap/base5_v2_261008/`:
- `endpoint_linear_candidate_v117b/`: 원본 hash·끝점 derivation을 보존한 pending 후보.
- `endpoint_linear_live_v117/`, `endpoint_linear_close_live_v117/`: 모든 성공/실패 raw.
- `endpoint_linear_release56_v117/`: 새 LIVE lookup 및112승인/127시도 report.
- `endpoint_linear_final_5k_v117/`: config,5000scenario/metadata,offline/geometry/coverage audit,대표18manifest.
- `endpoint_linear_release_pilot18_v117/`: 일반 생성기 batch/resume/실패보존 근거.

### 일반 생성기 batch·resume·변환 최종 결과

새 manifest에서 live 결과를 보기 전에18셀을 골랐다. 두 구역×걷기/달리기×양방향8층을
모두 포함하고, 두 mover 혼합·Animal/Pet·접근/횡단/양보/통과후 및 정적 장면을 함께 검사했다.
18설계셀18승인/부족분0, 실제21capture, parent 누적wall184.685s다.
첫4셀 뒤 pause→같은 계약으로 나머지14셀 resume을 검증했다.

정적 근접 Man1 `b5v2_00009`의 기존 actor 누락은 이번에도3review였다.
Object/센서 annotation 모두 actor가 없고 높이 부유나 실제 접촉으로 인증하지 않는다.
같은 primary/family/phase/거리/split 계약의 bounded replacement로 최종 승인됐으며
3개 실패 raw를 보존한다. 선택18승인은 최초시도18/18 성공률이라는 뜻이 아니다.
기존 정상15개 실제 이동 대표 캡처는 모두 승인됐다.

control_readiness 중 MORAI service handler None 응답2회는 event log에 남고 재요청으로 복구했다.
최종/경계 Park는 모두 확인했으며 service timeout0, alive worker0, unsafe session0이다.
응답 오류 자체가 없었다고 보고하지 않는다. terrain/static-reference/높이 재촬영은 추가하지 않았다.

완료resume 검증은 원본capture719파일의SHA256/inode/mtime/size,8개event log,
선택ledger와누적wall 불변 PASS다. 추가capture0이다.
CPU변환18프레임(train13/val5): Pedestrian15/Animal7/Obstacle5/Vehicle1=28GT,
추가Animal1component는 가시성 정책에서 제외하되 원본에 보존했다.
소량 split은 최종 성능평가용이 아니다. 모델 학습/추론은 실행하지 않았다.
Base5 regression300tests PASS(58.784s), py_compile·diff whitespace 검사 PASS다.

이동 경로 검증127시도 평균8.420s, 정상 생성기 실패보충 포함18셀 승인당10.260s다.
서로 다른 구성의 v116/v117 wall을 waypoint 단축의 인과적 속도 개선으로 발표하지 않는다.
6초 자연 이동 준비가 남아 있고 전체5K는 여전히 과거 정적 비용 가정에 따라약10시간 안팎의
조건부 예상이다. 새로운 전체family 장기rate 인증은 없으며 본수집dashboard로 갱신한다.
위의 OFFLINE_PASS/18승인은 본5K live 인증이 아니다. 본수집은 사용자 승인 대기다.

재현 보조 도구:
`prepare_base5_endpoint_linear.py` → `prepare_base5_intersection_ground_pilot.py` →
`report_base5_intersection_ground.py`(이전 실패 `--prior-sessions`) →
`freeze_base5_intersection_release.py --endpoint-linear`.
새본수집은 [현재 런북](261007_base5_5k_capture_runbook.md)의 새 manifest/output을 쓴다.

## v116 이력 — 수동 다점 높이 보간

2026-10-08. 사용자 수동 경로를 실제 XY 기준으로 보간하고, 이동 외관 캡처를 사거리에
제한해 초기 높이와 수집 비용을 보완했다. 작은 높이 오차 때문에 반복 촬영하지 않는 것이 목적이다.
BEVFusion production·threshold·tracker·planning/control/localization은 변경하지 않았다.

## 수동 파일과 보간

- 원본 `0_1_…0_10_`와 사용자 `1.json`…`10.json` 모두 확인했다. 수동 waypoint49개다.
  `1.json`만4점/나머지5점이며 `2_MN.json`은 missing으로 기록했다.
- 원본/보정본·각 EgoNetwork sidecar를 copy/hash 확인한 뒤 MR에서 정확히 해당39파일만 정리했다.
  다른 사용자 숫자 파일이나 기존 raw는 이동/삭제하지 않았다.
- 실제 저장 XY를 횡단보도 축의(s,l)로 투영해 Z를 구간별 선형 보간한다.
  사람이 비스듬히 찍은 XY를 정렬한 뒤 같은 높이라고 가정하지 않았다.
  actor의 world Z와 waypoint 내부 Y를 구분했다: world→waypoint=(-x,z,-y).
- 처음 waypoint Z는 provisional seed와 같아 독립적으로 수정된 지면 표본이라고 인증하지 않는다.
  여러 경로의 중간/끝 높이는 약28.36–28.57m로 달라진다. 전 구간 flat Z가 아니다.
- actor 저장 위치가 출발 waypoint에서 최대 약73cm 이동한 경우가 있어 live scene에서는 실제
  경로 시작점으로 돌려 놓았다. 원본 수동 파일은 수정하지 않는다.
- 경로 길이 밖은 외삽하지 않는다. 실제 path 주변 최대1m 띠까지만 초기 높이 후보로 사용한다.
  평행±0.5m는 대표 사람 live로 확인했으며 다른 모든 평행 위치의 지형 인증은 아니다.

## Live 검증

| 검증 | 캡처 | 승인 | wall |
|---|---:|---:|---:|
| Woman2/Dog1/Dog2/Deer1/Ped_Pet1 × 두 횡단보도 × 양방향 |20|20|166.180s|
| Woman2 running × 두 횡단보도 × 양방향 |4|4|35.207s|
| Woman2 평행±0.5m × 두 횡단보도 × 양방향 |8|8|68.686s|
| Woman2/Dog2, Ego 약8m 위치 × 두 횡단보도 × 양방향 |8|8|66.709s|
| 다른 일반 사람23종 × 두 횡단보도 × 양방향 |92|92|747.563s|
| 합계 |132|132|1084.345s|

기본5자산20회의 평균8.309s, 추가 조건 포함132회 평균8.215s/capture다.
실제 Object Z↔지정 경로 높이 최대차3.415mm, 최대 연속 Z step2.302mm다.
측정 가능한 raw annotation 하단↔local plane 최대절대 gap8.583cm다.
Object Z residual은 지정 경로를 따르는 지표이며, 이것만으로 물체의 물리적 접촉을 증명하지 않는다.

추가 empty-terrain/static-reference/height recapture는0이다. 원본 cloud/annotation 자체를 검사한다.
불확실 plane·중간 정도 gap은 warning이다. 신뢰 plane 기준±40cm 초과, 심한 Z step,
경로 domain 밖·실제 충돌·Ego 정지 실패는 계속 제외/중단 정책을 유지한다.
모든 종료 Park 확인과 실제 Ego 정지·actor 이동·GT/가시성/CollisionData 검사를 통과했다.

다른 사람23종은 Woman2 경로 높이를 **후보**로 공유한 뒤 각각 양방향 live QA를 통과했다.
이28종 검증을28종 전부의 외관 차이 A/B 실험으로 해석하지 않는다.
기존 대표 외관 변화 근거5종과 개별 높이/이동 QA는 다른 증거다.

## 생성기·수집 경로

- `dynamic_region=intersection-only-v116`: 신호등 사거리의 미션 관측 장면만 실제 이동한다.
  범용/정적 회피의 같은 자산은 standing/static으로 남긴다. 자산이나 희귀 quota는 제거하지 않는다.
- 일반 사람24종+Dog1/2/Deer1+Ped_Pet1=28종의56profile을 발행했다.
  AI police1 및 NCAP는 standing이며 실제 이동 검증 대상으로 세지 않는다.
- pending lookup은 `calibration_only`에서만 사용한다. 실제 수집 profile은 양방향 합격 report,
  원본/사용자 보정본/template hash로 고정한다. source/report 변경 시 검증 실패한다.
- 새release는 `require_covered_moving_paths=true`이다. 출발→목표→끝 전체를 보간할 수 없으면
  **초기 생성에서 같은 설계 셀로 재시도**한다. 속도·희귀 자산·family/phase/거리 quota를 몰래 바꾸지 않는다.
  legacy opt-out은 독립 terrain/static-reference 준비를 유지한다.
- 모든 mover가 editor-proof를 가진 경우에만 quality loop/shared-ROS batch의 고비용 준비를 생략한다.
  하나라도 미준비 mover가 있으면 기존 엄격한 준비로 남는다. 매 장면 네트워크 reload는 유지한다.
- 촘촘한 waypoint가 되면서 swept-path 검사 끝점을 `points[1]`이 아닌 `points[-1]`로 수정했다.
  원래 cloud나 raw sensor GT를 보정해서 오차를 숨기지 않는다.

## 새 5K offline 동결

| 패밀리 | 장면 |
|---|---:|
| 고주로 |1500|
| 로터리 |350|
| 기타/정적 차량 |150|
| 실체 장애물 |1500|
| 보행자/동물/복합/차량 횡단 관측 |1250|
| 빈 배경 |250|
| 합계 |5000|

정적4846/실제 이동154장면이다. 전체122종 primary 최소4회를 유지한다.
5000개 scenario hash와154개 이동 scene의 actual metadata/raw waypoint/lookup source/report 검증을 통과했다.
이동 actor 수는 장면 수와 다르며 mixed 장면에는 두 mover가 있을 수 있다.
관측 단계별 실제 이동 장면: 접근73/횡단37/양보25/통과후19이다.
Sunny13시·single-frame·원본 component GT와 가시성 정책을 유지한다.

첫 CW000002의 사람 수동 경로는+방향 끝이 짧다. 준비6초/달리기2.2m/s를 만족하는
역방향 사람 경로가 그 범위에 들어가지 않아 방향 편향이 남는다.
이동 actor별 CW000002 running39건은+37/−2이며 −2는 사람 running 외관을 인증한 값이 아니다.
CW000000 running은+21/−11이다. 전체 걷기/달리기 양방향을 포함하지만 정확한 방향 균형은 아니다.
보강하려면 더 긴 사용자 경로/독립 검증된 높이가 필요하다. 미측정 구간 flat 외삽으로 채우지 않는다.

## 일반 생성기 batch 검증 중 발견한 별도 문제

최초 isolated 대표4개4승인47.250s, 첫 shared 대표4개4승인36.564s였다.
전체 shared 대표12개를 이어 검증하던 중 정적 근접 Man1 장면(`b5v2_00009`, 중심거리1.649m,
기하학적 BBox gap0.357m)에서 Object/센서 annotation 양쪽 actor가 없었다.
fresh topic은 계속 왔지만 shared post-load gate가 이를 통신 오류로 분류해 전체를 중단했다.
단독 원본 캡처에서도 `missing_actor_annotation`, `missing_runtime_actor`,
`primary_no_sensor_evidence`가 재현됐다. 높이 보간 오류나 실제 충돌의 근거로 해석하지 않는다.
정확한 MR spawn 거부/누락 기전은 미확정이다.

명시된 경량 all-static 장면은 fresh transport와 기존 Park/Ego/CollisionData 확인을 유지한 채
원본 capture/annotation으로 누락을 review할 수 있도록 수정했다.
누락은 합격으로 바꾸지 않으며 기존 bounded same-quota replacement/deficit 경로로 보낸다.
legacy 또는 이동 primary, stale topic에는 이 예외가 적용되지 않는다.
첫 halted output/단독 raw는 실패 근거로 보존하고 새 output으로 재검증한다.

## 최종 일반 생성기·resume·변환 결과

수정 후 대표12설계 셀은12승인/부족분0이다. 실제15capture, 실패 재배치 포함parent wall134.079s다.
처음4개 뒤 의도적으로pause→같은계약resume으로 나머지를 수집했다.
근접 Man1 누락은3review raw를 보존한 뒤 같은primary/phase/거리/split 재배치로 성공했다.
그 외8개 이동 장면(두actor 혼합·달리기·사람·동물·Pet·접근/횡단/양보/통과후)과
일반 정지 혼합·고주로·장애물 혼합도 승인됐다. 임의 GT 추가/누락 성공 처리는 하지 않았다.
terrain/static-reference/height recapture0, service error/timeout0, unsafe session0이다.

완료resume 검증: 원본capture511파일의SHA256/inode/mtime/size, 선택ledger, 누적wall과
service event log가 불변이다. 추가capture0이며 보존파일 수는 JSON 근거/센서 annotation도 포함한다.
CPU변환12프레임(train7/val5), Pedestrian10/Animal5/Obstacle5/Vehicle1=21GT다.
추가Animal1GT는 가시성 정책에서 제외했으나 원본에 남긴다. 학습·모델 추론은 돌리지 않았다.
Base5 unit regression293개 PASS(59.987s), diff whitespace 검사 PASS다.

동적대표132회는 평균8.215s/capture, 실패 보충 포함12셀은 승인당11.173s다.
둘 다 전체5K의 장기 처리율 인증은 아니다. 과거정적40셀의family별비용을 그대로 가정해
4846정적+154이동에 적용하면 대략10시간 안팎이지만, revised mixed 구성·근접누락/재배치·
장기 통신/디스크 변동 때문에 보장시간이나새family-weighted 실측으로 발표하지 않는다.
본수집 중dashboard 누적rate로 갱신한다. 기존4171/829나45.7h를 새실행 예상으로 쓰지 않는다.

본5K capture/학습은 미시작이다. 사용자 승인 뒤 새output으로 수집한다.

## 산출물

공통 root: `~/morai_cap/base5_v2_261008/` (ignored runtime evidence; 강제 Git 추가하지 않는다).

- `manual_ground_archive_v116/`: original/corrected JSON·sidecar·49waypoint/pending lookup.
- `editor_ground_live40_v116/`, `editor_people_live92_v116/`: 개별 live report/lookup.
- `editor_ground_release56_v116/`: 원본/각 live 근거의 hash를 합친56profile release.
- `editor_ground_final_5k_v116/`: config, scenarios_5k/manifest, offline_audit, pilot_manifest.
- `editor_ground_release_pilot12_v116/`: isolated 대표4개 후 의도적 pause 이력.
- `editor_ground_release_fast12_v116/`: static primary 부재로 halt한 초기shared 근거.
- `editor_ground_static_missing_primary_probe_v116/`: 같은payload 단독 원본/review 근거.
- `editor_ground_release_fast12_v116b/`: 수정 후 대표 batch/resume 검증.
- 위폴더 `complete_resume_verification.json`, `cost_report.json`과
  `editor_ground_release_infos12_v116b/`: 완료resume·실제비용·CPU변환 근거.

재현 도구: `import_base5_intersection_manual_ground.py` →
`prepare_base5_intersection_ground_pilot.py` / `prepare_base5_intersection_people_transfer.py` →
`report_base5_intersection_ground.py` → `merge_base5_intersection_ground.py` →
`freeze_base5_intersection_release.py`. 모두 새 output 요구, raw는 덮어쓰지 않는다.
