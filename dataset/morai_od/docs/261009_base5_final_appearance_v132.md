# Base5 최종 정지 외관·방향 설계 v132 — CURRENT / EVIDENCE

2026-10-09 00:45 KST 검증 스냅샷.
같은 날짜 후속: 사용자가 Sunny13 본 5K를 시작했다. 아래 미시작 표기는 00:45 준비 시점의 기록이다.
[수집 중 상태와 종료 후 비용 개선 계획](261009_base5_collection_cost_and_next_experiments.md)을 우선한다.
사용자 승인에 따라 v129의 부분 yaw 보강을 전면 보완하고 v131 극근접 compound 누락을 보완했다. 본 5K 캡처와 학습은 시작하지 않는다.
최신 release는 `~/morai_cap/base5_v2_261008/final_appearance_5k_v132_release`다.
이 문서는 v123/v129의 당시 경로·노출 표보다 우선하며, 이전 raw·실험은 삭제하지 않는다.

## 결정 사항

- Base5-only 5K 독립 배치, Sunny13부터 sunny/foggy × 11/13/15의 같은 구성으로 30K 관측한다.
  30K 독립 배치가 아니며 같은 scene_group_id의 6 환경은 동일 split이다.
- 정지 실체 장애물과 정지 사람·더미·동물·복합은 자유 360°다. primary를 12개 30° sector로
  균형 배정하고 sector 안에서 연속 각도를 뽑는다. 12개 고정 각도가 아니다.
- 정적 OBJ 차량은 자유 70%/도로 평행·반대 방향±15°30%, 일반 차량의 정적 미션·음영은
  도로 방향 70%/자유 30%다. 범용 일반 차량·고주로·로터리 NPC는 도로 경로 yaw를 유지한다.
- 횡단 관측은 이동 여부와 무관하게 crossing 맥락을 유지한다. 실제 이동 actor의 yaw를
  무작위로 돌리지 않는다. 음영 이동 24개는 평행 이동이며 걷기/달리기 각각 접근 6/이탈 6이다.
- yaw를 먼저 적용하고 회전 footprint·pivot offset으로 간격을 푼다. retry는 같은 mode/sector,
  primary/거리/방향/미션 단계/group/split을 유지하면서 각도를 다시 표본화한다.
- 정적 배치의 미확인 형상 41종은 승인 raw의 native component OBB를 asset local frame으로
  변환한 parent envelope로 보완한다. 기존 측정 catalog 항목은 덮어쓰지 않는다.
  이것은 **배치 간격/크기 예산용**이다. 학습 GT는 원본 component별 센서 annotation 그대로다.
- scale 1·4class Vehicle/Pedestrian/Obstacle/Animal·7D·velocity 학습 off·LiDAR/Ego ROI
  `[-100,-30,-3,100,30,2]`를 유지한다. native camera/LiDAR visibility와 부분 가림을 검사한다.
- Ego Park·실제 정지·최종 도로 지지·CollisionData·심각한 Z gate를 완화하지 않았다.
  정적은 원래 캡처만 검사한다. 모든 정지 장면에 추가 지면 캡처를 넣지 않는다.
- E25 production 모델/threshold·tracker·planning/control/localization 변경 없음.

## 현재 12개 관측 카테고리

family는 장면 맥락이며 GT class가 아니다. 정적 회피 600+50셀 전체에는 122종 전체가 배분된다.
도로 범위와 거리 deck은 v129를 유지한다. ego yaw/횡 offset도 최종 road-support 안에서만 적용한다.

| 카테고리 | 장면 | 포함 객체 / 동작 | yaw·배치 | 목적·남은 한계 |
|---|---:|---|---|---|
| 고주로 범용 |740|일반 차량 43종, 정지 외관 snapshot|도로 접선 방향; 허용 고주로 link에 1~여러 대, 전후좌우·원거리·근접|차량 종류·가림·밀집 다양성. 본선 속도/간격을 고정하지 않음|
| 고주로 미션 |600|일반 차량, 정지 외관|샘플 NPC 경로 범위·합류/순항/앞옆뒤 추월/차로 축소, NPC 임의 XY/yaw 없음|실제 마주칠 위치·초근접·60~99m 관측. 실제 motion GT 학습 아님|
| 로터리 미션 |350|일반 차량, 정지 외관|실제 Ego/NPC 경로. 접근 70/진입 80/원형 130/탈출 70, 앞뒤 차량 및 밀집도|진입·앞뒤 근접 판단. 차종·속도·생성주기 공개값으로 하드코딩 안 함|
| 기타 차량 범용 |80|일반 차량·OBJ 차량 중심, 혼합 보조|고주로/로터리/음영 외 허용도로. 일반 차량 도로 yaw, OBJ 자유 70/도로 30|정지 차량과 여러 방향 외관. OBJ를 NPC와 동일 취급하지 않음|
| 기타 차량 회피 |50|정적 OBJ 차량 중심|접근/진입/좌우 alongside/통과후; OBJ 자유 70/도로 30|정적 차량의 비스듬한 근접 회피. 실제 collider 대신 보수적 footprint 사용|
| 실체 장애물 범용 |770|실체 장애물·OBJ·사람/동물/복합 보조, 정지|다른 허용구역 도로/도로변; 정지 비차량 360°, 크기 budget 혼합|일반 형상·방향·가림·작은 객체. 보이는 객체 수는 raw로 따로 검사|
| 정적 장애물 회피 |600|실체 장애물뿐 아니라 일반차량·사람·동물·복합 122종, 전부 정지|정지 비차량 360°, 일반차량 도로 70/자유 30·OBJ 자유 70/도로 30; 좌우 통과/회피 단계|실제 임의 yaw 회피. 센서가 안 보는 낮은 객체를 가짜 GT로 강제 유지하지 않음|
| 사람·동물·복합 범용 |830|사람·더미·동물·반려동물 동반·유모차·탑승자; 혼합 장애물/차량 보조|고주로/로터리/음영 외 도로·도로변, 정지 360°; 전/후/좌/우 50/10/20/20|범용 외관·후방/측방·가림. 범용 이동 요청도 실제 정지 adapter 뒤 geometry를 적용|
| 동적 장애물 사거리 미션 |250|사람·동물·복합·더미·차량. 실제 이동 154, 정지 외관 96|두 횡단보도·접근/횡단/양보/이탈; 실제 이동은 출발→도착 2점/양방향 경로 yaw, 정지 횡단도 crossing 맥락|이동 외관 학습; 속도 회귀 아님. 정지와 이동 외형이 비슷한 자산은 굳이 움직이지 않음|
| 음영 정지 관측 |476|전체 roster, 정지; 이 중 20장 객체 없는 음영 배경|입구밖→내부 관측/진입/중앙/출구. 정지 비차량 360°, 차량 prior는 위와 같음. 오른쪽 80/왼쪽 20|어두운 실체 객체·미리 발견·엇갈린 배치. 카메라 조명은 실제 MR 환경에 의존|
| 음영 이동 관측 |24|일반 사람·반려동물 동반·개·사슴|도로 평행 이동, 걷기 12/달리기 12; 각 접근 6/이탈 6, 독립 지면 준비|지속 움직임 외관. 검증된 수동 사거리 높이를 터널에 무조건 외삽하지 않음|
| 빈 배경 |230|새 GT 객체 없음, 기존 도로시설 배경 유지|허용 맵/구역 및 Ego offset; 도로시설을 장애물 개수에 더하지 않음|false positive 제어. Fire/Tree/시설류 배경 정책 유지|
| 합계 |5000|정적 4822 / 실제 이동 178|첫 Sunny13, 이후 같은 6 환경 30K|대표 smoke는 전수 실제 캡처 성공률 인증이 아님|

범용 positive는 single/regular/dense30/50/20과 크기 budget을 유지한다. 큰 자산은 높은 비용,
작은 자산은 낮은 비용을 쓰므로 최대 8개를 무조건 채우지 않는다. 미션에 범용 dense를 강제하지 않는다.
정적 600의 pool은 실체 224/일반차량 180/OBJ16/사람 90/더미 12/동물 18/반려동물 12/유모차 18/탑승자 30이다.
별도의 기타 차량 회피 50개와 합쳐 OBJ66개다.

## 재설계·검증 중 잡은 문제

1. 기존 단순 네 방향 prior는 정적 회피·음영의 임의 회전 출제에 충분한 보장이 없어 자유 yaw로 바꿨다.
2. 정적 OBJ·유모차·반려동물 등의 geometry가 fallback인 경우 근접 간격이 1m로 clamp되었다.
   accepted raw의 native component를 모두 합친 parent envelope를 사용해 해당 근접 계산을 보완했다.
   자산별 최소 2개 정지 관측, 실제 pose/정지/무접촉/annotation hash를 요구하고 XY envelope에 3cm 여유를 둔다.
3. 범용 row의 원래 motion 요청이 walking/running이더라도 regional adapter는 standing으로 캡처한다.
   최초 v130에서 geometry 보완이 원래 요청 static/standing에만 적용되는 누락을 전수 감사로 발견했다.
   v130 pilot은 16/16 승인 후 PAUSE·Park 종료, raw를 보존했고 v132의 새 manifest/output으로 분리했다.
4. police1은 native GT는 있으나 ROS Object 배열에 없어서 pivot을 검증할 수 없다.
   정적 measured parent reference의 유일한 미확인 자산으로 유지하며 GT/자산을 제외하지 않는다.
   v132 dedicated alongside의 geometry 미확인 primary는 0개다. 이동/정지 횡단은 정지 reference override 대상이 아니다.

## 동결 스냅샷 / 계획 부모 객체 노출

2026-10-09 v132에만 적용한다. v123의 10079/60474 및 v129의 10081/60486 표는 역사적 스냅샷이다.
새 yaw·형상·충돌 전 배치 재표본화로 secondary 개수는 달라졌다. **실제 가시 GT 수가 아니다.**

| 객체군 | Sunny13 5K 계획 | 같은 6 환경 30K 계획 |
|---|---:|---:|
| 일반 차량 |4234|25404|
| 실체 장애물 |3296|19776|
| 일반 사람 |2343|14058|
| 인체 더미 |146|876|
| 동물 |150|900|
| 반려동물 동반 사람 |156|936|
| 유모차 |140|840|
| 자전거·스쿠터 등 탑승자 |155|930|
| 차량형 Obstacle |118|708|
| 합계 |10738|64428|

- 기본 입력 `scenarios_5k/manifest.json` SHA256:
  `f99c1541abb0ff7f4b445a20d11750038a78406ac11c991164c8f6791cfdd0e9`.
- Sunny13 순서입력 SHA256:
  `89b053e7332e20f284c2de28d2f2365a3a9cf157d017c628481d9715ec5a1a77`.
- 2810셀 재생성/2190payload 보존. 보호된 고주로 1340/로터리 350/사거리 250/빈배경 230 및 음영빈배경 20은 그대로다.
- 122종 primary 횟수는 v129와 정확히 같고 최소 5회다. whole-link split 재시도로 89셀이 바뀌었으며
  새 6 환경 안에서는 동일 split/group이다. 구 raw를 자동 혼합하지 않는다.
- 전수 5K OFFLINE_PASS/고유 5000/오류 0. 실제 serialized yaw·균형 sector·정적 geometry 적용·6 환경 hash/group/split/
  정적우선순서·음영 approach/recede를 별도 `validation_offline.json`으로 검사했다.
- Base5 전수 단위검증 및 실제 MR 대표 55개 최종 결과/CPU 변환/resume/비용은 아래에 기록한다.

## 최종 live 및 운영 확인

### 최종 v132 결과

- 사전 등록 55셀을 새 output에서 전부 실행해 **55/55 승인/부족 0**으로 완료했다.
  정지 48/실제 이동 7이며 고주로/로터리/사거리/배경 control을 포함한다.
  초기 실패 및 추가 높이 준비/제한적 재배치도 포함한 610.850초다. 모든 장면이 첫 캡처에서 통과했다는 뜻은 아니다.
- 승인 부모 배치 90/가시 positive parent 86/native positive component 107이다.
  CPU 변환 55프레임/107GT: Vehicle21/Pedestrian40/Obstacle27/Animal19, train 51/val 4.
  visibility로 제외한 9component는 raw에 보존하며 안 보이는 객체를 정답으로 강제하지 않는다.
- selected raw에서 실제 정지·CollisionData 무접촉·review없음을 검사했다.
  실제 annotation 기준 근접 간격: OBJ차량 0.302~0.473m, 동물 0.351~0.372m,
  유모차 0.348~0.836m, 탑승자 0.349~0.669m. component별 간격이므로 parent envelope 간격과 다를 수 있다.
  사람이 0.715m, pet-person0.880~1.107m였으며 내부 collider/spawn 임계값 인증은 아니다.
- 안전 종료 45session 모두 Park 확인. 완료 ordinary resume은 추가 촬영 0, sensor1858파일
  SHA/size/inode/mtime 불변·selected ledger/누적시간/181event요청/45timing파일 불변이다.
- 전체 Base5 단위검증 416개 PASS. 본 5K와 모델 학습은 시작하지 않았다.
  representative55 성공을 5K 전수성공·전 asset접지/collider·학습성능 인증으로 해석하지 않는다.

### v131 실패 대조와 한정 보완

v131의 실제 MR55셀은 53 승인/2 부족이었다. 완료 후 2회 보충에서도 초근접 Ped_Pet1 좌우 2셀이 남았다.
좌측 간격 대조 .30/.45/.60은 누락, .80은 캡처됐으며 1.0m 추가 대조도 승인됐다.
우측의 .80/1.0m 대조 2개도 승인됐다. 모든 대조는 원래 yaw/재생성 seed를 유지한 matched gap 실험이다.
v132는 정적 alongside pet_person primary 3셀만 요청 gap .8–1.0m로 변경한다.
원래 bin·자산·좌우·phase·yaw sector를 보존하고 native GT/충돌 gate는 변경하지 않는다.
이 보완은 내부 collider 크기 확인이 아니라 실제 spawn 근거에 따른 자산 제한 prior다.

v131→v132의 실제 scenario payload 변경은 `b5v2_00191`, `b5v2_01740`, `b5v2_03227`의 3개뿐이다.
새 55셀 pilot에는 앞의 좌우 2셀이 포함되며, 나머지 1셀도 `compound_third_cell`의 별도 bounded 대조로
1/1 승인/부족 0·Park 종료를 확인했다(8.758초). 따라서 수정 3셀 모두 실제 새 배치에서 캡처됐다.

### 비용·디스크·백업

- `pilot_cost.json`은 정적 4822/사거리 이동 154/음영 이동 24를 각각 가중한다.
  장당 정적 6.719초/사거리 7.548초/음영 46.796초, 가중 외삽 5K9.635시간이다.
  사거리 표본 1개/음영 6개이며 의도적인 어려운 strata 표본이다. 처리량 보장이나 confidence interval이 아니다.
  **운영 예산 5K10~13시간, 같은 6 환경 30K60~78 시간**을 유지하고 첫 100~200 승인 rate로 갱신한다.
- 실패/QA·metadata를 포함하고 hardlink를 한 번만 계산한 저장량의 조건별 가중 외삽은
  **5K32.572GiB/30K195.429GiB**다. 어려운 이동 표본을 전체로 단순 외삽한 51.09GiB/5K는 권장 예상값이 아니다.
  변환캐시·전달압축복제·학습·다른작업 저장공간은 별도이며 다음 환경마다 실제 디스크를 확인한다.
  대시보드 용량은 본 수집 누적 실측으로 갱신한다.
- `frozen_input_v132.tar.zst`에 시나리오/spec/config/suite/placement reference 및 수동 높이/로터리 reference를,
  `runtime_v132_source_reference.tar.zst`에 scripts/tests/센서·네트워크를 보존했다. `tar -df` 전수 차이 0 확인.
  입력 SHA `fd7b3d82fc066470d17c7af7302fdc5d21494341c7353edeec60c57f06df886c`,
  runtime SHA `08c4decebef517e8e3732c4ef83ae9ec8a7f8201e9b75585211bd025a2028e52`.
  기존 지도/자산/source51참조는 `prebulk_dashboard_v126/external_references_snapshot.tar.zst`도 유지한다.
- 8075는 v132 본 수집 root의 `not_started/5000/6환경30000/경고0`을 확인했다. 본 수집 output은 아직 없다.

## 재현 파일 / 다음 단계

- 새 도구: `revise_base5_final_appearance.py`, `calibrate_base5_parent_envelopes.py`,
  `prepare_base5_geometry_calibration.py`, `report_base5_final_appearance.py`, `base5_placement_geometry.py`.
- release의 `frozen_specs.json`, `config.json`, `placement_geometry.json`, `revision_report.json`,
  `pilot_manifest.json`, `pilot_strata.json`, `offline_audit.json`, `validation_offline.json`을 함께 보존한다.
- 새 runtime으로 구 v123/v129/v130/v131 output을 resume하지 않는다. 성공/실패 raw와 backup은 보존한다.
- 승인 후 [단일 운영 런북](261007_base5_5k_capture_runbook.md)의 v132 새 manifest/output으로 Sunny13 5K를 시작한다.
  대시보드와 완료 후 bounded same-quota 보충은 유지한다. 본 수집 후 실제 가시 자산 coverage를 검사한다.
- threshold는 새학습 후 mission rosbag에서 선정한다. 임의 yaw가 모델 성능을 개선했는지 지금 단정하지 않는다.
