# Base5 범용 크기 예산 / 미션 상황별 개수 — v89 구현·검증

2026-10-07. 현재는1번 보완의 prototype이며 새10K 본수집 승인이 아니다.
최종 후보[5개 gate](261007_base5_final_candidate_five_gates_plan.md)의 나머지2–5번은
이 작업만으로 해결되지 않는다. raw annotation·production·threshold는 미변경이다.

## 확정한 역할 분리

범용6,500(배경500 포함) / 미션3,500 및 원래primary family·subfamily·거리·방향
quota는 유지한다. 범용의positive6,000장에만 크기 예산을 사용한다.

| 범용 mode | 6,000장 비율 | 장면 수 | 규칙 |
|---|---:|---:|---|
| 단일 | 30% | 1,800 | primary1개, 예산 채우기 없음 |
| 일반 혼합 | 50% | 3,000 | 총10–14점,2–8개 |
| 밀집 혼합 | 20% | 1,200 | 총16–22점,2–8개 |
| 별도 빈 배경 | — | 500 |0개 |

각family 안에서 정확한 mode deck을 배정한다. 이 숫자는8개quota를 강제하지 않는다.
작은 자산이 많으면 개수가 늘고 큰 자산이면 적은 개수로 예산을 채운다.
primary는원래 asset_index 순환을 보존하고, 보조는같은pool60%/vehicle10%/
obstacle20%/person10% 후보군을 남은 예산에 맞춰 선택한다. 비용 제한으로 실제
보조 클래스 비율은 원래 후보 가중치와 달라질 수 있으므로 결과를 집계한다.

비용은geometric reference의XY footprint면적과최장변을 함께 사용한다.

| 조건(면적·최장변 모두 충족) | 비용 |
|---|---:|
| ≤1m²·≤1.5m |2 |
| ≤3m²·≤3m |3 |
| ≤14m²·≤6m |6 |
| 나머지 |8 |

geometry가미확인인자산은기존보수적proxy로계산하고미확인임을남긴다.
실제자산작음/접지/가시성의인증값이아니다. 점수는LiDAR point가아니다.
예산충족불가능이면배치를재샘플링하며상한을깨거나무한히객체를추가하지않는다.

## 미션은 개수 목표가 아니라 상황을 유지

| 미션family |1개 |2개 |3개 |
|---|---:|---:|---:|
| 고주로 |50% |35% |15% |
| 로터리 |40% |40% |20% |
| 정적 차량 회피 |80% |20% |0% |
| 장애물 회피 |75% |20% |5% |
| 보행자/동적·복합 |80% |20% |0% |

generic예산을채우기위한보조객체를추가하지않는다. 보조는primarypool에서고르고,
차량은기존합법도로경로/고주로mission sample corridor를유지한다.
현재이표는상황별count초안이며고주로추월전후·로터리bumper간격은아직기존
거리deck이다. **3번 초근접gate 구현으로 혼동하지 않는다.**

## 방향·가시성·안전

- 범용 layout요청은분산60%/동일방향군40%. 분산은보조방향을순환하고
  동일방향군은primary관측sector에두되보조거리구간을넓힌다.
- initial좁은3–5m ring에다수movingactor를모으는방식은512번retry해도
  채울수없는셀이발생해폐기했다. primary거리quota는유지하고보조는최소15m폭의
  구간으로배치한다. 보조도같은거리bin이라는뜻이아니다.
- routevehicle은layout요청보다합법차선을우선한다. 따라서실제방향분산률을
  요청60/40과같다고주장하지않는다. actual_ego_role로별도감사한다.
- 모든actor기존접지/충돌/ROI/동적경로/가시성gate를유지한다.
- `population_visibility`는ROI안positivecomponent가있는**parent actor 수**와
  component수를분리한다. 유모차3GT를객체3개로세지않는다.
- 후속v92에서는범용혼합에최소2개·배치actor의절반이상(올림)이ROI안positive로
  보이도록hard gate를추가했다. 부족하면`population_visible_actor_deficit`로
  재시도/부족분처리한다. 전부보이도록강제하지않아가림학습은유지한다.
  복합child개수로parentquota를채우지않는다. 미션/단일/배경은이추가gate의대상이아니다.
- retry/replacement는primary/quota/split/mode/예산구간을유지한다.
  budget방식이므로replacement의보조구성·실제개수는달라질수있다.
- helper hash를생성provenance와campaign/resume safety계약에포함한다.
  이전v86output강제resume금지,새output만사용한다.

## 오프라인 결과

`/home/luv/morai_cap/base5_v2_261007/` 아래에실측자료를보존한다.

- `population_v88_offline128` + `population_v89_offline384`:합계512장,
  양쪽OFFLINE_PASS/오류0. payload중복0,1–8개/배경0개생성확인.
- 512장평균resample1.35,최대62. 10K전수감사/실측throughput가아니다.
- 실제7개장면2개,8개장면1개도생성했다. 아직해당3장live인증은아니다.
- 초기실패`population_v87_offline96*`도이력으로보존한다.
- 분포report: `population_v89_report_offline.json`.
- MR 소량capture/변환/완료resume최종수치는아래후속결과에기록한다.

## 실제 혼합 smoke에서 확인한 복구 오류

- `population_v89_live`:6조건요청,2승인후6actor조건에서133.17초에안전중단.
  3actor/4actor승인장면은각각3/3·4/4 parent가positive이고후자camera-only1개도유지했다.
  CPU변환2장/GT7(Vehicle1/Pedestrian1/Obstacle5), raw미수정이다.
- 중단원인은QA view에없는shutdown_safety.json을읽던near-height연결오류다.
  `parked_source_row`는QA/raw hash를검증하고원래실제session의Park/ledger까지
  추적한다. QA에가상Park파일을만들거나기존session검사를면제하지않는다.
- `population_v90_live`에서는같은다중QA중메모리ground hull tuple와저장JSON list를
  다르게비교하는오류가재현됐다. 실제재계산의JSON값은원래audit와정확히같았다.
  새reaudit반환값을저장값과동일한JSON형태로정규화하고연쇄tuple회귀를추가했다.
- 실패raw/ledger는보존하고halted output을강제resume하지않았다.
  실패실데이터의QA→near height오프라인재현도`population_v90_offline_repro`에서통과했다.
- 최종회귀207 tests PASS. 수정후실제재검증은`population_v91_live`의새output이다.

## v91 결과와 v92 가시성 하한

- v91 재검증4/4통과. 6개정적혼합보행자셀은재배치후3개중2개positive,
  6개장애물혼합은6개중4개positive였다. 두mission은각1개중1개다.
  보충시예산quota가고정되며배치개수자체는고정아님을실제로확인했다.
- CPU변환4장(train2/val2),GT8(Vehicle2/Pedestrian1/Obstacle5),
  invisibleObstacle3component는positive제외/raw보존. invisibleGT를억지로학습시키지않는다.
- 이관측후v92에서위의가시성하한을추가하고새32장JSON을생성했다.
  `population_v92_final32`/`population_v92_live`는최종하한검증용새경로다.
  기존v89/v90halted output 및코드변경전v91을강제resume하지않는다.

## 최종 v93 live / 변환

- 가시성 하한을 접지용 단독 calibration에까지 적용하던 v92 연결은 수정했다.
  calibration_only는 학습 quota를 채우지 않으므로 이 하한에서 제외한다.
  primary 가시성·ground·CollisionData·Park 검사는 그대로 유지한다.
  v92는 STOP으로 정리했고 새 v93 출력에서 재검증했다.
- `population_v93_live`: 최종4/4,70.761초,부족0/종료complete.
  범용3개→positive3개,범용4개→positive4개(그중camera-only1개),
  장애물회피1개→1개,고주로mission1개→1개다.
- `population_v93_infos`: CPU변환4장(train3/val1),GT9
  (Vehicle2/Pedestrian1/Obstacle6),GT제외0/raw미수정.
- 완료resume는 검증 전용이며 selected ledger SHA 불변을 확인한다.
  재학습/OD추론/모델성능 향상 검증은 하지 않았다.
- 최종 회귀207 tests PASS, py_compile/diff check PASS.
- 실제6actor→4visible 결과는v91에서확인했고,7–8개장면은아직오프라인검증만했다.
  모든자산·애니메이션·밀집조건이live인증됐다는뜻이아니다.
- 512장 generation·최종32장 감사는소량smoke다. 수정정책으로10K를생성하거나
  본수집하지않았다. 비용증가·geometry proxy·dense 동적지형지원제한은 남는다.

## 재현

```bash
python3 dataset/morai_od/scripts/generate_base5_independent.py \
  --output "$HOME/morai_cap/base5_new_population_pilot" --count 64
python3 dataset/morai_od/scripts/audit_base5_scenarios.py \
  --manifest "$HOME/morai_cap/base5_new_population_pilot/manifest.json" \
  --output "$HOME/morai_cap/base5_new_population_pilot/offline_audit.json"
python3 dataset/morai_od/scripts/report_base5_population.py \
  --manifest "$HOME/morai_cap/base5_new_population_pilot/manifest.json" \
  --output "$HOME/morai_cap/base5_new_population_pilot/population_report.json"
```

live 검증은 [기존 운영 런북](261006_base5_bulk_resume_v45.md)의 guarded bridge/
Park/CollisionData/Capture Mode를 사용한다. 구 manifest 본수집 명령은 실행하지 않는다.
次はlive可視率・保守的proxyによる偏り・多actor地面QA費用を見て
この初期policyを採用/修正し、残る2–5番gateを進める。
