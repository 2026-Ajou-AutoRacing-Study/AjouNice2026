# 작은·가는 물리 장애물의 명시적 거리 정책: v152

문서 상태: EVIDENCE / QUALIFIED SUN13. 근거 확인일은 2026-10-15이다.
전체 5K CPU 감사와 아래 소량 native 검증을 마쳤다. 새 Sunny13 release는 `bulk_ready=true`이나 본수집은 실행하지 않았다.
완료 Sunny13 5K, 기존 v151 원본·실패 증거·release, E24 가중치는 변경하지 않는다.

## 변경 이유와 범위

v151에서 남은 원거리 주 객체의 실패는 Z 오류나 충돌이 아니라 센서 근거 부족이었다.
추가 actor의 전방 지원과 주 객체의 관측 가능성은 별개이다. 멀리 있는 가는 물체를 같은 거리 계약으로
반복 재배치하는 대신, 자산의 반복 native 치수에 따라 새 배치 명세의 거리를 제한한다.
특정 실패 ID나 자산 이름을 예외로 처리하는 정책이 아니다.

이 정책은 **새 배치의 주 객체 거리 분포**를 바꾼다. 모델의 인식 ROI, 기존 GT,
평가 거리, tracker 출력 범위를 자르거나 원거리에서 보이는 정답을 삭제하는 기능이 아니다.
원거리 Vehicle을 보강하려는 계획도 유지한다.

적용 조건은 다음을 모두 만족하는 경우이다.

- `family=obstacle`, `mission=false`이며 `shadow_phase`와 `avoidance_phase`가 없다.
- 주 객체가 정적으로 배치되는 `objectList`의 물리 장애물이다.
- Catalog의 `semantic_class=obstacle`이다. 같은 `objectList`라도 OBJ 차량이나 인체 더미는 이 조건으로 바꾸지 않는다.
- 반복 native 크기 기준이 있고, 기존 명세의 상한이 해당 자산의 새 상한을 넘는다.

정적 회피의 표면 간격 solver, 범용 초근접 회피 관측, 음영, 실제 이동 경로와 미션 배치는 유지한다.
보조 actor에는 이 정책을 직접 적용하지 않는다. 기존 v151 전방 지원과 보호된 근·중거리 보조 정책은 유지한다.

## 거리 분류에는 배치용 envelope가 아닌 native 반복 치수를 사용한다

완료 Sunny13 5K의 19종 물리 장애물에서 정적·scale 1·단일 component parent 조건을 확인한
native 관측 3,299개를 사용하였다. 관측이 포함된 scale 1 시나리오는 1,637개이다.
3,299는 프레임 수나 학습 GT 수가 아니다. 512 입력의 최종 GT에서 제외된 component도 치수 조사에는 포함한다.

Native full RPY를 사용하되 parent yaw를 제거한 축별 enclosure를 구하고,
각 자산의 X·Y·Z 치수를 각각 중앙값으로 요약하였다. 자산의 방향을 바꿨다는 이유로
크기가 크게 부풀려지지 않도록 하는 것이다. 회전·기울기를 무시한 임의의 크기 재정의는 아니다.

기존 placement envelope는 배치 간격·footprint·지면 초기화에 쓰는 별도 자료이다.
여러 관측을 보수적으로 감싼 값과 실제 반복 치수는 목적이 다르다. 기존 envelope나 collider 간격을
이번 native 중앙값으로 대체하지 않는다. Native annotation과 학습 GT도 수정하지 않는다.

| 예시 자산 | v151 배치용 추정 치수 X/Y/Z | 반복 native 중앙값 X/Y/Z | 새 상한 |
|---|---|---|---:|
| Lane_Separation_Bar | 0.292 / 0.283 / 1.338m | 0.213 / 0.213 / 0.752m | 20m |
| Bollard01 | 0.457 / 0.440 / 1.596m | 0.369 / 0.370 / 0.856m | 30m |
| Bollard04 | 0.446 / 0.482 / 1.328m | 0.370 / 0.370 / 0.856m | 30m |
| Concrete_Barricade01 | 0.442 / 2.172 / 0.920m | 0.393 / 1.921 / 0.821m | 40m |

배치용 수치는 v151 metadata의 예시이며 native 중앙값과 같은 통계량이 아니다.
이 표는 모든 envelope가 과도하다거나 모든 축을 줄여야 한다는 뜻이 아니다.

## 실제 적용 규칙

`d=min(X,Y,Z)`, `w=max(X,Y)`는 위 반복 native 중앙값에서 계산한다.

| 먼저 만족하는 자산 치수 조건 | 주 객체 거리 상한 |
|---|---:|
| `d < 0.25m` | 20m |
| `d < 0.35m` 또는 `d < 0.40m`이면서 `w < 1.5m` | 30m |
| `d < 0.60m` | 40m |
| 나머지 | 이 정책으로 제한하지 않음 |

두 번째 조건의 compact 분기는 가늘고 작은 자산을 위한 일반 조건이다.
길이가 약 1.92m인 Concrete_Barricade01을 작은 Bollard와 같은 30m 분기로 묶지 않는다.
숫자는 현재 native 근거를 반영한 v152 후보값이며 모든 환경의 관측 가능성을 보장하지 않는다.

기존 명세가 `[lo, hi)`, 새 상한이 `cap`이면 다음과 같이 명시적으로 바꾼다.

- `hi <= cap`이면 원래 spec·payload·metadata를 그대로 복사한다.
- `hi > cap`, `lo < cap`이면 `[lo, cap)`으로 바꾼다.
- `hi > cap`, `lo >= cap`이면 `[cap-10, cap)`으로 바꾼다.

`distance_bounds`, `distance_bin`, `primary_obstacle_range` plan과 필요한 frozen actor ID 목록을
새 spec에 기록한다. 생성기 내부에서 실제 거리만 몰래 줄이지 않는다.
Native audit도 이 새 명세의 실제 GT 중심 거리를 기존 허용 오차와 함께 검사한다.

## 63개 예비 집계와 최종 53개 계약의 차이

초기 넓은 범위 감사에서는 상한을 바꿀 수 있는 주 객체 명세가 63개였다.
그중 10개는 `avoidance_phase`가 있는 범용 초근접 관측이었다. 이들은 중심 거리 명세를
단순히 줄이는 대신 표면 간격 solver를 사용하므로, 이번 엄격한 적용 범위에서 제외하였다.
따라서 최종 대상은 **53개**이다. 예비 63개 자료는 수정하지 않고 이 범위 차이를 구분한다.

| 최종 CPU 후보에서 확인한 항목 | 결과 |
|---|---:|
| 명시적인 거리 plan이 생긴 장면 | 53개 |
| 상한별 명세 | 20m: 22개 / 30m: 7개 / 40m: 24개 |
| 새 `[lo, hi)` 밖에 있어 실제 geometry를 다시 생성한 장면 | 33개 |
| 이미 새 `[lo, hi)` 안이어서 payload를 보존한 장면 | 20개 |
| 전체 5K에서 scenario payload가 원본과 동일한 장면 | 4,967개 |

20개는 새 spec·plan·frozen ID를 metadata에 기록하되 기존 scenario 바이트를 그대로 보존한다.
상한만 비교하지 않고 하한도 확인하며, 상한과 같은 위치는 반개구간 밖으로 처리한다.
명세가 바뀌지 않은 나머지 장면은 scenario뿐 아니라 metadata도 바이트 단위로 복사한다.

33개 재생성 장면은 원래 허용 spatial window와 split 안에서 전체 배치를 다시 만들 수 있다.
따라서 같은 장면 안에서 Ego나 보조 객체의 좌표가 완전히 고정되었다고 표현하지 않는다.
유지하는 것은 자산 정체성·수·동작·seed·방향 계약·yaw 정책과 허용 공간 계약이다.

## 보존 및 복구 계약

전체 5K CPU 감사에서 actor 정체성과 기존 front allocation의 전후 digest가 일치하였다.
기존 actor의 DataID·runtime UID·이름·container·population pool·motion, 장면 split과 seed를 보존한다.
Free yaw와 기존 primary sector, 연속 Ego의 spatial plan도 바꾸지 않는다.
Vehicle primary 2,214장·59종은 scenario와 metadata가 모두 바이트 단위로 동일하다.
그중 30m 이상은 1,003장, 50m 이상은 587장이다. 두 숫자의 차이 416장은 30~50m 구간이며
서로 다른 거리 정의를 같은 원거리 분모로 혼용하지 않는다.
v151의 추가 장애물 672개와 전방 선택 338개·기존 배치 334개 할당도 유지한다.

Plan에는 helper SHA, 명시적인 config, native 크기 reference의 경로·SHA, 자산·치수·거리,
전체 frozen actor 목록과 기존 front plan의 digest를 기록한다.
재시도는 같은 plan과 seed를 유지하고 새로운 retry 구간에서 geometry만 제안한다.
다른 donor의 range/front plan을 가져오거나 부족한 subset에서 할당을 다시 계산하지 않는다.

Range-only 보충은 최대 512개 CPU 제안 중 유효 후보 8개를 비교한다.
이미 사용한 payload와 사실상 같은 관측을 제외하며, 최종 승인은 기존 native QA가 결정한다.
이는 512번 촬영한다는 뜻이 아니다. 기존 유한 시도·시간 상한, 실제 정지·Park·통신·CollisionData 기준을 유지한다.

`RangeGeometryExhausted`만 일반적인 제안 소진으로 처리한다.
Helper·config·reference·자산·명세 불일치는 `RangeContractError`로 중단한다.
이 오류는 `RuntimeError` 계열이므로 기존 front adapter의 일반 `ValueError` 처리에 흡수되어
단순한 부족분으로 바뀌지 않는다. 부분·완료 resume도 원본과 기존 성공분을 수정하지 않아야 한다.

## 현재 후보와 재현 근거

실험 root는 `artifacts/evaluation/base5_obstacle_range_261015`이다.

| 자료 | 의미 |
|---|---|
| `range_size_reference.json` | 정적 scale 1 native 반복 치수. SHA `890dec2c18ce9e5067424a04285417482e2ac98141c9afc40462440deffdc781` |
| `v151_primary_inventory_final_rule_comparison.json` | 넓은 범위의 예비 비교 자료. 최종 no-avoidance 53개 계약과 구분한다. |
| `candidate_v152b/manifest.json` | 현재 후보 5K 명세. SHA `bc5ba39b34a864b08570e945db303b644076cc11e21db557bd7ea5c1e5315a44` |
| `candidate_v152b/frozen_specs.json` | 실제 새 명세 전체의 동결본 |
| `candidate_v152b/preservation_audit.json` | 53개 명세·33개 geometry·20개 payload 보존과 source/hash 감사 |
| `candidate_v152b/primary_range_delta.json` | 변경 자산·원래/새 거리·재생성 여부·계보 |
| `candidate_v152b/scenario_audit.json` | 5,000장 `OFFLINE_PASS`, 오류 0개 |

첫 `candidate_v152`는 계약 오류 예외의 전파 방식 보완 전 CPU 결과로 보존한다.
현재 helper와 hash가 일치하는 새 후보는 `candidate_v152b`이며, 과거 output을 현재 코드로 강제 resume하지 않는다.

구현은 [거리 정책 helper](../scripts/base5_primary_obstacle_range.py)와
[새 후보 준비기](../scripts/prepare_base5_primary_obstacle_range.py)를 따른다.
완료 5K에서 정답을 사후 삭제하거나 본수집을 실행하는 도구가 아니다.

## 거리 조정만으로 해결되지 않은 완전 가림

거리 계약 53개를 처음 전수 확인했을 때 원 payload의 native 저장은 53회였고,
44회 승인·9회 review였다. 일반 품질 복구 15회 중 8회가 승인되어 최종 52/53장이 되었다.
모든 최초 53장에 실제 raw와 audit가 있으므로 중간 진행 로그를 보고 미촬영으로 세지 않는다.
같은 payload를 다시 저장한 경우도 있어 저장 68회와 고유 payload 65개는 다른 수이다.

남은 `01762`의 CargoBox는 명시적 `[30, 40)` 계약 안에서 새 배치 세 번을 더 촬영했지만
모두 LiDAR 0점·native camera instance 0px로 제외되었다. 성공 52장과 이전 실패 raw는 보존했다.
이 시점까지 총 native 저장은 71회이며, 추가 세 번은 실제 부족분의 유한 보충이지 계획된 probe가 아니다.

원본·일반 복구·새 보충의 일곱 관측에서 가까운 Pacifica가 CargoBox 앞에 있었다.
관측한 native 3D OBB의 27개 표본으로 대조하면 전방 카메라와 LiDAR에서 표본으로 향하는
모든 선분이 차량 OBB를 먼저 통과했다. Native 위치와 계획 위치는 mm~cm로 일치하고
예상 이미지 영역도 FoV 안이었다. 지면 평면은 미측정이므로 Z 오류가 없다는 인증으로 바꾸지 않는다.

기존 후보 점수도 세 번 모두 primary의 `lidar_beam_proxy=0`, `angular_occlusion_proxy=1.0`을
기록했지만, 다른 actor 세 개의 지원으로 전체 점수가 높아 후보로 선택되었다.
XY footprint 가림은 높이를 증명하지 못하므로 기존 구현은 의도적으로 soft penalty를 사용했다.
따라서 XY proxy가 1이라는 이유만으로 모든 장면을 버리는 수정은 적절하지 않다.
이번 보완은 높이·센서별 시차를 고려한 3D 후보 검사로 명백한 완전 가림만 피하는 방향이다.
부분 가림 학습을 없애거나 실제 native QA 실패를 승인으로 바꾸는 기능이 아니다.

별도 PE_Drum은 약 37m의 비슷한 거리에서 차량 가림이 사라지자 2점·123px를 확보했다.
이는 CargoBox 한 ID만의 예외보다 혼합 배치 시선 차폐의 공통 원인을 뒷받침한다.
OBB는 mesh ray 검사와 다르므로 예측·실제 mask·센서 점을 구분한다.

52개 승인 primary의 LiDAR 점 수는 중앙값 1점·최대 11점이었다. 거리 상한은 관측 가능성을
높이는 잠정 분포이지 풍부한 LiDAR 형상을 보장하는 규칙이 아니다. 이 코호트를 근거리의
선명한 학습 사례나 detector AP 개선으로 표현하지 않는다.

### 3D 가림 후보 선택 보완과 실제 보충

같은 source의 CPU 후보를 다시 비교하니 세 번 모두 기존 첫 8개 후보 안에 덜 가린 배치가 있었다.
512개 retry 중 유효 후보 359개에 가림이 적은 후보 83개가 있었으므로, 새 자산이나 다른 거리로
바꾸지 않고도 같은 계약 안에서 개선할 여지가 있었다. 단순히 재시도 횟수를 늘리는 문제와 구분한다.

[전용 후보 검사](../scripts/base5_primary_visibility_proposal.py)는 range plan이 있는 정적 물리 주 객체의
fresh completion에만 적용한다. 기존 front plan과 겹치는 경우도 같은 callback을 사용한다.
일반 legacy·미션·복합체·원거리 Vehicle 배치나 초기 5K의 저장 payload를 바꾸지 않는다.

- primary의 예상 LiDAR beam이 0일 때만 보조 검사를 한다.
- primary OBB는 표면별 5cm 확대하고, foreground 차체 OBB는 표면별 10cm 축소한다.
- 알려진 geometry·높이·도로 지지가 있는 정적 `vehicleList` 중 길이 3m·폭 1.4m 이상 차체만 blocker로 본다.
- primary를 볼 수 있는 모든 카메라와 LiDAR에서 27개 표본이 같은 가까운 차체에 전부 가려질 때만 후보를 제외한다.
- 부분 가림, 위로 보이는 높은 객체, 센서별 시차, 불확실한 높이, 사람·복합체의 빈 공간은 이 조건으로 제외하지 않는다.

카메라 FoV는 기존 수집 proxy의 320×896 bottom-crop 좌표를 유지한다. 학습 입력 크기나
native 가시성 기준을 변경한 것이 아니다. 후보 OBB는 upright 추정이며 실제 mesh의 폐색 인증이 아니다.
표본 하나라도 차폐가 불확실하면 기존 점수를 유지하고, 최종 승인은 native QA로 결정한다.

새 검사로 실패한 CargoBox 7개를 모두 후보에서 제외했고, 실제 승인 52개는 하나도 제외하지 않았다.
별도 static64까지 합친 범위 내 승인 54개도 유지되었다. 단위 검사에는 부분 가림·높이·시차·불확실성을 포함한다.
590회 점수 대조의 추가 CPU 비용은 평균 약 0.102ms, p95 약 0.740ms였다. 메타데이터 I/O와 배치 생성 비용은 제외한 값이다.

최신 구현의 새 output에서 같은 1장 부족분을 보충하자 **첫 native 시도에 승인되어 53/53**이 되었다.
CargoBox는 32.516m에서 LiDAR 2점·native camera 최대 136px였고, 더 이상 앞 차량 뒤에 숨지 않았다.
기존 성공 52장의 선택 row와 원본은 그대로 유지했다. 앞선 실패 3회의 journal은 고치거나 재개하지 않았다.
이는 알고리즘을 보완한 새 실험이며 기존 시도 상한을 반복해서 초기화하는 운영 방법이 아니다.

## Native 최종 결과와 승격 판정

| 검증 항목 | 최종 결과 |
|---|---|
| 정적 64장 회귀 | 최초 62승인 + 일반 복구 2승인 = 최종 64/64. 실제 저장 66회 |
| 정적 부모의 후속 completion | 실제 부족분 0·native 추가 0. 실패를 만들거나 성공분을 빼지 않음 |
| 실제 이동 8장 회귀 | 최종 8/8. 교차로 4·음영 4, 실제 이동 witness·무접촉·준비 포함 27세션 Park 확인 |
| 거리 plan 53개 전수 | 최초 44/53 → 일반 복구 후 52/53 → 최신 선택기의 실제 보충 1회로 53/53 |
| 기존 실패 보존 | 선택기 보완 전 보충 3회는 모두 review로 보존. 새 알고리즘 output과 구분 |
| 19종 전방 선택/비선택 stratum | 38/38. 기존 긴 정지 NPC 24/24도 원래 계획 payload·정지 witness 확인 |
| 독립 대체 경로 probe | 4/4 승인. range-only/front 중첩 × cap20/30plus 사전 등록, 훈련 데이터나 실제 부족분으로 세지 않음 |
| 부분 resume | 첫 16장의 선택 row·raw SHA/inode/mtime/size·기존 제어 이벤트 보존 |
| 완료 resume | 정적 부모·zero completion·range 부모·실제 보충·4 probe 모두 native 추가 0. raw·선택·state·누적시간 보존 |
| 전체 CPU 검사 | 857개 통과, 실패·오류 0. `git diff --check` 통과 |
| 새 release | `release_v152_sunny13/release.json`, `bulk_ready=true`, blocker 0, `bulk_started=false` |

이 검증이 끝나더라도 detector AP 향상이나 모든 환경의 원거리 관측 가능성을 인증하는 것은 아니다.
다른 다섯 환경에는 독립 geometry 생성과 해당 환경 검증을 별도로 수행한다.

정적 승인64의 지면 actor255개 중 161개는 로컬 평면으로 gap을 측정했고 94개는 미측정이다.
경고95개는 원래 정적 advisory 정책대로 보존했으며 심각한 지면 QA 실패는 없다.
측정 gap은 −7.59~+16.87cm, Ego 캡처 Z span 최대는 약 0.48mm였다.
이동8의 관측 Z step 최대는 약 0.767mm다. 지면 미측정·안정된 Z·mesh 접지는 서로 다른 증거이다.
이동 QA의 review4건과 대체 선택2건도 보존했으며 처음부터 모든 준비가 성공했다는 뜻은 아니다.
4개 probe에는 cap30 별도 복구가 없고, cap30의 7개 명세는 53개 전수 native에서 확인했다.

완료 재실행 때 부모 status의 `updated_at`만 갱신되었다. 당시 보고서의 status SHA와 현재 SHA는
별도 refresh receipt로 연결한다. 원본 파일이나 과거 보고서를 옛 SHA에 맞추려고 고치지 않는다.
부족분이 없는 completion은 자체 raw가 없는 것이 정상이다. v152 검증기만 이 경우를 명시적으로
분기해 부모의 실제 raw, 동일 selected64, 빈 deficits/attempts, native/control0, source/runtime,
부분16·완료 snapshot·state/status·누적시간을 확인한다. 기존 v151 검증기는 변경하지 않았다.

동결한 runtime은 81개, source/input/native evidence는 18,463개다. 실제 launcher의 읽기 전용
preflight를 확인했으며 native driver는 호출하지 않았다. 완료 Sunny13 canonical ledger SHA는
기존 `026e6244aa311f6342429a30893694ce646b49222ab354f97cfa08539df74a43` 그대로다.
학습·E24·추론 compile OFF·LiDAR50·planning/tracker 설정은 바꾸지 않았다.

## 비용과 실행 기준

동일 64+8 승인 표본의 QA·거절 비용을 포함한 그룹 가중 예상은 새 5K **약 7.95시간 / 30.43GiB**다.
정적 약 5.27초, 사거리 이동 약 12.05초, 음영 이동 약 55.35초/승인 장면을 사용했다.
거리 전수53·별도 보충·진단 probe 비용은 qualification 비용으로 따로 기록하며 이 가중 평균에는 넣지 않는다.
운영 예산은 8~12시간 / 45GiB를 유지하고 본수집 누적 rate로 갱신한다.
v151의 9.75시간은 별도 실행·실패 비용을 가진 값이므로 차이를 순수 코드 속도 개선율로 해석하지 않는다.

새 release의 기본 사전 검사는 아래와 같다. `--run`이 없으므로 본수집을 시작하지 않는다.

```bash
ROOT="$PWD/artifacts/evaluation/base5_obstacle_range_261015"
python3 dataset/morai_od/scripts/launch_base5_obstacle_support.py \
  --release "$ROOT/release_v152_sunny13/release.json" --environment sunny13
```

본수집은 사용자가 별도 승인한 뒤 MR의 Sunny13·Capture Mode·센서 Instance/GT·현재 네트워크와
디스크 여유를 다시 확인하고 `--run --confirmed-environment sunny13 --complete`를 명시한다.
이 문서 작성 작업에서는 5K 본수집·학습을 실행하지 않았다. 바뀐 코드로 과거 output을 강제 resume하지 않는다.

## 최종 증거 경로

위 실험 root 아래 경로이다. raw·큰 JSON·실패 원본은 Git에 넣지 않는다.

| 경로 | 의미 |
|---|---|
| `registration_v152b/registration.json` | full source와 64/8/53/4 사전 등록 |
| `qualification_static64_v152b` | 최초62 + 일반 복구2, 부분16 재개 |
| `qualification_static64_completed_final_v152b` | 현재 runtime의 실제 zero-deficit completion |
| `qualification_animated8_v152b` | 현재 capture 계약의 실제 이동 회귀 |
| `qualification_range_static_v152b` | 최초53·일반 복구15, 52승인/1부족 원본 |
| `qualification_range_completed_v152b` | 가림 보완 전 실제 부족분3회 실패, 보존본 |
| `qualification_range_completed_visible_v152b` | 보완 후 1회 승인, 기존52 보존, 최종53 |
| `range_recovery4_final_v152b` | 최신 runtime의 계획된 native4와 완료 재실행0 proof |
| `primary_visibility_proposal_cpu_v1.json` | 7실패 제외·52승인 유지·추가 CPU 비용 |
| `proofs_v152b/*native_support.json`, `range_failure_occlusion.json` | 센서·ground·Z·가림의 독립 대조 |
| `proofs_v152b/cpu_orchestration_receipt_v3.json` | 실제857개 검사·runtime81·test124개 파일 동결 |
| `proofs_v152b/partial_resume.json` | 첫16 보존 증거 |
| `proofs_v152b/terminal_final_resume.json`, `range_terminal_visible_resume.json` | 현재 두 completion의 완료 재실행0 증거 |
| `release_v152_sunny13/release.json`, `cost_estimate.json`, `launcher_preflight.json` | 최종 source 동결·비용·읽기 전용 시작 검사 |

`candidate_v152`, 초기 CPU-only probe와 이전 828/833개 검사 receipt는 중간 구현의 보존 기록이다.
현재 소량 검증은 위 `candidate_v152b`, 최신 runtime, `final`/`visible` output과 v3 receipt를 사용한다.
