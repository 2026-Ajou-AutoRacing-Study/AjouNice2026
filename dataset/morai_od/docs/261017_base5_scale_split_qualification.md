# Base5 확대 안정성과 지도 전역 학습 검증

문서 상태: EVIDENCE / 확대 수집 중단. 최종 판정: 2026-10-17 14:10 KST. 기존 실험 snapshot과 현재 판정을 구분한다.
목표는 새 Sunny13 5K 생성기에 확대 장애물과 지도 전역 학습 분할을 통합하되,
native GT 불일치나 재취득 비용이 큰 규격은 본수집 전에 제외하는 것이다.
기존 완료 5K·E24·v155m·실패 원본은 보존한다. 새 5K 본수집과 학습은 실행하지 않는다.

현재 판정은 [확대 수집 중단과 다음 원형 경로](#확대-수집-중단과-다음-원형-경로)다.
아래 확대 규격·v156e 분포는 검증한 실험 후보의 이력이며 다음 본수집 구성으로 채택하지 않는다.

후속 원형 대조에서는 다시 재시작한 확대 미노출 프로세스에서 89장·물리 19종 352부모 관측의
native 치수 불일치가 0이었다. 과거 WoodBox 실패 payload도 정상으로 회복됐다.
[원형 검증 근거](261017_base5_unit_only_restart_validation.md)를 따른다. 아래 혼합 확대 실패
원문·판정을 취소하거나 원형 full source의 최종 본수집 승인을 발급한 것은 아니다.

## 실행 순서와 판정

| 순서 | 과제 | 검사와 판정 |
|---|---|---|
| 1 | 기존 크기 참조 정리 | 완료 5K 전체의 native 관측을 검산한다. 원형 치수·배치 envelope·높이 initializer·프레임별 GT를 구분한다. |
| 2 | RedBarrel 진단 | 기존 원문과 같은 조건의 warm 반복에서 실제 mask 외곽·native 치수·예상 배율·중심 이동을 비교한다. 불안정한 GT를 임의 축소하지 않는다. |
| 3 | 큰 배율 적격성 | 상자 2/3배·샘플 `(2,3,2)`, 통 2배, 방호물 길이 2/3배를 단독 시험한다. 실물 치수·외형·접지·센서/GT를 보고 채택한다. |
| 4 | 전역 train coverage | known-map layout-group 분할을 사용한다. 같은 실제 배치는 묶고, 모든 관측 구간과 미션을 train에 포함한다. 비율은 약 85/15다. |
| 5 | 최종 규격의 안정성 | 실제 새 후보에서 서로 다른 확대 배치와 대응 scale 1 대조군을 outcome 없이 등록한다. 최초 실패와 재취득 비용을 함께 센다. |
| 6 | 입력·복구·동결 | Q90/upright/512 실제 입력, 부분/완료 resume, 동일 계약의 유한 보충, 전체 CPU 감사와 새 release를 검증한다. |

원형은 다수로 유지한다. 1.25/1.5배를 무의미하다고 가정하거나 2/3배의 공식 출제 확률이
높다고 단정하지 않는다. 확대 후 다른 제품처럼 자연스러운 규격만 기존 장면 slot에 통합한다.
차량·차량형 OBJ·사람·동물·복합체·유모차·탑승자·볼라드·차선 분리봉은 확대 대상이 아니다.

안정성 목표는 최종 확대 60장과 대응 원형 60장이다. 규격별 표본 수와 교차 조건에 따라
최소 반복 수를 명시한다. 최초 계획은 총 240장 이내였으나 혼합 annotation bound 변동이라는
새 blocker를 분리하기 위한 cold/hot 대조와 사전 pilot 2장을 포함해 상한을 256장으로 조정했다.
단독 적격성·Red 진단·보호 회귀도 이 상한에 포함하며 실패 payload의 무한 재시도는 하지 않는다.
같은 장면 반복을 독립 배치 표본으로 세지 않는다. 확대 실패율이 원형보다 5%p 높거나
승인 장면당 총비용이 10% 넘게 늘면 원인을 검토해 해당 규격을 축소·제외한다.
이는 운영용 선택 기준이지 실패율 1% 이하의 통계적 보증이나 AP 향상 인증이 아니다.

## 원본 보존과 재시도

최초 review/failed도 원문과 시간에 포함한다. 배치·확대 원인, 가시성, 통신·설정, 안전 중단과
원인 불명을 분리한다. 확대 장면에서 실패했다는 이유만으로 확대가 원인이라고 분류하지 않는다.
같은 실패 payload를 계속 반복하거나 실패 후 몰래 scale 1로 바꾸지 않는다.

실제 확대 실패가 없으면 실제 실패 이후 자동 보충은 미검증으로 남긴다.
계약을 검증하는 counterfactual 시험과 production 실패 복구는 구분한다.
source/runtime·센서·분할·규격이 바뀌면 새 경로와 새 release를 쓰며 구 release에 강제 resume하지 않는다.

각 matched 조건의 최초 승인율도 최소 90%로 등록한다. 조건 간 실패율 차이만 작고 두 조건 모두
불안정한 상태를 승인하지 않는다. 이 값은 paired 취득 결과를 보기 전에 정한 운영 기준이다.
정식 paired campaign 중 어느 조건이든 비승인 7장에 도달하면 남은 장면이 전부 통과해도
60장 중 90%를 만족하지 못하므로 안전 종료하고 원인을 검토한다. 완주하지 않은 campaign은
120장 검증 통과로 부르지 않는다. 통신·Park 등 안전 중단은 이 수치보다 항상 우선한다.

## 현재 관측

- [크기 참조 전수 감사](261017_base5_geometry_reference_audit.md): 완료 5K의 11,372개 native 성분 중
  물리 장애물 19종 3,299개를 검산했다. 과거 거리용 upright 중앙값 19종을 모두 재현했고 원문은 불변이다.
- 큰 배율 39장 사전 등록은 collector 입장 검사 39/39를 통과했다.
  방호물은 첫 yaw만 포함하므로 채택 후보는 반대 yaw의 별도 시험이 필요하다.
- 첫 live 취득은 RGB·LiDAR만 저장되고 Instance/GT가 없어 중단했다. 종료 P를 확인했다.
  이는 확대 실패율에 섞지 않는 센서 export 설정 문제다. 실패 원문은 보존했다.
- 정상 export에는 추가 대기를 주지 않고, 원본 센서 파일만 안정적으로 저장되는데 annotation이
  전혀 없으면 다음 장면 전 전체 중단하도록 보호를 추가했다. 센서 UI 설정을 대신 바꾸지는 않는다.
- RedBarrel의 과거 4캡처·12관측을 원본 mask 외곽과 대조했다. 일부 native 치수 기반 타원기둥
  투영은 실제 관측 외곽보다 컸고, 같은 중심의 `/1.25` 가설이 더 가까웠다.
  가림·mesh 여유·중심 이동을 완전히 분리한 결과는 아니므로 GT를 자동 수정하거나 확대를 승인하지 않는다.
- 새 분할 후보는 파생 학습 분할만 변경한다. 원본 capture split·spec·배치·scale 계약은 유지하고,
  metadata에 해시가 고정된 overlay 계보를 남긴다. 실제 입력과 새 release 검증은 후속 단계다.

## 큰 규격과 원형 크기 검사

`large_visual_v3/report.json`은 새 native 48장을 재검산한 결과다. 7종 모두 두 yaw에서 원형·확대를
직접 취득했고 최초 QA 48/48을 통과했다. `asset_*.png` 7장을 직접 확인해 상자/통/방호물 의미와
외형을 유지하는 아래 규격을 채택했다. 이는 혼합 안정성이나 AP 승인을 대신하지 않는다.

| 자산 | 확대 후보 | 확대된 경우의 조건부 가중치 |
|---|---|---|
| CargoBox | 1.5 / 2 / 3 / `(2,3,2)` | 30 / 35 / 10 / 25 |
| WoodBox | 1.5 / 2 / 3 | 40 / 45 / 15 |
| YellowBarrel·PE_Drum | 1.5 / 2 | 70 / 30 |
| PE_Firewall_Orange·Steel_Barricade·Concrete_Barricade01 | 길이 2 / 3, 폭·높이 원형 | 70 / 30 |
| RedBarrel | 확대 보류, 원형 유지 | 해당 없음 |

기존 자산별 목표 확대 비율은 상자 30%, 통/방호물 20%이며 원형 최소 보호를 유지한다.
가중치는 운영 설계이지 출제 확률이 아니다. 3배 WoodBox의 실제 native 치수는 약 3.6m 정육면체,
길이 3배 Steel_Barricade는 긴 축 약 9.04m다. 이런 큰 꼬리는 낮은 가중치로 제한한다.
전체 5K 동결 전 feasibility 검사에서 객체 수·크기 budget과 근접 크기 tier를 지키며
49개 actor는 원형을 유지하고 7개는 같은 자산의 다른 승인 규격을 선택했다.
그 결과 확대는 284장·312부모이며, 4,716개 원형 source entry는 그대로다.
CargoBox 3배 재고는 독립 배치 3개여서 후속 안정성 사전등록을 최소 3개로 명시하고 모두 포함한다.
실패 결과를 보고 표본 기준을 낮춘 것이 아니며, 세 사례로 일반화 성공률을 보장하지 않는다.

원형 QA는 `native_unit_geometry_reference`를 명시한 새 후보에만 적용한다.
물리 장애물 19종의 native 원형 크기를 비교하며 기존 배치 envelope·높이 initializer·GT를 덮어쓰지
않는다. 기존 5K RAM-only 대조는 3,299개 모두 일치했고, 독립 과거 Red 원형 6개 중 변동 2개를
탐지했다. 원본 15,034개 SHA는 불변이다. 완료 resume도 actor 정체성·native annotation·저장 QA를
다시 대조하며 이름 변경으로 검사에서 빠지는 것을 막는다.

## RedBarrel 재현 범위

`capture_red_single_warm_v1`의 단독 18장은 원형/1.25/1.5배·두 yaw·세 반복에서 모두 통과했다.
하지만 과거 실제 혼합 payload 두 개를 각 세 번 그대로 재생한
`capture_red_exact_mixed_warm_v1`은 6/6이 `scaled_native_bbox_size_mismatch`로 남았다.
같은 payload에서도 일부 native 치수가 정상과 약 1.25배 사이로 바뀌었고 원형에도 한 번 관측됐다.
따라서 단독 성공으로 혼합 예외를 취소하거나 Red 확대를 재승인하지 않는다.

`red_new_surface_v1/report.json`은 같은 UID·timestamp의 mask 외곽과 native 타원기둥 proxy를
대조한다. 일부 예외에서 native보다 `/1.25` 외곽이 실제 이미지에 더 가깝다. 이는 단순한
"BBox 안에 LiDAR 점이 있다"보다 독립적인 근거지만, 정확한 mesh·가림·중심/pivot을 완전히
복원한 증명은 아니다. 저장 GT를 나눠 고치지 않는다. 원형만 사용한 혼합 대조는 별도로 수행한다.

### 후속 반례: Red 확대 제외만으로 충분하지 않음

이 단락은 위 단독 성공보다 뒤에 확인한 결과다. `capture_red_all_unit_mixed_warm_v1`은
Red 3개를 모두 원형으로 바꾼 두 배치 × 2회지만 4/4이 원형 치수 검사에 걸렸다.
동일 장면의 CargoBox 1.25배 두 개 중 하나에서도 추가 1.25배 native 치수 예외가 관측됐다.
따라서 Red 자산만의 문제로 확정하지 않는다.

`capture_red_all_unit_mixed_fresh_uids_v1`은 형상·위치·scale을 보존하고 26개 object UID만
장면별 고유 값으로 변경한 대조다. 4/4 review이며, 이상 치수가 나타나는 parent는 달라졌다.
UID 변경을 해결책으로 채택하지 않고 최종 후보의 UID 정책도 바꾸지 않는다.
객체 없는 배경 1장은 통과했지만, 그 뒤 `capture_red_all_unit_after_empty_v1`의 같은 4장도
4/4 review였다. 빈 장면 로드 역시 해결책이 아니다.

이 증거는 simulator의 재사용 상태·내부 bounds 등의 가능성을 지지하지만 정확한 내부 원인을
확정한 결과는 아니다. LOD·mesh와 annotation 캐시를 임의로 단정하지 않는다.
현재 live 취득은 P·정지로 마감했고 새 MORAI 프로세스 대조를 요청했다.
혼합 GT의 안정성을 해결하지 않은 상태에서 120장 검사를 계속하거나 새 release를 승인하지 않는다.
기존 완료 5K의 원형 3,299개 일치 결과와 새 확대 실험의 문제를 구분한다.

후속 [동일 마스크 직접 대조](261017_base5_native_extent_repeat_mask_evidence.md)에서는
old mixed Red의 native 치수가 1.5→1.875배로 바뀌어도 camera1 mask 외곽은 148×243px로 같고
IoU 0.999609였다. 실제 actor 위치 차이는 0m다. 원통 proxy 없이도 annotation bound 변동을
강하게 지지한다. 실제로 움직인 Cargo 대조는 별도 반례로 남기며 모든 객체에 일반화하지 않는다.

사용자가 MORAI와 ROS를 재시작한 뒤 all-physical-unit 4장은 4/4 승인됐다.
첫 시작 검사 1회는 로드·capture 없이 중단했고 P를 확인한 뒤 새 session에서 검사했다.
새 최종 규격의 사전 대응쌍 2장도 통과했으며 그 뒤 동일 all-physical-unit 4장 역시 4/4 통과했다.
이 최초 2장은 최종 통합 preregistration 이전의 pilot이므로 공식 paired120 승인·실패율·비용에
합산하지 않는다. 이후 전용 campaign에서 실제 120장을 새로 취득한다. 원래 paired geometry
선정은 이 pilot보다 먼저 고정돼 있었고 최종 source/선정은 바꾸지 않았다.

## 현재 후보 계보와 실제 입력

큰 규격 후보는 다음 순서다. 지면 준비를 분할 뒤에 적용하면 두 음영 장면의 실제 geometry가
바뀌므로 반드시 먼저 확정한다. 저장 terrain을 재검증해 **입력 시나리오**만 재사용하며 과거
raw를 새 취득 데이터로 복사하지 않는다.

1. `candidate_scale_v156d/manifest.json`: 새 7종·17규격 full 5K, source audit/preservation 통과.
2. `materialized_large_v156d/manifest.json`: 저장된 두 음영 경로를 현재 helper로 다시 등록·준비.
3. `unit_large_v156d/manifest.json`: 원형 QA 참조를 새 metadata에 추가.
4. `candidate_large_v156e/manifest.json`: 최종 예정 geometry에 파생 학습 분할을 부여.

v156e manifest SHA는 `82a9cc24ce9d6ce476cedd574ba0bf07ced2093ae8f32706aeda4f5c8574fdd4`,
overlay SHA는 `cc81f954ce4dd634d18da45947ebb2ceabf2c7fb560bb0fb44bed13b00142552`다.
Train/val 4,250/750, 관측 69 link·316개 10m 셀 모두 train 포함, asset 누락은 없다.
`candidate_current7_v156c`와 그 전 v156b는 이전 14규격의 중간 CPU snapshot이다.

중간 v156c의 실제 과거 native 17장을 새 overlay로 Q90/upright/3-class/512 CPU 입력까지 변환했다.
33개 GT train·10개 GT val이며 기존 raw는 불변이다. 이는 새 v156e 취득 승인이나 새 데이터가
아니다. 최종 v156e의 혼합 안정성·정확 source smoke·재개·실제 입력 검증은 아직 필요하다.

이후 6환경 30K를 합칠 때도 실제 geometry/clip 그룹을 전체 선택분에서 다시 감사한다.
각 환경의 독립 85/15 할당만으로 환경 간 같은 배치의 cross-split 부재를 보장하지 않는다.
충돌하면 원본을 재취득하기보다 파생 split을 전체 기준으로 다시 배정하고 검증한다.

완료 합본 sidecar는 검증한 planned metadata core를 사용한다. `capture_runtime_*` 진단은 원본
RAW metadata에 보존하고 별도 hash/계보로 연결한다. 지면 prepared core의 exact 검사를 느슨하게
만들거나 RAW 필드를 삭제하지 않는다. 종료 Park receipt가 없으면 완료로 간주하지 않는다.

실험 root는 `artifacts/evaluation/base5_scale_qualification_261017/`다.
기존 v155m의 17/17 승인과 `bulk_ready=true`는 당시 snapshot으로 보존한다.
현재 수정 중인 코드와 새 큰 배율·분할 정책의 승인으로 승계하지 않는다.

## 정식 혼합 시험의 중단 결과

수정된 source의 전체 CPU 회귀 v3는 **1,819 passed / 1 deselected, 136.43초**였다.
제외 1개는 host torch가 없는 기존 Base3 segmentation metrics 검사이며 새 Base5 실패를
숨기기 위한 제외가 아니다. 회귀 receipt와 runtime/test 전후 SHA를 고정했다.

준비 과정에서 원형 reference 목록과 전체 입력 해시 목록이 같은 mutable dict를 공유하는
오류를 찾았다. reference 목록을 깊은 복사로 분리하고 실제 prepare/JSON 경로의 회귀를 추가했다.
기존 v1은 보존하고, 같은 실제 배치·순서를 갖는 `stability_large_v156e_v2`를 새로 만들었다.
이 오류는 CPU 사전 gate에서 발견했으며 simulator의 native 치수 변동과 다른 문제다.

`final_smoke_registration_v3`를 완료한 뒤 전용 `capture_stability_final_v3/part_a`에서
처음 26장, 독립 배치 13쌍을 취득했다. 원형 조건의 비승인이 7장에 도달해 90% gate가
불가능해지자 STOP을 요청했다. 종료 처리 전 진행된 장면까지 모두 포함한 최종값은 다음과 같다.

| 조건 | 관측 | 승인 | review | native 치수 불일치가 있는 장면 | ledger 시간 합계 |
|---|---:|---:|---:|---:|---:|
| 원형 | 13 | 5 | 8 | 5 | 61.60초 |
| 확대 | 13 | 6 | 7 | 5 | 58.50초 |

원형 review는 치수 5·심각한 지면 아래 1·실제 보이는 개수 부족 1·primary 센서 근거 없음 1이다.
확대 review는 확대 치수 4·미확대 actor 치수 1·actor/runtime/ground annotation 누락 묶음 1·
primary 센서 근거 없음 1이다. 모든 review를 scale 원인으로 확정하지 않는다.
목적형 표본이고 중간에 중단했으므로 이 비율을 일반 원형 5K의 실패율로 추정하지 않는다.
또한 승인 수가 적고 GT 문제가 있으므로 확대가 더 빠르다는 결론도 내리지 않는다.

native 불일치 12개 부모 관측은 모두 WoodBox였다. 한 변 약 1.2m인 원형 기준이다.

| 배치 / UID | 확대 조건의 요청과 native 관측 | 대응 원형 조건의 native 관측 |
|---|---|---|
| 001 / 8504 | 1.5배: 예상 1.8m → 2.7m | 예상 1.2m → 1.8m |
| 005 / 8501 | 3배: 예상 3.6m → 5.4m | 예상 1.2m → 2.4m |
| 005 / 8503 | 2배: 예상 2.4m → 4.8m | 예상 1.2m → 1.8m |
| 008 / 8500 | 1.5배: 예상 1.8m → 3.6m | 예상 1.2m → 1.8m |
| 010 / 8502 | 이 actor는 원형: 예상 1.2m → 2.4m | 예상 1.2m → 1.8m |
| 011 / 8503 | 2배: 예상 2.4m → 3.6m | 예상 1.2m → 2.4m |

세 축에서 같은 비율이다. 첫 실패 배치에는 WoodBox가 하나뿐이므로 같은 자산의 장면 내
중복만 막으면 해결된다고 할 수 없다. 이전 확대 이후 원형으로 돌려도 상태가 남으며,
1.25배를 제외한 현재 후보에서도 재현됐다. 내부 cache/pooling/LOD/shared bounds 중 어떤
기전인지는 확정하지 않는다. 새 WoodBox 관측 전부에서 실제 mesh도 같은 비율로 변했는지는
별도 문제다. native GT만 보고 이를 확정하거나 예측 배율로 GT를 역보정하지 않는다.
추가 surface 대조는 엄격한 camera companion 검사에서 중단됐고 승인 근거로 쓰지 않았다.

종료는 `park_verified=true`, gear 1, 오류 0으로 확인했다. 실제 상태는 `interrupted`이며
part_b·후속 exact smoke 24장·그 입력의 CPU loader·새 release 발급은 실행하지 않았다.
등록 120장 중 26장 관측이라는 사실을 120장 검증 통과로 바꾸지 않는다.

| 최종 근거 | SHA256 |
|---|---|
| `stability_large_v156e_v2/manifest.json` | `a4e05d0bd2da3bbfc5907ddc00216040365b7fb9e3a41b8e4efa8f87ccefb63e` |
| `final_smoke_registration_v3/registration.json` | `10a645337fe5564e752400162a30727d62fde06ebde5aa722ae5c512c0f1efd6` |
| `capture_stability_final_v3/part_a/ledger.jsonl` | `5b3b4daf8986ff0d6ab92d65cfdeabcf41d5f41fcbed1c2d088acde8f9ed8133` |
| `stability_final_v3_stopped/report.json` | `61df3d05ff47ebabc22ee093ca4eb7cd39c4081796f01274b1fa940bf8ded7a8` |
| `regression_final_bounded_v3/receipt.json` | `45c7c78e8da735beb2a64d4e72a24933c7bcedac643d3e7b1ffa38f2b73c7526` |
| `regression_final_bounded_v3/tests.xml` | `00a2ef8cae6474b0ca224b38e93056bb3008c925a350809339bb9b6abc9a328f` |

## 확대 수집 중단과 다음 원형 경로

사용자는 확대 분포의 출제 근거가 불확실하고 임의 배율이 추가 위험을 만든다는 점을 들어
확대 수집 중단을 제안했다. 단독 외형·native 시험의 성공보다 혼합 재현성 실패를 우선해
확대 수집을 여기서 중단한다. 다음 구성은 **scale `(1,1,1)`**로 준비한다.
실험 도우미·48장 단독 성공·실패 raw·정책·과거 release는 연구 이력으로 보존한다.
실험 확대 manifest를 다음 수집 명령에 넣거나 새 학습 데이터에 합산하지 않는다.

원형 학습 모델의 샘플 주행 성공은 유용한 근거지만 모든 미공개 확대율에 대한 성능 보장은
아니다. 반대로 수집 중 GT가 불안정한 데이터를 의도적으로 늘리는 것보다, 현재 검증 가능한
원형 분포에서 근거리 장애물·원거리 차량·방향·가림·배경의 질을 확보하는 쪽을 선택한다.

| 과제 | 이번 완료 범위 | 다음 처리 |
|---|---|---|
| 크기 참조와 native GT 구분 | 기존 5K 전수 감사, 19종 3,299개 원형 검산, 별도 QA·resume 보호 | 유지. envelope/높이 초기값과 GT를 일괄 교체하지 않음 |
| RedBarrel 원인 분리 | 단독 성공과 혼합 실패, 실제 동일 mask/RGB 대비 native 변화 확인 | 확대 제외. 원형 자체의 일반적 오류로 과장하지 않음 |
| 2/3배·비등방 확대 | 7종·17규격 단독 48/48 및 외형 검토 | 혼합 안정성 실패로 수집 중단. 단독 승인을 production에 승계하지 않음 |
| 전역 train과 분리 | v156e에서 train 4,250 / val 750, 69 link·316셀 train 포함 | 유지하되 최종 원형 geometry에서 overlay 재생성·감사 |
| 수집·입력·복구 | 코드 회귀 1,819개, 과거 17장 실제 CPU 입력/loader와 새 계약 보호 | 최종 원형 exact smoke·부분/완료 resume·입력/loader·release는 아직 필요 |

다음은 확대 이전의 `base5_static_extent_261016/candidate_extent_v155j/manifest.json` 계보에서
원형 여부를 전수 확인하고, 정적 구역 확장·근거리 장애물·원거리 차량·기존 5K 비율과 primary
자산을 보존하는 경로다. 지면 준비를 먼저 materialize하고 원형 QA·known-map overlay를 새로
부여한다. 확대 v156e의 metadata에서 scale만 지우거나 그 overlay를 그대로 전용하지 않는다.

현재 MORAI 프로세스는 확대 시험 후 native 치수 상태가 남을 수 있으므로 원형 검증 전에
새 프로세스로 재시작하고 원형 혼합 control을 확인해야 한다. 그 뒤 최종 원형 source의
소량 native·Q90/upright/512 실제 CPU loader·재개를 검증해 새 release를 동결한다.
당장은 기존 완료 5K·E24·과거 분할·원본을 변경하지 않으며 새 본수집 승인도 발급하지 않는다.
