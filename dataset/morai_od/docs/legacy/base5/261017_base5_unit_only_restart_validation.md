# Base5 재시작 후 원형 혼합 장면의 정답 치수 검증

문서 상태: EVIDENCE. 기준일: 2026-10-17, Asia/Seoul.
후속 갱신: 원형 full5K 조립·초근접 보완·입력/재개 검증은 [최신 원형 수집 근거](261017_base5_unit_release_evidence.md)를 따른다. 아래 89장 결과와 당시 판정은 그대로 보존한다.
확대 수집을 중단한 뒤 새 MORAI 프로세스에서 원형 장면만 로드해 native 정답 치수의
재현성을 확인한다. 확대 실패 이력은 [이전 검증](261017_base5_scale_split_qualification.md)에 보존한다.
이번 취득은 진단용이며 기존 완료 5K와 학습 데이터에 합치지 않는다.

결론: **재시작 후 확대를 로드하지 않은 원형 89장, 물리 장애물 19종의 부모 관측 352회에서
native 치수 불일치는 재현되지 않았다.** 독립 표본은 핵심 64배치이며 반복·기존 대조를
독립 표본 수에 합산하지 않는다. 원형 전용 수집 방향은 유지하고 확대 수집은 재개하지 않는다.

## 확인할 문제와 시험 범위

이전 혼합 확대 시험에서는 WoodBox와 RedBarrel 등의 native 치수가 요청 배율과 다르고,
원형으로 되돌린 뒤에도 이상 치수가 남았다. 사용자가 재시작한 후에는 확대 장면을 하나도
로드하지 않고 같은 문제가 발생하는지 확인한다. 재시작과 비확대 운용을 함께 바꾸므로,
둘 중 하나만의 인과 효과를 분리한 실험으로 해석하지 않는다.

확인한 MORAI 프로세스는 PID `3233137`, 시작 시각은 2026-10-17 14:18:56 KST다.
Sunny13·Capture Mode·현재 수집 네트워크·annotation 저장을 유지한다.
수집은 기본 정지·충돌·통신·가시성 검사를 사용하며, 추가 native 치수 검사는 GT를 고치지 않는다.

| 단계 | 장면 구성 | 목적과 독립성 |
|---|---|---|
| 초기 대조 | 기존 all-physical-unit 4장 | 두 혼합 배치를 각각 두 번 취득. 재시작 직후 과거 문제 배치를 확인 |
| 이전 실패 배치 대조 | WoodBox 문제가 있던 정확한 원형 payload 5개 | 확대 이후 실패했던 같은 원형을 확대 미노출 프로세스에서 확인. 후속 64개의 분모와 분리 |
| 독립 배치 | 확대 전 원형 source의 서로 다른 혼합 배치 64개 | 19종 물리 장애물, WoodBox·RedBarrel 각각 최소 16개 독립 배치 포함 |
| 장면 반복 | 위 64개 중 미리 고정한 16개를 후반에 재취득 | 같은 payload를 다양한 다른 배치 로드 후 다시 확인. 별도 독립 표본으로 세지 않음 |

후속 80장의 실제 선택과 순서는 native 결과를 보기 전에 고정한다. 모든 scenario의 actor가
원형인지 전수 검사하고, source SHA·native 기준 치수·준비 도우미와 수집 코드의 해시를 기록한다.
확대된 payload를 몰래 원형으로 고치는 대신 확대 이전 source의 실제 원형 payload를 사용한다.
각 진단에는 명시적인 계보와 새 QA metadata만 붙이고 원문을 보존한다.

native 치수 불일치가 나타나면 즉시 안전 중단하고 원문·8개 꼭짓점·actor 정체성·참조를
대조한다. 실패 payload를 정상 결과가 나올 때까지 반복하거나 GT를 비율로 나누지 않는다.
일반적인 가시성·지면 review는 치수 불일치와 따로 기록한다. 통신·Park 실패는 항상 전체 중단한다.

## 초기 원형 대조 결과

`artifacts/evaluation/base5_unit_only_stability_261017/capture_cold_four_v1`은 4/4 승인,
review 0, failed 0이다. RedBarrel 12·TrafficBarrel 4·PE_Drum 4·CargoBox 4·YellowBarrel 2,
총 26개 물리 장애물 부모의 native 치수와 기준값이 일치했다.
정확한 timestamp의 Instance와 8Points를 독립 재계산했을 때 최대 꼭짓점 residual은 약
0.000030924m, 기준 치수 대비 최대 상대 차이는 약 0.003588%였다.
종료 `park_verified=true`, gear 1, 오류 0을 확인했다.

첫 대조에는 WoodBox가 없으므로 후속 독립 배치 결과 없이 WoodBox 안정성을 선언하지 않는다.
ledger SHA는 `22907aeb06a9e4cff883b37c486fde9531a9b764ba9ac7978988f6b137aceb5f`다.

## 이전 WoodBox 실패 배치의 회복

`capture_previous_wood_controls_v1`에서 이전에 치수 문제를 보였던 정확한 원형 payload 5개를
취득했다. 선택한 모든 object의 숫자형 scale과 존재하는 `_x/_y/_z` UI 문자열을 읽어 111인지
먼저 검사했다. 이전 paired manifest의 **원형 indices 3·11·16·20·23만** 사용했고 확대 장면은
로드하지 않았다. 5/5 승인, review 0, failed 0, 종료 Park 정상이다.
WoodBox 6개를 포함한 물리 장애물 26개의 정답 치수가 모두 일치했고, WoodBox의 최대 기준
상대 차이는 약 0.003428%였다. 이전 같은 payload의 WoodBox는 1.8/2.4m였지만 이번에는
정상인 약 1.2m로 기록됐다.

001 배치의 WoodBox UID 8504는 camera3 원문 RGBA를 독립 매핑해 과거와 직접 비교했다.
양쪽 scenario SHA는 `47b37e8f2f82d92c460eee8ce88482f2ed0862f4db1b3f9018854145feb3e201`,
camera calibration SHA도 같다. mask는 179픽셀·bounds `[393,217,410,229]`로 완전히 같고
IoU 1·XOR 0이었다. RGB 채널 평균 절대 차이는 0.423/255, p95는 1/255이며 실제 actor 위치와
heading 차이는 0이다. Ego 위치 차이는 0.073mm였다. 반면 native 꼭짓점에서 계산한 한 변은
약 1.8m에서 1.2m로 회복됐다. 이 작은 물체 사례는 동일한 외형에서 annotation bound만
달라졌다는 강한 근거지만 정확한 mesh/collider 복원이나 모든 자산의 원인 확정을 뜻하지 않는다.

영상 외곽과 투영 대조의 source pin은 `wood_cold_surface_v1/report.json`에 있다. 실제 mask의
동일성·RGB·pose 차이는 기존 반복 진단 함수로 원문을 RAM에서 재검산했고 16개 source SHA가
전후 불변이었다. 원본 GT나 과거 review 판정은 바꾸지 않았다.

## 후속 원형 전용 사전등록

최종 준비본은 `artifacts/evaluation/base5_unit_only_stability_261017/registered_v2/manifest.json`,
SHA는 `8e817f2f64579f77fa6c4bcd4f79e6d2ee9d5349b4ea6a702f253c708bb032c7`이다.
`registered_v1`은 UI scale mirror 검사 전 준비 이력이며 실제 취득에 사용하지 않았다.
확대 전 full5K를 전수 검사해 objectList의 명시적 scale 111을 확인했다.
차량·보행자 serializer가 scale을 생략한 경우는 기본 외형이며, 숫자 111이 명시돼 있다고
바꿔 쓰지 않는다. 모든 source payload와 원래 metadata는 보존했다.

독립 64개에 WoodBox 23·RedBarrel 18개 장면, 다른 17종도 각각 5~14개 장면이 포함됐다.
별도 부모 수로는 첫 64개에 물리 장애물 221개(WoodBox 27·RedBarrel 20), 반복 16개까지
합치면 300개(WoodBox 44·RedBarrel 36) 관측 계획이다. 실제 측정 수는 취득 후 따로 센다.
범용 장애물·사람 계열·기타 차량·정적 미션·음영의 5개 context, 정적 구간 전/중/후,
거리·방향·2~3/4~5/6개 이상 부모와 각도상 겹침 proxy를 포함한다.
proxy는 실제 가림 정도나 가시성 GT가 아니다.

## 최종 취득 결과

모든 첫 시도와 review를 원본 ledger에서 읽었다. 실패한 장면을 다시 찍어 정상으로 바꾸는
재시도는 없었고, 16개 반복은 최초 취득 전 고정한 독립적인 시험 단계다.

| 구분 | 취득 | 전체 장면 QA 승인 / review | 예상 / 관측 / 실제 치수 비교 부모 | 치수 불일치 |
|---|---:|---:|---:|---:|
| 초기 원형 대조 | 4 | 4 / 0 | 26 / 26 / 26 | 0 |
| 과거 WoodBox 실패 payload 대조 | 5 | 5 / 0 | 26 / 26 / 26 | 0 |
| 독립 원형 혼합 배치 | 64 | 59 / 5 | 221 / 221 / 221 | 0 |
| 고정 payload 재반복 | 16 | 14 / 2 | 79 / 79 / 79 | 0 |
| 전체 진단 취득 | 89 | 82 / 7 | 352 / 352 / 352 | 0 |

핵심 80장에는 미관측 ID·native가 없는 시도·비교하지 못한 물리 부모가 모두 0이다.
독립 64개에서 19종을 모두 관측했고 WoodBox 27부모·23장, RedBarrel 20부모·18장을 확인했다.
반복 16개에서도 19종을 모두 관측했고, 대응하는 79부모 전부 양쪽 native를 비교했다.
모든 session에서 종료 `park_verified=true`, gear 1, 오류 0을 확인했다.

## 치수 차이의 크기

정확한 timestamp의 Instance와 8Points를 연결해 UID·DataID·class·component·중심·full RPY·
8개 꼭짓점을 재계산했다. 저장 audit와 원형 참조를 대조하고 원본을 수정하지 않았다.
아래는 **부모마다 세 축 중 가장 큰 절대 치수 차이**의 분포이며 단위는 mm다.

| 비교 | 부모 관측 | 평균 | 중앙값 | p95 | 최대 |
|---|---:|---:|---:|---:|---:|
| 독립 배치 native와 기준 치수 | 221 | 0.00989 | 0.00705 | 0.02756 | 0.04983 |
| 반복 배치 native와 기준 치수 | 79 | 0.01083 | 0.00798 | 0.02964 | 0.04391 |
| 같은 payload의 최초와 후반 재취득 | 79 | 0.00364 | 0.00248 | 0.00983 | 0.03844 |

독립 배치의 최대 축별 상대 차이는 약 0.01281%였다. 이는 시뮬레이터가 저장한 native 치수의
재현성과 꼭짓점 수치 검산 결과이며, mesh 접촉이나 실제 물리 크기를 0.05mm 정확도로
측정했다는 뜻은 아니다. 기존 원형 QA의 허용치는 축별 `max(3cm, 기준값의 3%)`로 유지했다.

## 치수와 다른 장면 검토 사유

독립 64개의 review 5장은 primary 센서 근거 부족 2장, 심각한 지면 아래 배치 1장,
actor annotation/runtime/ground 누락 묶음 1장, 실제 보이는 개수 부족과 primary 센서 근거
부족이 겹친 1장이다. 이 누락 장면도 19종 물리 장애물의 치수 비교 대상은 모두 관측됐고,
누락된 다른 actor를 정상으로 승인하거나 제외하지 않았다.

반복 16개 중 review 2개는 앞선 primary 센서 근거 부족 1개와 지면 아래 배치 1개가 같은
사유로 다시 review된 것이다. 7개 review는 5개의 서로 다른 source payload에 해당하며
치수 오류나 source의 일반적 실패 확률로 바꿔 해석하지 않는다. 가시성·지면 검사는 계속 유지한다.

## 판정과 남은 작업

확대 이후 보였던 이상 치수는 같은 원형 payload에서도 재시작 뒤 정상으로 회복됐고,
확대 미노출 상태의 64개 배치·16개 반복에서는 다시 발생하지 않았다. 현재 근거로는 새
프로세스에서 원형만 사용하는 쪽이 합리적이다. 내부 캐시·LOD·pooling의 정확한 원인은
확정하지 않으며 장시간 전체 5K에서 오류가 절대 없다는 보장도 아니다.

확대 시험을 한 프로세스에서 그대로 원형 본수집을 시작하지 않는다. 원형 수집 중에는
19종 native 치수 QA를 유지하고 불일치 원문을 보존한다. 다음 원형 full source의 새 조립,
지면 준비 후 분할 overlay 생성, 실제 입력·부분/완료 resume·새 release 동결은 별도 작업이다.
이번 결과를 그 full source의 본수집 완료나 자동 보충 승인으로 승격하지 않는다.

## 근거 파일과 도우미

artifact root는 `artifacts/evaluation/base5_unit_only_stability_261017/`다.

| 파일 | SHA256 |
|---|---|
| `registered_v2/registration.json` | `7080c2092ee04296f076d39cd95e1006bf9dfbaf14549904b6ae45fe15dcc142` |
| `capture_main_v2/part_a/ledger.jsonl` | `403115cfd10fe310ac649024f74a5286973274c7717f569658ffd5439d6daadf` |
| `capture_main_v2/part_b/ledger.jsonl` | `bd4268e8337f1468e7b3229878d7b4c42b8115776386fb8cafa02eeddd9bbd2e` |
| `capture_previous_wood_controls_v1/ledger.jsonl` | `a44785b3418a3b3775d6546fa29d7ad3d3fc0df90264402e125bc3f4dfc977f2` |
| `wood_cold_surface_v1/report.json` | `0db4ae4564db65c9c9fcb05fe19b5a5dc102c18cf57683b71831bbcb68d4f3d8` |
| `final_report_v1/report.json` | `8a7e34d1d914048d6f45b39583d39c7cf52bf2bd3ffa323a4b237aedb3d28b04` |

`prepare_base5_unit_only_stability.py`는 source 전수 비확대 검사와 결과 비참조 선정을 담당한다.
`report_base5_unit_only_stability.py`는 source·등록 이후 실제 scene event·수집 runtime·native 원문·
종료 Park를 다시 확인한다. 최초/반복/재시도·예상/관측/실제 비교·일반 QA/치수 오류 분모를
분리한다. 두 도우미는 ROS 제어·GT 보정·본수집·학습을 실행하지 않는다.
신규 준비 검사 7개와 독립 리뷰를 거친 보고서 검사 39개가 통과했다.

최종 source 전체 CPU 회귀는 **1,865 passed / 1 deselected, 129.01초**다. 제외 1개는 host에
torch가 없는 기존 Base3 segmentation metrics 검사이며 새 Base5 검사를 제외한 것이 아니다.
실제 JUnit과 source/test 해시를 연결한 receipt를 다시 검증했다. 보고서의 독립 64·반복 16·
모든 native 80·실제 비교 300·반복 대응 부모 79·누락 0 분모도 별도 검산했다.

- `regression_v1/receipt.json`: `f33d23642d4c8a0d139036bc67a910e6de3ab917e08bc3d8e0e6a08ebd0ceaed`
- `regression_v1/tests.xml`: `7d08f31716321869a602c10701d6f268bf06d3906a6ca7b0e960070882b8e29c`
