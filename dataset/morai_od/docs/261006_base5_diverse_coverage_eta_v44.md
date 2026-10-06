# Base5 v44 — 후방·복합 검증과 10K 시간 민감도

2026-10-06 / EVIDENCE. 본10K 캡처·학습 미시작. 기본 준비9초 미변경.

## 후방·복합 자연 이동

원본 v35에서 후방6셀(걷기/달리기,3–5/15–30m), Ped_Pet1 두actor1셀,
Deer1+Dog1 두actor1셀을6초 준비 후보로 재생성했다. 목표geometry/속도/Ego/split 고정.
OFFLINE_PASS8, 실제8/8통과/8촬영/68.263초. Event18회, 오류/timeout/미종료worker0,
모든 boundary/final Park 확인. 목표 누락 재시도0, 품질 보완0.
CPU train6/val2/test0, GT14(vehicle1/pedestrian9/animal4), review/failed 편입0.
후방 전체 거리/모든 자산이나 장시간 안정성을 인증하는 결과는 아니다.

## 10K 예상 시간은 어떻게 줄어드는가

Canonical v35 metadata 전수집계: 자연 이동 대상1,646/10,000셀(16.46%).
나머지8,354셀은 이 자연 이동 준비 여유를 줄여도3초씩 절약되지 않는다.
정상1회 촬영만 계산하면9→6초 절약량은1,646×3/3600=1.372시간이다.
v43 대표32셀의 보완비용까지 포함한 같은 family 가중 외삽은20.19→18.71시간,
약1.48시간 감소였다. v36부터의 전체 최적화 누적은27.24→18.71시간(약31.3% 감소).
이는 소량 coverage 표본의 외삽이지10K 완료 보장이나 A/B 반복 신뢰구간이 아니다.

목표 누락 재시도는 정상 v43 표본에 없었다. 다음은 **자연 이동1,646셀만** 대상으로,
재시도1회당 추가30초를 가정한 민감도다.30초와 발생률은 실제 분포 추정치가 아니다.
v43 비용에 이미 들어 있는 지면/가시성 품질 보완은 다시 더하지 않는다.

| 자연 이동 장면의 목표 누락 재시도율 가정 | 추가 시간 | 10K 후보 예상 |
| --- | ---: | ---: |
| 0% | 0시간 | 18.71시간 |
| 1% | 0.14시간 | 18.85시간 |
| 5% | 0.69시간 | 19.40시간 |
| 10% | 1.37시간 | 20.09시간 |

`report_base5_setup_budget_eta.py`가 canonical metadata와 완료된 비용report로 위 수치를
재계산한다. 비용report 미완료·10K 아닌 manifest·비정상 숫자는 거부한다.

공통 root /home/luv/morai_cap/base5_v2_261006/ 아래:

- budget6_rear_complex_v44:8셀 deck/OFFLINE_PASS.
- budget6_rear_complex_capture_v44, budget6_rear_complex_cost_v44.json: 실제 근거.
- budget6_rear_complex_infos_v44:8장 학습 형식, 본학습 미실행.
- budget6_eta_sensitivity_v44.json: 가정별10K 시간.
- budget6_diverse56_v44: 새56셀 자산 다양성 검증 deck/OFFLINE_PASS.

새56셀 구성: highway16, pedestrian_related8(자연 이동 대기 없는 정적/고정형),
obstacle16, other_vehicle6, roundabout6, background4. 이 정적 중심 표본을
전체10K 비율로 간주하거나 자연 이동 후보18.71시간을 더 낮추는 근거로 쓰지 않는다.

## 새 자산 혼합56셀 결과

56/56 최종통과, 실제58촬영, driver314.356초. 가시성 부족2셀은 같은quota replacement
한 번씩으로 복구했다. Event134회, 짧은 handler 응답 오류2회는retry 회복,
timeout/미종료worker0. 모든 boundary/final Park 확인, unsafe session0.
CPU train51/val5/test0, GT81(obstacle25/vehicle40/pedestrian16).
후방8셀과 합쳐 추가64장/66촬영/GT95, 실수집wall 합382.62초다.
56셀 cost report의 자동 family 외삽15.615시간은 자연 이동 내부비율을 반영하지 못하므로
현재10K 예상으로 채택하지 않는다. **대표 자연 이동 포함 표본18.714시간을 유지한다.**
완료56셀 resume 재검증은exit0, ledger SHA/elapsed314.356초/Event134회 불변,
추가 서비스요청·재촬영0이었다.
138tests PASS. 전체pool/하루규모 안정성 인증이 아니며 기본9초/본수집 미시작 유지.
live dashboard8078은후방8셀,8079는자산56셀 완료결과다.
새 산출물 budget6_diverse56_capture_v44 / budget6_diverse56_cost_v44.json /
budget6_diverse56_infos_v44는 같은공통root에 있다.
