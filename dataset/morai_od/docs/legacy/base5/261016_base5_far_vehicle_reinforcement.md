# 80~100m 차량을 포함한 원거리 보강: v153

문서 상태: EVIDENCE / CANDIDATE. 기준일은 2026-10-16, Asia/Seoul이다.
80~100m도 개선 목표로 유지한다. 기존 Sunny13 5K·native GT·E24 가중치는 변경하지 않는다.
새 배치와 소량 센서 검증은 재학습 후 검출 성능 개선의 증명이 아니다.
후속 상태 주석: v153 당시 결과는 아래에 보존한다. 초기 거리 분산을 보완한 새 source와 현재 실행 여부는
[v154 최종 검토](261016_base5_precollection_final_review.md)·[CURRENT](BASE5_V2_CURRENT.md)를 따른다.

## 기존 데이터에서 확인한 것

완료 5K의 frozen infos와 현재 512 입력의 instance cache를 정확한 계보로 연결하였다.
원본 audit·metadata 10,000개 파일과 infos/cache 11,052개 component identity를 검사하였다.
아래는 실제 LiDAR XY 거리의 **유지된 Vehicle GT 수**이며 장면 수가 아니다.

| 실제 거리 | Vehicle GT | LiDAR 0점 | 1~4점 | 5점 이상 | 카메라 gate 충족 | val GT |
|---|---:|---:|---:|---:|---:|---:|
| 50~80m | 428 | 12 | 62 | 354 | 414 | 51 |
| 80~90m | 125 | 13 | 60 | 52 | 119 | 2 |
| 90~100m | 94 | 24 | 57 | 13 | 94 | 4 |

90~100m 전방의 일반 승용·SUV·MPV 33개 중 20개가 LiDAR 0점이지만 모두 카메라 gate를 충족하였다.
이를 삭제하거나 LiDAR 점이 잘 잡히는 트럭으로 바꾸면 개선하려는 핵심 난도를 피하게 된다.
카메라 gate는 실제 입력의 관측 근거이지, 모델이 정확하게 분류할 수 있다는 보장은 아니다.
전체 원거리 native Vehicle 660개 중 현재 GT에서 제외된 13개는 모두 0점·camera gate 미달이다.
일부에 작은 픽셀이 남아 있으므로 모두 완전 비가시라고 단정하지 않는다.

v152의 계획상 Vehicle primary는 2,214개이며 50m 이상은 587개, 80~100m는 209개·43차종이다.
차종이 통째로 빠진 문제보다 거리·방향·시야별 관측의 희소성이 중요하다.
특히 고주로 범용 보조 차량 664개는 0~30m 344개, 30~50m 320개로 **50m 이상이 없다**.
가까운 주요 차량과 먼 보조 차량을 함께 학습하는 경로를 보강할 여지가 있다.

## 저장된 E24 출력은 무엇을 말해 주는가

10/14의 nearest·compile OFF 출력 1,570개를 같은 센서 hash의 full-ROI GT와 CPU로 대조하였다.
적격 1,413프레임에서 XY 2m 대응을 검사하였다. 이 표는 AP나 독립 검출 확률이 아니다.

| GT 거리 | GT 관측 | Vehicle score 0.4 대응 | score 0.1 대응 |
|---|---:|---:|---:|
| 50~80m | 276 | 191 | 212 |
| 80~90m | 49 | 10 | 17 |
| 90~100m | 42 | 2 | 8 |

80~100m의 91관측 중 현재 threshold 통과는 12개, 0.1~0.4 후보로 추가 회복 가능한 것은 13개이다.
나머지는 가까운 Vehicle query가 있지만 0.1 미만인 33개와, 2m 안에 Vehicle query가 없는 33개이다.
앞의 33개 score 중앙값은 0.00107이고 26개는 0.01 미만이다. Threshold를 조금 낮추거나
tracker 기억을 늘리는 것만으로 해결될 문제라고 보지 않는다. 운영 threshold와 기억은 유지한다.

이 91개는 **6 actor/segment·3차종의 반복 관측이며 모두 멀어지는 경우**이다.
처음 접근하는 100m 차량의 bootstrap 성능, native mask 기반 실제 가시성, 일반적인 100m AP를
이 자료로 확정할 수 없다. 0점 28관측은 score 0.1에서도 대응이 없었으며, 5~19점 54관측도
0.4에서 11개만 대응하였다. Camera-only와 센서 지원이 있는 저-score 사례를 나누어 개선한다.

## 새 후보의 설계

고주로 범용의 기존 30~50m 보조 차량 일부만 옮긴다. 장면당 최대 한 대이며 actor를 추가하지 않는다.
목표 할당은 60~80m 60개, 80~90m 50개, 90~95m 24개, 95~99m 26개로 총 160개이다.
각 구간은 전후방을 절반씩 배분한다. 99m 상한은 100m ROI 경계의 배치 여유이며 90~100m 목표를 포기하는 뜻이 아니다.
이 수는 최적 AP 비율이 아니라, 기존 중거리 보조의 절반과 모든 근접 보조를 지키는 첫 후보값이다.

| 대상의 Ego 중심 거리 | 전방 | 후방 | 합계 |
|---|---:|---:|---:|
| 60~80m | 30 | 30 | 60 |
| 80~90m | 25 | 25 | 50 |
| 90~95m | 12 | 12 | 24 |
| 95~99m | 13 | 13 | 26 |
| 합계 | 80 | 80 | 160 |

80~99m 보조 100대는 기존 원거리 primary를 교체하지 않고 추가 학습 기회를 제공한다.
장면·차량 총수를 늘린 것이 아니라 기존 보조 160대의 거리를 바꾼 것이다.

초기 v153의 90~99m 50개는 가시성 proxy 순위 때문에 90~93m 쪽으로 쏠렸고 95m 이상은 6개뿐이었다.
따라서 후속 후보에서는 95~99m를 별도 quota로 나누었다. 같은 band 이름만 붙이고
실제로는 낮은 거리만 강화하는 편향을 피하기 위한 변경이다. v153과 v153b는 CPU 중간 근거로 보존한다.

- 모든 primary, 미션, 0~30m 보조, 객체 수·차종·seed·split을 보존한다.
- 선택 장면의 Ego와 대상 외 모든 actor payload·metadata까지 그대로 유지한다.
- 대상 NPC도 원래 합법 MGeo 차로 계열을 유지하고 도로 yaw·지면 초기화를 사용한다.
- 실제 중심 거리·앞뒤·ROI·차체 간격·도로 지지를 검사한다. 경로 arc 이동량만으로 통과시키지 않는다.
- source scenario·metadata·manifest·helper·환경 hash를 동결한다. 다른 환경은 새 source에서 다시 할당한다.
- source-only 재배치·유한 탐색·중복 payload 차단을 사용한다. 실패 시 거리나 차종을 임의로 완화하지 않는다.

캡처에서는 원거리 대상 자체에 **LiDAR 1점 이상 또는 현재 512×1056 crop의 instance 근거**가
있는지 추가로 검사한다. 가까운 주 객체가 보였다는 이유만으로 원거리 대상의 부재를 승인하지 않는다.
카메라 근거는 16 mask pixel 이상·가시 BBox 양변 4px 이상이며, native 실제 거리/방향도 대조한다.
실제 학습 GT의 LiDAR-frame 중심 ROI 밖에 있어 제외될 대상은 원거리 보강 성공으로 세지 않는다.
0점 camera-only를 허용하며 기존 component GT·320 근거·충돌·Park·정지·심각한 지면 QA를 낮추지 않는다.
512 마스크는 실제 전처리와 같은 int resize·OpenCV nearest·crop clipping으로 계산한다.
선택한 원거리 parent의 인구 가시성 검사에도 이 512 근거를 사용한다. 기존 320 필드와 다른 parent의 기준은 보존한다.

## 첫 native 검증과 복구에서 발견한 것

v153c의 고정 32장은 4거리 구간×전후방×동일/인접 차선×2장이다. 23차종을 포함하며,
대상 UID의 실제 native box를 대조하였다. 첫 중단 실행과 원본을 보존한 뒤 R2를 새 경로에서 실행하였다.
R2는 **27/32 승인·5장 부족**으로 안전 종료하였다. 총 raw 47회는 승인 27회와 review 20회이다.
이는 독립 47장이나 100m AP가 아니다. 부족 5장은 모두 대상의 센서 근거 부족이며,
primary·ROI·심각한 Z·충돌·통신/Park 실패로 분류된 것은 없었다.

| 확인 항목 | R2 결과와 해석 |
|---|---|
| Camera-only 승인 | 3개. LiDAR 0점이라고 모두 버리지 않음 |
| 승인 target의 현재 512 카메라 gate | 24/27개. 나머지 3개는 LiDAR 지원 |
| 전방 95~99m Ego cohort | 승인 3개의 실제 LiDAR 거리는 94.31~94.74m. LiDAR 95m 이상 전방을 검증했다고 쓰지 않음 |
| 후방 tail | 실제 LiDAR 96.04~99.56m 승인 4개 |
| 같은 실패 payload의 재촬영 | 5개 payload가 각각 두 번 등장. 원본을 보존하고 후속 runtime에서 중복 차단을 보완 |
| 완료 후 resume | 추가 촬영 0회. native raw의 내용·inode·mtime와 누적시간 보존. 상태 timestamp 갱신과 구분 |

첫 실행의 복구는 기존 `obstacle_support_pair`를 변경하여 동결 계약 위반으로 중단되었다.
로드 전에 차단하고 Park를 확인하였다. Far-only 복구 provenance를 별도 필드로 분리하여
원래 primary/Ego 및 과거 replacement 계보를 보존한 R2에서 전체 32조건의 처리를 마쳤다.
이전 실패 실행이나 runtime 계약을 덮어쓰지 않았다.

### 후속 runtime의 보완

기존 장애물용 helper는 그대로 두고 차량용 3D 완전 가림 제안 필터를 별도로 만들었다.
차량 중심 Z는 단순한 차체 높이 절반이 아니라 측정된 pivot→BBox 중심 offset을 사용한다.
대상 box는 확장하고 가리는 box는 축소한 뒤 LiDAR·관련 카메라의 27개 표본선 모두를
같은 차량 한 대가 가리는 경우만 초기 후보에서 제외한다. 부분 가림·복합체·불확실한 형상은 유지한다.
이는 창문·메시를 보증하는 렌더러가 아니며 native QA나 GT 삭제 정책을 대체하지 않는다.

R2의 기존 승인 27개는 이 필터에서 모두 유지되었다. 반복 실패한 Stinger 장면의
합법 후보 38곳은 같은 Universe 버스에 완전히 가리는 것으로 추정되었다.
해당 한 scene에서는 재시도를 계속하기보다 초기 할당에서 다음 동일 quota source를 찾는다.
판정 비용 CPU 평균 약 1.05ms·p95 1.28ms는 작은 저장 표본의 helper 측정이며 전체 캡처 비용이 아니다.

복구에서는 이미 찍힌 성공·실패 payload SHA를 함께 기억한다. 외부 quota round는
서로 다른 결정적 retry block을 사용하고, 내부 복구도 과거 native ledger를 대조한다.
종료 후 fresh 보충에서도 부모의 실패 시도까지 수집하여 같은 실패 payload를 다시 고르지 않는다.
탐색 소진은 부족분이며 안전·hash 오류를 우회하지 않는다. Far 정책의 imported-prefix 부모는
아직 이 비반복 증거를 지원하지 않아 명시적으로 거부한다. 일반 legacy 경로의 의미는 바꾸지 않는다.

95~99m는 전후 각각 [95,97)m 6개와 [97,99)m 7개의 sampling window로 다시 나눈다.
outer band의 26개 quota는 그대로이며 재시도도 해당 window를 유지한다.
이는 Ego와 LiDAR 원점 차이를 숨기지 않고 tail을 확보하기 위한 초기 분포 계약이다.
실제 LiDAR 95m 이상 여부는 새 촬영으로 확인해야 한다.

좁은 97~99m window에는 합법 도로점이 약 4개인 장면이 있다. 초기 구현은 순위 후보
8개를 모으려다 같은 점을 반복하여 512회 상한까지 탐색했다. 후속 구현은 같은 RNG
draw의 도로점 identity를 기록하여 각 점의 첫 제안만 실제 build하고, 전수 소진하면 끝낸다.
실제 build의 source/hash·지면·차선·가림 QA는 유지한다. 중단한 CPU 후보와 그 당시 소스는
보존하고 새 `candidate_v153d_fast` 경로에서 다시 생성한다. 본수집 속도 개선 실측과 혼동하지 않는다.

완료 후 검증에는 저장된 scenario의 SHA와 해당 예정 entry의 SHA 대조도 추가했다.
같은 spec·source 안의 다른 합법 pose로 raw와 metadata가 함께 바뀌는 일관 변조를
그냥 ‘합법 배치’로 통과시키지 않는다. 기존 촬영에서 그런 오류가 관측된 것은 아니며
동결·resume 검증의 빠진 경계를 CPU 재현 후 보완한 것이다.

## v153d_fast의 전수 감사와 사전 고정 표본

`candidate_v153d_fast/manifest.json`의 SHA는
`57ff8e24930af877b0ff45081463633f177495f4361e7b8538ce5d653d0cd0cc`이다.
5,000개 unique payload의 OFFLINE_PASS·오류 0을 확인하였다. 160장만 변경하였고
4,840개의 원래 경로·payload, 전체 Ego·primary·다른 actor·seed·split을 보존하였다.
대상 43차종, 동일/인접 차선 85/75개, 기존 source split은 train 156/val 4개이다.
이 적은 원거리 val만으로 일반화 성능을 확정하지 않는다.

총 준비 시간은 127.45초이다. 동일 초반 20개 할당의 준비 시간은 반복 도로점 평가를
줄이기 전 약 145초에서 14.5초가 되었다. 본수집 촬영 시간의 10배 개선이라는 뜻은 아니다.
Source 불가 43건 중 완전 가림 추정은 11건·7 source ID이며 동일 quota의 다음 source로 넘겼다.
선택한 160개는 이 proxy에서 완전 가림 추정 0개이나 실제 native 지원을 보증하지는 않는다.

사전 고정한 `qualification_v153d_fast_far32`는 32장·19차종이며 runtime 86개·입력 156개를 pin하였다.
각 거리/방향에서 동일 차선 2장·인접 차선 2장, 3차종 이상·승용/SUV 포함을 유지한다.
95~99m 전방은 하단 1장·97~99m 3장, 후방은 각 2장이다. 전후 모두 상단에 동일/인접 차선이 있다.
전방 상단에는 일반 승용 DEFAULT_WAGON도 포함한다. 이 선택은 native 결과를 보기 전에 확정하였다.

c와 d의 공통 logical slot은 10개지만 6개는 거리 band가 바뀌었다. 대상·거리/방향·차선까지
대응하는 것은 4개이며 시각적 동일 배치 control은 2개뿐이다. 전체 승인율 차이를
가림 필터 하나의 순수한 A/B 개선량이나 공간적으로 독립된 시험 성능으로 쓰지 않는다.

### 고정 runtime의 native 검증 결과

새 32조건은 **첫 촬영 32/32 승인**, review·재촬영·부족 0개로 종료하였다.
누적 활성시간은 175.99초이며, 요청한 4장 뒤 일시정지·resume을 포함한다.
Camera-only 6개, LiDAR 1~4점 11개, 5점 이상 15개이다. 현재 512 camera gate는 29개가
충족하고 나머지 3개는 LiDAR 근거가 있다. 32개 모두 native 점 수와 모델 ROI 내 점 수가 같다.

| 실제 LiDAR XY 거리 95m 이상 | 전방 | 후방 |
|---|---:|---:|
| 승인 대상 | 3 | 5 |
| 그중 camera-only | 2 | 0 |
| 실제 거리 범위 | 96.35~96.44m | 95.30~97.89m |

일반 승용 DEFAULT_WAGON은 실제 LiDAR 96.392m·Ego 97.247m에서 0점·현재 mask 69px로 승인되었다.
후방 Ioniq는 실제 LiDAR 97.889m에서 4점·mask 84px였다. 계획과 달리 실제 95m를 넘은
후방 90~95m cohort도 실제 거리 집계에 포함하였다.
**전방 실제 LiDAR 97~100m 표본은 이번 32장에 없다.** 이를 100m 전방 검증으로 확대하지 않는다.
이전 R2의 후방 99.56m 근거는 별도 당시 runtime의 결과로 보존한다.

공식 read-only 보고기는 전체 source·runtime/input pin·target UID·native box·현재 crop을
다시 대조하고 원본·ledger·status가 보고 수행 중 변경되지 않았음을 확인하였다.
실행별/장면 경계 Park·충돌 무접촉과 실제 정지의 기존 검사는 통과하였다.
원거리 대상의 지면 plane은 모두 측정된 것이 아니며 ground warning을 지우지 않았다.
초기 지도/asset 보정과 현 QA에서 통과한 센서 근거이지 지면 mesh 정합 인증은 아니다.

부분 resume은 기존 승인 4개·raw 140파일·event log 1개를 그대로 보존했다.
완료 resume은 추가 촬영 0회이며 전체 raw 1,104파일의 SHA/inode/mtime/size,
event log 8개·selected ledger 내용·175.99초 누적시간이 같다. 상태의 확인 timestamp와 구분한다.
이 표본은 선언한 32조건의 native support qualification이며 전체 160개·43차종이나
다른 다섯 환경의 전수 촬영 인증이 아니다. 새 본수집 release의 최종 동결은 별도이며 `bulk_ready=false`를 유지한다.

최종 CPU 회귀는 1,139개 통과이며 원거리 관련 171개도 포함한다.
호스트에 PyTorch가 없어 무관한 Base3 분할 학습 지표 테스트 1개만 명시적으로 제외했다.

```bash
python3 -m pytest -q dataset/morai_od/tests \
  --deselect=dataset/morai_od/tests/test_prepare_base3_lraspp_4class.py::test_segmentation_metrics_include_foreground_quality
```

## 평가와 학습에서 남은 조건

기존 val의 80~90m GT는 2개, 90~100m는 4개뿐이다. 원거리 보조를 늘려도 같은 물리 link에
집중된 val의 한계는 해결되지 않는다. 기존 split을 사후 이동하지 않고 별도의 비학습 진단을 유지한다.
소량 native 검증도 새로운 seed라는 이유만으로 공간적으로 독립된 test set이라고 부르지 않는다.

다음 짧은 학습 비교에서는 0~30m 회귀와 50~80/80~90/90~100m를 함께 확인한다.
Camera-only/1~4점/5점 이상, 전후방·일반차/대형차를 분리한다. 접근 bootstrap과 연속 검출은
정지 SaveSensorData가 아닌 별도 주행 입력이 필요하다. Camera depth/BEV/fusion의 기여는
가중치 고정 ablation과 새 학습 비교의 범위를 구분하여 검사한다.

다음 모델 검토는 아래 순서로 진행한다. 이번 작업에서는 학습·GPU 추론을 시작하지 않는다.

1. 새 데이터의 실제 유효 GT 수·현재 512 crop·거리·차종별 분포를 먼저 동결한다.
   계획 160대와 학습에 유지된 160개 GT가 자동으로 같은 수라고 가정하지 않는다.
2. 공통 compile OFF·nearest·초기화 계약에서 짧은 학습 비교를 한다.
   0~30m를 보호하면서 50~80/80~90/90~100m의 AP·precision/recall을 함께 확인한다.
   Camera-only·1~4점·5점 이상과 전후방·승용/SUV·대형차를 분리한다.
3. 처음 접근하는 차량의 초기 검출, 검출 연속성·위치 오차·오검출 잔류를 별도 주행 bag에서 본다.
   현재 6 actor/segment의 멀어지는 차량만으로 threshold·tracker 수명이나 개선 폭을 확정하지 않는다.

먼 차량을 크게 만드는 것만으로 비교가 쉬워지지 않도록 승용·SUV를 유지한다.
동시에 근접 가림·복합체·장애물의 기존 보강을 원거리 차량 성능과 교환하지 않는다.
Native 가시성 성공 이후에도 학습 개선이 작다면 그때 카메라 feature/depth/fusion의 기여를
고정 입력에서 분리한다. 현재 작은 표본만으로 네트워크·loss·해상도를 추가 변경하지 않는다.

## 재현 자료

- Native·계획 감사: `artifacts/evaluation/base5_far_vehicle_261016/audit_native_and_plan_v1/`.
- E24 진단 정본: `artifacts/evaluation/base5_far_vehicle_261016/model_diagnosis_v1/with_candidate_class_details/report.json`.
- 이전 source 보존: Git `30cec44c`와 `source_v152_backup/dataset_runtime_at_30cec44c.tar`.
- R2 보고서: `native_report_v153c_r2.json`, 완료 resume: `terminal_resume_v153c_r2.json`.
- R2 runtime 원문: `source_v152_backup/runtime_v153c_r2_before_visibility.tar.gz`.
- 차량용 가림 제안의 저장 표본 대조: `far_visibility_proposal_cpu_v1.json`.
- 최종 새 후보/표본: `candidate_v153d_fast/`, `qualification_v153d_fast_far32/`.
- 새 native 보고/거리 요약: `native_report_v153d_fast.json`, `native_summary_v153d_fast.json`.
- 새 부분/완료 resume: `partial_resume_v153d_fast.json`, `terminal_resume_v153d_fast.json`.
- 새 runtime 원문: `runtime_v153d_fast_source.tar.gz`. 중단한 d 원문은 `runtime_v153d_before_fast_cpu.tar.gz`.

위 상대 artifact 경로의 기준은 `artifacts/evaluation/base5_far_vehicle_261016/`이다.

v152의 당시 `bulk_ready=true`를 현재 수정된 runtime의 승인으로 재사용하지 않는다.
이전 release·실패 raw·journal을 수정하거나 hash 검사를 우회하여 resume하지 않는다.
