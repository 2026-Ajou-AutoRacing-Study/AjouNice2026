# Foggy13 근거 기반 보강과 수집

문서 상태: EVIDENCE. 2026-10-19 KST. 학습 로그·최종 GT·실제 Foggy RGB를 함께 확인한 기록이다.
현재까지의 소량 진단을 5K 수집 완료나 AP 개선으로 해석하지 않는다.

## 목적과 보존

사용자가 로컬 Foggy13의 설계·소량 검증·5K 본수집 시작·웹 관측을 승인했다.
원격 Sunny11 수집과 완료 Sunny13/15 원본·선택 ledger·동결 실행기는 변경하지 않는다.
환경별 생성기 Python을 분리하지 않고 공통 생성기와 명시적인 보강 프로파일을 사용한다.
별도 사용자 요청인 Vast 새 Sunny13의 E24 조건 대조 학습은 독립된 작업·계약이다.

## 학습과 실제 GT의 대조

배치 부모 수가 아니라 Q90/upright 3클래스 최종 infos의 GT를 전수 집계했다.
학습 로그의 거리별 GT 54개 항목도 실제 infos와 일치했다.

| 최종 GT / 5K | 구 Sunny13 E24 입력 | 새 Sunny13 | 새 Sunny15 |
|---|---:|---:|---:|
| Vehicle | 4,519 | 4,496 | 4,499 |
| Pedestrian, 동물 포함 | 3,244 | 3,229 | 3,180 |
| Obstacle | 3,289 | 3,928 | 3,958 |
| Obstacle 0~20m | 2,013 | 2,834 | 2,844 |
| Obstacle 20~30m | 753 | 822 | 793 |
| Obstacle 30~40m | 278 | 110 | 158 |
| Obstacle 40~50m | 137 | 63 | 72 |
| Vehicle 80~100m | 219 | 318 | 304 |

30~40m 장애물 감소는 대부분 GT 제거가 아니라 배치 자체의 감소다. 해당 native component도
304→115/170개로 줄었다. 낮은 CargoBox의 20~30m GT는 47→35/40개로 감소했지만
Bollard01/04는 새 Sunny15에서 각각 44/46개로 늘었다. 얇은 객체만 늘려서는 낮은 박스 부족을
해결하지 못한다. 서로 다른 val의 AP 차이를 순수 데이터 개선량으로 단정하지 않는다.

Sunny15 cosine E8의 80~100m 차량 AP는 전방 51.92%(GT 22), 후방 94.10%(20)다.
restart도 같은 방향 편차를 보인다. 전방 train 122개 중 LiDAR 0점은 50개지만 후방은
140개 중 2개다. 단순 장수 부족으로만 설명하거나 먼 차량을 가까이 옮기지 않는다.
일반 보행자 20~30/30~50m AP는 86.68/80.64%다. 일반 사람과 동물·복합체의 subgroup recall은
따로 보며, 작은 동물 표본이나 복합체 camera 연결의 모호성을 전체 pedestrian GT 오류로 부르지 않는다.

근거: `artifacts/evaluation/base5_foggy13_design_261019/{training_review,data_audit}/`.
학습·자산·거리·방향별 CSV와 입력 SHA, component별 JSONL을 보존했다.

### 추가 요청: E24 조건의 새 Sunny13 학습 중간 대조

이 절은 학습 진행 중의 관측이다. 이후 완료된 24 epoch·Drive 백업·동일 val 교차 평가의
최종 결과는 [모델 근거의 완료 절](../../../docs/bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#sunny13-matched-e24-result-261019)을 따른다.

원형 로그와 새 학습의 E13까지 완료된 로그를 읽기 전용으로 대조했다.
기존 E24 실험의 실제 총 길이는 24에폭이다. 기존 학습 조건을 유지한 새 run을 시작했으며
이 중간 표는 **서로 다른 own-val**의 점수다. 순수 데이터 개선량으로 해석하지 않는다.

| 에폭 | 기존 epoch 평균 loss | 신규 epoch 평균 loss | 기존 own-val mAP | 신규 own-val mAP |
|---|---:|---:|---:|---:|
| 1 | 3.422 | 3.412 | 70.19% | 78.50% |
| 4 | 1.009 | 1.014 | 80.61% | 86.28% |
| 8 | 0.714 | 0.746 | 82.57% | 88.21% |
| 12 | 0.545 | 0.578 | 82.35% | 89.15% |

두 학습 모두 에폭 평균 loss와 유한 gradient 표본의 중앙값이 감소해 전반적인 학습 붕괴 징후는 없었다.
AMP 중 일부 비유한 gradient 로그가 있어 모든 optimizer update의 정상 수행을 인증하는
분석은 아니다. 기존 run의 own-val은 E9 이후 포화 경향이지만 신규는 E13까지 개선되어
아직 같은 포화 시점이라고 단정하지 않는다. 평가 GT는 차량 341→676, 보행자 687→468,
장애물 674→594로 달라졌다. 특히 신규 장애물 30~50m는 GT 29개에서 E12/E13 AP가
0.39/1.49%로 낮아, 전체 장애물 AP가 올라도 이 구간이 해결됐다고 볼 수 없다.
장애물 20~30m는 E8/E12에서 기존 72.90/76.18%, 신규 75.77/74.98%로 우열이 고정되지 않았다.

최종 24에폭과 재개용 Drive 백업이 실제 완료된 뒤 기존/신규 모델 × 기존/신규 val의
2×2 공통 평가를 실행하도록 별도 감시기를 준비했다. 네 평가는 동일한 current-nearest
depth raster·compile OFF·AMP·batch 1·증강 OFF 조건을 사용한다. 아직 이 최종 결과는 없다.
원형의 먼 거리 로깅은 E11부터라 앞 에폭의 빈 값은 AP 0이 아니다. GT 0도 정의되지 않은
AP로 구분한다. 상세 CSV·정의·원형 로그 SHA는
`training_review/matched13_progress_compare/`에 보존했다. 원격 학습·설정·백업은 변경하지 않았다.

### 전방 원거리 LiDAR 0점의 추가 원인 검사

새 Sunny13+15를 합친 80~100m의 실제 앞/뒤 차량은 305/317개이며, LiDAR 0점은
125개(41.0%)/3개(0.95%)다. 전방 0점 125개 중 100개는 native full-RPY 박스의 수직
범위와 교차할 수 있는 LiDAR 빔 자체가 없었다. 실측 도로점의 센서 좌표계 기울기 약
0.175°와 IMU pitch 약 0.17°가 유사하고, 지도상 객체·ego 지면 높이 차 중앙값은
양방향 모두 약 8mm다. 16채널의 빔 간격과 작은 센서/차체 자세가 함께 작용하는 설명이 강하다.

50~100m 차량 1,579개의 native 점군→full-RPY OBB 독립 재계산은 infos 점 수와 모두 일치했다.
native/infos 중심 차 최대 3.84µm, 8Points 모서리 차 최대 0.066mm이며 80~100m 차량의
ROI 제거점은 0이었다. 단순 축·강도·거리 crop·중심 오류는 발견하지 못했다.
이는 수직 envelope 기하 검사이며 가림·mesh·수평 샘플링까지 포함한 인과 인증은 아니다.
대조 중 센서 pitch·extrinsics·GT는 바꾸지 않는다. 전방 camera-only와 원거리 차량을
유지하고 따로 평가한다. 상세 근거는 `training_review/front_rear_sensor_diagnosis/`다.

## 실제 Sunny13 / Foggy13 동일 배치 진단

공식 `WeatherTimeChanger_Linux`로 로컬 Foggy·13시를 설정했다. 도구의 readback과 로그를
확인했으며 자동 순환은 OFF다. 사용자에 따르면 Foggy 버튼의 기본값은 Density=1·Distance=0이다.
ScenarioLoad는 날씨·시간을 다시 로드하지 않는다. 수집용 네트워크·센서·annotation 계약을 유지했다.

독립 학습 배치와 분리한 정적 28장과 camera-only 추가 4장, 총 32장을 캡처했다.
모두 native QA와 최종 정지/P를 통과했다. 기존 Sunny13의 실제 scenario SHA가 같은 장면만
대조했으며 baseline Q90 JPEG는 재압축하지 않았다. 새 Foggy PNG만 Q90으로 한 번 변환했다.

- 최초 28장: LiDAR 점 수가 26장은 같고 2장만 한 점씩 감소했다. BBox 중심 차이 최대 6.7mm,
  치수 차이 최대 13.7mm, yaw 차이 최대 0.0056°로 비교 geometry는 거의 같았다.
- 90m 차량과 35~49m 보행자는 RGB 대비가 줄었지만 실루엣이 남았다.
- LiDAR 0점인 CargoBox 22.4m·RedBarrel 45m도 실제 영상에 형태가 남았다.
- 추가 4장은 LiDAR 0점인 차량 89/97m와 보행자 27/37m다. Foggy에서도 영상 형태가 남았고
  현재 모델 crop의 국소 대비 잔존 비율은 약 0.22~0.46이었다.
- 67.6m 흰 PE 방호벽은 RGB가 매우 약해졌지만 LiDAR 3점은 유지됐다.

이 결과는 40m 일괄 상한이나 저점 수·카메라 전용 GT 일괄 삭제를 뒷받침하지 않는다.
Instance mask는 안개에서도 거의 유지되므로 RGB 가시성의 대용으로 쓰지 않는다.
국소 대비 수치는 진단용이며 AP·검출 가능성·GT 삭제 threshold가 아니다.
이번 MORAI 표본의 LiDAR 불변 경향을 실제 안개에서도 LiDAR가 감쇠하지 않는다는 물리 주장으로
확대하지 않는다. 실제 배포 환경과 시뮬레이터 센서 모델을 구분한다.
28장 최종 정량은 `data_audit/paired_rgb_report_v2/report.json`을 사용한다.
v1의 그룹 중복 집계 오류는 v2에서 수정했으며 과거 파일은 보존한다.
추가 4장은 `data_audit/camera_only_four/rgb_report_v1/`에 별도 보존한다.

## 보강 정책과 검증 경계

독립 seed `26101913`의 공통 v160 baseline 5K를 새로 생성했다. 이전 Sunny 장면을 복제한
학습 5K가 아니다. 같은 정책에서 나온 원형 배치를 보존하고, 별도 명시 프로파일
`configs/dataset/base5_midrange_reinforcement_v163.json`로 풍부한 generic 근거리 장애물 일부의
primary 거리만 재할당하는 후보를 만든다.

- 20~30m는 낮은 박스와 다양한 물리 장애물이 중심이다.
- 30~45m는 기존 native 치수 정책과 실제 관측 근거에 맞는 큰 물체를 우선한다. 모든 자산의
  공통 상한을 40m로 새로 정한 것이 아니다.
- 자산·부모 수·motion·미션·capture split·원형 scale을 보존하고 현재 near actor 감소 예산을 제한한다.
- Vehicle과 pedestrian의 기존 원거리·이동·동물·복합 구성은 보존한다. 낮은 AP만 보고 분포를
  강제로 바꾸지 않으며 별도 진단과 후속 common-val 평가 대상으로 남긴다.
- 기존 v160 receipt를 수정하지 않는다. 새 거리 계약과 build extension, native 등록·release를
  별도로 검증한다. 본수집의 native QA·CollisionData·정지/P 기준은 완화하지 않는다.

CPU 공통 데이터 회귀는 2,376개 통과·1개 명시적 deselect다. 보강 프로파일과 공통 실행기
회귀·소량 native·최종 여섯 자격 검사·5K 시작은 각각 실제 완료 뒤 이 문서에 추가한다.

### 생성 가능성과 native 일반 실패를 구분

첫 CPU 후보에서는 원거리 primary를 옮길 때 기존 가까운 보조 객체를 보호해야 하는
clustered 구조의 충돌을 발견했다. 일부 front 30~45m 후보는 보호된 앞쪽 보조 객체에
가리거나 기존 허용 공간과 맞지 않아 유한 512/4,096회 제안에서도 생성되지 않았다.
허용 도로를 넓히거나 보조 객체를 삭제하지 않는다. 다음 후보는 실제 생성 로직으로
CPU 배치 가능성을 먼저 확인하고, 그 결과와 탈락 사유를 명시해 결정한다.
이는 실제 캡처 성공 여부를 보고 쉬운 장면만 고르는 절차와 다르다.
v161 설정·코드는 첫 후보의 이력이며, v162는 CPU 사전 생성 가능성 검사와 겹치는 자산의
가장 엄격한 누적 근거리 보호 예산을 명시한 별도 후보다. 환경별 Python 분기가 아니다.
v162의 rear concrete-barrier 한 후보는 512회 중 511회 도로 밖, 1회 footprint 간격 문제로
실패했다. 실제 native 실패를 보고 제외한 것이 아니다. v163은 해당 자산의 30~45m 요청만
8→4로 명시 변경했다. 다른 자산·거리·seed·근거리 예산은 유지한다.

v163은 CPU 150개 후보 중 149개의 실제 생성 가능 geometry를 확보했고, 최종 149개 할당을
모두 채웠다. 20~30m 118장·30~45m 31장을 재배치하며 기존 5K 장수·actor 구성은 유지한다.
대상 19종 물리 자산의 계획상 근거리 부모는 2,857→2,708개다. 이는 최종 학습 GT 개수가
아니며, 실제 승인분 변환 뒤 거리별 GT를 따로 센다. 32장 pilot과 최종 full의 해당 payload는
바이트 단위로 같다. source·분할·전체 CPU 감사 통과를 native 5K 완료로 해석하지 않는다.

공통 portable 도구 전체 회귀는 실제 호스트에서 1,727개 통과했다.
샌드박스에서는 localhost 소켓 권한 검사 하나가 PermissionError였으며, 그 실패 XML도
보존하고 권한이 있는 동일 호스트에서 전체를 다시 실행했다. 최종 근거는
`portable_regression_v163_host.xml`이다. 기존 동결 dataset 회귀 2,376개와 다른 시험 범위다.

소량 native의 `...00997`은 21.17m 흰 PE 방호벽 자체가 LiDAR 9점·카메라 양쪽에서
확인됐지만 보조 Man5가 로드 뒤 빠졌다. 주 객체 GT 오류나 안개 때문에 보이지 않은
경우가 아니다. 검토 원본과 실제 대체 배치를 보존하며, 원래 geometry가 승인된 것처럼
기록하지 않는다. 최종 source가 바뀌면 그 실제 배치와 등록·분할·검증 계보를 다시 연결한다.

첫 보강 pilot은 최종 배치 후보를 바꾸기로 한 뒤 공식 STOP 경로로 종료했다. 당시 부모
체크포인트는 검사 28·승인 27·부족 1이며, 이는 최초 sunny/foggy 비교 32장과 별개다.
STOP 때문에 남은 원래 상태는 `halted`이며 정상 완료로 바꾸지 않았다. 실제 native session
12개의 종료 Park는 모두 확인됐고, 수집 자식 프로세스가 없으며 최신 Ego의 정지·brake=1을
확인했다. 단순 센서 근거 부족과 보조 actor 로드 누락 원본·복구 시도를 모두 보존한다.
이 후보를 exact32 자격 통과로 사용하지 않는다. 당시 portable 코드와 설정은
`generator_review/snapshots_v1/`에 별도 해시와 함께 보관했다.

### v163 실제 보강 진단과 지도 가림

새 등록의 원본 32장은 native 승인 29·검토 3이었다. 실제 새 배치 3개의 승인으로
canonical 32개를 확보했고 부모는 오류·부족 0으로 종료했다. 원본 32/32 승인이라고
표현하지 않는다. 이 시험은 변경 대상과 원거리 자산을 의도적으로 많이 포함한 진단이므로
3/32를 그대로 5K 예상 실패율로 사용하지 않는다. 원본·선택·복구·종료 Park 근거는
`reinforcement_pilot_capture_v2/`에 보존했다.

검토된 TrafficBarrel `...00128`은 계획 Ego 거리 42.36m, 실제 LiDAR 중심 거리 41.54m다.
native 8모서리를 실제 calibration으로 투영한 CAMERA_1·2의 영역은 각각 306/306·144/144개
semantic pixel이 중앙 가드레일과 같은 색이었다. CAMERA_2의 정확한 투영 bbox에는
7.98~9.33m의 더 가까운 LiDAR 점 5개도 있었다. RGB 확대에서도 검은 차폐 기둥과 겹친다.
이 사례는 지도 구조물 가림의 근거가 강하다. 원본 8-bit Depth의 단위는 검증하지 않아
미터로 임의 해석하지 않았고, ray 단위 전면 차폐나 안개 인과까지 증명한 것은 아니다.
안개만의 소실로 분류하거나 40m 일괄 상한의 근거로 쓰지 않는다.
정량·분석 이미지는 `case_visibility/`에 보존했다.

### 최종 자료 검사 비용과 경계

CPU 조립의 전체 5K 감사와 최초 binding 대조는 통과했지만, 같은 자료를 후행 재검산하는
비용이 컸다. 중복 CPU 작업 두 개는 메모리 여유를 확보하려고 SIGTERM으로 종료했고
exit 143·중간 출력·로그를 보존했다. 이 종료를 검증 성공으로 쓰지 않는다.
`duplicate_cpu_cleanup_v1.json`에 실제 PID·명령 SHA·보존한 binding SHA를 남겼다.
최종 자격은 별도 실제 binding 재검증과 원래 native 여섯 검사로 확인한다.

처음에는 과거 base 참조에만 SHA 재사용을 적용했으나, 읽기 전용 관측에서 새로 생성이
끝난 `01_fresh/manifest.json`·분할표도 반복 해시 계산되는 것을 확인했다. 다음 검사 경로는
정확한 binding에 있는 정적 source·코드·지도·설정만 명시적으로 고정한다. 시작·끝에는
실제 SHA를 다시 읽고, 재사용 시 파일·상위 디렉터리의 identity 변경도 거부한다.
실제 native raw·session·ledger·event·audit·review와 JSON 객체·QA 결과·context 계산은
재사용하지 않는다. 실제 촬영 전에는 이 CPU scope를 완전히 해제한다.
이 최적화는 동결 dataset 스크립트나 촬영 QA를 수정하는 작업이 아니다.

실행용 helper와 회귀 근거는 `static_input_cpu_scope_v2.py`·`run_static_phase_v2.py` 및
같은 artifact 폴더의 테스트·실측 receipt로 보존한다. scope 종료 기록은 경계 검사 완료이며
원래 작업 자체의 성공은 아니다. 단계별 종료코드와 실제 결과 파일을 함께 확인한다.
새 scope 회귀 79개가 통과했다. 실제 정적 입력 25,432개(약 1.186GB)를 시작·끝에 읽고
frozen/portable의 실제 SHA alias로 600회 반복 대조·복원을 확인한 시험은 약 7.16초였다.
8,598개 근거는 이 재사용 대상에서 제외했다. 이는 SHA scope 자체의 측정이며 전체 자격시험이나
실제 촬영 시간이 7초라는 뜻이 아니다. JSON parse·실제 QA·원본 근거 대조 비용은 남는다.

`register_static_v2`의 실제 원형 등록은 약 617.1초 뒤 exit 0으로 완료됐다. 원래 binding 검증과
source·전체 감사·보존 검사를 통과해 `qualification_v1/registered/`에 32장을 사전 등록했다.
재사용 SHA 1,839,293회·변경 오류 0이었으며 양경계의 실제 정적 파일 검산도 완료됐다.
이는 새 native 촬영의 성공이 아니다. 뒤 첫 16장 실행의 중복 CPU 사전검사는
native intent·출력·자식 프로세스가 모두 없는 상태에서만 중단했고, 해당 exit 143도 보존했다.
기존 등록 성공과 이 중단을 혼동하지 않는다.

잔여 병목은 SHA만이 아니었다. 작업용 `local_workflow.py`가 모든 단계 앞에서 전체
`verify_binding`을 호출하고, register/freeze 본문도 같은 검증을 수행해 계보 계산이 중복됐다.
`direct_phase_v3.py`는 원래 단계 본문을 유지한 채 이 추가 공통 사전검사만 분리한다.
register/freeze의 자체 전체 검증·원래 여섯 자격 검사는 유지한다. utility도 원래 함수로
검사하고 최종 freeze에서 다시 통합 대조한다. 성공했다고 가정한 QA/context를 캐시하지 않는다.

native 진입은 실제 성공한 등록(exit 0), source·assembly·원형 helper/runtime, 정확한 32장과
선행 등록 시각을 확인한다. 원형 선정·coverage·raw scale·profile도 실제 자료에서 다시 계산한다.
binding 근거 34,030개(약 2.850GB)는 시작·끝에 전량 실제 SHA를 읽는다. 등록과 v3 helper의 SHA도
원형 intent pins에 추가해 worker·재개에서 대조한다. native 실행 중 SHA scope는 설치하지 않는다.
115개 회귀와 독립 리뷰를 통과했고, 실제 등록·전체 해시 경계의 읽기 전용 시험은 약 15.15초였다.
근거는 `direct_phase_v3_validation_receipt.json`·`direct_phase_v3_tests.xml`이다.
이후 실제 첫 16장 촬영에 진입했다. 개별 승인·재개·최종 자격은 각 실제 결과로 별도 확인한다.

### 최종 source 32장 시험과 추가 복구

`qualification_v1/exact32/`는 첫 16장을 전부 승인한 뒤 정상 일시정지·partial snapshot 저장을
거쳐 재개했다. 06:21 KST 실제 부모 종료 상태는 **승인 32·부족 0·실패 0·오류 없음**이다.
다만 후속 입력 변환에서 1개 복구 overlay를 발견했다. 원래 geometry의 무검토 32/32나
최종 exact32 자격 통과로 해석하면 안 된다. 원형 31장과 실제 새 복구 1장이며, 이 32장을
본수집 5K에 더하지 않는다. 당시 완료 재개 검사는 통과했지만 입력/최종 자격은 미통과였다.

해당 canonical `...00674`는 고주로 뒤차 10~30m 장면이다. 원본 regular ledger의
`population_visible_actor_deficit` 검토를 보존하고, ordinal 2의 실제 승인 배치
`...00674_replace_replace`로 회복했다. source payload `e1337c91...`와 승인 payload
`74cc668d...`가 다르므로 같은 source의 exact 관측으로 셀 수 없다. 입력 qualifier의 첫 overlay
자동선택과도 충돌해 `Captured split overlay content differs`로 중단됐다.
이는 GT 손상 증거가 아니라 당시 authoritative source로 채택하지 않은 정상 복구 계보 문제였다.
31+1 overlay를 임시 합치거나 32장을 전체 5K native 선택으로 위장해 finalize하지 않는다.
이미 지원하는 depth 2 실제 native adoption으로 source를 새로 조립한 뒤 새 등록·촬영으로 검증하기로 했다.
기존 source·등록·원본·입력 변환 실패 출력과 로그는 그대로 보존한다.

### 00674 원형과 실제 복구의 가림 대조

원형과 실제 복구 3회, 총 4개 캡처를 읽기 전용으로 대조했다. 모두 예상 UID 8500/8501이
native GT와 촬영 전후 Object_topic에 있었고, 원형의 실제 위치 차이는 각각 약 5.4/5.2mm였다.
NPC 로드 누락보다 가까운 MPV가 보조 Sonata를 가린 근거가 강하다. 원형 Sonata는 17.16m에서
LiDAR 0점·카메라 instance 0픽셀이었고, 대상 OBB를 지나는 실제 반환 방향 52개 모두 앞 MPV
안에서 먼저 반환됐다. 대상 박스를 5cm 넓히고 앞차 박스를 면마다 10cm 줄인 보수적 기하
검사에서도 LiDAR·후방 카메라 각각 27/27개 표본이 가렸다. 반면 실제 승인 복구의 Sonata는
더 먼 40.90m에서도 LiDAR 6점·카메라 295픽셀, 현재 512×1056 입력 crop에서 204픽셀이 남았다.

OBB는 실제 mesh가 아니므로 완전 차폐나 안개 인과의 인증으로 확대하지 않는다. 동일 geometry의
반복·맑음 대조도 없어 전체 실패율이나 Foggy 거리 상한·차량 quota 변경의 근거가 아니다.
완전히 가려진 보조 객체를 기존 품질 검토·실제 새 배치 복구로 처리한 사례로만 해석한다.
이 분석은 부모 승인 32와 원형 승인 31+복구 1의 구분을 바꾸지 않는다.
이 분석 기록 당시 source3의 depth 2 native adoption을 준비 중이었으며 본수집 시작이나
최종 자격 통과가 아니었다. 이후 실제 완료 상태는 다음 절을 따른다.

근거는 `generator_review/case00674_native_visibility_v1.json`과
`generator_review/case00674_native_visibility_validation_v1.json`이다. 입력 77개 SHA를 다시
확인했고, 4개 캡처의 8개 component 포인트 재계산은 모두 audit와 일치했다.
같은 폴더의 `analyze_case00674.py`로 읽기 전용 재현이 가능하다. 원본·runtime·정책은 변경하지 않았다.

### Depth 2 실제 채택 완료와 새 최종 검증

06:48 KST 원형 `prepare`는 후행 `verify_binding`까지 실제 exit 0으로 완료됐다.
실행 시간은 약 1,238.7초였으며 정적 SHA scope의 양경계 실제 검산과 failures 0을 확인했다.
새 source는 `reinforced_source_v3/assembled_source/final_source/manifest.json`이며 SHA256은
`83f3b86fc1d8640e9cd148054f04923ff04dd89e08979563b8d7390eccbae595`다.
이전 source의 4,999 payload와 149개 보강 목표·actor 순서·capture split을 보존하고,
canonical `...00674` 한 장만 실제 ordinal 2 승인 geometry로 채택했다.

새 binding과 이전 depth 1 부모의 연결을 유지한다. 새 source의 native exact32는 별도
`qualification_v2`에 사전 등록한 뒤 다시 관측했다. 원형 등록 함수는 한 번만 호출했으며,
이전 v3 단계 본문을 변경하지 않은 새 `direct_phase_v4.py`가 정확한 새 binding·출력 경로를
고정한다. source 양쪽의 정적 파일 SHA만 재사용하고 native raw·session·ledger·QA/context는
재사용하지 않는다. 독립 부정 테스트를 포함한 v4·v3·static 회귀 151개가 통과했다.
`source3_preparation_validation_receipt.json`과 `direct_phase_v4_code_validation_receipt.json`에
실제 입력 SHA·완료 코드·회귀 결과를 보존한다. 이 준비 완료는 native 자격이나 본수집 시작이 아니다.

새 source의 원형 등록은 약 488.4초에 exit 0으로 완료됐다. 이후 `qualification_v2/exact32`의
원형 regular ledger 32개가 **모두 승인**됐고, 선택 ID도 원형 32개와 같으며 recovery ledger는
없었다. 첫 16장 뒤 정상 일시정지·partial 저장, 나머지 16장 재개와 완료 상태의 읽기 전용
재개 검증도 통과했다. 앞선 `qualification_v1`의 31+1과 다른 실제 새 관측이다.

동일 32장의 Q90/upright 입력 변환과 GT 76개를 실제 CPU train/val 전처리·packing 경로
모두에서 확인했다. train 32·val 32는 같은 소량 자료를 두 입력 경로로 시험한 숫자이며
독립 64장이나 실제 학습 분할 장수가 아니다. GPU 학습이나 AP 검증도 아니다.
별도로 사전 등록한 음영 이동 animal/human 두 장면도 **2/2 승인·검토 0·오류 없음**과
실제 종료·이동 근거 대조를 통과했다. 소량 시험은 본수집 5K에 합산하지 않는다.

최종 원형 freeze는 약 562.2초에 실제 exit 0으로 완료됐다. native exact·독립 음영 이동·
training inputs·CPU 회귀·partial resume·completed resume **여섯 gate가 모두 통과**했고,
`qualification_v2/release/release.json`의 `bulk_ready=true`, 저장 공간 `start_blockers=[]`를
확인했다. 당시 `bulk_started=false`와 학습 관측 0은 올바른 동결 기록이다.
이후 실제 공통 CLI의 `--prepare-fresh`를 시작했다. release 통과만으로 본수집 시작을
주장하지 않으며, 공통 실행기의 5장 촬영·정상 재개 검사와 실제 5K 출력 상태를 별도로 확인한다.

<a id="foggy13-live-261019"></a>

## Foggy13 본수집 시작과 관측

07:27 KST 공통 실행기 5장 시험이 **5/5 승인**과 실제 일시정지·재개·추가 촬영 없는 완료
재생 검사를 통과한 뒤 5K 본수집에 진입했다. 첫 묶음의 **실제 새 원본 16장·승인 16·검토 0·
오류 없음**과 다음 묶음 진행을 확인했다. 앞선 진단·exact32·음영 이동·공통 5장 시험은
본수집 승인 수에 더하지 않는다. 이는 5K 완료가 아니라 실제 본수집 시작 확인이다.

- 최종 source: `reinforced_source_v3/assembled_source/final_source/manifest.json`.
- 동결 자격: `qualification_v2/release/release.json`.
- 공통 프로파일: `qualification_v2/common-profile.json`.
- 실제 본수집: `~/morai_cap/base5_foggy13_final_v163_261019/collection_5k_v1`.
- 수집 세션: `foggy13-common-depth2-v1`. 실행 중에는 같은 명령을 중복 실행하지 않는다.
- 대시보드: **http://localhost:8081**, 세션 `foggy13-dashboard-8081`.
  HTTP에서 정확한 Foggy13 manifest·5K 목표·실제 새 승인 갱신을 확인했고 브라우저도 열었다.

기존 로컬 관측기와 빌린 PC Sunny11 수집·대시보드는 중단하거나 변경하지 않았다.
일반 센서 근거 부족은 기존 검토·부족분/실제 새 배치 복구로 처리하고, 통신·정지/P·동결
입력 검증 실패는 계속 안전 중단한다. 새 PNG/native 원본의 삭제나 JPEG 최종 전달본 변환은
이번 본수집 시작 단계에서 수행하지 않았다.

공통 초기 준비는 약 501.0초였다. 읽기 전용 15.01초 표본에서 CPU 15.00초·논리 읽기
약 5.62GB·실제 disk read 0B였으므로 회선/디스크 대기보다 반복 SHA/JSON CPU 비용의
근거가 강하다. portable의 미리 가져온 SHA 별칭 일부는 기존 frozen-directory 설치 범위에서
빠진다. SHA와 JSON 각각의 기여율은 측정하지 않았다. 현재 실행·동결 코드를 바꾸지 않고
`common_prepare_cpu_readonly_diagnosis_v1.json`에 후속 개선 근거로 보존했다.
준비 때 저장한 `source_authority`와 전체 경계 검산을 사용하므로, 이 전체 자격 context가
본수집의 매 복구에서 다시 실행되는 구조는 아니다. 새 대체 자료·실제 QA 검사는 유지한다.

Vast 새 Sunny13의 동일 조건 24에폭 학습과 Drive 백업도 별도 세션에서 진행 중이다.
최종 데이터 개선량·새 production 선정·30K 수집 완료는 아직 인증하지 않는다.

위 문장은 본수집 시작 당시의 기록이다. 이후 새 Sunny13 학습·백업·동일 val 네 칸 평가는
완료했다. [최종 모델 비교](../../../docs/bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#sunny13-matched-e24-result-261019)를 따르며,
Foggy13 거리 보강의 학습 결과나 production 교체로 해석하지 않는다.

<a id="foggy13-crash-continuation-261019"></a>

## MORAI 종료와 첫 crash 인계

이 절은 첫 사고와 당시 복구의 이력이다. 이후 두 번째 종료 뒤의 현재 출력·실행 명령은
[두 번째 crash 인계](#foggy13-second-crash-continuation-261019)를 따른다.

본수집의 마지막 확정 checkpoint는 **승인 2,043장·검토 21장·검사 완료 2,064장**이다.
이후 `chunk_0129`에서 Ego/Object/CollisionData 수신이 끊겨 마지막 Park를 확인하지 못했고,
부모는 `Batch final Park unverified`로 중단했다. 사용자도 MORAI가 꺼졌음을 확인했다.
MORAI의 `Player-prev.log`에는 fatal signal 11이 남았다. 그 앞의 차량 제어 예외만으로
crash의 직접 원인을 확정하지 않는다. 당시 고정 입력 검사는 통과했으므로 hash 불일치를
통신 중단 원인으로 해석하지 않는다.

마지막 미확정 묶음의 native 15행은 승인 후보 14개와 stale 실패 1개다. 원본과 실패 근거는
그대로 보존하되 **이 15행을 최종 선택에 넣지 않고 다음 16장 묶음 전체를 다시 취득**한다.
이미 확정된 2,043개 선택 행·21개 부족 사유·과거 경과시간은 새 출력으로 인계한다.
구 출력의 `halted`나 `park_verified=false`를 정상으로 바꾸지 않는다.

일반 환경용 `scripts/portable_base5/recover_unified_fresh.py`를 별도로 추가했다.
원형 source·여섯 release gate·공통 5장 자격·fresh empty origin을 변경하지 않고,
기존 source 권한을 원래 출력 기준으로 검사한 뒤 새 sibling 출력에 명시적으로 연결한다.
worker의 parent PID·start ticks·sealed memfd 검사는 유지한다. 기존 동결 수집 코드는
변경하지 않았다. **원래 fresh 출력의 첫 crash 인계만 지원**하며 반복 crash 전부를
자동 복구하는 기능을 완성했다고 주장하지 않는다.

CPU 인계는 기존 crash verifier를 그대로 사용한다. 이 verifier는 파일 hash만 읽는 것이
아니라 **기존 묶음의 static-attitude 지면 판정도 다시 계산**하므로 준비 비용이 든다.
5K 재촬영이나 매 장면마다 수행하는 새 검사는 아니다. 검증을 생략해 과거 승인분을
임의 채택하지 않는다. 실제 준비 종료와 새 native 승인은 별도로 확인한다.

- 기존 출력: `~/morai_cap/base5_foggy13_final_v163_261019/collection_5k_v1`.
- 새 출력: 같은 부모의 `collection_5k_v1_after_crash_v1`.
- 기존 프로파일: `artifacts/evaluation/base5_foggy13_design_261019/qualification_v2/common-profile.json`.
- 새 프로파일·로그·종료 코드: `artifacts/evaluation/base5_foggy13_crash_recovery_261019_v1/`.
- CPU 인계 세션: `foggy13-crash-import-v1`; 현재 실제 수집 세션은 `foggy13-crash-resume-importpath-v2`다.
- 재시작 후 raw 통신·P/gear 1을 실제 확인했고, 사용자에게 Capture Mode와 카메라 1~4·LiDAR 5 annotation ON도 재확인받았다.
- 공식 도구로 Foggy·13:00을 적용했다. 새 검증이나 촬영을 Sunny로 대체하지 않는다.

집중 회귀 323개와 별도 두 새 실행기 테스트 82개를 통과했다. portable 전체는
1,805개 통과·기존 socket 테스트 1개 sandbox 권한 실패였고, 해당 대시보드 테스트를
host에서 다시 실행해 25개 모두 통과했다. 이 수는 중복 없는 단일 합계로 더하지 않는다.
원본·PNG 삭제나 JPEG 최종 변환은 이 crash 복구에서 수행하지 않는다.

### 실제 인계·재개 확인

CPU 인계는 19:31 KST에 **exit 0·승인 2,043·부족 21·확정 2,064·새 촬영 0**으로
완료했다. 첫 `--run`은 원형 모듈 검색 경로가 없어서 native 진입 전에 실패했다.
`verify_fresh_prefix`가 `common.modules()`보다 먼저 importer를 불러오는 호출 순서가
원인이었다. 이 실패 로그·exit 1도 보존했다. 동결 wrapper나 준비된 profile·SHA를
고치지 않고, 해당 Python 실행에만 원형 scripts의 `PYTHONPATH`를 명시해 재실행했다.
실물 원형 importer 두 모듈을 읽는 CPU 회귀도 추가했다.

19:36 KST **새 첫 묶음 16/16 승인·검토 0·최종 P/gear 1·오류 없음**과 다음 묶음
진행을 확인했다. 부모 checkpoint는 **승인 2,059·검토 21·확정 2,080장**이다.
기존 2,043개 선택 행이 모두 그대로 있고 ID·원본 경로가 고유하며, 구 미확정 suffix의
원본은 하나도 선택되지 않았다. 원형 재개부터 실제 worker/중첩 worker까지 성공한 결과다.
미확정 묶음을 다시 찍어 얻은 새 16개이므로 과거 후보 14개와 중복 합산하지 않는다.

8081의 구 관측기만 종료하고 새 continuation profile로 전환했다. 기존 서버 종료 뒤
TCP TIME-WAIT 때문에 포트 재사용을 잠깐 기다렸으며, 수집은 중단하지 않았다.
**http://localhost:8081/** 의 실제 HTTP·새 root·승인 갱신을 확인했다.
근거는 위 recovery 폴더의 `prepare.{log,exit}`·`resume.{log,exit}`·
`resume_importpath_v2.log`·`live_observation_1792406200623990734.json`이다.

다음 명령은 **현재 수집 중에는 중복 실행하지 않는다.** 정상 PAUSE 뒤 같은 Foggy13에서
재개할 때만 사용한다. 새로운 crash나 Park 미확인을 이 명령으로 강제 우회하지 않는다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
PYTHONPATH="$PWD/dataset/morai_od/scripts${PYTHONPATH:+:$PYTHONPATH}" \
python3 -B scripts/portable_base5/recover_unified_fresh.py --run \
  --source-profile "$PWD/artifacts/evaluation/base5_foggy13_design_261019/qualification_v2/common-profile.json" \
  --profile "$PWD/artifacts/evaluation/base5_foggy13_crash_recovery_261019_v1/continuation-profile.json" \
  --output /home/luv/morai_cap/base5_foggy13_final_v163_261019/collection_5k_v1_after_crash_v1 \
  --confirmed-environment foggy13
```

<a id="foggy13-second-crash-continuation-261019"></a>

## 두 번째 MORAI 종료와 계보 보존 인계

첫 continuation에서 **승인 2,657장·검토 31장·확정 2,688장**까지 수집한 뒤 다시
MORAI 연결이 끊겼다. 기존 출력의 오류는 `Batch final Park unverified`이며 마지막
`chunk_0168`은 native 후보 15개와 stale 실패 1개다. 이 미확정 16행의 원본과
`park_verified=false`는 그대로 보존했다. 후보 15개를 승인으로 합산하거나 기존
`halted`를 정상으로 고치지 않고, 동일한 다음 16장 묶음을 새 출력에서 다시 취득했다.

기존 `recover_unified_fresh.py`는 원래 fresh 출력의 첫 인계만 지원한다. 이미 동결된
이 파일을 고치지 않고 **`scripts/portable_base5/recover_unified_fresh_chain.py`**를 추가했다.
이 실행기는 첫 wrapper의 `prepared` 검증을 재사용해 이전 profile·receipt·원형 공통
5장 자격을 확인한다. 원래 source 권한의 `canonical_output`과 빈 `old_output` origin은
처음 값 그대로 유지하고, 바로 이전 profile의 SHA·출력·계보 깊이를 새 계약에 기록한다.
중첩 `imported_prefix`는 기존 crash exporter와 원형 schema 1 verifier로 검증한다.
worker의 parent PID·start ticks·sealed memfd·허용된 native target 범위도 유지한다.
반복 인계 계보를 지원하지만 MORAI의 자동 재시작이나 모든 사고의 무인 복구를
보장하는 기능은 아니다. 새 crash에 기존 출력을 강제로 resume하지 않는다.

CPU 준비는 **21:49:17~21:57:27 KST, 8분 10초·exit 0**으로 완료했다.
기존 2,657개 선택 행·31개 부족 사유·경과시간을 보존했고 준비 중 새 촬영은 0장이다.
원형 완료 native 세션 103개와 recovery 41개의 근거를 확인했다. 새/첫 복구·공통 수집·
crash/paused import·경계 adapter 집중 회귀 **375개가 통과**했다. XML은 새 recovery
폴더의 `tests.xml`에 보존했다. 기존 수집기·QA·manifest·원본은 수정하지 않았다.

| 항목 | 현재 경로·근거 |
|---|---|
| 보존된 이전 출력 | `~/morai_cap/base5_foggy13_final_v163_261019/collection_5k_v1_after_crash_v1` |
| 실제 새 출력 | 같은 부모의 `collection_5k_v1_after_crash_v2` |
| 이전 profile | `artifacts/evaluation/base5_foggy13_crash_recovery_261019_v1/continuation-profile.json` |
| 새 profile·로그·receipt | `artifacts/evaluation/base5_foggy13_crash_recovery_261019_v2/` |
| CPU 준비 세션 | `foggy13-crash-import-v2` — 정상 종료 |
| 실제 수집 세션 | `foggy13-crash-resume-v2` |
| 관측 | **http://localhost:8081/** — v2 profile에 연결, 관측기만 전환 |

이전 profile SHA는 `010b535d5556cb25571ebd85dfa94c41696e0cc9290862d612da564ef3b085f4`,
첫 wrapper SHA는 `706c603fb13a329b36e3e72d0e108b5a3a0f899b6355b5fd2ce8cd55f7214a50`로
그대로다. 새 profile SHA는 `2caae18f80adc8e9d9019f3d1232bc15d51c4e0c0d99f661d075ef601fbb7871`,
새 imported prefix SHA는 `eb058e95cff6de9eaf94a7b5892debc34ba84f723000ced49871c488da251140`이다.

### 실제 재촬영·승인·관측 확인

사용자가 재시작 후 Capture Mode와 카메라 1~4·LiDAR 5 annotation ON을 재확인했다.
이후 실제 통신·P/gear 1과 공식 도구의 Foggy·13:00을 확인한 뒤 재개했다.
**22:03:17 KST 첫 새 묶음 16/16 승인·검토 0·실패 0·최종 P/gear 1**을 확인했다.
각 행의 중간 P와 native audit 승인도 모두 통과했다. 새 원본 경로는 v2 아래이며,
구 미확정 후보 15개의 원본 경로가 선택에 들어가지 않았다. 따라서 첫 묶음의 확정
신규 승인은 기존 2,657장에 더하는 16장이지, 구 후보 15장까지 더한 31장이 아니다.

16장 모두 카메라 Instance/RGB 4개 뷰, 총 64세트와 LiDAR annotation 16세트를 확인했다.
카메라 BBox text는 26개 뷰에 쌍으로 존재했고 나머지 38개 뷰에는 없었다. 전 뷰의
Instance PNG와 원형 native QA 통과를 확인했으며, 각 카메라에 BBox text가 반드시
있어야 한다는 별도 조건을 추가하지 않았다. BBox text 부재만으로 annotation 설정
OFF라고 판정하지 않는다. 이 검사는 파일 존재·native QA·계보의 확인이지 각 뷰의
상세 가시성·GT 기하 정확도나 학습 AP 인증은 아니다.

**22:06:32 KST** 읽기 전용 관측에서 부모는 **실행 중·승인 후보 2,686·검토 32·
검사 2,718·오류 없음**이며 다음 `chunk_0169`가 진행 중이었다. 공통 bulk 실행기는
64개 검사 경계마다 부모 선택 ledger를 snapshot하므로 당시 부모 ledger 2,657행·
`checkpoint_finished=2688`과 더 최신 native 진행 수가 다른 것은 정상이다.
기존 2,657행이 모두 그대로 남아 있음을 비교했고, 확정 checkpoint와 진행 중 후보를
중복 합산하지 않았다. 근거는 `live_observation_1792415192734348738.json`이며 SHA는
`34fa7374b2aaed88fe2c1799734a065a9ffd575852d09e298c6427d722597f3d`이다.

재개 직후 부모 상태가 잠시 `prepared_waiting_for_resume`이고 상위 로그가 비어 있어도
무한 대기라고 단정하지 않는다. 이 재개는 원형 imported-prefix 검증을 두 번 수행한다.
각 검증은 증거 파일 123,152개·약 17.38GB를 읽으며 그중 센서 파일은 약 10.44GB다.
그 외 고정 입력 경계·source 확인도 있다. 실제 native worker가 이미 시작된 동안에도
부모 checkpoint와 상위 로그는 늦게 갱신될 수 있어 child `status.json`·ledger·batch log를
함께 확인했다. 기존 동결 검증을 이번 실행 중 생략하거나 바꾸지는 않았다.

아래는 **현재 실행 중에는 중복 입력하지 않는다.** 같은 환경의 정상 PAUSE 뒤
재개에만 사용한다. 새로운 crash나 Park 미확인에는 새 안전 인계가 필요하다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
python3 -B scripts/portable_base5/recover_unified_fresh_chain.py --run \
  --source-profile "$PWD/artifacts/evaluation/base5_foggy13_crash_recovery_261019_v1/continuation-profile.json" \
  --profile "$PWD/artifacts/evaluation/base5_foggy13_crash_recovery_261019_v2/continuation-profile.json" \
  --output /home/luv/morai_cap/base5_foggy13_final_v163_261019/collection_5k_v1_after_crash_v2 \
  --confirmed-environment foggy13
```

새 실행기는 원형 scripts의 모듈 검색 경로를 직접 등록하므로 첫 사고 때의 별도
`PYTHONPATH` 보완 없이 worker까지 동작한다. 원형 importer 실제 로딩과 두 번째·
세 번째 인계 fixture를 회귀 검사했다. 이 작업에서 PNG 삭제·JPEG 변환·5K 완료
판정은 수행하지 않았다.

### 10/20 종료 후 보충 경로의 사전 점검

본수집이 아직 실행 중인 상태에서 종료 후 보충 경로만 읽기 전용으로 점검했다.
현재 crash-chain의 `old_output`은 최초 origin이고 prefix의 `original_output`은 직전
crash 출력이므로 일반 completion의 경로 동일성 비교를 그대로 적용할 수 없다.
새 운영 helper는 기존 chain receipt·prefix SHA를 확인하고 그 비교 한 곳만 직전 부모로
연결한다. 원형 source authority·native QA·승인/검토 판정은 유지하며 기존 Sunny11에서
사용한 0촬영 seed API와 공통 유연 보충·finalize를 재사용한다.

`artifacts/evaluation/base5_foggy13_completion_261020/`의 helper fixture는 **15/15 통과**했고,
root의 실제 profile/계보 읽기 전용 점검도 depth 2 연결을 통과했다. 이때 seed·보충 출력은
없었고 촬영·원형 부모 verifier 실행·원본/profile 변경은 모두 0이었다. 정상 종료·최종 P·
부모/worker 종료를 확인한 뒤에만 prepare/run할 수 있다. 기존 5K native QA 전수 재판정을
새로 추가하거나 부모와 child의 누적 승인을 중복 합산하지 않는다. 명령과 종료 조건은
같은 폴더의 `README.md`를 따른다. **이 준비를 실제 보충 완료로 해석하지 않는다.**

## Sunny 추가 보강 초안

Sunny 15K 완료 후 추가 보강은 별도 초안으로 남겼다. 잠정 예산은 750장으로,
11·13·15시 각 250장이다. 각 환경은 장애물 20~30m 180장과 큰 장애물 30~45m 70장을
검토한다. 추가 750장은 matched E24·공통 val·자산별 관측 근거를 확인한 뒤 결정한다.
현재 초안은 기존 Sunny 자료와 Foggy 수집을 바꾸지 않으며, 실제 보강 캡처를 시작하지 않았다.
149장을 재할당하는 현재 Foggy 프로파일을 750장 추가 수집 실행기로 사용하지 않는다.

`generator_review/sunny15k_supplement_draft_v1.json`에 분석 근거 7개의 경로·SHA를 보존했고
작성 시 모두 재검산했다. 수량은 출제 확률이나 성능 개선을 보장하는 확정 정책이 아니다.
얇은 객체·작은 동물 표본·전방 원거리 차량을 낮은 AP만 보고 일괄 증량하지 않는다.

<a id="foggy15-independent-borrowed-261019"></a>

## 빌린 PC Foggy15 독립 배치의 CPU 준비

공통 v160 생성기에 **독립 seed 26101915**와 기존 v163 장애물 보강 프로파일을 적용해
새 Foggy15 **5,000장 source를 CPU로 준비**했다. Foggy13의 장면을 복사하거나 기존
수집을 재생성한 것이 아니다. 새 seed에서 보강 규칙에 해당한 장면은 **152장**으로,
20~30m 낮은·부피형 119장과 30~45m 큰 가시형 33장이다. Foggy13의 149장과 수가 다른
것은 독립 배치에서 규칙에 맞는 후보 수가 달라졌기 때문이며 v163 정책은 변경하지 않았다.

| 고정 입력 | SHA256 |
|---|---|
| 기존 v163 프로파일 | `052dd5d1b79f89371d113fcb2fc928b978843c91abb4726f4cf3e1edb8ba50a9` |
| 공통 Foggy15 5K manifest | `7cf1eb24278b69c835ceb1b66fd2fa3d33bd6828cfb070167acfb741b3eba9cf` |
| 보강 후 최종 5K manifest | `8e2fef9ad217d8bc42e428f4da387f274628162dc9c1fdd46949b200ca17d134` |

기존 Foggy13와 합쳐 geometry를 추가 대조했을 때 10,000장 중 로터리 출구의
`DEFAULT_SUV` **근접 동등 배치 1쌍**이 같은 그룹으로 나왔다. 두 장면은 각각 독립
seed로 생성됐고 geometry SHA는 다르지만 actor의 route snapshot 위치는 같으며 Ego는
약 3.1cm·0.103° 차이다. 둘 다 유효 **train/train**이고 교차 split 쌍은 0이다.
이 1쌍을 명시한 train-only 중첩으로 수용하되, 원감사의 **`passed=false`는 수정하지
않았다.** `cross_foggy13_geometry_review.json`에 별도 판단을 남겼다. 기존 Sunny13 대비
필수 공통 감사의 통과 결과·비교 문턱은 변경하지 않았으며 완전한 지리·배치 독립성이나
원격 센서 가시성 검증을 주장하지 않는다.

실행은 새 공통 진입점 `scripts/portable_base5/reinforced_environment_workflow.py`와
`reinforced_borrowed.py`를 사용하도록 준비했다. 기존 Foggy13 orchestration 본문은
SHA `e255792210e6aa98c47ab11beb57704ff86d2195ff801f871e95c17de187f482`로 동결하고
환경·경로·intent identity만 명시적으로 연결한다. 독립 검토의 CPU 회귀 **139개**와 실제
CLI/import·논리 경로/구현 SHA 검사를 통과했으나, 이는 native 자격의 대체 증거가 아니다.

로컬 source package v2는 **60,037개 파일·3,913,544,411바이트(약 3.9GB)**였다.
이후 CPU 검사 코드 등 빠진 참조의 closure를 보완한 **v3는 60,250개·3,915,249,775바이트**,
입력 identity SHA는 `0994e8ac48cee111b87e544f193c2753e633d71131fb4b22fb872fb025a90685`다.
원본 runtime closure 439개·실제 scripts overlay 371개·helper 16개를 보존했고,
v3에는 CPU test 212개와 조상 source·공통 생성기 runtime 참조까지 포함했다.
이 수치는 압축본 크기나 원격 배포 완료 수치가 아니라 **로컬 배포 staging의 입력 목록**이다.

현재 근거는 `artifacts/evaluation/base5_foggy15_design_261019/`의 `cpu_complete.json`,
`source_package_complete_v2.json`·`source_package_complete_v3.json`,
`cross_foggy13_geometry_audit.json`·`cross_foggy13_geometry_review.json`,
`independent_reinforced_workflow_review_v1.json`이다. 원격 후보 경로는
`~/ajounice_base5_capture/work/fresh-foggy15-v1/`이며 기존 Sunny11/Sunny13 source·export·
다른 work는 RO로 보존하도록 계획했다. 최종 배포의 실제 mount·SHA 검증은 별도 확인 대상이다.

**이 절 작성 시점에는 CPU/source 준비만 완료됐고 Foggy15 native 촬영은 0장이다.**
원격 배포·실제 Foggy15/annotation 설정·32장 및 독립 이동 검증·원래 6개 release gate·
공통 5장 pause/resume 자격·5K 본수집은 아직 완료로 판정하지 않는다. 기존 Foggy13 native
증거를 Foggy15 인증으로 재사용하지 않으며, 자격용 표본은 본수집 5K에 합산하지 않는다.
이후 실제 배포·소량 자격·날씨·본수집 결과는 아래 후속 절에 관측 시각과 근거를 추가한다.

### 원격 배포와 실제 Foggy15 설정 확인

Sunny11은 [로컬 수령 검증까지 완료](261018_base5_unified_generation_capture.md#sunny11-local-retrieved-261019)한 뒤
환경을 전환했다. 기존 PNG는 삭제하지 않았다. 새 원격 root의 CPU 배포는 전체 60,250개
입력 SHA를 통과했다. 이미 배포된 과거 정적 source 중 같은 경로·SHA인 33,979개를
**새 root에 일반 복사**하고, 새 입력만 136,566,347바이트의 단일 delta로 전송했다.
기존 파일은 각 복사 전후 SHA·identity, 새 파일은 SHA·별도 inode를 확인했으며 기존 파일에
하드링크하거나 내용을 바꾸지 않았다. 기존 Sunny11 완료 제어 파일 9개와 전달 완료 근거
4개의 SHA·크기·mtime도 배포 전후 같았다. 새 collection 촬영이나 기존 사진 전수 재검사는
이 배포 검사의 내용이 아니다.

공식 `WeatherTimeChanger_Linux`를 새
`~/ajounice_base5_capture/work/weather-time-changer-261019-v1/`에 복사했고, 원본 SHA
`8c47f48a8d54b79b657920a661f582bd587c9f9065b6b55c9e911d42a0369609`가 같음을 확인했다.
이 도구와 관측 helper의 HOME·XDG·임시 경로도 같은 허용 폴더 안에 두었다.
MORAI의 `127.0.0.1:7789`에 연결한 뒤 15:00과 Foggy를 적용했다. 두 자동 순환은 OFF다.

**버튼 직후 `Current`와 로그는 이전 11:00/Sunny를 표시했다.** 이 표시만으로 적용 실패나
성공을 단정하지 않았다. MORAI 단일 창에서 실제 안개를 확인하고 공식 도구를
Disconnect → Connect한 뒤, 새 응답에서 **`Current: 15:00` / `Current: Foggy`**를 확인했다.
도구의 버튼 직후 응답이 적용 전 값일 가능성을 보여 주는 관측이며, 내부 통신 원인을
확정한 것은 아니다. 이후 수집에서는 클릭 사실보다 재연결 후 readback을 환경 근거로 사용한다.
Foggy 버튼의 기본 preset을 그대로 사용했고 숫자 slider를 임의로 변경하지 않았다.

환경 근거는 로컬 `base5_sunny11_training_export_261019_v1/`의
`weather_connected_2313.png`·`weather_foggy15_2313.png`·`simulator_initial_2317.png`·
`weather_reconnected_2318.png`다. 이 폴더는 `artifacts/evaluation/` 아래에 있다.
이 환경 확인 시점에는 원격 native 취득을 시작하지 않았으며, 확인 이후 단독 소량 검증
제어권을 인계했다. 로컬 Foggy13 수집·날씨·제어에는 개입하지 않았다.

<a id="front-far-sensor-followup-261019"></a>

### 전방 초장거리와 후속 센서 배치 검토의 경계

전방 80~100m의 LiDAR 지원 부족은 위 진단의 **전방 305개 중 125개 0점**, 후방
317개 중 3개 0점 관측을 따른다. 전방 0점 125개 중 100개는 native full-RPY 박스와
교차 가능한 수직 빔 자체가 없었다. 약 0.17°의 작은 차체/도로 상대 기울기도 90m에서
약 27cm 높이 차이가 되어 16채널 빔 사이의 공백을 만들 수 있다. 이는 관측과 잘 맞는
기하 설명이지 장착각 오류나 가림·mesh·수평 샘플링까지 확정한 인과 분석은 아니다.

Foggy13에는 공통 v160이 계승한 원거리 보조·거리 분산·fallback이 들어 있다. 그러나
**이번 v163의 신규 149장 변경은 장애물 거리 재할당뿐**이며 전방 100m 빔 공백을 겨냥한
차량 추가 정책은 아니다. 차량·보행자 분포를 유지했다는 사실과 기존 원거리 보강을
계승했다는 사실을 구분한다. 작은 val의 방향별 AP를 프레임별 검출 확률로 해석하지 않는다.

사용자는 현재 센서 배치로 계획한 30K를 마치고 기준 모델을 확보한 뒤 새 센서 배치와
추가 수집을 검토하자는 방향을 제안했다. 현재 수집의 센서 위치·각도·extrinsics·분포는
바꾸지 않는다. 후속 변경은 먼저 대회에서의 허용 범위를 확인하고 같은 장면의 소량 A/B로
전방 80~100m, 후방, 근거리의 손익을 함께 검증한다. 효과가 확인된 경우 새 calibration과
별도 수집·학습·배포 버전으로 비교한다. 새로운 30K의 즉시 수집이나 기존 모델의 새 배치
동등 성능을 확정한 것은 아니다.

운영 안전망은 **기존 배치·기존 calibration·현재 30K 기반 모델**의 묶음으로 유지한다.
새 배치의 목표는 전방 원거리 개선이며, 후방·근거리·낮은 객체·도로 경사 조건을 포함한
회귀 확인 없이 마지막 모델을 자동 교체하지 않는다. 새 배치의 소량 비교에서 이득을
확인한 뒤 필요한 추가 수집량을 정한다. 추가 30K는 가능한 규모이지 먼저 채워야 하는
조건이 아니며, 기존 모델과 센서 설정을 함께 복귀할 수 있도록 버전을 분리한다.
새 센서는 점군·native GT·카메라 투영·ROS 출력 변환까지 같은 좌표 계약인지 확인한다.
특히 pitch를 바꾸는 경우 기존 upright box·수직축 가정을 그대로 적용할 수 있는지도
점검한다. 설정 파일의 센서 위치/각도만 바꾸면 모든 변환이 자동으로 맞는다고 가정하지 않는다.

### Foggy15 준비 관측 대시보드

10/19 23시대에 빌린 PC **127.0.0.1:8083**, 이 PC SSH 터널 **localhost:8084**의
실제 HTTP 응답을 확인하고 로컬 브라우저에 열었다. 원형 UI/HTML의 동결 SHA를 확인했고
관측 이력은 새 `work/fresh-foggy15-v1/observer-state-port8083/`에만 저장한다. 수집 출력이나
제어 상태 파일을 만들거나 수정하지 않았다. 이전 Sunny11 완료 대시보드와 별개다.

이때 표시되는 것은 위 SHA `8e2fef9a…ca17d134`의 **Foggy15 CPU full 계획**이며
`not_started`, **0/5,000**이 실제 상태다. 최종 native adoption manifest와 profile은 아직
준비 전이므로 소량 자격이나 본수집 시작의 증거가 아니다. 최종 profile 준비 후에는 실제
adoption 입력을 확인하는 정식 fresh observer로 이 임시 관측 서버만 교체한다.

<a id="foggy15-native-qualification-261020"></a>

### 10/20 새벽 소량 native와 CPU 환경의 실제 재현

원격 배포 이후 보강 대상 진단 32장을 실제 촬영했다. 첫 정규 묶음 16장은 최종 P·기어 1·
실행 오류 없음을 확인했다. 최종 파일은 RGB뿐 아니라 카메라 Instance 4개와 LiDAR
Instance·8Points를 포함한다. 최초 실패 판정을 승인으로 바꾸지 않았으며, 기존 동일 자산·
순서·거리·방향·split 계약 안에서 새 배치를 촬영한 결과 최초 진단 실행은
**승인 31장·부족 1장·실행 오류 0건**으로 끝났다. 승인 31장에는 실제 대체 5장이 포함된다.
이 수치는 본수집 5K의 진척이 아니다.

원격 CPU의 지도 감사에서는 5K 중 `01363`의 NCAP_EBT 지지면 후보가 한 번 달라졌다.
원본 배열 4개의 SHA는 같았지만 NumPy/SciPy 버전에 따라 KD-tree의 가까운 32개 후보
집합에 선호 link가 포함되는지가 달랐다. 원형 k=32·허용오차·지도 판정은 바꾸지 않고,
생성 당시 Python 3.8.10·NumPy 1.17.4·SciPy 1.3.3 등을 새 작업 폴더의 CPU 전용
읽기 전용 환경에 분리했다. 실제 5K 감사는 **32.52초·오류 0·OFFLINE_PASS**였고,
저장된 생성 당시 감사와 정확히 일치했다. 원격 시스템 설치·ROS/실제 촬영 환경은 변경하지
않았다. 동결 runtime 439개와 test source 212개의 SHA도 유지됐다.

이 재현과 별개로, 원격의 최초 전체 CPU suite 실패 로그는 보존한다. 기존 동일 source의
**2,376개 통과·1개 명시적 deselect** 근거를 검증해 재사용한 것이지, 원격에서 그 전체
suite가 새로 통과했다고 주장하지 않는다. CPU 전용 파일 SHA 재사용도 검증 범위 안에서만
실행하며 실제 native QA·GT·새 manifest·세션 결과를 캐시하지 않는다.

마지막 부족 slot은 최종 selected ledger의 `source_cell_id`를 등록된 32개 원본 slot과
대조해 **`03160`**으로 확정했다. 중간 복구 ID나 현재 표시 장면으로 추정하면 `02908`을
잘못 지목할 수 있었지만, 실제 `02908`은 이미 승인된 대체에 포함돼 있었다. raw ID 차집합을
그대로 부족 장면 수로 세지 않는다. 관련 원문은 `actual_adoption_pairs_v1.json`에 보존한다.

`03160`은 전방 30~45m OrangePEFirewall 앞에 보조 Niro가 놓여 완전 가림이 반복됐다.
원형 후보 한도 512회·layout 한도 8개를 늘리지 않고, CPU 제안 단계에서만 명백한 3D
완전 가림을 반려했다. 358회 build 중 17개를 이 기준으로 반려한 뒤, 장애물 약 34.6m·
Niro 약 38.5m의 후보를 얻었다. 자산·개수·순서·거리 목적·split·native QA는 유지했다.
OBB ray 검사는 후보 선택 보조이지 mesh 가시성의 증명은 아니며, 기존 높이 초기화가
불확실하다는 원문 근거도 함께 보존한다.

첫 실제 재촬영은 **1/1 승인·검토 0·실행 오류 0·최종 P 확인**이었다. 실제 장애물 거리는
약 34.98m였고 LiDAR 0점은 그대로지만 전방 카메라 Instance 436픽셀에서 대응이 생겼다.
따라서 앞 차량의 완전 가림을 피한 성공이지 LiDAR 원거리 빔 공백의 해결이 아니다.

다만 이 한 장을 정식 source에 연결하는 첫 CPU adoption은
**`Replacement root/spec provenance differs`로 중단**했다. 기존 대체 5장의 연결은 맞았지만,
새 `03160` helper가 등록 pilot root 대신 full root에서 출발해 root entry의 두 경로와
metadata SHA가 달랐다. 실제 촬영 승인과 정식 연결의 성공을 구분한다. 과거 PASS·native
metadata·manifest는 수정하지 않는다. 올바른 등록 root의 새 계보를 촬영 전에 검증하고
별도 출력에서 최소 재촬영하는 방식을 택했다. **이 기록 시점에는 최종 adoption·정규 32장·
음영 이동·여섯 release gate·공통 pause/resume·5K 본수집은 아직 완료가 아니다.**

근거는 로컬 `artifacts/evaluation/base5_foggy15_design_261019/`의
`cpu_gate_prepare_v1/producer_cpu38_summary_v1.json`, `actual_adoption_pairs_v1.json`,
`actual_four_reviews_and_cpu_candidate_occlusion_v1.json`과 원격 새 root의
`qualification/pilot_v1/`, `qualification/pilot_supplement_03160_v3/`,
`qualification/full-adopt-actual-six-pairs-v1.log`다. 이후 성공은 새 절에 추가하며 이 중단
기록이나 최초 31/32 결과를 소급 수정하지 않는다.

### 올바른 등록 root의 1장 재검증

이후 새 helper는 pilot 5K entry와 원등록 subset entry의 동일성부터 검사했다. 원형 제안
루프에서 기존 승인 geometry SHA `3ebf0036…`를 9.70초에 정확히 재현했고, root entry·
root metadata SHA·ordinal·부모 불변·분할 최종화 대기 필드를 촬영 전에 모두 대조했다.
새 runner는 P 응답 후 원형 base capture의 **`--limit 1`만 허용**했다. 방향 복구 재시도
여러 번을 숨긴 1회 명령이 아니다. 이 base capture는 원형 `verify_accepted_native`가
지원하는 경로이며 본수집의 native 정책이나 허용오차를 바꾼 것이 아니다.

`qualification/pilot_supplement_03160_v4/`의 실제 촬영은 **13.89초·승인 1·검토 0·
실행 오류 0·충돌/최종 P 검사 통과**였다. 새 manifest SHA는
`37df97d4deb4f223c921dedb8c61a4cb1457b1ecd92e61721096138e4ee23a65`다.
기존 대체 5장과 이 올바른 새 1장은 `actual_adoption_pairs_v2.json`에 연결했다.
과거 승인 사진·잘못된 계보의 성공 기록·실패 4장·첫 adoption 중단은 모두 보존한다.
이 한 장의 성공만으로 정식 adoption이나 후속 여섯 gate·본수집 시작을 주장하지 않는다.

6개 실제 후보는 원형 `profile.verify_transition`의 거리·방향·spec·roster·split과 보조
장애물의 기존 `<20m` / `20~30m` 보호를 개별 선검사해 **6/6 통과**했다. 전역 near budget과
전체 source의 자격은 이 개별 검사로 대체하지 않는다. 원문은
`actual_six_profile_transitions_v1.json`이다.

후행 CPU 연결에서 동일 과거 참조 blob의 SHA를 수십 번 다시 읽는 비용을 줄이기 위해
기존 411개 고정 입력에 **원형 v4 source의 정확한 PNG 768개·bag 2개**만 별도 예외로
더했다. 추가분은 1,109,071,378바이트(약 1.03GiB), 경로→SHA 목록 identity는
`a19af31f13d1566fad2c0f840b080c8f48fed16f3c516be4b3c85321c0b28e8a`다.
임의 PNG/bag 전체가 아니라 이 정확한 읽기 전용 source 770개만 허용한다. 기존 VIC·
StaticScope 본문은 유지하며 entry/exit 실제 SHA·각 hit의 파일/상위 경로 확인·subprocess
금지·실제 촬영 전 scope 종료를 유지한다. 현재 native·audit·ledger·등록/대체 control은
이 cache와 분리해 실제 SHA를 검사한다. JSON 객체·문맥/QA 결과를 재사용하는 방식이 아니다.

첫 추가 controller는 원형 cache 모듈의 import 경로 누락으로 0.394초에 종료됐다.
adoption 출력이나 촬영은 생성하지 않았고 실패 로그도 보존했다. 새 버전에서 정확히
핀된 `CACHE.parent` 경로만 보완했다. 실제 producer namespace의 import-only probe가
Python 3.8.10·원형 모듈 경로/SHA·동일 VIC 클래스·의존 모듈과 runtime 439/test 212
전후 불변을 확인했다. 그 뒤 새 authority로 원형 adoption을 재개했다. 검토·probe의
성공과 실제 adoption 완료는 구분하며, 이때 본수집은 여전히 시작하지 않았다.

### 생성 감사와 native 검증의 수치 환경 분리

그다음 실제 adoption은 523.04초에 `Saved attitude decision differs from recomputation`으로
종료됐다. 새 5K source의 지도 감사는 고유 5,000장·오류 0·`OFFLINE_PASS`였지만,
실제 native 상세 판정의 재계산이 저장 dict와 정확히 일치하지 않았다. 이 부분 출력은
`native-adoption-v1/`에 보존하며 최종 binding이나 정식 adoption 성공으로 쓰지 않는다.

원형 32장 전체(승인 26장·검토 6장)를 읽기 전용으로 비교해 원인을 구분했다.

| 재계산 환경 | 저장된 상세 판정과 완전 일치 | 차이 |
| --- | ---: | --- |
| 실제 native 촬영 환경 Python 3.10·NumPy 1.23.5 | 32/32 | 없음 |
| 생성 감사 재현용 Python 3.8·NumPy 1.17.4 | 5/32 | 27장의 수치 필드 57개, 최대 약 `5.684e-14m` |

3.8 비교의 비수치 필드 차이는 0개였고, 32장 모두 eligibility·classification·review
reason이 같았다. raw/source SHA도 전후 동일했다. 원형 검사는 dict의 실수 값까지
`==`로 비교하므로 이 극미세한 차이도 거부한다. 이를 근거로 native GT나 QA를 고치거나
허용오차를 늘리지 않는다. 또한 이 상세 판정의 반올림 수준 차이와 앞서 확인한
SciPy KD-tree 후보 집합 차이는 별개다.

따라서 native 상세 검증은 원래 촬영 환경 3.10에 두고, 생성 source의 지도 감사 함수만
원래 생성 환경 3.8의 별도 읽기 전용 실행에서 매번 실제 재계산하는 연결을 검증한다.
보존된 partial 5K로 실행한 3.8 감사는 실제 함수 1회 호출·결과 cache 없이
`OFFLINE_PASS`였고 저장 감사 dict와 완전히 같았다. geometry 10,001개와 runtime 439개의
SHA도 전후 같았다. 원문은 `native_attitude_replay_host310_all32_v2.json`,
`actual_attitude_cohort32_producer38_v2.json` 및 CPU gate prepare 근거에 보존한다.
이 비교·분리 probe는 실제 정식 adoption·후속 native 시험·본수집의 대체 인증이 아니다.

이 문제는 센서 배치 변경이나 모델 재학습이 필요한 증거가 아니라, 서로 다른 CPU 수치
환경을 정확한 dict 동일성 검사에 연결한 이식성 문제다. 이번 동결 수집에서는 원래 환경을
재현해 처리한다. 후속 공통 실행기 정리 시 생성·촬영·검증 환경의 버전 계약을 명시하고,
수치 중간값과 실제 승인/검토 결정을 구분하는 검증 설계를 별도 release에서 검토한다.
현재 수집 도중 이 개선을 이유로 원형 검사나 허용오차를 변경하지 않는다.

이후 실제 분리 실행 probe는 **38.48초·exit 0**으로 통과했다. 1,181개 고정 입력의
SHA scope 종료 → 원형 3.8의 실제 5K 감사 → 새 scope 진입 → 원래 3.10의 native 상세
판정 exact 검증 → 실행 환경/함수 복원까지 실제로 확인했다. 메타데이터 로더는 실제
배포된 동일 SHA helper와 원래 작업자 HOME을 사용하고, source 수치 계산은 기존
canonical 환경에 둔다. 중간 연결부 probe 실패들은 별도로 보존했다. 그 뒤 올바른
실제 대체 6개와 원래 ledger를 새 `native-adoption-v2/`에 연결하는 adoption을 시작했다.
이 probe의 성공이나 adoption 시작을 adoption 완료·추가 native 촬영·5K 본수집으로
집계하지 않는다. 원문 로그는 `qualification/full-adopt-hybrid-correct-v4.log`다.

후반 검산의 성능 후보도 별도로 확인했다. 원형 `effective_diagnostic`는 승인된 진단 행마다
`verify_regular(session, chunk_manifest)`를 호출하며, 이 함수는 같은 묶음의 승인·검토
전체를 다시 확인한다. 따라서 같은 16장 session이 반복 검증될 수 있다. 이는 실제 코드의
호출 구조 관측이지 개별 비용을 측정한 병목 확정은 아니다. 이번 동결 실행은 유지하고,
후속 release에서 입력/종료 경계가 동일한 session 검증을 한 번으로 묶을 수 있는지 검토한다.
현재 실행에 QA 결과 cache나 검사 생략을 추가하지 않는다.

### 올바른 6개 대체의 정식 source 연결 완료

동일 실행은 **828.79초·exit 0**으로 끝났다. 새 binding을 쓴 뒤 원형
`verify_binding()`의 최종 자기 검증까지 통과했으며, 앞서 중단된 v1 출력과 원본 native·
검토 근거는 그대로 보존했다. 원문 3개를 회수하고 로컬/원격 SHA를 대조했다.

| 실제 완료 산출물 | SHA-256 |
| --- | --- |
| `qualification/full-adopt-hybrid-correct-v4.json` | `ab9725fa62d175da59fb50ff6f26a2c0fc585596a2a7e9bc732cd7897eb5a3de` |
| `native-adoption-v2/binding.json` | `a43d0fd0a86ecd478df1e2c5e1a7ae18d64a922f29656c208d465474e5cfb769` |
| `native-adoption-v2/assembled_source/final_source/manifest.json` | `f3db513401a92bcfebad99f514ffa7dc1ecb00f23db0e1766eedebb4504ffa91` |

경로의 원격 root는 위 `work/fresh-foggy15-v1/`다. 이는 실제 대체 6개를 반영한
**5K source 조립과 소량 근거 연결 완료**이며, 본수집 승인 5,000장이 아니다.
이 완료 시점에는 정규 canonical 32장·독립 shadow 2장·여섯 release gate·공통
pause/resume 자격과 본수집은 아직 완료되지 않았다. 다음 단계는 이 실제 binding에서
원형 route witness와 등록 권위를 만들며, CPU-only 독립 등록은 native 단독 제어와
서로 분리된 출력에서 병렬 준비한다.

이후 원형 producer route export는 **0.898초·exit 0**, 실제 route witness SHA는
`66d3214269799487e4627ec571332823ad7b8bf51a881038b185c52688465b59`였다.
이를 연결한 정규 canonical 32장 등록은 **627.61초·exit 0**으로 통과했다.
병렬 CPU 근거 대조도 **264.03초·exit 0**이었으며, 동일 source의 기존 2,376 PASS
receipt와 runtime 439/test source 212개의 일치 확인이지 원격 신규 전체 테스트 결과가
아니다. 최신 Ego/Object/Collision freshness·P·정지를 확인한 뒤 정규 first16 실행에
진입했다. 이 native 진입도 먼저 원형 binding을 검산하므로 명령 시작과 첫 이미지
생성 시점은 다르다. 여기까지도 본수집은 시작 전이며 정규 32장 완료를 주장하지 않는다.

첫 first16 실행은 **568.31초·exit 1·`Adoption evidence inventory changed`**로
촬영/intent 생성 전에 종료됐다. 원형 adoption summary 비교는 통과했지만
`evidence_hashes` 집합이 달랐으므로 기준을 완화하거나 승인으로 바꾸지 않았다.
실패 원문을 보존하고 생성 모듈의 HOME 기반 기본 asset 참조 경로를 읽기 전용으로
비교한다. 후속 연결은 실제 source 계산용 기본 참조와 원래 native worker의 HOME·
scenario-dir·sensor-dir·ROS 로그/출력 경로를 분리해 맞추는 범위에서 검토한다.
이 중단을 native GT 실패나 추가 승인 장수로 해석하지 않는다.

실제 native namespace의 기본 참조만 대조한 시험은 **1.473초·exit 0**이었다.
원래 live HOME인 `work/ros-home-v2`에서 asset JSON 3개가 실제
`/home/ddeungbuntu/MoraiLauncher_Lin/...`로 resolve됐고, binding에는 대응하는
`/home/luv/MoraiLauncher_Lin/...` 키가 있었다. **3개 모두 actual SHA·선언 SHA·canonical
SHA가 동일**했다. 같은 바이트의 다른 경로 키 3건을 실제 확인한 것이며, 전체 34,172개
pin map을 다시 만든 시험은 아니다. 전체 일치는 후속 원형 가드에서 확인한다.
helper 논리 경로/SHA와 runtime 439개도 같았다. 원문은
`cpu_gate_prepare_v1/actual_native_default_reference_paths_v2.json`이다.
source-only 생성 모듈 초기화 뒤 복원할 대상은 메타데이터 로더의 사용자 HOME가 아니라
기존 live HOME `work/ros-home-v2`다. native argv·환경·ROS 로그·캡처 기본 경로는 원형을 유지한다.

source-only preimport의 실제 회귀는 **1.739초·exit 0**이었다. 원형 생성 모듈을 canonical
HOME에서 초기화한 뒤 원래 live HOME·전체 환경을 복원했고, 기본 JSON 3개의 키가 binding과
일치했다. 원형 collector parser의 실제 default 식으로 scenario-dir·sensor-dir도 전후
같음을 확인했다. runtime 439개·Popen·원형 코드/가드는 변경하지 않았다. 이 시험은 원격
쓰기·촬영 없이 네트워크가 격리된 환경에서 수행했다. 전체 binding guard 통과나 실제
first16 완료의 대체 근거는 아니며, 새 별도 entry로 원형 검증에 재진입한다.
후속 공통 release에서는 source asset 위치를 명시적인 입력 경로로 다루고 live HOME에서
암묵적으로 유도하지 않도록 정리할 후보로 남긴다. 현재 동결 source는 변경하지 않는다.

### 본수집 전 발견한 immutable source 폴더명 충돌

공통 `SessionInputBoundary`는 부모 경로에 `native`가 있는 JSON을 원본/가변 근거로
분류한다. 새 원격 조립 root를 `native-adoption-v2`라고 지은 탓에 정당한 고정 입력인
manifest와 metadata 5,000개까지 이 분류에 걸리는 것을 실행 전에 발견했다.
그대로 진행하면 `warm_boundary→remember_input_pins`가 고정 manifest를 인정하지 못한다.
이는 품질 검사 강화나 센서 문제가 아니라 우리 배포 경로와 기존 분류 규칙의 충돌이다.

원격 전용 guard 예외를 추가하지 않고, **`adoption-v3`의 새 경로에 원형 함수로 조립**하는
방향을 선택했다. 실제 manifest entry 경로와 원형 predicate를 읽기 전용으로 대조했다.
이름을 바꾸면 manifest·metadata가 고정 후보가 되고 기존 `pilot_v1/native/...`의 실제
proposal JSON 5개만 원래 nonfixed passthrough로 남는다. raw/native 전체를 고정 cache로
승격하지 않는다. v2 binding/manifest·등록·실패·CPU 완료 이력은 그대로 보존한다.

재실행 범위는 원형 adoption(기존 실측 약 14분) → 새 route witness → 새 정규 등록
(약 10분)이다. **기존 실제 대체 6개의 native를 재사용하며 재촬영하지 않는다.**
아직 canonical 촬영은 0장이므로 이후 정규 32장·shadow 2장은 새 입력으로 처음 촬영한다.
runtime 439/test source 212개의 기존 CPU 회귀 원문도 변하지 않으며 전체 테스트를 새로
반복할 이유는 없다. 최종 원형 gate는 새 source와 실제 새 native 근거에 연결한다.
새 조립·등록 완료와 본수집 시작은 이후 실제 결과로 별도 기록한다.

새 경로의 촬영 없는 연결 점검은 **2.899초·두 실행 환경 모두 exit 0**이었다.
실제 host 3.10의 phase import/argv와 producer 3.8의 원형 감사 import/context만
확인했으며, 대형 감사·native 촬영·추가 품질 gate를 실행한 시험은 아니다.
원형 boundary predicate가 `adoption-v3`의 manifest를 고정 입력으로 분류하는 것도
확인했다. runtime 439개의 SHA는 유지됐다. 새 권위 SHA는
`460323d93cf71bf3bd520149b639d9f2cf21dcfce38687b8ceab47e08457dcf3`이다.
그 뒤 원형 함수의 실제 재조립을 시작했으며 로그는
`qualification/full-adopt-hybrid-immutable-v3.log`에 보존한다. 디렉터리 생성이나
실행 프로세스 관측만으로 binding 완료·정규 32장·본수집 시작을 주장하지 않는다.

이후 실제 v3 재조립이 **821.97초·exit 0**으로 완료됐다. 종료 원문·binding·manifest
세 파일을 로컬로 회수하고 root와 담당 에이전트가 각각 SHA를 대조했다.

| 실제 v3 완료 산출물 | SHA-256 |
| --- | --- |
| `qualification/full-adopt-hybrid-immutable-v3.json` | `20df02a82da58265603916b8315e5a4ff0dcf092fc603158f5aec9ad8eb2e1e3` |
| `adoption-v3/binding.json` | `e8863c936fb9c9ddb565e968c111bf540fd8b4849301e936271458c133a4e390` |
| `adoption-v3/assembled_source/final_source/manifest.json` | `64be28a3ef975569ea0a8c9b206902901148e5b614efd63e4b91105bc707e272` |

회수한 v2/v3 manifest에서 ID별 `(scenario_sha256, split)`을 비교한 결과
**5,000개 모두 동일·변경 0개**였다. 경로를 바로잡은 재조립이며 geometry 재생성이나
재촬영이 아니다. 로컬 원문은 위 CPU gate 준비 폴더의
`immutable_followup_actual_v1/{manifest,binding,execution}.json`에 있다.
기존 v2와 첫 실행 실패는 보존했다. 이 완료 시점에도 정규 canonical native·본수집은
0장이며, 새 route·`qualification/full_v2` 등록과 원래 후속 gate를 이어간다.
