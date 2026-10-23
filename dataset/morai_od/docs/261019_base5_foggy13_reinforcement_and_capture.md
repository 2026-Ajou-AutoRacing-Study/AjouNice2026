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

### v3 source의 route·정규 등록과 실제 촬영 진입

새 source의 원형 route witness export는 **0.885초·exit 0**으로 완료됐다. witness는
`adoption-v3/route-witness-v1.json`이며 SHA는
`0a6d46974cf17bbbdd3ae921bc738db587fedc92d8f3ec81516a4346810eb031`이다.
이어 `qualification/full_v2/registered/`의 정규 32장 등록을 실제 실행해
**628.28초·exit 0**으로 마쳤다. 종료 원문
`qualification/full-register-hybrid-immutable-v2.json`의 SHA는
`864ef6735f68d126e8abf0c4201d7deb90000e4b70a448c0f4f4f4c82fbffd35`다.
이전 `full_v1` 등록이나 실패 산출물은 변경하지 않았다.

첫 정규 촬영 전 새 Ego/Object/Collision 수신·정지·P를 확인했고, 동일 SHA 생성 모듈의
source-only canonical HOME 초기화 후 기존 native HOME `work/ros-home-v2`와 전체 환경을
복원하는 별도 entry를 사용했다. 원형 guard·worker argv·QA는 유지했다. 긴 원형 binding
재검산 뒤 **2026-10-20 04:05 KST에 실제 `exact32` 상태와 첫 native 묶음이 생성**됐다.
명령 시작을 촬영 완료로 계산하지 않으며, 소량 시험의 원본·일반 검토·복구는 모두 보존한다.
독립 shadow 2장 CPU 등록도 **265.50초·exit 0**으로 끝났지만 이 등록은 실제 shadow 촬영
성공이나 release 자격이 아니다. 본수집 시작은 후속 실제 완료 근거로 별도 기록한다.

정규 first16은 **728.37초·exit 0**, 실제 상태 **정상 pause·16/16 승인·검토 0·실패 0**으로
마쳤다. binding 검산 이후 실제 native 구간은 161.71초였다. shared 묶음의 후방 차량
1장이 처음에는 센서 근거 부족으로 검토됐지만 원형 direction-reference 복구의 실제
재확인을 통과했다. 최초 검토 raw와 복구 근거는 모두 보존하며 과거 판정을 고치지 않았다.
shared 묶음 종료와 별도 shadow 시작 전 모두 실제 P gear 1·오류 없음을 확인했다.
이 16장은 자격시험 자료이며 본수집 5K에 합산하지 않는다.

직후 선택 ledger와 등록 SHA를 대조해 위 집계의 한계를 확인했다. 후방 차량 `01228`의
원형 복구 승인은 **동일 geometry 재확인이 아니라 `01228_replace`의 새 geometry**였다.
부모가 16/16 승인으로 집계했더라도 원형 최종 exact gate는 `id == source_cell_id`와
등록 scenario SHA의 일치를 요구하므로 이 상태를 exact16 통과로 해석하면 안 된다.
최초 검토·실제 대체 승인·부모 집계를 그대로 보존하고, 등록 32개 전체의 원래 관측 이력을
확보한 뒤 원래 지원되는 depth 1→2 후속 adoption을 검토한다. 기존 6개 대체는 parent
binding으로 계승하며, 새 대체를 옛 geometry의 exact 승인처럼 고쳐 쓰지 않는다.

별도 shadow 관측은 **737.18초·exit 0**, 실제 native 상태 **2/2 승인·검토 0·실패 0**으로
완료했다. 첫 동물 장면의 일반 검토와 원형 복구는 보존했다. 실제 native 구간은 190.87초였고
종료 후 최신 수신·정지·P를 다시 확인했다. `full_v2/shadow-cases.json`을 만드는 CPU phase도
**254.53초·exit 0**으로 끝났다. 고정 SHA scope는 실제 진입·종료 해시 검사를 마쳤으며
native/QA/JSON/context 결과 cache는 사용하지 않았다. 이 phase는 실제 case 참조를
만드는 단계이고 원형 `verify_shadow()`의 최종 gate 통과는 후속 release 검증에서 구분한다.
나머지 정규 16개의 원래 관측은 새 관측 결과와 검토·실제 대체를 보존하는 진단으로 이어간다.

이 재개는 **647.86초·exit 0**으로 끝났고, 최종 부모 상태는 complete·32/32 승인·검토 0·
실패 0이었다. 재개 중 기존 ledger를 재구성할 때 보인 3개 등의 중간 수치를 이전 16개에
더하지 않았다. 알려진 `01228_replace` 때문에 이 부모 집계를 unchanged exact32 PASS로
표현하지 않는다. 원래 등록 32개 관측의 실제 ledger·검토·대체·최종 P를 보존한 뒤, 원형이
지원하는 마지막 bounded depth 1→2 adoption에 필요한 입력으로 사용한다.

등록 32개 원문을 읽기 전용으로 대조한 결과 **원래 승인 31 / 원래 검토 1**, 부모의
canonical slot은 고유 32개였고 실제 새 대체는 `01228` 한 개뿐이었다. 원형
`replacement_shape()`의 다섯 계보 필드와 동일 spec·ordered roster·split을 확인했으며,
세 실제 session의 종료 P와 원형 parent row 변환 뒤 나머지 값 차이 0도 확인했다.
해당 slot은 reinforcement allocation target이 아니므로 target transition 검사는 해당하지
않는다. 전체 nearbudget와 native QA는 후속 원형 adoption에서 실제로 다시 검사한다.

| 실제 새 대체 입력 | SHA-256 |
| --- | --- |
| session 계약이 가리키는 `replacement_deck_0/manifest.json` | `d49d77e890c9418b21e23669944bd8b3a5ea8914a8388b999c5ac25810009db3` |
| 실제 승인 session의 `ledger.jsonl` | `e7e193b4c20e5a00bd9be6976ce6a978ef0311657a33e32c6bb40f3a462791e2` |

원래 scenario SHA `ec26c8e4…`에서 실제 승인 대체 SHA `6819940c…`로 바뀌었다.
자동 discovery의 제한된 탐색은 이 nested manifest를 찾지 못하므로 코어를 수정하지 않고
원형 CLI의 원래 ledger 두 개와 실제 `manifest:ledger` 명시쌍을 사용한다. 보조 수령기의
import/pin map/parent slot 변환/manifest 탐색 연결 실패들은 별도 로그로 보존했다.
이 대조는 원문 확보·계보 사전검사이며 unchanged exact32 gate나 본수집 승인이 아니다.
로컬 근거는 `cpu_gate_prepare_v1/depth2_diagnostic32_controls_actual_v6/`에 있다.

명시쌍을 봉인한 새 CPU 권위의 실제 분리 probe는 **36.76초·exit 0**이었다. 원형 producer
3.8의 실제 v3 5K 감사 dict와 저장 감사의 정확한 일치, host 3.10의 native 상세 판정,
1,181개 고정 입력의 scope 종료·실제 재진입 SHA, 환경·함수 복원을 확인했다. QA 결과나
새 native 입력을 cache하지 않았다. 권위 SHA는
`94e94d28477bbc39842e6544f1bd89a24159e79d4dff242ea96984a226768ec6`이며 probe 원문은
`qualification/hybrid-split-depth2-probe-v1.{json,log}`다.
이어 **04:50 KST에 `adoption-v4`의 원형 depth2 adoption을 실제 시작**했다. 기존 6개는
parent binding으로 계승하고 새 실제 대체 한 개만 명시한다. 실행 로그는
`qualification/full-adopt-hybrid-depth2-immutable-v4.log`다. 이 시작을 source 연결 완료나
새 exact32·본수집 완료로 집계하지 않는다.

<a id="foggy13-main-completion-261020"></a>

## 로컬 Foggy13 본수집 종료와 4,998장 보충 상태

로컬 본수집의 실제 terminal 시점은 **2026-10-20 03:51:25.393 KST**였다.
5,000개 검사가 끝났고 원래 승인 4,936 / 부족 64 / 미검사 0 / 실패 0으로 정상 종료했다.
main exit 0·작업자 종료·idle lock·마지막 묶음의 8개 between-scene P와 최종 P gear 1·
오류 없음을 담당 에이전트가 확인했다. 기존 input boundary도 passed/result 0이었으며
SHA는 `828b2cd0e248cca1a2cd12dcd338bc365a6d4ba5d3dfd6ebdab489d1b811d3b6`이었다.

기존 승인 4,936행과 원본을 그대로 참조하는 0촬영 seed를 준비했다. 이때 geometry 생성·
native 촬영·기존 원본 QA 재계산은 0이었다. seed 원문은
`artifacts/evaluation/base5_foggy13_completion_261020/seed_v1/zero_capture_seed.json`, SHA는
`cbc8f17ee247cae2d2f29c252d0703986c847e81b90b1afc45a830a23f4b17b5`다.
이어 기존 bounded 보충을 3,600초/3 attempts 한도에서 실행했고
**04:50:36.945 KST에 exit 0·승인 4,998 / 부족 2 / 미검사 0 / 실패 0**으로 종료했다.
새 승인 62개를 더한 수치이며 부모 4,936행을 중복 합산하지 않았다.

실제 후보는 65개(승인 62 / 검토 3), attempt 기록은 69행이었다. 나머지 4행은 같은
목적 donor 부재로 새 native를 촬영하지 않은 기록이다. 마지막 실제 후보 `00008`의
두 번째 시도는 승인됐고 종료 P·gear 1·오류 없음을 확인했다. 남은 slot은 `02260`
(첫 native의 `primary_no_sensor_evidence`)과 `00016`
(`motion_ground_preparation_unavailable`)이며, 후속 차례에 donor가 없어 촬영 0개였다.
이를 native 실패 4장이나 완료 5K로 바꾸어 표현하지 않는다.

현재 보충 출력은 `artifacts/evaluation/base5_foggy13_completion_261020/flexible_v1/`이다.
기존 parent pins·source/runtime·QA·센서는 바꾸지 않았고, 5K finalize도 아직 하지 않았다.
현재 status의 4,998/2/실패 0을 직접 대조했다. 종료 원문은 같은 준비 root의
`terminal_receipt_v1.json`, SHA
`2cb70961755b0777d89336e6745dfea856b6911c35dda2785c88ec746d3ad239`다.
새 후속 수집은 별도 범위를 확인한 뒤 진행한다. 원격 Foggy15의 동결 runtime이나 제어와
섞지 않는다.

<a id="foggy15-depth2-source-completed-261020"></a>

## 원격 Foggy15의 원형 depth2 source 연결 완료

`adoption-v4` 조립은 **10/20 05:17 KST까지 실제 1,547.30초·exit 0**으로 끝났다.
원형 5K 감사는 `OFFLINE_PASS`·5,000 scenes·5,000 unique payloads·오류 0이었고,
마지막 `verify_binding()`까지 정상 종료한 실행 원문을 별도로 수령했다. CPU 검산은
host 3.10의 실제 native QA와 producer 3.8의 실제 source audit 분리를 유지했으며,
QA·context 결과를 cache하거나 원형 판정을 바꾸지 않았다.

| 실제 완료 원문 | SHA-256 |
| --- | --- |
| `qualification/full-adopt-hybrid-depth2-immutable-v4.json` | `a3bff239ddd7c7e69ac485ef719d82d24cec01f2c529623e45a0818a37130174` |
| `adoption-v4/binding.json` | `5cc6a7b86c3ea719685bf31fdcb62762c335d0df60b7e1fe6631575dfb284bc0` |
| `adoption-v4/assembled_source/final_source/manifest.json` | `012a60fea570620816c1036b491e1ee975d025e79bd4b4319a13f4decb4ffa54` |

로컬 수령본은 `cpu_gate_prepare_v1/source4_followup_actual_v1/`에 원문 bytes로 보존한다.
v3/v4의 고유 5,000 ID 집합은 같고 scenario SHA가 달라진 것은 실제 승인 대체를 연결한
`01228` 한 개뿐이며, 5,000개 entry의 split 변경은 0개다. 기존 6개 대체의 계보는 parent binding으로
계승했다. 이 binding은 명시적 `adoption_depth=2`이며 `native_qualified`, `bulk_ready`,
`bulk_started`는 모두 false다. 따라서 새 source의 등록·unchanged exact32·독립 shadow·
나머지 원형 gate와 공통5를 별도로 통과해야 한다. 이 source 연결 완료를 본수집 5K나
새 native 자격 완료로 표현하지 않는다.

이어 source4의 원형 route witness를 **0.83초·exit 0**으로 생성·수령했다. 실제 witness는
`adoption-v4/route-witness-v1.json`, SHA
`54c6a5f94d9045117c9070ee98cff1198d44b40729817ecbc70390aa20de4807`이다.
새 `qualification/full_v3`의 32장 CPU 사전 등록도 **05:38 KST까지 935.47초·exit 0**으로
끝났다. 실행 원문은 `qualification/full-register-hybrid-depth2-immutable-v3.json`이며,
등록 권위 SHA는 `746d8df33ecd179000bfe91a525a96ed06fc7e18ccf9151475dc41fb77af1040`이다.
기존 107개 제어 pin에 실제 source4·binding·witness·adopt 실행 원문의 4개를 더했으며,
기존 고정 SHA 1,181개·원형 QA는 바꾸지 않았다. 이 시점에는 새 native 촬영과 본수집은 0이다.

새 native3 진입 권위는 `eab5a360b188e65e12d1cb007e77303f20b5f80ac891f65d65470e06890d61fd`다.
기존 native 환경·`ros-home-v2` HOME·source 모듈 3개의 pin을 그대로 유지했다. 촬영 직전
원형 서비스·정지/P를 새로 실측했고 P gear 1·정지 sample 17개·오류 0이었다.
첫16장은 **959.74초·exit 0**으로 정상 PAUSE했다. 실제 parent 선택 16개가 고유하고
등록 ID 및 scenario SHA와 모두 일치했으며, 원래 session의 16행도 전부 승인·검토 0이었다.
각 장면 사이 P 16/16과 마지막 shutdown P·gear 1·오류 0을 확인했다. 이전 `01228`도
이번 등록 geometry 그대로 실제 승인됐다. 실행 원문은
`qualification/full-canonical-first16-depth2-source-home-v4.json`, SHA
`b18d6cc0b699d5c4b287f1237fa44ce1af0a7ba0efd335d9996c05842aaa7837`다.
이것은 원형 exact32의 조기 ID·geometry 대조이며 전체 32장 최종 gate PASS는 아니다.

정상 PAUSE 경계에서 기존 음영 2장의 raw·session을 바꾸지 않는 참조 문서 한 개를 새
`full_v3/shadow-cases.json`으로 전달했다(SHA `fc20a759…b50ff`). 새 등록32와 음영 ID
`00000`·`00003`의 교집합은 0이며, 원래 cases에서 source SHA 필드만 실제 parent source로
바꿨다. 이것은 새 음영 촬영이나 원형 `verify_shadow()` 통과를 주장하는 자료가 아니다.
전체 원형 검증은 freeze에서 실제 수행한다. 첫16장 출력이 멈춘 상태에서 snapshot34를
시작했고, 그 실제 exit 0 전에 remaining16을 실행하지 않는다. 본수집은 아직 0장이다.

snapshot34는 이후 **410.23초·exit 0**으로 끝났고 실제 산출물은 `full_v3/partial.json`이다.
SHA scope 1,181개의 진입·종료 실제 검산과 모든 alias 복원, source audit 호출 0,
QA/context 결과 cache 0을 확인했다. 이 실제 종료 뒤 서비스·정지/P를 다시 실측해
P gear 1·정지 sample 16개·오류 0을 확인하고 remaining16 실행기를 시작했다.
이 실행기 시작은 아직 나머지16장 승인이나 전체 exact32 완료로 집계하지 않는다.

remaining16은 이후 **945.86초·exit 0**으로 정상 종료했다. 새 `full_v3/exact32`는
complete·고유 승인 32 / 검토 0 / 실패 0 / 오류 없음이었다. 등록32의 ID 집합이 정확히
같았고 모든 선택 행의 `id == source_cell_id`·등록 scenario SHA가 일치했다. 두 원래
session의 32행도 전부 승인됐으며 between-scene P 32/32·양쪽 shutdown P gear 1·오류 0을
확인했다. 새 geometry 복구를 등록 원형 승인처럼 합산한 결과가 아니다.
누적 실제 native elapsed는 **173.68초**이며 첫16장 시간을 다시 더하지 않는다.

| 실제 완료 원문 | SHA-256 |
| --- | --- |
| `qualification/full-canonical-remaining16-depth2-source-home-v4.json` | `c2bb906fa25d70740dd6815eab1165263ff94daed1789a19a115b8b6620da012` |
| `qualification/full_v3/exact32/selected_ledger.jsonl` | `33169f67567a78c98777d2b4e7278de8f73aa17fd512a09d97b38ac2a9f225c5` |

이후 원형 partial/completed resume35 CPU 검증을 실제 시작했다. 입력 자료 준비도 이
정상 종료 원문을 기준으로 진행하며, unit corner·독립 음영·입력/loader·CPU·재개를
포함한 전체 6 gate PASS는 아직 freeze의 실제 결과가 필요하다. 본수집은 아직 0장이다.

재개 검증은 이후 **418.33초·exit 0**으로 끝났다. 원형 partial receipt는 `passed=true`·
기존 선택 16개 보존·이전 raw 불변이고, completed replay receipt도 `passed=true`였다.
이는 중간 PAUSE 뒤 나머지 촬영과 완료 뒤 읽기 전용 재실행의 근거이며, 5K 중단 장면
복구나 본수집 완료 인증은 아니다.

| 실제 재개 검증 원문 | SHA-256 |
| --- | --- |
| `qualification/full-resume-proof-hybrid-depth2-immutable-v3.json` | `6fc3a9f28eac2e3da9fad80783ccfef505c3803f66e21675bda7bd63534d24f7` |
| `qualification/full_v3/partial-resume.json` | `9062ff65280cccd1c489fab5379f623972f9baee9e52393878558ae75a0cb4ed` |
| `qualification/full_v3/completed-resume.json` | `36f8b7bc7870e6428a8b651312c36297ec7b05845cb284e5b5032e4bd295b982` |

별도 학습입력 v4 실행은 **875.03초 뒤 exit 1**이었다. 원래 qualifier가 명시하는
`copy_mode='hardlink'`에서 raw와 출력 사이 `EXDEV`가 발생했다. 원인은 입력 전용
장식에서 추가한 `exact32` 읽기 전용 bind mount와 출력 mount의 분리다. 촬영·GT 승인
실패와 구분하며 실패 출력·로그를 보존한다. 일반복사 staging도 원형 source snapshot의
inode·mtime exact 계약과 충돌하므로 그대로 채택하지 않는다. 원형 코드·QA·snapshot
비교는 유지하고, 새 출력 경로에서 원래 hardlink 연결을 만족시키는 최소 namespace
수정안을 검토한다. 이 시점에 입력/loader·전체 6 gate·본수집은 완료되지 않았다.

후속으로 **원형 `fresh.namespace`의 동일 mount를 유지하는 public qualifier CLI**를
새 `full_v3/training-inputs-v5`에 실제 실행했다. 추가했던 `exact32` subtree RO bind만
입력 변환에서 제외했고, 기존 Sunny work/collection 보호·원형 qualifier·117개 기존
control pin·실패 출력은 유지했다. 저렴한 동일 mount/dummy hardlink 확인은
**0.189초·exit 0**, 실제 입력 변환은 **9.357초·exit 0**이었다. 이전 875초는 변환
자체의 필요 시간이 아니라 상위 workflow context를 반복한 경로까지 포함한 시간이다.

원형 보고서는 고유 raw·payload·source cell 각 32개, `source_unchanged=true`,
retained component 누락·추가 0을 확인했다. before/after 비교의 SHA·bytes·inode·mtime
exact 조건은 그대로다. hardlink가 필요한 원형 경로이므로 raw가 mount-RO였다고
표현하지 않는다. 원본 내용 쓰기·chmod·unlink는 없고 nlink/ctime 변화 가능성은 원형
hardlink 특성과 구분한다. 실패 출력 변경·원형 qualifier 변경·추가 native는 모두 false다.

| 실제 입력 완료 원문 | SHA-256 |
| --- | --- |
| `qualification/full-inputs-original-same-mount-v5.json` | `3d1f16a2228f6e6458d2dfb729cc783d77b4a9ee2bf693ccae4a88bd713cfa73` |
| `qualification/full_v3/training-inputs-v5/report.json` | `a30a337d7ad6652692a57b57c8c9eca09fb0c09246ca1d0ce88c26034f256232` |

다음 loader와 원형 freeze에는 새 보고서 경로를 인자로 명시하며, 저장 보고서·PKL을
사후 수정하지 않는다. 입력 변환 성공을 실제 모델 AP 개선이나 전체 6 gate 완료로
표현하지 않는다.

실제 입력 package 3,816파일·295,016,651 bytes를 131.67초에 수령·전수 대조한 뒤,
고정된 기존 Docker image의 CPU-only loader도 **14.148초·exit 0**으로 끝났다.
기존 loader 본문에서 새 `training-inputs-v5` basename만 명시한 연결이며,
`cuda_available=false`·`gpu_execution=false`·`training=false`, source GT 101개를 확인했다.
원문 보고서는 `qualification/full_v3/loader-report-v5.json`에 EXCL로 전달했고 SHA는
`0b0dccd402edcc0488017dc746a567bfc05eed009b7a2d0000e0df540b59baa7`이다.
원형 freeze에는 이 원문과 입력 보고서 경로를 public API 값으로 명시한다.
상위 workflow context의 동일 binding 선행 재검산은 다시 호출하지 않는다.

원형 `reinforcement_native_adoption.freeze(values, output)`는 이후 **590.339초·exit 0**으로
끝났다. `native_exact`, `independent_shadow`, `training_inputs`, `cpu_regression`,
`partial_resume`, `completed_resume` 여섯 gate가 실제로 모두 통과했고,
`native_exact_observations=32`·`bulk_training_observations=0`·`bulk_ready=true`였다.
원형 QA·source audit·판정 문턱을 바꾸거나 과거 검토를 승인으로 승격한 결과가 아니다.

| 실제 최종 자격 원문 | SHA-256 |
| --- | --- |
| `qualification/full-freeze-direct-original-v5.json` | `8bcb2727a454da90b48fd2890e0c8d51c9200b29972b5b82a050a0ad10320ddc` |
| `qualification/full_v3/release/release.json` | `618db882b7b0b149c6a30151b8408a35e5eaba9a33503c5d20ef182a2e174474` |
| `deployment-full-source-v5.json` | `458af54c9d8a4b914e3d774fe309bb40f736fb599be7016f8314e08d689ccaaf` |

순수 final metadata factory도 실제 release와 source4 원문을 연결해 완료했다.
사용자의 명시적 신속 진행 요청에 따라 다음 CPU prepare는 이미 완료된 같은 6 gate를
다시 계산하지 않는 핸드오프를 사용한다. 원형 `qualified_release_context`의 전체 evidence
실제 SHA·현재 runtime·gate 파일 내용·source/output 안전·audit 검사는 그대로 실행한다.
그 검증 안의 `qualify` 콜백에만 실제 성공 freeze가 쓴 원문 결과를 RAM으로 전달하고,
콜백은 단일 prepare 프로세스 안에서만 설치·복원한다. 실패 PASS 변조나 native QA 캐시
확대가 아니며, 원형 profile writer·실제 common5·P/재개·본수집 worker는 그대로 유지한다.
이 시점에는 prepare 실행 중이며 본수집은 아직 0장이다.

한계 정정: 실제 완료된 입력5 즉시 실행기는 원형 namespace를 그대로 사용했으므로
별도 `--unshare-net` 차단 증거가 아니다. 원형 qualifier는 로컬 파일 처리·추가 native 0·
원본 snapshot 불변으로 통과했다. 이 원문은 사후 수정하지 않는다. CPU Docker loader의
실제 `network=none`·GPU 0 및 source-audit RPC의 원래 network-off와 구분한다.

<a id="foggy15-actual-bulk-start-261020"></a>

## 원격 Foggy15 공통5 통과와 실제 본수집 시작

완료된 release의 핸드오프를 사용하는 공통 prepare는 **57.387초·exit 0**으로 끝났다.
원형 profile writer가 새 5K 출력과 source4를 연결했으며, 기존 6 gate의 재계산만 생략했다.
원형 전체 evidence SHA·runtime·source/output 안전 검사는 유지했다. 실제 profile과
prepare 원문은 다음과 같다. 원격 상대 경로는 모두
`~/ajounice_base5_capture/work/fresh-foggy15-v1/` 기준이다.

| 실제 완료 원문 | SHA-256 |
| --- | --- |
| `profile.json` | `3083db331078654a5161b088ad902a95b68843016f7fc2b29f4d54d25c69ac77` |
| `qualification/common-prepare-completed-release-v1.json` | `5e4de15d8ab27eb70a09f00973268f51f786730cc72a258f5432217f4c8e2d6d` |

새 서비스 freshness·정지/P를 실측한 뒤 원형 공통5를 source indices `0,1,2,3,4`로
실행했다. **147.498초·exit 0·승인 5/5·검토/실패 0**이었으며, 실제 PAUSE→resume와
완료 뒤 추가 촬영 없는 replay를 통과했다. 첫 regular `00001`의 센서 근거 부족은
원래 review로 보존했고 원형 bounded recovery가 새 승인 장면을 확보했다. 따라서
이를 최초 geometry 5개가 모두 unchanged 승인됐다고 표현하지 않는다. 이 공통5는
실행기 검증용 별도 출력이며 본수집에 합산하지 않는다.

실제 공통5 원문은 collection의 `unified_live_qualification.json`이며 SHA는
`2f3ea9eb9cb66f1a65e760f338245e90ce7d958c396cc8e85db3272a1b2720b2`다.
고정 입력 boundary는 실제 SHA 확인 2회·7,571,551,168 bytes·9.755초였고,
고정 pin hit 566,681회와 passthrough 9,628회를 기록했다. 이는 고정 입력 SHA 재사용의
근거이지 native QA 결과 캐시가 아니다.

공식 관측기도 최종 profile의 `adoption-v4` manifest로 교체했다. 빌린 PC
`http://127.0.0.1:8083/`와 로컬 터널 `http://127.0.0.1:8084/`에서 source SHA
`012a60fe…fa54`, 새 5K collection, `diagnostic_warnings=[]`·오류 없음을 확인했다.
최종 관측기 supervisor는 PID 3553773, 실제 서버는 PID 3554385이며 SSH 종료 뒤에도
유지됐다. 전환 원문 `observer_final_switch_v1.json`의 SHA는
`156501ade3f8080fddd9b94cf7680c7a8e5ca5e28a4ffd83158554889df4c44e`다.
기존 임시 UI만 교체했으며 MORAI·다른 브라우저·수집 제어는 건드리지 않았다.

이어 새 정지/P·gear 1·정지 sample 17개·오류 없음을 확인하고, 이미 준비한 원형
detached launcher에 실제 deployment/profile/공통5 원문을 연결했다. 실제 본수집
**PID 3555237**은 별도 session·stdin `/dev/null`·일반 파일 로그로 시작했으며,
SSH 종료 뒤 PPID 1로 유지됐다. 실제 명령은 다음과 같다.

```bash
/usr/bin/python3 -B ~/ajounice_base5_capture/work/fresh-foggy15-v1/checks/reinforced_borrowed.py \
  --deployment ~/ajounice_base5_capture/work/fresh-foggy15-v1/deployment-full-source-v5.json \
  --environment foggy15 run --confirmed-environment foggy15
```

본수집 출력은 `~/ajounice_base5_capture/collections/foggy15_5k_v163_v1`이다.
설정은 **limit 5,000 / chunk 16 / stop-after-chunks 0 / 외부 timeout 없음**이며,
원형 묶음·복구 timeout, capture lock, P·freshness·접촉/native QA를 유지했다.
실행 원문은 `bulk-launch-v1/launch-receipt.json`, 로그는 `bulk-launch-v1/bulk.log`다.
이 시점에는 실제 본수집 `running`과 첫 slot의 원형 촬영·복구 진입을 확인했으며,
프로세스 시작만으로 첫 묶음 통과나 5K 완료를 주장하지 않는다.

이후 첫 `chunk_0000`이 실제로 끝났고 부모 status가 **finished 16 / 승인 16 / 검토 0 /
실패 0 / 오류 없음**으로 집계됐다. regular 원문은 승인 15 / 검토 1이며, 검토 원문을
보존한 별도 원형 recovery의 승인 1개가 같은 source slot을 채웠다. regular의 16개
between-scene P가 모두 true이고 마지막 shutdown도 P gear 1·오류 없음을 확인했다.
따라서 최초 geometry 16개 unchanged 승인을 주장하지 않으며, 실제 선택 계보를 따른다.

첫 묶음 집계 후 **`chunk_0001`의 실제 regular 4행이 생성되는 중**인 것을 확인했다.
본수집 PID 3555237은 별도 session·PPID 1로 생존했고 부모 상태는 running이었다.
원형 bulk checkpoint의 selected ledger/quota는 64장 경계에서 갱신될 수 있으므로,
초기 ledger 0행을 촬영 중단으로 해석하지 않는다. 실시간 상태는 status와 현재 child의
원형 ledger를 함께 확인한다. 실행 원문의 수령 SHA는
`c7957a2ff2bf8fcf6234020e894b3b4285da30859bef9bb93a83203213e04d0a`이며,
로컬 근거는 `cpu_gate_prepare_v1/bulk_actual_received_v1/launch-receipt.json`이다.
본수집을 계속 실행한 상태로 문서만 갱신했으며 5K 완료나 이후 무중단을 선인증하지 않는다.

동시 진행한 로컬 Foggy13 담당자는 이후 새 승인 동물 1장과 마지막 walking 1장으로
**5,000 승인 / 부족 0 / 실패 0 / 오류 없음 / 작업자 종료**를 확인했다. 원래 본수집
4,936행 + 기존 보충 62행 + 새 2행의 구성이고, 직전 4,999행을 그대로 보존했다.
마지막 walking은 20~30m 목적과 원형 native QA·P를 통과했으며 거리 fallback은 0이었다.
실제 최종 선택 출력은 `artifacts/evaluation/base5_foggy13_completion_261020/flexible_last_walking_v7/`다.
담당자가 원형 split finalize를 시작했으며 이 기록 시점의 JPEG 변환·PNG 삭제와
독립 Foggy11의 native 촬영은 아직 0이다. 기존 4,998/2 terminal과 실패 근거를
소급 수정하지 않고 후속 실제 승인으로 구분한다.

<a id="inactive-source-cache-cleanup-261020"></a>

## 비활성 로컬 복제·전송 cache 정리

부모 에이전트는 현재 실행·CPU 후속 경로에서 참조하지 않는 로컬 복제 8개 subtree와
Sunny13·Sunny11·Sunny15의 중복 로컬 전송 tar 3개만 검증 후 삭제했다.
실측은 **163,852파일·26,278,735,872 bytes(24.47GiB)**이며, 완료 직후 여유는
52,002,385,920 bytes였다. 이는 Foggy13 선택 PNG 삭제 전의 수치다.

Sunny13·Sunny11의 `training_retrieved_*/dataset/`과 Sunny15의
`training_export_261018_v2/sunny15/training3_jpeg_portable` 총 15K는 각각 전체
payload SHA와 manifest·당시 receipt를 대조하고 보존했다. 원본 capture·GT·모델·
receipt, 원격 source/runtime·PNG와 원격 원본 archive는 삭제하지 않았다.
삭제한 tar 경로는 당시 회수 근거이지 현재 존재하는 학습 입력 경로가 아니다.
완료 근거는 `artifacts/evaluation/base5_storage_cleanup_261020/COMPLETE.json`이다.

별도 Foggy13 no-archive RGB prune 보완의 단위 테스트 9개 통과는 준비 근거다.
이 문단 작성 시점의 실제 Foggy13 JPEG 변환·선택 PNG 삭제는 아직 0이며,
후속 완료 원문이 생기기 전에는 확보 용량이나 삭제 완료로 합산하지 않는다.

<a id="foggy13-supported-training-export-261020"></a>

## Foggy13 native 5K 분할과 지원 GT 4,999장 전달본

최종 원형 분할은 `flexible_last_walking_v7/layout_split_final/result.json`에서
**native 5,000장·train 4,250 / val 750**으로 완료했다. 직전 승인 4,999행을 보존했고
native QA·geometry·누수 검사도 유지했다. 실제 native 선택 ledger SHA는
`be2c2a073a52fb2bd611f60851f2c6e18c8b8e2711e66efeb927afce3874afb2`다.

같은 현재 변환 정책의 최초 5K 변환에서는 `foggy13_sunny13_sunny11_b5v2_00001`이
유효 GT 0으로 제외되어 전체 5K 입력 자격을 통과하지 못했다. 이 장면은 후방 약
49.52m의 Bollard04이며 LiDAR 0점·후방 camera native instance 23pixels다.
원형 `lidar1_camera0` 판정은 `positive_lidar1_camera0=false`였다. native 승인과
학습 지원 component를 구분하며, 해당 객체를 강제 GT 또는 배경으로 바꾸지 않았다.
최초 실패 출력 `qualified/foggy13/derived/conversion_report.json`과 원래 audit를 보존한다.

학습용은 이 한 행만 명시적으로 제외한 **지원 4,999장·train 4,249 / val 750**의
새 `supported_selected_v2.jsonl`을 사용한다. 다른 선택 행의 bytes·순서와 원본 5K
선택은 바꾸지 않았다. 근거 루트는
`artifacts/evaluation/base5_foggy13_training_export_261020_v1/`이며, 08:01 KST의
`qualified_supported_v2/report.json`과 실제 `supported_conversion_COMPLETE_v2.json`은
4,999개 고유 raw/payload/source cell·Q90 JPEG 19,996개·유효 GT 11,652개·지원
component 누락/추가 0·`source_unchanged=true`를 확인했다. 보고서 SHA는
`99a697b1e7d6fc9f866c80be4ee8046f97b9838e448f4c90a7d0fe151ab29530`이다.
현재 입력은 같은 루트의
`qualified_supported_v2/foggy13/training3_jpeg_portable/`이며 archive는 만들지 않았다.
이는 입력 자격이지 tensor loader·학습·AP 개선의 검증이 아니다.

제외한 장면의 native GT·원본과 RGB PNG 4개는 모두 보존한다. 지원 4,999장에만
적용하는 `prune_supported_subset_v2.py`의 원형 전체 검사와 삭제 계획은 별도 단계다.
08:04 KST 확인 시점에는 계획 검사 중이며 실제 PNG 삭제는 0개다. 계획 또는
단위 테스트 통과를 실제 삭제·확보 용량으로 합산하지 않는다.

이후 원형 전수 검사를 마친 계획의 실제 삭제가 **08:20 KST**에 완료됐다.
작은 원문 `rgb_prune_supported_v2_receipt/COMPLETE.json`은 `complete=true`·
**삭제 이름 39,992개**(지원 4,999장 × 원본/portable 2개 경로 × 4camera),
logical unlink 27,950,958,512 bytes·inode 회수 추정 13,975,479,256 bytes(약 13.02GiB)를
기록했다. 두 hardlink 이름의 logical bytes를 실제 회수량으로 중복 집계하지 않는다.
계획 SHA는 `bd0b7ebc9007efaaed3baeb09b11580c75b043892ef0103362a1aec732d5cf3c`다.
원형 원문은 auxiliary 335,401개 보존·과거 integrity proof 수정 0을 확인했다.
이것은 사용자 승인 범위의 지원 4,999장 RGB PNG 삭제이며, Q90 JPEG는 손실 변환이다.
별도 무손실 archive를 만들었다고 표현하지 않고 과거 source-RGB 검사는 삭제 뒤
그대로 재실행할 수 없다는 한계도 보존한다. 부모 담당자의 08:20 공간 실측은
여유 57.13GiB였으며, helper 종료 뒤 제외 장면의 원본 불변 확인은 별도 종료 근거로 남긴다.

후속 helper 최종 검사도 `excluded_raw_preserved=true`로 끝났다. 제외한 `00001`의
전체 native 원본과 RGB PNG 4개는 이전 SHA 그대로 보존했고, native 5K 선택·GT를
지원 4,999장 학습 분모에 맞춰 수정하지 않았다. 담당자는 실제 여유 약 57.1GiB와
55GiB 목표 통과를 확인했으며 JPEG 입력 폴더만 유지하고 tar는 보류했다.
운영 종료 근거는 `prune_supported_v2_owner_exit.json`이며, 이미 관측한 실제
session 57988의 exit 0·최종 stdout을 후속 전사한 자료다. 이를 위한 재검산은 0이고
원형 COMPLETE SHA `bba246f2d1d09a48adb0c75ad77b87c681fc2679cbd00892f2ce69beaf648e3b`와
원본 proof는 바꾸지 않았다.

<a id="foggy11-independent-preparation-261020"></a>

## 독립 Foggy11 준비와 반복 CPU 검사 범위

준비 루트는 `artifacts/evaluation/base5_foggy11_design_261020/`이다. 공통 v160·같은
v163 정책에 독립 seed **26102011**을 사용했고, 실제 CPU 완료 원문 `cpu_complete.json`은
보강 141장(20~30m 117 / 30~45m 24)을 기록했다. Foggy13 장면을 Foggy11로 이름만
바꿔 쓰거나 이전 native 성공을 새 환경 승인으로 합산하지 않는다.

`qualification/pilot_v1/exact32/status.json`은 실제 승인 32/32·검토/실패 0·오류 없음을
기록했다. 이 중 3개는 원래 검토 원문을 보존한 새 geometry 복구 승인이다. 따라서
원래 pilot의 unchanged exact32 또는 전체 6 gate 완료로 표현하지 않는다. 실제
복구 계보를 연결한 새 source의 원형 adoption·등록·자격은 별도 확인한다.

최초 `adoption-v1` CPU 실행은 고정 source/지도 근거의 SHA·JSON을 상위 context마다
반복 읽는 경로에서 약 11분이 소요되어 정상 SIGINT로 중단했다. 실패 로그·부분 출력과
`cpu_adoption_v1_normal_interrupt.json`을 보존하며 성공으로 바꾸지 않는다. 새
`adoption-v2`는 기존 검증 cache의 고정 code 388개·과거 source blob 770개·명시된
준비 manifest/overlay 14개만 exact SHA 경계 안에서 재사용한다. 원래 immutable overlay
reader의 SHA 검증은 유지하고 raw/session·native QA·context 판정 결과는 cache하지 않는다.
CPU 범위에서는 child 실행을 금지하고 종료 후 원래 함수를 복원한다.

이후 prepare용 `completed_release_prepare_v1.py`는 실제 성공 freeze·같은 source와
전체 evidence/runtime SHA가 있어야만 완료된 6 gate 결과를 다시 연결할 수 있다.
helper·단위 테스트만으로 release나 native gate를 통과시키지 않는다. 실제 common5·
P/freshness·접촉/native QA·resume·본수집 worker는 유지한다. 08:04 KST 시점에는
새 adoption 원형 검증 중이며 binding/freeze·공통5·본수집은 아직 완료되지 않았다.
`http://localhost:8085/`는 읽기 전용 준비 관측기이며 실제 촬영 수집기는 아니다.
예정 본수집 출력은 `/home/luv/morai_cap/base5_foggy11_final_v163_261020/collection_5k_v1`이고,
실제 시작 후 최종 수집 관측기는 로컬 8081로 연결한다. 원격 Foggy15의 8084와 구분한다.

이후 `cached_adoption_v2_exit.json`이 실제 **870.715초·returncode 0**으로 생성됐다.
원형 adoption이 3개 실제 복구를 새 5K source에 연결했고 4,997개 payload를 유지했다.
최종 manifest SHA는 `a60f5c63b0ad2dffb4e73f5736830c1efadd293dc272dc9445d12efa83255d12`다.
CPU cache scope는 native 전에 닫혔으며 QA 변경은 0이다. binding의
`native_qualified`·`bulk_ready`·`bulk_started`는 여전히 false다.
`adopted_pipeline_v2/00_route_exit.json`도 **0.508초·exit 0**이며, 08:10 KST에는
새 source의 원형 등록을 실제 시작했다. adoption·route 완료를 새 exact32·6 gate·
공통5·본수집 완료로 올려 세지 않는다.

<a id="foggy15-running-observation-0805-261020"></a>

## Foggy15 08:05 KST 실행 관측

부모 담당자는 기존 8084의 `/status.json`을 읽기 전용으로 확인했다. 원격 Foggy15는
**running·부모 승인 285 / finished 288 / review 3·error null**이었으며, child
`chunk_0018`의 실제 진행 7/16과 P gear 1을 관측했다. `diagnostic_warnings=[]`·
`status_stale=false`였고 원격 여유는 68.18GiB였다. 이 시점 관측은 기존 본수집 실행
근거에 추가하는 상태이며, 원격 파일 복사·수집 제어·새 쓰기는 수행하지 않았다.
review 원문과 복구 계보를 유지하며 5K 완료·모든 원래 geometry 승인·향후 무중단을
선인증하지 않는다. 로컬 Foggy11의 준비 관측과 혼동하지 않는다.

<a id="foggy11-exact32-review-blocker-261020"></a>

## 오전 Foggy11 strict exact32의 일반 센서 review와 당시 본수집 미시작

새 source의 원형 등록은 `adopted_pipeline_v2/01_register_exit.json`에서
**797.617초·exit 0**으로 끝났다. 첫16장도 실제 승인 16/16·P 통과 후
`adopted_pipeline_v3/00_first16_exit.json`의 **377.070초·exit 0**으로 종료했다.
그러나 나머지16장 뒤 최종 원형 집계는 **검사 32 / 승인 31 / review 1 / 실패 0 /
error null·complete_with_deficit**이었다. `adopted_pipeline_v5/00_remaining16_exit.json`은
**201.887초·exit 1**이며, native worker의 실행 오류가 아니라 review 0을 요구하는
원형 source 자격 조건에서 후속 workflow가 중단된 것이다.

남은 ID는 `foggy11_sunny13_sunny11_b5v2_03980`이다. Dog2 약 38.9m 장면의
LiDAR 0점·CAM1 native instance 28pixels로 `primary_no_sensor_evidence`였고
원형 bounded 복구 3회도 승인에 이르지 못했다. 원래 검토·실패 후보·raw·QA와
이전 31개 실제 승인은 그대로 보존했다. 검토를 승인으로 고치거나 QA/P를 위조하지 않았다.

실제 종료 근거는 `exact32_review_stop_v1.json`이다. 최종 owner 3169446·child 3169447·
자동 main 대기 caller 3169461은 모두 종료됐고, 마지막 실제 shutdown은
`park_verified=true`·P gear 1·errors `[]`였다. shutdown 원문 SHA는
`64161b7443a9b339a93d6ab4572b8e8cd8718be784ffcbb571785942b725c1ef`다.
통신·정지/P 장애가 아닌 일반 센서 review로 구분한다. **후속 6 gate·공통5·본수집은
미완료이며 최종 profile·본수집 출력 자체가 없다. 본수집 0장·중단 뒤 새 adoption/추가 촬영 0**이다.
8085는 준비 RO 관측기로 유지하고 8081을 Foggy11 본수집 관측기로 전환하지 않았다.
원형 엄격 계약 아래 새 source adoption/재검증을 할지, 이 일반 센서 부족을 사전
인증과 분리하는 정식 정책을 정할지는 사용자 결정 대기로 남긴다. 이 문서가 정책
완화·추가 MORAI 조작을 승인하거나 이미 시작했다고 뜻하지 않는다.

반복 CPU 지연의 최종 경계도 실제 실행과 구분해 기록한다.
`completed_binding_phase_entry_v1.py`는 실제 완료 REGISTER·binding·등록본 SHA를
고정하고, 초기 `bind_paths` 안의 중복 source geometry 재계산 한 호출에만 완료 근거를
재사용했다. 각 phase의 전체 binding evidence/runtime는 원형 `verify_hashes`로 실제
검사하며, 최종 remaining16 로그 첫 행은 **입력 33,990개·실제 pin 검산·
`gate_pass_claimed=false`**를 기록했다. hook은 phase 본문 전에 복원했으며 native QA·
freeze 본문의 원형 검증과 6 gate 문턱은 바꾸지 않았다. 이 최적화가 일반 review를
PASS로 바꾸지 않았다는 실제 결과가 위 exit 1이다. 완료 release 재사용 helper와
그 테스트는 준비 근거일 뿐, 이번 실행의 freeze·공통5·본수집 성공 근거는 아니다.

<a id="foggy11-user-authorized-start-permit-261020"></a>

## 사용자 승인 start permit과 Foggy11 본수집 경계

10/20 사용자는 일반 센서 근거 부족의 review를 review/skip으로 남기고 본수집을
진행하도록 명시적으로 승인했다. 이에 새 start permit 정책을 분리했다. 오전 원형
exact32의 **관측 32 / 승인 31 / review 1·exit 1**과 중단 근거는 그대로 보존하며,
31개 승인을 32개 승인이나 엄격 자격 PASS로 바꾸지 않는다. 이는 완료된 6 gate를
재사용한 경로가 아니라 사용자가 승인한 별도 시작 권한이다.

근거 루트는 `artifacts/evaluation/base5_foggy11_design_261020/soft_review_start_v1/`다.
실제 `start_permit.json`의 정책은
`user-authorized-start-not-strict-six-gate-or-common5-pass-v1`이며,
`strict_exact32_passed=false`·`strict_six_gate_passed=false`·`common5_completed=false`·
`gate_evidence={}`를 명시한다. 새 exact32·공통5를 추가 촬영하지 않았고, 과거 시험
관측을 본수집에 상속·합산한 수는 0이다. 기존 source manifest SHA
`a60f5c63b0ad2dffb4e73f5736830c1efadd293dc272dc9445d12efa83255d12`도 유지한다.

새 진입점 `soft_review_start_entry_v1.py`의 SHA는
`46984bb4e990c5df35fa31e2e0831df76759fbe5a2f05741612064bf70112e08`이다.
source/runtime·원래 raw·P 근거의 전체 입력 SHA를 실제 경계에서 확인한다.
원형 profile 생성·초기화·parent run·collector·native QA·P·STOP/PAUSE·bounded 복구는
유지하며 일반 review를 강제 승인으로 바꾸지 않는다. 새 entry의 경로/SHA를 연결하는
명시적 worker adapter만 사용하고 원형 core의 파일 identity를 alias로 가장하지 않는다.

실제 `operator_ready_v1.json`은 통신·CollisionData 수신·P·중복 collector 없음과
Foggy·11:00·시간/날씨 자동 순환 OFF·당시 여유 56.89GiB를 기록했다.
`prepare_v1_exit.json`은 **62.539초·exit 0·native_captures_started=false**다.
이 prepare 완료 자체는 실제 본수집 시작·첫 묶음 승인·5K 완료를 뜻하지 않는다.

이후 실제 detached 본수집 **PID/SID 3543856·PPID 1299**가 시작됐고 5분 이상 생존을
확인했다. 출력은 `/home/luv/morai_cap/base5_foggy11_final_v163_261020/collection_5k_v1`이다.
`bulk_v1_launch.json`·`bulk_started_first_chunks_v1.json`이 실제 시작과 첫 묶음 근거다.
후자의 permit SHA는 `e0af0445bdd5ca67937b4ccf8c789399b2bc41a631fa40cecc53c819f76a335c`,
profile SHA는 `b0364b87212129537d8d1aa69139c8236422c8ca66d3a1fc9e81d63851ee7e4a`다.
limit 5,000 / chunk 16 / stop-after-chunks 0 / 외부 timeout 없음으로 실행한다.

**17:13 KST 실제 부모는 running·승인 32 / finished 32 / review 0 / 실패 0 / error null**이며
다음 묶음 촬영을 확인했다. 첫 regular 16장은 승인 15 / 검토 1이고, 원래 검토를 보존한
원형 recovery의 새 승인 `00001_replace`가 해당 slot을 채웠다. 첫 shutdown·복구 마지막
P는 모두 gear 1·errors `[]`였다. 따라서 최초 geometry 전부의 unchanged 승인이 아니다.
원래 시험 31장과 새 본수집 성공을 중복 합산하지 않는다.
이후 최종 실제 부모 관측은 **승인 48 / finished 48 / review 0 / 실패 0 / error null·running**이며
`chunk_0003` 촬영 중이었다. `bulk_first_two_chunks_park_v1.json`은 첫 묶음의 원래
15승인/1검토와 recovery 1승인, 둘째 묶음 16/16·최종 P gear 1/errors `[]`를 보존한다.
**http://localhost:8081/**는 이 실제 profile/출력을 관측하며 `status_stale=false`·
`diagnostic_warnings=[]`였고 실제 브라우저도 열었다. 8085는 이전 준비 RO 관측기다.
실제 수집 시작과 첫 묶음 승인을 기록하는 것이며 5K 완료나 엄격 6 gate PASS 주장이 아니다.

동시 확인한 원격 Foggy15의 **http://localhost:8084/**도 **running·승인 4,434 /
inspected 4,483 / review 49 / 실패 0 / error null·stale false·warnings `[]`**였다.
기존 source4·수집·안전/P를 유지한 채 계속 실행 중이며 원격 수집 제어는 변경하지 않았다.
실제 8084 브라우저도 열었고, review 49개를 승인으로 올리거나 완료 5K로 표시하지 않았다.

<a id="foggy15-completion-and-foggy11-running-261020"></a>

## 10/20 저녁: 원격 Foggy15 부족분 보충과 로컬 Foggy11 유지

사용자는 원격 부족분 수집과 로컬 실행 유지를 요청했다. 원격 Foggy15 본수집은
**검사 5,000 / 승인 4,939 / review 61 / 실패 0 / error null**의
`complete_with_deficit` 정상 종료였다. 마지막 실제 shutdown은
`park_verified=true`·`gear=1`·`errors=[]`였고 종료 고정 입력 경계도 result 0이었다.
본수집이 도중에 죽은 상태와 구분하며, 원래 승인·검토·raw·native GT는 보존한다.

빌린 PC `~/ajounice_base5_capture/` 기준 새 경로는 다음과 같다.

| 용도 | 경로 |
|---|---|
| 보존하는 종료 부모 | `collections/foggy15_5k_v163_v1` |
| 촬영 0회 선택 seed | `collections/foggy15_5k_completion_seed_v1` |
| 실제 유연 보충 | `collections/foggy15_5k_flexible_v1` |
| 별도 실행기 | `work/foggy15-completion-runtime-v1/fresh_completion_borrowed.py` |
| 실제 run 로그 | `work/foggy15-completion-runtime-v1/run-service-v1.log` |

기존 Sunny11 공통 유연 보충 모듈 4종을 같은 SHA로 사용했다. 강화 source의 helper는
namespace 내부 logical 경로에 동결돼 있어, 새 wrapper에서 그 원래 helper를 import하도록
연결만 보완했다. 기존 source·동결 runtime·native QA를 수정한 것이 아니다.
관련 공통 어댑터 회귀는 36/36 통과했고, 실제 prepare는 177.88초·exit 0이었다.
기존 4,939장 픽셀·점군·native QA를 재실행하지 않았으며, 준비 중 새 native 촬영도 0회였다.
새 qualification 32장·공통5 촬영을 추가하지 않았다.

이후 별도 owner PID 3905893·run PID 3906474로 실제 보충을 시작했다.
**20:53 KST 독립 웹 확인: 승인 4,940 / 부족 60 / 새 승인 1 / attempts 2 /
error null·running·stale false**였다. 첫 `00211` 부족 slot에는 donor `00494`의 새 장면이
실제 native 취득 약 9.6157초로 승인됐다. 해당 child는 1승인·검토/실패 0·error null이며
최종 P도 `park_verified=true`·gear 1·errors `[]`였다. 다음 `00242` 셀 진행까지 확인했다.
기존 승인에 새 승인만 더한 수치이며, 부족분을 검토 없이 강제 승인한 것이 아니다.

후속 실제 인계 관측은 **새 승인 3 / 누적 4,942 / 부족 58·running·error null**이었다.
새 3개 세션 모두 최종 P·gear 1·errors `[]`를 확인했고, 본수집 4,939개 선택 행 전부가
동일하며 전체 source ID도 고유했다. 로컬 근거는
`artifacts/evaluation/base5_foggy15_completion_261020/actual_supplement_running_receipt_v1.json`이다.
수집은 셀당 최대 3회·7,200초의 원래 유한 예산으로 계속 실행한다. 별도 watcher PID 3907279는
run exit 0·실제 5,000장·부족 0·오류 없는 정상 종료 때만 기존 finalize를 한 번 호출하며,
부족분이 남거나 안전 중단이면 새 촬영·finalize 없이 끝난다. 현재 관측을 완료로 해석하지 않는다.
상태·로그·정상 PAUSE 이후 재개 명령은 빌린 PC의
`work/foggy15-completion-runtime-v1/COMMANDS_foggy15_completion_261020.txt`에 남겼다.

현재 보충 웹은 **빌린 PC 8086 / 로컬 http://localhost:8087/**다. 실제 HTTP/JSON의
Foggy15 manifest·새 보충 출력·누적 4,939+1을 확인하고 브라우저에도 열었다.
기존 빌린 PC 8083 / 로컬 8084는 종료된 본수집 관측기로 남겼다. 두 화면을 합산하지 않는다.
이 기록은 보충 시작·첫 승인의 근거이며 부족 0이나 5K 최종 분할 완료 주장은 아니다.

로컬 Foggy11은 수집·MORAI·bridge를 중단하거나 변경하지 않았다. 같은 PID 3543856과
별도 SID가 3시간 이상 살아 있었고, 일반 review의 bounded 복구 뒤 다음 묶음 진행을
확인했다. **20:53 KST http://localhost:8081/ 독립 관측은 승인 1,630 / 검사 1,648 /
검토 18 / 실패 0 / error null·running·stale false·warnings `[]`**였다. 여유는 약
46.64GiB였다. 이는 실행 중 관측이며 전체 5K 완료 인증은 아니다.

Foggy11이 정규 Sunny11/13/15·Foggy11/13/15의 마지막 본수집이다. 양쪽 부족분까지 채워야
native 30K가 완료된다. Sunny 추가 보강은 별도 선택 계획이며, JPEG 학습 입력 변환·선택
분할·회수 완료 여부도 native 장면 수와 구분한다.

<a id="foggy15-terminal-two-and-jpeg-retrieval-261021"></a>

## 10/21 Foggy15 마지막 두 장의 실제 완료와 JPEG 회수

앞 절의 유연 보충은 10/20 21:36:23 KST에 **승인 4,998 / 부족 2**로 중단됐다.
상위 오류는 `Native safety/communication/Park child failed; no next load`였지만 실제
원인은 `00217`의 준비 payload가 이미 관측된 SHA와 같아 재촬영을 막은 가드였다.
`00008`은 기존 목적을 유지하는 donor 탐색을 소진했다. 마지막 두 준비 세션의
정지/P·CollisionData는 정상이었으며 MORAI 재부팅이 필요한 통신 장애의 증거는 없었다.
기존 halted 상태·부족분·시도 이력을 정상으로 고쳐 쓰지 않았다.

사용자가 마지막 두 장의 쉬운 대체와 JPEG 회수를 승인해 새 별도 출력
`~/ajounice_base5_capture/collections/foggy15_5k_terminal_v1`을 사용했다. 앞선 4,998개
선택 행을 문자 그대로 보존하고, 두 slot의 기존 ordinal 1~3 이력과 알려진 payload를
상속한 뒤 새 ordinal 4에서 각각 다른 native 장면을 촬영했다. 두 장에 한해
family·동작·의미군·거리 목적을 정적 차량 5~30m로 완화했으며 split과 실제 native QA·
중복 재촬영 차단·최종 정지/P는 유지했다. 재시도 횟수를 0으로 되돌린 것이 아니다.

새 `00008`은 truck 약 11.41m·LiDAR 510점, `00217`은 OBJ truck 약 22.22m·214점이었다.
둘 다 실제 native 승인·raw complete·충돌 검증·최종 P gear 1/errors `[]`를 확인했다.
10/21 00:16경 실제 status와 정상 run exit 0에서 **승인 5,000 / 부족 0 / 실패 0 /
error null·complete·selected_not_certified=false**를 확인했다. 기존 4,998행은 동일하고
source slot·실제 ID·raw 경로·payload는 각각 5,000개 고유했다.

원격 native 완료 근거는
`work/foggy15-terminal-finalize-v1/NATIVE_COMPLETE.json`이다. 새 운영 helper·과거 준비 실패
근거는 `work/foggy15-terminal-two-v1`~`v4`에 보존한다. 시작 전 스키마 검사 실패와 FD 한도
실패는 native 촬영 0회였으며, 완료된 59개 보충은 저장된 원형 인증의 작은 evidence SHA로
연결했다. 원본 4,939장 native QA나 완료된 새 59장의 원본 전체 QA를 다시 실행하지 않았다.
작업자만 umask `0077`·FD soft limit 65,536을 사용했고 시스템 한도는 변경하지 않았다.

이 native 완료와 최종 분할·JPEG 변환·회수 완료는 별도 단계다. 원격 RGB PNG·native GT·
실패 원본은 삭제하지 않으며, 로컬 Foggy11 수집도 중단하거나 변경하지 않는다.

이후 선택 기반 최종화가 **4분 42.7초·exit 0 / train 4,250 / val 750**으로 완료됐다.
원래 capture split은 보존하고 실제 선택 geometry 261행을 파생 overlay에 연결했다.
변환 입력은 최종 선택 아래 `layout_split_final/{manifest.json,derived_selected_ledger.jsonl,
split_overlay.json}`이며 manifest SHA는
`e0dcac0f664d89328ceadef705a8f015c7ac7f71fb2b94730192a3ba41c673a0`,
derived ledger는 `eff0242ffb898b8f9e3c6b900bc0b049e4d8f14e85a1860abd49230860db3f96`이다.
로컬 `artifacts/evaluation/base5_foggy15_completion_261021/LAYOUT_JPEG_HANDOFF.json`의
9개 제어 SHA와 native 완료 receipt를 확인한 뒤에만 JPEG 변환에 넘겼다.

기존 웹 서버만 종료하고 별도 최종 관측기로 바꿨다. **빌린 PC 8086 / 로컬 8087**의
실제 JSON이 `foggy15_5k_terminal_v1`·complete·5,000/0/0·error null·stale false임을
독립 확인했다. 이전 4,998장 중단 출력은 수정하지 않았다.

### JPEG 변환의 저장 방식만 보완

원격 남은 공간은 약 33.6GiB였다. 원형의 원본 RO namespace에서는 초기 PNG portable
23.6GB를 별도 복사하므로 JPEG·압축본과 10GiB 안전 여유까지 유지할 수 없었다.
PNG 삭제 대신 새 export 내부에 원본과 같은 inode를 가리키는 alias를 준비하고,
원형 portable builder의 첫 hardlink 입력만 그 alias로 연결했다. native RGB·auxiliary
187,826개 파일의 SHA·device/inode·크기와 최종 5,000개 선택 identity를 고정했다.
초기 PNG payload 중복 복사는 실제 0바이트다.

이는 JPEG 인코더·픽셀·GT·가시성·분할 규칙의 변경이 아니다. 기존 Sunny15의 변환 helper
11개와 센서 SHA가 일치했고, 같은 native 해상도 Q90·upright 7D·Animal→Pedestrian
3클래스 경로를 사용한다. 별도 저장 adapter의 12개 fixture와 원형 writer의 쓰기 경로를
검토했다. 원래 함수의 새 빈 export 검사에서 정확히 검증한 alias 디렉터리 하나만 허용하고,
기존 INTENT·다른 파일·비어 있지 않은 tmp·잘못된 inode·경로 이탈·복사 fallback은 거부한다.

hardlink alias는 물리적인 inode RO 보호와 같지 않다. 새 경로와 원본이 inode를 공유하므로
동결된 원형 코드가 원본 payload를 덮어쓰지 않는지 확인하고 실행 전후 전체 원본 SHA를
대조한다. namespace에서 원본 경로가 RO라는 이유만으로 inode 전체가 RO라고 주장하지 않는다.
향후 RGB 이름만 삭제해도 alias·중간 portable의 link가 남으면 공간은 회수되지 않으므로,
현재 alias manifest와 원본 보존 증거를 남긴다. 이번 작업의 원격 PNG·alias 삭제는 0개다.

실제 변환은 `work/foggy15-training-export-runtime-v1/`의 별도 CPU service로 실행한다.
출력은 `work/foggy15-training-export-v1/`이며 원격 전체 디스크 여유 10GiB를 감시한다.
원본 before-SHA를 통과하고 **모델 crop용 mask 측정·학습 GT frame 검사 5,000/5,000 /
train 4,250 / val 750 / excluded_review_scenes=[]**를 확인했다. 지원 GT 부족으로 조용히
장면을 제외한 수는 0이다. 이 시점은 JPEG 생성 중이며 최종 압축·로컬 회수 완료와 구분한다.
운영 source·fixture·수령 준비는 로컬
`artifacts/evaluation/base5_foggy15_training_export_261021_v1/`에 보존한다.

원격 qualification은 **약 19분 05초·exit 0**으로 완료됐다. JPEG는 실제 20,000개이며
전체 5,000장·train 4,250 / val 750를 유지했다. 유효 학습 GT는 Vehicle 4,477 /
Pedestrian 3,148 / Obstacle 3,996, 합계 **11,621개**다. 원형 Animal→Pedestrian 매핑이며
새로운 의미 라벨 정책을 추가하지 않았다. 등록 package payload는 187,835개·
10,057,356,961바이트, manifest는 별도 38,497,998바이트였다.
`work/foggy15-training-export-v1/QUALIFIED.json`과 `qualified/report.json`이 이 입력 검증의
근거다. 이 시점은 원격 입력 검증 완료이며 압축·after-SHA·로컬 수령 완료는 별도로 확인한다.

이어 원격 압축·압축 해제 스트림 전수 검증·원본 after-SHA가 모두 정상 종료됐다.
압축본은 `base5_foggy15_5k_jpeg90_upright3_261021.tar.zst`이며
**2,836,797,490바이트·내부 파일 187,839개**다. 전체 압축본 SHA는
`3835b002025ff99071d60fc71be8fc341edc1ae3c2e891644a1d9a9081edc26e`다.
`archive_receipt/COMPLETE.json`의 `verified_decompressed_members=true`·
`source_package_unchanged=true`와 runtime `PIPELINE_COMPLETE.json`을 확인했다.
원본·alias 187,826개 전후 SHA도 같고 원격 PNG 삭제는 0개다. 이 원격 완료 근거를
로컬 `artifacts/evaluation/base5_foggy15_training_export_261021_v1/export_receipts/`에 수령했다.
실제 로컬 압축본 전송은 단일 rsync로 진행하며, 로컬 복원 완료는 후속 receipt로 구분한다.

### 로컬 수령·복원 완료

**10/21 01:06:21 KST 원형 복원 COMPLETE, 01:06:22 후속 감사 완료, receive exit 0**을
확인했다. 단일 압축본의 실제 전송은 14분 34초였으며 전체 archive SHA와 압축 해제된
187,839개 멤버의 크기·SHA·등록 목록 검사를 통과했다. 실제 학습 입력은 다음과 같다.

- dataset: `artifacts/evaluation/base5_foggy15_training_retrieved_261021_v1/dataset/`
- 압축본: `artifacts/handoff/base5_foggy15_5k_jpeg90_upright3_261021.tar.zst`
- 복원 근거: `artifacts/evaluation/base5_foggy15_training_retrieved_261021_v1/COMPLETE.json`
- 후속 계보·GT 감사: `artifacts/evaluation/base5_foggy15_training_export_261021_v1/received_audit.json`

복원 입력은 **5,000장·train 4,250 / val 750·Q90 JPEG 20,000개·유효 GT 11,621개**이며
package manifest SHA는 `2a735fea803e91d4d3f92cae84b70452093dd2417d102998fd0d624960dfce75`다.
최종 derived ledger·split overlay와 원격 qualification의 입력 SHA가 같고, class별 GT도
Vehicle 4,477 / Pedestrian 3,148 / Obstacle 3,996으로 일치했다. 학습은 시작하지 않았다.
원격 PNG·native GT·실패 원본의 삭제는 0개다. Q90 JPEG는 손실 변환이고 archive 압축과 구분한다.

복원 전 로컬 Foggy11의 당시 남은 수집 예상 9.35GiB에 여유 5GiB를 더해 예약했고,
저우선순위로 복원했다. **01:07 KST Foggy11은 승인 3,502 / 검사 3,537 / 검토 35 /
실패 0 / error null·running·stale false**로 계속 진행했다. 당시 여유는 22.32GiB,
남은 수집 예상은 9.28GiB였으며 수집·MORAI·ROS·bridge를 변경하거나 재시작하지 않았다.
이는 Foggy15 전달 완료 기록이며 정규 30K 전체 완료 주장은 아니다.

<a id="sunny-obstacle-supplement-750-261021"></a>

## 10/21 Sunny 장애물 단독 750장 추가 수집

사용자는 원격 추가 수집을 승인한 뒤 혼합 후보를 취소하고 **장애물 단독 750장**을
최종 선택했다. OBJ 컨테이너의 차량도 제외하라고 별도로 명시했다. Sunny11·13·15시
각 250장, 환경별 20~30m 125장 / 30~45m 125장이다. 낮고 넓은 CargoBox·Concrete01도
원래 40m cap 안에서 원거리 후보에 포함한다. 큰 자산은 확대가 아닌 native 원형이다.

`artifacts/evaluation/base5_sunny_supplement_261021/generation/final_v5/`의 실제 750장과
catalog semantic·native unit raw class를 전수 대조했다. 모두 순수 Obstacle 1개이며
차량 OBJ·일반 차량·사람·동물·복합체·spawn·이동 경로는 0이다. 12종만 사용하고
모든 scale XYZ는 (1,1,1)이다. 공통 sampler·geometry·split overlay 검증과 기존 Sunny15K
대비 exact/near 중복 0을 확인했다. 계획 split은 환경별 212/38, 총 636/114다.
이는 새 CPU source 검증이며 native 승인 750장을 미리 주장하는 것이 아니다.

환경별 manifest SHA는 다음과 같다.

- Sunny11: `d95f0c4e184d18240f193a332080eb994139bfd247e487456a9160a27ae8e0a1`
- Sunny13: `bc94af71ed216593b20311fbf154d7c8f48f0e57685a1c483acc2ef2d17aa40d`
- Sunny15: `d1553288150831671c606ed1b916d4dbb1c98b89378ffc58e855ae53ead23d21`

원격 작업은 `~/ajounice_base5_capture/work/sunny-supplement-261021-v1/`, 별도 본수집은
`collections/sunny_supplement_750_v1/`이다. 기존 5K source를 250장으로 위장하거나 과거
32장 자격을 상속하지 않는다. 같은 원형 native 부모의 64/64/64/58 묶음과 청크 8·
정지/P·CollisionData·QA·strict recovery·sealed worker를 사용한다. 기존 76,262개 입력을
매번 재검산하는 대신 명시적인 기존 서비스·동결 runtime 재사용 모드를 적용했고,
새 source·실제 runtime·지도 호환 자료는 시작·재개·종료 경계에서 검사한다.
별도 실행기 25개와 공통 회귀 237개를 통과했다. global PAUSE의 active block 전달과
완료 block의 읽기 전용 재검증·선택/부족분 정확 분할·무결성 실패 격리를 확인했다.

전달은 신규 source·설정 계보 2,615개를 담은 **27.70MB delta**만 사용했다. 배포 경계에서
기존 참조 1,507개의 실제 SHA·신규 파일 전체 SHA·기존 deployment 전후 불변을 확인했다.
기존 raw PNG·bag을 다시 전송하거나 과거 source를 수정하지 않았다. `SOURCE_INSTALLED.json`이
실제 설치 근거다. CPU prepare는 **27.108초·exit 0·native 촬영 0장**이었다.

날씨는 공식 도구의 정확한 Tk 앱 버튼 callback을 직접 호출했다. 마우스 좌표 클릭과
GDK 화면 캡처는 과거 화면을 반환해 환경 증거로 사용하지 않는다. 공식 시각·Sunny 버튼,
Disconnect→Connect 후 실제 Tk label **13:00 / Sunny**를 확인했다. 자동 순환은 OFF다.
기존 공식 도구·SDK·네트워크·센서 설정은 수정하지 않았다.

Sunny13 첫 실제 8장은 **8/8 승인·검토/실패 0·error null**, 정상 PAUSE와 최종 P를
확인했다. 낮은/큰/긴 형상·전후·두 거리 그룹을 포함한다. 실제 native annotation은 모두
Obstacle 1개, 원형 치수 비교 일치·충돌 확인을 통과했다. 44.75m WoodBox·44.55m Steel과
36.55m CargoBox는 LiDAR 0점이지만 실제 카메라 instance 근거가 남았다. WoodBox의 실제
RGB도 원경에 형상이 보임을 확인했다. 이것은 모델 AP나 모든 원거리 GT의 인식 보장이 아니다.
`operations/pilot_qualification_sunny13_v1.json`이 실제 8장 근거다. 이후 같은 부모에서
승인분을 보존하고 수집을 재개했으며, 세 환경 완료·실제 부족분·Q90 변환·회수를 각각
확인할 예정이다. 전체 750장 완료로 표시하지 않는다.

읽기 전용 보강 웹은 **빌린 PC 8088 / 로컬 http://localhost:8089/**다. 12개 부모의 확정
상태만 한 번씩 합산하며 진행 중 child를 중복 더하지 않는다. 구 8086/8087 Foggy15 완료
대시보드와 구분한다. 원격 RGB PNG·native GT·이전 실패 원본은 보존한다.

후속 Sunny13 본수집은 **249장 승인 / 검토 1 / 실패 0 / 오류 없음**으로 정상 종료했다.
그 한 셀만 같은 물리 자산·거리·방향·분할의 새 payload로 촬영해 승인했고,
보충 owner는 **47.837초·exit 0**이었다. 원래 249개 행과 실패 근거는 보존했다.
`collections/sunny_supplement_completion_v1/sunny13/layout_split_final/`에서
**실제 250장 / 부족 0·train 212 / val 38**을 최종화했다. 이후 공식 Tk reconnect
readback **11:00 / Sunny·자동 순환 OFF**를 확인하고 Sunny11 첫 청크를 시작했다.
기존 8088 관측기의 249/검토 1은 본수집 이력이며 이 최종 보충 선택에 다시 더하지 않는다.
전체 750장과 JPEG 전달·회수는 아직 후속 단계다.

후속 실제 native·분할 최종화는 세 환경 모두 완료했다.

| 환경 | 본수집 승인 / 검토 | 추가 보충 승인 | 최종 선택 | 실제 train / val |
|---|---:|---:|---:|---:|
| Sunny11 | 250 / 0 | 0 | 250 | 212 / 38 |
| Sunny13 | 249 / 1 | 1 | 250 | 212 / 38 |
| Sunny15 | 250 / 0 | 0 | 250 | 212 / 38 |
| 합계 | 749 / 1 | 1 | 750 | 636 / 114 |

원형 QA·충돌·실제 정지/P와 선택 원본을 검사했다. 전역 ID·raw root·payload는 각각
750개 고유하며 실제 raw class도 Obstacle 750개다. 두 거리 그룹은 실제 annotation 중심으로도
375/375다. LiDAR 0점인 camera-only 양성은 근거리 29 / 원거리 170개이며, 원형 positive와
실제 instance 근거를 통과한 행만 유지했다. 이것은 모델 AP 개선의 인증이 아니다.
각 환경의 실제 선택은 원격 `collections/sunny_supplement_completion_v1/<환경>/layout_split_final/`이다.
공통 보충 상한 20회 중 1회만 사용했고 과거 실패·본수집 상태는 보존했다.

원격 최종 인계는 `operations/native_final_handoff_1792531701399708389.json`,
SHA `f8ed14b53095a56d38a4762084e23eb588e2eedc7f0af93a79048fbd8a67df55`다.
root도 별도로 세 실제 ledger·class·거리·분할·전역 고유성을 읽어 대조했다.
전역 completion state SHA는 `1c624b1c384923ff600d20afd065207c06f2c7e4c7cf29c53440cf6bdfa40261`이다.
이 시점부터 native 운영권을 반환하고 별도 CPU 전달 작업자로 인계했다.
JPEG·압축·회수는 별도 검증 단계이며 원격 PNG 삭제는 허용하지 않는다.

### Q90/upright 전달본 생성 완료와 로컬 수령 준비

세 환경 모두 같은 Q90·native full-RPY upright enclosure·3클래스 규칙으로 변환했다.
250장 단위 admission만 별도 scope로 명시했으며 기존 11개 변환 helper와 센서 기준은
유지했다. **750장·JPEG 3,000개·Obstacle GT 750개·train 636 / val 114**가 남았고,
지원 component 누락/추가 0·원본 전후 SHA 동일·원격 PNG 삭제 0을 확인했다.
원격 `work/sunny-supplement-training-runtime-261021-v1/PIPELINE_COMPLETE.json` SHA는
`1acb5a24dc2026ab01da737ca7de8d54710fe26326c0b8672a1edce2e2efbe7e`다.

세 압축본 합계는 **480,708,691바이트**, 내부 파일은 **27,483개**다. 원격에서 압축본 전체
SHA와 압축 해제된 전 멤버의 크기·SHA를 검증했다. 복원 파일의 논리 크기 합계는
1,471,850,194바이트이며 파일·디렉터리의 4KiB 단위 할당 추정은 1,555,034,112바이트다.
로컬 공간이 빠듯하므로 압축본만 새 전용 RAM cache에 수령하고 검증된 dataset은
디스크에 복원하는 별도 수령기 v2를 준비했다. 이 시점은 원격 전달본 완료이며
로컬 회수 완료로 해석하지 않는다.

새 수령기는 디스크 전체 할당 추정 + 128MiB + **7GiB 잔여**, 실제 MemAvailable의
압축본 합계 + 작업 여유 2GiB + 잔여 4GiB를 시작 전에 요구한다. 실행 중에도 자원 하한을
검사한다. 기존 복원 core의 전체 SHA·전 멤버·750장 최종 감사를 그대로 사용하고,
모두 통과한 뒤에만 이번 RAM cache의 정확한 세 압축본을 지운다. 과거 archive나 원본을
삭제하지 않는다. 이 7GiB는 새 수령 scope의 예산이며, 이미 완료한 로컬 52장 batch parent의
8GiB 기준을 낮추거나 추가 native 촬영을 승인한 것이 아니다.

### 로컬 수령·복원·전체 감사 완료

검토한 수령기 v2 SHA `cf26fe4c71a8cd6fc3a5d92a980f16a86c0d61da090ee5bbc2fc82a05c1bb10b`로
실제 전송·세 환경 복원·최종 감사를 마쳤고 **exit 0**이다. 원형 복원 core의 archive 전체 SHA,
27,483개 멤버의 크기·SHA·등록 목록 검사와 portable 입력 전수 검증을 모두 통과했다.
최종 입력은 다음 경로다.

`artifacts/evaluation/base5_sunny_supplement_training_retrieved_261021_v1/<sunny11|sunny13|sunny15>/dataset/`

각 환경 **250장 / Q90 JPEG 1,000개 / Obstacle GT 250개 / train 212 / val 38**이며
합계는 **750 / 3,000 / 750 / train 636 / val 114**다. root도 실제 COMPLETE·manifest SHA·
PKL·JPEG 경로·원 의미 obstacle 750개와 전역 token·group 고유성을 별도로 읽어 확인했다.
독립 작업자는 recovery alias 12개까지 해석해 원 계획→최종 overlay→실제 PKL group/split이
750/750 일치하고 환경 간 train/val group 충돌도 0임을 확인했다.

전달 감사 `handoff/received_audit.json` SHA는
`31100d1fb194a8a1634fa62588dd2ba4e051c5377028aa5a489f999eb091a012`,
실행 완료 `handoff/receiver_v2_complete.json` SHA는
`e5cd50aea5afcf3f1b896971874dbe405cad7d1257a252bc534675be2a8c9590`이다.
전체 감사 성공 후 이번 전용 RAM cache의 세 압축본만 정확한 경로·SHA·소유권·하드링크 검사를
거쳐 삭제했고 cache 부재도 확인했다. **원격 archive·PNG·native GT는 삭제하지 않았다.**
완료 시 로컬 디스크 여유는 7,937,249,280바이트로 새 수령기의 7GiB 기준 이상이다.
이는 향후 두 장의 추가 촬영·5K JPEG 변환 용량을 보장하는 수치가 아니므로 다시 측정해야 한다.

거리 통계의 원점도 구분한다. 원형 annotation을 Ego 원점에서 집계하면 환경별 125/125지만,
학습 bbox는 LiDAR 원점이다. 기록된 `lidar_to_ego`는 전방 **0.855m**, 높이 1.388m 평행이동이며
이를 실제 bbox에 적용하면 환경별 20~30m 125 / 30~45m 125가 그대로 복원된다.
LiDAR 원점 기준 학습 집계는 **20m 미만 19 / 20~30m 357 / 30~40m 288 / 40~50m 86**이다.
실제 최솟값/최댓값은 약 19.241/45.392m다. 같은 정답의 좌표 원점 차이이며 GT를 옮기거나
거리 경계에 맞추어 다시 배치하지 않았다. LiDAR 0점·카메라 지원 양성은 총 199개로 유지된다.
root 독립 확인은 `handoff/root_received_check_v1.json`에 남겼다. 학습은 시작하지 않았다.

<a id="foggy11-weather-repair-261021"></a>

## 10/21 로컬 Foggy11 부족분과 후반 날씨 복구

본수집은 **승인 4,945 / 부족 55 / 실패 0 / error null**로 정상 종료했다. 보충 준비 중
공식 도구의 현재 날씨가 Sunny인 것을 발견했고 실제 RGB를 대조했다. 초기·중간 표본은
안개가 보였지만 후반 51장은 청색 하늘뿐 아니라 같은 도로권 원경 대비가 명확하게 달랐다.
원형 selected ledger와 native QA는 보존한다. annotation 좌표 오류가 아니라 실제 날씨와
의도한 환경의 불일치이며, native 합계 5,000만으로 순수 Foggy11 5K라고 인증하지 않는다.

독립 감사는 마지막 256장의 작은 단일 카메라 관측과 전환 양쪽 원본·51장 전체 시각 확인을
사용했다. 마지막 확인 Foggy `03329` 04:47:27.12와 첫 Sunny `03353` 04:47:35.55 사이
약 8.428초다. **확정 Sunny 51개와 보수적인 직전 경계 1개, 총 52개**를 별도로 구분했다.
하늘 B−R은 이 표본의 진단값이지 일반적인 날씨 분류 threshold나 전 프레임 환경 인증이 아니다.
Player.log에는 장면/연결 이력이 있지만 명시적인 weather 변경 기록이 없어 조작 주체나
정확한 변경 시각을 그 로그로 확정하지 않는다.

근거는 `artifacts/evaluation/base5_foggy11_weather_audit_261021/`의
`independent_report.json`, `definite_repair_source_ids.json`(51개),
`conservative_repair_source_ids.json`(52개)다. 과거 raw·선택·status·판정은 바꾸지 않는다.
실제 Foggy·11시 복원 후 기존 부족 55장을 별도 출력에서 보충하고, 52개 잘못된/경계 장면은
파생 Foggy 선택에서만 제외한 뒤 같은 자산·동작·거리·분할의 새 관측으로 채울 계획이다.
해당 52개는 교차로 실제 이동 계열이며 editor-ready·기존 두 waypoint 높이 lookup을
유지할 수 있는지 새 geometry에서 확인한다. 원본 payload 반복 차단·실패 예산을 무력화하지 않는다.

사용자는 원격 보강·부족분·JPEG 회수와 로컬 부족분·JPEG 검증·PNG 안전 삭제까지
끝까지 관측하며 진행하도록 추가 승인했다. 최종 올바른 Foggy 선택·분할·Q90/upright
입력 전수 검증 뒤에만 선택 RGB PNG를 삭제한다. 제외 날씨·실패 근거·native GT·mask/depth는
보존한다. 현재 이 절은 복구 진행 근거이며 순수 Foggy11 5K·JPEG·삭제 완료 기록이 아니다.

### 실제 부족분 53장 인계와 날씨 52장 완료

55개 일반 부족분 중 53개는 새 관측으로 승인됐다. 마지막 달리는 Pet1·Man4 두 장은 남았고,
Man4 세 번째 시도는 이전 빈 지면 probe와 동일한 payload를 다시 보내려다 **로드 전에 차단**됐다.
원형의 포괄 오류 문자열은 `Native safety/communication/Park child failed; no next load`지만,
실제 품질 report는 `Native payload already observed; no new MORAI load/capture`다.
해당 시도의 앞선 두 calibration은 실제 촬영됐고 각각 정지/P·충돌 확인을 통과했다.
차단된 near-probe와 최종 이동 본체는 session·capture가 시작되지 않았다.
전체 시도의 실제 촬영이 0회였다고 쓰지 않는다.

옛 `flexible_v1`의 `halted / exit 1 / selected_not_certified=true`는 그대로 보존했다.
별도 `nonrepeat_handoff_v1/QUALIFIED.json`에서 원래 4,945개 행의 내용·투영 순서를
보존하고 추가 53개만 원형 QA·evidence inventory·실제 child P 54개로 독립 재검증했다.
현재 source/runtime byte 경계는 실제 검사했지만, 과거 finally에는 별도 영수증이 없으므로
당시 통과 여부는 실행 흔적에 따른 추론으로만 구분했다. 옛 상태를 정상으로 재라벨하지 않았다.
인계 SHA는 `b06b8b210cf5b3cc6b56d8d4f939912d130edae75085ab0707a4c840861d2585`다.

이 인계와 최신 실제 P·정지·CollisionData를 확인한 뒤 **새 계약·새 출력**의 52장 scope를 실행했다.
기존 v1 실행기·미사용 계약은 보존하고 원형 memfd worker·native QA·중복 방지를 유지했다.
새 source 52개 모두 editor-ready·독립 지면 준비 0이며 walking 32 / running 20,
총 moving actor 62개다. 첫 8장 정상 PAUSE·검증 뒤 같은 출력에서 나머지를 이어갔다.
`weather_repair_capture_after_guard_v2`는 **52/52 승인·검토 0·실패 0·error null·exit 0**으로
완료했다. 실제 7개 세션의 최종 P·62 actor 동작·정확한 51+1 canonical 매핑과 source 경계를
확인했고, 공식 Tk 실제 텍스트·첫/중간/끝 RGB에서도 Foggy11을 확인했다.
과거 화면을 반환할 수 있는 GDK 캡처만으로 날씨를 인증하지 않았다.

완료 근거는 `weather_complete52_after_guard_verified_v2.json`,
SHA `794b5f7bdf7c631b3a7f447f7ed85068aee9e6dd7eabcb14bdf9ebe4ed1dfa11`이다.
현재 로컬 인계 `LOCAL_HANDOFF_AFTER52_v2.json` SHA는
`337ee15d51d04da909db33153b47ec18e18d8c05b9260ea75835b91dd1051efb`다.
원래 Sunny 51장·경계 1장·실패 raw는 보존한다. 날씨 교체는 수량을 52장 더하는 것이 아니므로
현재 올바른 관측은 **4,998장 / 일반 부족 2장**이다. 두 자산의 동일 동작 새 geometry는
CPU에서만 준비됐고 별도 추가 시험의 사용자 승인은 아직 없다. 추가 native와 최종 5K 조립·
분할·JPEG·PNG 삭제는 완료했다고 표현하지 않는다. 승인 검사·fixture와 별개로 실제
추가 native owner 구현·검토도 후속 단계다. **JPEG 0 / PNG 삭제 0**을 유지한다.

그 뒤 별도 CPU 준비에서 동일 자산·동작의 두 새 geometry와 1회 한정 owner를 구현·검토했다.
원래 각 3회·총 60회 이력은 보존하며, 실제 사용자 추가 승인과 검토한 owner SHA가 없으면
새 촬영도 시작할 수 없다. root 독립 owner/authority 검사 23개와 4,998+2 조립 검사 3개를
통과했지만 실제 추가 native는 0회다. 현재 snapshot을 정상 5K로 고친 것이 아니다.
`LOCAL_BUDGET_HANDOFF_NOTE_v3.json`에는 완료한 52장 parent의 8GiB와 기존 tail 경로를
구분했다. 향후 두 장의 촬영·JPEG 용량은 아직 계획 추정이며, 승인 후 실제 디스크 상태로
다시 판단해야 한다. 기존 계약·owner·과거 snapshot의 해시는 바꾸지 않았다.

### 사용자 추가 승인과 실제 촬영 전 중복 차단

사용자는 앞서 Sunny13으로 바꾸었다는 말이 **빌린 원격 PC에 대한 설정**이었다고 확인했다.
이를 로컬 Foggy11의 날씨 변경 원인이나 조작 주체의 증거로 사용하지 않는다.
정정된 두 장 질문에 **“두 장 모두 같은 조건으로 각 1회 추가 시험”**이라고 답했다.
실제 응답은 `ACTUAL_USER_REPLY_261021_v1.json` SHA
`989b833d17fc11f5755ff6a3831990ebc42411dbe734401291cadd68a7f371fa`에 남겼다.

같은 MORAI 프로세스·실제 Tk Foggy11/Connected/auto OFF·새 정지/P·수신을 확인하고,
disk 약 7.391GiB와 bounded 두 장·기존 변환 시작 4GiB 조건을 대조했다. 원래 총 60회·
각 3회 기록을 보존한 별도 승인 SHA는
`505489f6cd063c7d6b72fb81f046929a591a3caca721d44737efdcb0ff86099a`다.
검토한 e67 실행기로 Pet1 `00062`의 추가 논리 시험 1회를 시작했지만 첫 empty-terrain
준비에서 **`Native payload already observed`**가 발생했다. Man4 `00124`는 시작하지 않았다.

이번 새 report는 `attempts=[] / selected=[]`, 실제 session contract·native event·capture·
shutdown receipt도 모두 0이다. 별도 사전 정지/P 호출만 있었다. source 종료 경계 2회도
통과했다. 포괄 오류 문구가 다시 Park/통신 실패처럼 보이지만, **이번에 물리적인 P 장애나
통신 장애를 관측한 것은 아니다.** 새 `halted`·소진한 claim을 정상화하거나 자동 재시도하지 않았다.

root와 독립 작업자가 원문 바이트를 대조해 사전 검사의 누락 두 가지를 확인했다.

1. 새 Pet1 본 장면 SHA `79f28bbb…57bc3`는 최초 slot보다 Ego가 32.66m 옮겨졌지만,
   과거 baseline `replacement_1/proposal_00001/`의 **미촬영 실패 계획과 바이트까지 같았다.**
   기존 freshness 집합은 최초 deck·ledger·명시적 시도 중심이라 이 과거 계획을 놓쳤다.
2. 파생 첫 빈 지면 SHA `cb214598…b5251b`는 과거 `round_1/motion_terrain`의 실제
   accepted ledger·raw·최종 P가 있는 촬영분과 바이트까지 같았다. 본 장면 검사만으로
   파생 calibration의 신선도를 보장할 수 없는데 촬영 전 CPU 대조에서 이를 빠뜨렸다.

독립 CPU 검사에서 미실행 Man4 v1도 과거 `replacement_1/proposal_00004/`와 같은 본 장면이며,
첫 empty/static 준비 둘 다 이전 촬영분과 겹쳤다. 두 v1 후보를 그대로 다시 실행하지 않는다.
이 결함은 CPU 후보 선정·진입 검사의 누락이며, 기존 native 중복 guard는 정확히 차단했다.
이미 승인된 4,998개 관측과 새 날씨 52개의 판정·GT를 바꾸는 사유가 아니다.

root 독립 진단은 `TAIL_ZERO_LOAD_DIAGNOSIS_v1.json` SHA
`a87c2798624e3a9072c1171ccb15138191a1b621191e67deebc97b70b9f081e9`다.
추가 scope의 `TERMINAL.json` SHA는
`c43354aaa2a220ab9a54e5ebc13ce455a5454956e83d893b5bed47a32086018f`다.
별도 CPU v2에서는 과거 manifest 계획·ledger 선언·실제 raw 관측을 구분하고, 본 장면과
원형 empty/static·가능한 near witness를 사전 대조한다. ledger SHA 선언만으로 실제 촬영
완료라고 단정하지 않으며 보수적 중복 금지 집합은 줄이지 않는다. 이 CPU 준비는 새 촬영
승인이나 5K 완료가 아니다. 추가 실제 촬영 범위는 사용자에게 다시 확인 중이며 **JPEG 0 / PNG 삭제 0**이다.

### 마감 우선 정책과 마지막 두 장 실제 승인

위 절은 당시 중단 이력이다. 이후 사용자는 검증 실험을 더 늘리기보다 수집을 빨리 마치고
학습을 준비하도록 우선순위를 바꿨고, **정규 6환경 약 30K + 장애물 보강 750장 전체 학습 준비**를
명시적으로 승인했다. 마지막 두 장은 어려운 달리기 자산을 계속 재시도하지 않고, 같은 음영
패밀리·기존 split/group의 일반 정적 보행자 장면으로 마감했다. GT 라벨이나 실패 판정을
바꾼 것이 아니라 새 장면 두 개를 실제 촬영한 것이다.

`terminal_easy_v2`에서 Woman1과 Woman3가 각각 실제 촬영 1회에 승인됐다. 각 실행의
검토 0·오류 없음·원형 native QA·최종 정지/P·입력 경계 검사를 확인했다.
Woman1은 약 28.32m/LiDAR 4점, Woman3는 약 29.85m/LiDAR 6점이다. 옛 Pet1 claim과
`halted`·실패 예산은 그대로 보존했고, 새 정적 보행자 성공을 동물 달리기 성공으로 집계하지 않는다.
새 범위의 실제 사용자 지시와 권한은 `USER_FAST_COMPLETION_PLAN_261021_v1.json` 및
`terminal_easy_v2/user_authority.json`에 남겼다.

`terminal_easy_v2/pre_weather_native_5000_v1`은 **기존 4,998행 + 새 2행 = 실제 5,000행**으로
조립을 마쳤다. 원래 4,998행의 내용과 순서 투영을 유지했고 ID·source ID·raw 경로·scenario SHA의
고유성을 독립 확인했다. 조립 receipt SHA는
`5ac331c6319b2e0b9c0133c53fe82cd3789ac9d2cb7ea7dd60abfb054d837b0b`다.
이 단계의 선택에는 교체 전 날씨 52행이 있으므로 순수 Foggy11 전달본이라고 인증하지 않는다.
이미 완료한 올바른 Foggy11 관측 52개를 치환한 최종 split과 기존 Q90/upright 변환을 이어간다.
추가 배치 연구·재촬영·검사층은 늘리지 않는다.

전체 학습 준비는 새 data/work/run identity로 분리한다. 지원 GT가 없는 Foggy13 한 장은
제외 상태를 유지하므로 Foggy11 5,000장 전달이 완료되면 실제 학습 입력은 **30,749장**이다.
일반 코사인·compile OFF·batch 4 및 기존 호환 nuScenes 초기화가 준비 후보이며, 기존 E24나
완료된 E8 optimizer를 변경된 전체 데이터에 강제로 resume하지 않는다. Epoch 확정과 실제
학습 시작은 수집·JPEG 완료와 구분한다. Vast 재접속과 기존 설치 보존은 확인했으며 새 GPU 대여나
CUDA 재설치를 하지 않고 준비한다.

### Foggy11 최종 날씨 치환·분할 완료

`weather_corrected_5000_v2`에서 **순수 Foggy11 5,000장 / 부족 0 / 오류 없음**과 실제 분할
**train 4,250 / val 750**을 확정했다. 원래 선택 중 4,948행은 내용 그대로 보존하고
Sunny 확정 51장·경계 1장만 이미 완료된 새 Foggy 관측 52개로 치환했다. 별도 독립 검사에서
canonical 순서·ID·source ID·raw 경로·scenario SHA의 고유성과 group 분할 충돌 0을 확인했다.

최종 입력은 `weather_corrected_5000_v2/layout_split_final/`의 세 파일이다.

- `manifest.json`: SHA `0016b00b5bb0fffd6108e850fbff5fc259033e29e84ee2e54be93891d547106e`
- `derived_selected_ledger.jsonl`: SHA `5b398d53a6a100c7367bea05053d3ef84c7f9aec00bc12607892736da9185437`
- `split_overlay.json`: SHA `8004f699b20a139483a242587b12c4eed8406332b6f610badb933bae815a0ddb`

정리 도중 발견한 두 CPU 경로 문제도 촬영 문제와 구분한다. 완료된 52장을 읽기 전용으로
검증할 때 새 촬영용 8GiB 공간 예측이 적용되는 경로는, subprocess 금지·기존 `_verification`
경로·실제 완료 52장에 한정한 별도 adapter로 분리했다. 원형 native 파일·8GiB 기준·옛 계약은
바꾸지 않았다. 또 정당한 배우 없는 배경 장면에 `runtime_actors[0]`을 읽던 정리 코드 한 곳은
배우가 없으면 원래 spec의 motion/static을 쓰도록 보완했다. 배경에 객체 GT를 만든 것이 아니다.
중간 실패 출력 v1은 보존하고 성공한 v2만 최종 전달에 사용한다.

같은 Sunny15 Q90/upright 3클래스 변환을 `export_weather_verified_v2.py`로 시작했다.
이 wrapper는 원형 변환 정책을 바꾸지 않고 최종 선택 경로만 v2에 연결한다. JPEG 입력 전수
확인·선택 RGB PNG 정리는 아래 실제 완료 근거가 생기기 전까지 진행 중으로 구분한다.

### Foggy11 Q90/upright 5K 전달본 완료

최종 host 실행은 **exit 0·5,000장·train 4,250 / val 750·GT 11,641개·Q90 JPEG 20,000개**로
완료했다. 실제 512×1056 resize/crop과 원형 native→upright→3클래스 연결을 전수 확인했고,
retained component 누락/추가 0·보조파일 무손실 보존·원본 전후 동일을 확인했다.
원형 파이프라인 수행 시간은 약 1,128.24초(18.8분)다. 배우 없는 정상 배경은 배경으로 유지했다.

최종 학습 입력은
`artifacts/evaluation/base5_foggy11_training_export_261021_v3/qualified/foggy11/training3_jpeg_portable/`이다.
manifest SHA는 `f0c7c7bab2b92ba191494f2cf6c92df30073504a5f1c250b8889b4cf5a0510e9`,
상위 `qualified/report.json` SHA는
`6d0da443c2483c7be136a26a0b5fb79cb8430d24ac1f655e631cb4cad9f8aed0`이다.
중간 v1/v2 출력은 실패·실행환경 진단으로 보존하며 학습에 쓰지 않는다.

중간 재실행 사유는 다음과 같다. 둘 다 새 데이터 수집이나 GT를 고치는 사유는 아니었다.

- `01967` WoodBox의 저장 audit 143점과 재검증 142점 차이는 float32→기본 float64로 바뀐
  연산 경계의 1점 차이였다. 원래 native parser와 같은 center/size dtype으로 재생하자 정확히
  143점이 됐다. 회전식·1e-5m 허용오차·저장 audit·GT·점수 조건을 바꾸거나 1점 허용을 추가하지 않았다.
  원형 corner-derived size의 float64 분기도 유지했다. 이 계산 경로는 최종 report에 명시했다.
- 앞선 실행은 격리된 filesystem에서 외부 raw와 workspace 사이 하드링크가 `EXDEV`로 실패했다.
  이는 실행환경 선택 오류였다. 실제 host의 같은 filesystem에서 원형 hardlink 방식으로 다시 실행했다.
  RGB/원본을 새로 복사하거나 저장된 native 판정을 바꾸지 않았다.

이제 정규 지원 29,999장과 별도 장애물 750장의 **전체 30,749장**이 로컬에 준비됐다.
9개 cohort의 실제 infos/manifest 논리 검사에서 **train 26,135 / val 4,614 / test 0**, 전역
token/raw/payload 중복 0·group 분할 충돌 0·공통 Q90/upright/Animal 매핑을 확인했다.
이는 픽셀을 다시 검사하거나 원격 전달까지 완료했다는 뜻이 아니다. 전체 학습은 별도 12-epoch
준비 계약이며 아직 시작하지 않았다. 선택 RGB PNG 정리는 receipt-bound 계획을 확인한 뒤 수행한다.

### Foggy11 선택 RGB PNG 정리 완료

JPEG 전달본 완료 뒤 사용자 승인 범위의 RGB PNG 이름 **40,000개**를 정리했고, 원형 작업자는
exit 0으로 종료했다. 20,000개 실제 RGB 파일을 native 원본과 중간 portable 경로의 하드링크
두 이름에서 지웠으므로, 논리 삭제량 27,314,894,664바이트를 실공간 확보량으로 중복 집계하지 않는다.
회수 가능한 고유 inode 용량은 **13,657,447,332바이트(약 12.72GiB)**였고, 완료 직후 실제
파일시스템 여유는 약 **16.91GiB**였다.

정확한 계획 SHA는 `201509fa7860af3a80e2786f1a59e9cd840de77500f3dcc05c2ad33d44a93244`다.
독립 범위 확인과 원형의 삭제 직전 재검사를 통과한 뒤 삭제했고, **보조 파일 335,705개·Q90 JPEG
20,000개·native GT·실패 원본·잘못된 날씨 52장 원본은 보존**했다. 과거 계약과 검증 기록은
수정하지 않았다. Q90은 손실 변환이므로 삭제한 원본 RGB의 무손실 백업이라고 표현하지 않는다.
PNG를 요구하는 과거 원본 재검사는 삭제 뒤 그대로 재생되지 않을 수 있다. 빌린 PC의 PNG는
이번 정리 범위가 아니며 보존한다.

완료 receipt는 `base5_foggy11_training_export_261021_v3/rgb_prune_no_archive_receipt_v1/COMPLETE.json`,
SHA는 `3d9ec48a25d704c8fdf24065ff5f03364df2c51886c83a5d9dfae1c8fafb070b`다.
이후 같은 JPEG 전달본을 원형 level 6·threads 2 압축기로 묶어 승인된 Drive 경유로 Vast에 전달한다.
압축·전송 완료는 해당 archive와 원격 복원 receipt가 생긴 뒤 별도로 기록한다.

### Foggy11 압축본·본인 Drive 게시 완료

위 정리 후 `artifacts/handoff/base5_foggy11_5k_jpeg90_upright3_261021.tar.zst`를 만들었다.
**2,875,912,188바이트·전체 187,865개 멤버 검증·source package 불변·exit 0**이다.
archive SHA는 `54e232169a1274eb6c1db05c5f9a4a632c2f56c1a83f928f032ebad7e8e91fcd`다.
기존 packer와 동일한 Q90/upright/3클래스 입력이며 새 native 재검증을 끼워 넣지 않았다.

사용자가 승인한 본인 Drive `dataset/Base5/full30749_261021/foggy11/`에 압축본과
`COMPLETE.json`·`intent.json`·`archive_manifest.json`·`SHA256SUMS`를 게시했다.
다섯 파일의 크기·로컬 SHA·Drive 서버 MD5가 일치했고 기존 Drive 파일은 덮어쓰거나 지우지 않았다.
갱신 가능한 인증정보도 원격에 공유하지 않았다. 게시 후 로컬 실제 여유는 약 14.19GiB다.

최종 Drive receipt는 `base5_full_data_training_delivery_261021_v1/foggy11/DELIVERED.json`,
SHA는 `168f9d1cd8fe930d5b3da8c02caa95d285d1086182cfd47a969c540e06789b20`이다.
이 receipt는 Drive 게시 완료의 근거다. Vast 전체 SHA·복원·학습 시작의 완료 근거로 대신 쓰지 않는다.

### 전체 30,749장 Vast 전달·복원 완료

이후 Sunny11·Foggy11/13/15와 Sunny 보강 3개 cohort의 **압축본 전체 SHA·원형 전체 멤버 복원**도
Vast에서 완료했다. 기존 새 Sunny13·Sunny15 두 cohort는 실제 manifest/infos SHA를 확인해 재사용했다.
최종 9개 입력은 **30,749장 / train 26,135 / val 4,614 / test 0**이며, 따로 모아 복제한
aggregate payload는 0개다. 각 원래 root·infos·GT·split을 그대로 연결한다.

`base5_full_data_training_draft_261021_v1/TARGET_READY.json`은 실제 target 9개 manifest/infos와
새 복원 7개의 COMPLETE·검토된 실행 코드·백업 경로 receipt를 대조한 완료 기록이다.
cohort index SHA는 `6605a203d440811293a2bc6e468b2bb6b3677617881c277cca8ef77bad81a097`,
게시 직후 Vast 여유는 **29,112,795,136바이트(약 27.11GiB)**였다.

Foggy13의 첫 HTTP 수신은 socket timeout으로 중단됐으나 실패 파일과 기록을 보존했다.
기수신 1,241,513,984바이트를 구간 해시로 확인해 재사용하고 빠진 1,646,350,684바이트만 받아
최종 전체 SHA까지 통과했다. 새 수신 scope의 socket 대기를 30초로 분리했으며 원형의
byte-range·길이·전체 SHA·덮어쓰기 방지는 유지했다. Drive와 Vast가 완전한 압축본을 보유한 뒤
자체 Foggy13 RAM 임시 압축본만 정리했고 원본 JPEG·영구 receipt·실패 근거는 보존했다.

이것은 데이터 전달 완료다. 12-epoch 실행 설정과 W&B·Drive 사전 검사, 실제 학습 시작은
[모델 학습 준비 기록](../../../docs/bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#full-data-12epoch-preparation-261021)에서 별도로 관리한다.

<a id="foggy13-terminal-one-30750-261021"></a>

## Foggy13 새 정적 장애물 1장으로 지원 입력 30,750장 마감 — 2026-10-21

사용자가 정상 장면 한 장을 새로 촬영해 전체 입력을 30,750장으로 채우도록 승인했다.
원래 `foggy13_sunny13_sunny11_b5v2_00001` Bollard04의 native 승인·GT·RGB·제외 판정과
위 30,749장 검증·전달 기록은 그대로 보존했다. 지원 GT가 없던 원래 장면을 양성 GT로
바꾸거나, 기존 4,999장 전달본을 덮어쓴 것이 아니다.

새 작업 범위는 `artifacts/evaluation/base5_foggy13_terminal_one_261021_v1/` 하나다.
이미 검증한 공통 `base5_spatial_diversity.build`와 원형 layout split 준비기를 재사용해
실제로 **count 1인 독립 source**를 만들었다. 5K 전체를 재생성하거나 재수집하지 않았다.
새 `source/generated_v1/placement/manifest.json`과 `source/CPU_READY.json`은 CPU 준비 근거이고,
실제 완료 근거는 같은 작업 범위의 `NATIVE_AND_EXPORT_COMPLETE.json`이다.

| 항목 | 실제 확인 결과 |
|---|---|
| 새 token / 별도 cohort | `foggy13_terminal261021_00001` / `foggy13_completion` |
| 원형 자산·배치 | WoodBox, DataID `40100002`, scale `(1,1,1)`, 정적, 전방 약 13.727m |
| 날씨·안전 | 공식 도구 전후 Foggy 13:00·Connected·자동 순환 OFF, native 승인 1 / 검토·실패 0, 종료 P·gear 1·errors 없음 |
| 실제 센서·학습 근거 | LiDAR 52점, CAMERA_1 Instance 6,750px, CAMERA_1~4와 LiDAR 5 Instance 저장, 유효 Obstacle GT 1개 |
| 학습 전달본 | 원형 Q90/upright 3클래스 변환, JPEG 4개, train 1 / val 0 / test 0, retained component 누락·추가 0 |
| 새 분할 그룹 | `layout:50d080d80ce89fc6af34909b`; 기존 9개 cohort와 token·raw·scenario payload·group 중복 0 |
| 원본 보존 | 새 RGB PNG 삭제 0개, 기존 native·GT·selected ledger·30,749장 계약 수정 0 |

실제 native 선택은 `native_capture_v1/selected_ledger.jsonl`이며 SHA는
`ffad255ec9e6e169cfcd1603ffa8ddfef3fc9e1613360608785c7b3225e4a313`이다.
원형 QA·충돌·정지·P 조건을 완화하지 않았고, 독립 최소 검사에서도 실제 raw audit·source payload·
종료 P와 최종 infos의 GT 1개를 대조했다. 기존 9개와 공통 metainfo가 일치하며,
허용한 차이는 원래와 같이 `layout_split_overlay_sha256` 출처 하나뿐이다.
기존 5K들의 RGB·native QA를 다시 전수 실행한 검사는 아니다.

새 학습 입력은 작업 범위 아래
`training_export_v1/qualified/foggy13_completion/training3_jpeg_portable/`이다.

- 입력 manifest SHA: `b49241759a2116196da9e247ceea1c55cd9c5403b1b999f0ce5c26a355915b2c`
- `training_export_v1/qualified/report.json` SHA: `bdc427f01ccf2b00541a1849ac4e9cc33e82e7d5f8feb996d23d4c1dc0415649`
- `NATIVE_AND_EXPORT_COMPLETE.json` SHA: `a1394ad2858a70ea4564664e5c4f18dbe2f341736c5b0dc0b73c2cc018da91a6`

같은 원형 packer로
`artifacts/handoff/base5_foggy13_completion_1_jpeg90_upright3_261021.tar.zst`를 만들었다.
**519,600바이트·51개 전체 멤버 검증·source package 불변**이며 SHA는
`c7e88a49ee65e2758e60d071bc1d7813d5ef75569dccba85019b601b9e1486fe`다.
`archive_receipt/COMPLETE.json` SHA는
`6cd013fd8f7d32b020cf951faa134dbbbf79f5c7209d118535632ff8ae1b3896`이다.
새 1장만 별도 전달·Vast 복원을 완료했고 기존 9개 입력의 root·infos·GT·분할은 유지했다.

따라서 Foggy13 지원 입력은 **기존 4,999장 + 새 독립 train 1장 = 5,000장 / train 4,250 / val 750**이다.
전체는 **10개 cohort·30,750장 / train 26,136 / val 4,614 / test 0**이며, 원래 GT 0인 한 장을
다시 더하거나 원래 5K native 승인분을 중복 합산하지 않는다. 새 target 대조 receipt는
`artifacts/evaluation/base5_full30750_training_261021_v1/TARGET_READY.json`이고,
10개 cohort index SHA는 `481008024038779fd6eaf8e6bb2676461ccd201081ad0f8cb762718399d42d24`다.

이는 데이터 완료 기록이지 모델 성능 향상이나 실제 학습 실행의 인증은 아니다.
**12 epoch 학습 준비만 승인됐고, 실제 학습 시작 승인은 없으며 학습도 시작하지 않았다.**
Vast의 새 전체 설정 `--prepare`·온라인 `--check`와 실제 Concat CPU 구성도 통과했다.
train 26,136 / val 4,614·19개 child·원래 경로 95개를 확인했으며 모델 학습은 실행하지 않았다.
최종 준비 receipt는 `artifacts/evaluation/base5_full30750_training_261021_v1/FINAL_PREPARATION.json`이다.
준비 계약·실패한 연결 시도 보존·실행 방법은
[전체 30,750장 학습 준비](../../../docs/bevfusion/261009_BASE5_VAST_4090_TRAINING_AND_INFERENCE_PLAN.md#full-data-30750-preparation-261021)에서 별도로 관리한다.
