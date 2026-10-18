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

## Sunny 추가 보강 초안

Sunny 15K 완료 후 추가 보강은 별도 초안으로 남겼다. 잠정 예산은 750장으로,
11·13·15시 각 250장이다. 각 환경은 장애물 20~30m 180장과 큰 장애물 30~45m 70장을
검토한다. 추가 750장은 matched E24·공통 val·자산별 관측 근거를 확인한 뒤 결정한다.
현재 초안은 기존 Sunny 자료와 Foggy 수집을 바꾸지 않으며, 실제 보강 캡처를 시작하지 않았다.
149장을 재할당하는 현재 Foggy 프로파일을 750장 추가 수집 실행기로 사용하지 않는다.

`generator_review/sunny15k_supplement_draft_v1.json`에 분석 근거 7개의 경로·SHA를 보존했고
작성 시 모두 재검산했다. 수량은 출제 확률이나 성능 개선을 보장하는 확정 정책이 아니다.
얇은 객체·작은 동물 표본·전방 원거리 차량을 낮은 AP만 보고 일괄 증량하지 않는다.
