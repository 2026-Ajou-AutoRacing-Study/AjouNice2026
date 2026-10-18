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
본수집 5K에 더하지 않는다. 완료 재개 검사는 통과했지만 입력/최종 자격은 아직 미통과다.

해당 canonical `...00674`는 고주로 뒤차 10~30m 장면이다. 원본 regular ledger의
`population_visible_actor_deficit` 검토를 보존하고, ordinal 2의 실제 승인 배치
`...00674_replace_replace`로 회복했다. source payload `e1337c91...`와 승인 payload
`74cc668d...`가 다르므로 같은 source의 exact 관측으로 셀 수 없다. 입력 qualifier의 첫 overlay
자동선택과도 충돌해 `Captured split overlay content differs`로 중단됐다.
이는 GT 손상 증거가 아니라 아직 authoritative source로 채택하지 않은 정상 복구 계보 문제다.
31+1 overlay를 임시 합치거나 32장을 전체 5K native 선택으로 위장해 finalize하지 않는다.
이미 지원하는 depth 2 실제 native adoption으로 source를 새로 조립한 뒤 새 등록·촬영으로 검증한다.
기존 source·등록·원본·입력 변환 실패 출력과 로그는 그대로 보존한다.

## Sunny 추가 보강 초안

Sunny 15K 완료 후 추가 보강은 별도 초안으로 남겼다. 잠정 예산은 750장으로,
11·13·15시 각 250장이다. 각 환경은 장애물 20~30m 180장과 큰 장애물 30~45m 70장을
검토한다. 추가 750장은 matched E24·공통 val·자산별 관측 근거를 확인한 뒤 결정한다.
현재 초안은 기존 Sunny 자료와 Foggy 수집을 바꾸지 않으며, 실제 보강 캡처를 시작하지 않았다.
149장을 재할당하는 현재 Foggy 프로파일을 750장 추가 수집 실행기로 사용하지 않는다.

`generator_review/sunny15k_supplement_draft_v1.json`에 분석 근거 7개의 경로·SHA를 보존했고
작성 시 모두 재검산했다. 수량은 출제 확률이나 성능 개선을 보장하는 확정 정책이 아니다.
얇은 객체·작은 동물 표본·전방 원거리 차량을 낮은 AP만 보고 일괄 증량하지 않는다.
