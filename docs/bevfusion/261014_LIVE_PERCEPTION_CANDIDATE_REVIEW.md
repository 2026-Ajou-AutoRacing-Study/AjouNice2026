# 2026-10-14 인지 후보 live 검토

상태: EVIDENCE — 1차 검토·반영, 2026-10-14. 보행자 순위 적용과 저속 이동의 후속은
[연결·이동 비교](261014_PEDESTRIAN_ASSOCIATION_AND_LIDAR_MOTION_FOLLOWUP.md)에 기록했다.
현재 운영 계약은 [CURRENT](CURRENT.md)를 따른다.
이 문서는 [오프라인 검토](261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md)의 미적용 후보를
새 MORAI 입력으로 확인한 근거다. 단순히 옵션을 구현한 것을 승격으로 해석하지 않는다.

## 범위와 보존

MORAI K-City·Sunny13·`Sensor_competition_local`과 `ego_network_validator_local`을 사용한다.
사용자가 켠 MORAI와 ROS master는 보존하고 시험 실행기가 만든 프로세스만 종료한다.
기존 Sunny13 5K, v150 release, 가중치, 학습 계약, validator 소스와 planning 정책은 수정하지 않는다.
대량 수집·재학습·유료 인스턴스 작업은 하지 않는다.

GPU 대조는 같은 E24 SHA256 `84c7a849406a37f05250f872d4d0dfa3ebcffdb66b48e47bdfcb3209efecc746`,
AMP·camera 병렬화 ON·compile OFF다. 원본 JPEG와 ROS XYZI를 고정하고 각 조건에서
독립적인 전체 forward를 수행한다. MORAI가 동시에 실행된 로컬 5060 Ti 비용은
4090 실전 latency나 24GB·LAN·풀스택 인증이 아니다.

## 실행 경로

| 단계 | 실행기 | 근거 경로 |
|---|---|---|
| 정지·보행 입력 | `scripts/rosbag/record/run_perception_candidate_probe.py` | `planner_logs/analysis/261014_live_candidate_probe_v6` |
| 두 사람 보완 | 같은 실행기의 `--scene pair_follow` | `planner_logs/analysis/261014_live_pair_follow_v7` |
| 근거리 박스 | 같은 실행기의 `--scene near_cargo` | `planner_logs/analysis/261014_live_near_cargo_v1` |
| 고정 입력 | `scripts/evaluation/experiment_live_perception_candidates.py prepare` | `planner_logs/analysis/261014_live_candidate_inputs_v3` |
| 물리 yaw 기준 GT | 같은 prepare 실행기 | `planner_logs/analysis/261014_live_candidate_inputs_physical_v4` |
| 반복 GPU 대조 | 같은 실행기의 `infer` | `planner_logs/analysis/261014_live_candidate_inference_v1` |
| CPU 투영 | `scripts/evaluation/experiment_base5_depth_raster.py --projection-cpu-only` | `planner_logs/analysis/261014_live_projection_cpu_v1` |
| EKF·GT 대조 | `scripts/evaluation/analyze_live_perception_candidates.py` | `planner_logs/analysis/261014_live_candidate_analysis_v1` |
| 실제 ROS 원점 | `scripts/rosbag/record/run_live_box_origin_gate.py` | `planner_logs/analysis/261014_live_box_origin_gate_v2` |
| 원점·단계 분석 | `scripts/evaluation/analyze_live_box_origin_gate.py` | `planner_logs/analysis/261014_live_box_origin_analysis_v2.json` |
| 실제 서버 반복 | `scripts/evaluation/check_live_candidate_server_parity.py` | `planner_logs/analysis/261014_live_actual_server_parity_v2.json` |

시나리오 사본은 짧은 hash 이름으로만 설치한다. MORAI ScenarioLoad의 파일 이름 필드는
30바이트다. 처음 긴 이름이 거부된 v2, 설정 잔류를 감지한 v3, 중단한 v4와 미실행 v5는
실패 근거로 보존하며 정상 비교에 합산하지 않는다.

## 실제 패킷 묶음

각 32초 녹화에서 실제 VelodyneScan·PointCloud·네 카메라·Ego·Object를 확인했다.
51/50/회전 경계는 sensor RPM·point intensity·stamp 정책을 바꾸지 않는다.

| 장면·실제 정책 | LiDAR header→PointCloud 수신 평균 | p95 | 실제 패킷 수 |
|---|---:|---:|---:|
| 정적·51 | 83.69ms | 95.85ms | 51 |
| 정적·50 | 44.03ms | 48.98ms | 50 |
| 정적·cut0 | 44.40ms | 48.49ms | 50 |
| Man1·51 | 84.29ms | 94.73ms | 51 |
| Man1·50 | 40.59ms | 46.81ms | 50 |
| Man1·cut0 | 39.63ms | 46.19ms | 50 |

이 차이는 GPU 추론 시간이 아니다. 첫 패킷 host-read midpoint와 실제 source 묶음,
서로 다른 녹화의 simulator 부하를 구분한다. 정확한 ray acquisition time이나 LAN 시계
동기화를 인증하지 않는다. 정적 세 녹화의 발행률은 각각 약 11.92/11.54/11.45Hz로,
묶음을 줄인 것만으로 발행률 향상을 주장할 수 없다.

### 발견하여 수정한 재시작 오류

`npackets:=50` 시험 뒤 기본 launch로 돌아가도 `/lidar/lidar_driver/npackets=50`이 남았다.
기본 launch는 이 파라미터를 명시하지 않으므로 드라이버가 잔류 값을 사용했다.
v3의 Man1 "fixed51" 녹화는 실제 50개여서 검증기가 중단했으며 비교에서 제외한다.

`lidar.launch`의 `lidar_driver`에만 `clear_params="true"`를 적용했다. manager와 transform
namespace는 지우지 않는다. 기본 resolved parameter는 그대로이고 드라이버의 올림 계산을
덮어쓰지 않는다. source와 이 PC의 installed launch를 동일하게 갱신했고,
순수 launch 검사 9개 및 실제 51→50→cut0→51 전환을 통과했다.

### 주행 대조의 제한

`run_lidar_live_policy_campaign.py`는 기존 validator의 load·제동·실패 기록 경로를 사용한다.
시험 프로세스 안에서만 driver launch 인자를 전달하며 validator 파일을 변경하지 않는다.
첫 고주로 기준은 GT detection에서도 TIMEOUT/MID_DRIVE_STOP으로 끝났다. 이 실패를
새 LiDAR 정책의 검출 성능이나 실제 E24 폐루프 결과로 부르지 않는다.
`planner_logs/analysis/261014_live_driver_moving_v1`에 보존했다. 물리 yaw를 맞춘
`261014_live_driver_moving_v2`도 같은 기준 고주로에서 TIMEOUT으로 끝났다.
yaw correction만으로 원인을 해결했다고 주장하지 않는다. 해당 원본·실패 기록은 보존하고
50/cut0의 실제 폐루프 승격 근거에서 제외한다.

## GT·배치 확인

처음 같은 방향 보행 두 명을 0.8m 간격으로 로드한 v6 장면은 Object 토픽에 UID301만
나타났다. 두 명 검증 성공으로 처리하지 않았다. 시작 간격을 1.6m로 넓힌 v7은
UID301/302를 모두 수신했다. 보행 경로는 이전 수동 높이 lookup의 두 끝점과 허용된
평행 offset만 사용한다. 새 생성기 정책이나 범용 객체 간격을 변경한 것은 아니다.
이후 실행기는 로드된 pedestrian UID가 요청과 다르면 녹화 전에 거부한다.

반대 방향 두 사람은 모두 수신했다. 반려동물 동반 사람의 raw Object는 UID301 하나지만,
실제 학습 GT는 여러 component일 수 있다. 기존 catalog에는 `Ped_Pet1`의 완전한 component
기하가 없으므로 해당 장면은 GT 대응·FP/FN에서 제외하고 반복성·출력·ID 진단만 한다.
카탈로그의 `semantic_class=null`을 `.lower()`로 처리하던 분석 오류도 수정했다.
이 자산을 임의의 hard negative나 일반 사람 한 박스로 라벨링하지 않는다.

### 물리 yaw와 기록된 기본 GT TF의 구분

이번 정지 Ego·Object heading은 저장한 시나리오 yaw보다 약 1.347° 크다.
초기 v3 분석 입력의 correction=0은 물리 sensor GT 비교에 적합하지 않았다.
v4는 correction=-1.347°를 적용하며 각 시나리오의 정지 yaw와 잔차 0.02° 이내인지 검사한다.
3,647개 token의 LiDAR stamp·점 payload SHA·JPEG SHA는 v3와 정확히 같다.
가중치·원본·이미 실행 중인 GPU 예측을 바꾸지 않고 GT만 별도 파생 대조한다.

기록된 기본 `maptf_morai` TF를 정확한 물리 yaw의 oracle로 가정하지 않는다.
후속 시험용 TF는 `localization_morai_gt.launch use_heading_correction:=true`를 명시한다.
실제 production localization·시간 정책·heading 보정값은 변경하지 않는다.

## Depth hint 반복성

10개 입력 case의 3,647개 연속 프레임을 보존한다. GT 평가의 상태 보간과 camera ±50ms
조건을 만족하지 않는 가장자리·입력은 score를 보기 전에 제외했다.
각 입력에서 legacy A/B와 nearest A/B의 전체 forward를 비교한다.
센서에 안 보이는 객체를 camera-only GT에서 임의로 삭제하지 않는다.

별도 CPU 투영 160프레임에서 서로 다른 깊이가 같은 픽셀에 겹친 개수는 카메라당 평균
정적51 3.39개, 정적50/cut0 1.00개, Man1 51/50/cut0 2.36/0.48/0.50개였다.
이것은 실제 입력의 픽셀 충돌 근거이며 GPU 비결정성 자체의 증명은 아니다.
전체 forward 14,588회를 실행했다. legacy A/A는 raw 출력 1,347/3,647장,
운영 후처리 출력 1,296장에서 달랐다. nearest A/A는 두 비교 모두 0장이며 정확히 같다.
legacy→nearest 대조의 운영 박스 탈락/2m 초과 이동은 0개, 추가는 1개였다.
catalog GT 1m 대응은 모든 case에서 유지됐고 정적 cut0의 Obstacle만 333→334였다.
이는 반복성 보완이지 AP 상승이나 새로운 학습 성능의 인증이 아니다.

실제 공용 Docker 이미지·TCP 서버에서도 10 case×16장=160개 동일 입력을 각각 두 번
실행하여 A/A raw·운영 출력 차이가 모두 0임을 확인했다. source-bootstrap 경로와는
실수 연산 차이가 남았다. 실제 서버와 저장 예측의 운영 박스 198개는 모두 대응됐으며
추가·탈락은 0개였다. 좌표 차이 최대 0.40mm, score 최대 0.00188,
축 방향 차이 최대 1.15°다. 서로 다른 실행 경로가 bitwise 같다고 주장하지 않는다.

## 보행자 association 후보

nearest 고정 출력으로 원래 C++ 코어와 pedestrian-only Gaussian XY ranking 복사본을
비교했다. gate·birth·coasting·class·threshold는 같고 source tracker는 바꾸지 않았다.

| 실제 장면 | confirmed 사람 ID 수 | ID 교체 후보 | 명확한 한 GT 근처 다중 UUID | XY 오차 평균 |
|---|---:|---:|---:|---:|
| Man1 단독·51 | 2→1 | 0→0 | 6→0 | 0.16064→0.16069m |
| 같은 방향 두 사람·1.6m 간격 | 5→3 | 5→0 | 44→19 | 0.19190→0.19261m |
| 반대 방향 두 사람·0.8m 평행 간격 | 5→4 | 2→0 | 33→17 | 0.14326→0.14356m |

두 사람 대응 관측은 각각 750/742개로 동일하다. 첫 장면의 XY p95는 두 조건 모두
0.27679m, 반대 방향은 0.22088→0.22340m다. 해당 명확한 사람 GT에서 한 UUID가 여러
GT를 바꾸어 맡는 후보는 0이었다. 모든 사람이 하나로 합쳐진 개선은 아니지만 중복도
완전히 사라지지 않았다.

반려동물 동반 장면의 작은 높이 박스가 존재한 프레임은 238→224/374장으로 줄었다.
작은 박스 중복은 28→10장, 큰 박스 중복은 138→0장으로 줄고 큰 박스 존재는 372장으로
같다. native component GT가 없으므로 작은 박스 14장의 감소를 단순 중복 제거 또는
정상 반려동물 누락으로 확정할 수 없다. 1차 판단은 보류였으나 후속에서 일반 사람의
정상 대응·실제 ROS·OFF 복귀를 검증하고 순위만 운영 적용했다. 복합체는 별도 관측한다.
수·중복 감소만 보고 자산을 합치거나 lifetime을 늘리지 않는다.

## 실제 ROS 원점과 운영 진입점

현재 NN 서버→실제 source bridge의 `gravity_center_v1` adapter→설치 C++ tracker를
30초 실행했다. 바닥 표현은 같은 delivered message에서 명시적으로 역변환한 shadow다.
따라서 이 비교는 서로 다른 두 NN 프레임의 결과가 아니다.

- 응답·검출 170프레임, 512개 박스가 tagged bottom→gravity oracle과 오차 0으로 같다.
- 새 gravity/legacy tracker 출력은 700/698프레임, confirmed ID는 각각 3개다.
- 두 node의 TF 오류 로그는 0건이다. 독립 30Hz scheduler의 13프레임은 시간 대조에서
  제외했고 같은 프레임인 것처럼 맞춰 계산하지 않았다.
- 대응 track 2,034개에서 높이 절반 shift의 잔차 평균 약 0.98µm, p95 약 6.8e−15m,
  최대 1.64mm다. XY 모드 차이는 평균 0.020mm, p95 0.028mm, 최대 13.59mm로,
  서로 다른 copy 시각·관측 전환을 포함한다. tracker의 모든 wire가 같다는 인증은 아니다.
- 실제 contact 0, 정지 Ego 속도는 충분히 낮았다. 시험용 TF는 명시적인 heading 보정을 썼다.

로컬 5060 Ti에서 MORAI와 동시에 실행한 이 시험의 서버 추론은 평균 163.49ms,
prepared queue 평균 59.75ms, 검출 header→bag 수신 평균 367.48ms였다.
4090 또는 실제 대회 PC의 latency로 해석하지 않는다. 이 조건에서 latest-frame overwrite가
발생하는 것은 예상 가능하며 추론 시간이 LiDAR 주기보다 짧다는 인증도 아니다.

실제 `run_bevfusion.sh` production 진입점도 새 config·태그·gravity 선택으로 시작했다.
설치 launch를 사용한 별도 12초 smoke에서 canonical 검출과 timing은 각각 65개,
Ego는 391개, packets는 99개이고 실제 packet count는 51이다. 실패 status는 0건이다.
`261014_live_canonical_runtime_smoke.bag`에 보존했다. 이 시험에는 planner가 없다.

서버는 현재 한 persistent client를 순차 처리한다. bridge 연결 뒤 별도 health ping은
timeout될 수 있으므로 운영 기동의 **bridge 연결 전** readiness를 사용했다.
이를 GPU 추론 실패나 검출 중단으로 오인하지 않는다. 다중 client 서버로 바꾸지는 않았다.

현재 박스 중심 수정과 Map-CV의 future-Z 정책은 별개다. 차량 future는 기존처럼 road
centerline에 수렴하므로 높이 절반 offset을 1.5초 뒤까지 보존하는 검증은 완료하지 않았다.
planner의 2D 경로 선택은 변경하지 않았다. 기존 tracker·planner memory에 새 Z 기준을
hot-switch하지 않고 detector·tracker·planner를 정지한 뒤 새 stack으로 시작해야 한다.

## 적용 결정

| 항목 | 현재 판단 | 남은 확인 |
|---|---|---|
| driver 파라미터 잔류 | 수정 반영 | 팀장님 checkout·installed launch 반영 |
| nearest depth hint | E24 production 반영 | 팀장님 4090 새 실행·폐루프 확인 |
| gravity-center ROS 출력 | E24 현재 박스에 반영 | fresh stack 필수, future-Z·실전 planner 검증 별도 |
| pedestrian Gaussian ranking | 후속 검증 뒤 운영 ON | 복합체 별도 관측·팀장님 fresh 전체 주행 |
| fixed50/cut0 | 저속 이동 후속 확인, 기본은 보류 | 고속·LAN·실제 시간 정합을 별도 대조 |
| E24 obstacle proposal peak 변경 | 기존 기각 유지 | 기존 native TP479→369/368 회귀를 뒤집을 근거 없음 |

임계값·긴 coasting·컴파일·학습 grid·dataset quota를 이 시험에 묶어 바꾸지 않는다.

## 배포와 검증

E24 config는 `bevfusion_base5_fine_512_depth25_nearest_depth_runtime.py`다.
수동 production과 validator E24 프리셋이 같은 정책을 선택하며 E9/E25 비교·복귀는 보존한다.
config graph·원점·depth policy를 새 계약에 고정하고 태그 없는 구 서버는 교정 출력의
readiness에서 거부한다. 이전 execution에 코드가 달라진 상태로 강제 resume하지 않는다.

source와 이 PC의 `sensor_interface`, `bevfusion_ros`, `planning_scenario_validator` 설치본을
증분 갱신했다. 다른 agent의 runner·latency test·runbook 소스는 수정하지 않았다.
새 ZIP은 동일 가중치와 62개 runtime 참조 파일만 포함하고 이전 ZIP은 보존한다.
현재 descriptor·다운로드·설치는 [production handoff](BASE5_BEVFUSION_PRODUCTION_HANDOFF.md)를 따른다.
이번 반영은 `dataset/base5`와 이 PC 설치본이며 팀장님 planning 브랜치의 자동 merge는 하지 않았다.

ZIP 업로드는 공용 OAuth 앱의 403 rate-limit으로 실패한 뒤 기존 전용
`ajou_drive_private` 클라이언트로 완료했다. Drive 크기 449,672,125 bytes와 MD5
`cf9a6e2b3e473603a2f974f0fea08f4f`가 로컬과 같다. archive SHA256과 내부 62개 참조
파일도 검사했고 이미 설치된 가중치를 재다운로드하지 않는 경로를 통과했다.
새 빈 HOME에 ZIP으로 설치한 뒤 재실행해도 같은 가중치를 보존하는 경로도 실제 통과했다.
runtime smoke 후 시험 ROS/GPU 프로세스는 종료하고 사용자 MORAI·rc는 유지한다.

validator 전체 236개 검사는 현재 source를 우선 import하고 Xvfb에서 실행해 통과했다.
첫 전체 검사에서 구 installed package가 먼저 import되어 실패한 결과는 완료 근거로
합산하지 않는다. 설치본도 갱신했으며 별도 model/GUI 계약 18개, production profile 7개,
runner 5개, bundle 6개, LiDAR launch 9개, 서버/통신 11개와 sensor geometry 1개를 확인했다.
새 실험 도구의 수학·원점·소스 보존·actor 경로 검사도 각각 통과했다.
