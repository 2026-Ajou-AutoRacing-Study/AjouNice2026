# 센서 시간축·중복 검출·추적 연속성·노드 비용 검토

문서 상태: EVIDENCE와 후속 검증 계획. 기준일: 2026-10-14.
MORAI를 끈 상태에서 저장된 입력으로 원인을 분리했다. 출력이 정확히 같은 후단 계산 최적화는
소스에 반영하고, 검출·association·센서 시각 후보는 운영 기본값과 분리한다.
E24 가중치, threshold, coasting, 지도 경로 선택, 기존 5K와 v150 release는 유지한다.
실제 배포 기준은 [모델 CURRENT](CURRENT.md)와
[후단 CURRENT](../../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md)가 관리한다.

## 범위와 근거 보존

이번 작업은 새 학습·본수집·원격 과금 작업·planner 정책 변경을 수행하지 않는다.
MORAI OFF 이후의 오프라인 작업이며 앞서 수집한 세 raw-packet bag을 사용한다.
팀장님 완주 bag은 Music 원본을 보존하고 `rosbags/eval/perception/261014_teammate_fullstack/`에
검증된 복사본을 사용한다. 분석 JSON·이미지·컴파일한 연구 코어는 Git 제외 경로에 둔다.

| 영역 | 근거 루트 | 검증 대상 |
|---|---|---|
| 원시 패킷 | `planner_logs/analysis/261014_lidar_packet_offline/` | 정지·고주로·로터리 3 bag, 원본 51패킷 cloud와 후보 |
| UUID·중복·association | `planner_logs/analysis/261014_offline_tracking_review/` | 최신 완주·기존 개발 bag·합성 반례·원본 C++ 코어 재생 |
| 후단 CPU | `planner_logs/analysis/261014_node_cpu_review/` | 6 bag, 동일 메시지 직렬화와 별도 프로세스 반복 |
| 모델·데이터 | `planner_logs/analysis/261014_base5_model_data_review/` | 완료 5K·native infos·동일 Base3 val·새 head 연구 후보 |

Raw 수집은 GT pose/detection을 이용한 제어 실험이다. 이 세 bag이 production E24로
폐루프 완주한 결과라고 해석하지 않는다. 최신 팀장님 bag은 E24 완주 근거지만 raw 센서·
저점수 decoder 출력·정확한 planner consumed object/path index가 없다.
신호등 노드의 실제 connection도 없으므로 YOLO와 BEVFusion 동시 GPU 부하를 인증하지 않는다.

<a id="packet"></a>

## LiDAR 50/51패킷과 시간축

세 bag의 75,888개 패킷에서 첫 block azimuth 간격은 중앙값 7.2°다.
원본 51패킷은 약 367.2°를 담고 다음 scan의 시작 각도가 약 7.2°씩 이동한다.
50패킷은 약 360°이며, angle-cut 후보는 부팅 시 부분 scan을 제외하면 50패킷이다.
이는 실제 MORAI 입력에서 측정한 값이지 모든 VLP16에서 50개가 맞다는 결론은 아니다.

상류 드라이버는 VLP16의 nominal 754 packet/s와 RPM으로 개수를 올림 계산한다.
angle-cut 경계 packet을 이전 scan에 포함하고, 설정에 따라 첫/마지막 packet stamp를 사용한다.
[Velodyne 1.7.0 driver.cc](https://github.com/ros-drivers/velodyne/blob/1.7.0/velodyne_driver/src/driver/driver.cc)
와 같은 경계 규칙으로 후보를 재구성했다.

| 오프라인 후보 | 각도 범위 | 한 scan의 서로 다른 footer 시각 | 역할 |
|---|---:|---|---|
| fixed51 | 약 367.2° | 2개 | 실제 운영 원본 대조 |
| fixed50 | 약 360° | 2개 | 수집 시작 위상에서 고정 개수 비교 |
| cut0 | 약 360° | 1개 | 현재 입력의 source batch 경계와 일치 |
| cut180 | 약 360° | 대부분 2개 | cut 위치 자체의 영향 분리 |
| cut0 + footer reference | cut0과 같은 XYZI | 1개 | reference/camera 선택만 바꾸는 별도 연구 조건 |

현재 MORAI 입력은 footer 값이 같은 50패킷을 묶음으로 보내는 양상이다. footer가 바뀌는
첫 packet의 host stamp는 footer보다 중앙값 약 37~39ms 이르지만, 나머지 packet의 host
stamp는 footer보다 약 0.6~0.7ms 늦다. 기본 UDP reader가 읽기 전후 ROS 시각의 중간값을
쓰는 구현과 일치한다. 기다리는 첫 packet에 이전 대기시간의 절반이 섞이는 것이다.
[Velodyne 1.7.0 input.cc](https://github.com/ros-drivers/velodyne/blob/1.7.0/velodyne_driver/src/lib/input.cc)

footer는 이 실험에서 source batch를 구분하는 근거다. 같은 footer가 모든 ray의 정확한
취득 시각이거나 순간 snapshot임을 보증하지 않는다. 같은 PC에서 일치한 시계도 다른 PC의
LAN clock 정합을 인증하지 않는다. 표준 VLP firing 모델로 생성된 point `time`을 MORAI의
실제 ray time으로 가정해 deskew를 다시 켜지 않는다.

설치된 VLP16 pointcloud plugin을 private ROS master에서 재사용해 원본을 복원했다.
원본 1,488 cloud의 XYZI·ring·time은 **1,488/1,488 전부 typed-field 기준 정확히 같다**.
구조체 padding byte가 아닌 실제 6개 field와 NaN을 비교했다. UDP driver와 production
master는 실행하지 않았다. 이 qualification 뒤 후보 cloud를 같은 E24로 추론했다.

공통 latest-footer anchor 100개씩, 3 case×5조건=1,500프레임을 비교했다.
startup partial·packet span 0.15초 초과·원본 시점에 확보되지 않은 camera는 입력 선정에서
제외하며 선택은 검출 score를 보기 전에 고정했다. operating postprocess는 동일하다.
단일 프레임의 출력 차이·catalog correspondence이지 연속 tracker·AP·live queue 인증이 아니다.

**현재 결론:** 51→50만으로 큰 실시간 이득을 약속하지 않는다. 실제 입력이 packet burst라
50/51의 대기시간이 단순 1/754초 차이가 아니다. angle/source boundary와 camera/pose
reference를 함께 분리해야 한다. 운영 드라이버·TF·timestamp 기본값은 이번에 바꾸지 않는다.
footer로 header를 바꿔 숫자상 age가 줄어도 물리적 계산 시간이 줄었다고 보고하지 않는다.

<a id="tracking"></a>

## 보행자 중복과 ID 교체 원인

최신 완주의 drive window는 tracking 4,870프레임·5,499박스다. IoU 0.5 초과의 강한
같은-frame track 중복은 없고 UUID의 class 변경도 없다. 그러나 같은 Man1 근처에
여러 UUID가 붙은 낮은-IoU 중복은 50프레임, 1.03%다. IoU만 검사하면 놓친다.
GT 1m gate로 더 좁혀도 12프레임에서 두 UUID가 남는다. 차량·CargoBox에서는 같은
급속 교체 양상을 확인하지 않았다.

비차량 ObjectStatus의 name이 비어 있어 companion scenario의 UNIQUEID/DataID를
모든 관측 위치와 대조한 별도 assisted 결과다. Man1은 5개 UUID로 분절됐다.
CargoBox 비균등 scale의 Unity 축은 raw Instance GT 없이 확정하지 않으며 그 크기 오차를
정밀한 GT 오류로 단정하지 않는다. 사람의 ID 분석은 이 scale 가정에 의존하지 않는다.

원본 C++ EKF 코어에 1,825 detector 입력을 재생해 ID birth/association/coast/death를 재현했다.

1. 같은 사람의 두 박스가 나오고 하나는 기존 ID에 연결된다.
2. 남은 높은-score 박스로 tentative ID가 새로 태어난다.
3. 초기 속도 불확실성이 다음 prediction에서 XY covariance를 넓혀 새 track의 Mahalanobis 비용이 작아지고 관측 경쟁에서 유리해진다.
4. 기존 track은 관측을 잃고 정상적인 0.5초 coasting 뒤 삭제된다.

해당 구간 detector 간격 최대는 약 0.19~0.20초다. 0.5초 이상 센서 공백이나 UUID 메모리
손상으로 설명되지 않는다. 현 association은 cost 정렬 후 greedy matching이며 Hungarian이 아니다.
초기 전체 covariance 값 100이 XY에 그대로 남는 것은 아니다. birth 때 첫 관측 update로 XY는
약 0.16m²로 줄고, 큰 초기 속도 분산이 다음 0.07~0.1초 prediction에서 XY 분산을 약
0.7~1.2m²로 넓히는 과정까지 원본 코어에서 확인했다.

Birth threshold를 올리거나 사람의 center suppression을 넓히면 이 clip의 중복은 줄지만,
약하게 새로 보이는 사람 또는 실제 가까운 두 사람·baby/dog component를 삭제하는 반례가 있다.
따라서 이 clip을 맞추려고 전역 score나 guard를 강화하지 않는다.
Confirmed ownership 및 covariance-volume을 반영하는 likelihood ranking은 별도 코어에서
검증한다. 원래 XY/class gate와 ranking을 구분하며, 후보 기본값을 자동 승격하지 않는다.

<a id="cpu"></a>

## 출력 동일 후단 CPU 최적화

지도 segment의 길이·접선·projection prefix와 route geometry만 재사용한다.
관측·lane rank·현재 투영점·미래 pose는 캐싱하지 않는다. route LRU 128개,
transition LRU 512개로 제한하고 짧은 lock으로 경쟁을 막는다. 같은 lane ID의 frozen
record가 교체되면 다시 계산하며 다른 map instance 사이에는 캐시를 공유하지 않는다.

고정 ROS schema의 primitive-aware 독립 복사로 generic deepcopy 비용을 줄였다.
UUID·covariance·shape·pose·path·stamp의 mutable 공유가 없음을 검사했다.
재귀 DFS closure의 reference cycle은 explicit stack으로 제거하되 방문·동점·분기 cap·
loop/depth 순서를 유지했다. GC를 끄거나 의심 표본을 버리지 않는다.

6 bag의 10,433 tracking frame·11,021객체를 각각 두 번 비교해 prediction과 UNKNOWN
정책 출력의 직렬화 byte·객체/경로 수가 모두 정확히 같다. 아래는 로컬 CPU의
메시지 생성+직렬화 비용이며 팀장님 PC의 live 전체 지연이 아니다.

| 입력 | 기존 평균 ms | 개선 평균 ms | 기존 p95 ms | 개선 p95 ms |
|---|---:|---:|---:|---:|
| 최신 풀주행 | 2.160 | 0.982 | 6.002 | 2.576 |
| 고주로 합류 | 1.031 | 0.471 | 4.544 | 1.423 |
| 고주로 순항 | 1.434 | 0.563 | 3.222 | 1.211 |
| 로터리 | 2.841 | 1.209 | 4.731 | 2.015 |
| 장애물 | 0.477 | 0.326 | 0.752 | 0.513 |
| 보행자 | 0.194 | 0.135 | 0.201 | 0.140 |

풀주행 p50/p90/p99는 각각 1.657→0.684 / 5.237→2.291 / 7.942→3.167ms다.
별도 프로세스의 후보 3회 반복은 평균 0.999ms·p95 2.554ms지만 **최대 86.976ms**이고,
해당 spike의 generation-2 GC는 85.285ms다. 따라서 최대 지연 해결이라고 주장하지 않는다.
근거는 `final_safe_v7/report.json`, `isolated_safe_candidate_v8/`에 있다.

소스 최적화와 설치본을 구분한다. `base_ws/install_aw149`의 기존 노드는 복사본이므로
이번 소스 변경만으로 실행 중 노드·팀장님 배포본이 바뀌지 않는다. 배포 후 ROS 입력 age·
drop·callback p95/p99·closed-loop를 별도로 확인한다.

<a id="diagnostics"></a>

## 다음 주행의 개별 노드 진단

Map-CV와 UNKNOWN 노드에 `publish_processing_diagnostics:=true`의 선택적 JSON 진단을
추가했다. 기본은 OFF이며 prediction/class/shape/UUID/stamp 출력은 바꾸지 않는다.

```text
/map_based_prediction/diagnostics/processing_time_ms
/tracked_object_motion_policy/diagnostics/processing_time_ms
```

Input header age, callback 간격, input header 간격, 메시지 생성, synchronous publish,
callback wall/thread CPU를 나눈다. 이 header age는 **30Hz track의 age**이지 마지막 detector
관측 age가 아니다. synchronous publish는 직렬화·잠금·enqueue 비용을 포함하고 downstream
전달 완료 시간이 아니다. 진단 JSON 생성·발행 비용은 callback 집계 밖이며 bookkeeping은 포함한다.
[보조 녹화 스크립트](../../scripts/rosbag/record/rosbag_for_perception_latency_eval.sh)는 두 토픽을
포함한다. validator 소유 파일을 대신 수정하지 않는다.

<a id="porting"></a>

## C++ 전환 판단과 안전 경계

| production 구성 | 실제 주 연산 | 전환 판단 |
|---|---|---|
| Velodyne driver/pointcloud | 이미 C++ | 언어 전환 대상 아님. packet 경계·clock을 독립 검증 |
| EKF tracker | 이미 C++ | association·불필요한 시각화 비용을 실제 코어에서 먼저 분리 |
| Map-CV predictor | Python + Lanelet2 C++ binding | 포팅 가능하나 최적화 후 해당 함수 평균 약 1ms. 이 부분만으로 큰 전체 배속 불가 |
| UNKNOWN motion policy | Python, 작은 복사/수치 처리 | 비용이 매우 작아 우선순위 낮음 |
| BEVFusion bridge/server | Python orchestration + NumPy/OpenCV C++ + Torch/spconv/CUDA | C++ wrapper와 TensorRT/custom-op engine 작업은 별개 |
| 신호등 YOLO | Python wrapper + Torch/OpenCV CUDA | 먼저 BEV 동시 부하·decode/callback 측정. 전체 C++라고 GPU 비용이 자동 감소하지 않음 |
| localization EKF | robot_localization C++ | adapter/smoothing의 Python 부분만 별도 측정 |
| 주요 Autoware planner/optimizer | 이미 C++ | 실제 consumed path와 모듈 processing time을 기록한 뒤 최적화 |

E24 compressed 경로의 host camera executor는 JPEG resize에 쓰이지 않는다.
원본 JPEG를 GPU server에 보내고 server OpenCV에서 decode/resize한다.
학습 DataLoader 8 workers가 실주행에서 돌아가는 구조도 아니다.

Map-CV 포팅 gate는 v85 map/regulatory loading, nearest/tie/DFS 순서, route clustering,
confidence, 31 poses, fallback, UUID/covariance/source stamp의 shadow 동등성이다.
fast-math는 초기 포팅에서 피한다. 이후 ROS age/drop·queue·p95/p99와 폐루프를 확인한다.
GPU 모델은 ONNX/TensorRT 변환과 custom sparse/BEV ops·checkpoint/precision 호환을 따로 검증한다.
현재 측정 없이 모든 노드를 C++로 옮기면 몇 % 빨라진다고 약속하지 않는다.

## 다음 검증 순서

1. 출력 동일 CPU 최적화의 설치본을 별도 배포하고 선택적 stage 진단을 기록한다.
2. packet/camera/pose reference는 shadow A/B로 비교한다. LAN clock·실제 ray time 불확실성을 해결하기 전 production stamp를 바꾸지 않는다.
3. 보행자 association 후보를 실제 근접 두 사람 교차·가림·새 약한 관측에서 검증한다.
4. 검출 proposal peak·score 성분과 얇은 장애물 중복을 전 클래스·native val·개발 bag에서 비교한다.
5. 완료 5K/v150을 보존한 채 전방 근접·얇은 barrier·희귀 asset support를 새 배치 실험으로 보강한다.
6. 독립 평가 split과 짧은 학습 비교 뒤 최종 학습한다. 개발 bag을 train으로 돌려쓰지 않는다.

단일 bag에 맞춰 threshold·거리·기억을 반복 조절하는 방식은 피한다.
검증을 기다리는 후보와 출력 동일 소스 개선, 실제 배포 완료 상태를 문서에서 구분한다.
