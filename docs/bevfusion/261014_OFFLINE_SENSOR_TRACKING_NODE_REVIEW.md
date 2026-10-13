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

### 표본 확대와 카메라 고정 대조

입력 조건을 만족하는 공통 anchor 전체 808개에 5조건을 적용해 4,040프레임을 추론했다.
Ego 보간에 필요한 두 고주로 anchor는 모든 조건에서 함께 제외하여 실제 코어 비교는
806개다. 해당 Ego 공백은 2.172초와 0.410초이며, 다른 시각의 pose를 대신 쓰지 않았다.
정지 353·고주로 236·로터리 217개 입력을 실제 EKF 원형에 재생했다. 비차량 name/
catalog 미확정과 native 가시성 GT 부재 때문에 아래 대응을 AP나 FP/FN으로 부르지 않는다.

| 동적 case와 지표 | fixed51 | fixed50 | cut0 | cut180 | cut0 + footer |
|---|---:|---:|---:|---:|---:|
| 고주로 Vehicle ID 수 | 18 | 18 | 17 | 18 | 18 |
| 고주로 출력 ID 교체 후보 | 1 | 1 | 0 | 1 | 1 |
| 로터리 Vehicle ID 수 | 10 | 10 | 11 | 10 | 10 |
| 로터리 출력 ID 교체 후보 | 0 | 2 | 1 | 0 | 2 |

GT 근처 여러 confirmed UUID가 겹치는 중복은 이 비교에서 0건이다. ID 교체는 기존
track의 마지막 검출 관측 뒤 0.503~0.528초가 지나 기억이 만료되고 다시 태어나는
양상이다. 아래 Man1의 새 ID가 관측을 빼앗는 사건과 다르다. cut0 고주로에는
일시적인 Pedestrian track도 있어 단순히 ID 수가 적은 조건을 우승으로 고르지 않는다.

별도로 300개 공통 anchor의 JPEG와 declared reference를 원본 51 조건에 고정해
geometry만 비교했다. 차량 중심 차이의 p95는 고주로 fixed50/cut0/cut180 각각
0.774/0.693/0.672m, 로터리 0.609/0.919/0.781m다. 이는 같은 입력의 예측 간
차이이지 GT 오차나 일관된 개선량이 아니다. 일반 비교의 camera 선택 변화만으로 모든
차이를 설명할 수 없었다. 이 대조의 `cut0_footer` 항목은 같은 cut0 입력 반복이며
footer 시간 변경 실험이 아니다. manifest의 `control_role`로 둘을 구분한다.

원시 packet·추론·추적·pose 보간 조건을 남긴 재사용 도구는
`scripts/evaluation/{lidar_packet_reassembly,decode_lidar_packet_candidates,infer_lidar_packet_candidates,analyze_lidar_packet_candidates}.py`와
`audit_tracking_continuity_packets.py`다. 실제 live driver의 publish/queue 또는 폐루프를
재현한 것이 아니다. 원본 decoder 복원·후단 코어 반복 동등성부터 확인한다.

선택적 시험 인자만 추가했다. 기본 `npackets`는 명시하지 않아 기존 올림 계산을 유지하고,
`cut_angle=-0.01`, `gps_time=false`, `timestamp_first_packet=true`도 그대로다.
`lidar.launch`는 `npackets:=50` 또는 `cut_angle:=0`, 상위 launch는
`lidar_npackets:=50` 또는 `lidar_cut_angle:=0`을 전달한다. 후자는 라디안이며 0~2π
범위다. 두 launch의 기본 resolved parameter가 이전 HEAD와 정확히 같음을 확인했다.
운영 실행 명령을 이 후보로 교체하거나 live UDP driver를 켜지는 않았다.

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

원래 gate를 유지한 pedestrian-only Gaussian likelihood ranking 후보는 두 clip에서
다음 결과를 보였다. 최신 bag의 core detector-input 시점 분모이며 위 30Hz 출력 분모와 다르다.

| 항목 | 최신 bag 기존→후보 | 이전 P01 기존→후보 |
|---|---|---|
| 보행자 confirmed ID 수 | 5→2 | 3→2 |
| 급속 switch 후보 | 2→0 | 2→0 |
| core 입력 시점 중복 | 16→0 | 6→0 |
| GT 2m 대응 관측 | 114→114 | 128→128 |

최신 bag의 평균 XY 오차는 0.414→0.429m로 소폭 증가했다. 같은 4개 개발 bag에서
비보행자 상태는 UUID counter만 제외하면 정확히 같다. 하지만 stale track이 재사용되는
경우, noisy newcomer와 ownership 시점이 달라지는 경우, 같은 XY의 adult/dog 속성이
섞이는 경우가 남았다. 한 사람 clip 두 개와 합성 반례 통과만으로 실제 군중의 안전성을
인증하지 않는다. association과 새 ID birth suppression은 다른 조치다.

첫 교체 약 66.881초 주변에는 longitudinal-min-acc invalid 3건과 Ego에 가까운 trajectory
목표 속도 8.047→6.279→0m/s가 겹쳤다. 두 번째 71.881/71.981초 주변에는 invalid·brake가
없고 목표 속도 인접 최대 차이는 0.475m/s다. 실제 consumed UUID/path, 횡단 진입·신호등·
지연의 분해가 없어 ID 교체가 제어 급변을 일으켰다고 단정하지 않는다. 완주·무충돌은 유지됐다.

Static avoidance의 UUID 실패 시 class 무관 1.5m spatial fallback과 5초 history는
현재 C++ 실행 경로에도 있다. 가까운 서로 다른 객체의 기억을 섞을 구조적 가능성은
있지만 이번 bag에서 실제 잘못 재연결됐다고 확증하지 못했다. planning 정책은 유지한다.

<a id="model-data"></a>

## BEVFusion과 Base5 보강 판단

완료 5K의 실제 11,052 native component와 infos 대응은 누락 0개다.
Train의 풍부한 sensor support proxy는 8,257/9,350, 88.31%; val은 1,393/1,702, 81.84%다.
정확도 보증이나 모든 부분 관측 GT 삭제 조건으로 사용하지 않는다.

장애물 20~30m의 610 GT 중 LiDAR 20점 이상은 27개, 4.43%지만 camera mask-rich까지
합치면 459개, 75.25%다. 전부 안 보이는 것이 아니라 depth·형상·heading 학습이 어려운
경우를 분리해야 한다. v150의 20~30m 짝 비교에서도 LiDAR-rich 증가는 확인하지 못했다.
0~10m 장애물은 후방 495/전방 285로, 후방을 없애지 않고 전방 근접 노출을 보충한다.
장애물 yaw의 30° 구간별 206~228개 분포는 이미 균등하다. yaw 추가 랜덤화가 우선은 아니다.

같은 Base3 retained GT 2,184개에서 score≥0.1·중심 1m 대응은 E25 2,112/E24 1,952개다.
E24 누락 232개 중 202개는 GT 1m 안에 같은-class raw box가 있지만 score가 낮다.
대응 gate 1→2→4m 확대는 이를 해결하지 않았다. floor 0.1→0.05는 TP +80이지만
미대응 +145·retained 단일 GT 근처 잉여 box +100이므로 전역 하향을 추천하지 않는다.

E24의 근접 잉여 box 114건 중 99건은 Steel_Barricade·concrete-barrier·PE Firewall다.
쌍의 중심 차이 중앙값 7.61cm, yaw 축 차이 53.35°, IoU 0.297이다. 같은 중심의 가느다란
방향 변종이 정상 NMS 0.5를 통과하는 양상이며 E25에서는 같은 진단이 11건이다.

### Query peak 후보: 현재 E24에 바로 적용하지 않음

Waymo 호환 분기의 class 1/2 kernel-1 예외가 MORAI Pedestrian/Obstacle에 적용된다.
Obstacle의 heatmap query peak를 kernel 3/5로 바꾸는 독립 subclass를 준비했다.
기본 None은 실제 원형에 위임하며 [3,1,1]은 같은 frozen BEV feature에서 원형과 정확히 같다.
가중치 key·GT·grid·학습 parameter는 변하지 않는다. 그러나 전체 Top-200 query 구성은 변한다.

Native val 전체 745장과 개발 bag 200장, 총 945장을 같은 E24 feature로 비교했다.

| Native val 장애물 항목 | 기존 | peak3 | peak5 |
|---|---:|---:|---:|
| GT 674개 중 중심 1m TP | 479 | 369 | 368 |
| 단일 GT 근접 잉여 box | 42 | 0 | 0 |
| 미대응 출력 | 75 | 16 | 15 |

Peak3에서 놓친 110개 중 107개는 가까운 raw box가 남지만 score<0.1이다.
대응 가능 108개 GT의 dense prior 중앙값은 0.574→0.784로 높아졌는데 decoder sigmoid는
0.625→0.056, 최종 score는 0.312→0.041로 줄었다. 현재 decoder가 바뀐 query 구성에
적응되지 않은 결과로 해석하며, dense prior가 약해져 실패했다고 설명하지 않는다.
0~10m와 LiDAR≥20점의 선명한 GT도 잃었고, 서로 다른 parent의 Bollard 1.43m 쌍은
2 hit→0이었다. 중복 감소만 보고 적용하면 안 된다. 차량 TP 281/341은 유지됐고
사람은 578/687→577/687로 거의 같지만 장애물 회귀만으로도 현 checkpoint 전환은 기각한다.
후속 학습 후보로 검토할 수 있으나 사후 head 변경이 무상 성능 개선은 아니다.

### Depth hint의 중복 pixel 쓰기: 반복 변동 원인 분리

현재 view transform은 LiDAR를 image pixel에 투영해 sparse depth hint를 만든다.
여러 점이 같은 pixel에 들어오면 `index_put` 계열의 중복 목적지 쓰기가 생긴다.
`accumulate=False`의 중복 index 결과는 정의되지 않는다는
[PyTorch 공식 명세](https://docs.pytorch.org/docs/stable/generated/torch.Tensor.index_put_.html)와
실제 GPU 반복이 일치했다. sparse LiDAR branch는 같은데 depth hint가 달라지는 입력을
확인했고, 그 차이가 camera BEV·fusion·head 입력까지 전파됐다. AMP·병렬화·TF32를
꺼도 같은 legacy 변동이 남았다. 앞선 모든 compile 오차의 원인이 이것이라고 단정하지 않는다.

같은 JPEG·reference의 20 anchor×3 case×5조건 CPU 투영에서 서로 다른 Z가 충돌하는
pixel 수는 카메라당 평균 고주로 fixed51 34.64/fixed50 0.80/cut0 0.88,
로터리 39.48/1.24/1.28개였다. packet 수 차이는 약 2%인데 충돌 수 차이는 크다.
중첩된 azimuth와 이동 중 서로 다른 source batch가 기여할 가능성을 보이며,
MORAI의 ray 취득 시각을 확정하거나 scan 정책을 바로 바꿀 근거로 사용하지 않는다.

실험용 subclass는 유한한 양의 **camera optical-Z**를 같은 pixel별로 정렬하고,
목적지가 하나씩만 남은 뒤 nearest/farthest/unique-last를 쓴다. Euclidean LiDAR range,
camera calibration·augmentation·BEV pooling·GT·sparse branch·가중치 key는 바꾸지 않는다.
기본 None은 실제 원형에 위임한다. nearest는 더 가까운 표면을 hint로 삼는 후보이지
신경망이 예측하는 depth bin을 25cm에서 다른 값으로 바꾸는 것이 아니다.

| qualification | 결과 |
|---|---|
| 고정 입력 5개 case의 legacy depth 반복 | canonical/None 합계 30회 중 9회가 반복 기준과 다름 |
| nearest/farthest/unique-last stage 반복 | 각 15회 depth→camera BEV→fusion→head 입력 정확히 같음 |
| O02와 native의 raw 최종 박스·label·score | 정책별 3회 정확히 같음; canonical O02는 한 번 다름 |
| native val 745장, nearest의 중심 1m TP | Vehicle 281/341, Pedestrian 578/687, Obstacle 479/674로 기존과 동일 |
| native·4개 dev bag 총 945장 | operating 검출·누락·미대응·근접 잉여 집계 변화 없음 |
| native 대응 박스 중심 변화 p95 | V 0.198mm / P 0.111mm / O 0.155mm |
| native yaw 축 변화 p95 | V 0.016° / P 0.240° / O 0.140° |
| 로컬 5060Ti 모델 평균 비용 | 123.19→124.10ms, 약 +0.91ms |

평균 비용은 hooks OFF의 독립 forward이며 4090 ROS 인증값이 아니다. 반복 안정화와
검출 성능 개선을 구분한다. `bevfusion_base5_fine_512_depth25_nearest_depth_trial.py`는
연구 후보로만 제공하며 E24 production config·checkpoint·학습 계약은 유지한다.
새 학습/추론 후보를 같은 hint 조건에서 비교하고, compile A/B도 eager A/A 변동과
분리하여 다시 검사한다. 모든 GPU·모든 입력의 bitwise 보장은 이번 표본보다 큰 주장이다.

### 다음 생성·학습 실험

기존 5K·v150은 보존한다. 전방 0~20m rich-floor, 얇은 barrier의 크기/방향/부분 관측,
실제 근접 두 객체, rare passive OBJ의 자산별 train support를 독립 보강 후보로 삼는다.
20~30m는 LiDAR만 늘리는 주장보다 camera support·두 score 성분·형상 회귀를 함께 본다.
차량 50~80m의 접근·후방·가림 노출을 보호한다. 원거리 positive GT 삭제나 복합체 mask의
unknown을 비가시로 치환하지 않는다. 개발 bag/Base3 val을 train에 재사용하지 않는다.

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

이 PC의 `perception_object_pipeline`·`bevfusion_ros`·`sensor_interface` 세 패키지만
`catkin_make_isolated --install --pkg`로 빌드했다. 설치 helper/launch와 소스의 hash도
대조했다. validator를 포함한 다른 패키지는 빌드하지 않았고 운영 노드는 켜지 않았다.
로컬 설치 완료와 팀장님 배포 완료는 다르다. ROS 입력 age·drop·callback p95/p99·
closed-loop는 별도로 확인하며 새로운 빌드만으로 주행 검증을 완료했다고 하지 않는다.

Private ROS master에 실제 설치된 UNKNOWN→Map-CV 두 노드만 띄워 최신 bag의 혼합
600프레임을 계층 샘플링했다. 센서·GPU·localization·planner·제어는 켜지 않았다.
30Hz에서 진단 OFF/ON 모두 각 노드 600개를 전달하고 publisher seq를 정규화한 나머지
wire는 순수 함수 기준과 정확히 같다. ON 진단도 각각 600개다. 약 90Hz 스트레스에서는
Map-CV가 OFF 484/ON 490개를 전달했으며, 모든 수신 출력은 기준의 정확한 subset이다.
queue_size=1은 포화 시 최신 입력을 남기는 정책이므로 속도를 올려 모든 과거 입력을
전달하는 기능으로 설명하지 않는다. 진단 자체도 queue 1이므로 과부하에서 진단 수와
callback 수가 다를 수 있다. OFF/ON 누락 차이를 진단 기능의 성능 향상이라고 해석하지 않는다.

30Hz ON의 callback thread CPU 평균/p95는 UNKNOWN 0.187/0.250ms, Map-CV
0.817/3.873ms다. 시험 driver→Map-CV의 ROS 전달 wall 평균/p95/최대는
1.896/6.392/21.586ms다. 입력 표본·동시 호스트 부하·transport를 포함하므로 앞의
단독 함수 비용 표와 직접 배속 비교하지 않는다. 원래 header를 보존하고 시험 clock을
생성하되 샘플 사이 wall gap을 압축했으며, 독립 TCP clock 순서 때문에 sim age가
음수인 경우도 있다. 이것을 실제 센서 age로 사용하지 않는다. 모든 private 프로세스는 종료했다.
최종 근거는 `private_ros_final_v4/report.json`과 `install_cleanup_receipt.json`이다.
empty 243/nonempty 357프레임, 차량 351·사람 142·UNKNOWN 134개와 mapped/fallback
343/284개를 포함한다. 소스/설치본 9파일과 config YAML의 hash도 같다.

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

BEVFusion bridge도 `publish_frame_timing:=true`로 단계별 JSON을 선택할 수 있다.
운영 실행기는 아래처럼 활성화한다. 기본 OFF이며 sensor/request payload와 출력은 불변이다.

```bash
BEVFUSION_PUBLISH_FRAME_TIMING=true bash scripts/runtime/run_bevfusion.sh
```

Canonical `/perception/bevfusion/frame_timing`, trial `/perception/base5_trial/frame_timing`을
보조 recorder가 함께 요청한다. callback header age·camera 선택·raw queue 대기·host 준비·
prepared queue 대기·TCP/server·object publish와 종료 age를 분리한다. 기존 Float32 latency는
camera 선택 뒤 enqueue부터 response 반환 전후까지의 기존 범위를 유지하며 publication을
포함하도록 몰래 재정의하지 않는다. receipt clock과 ROS source stamp를 서로 빼지 않는다.
실제 raw bag 20프레임 ON/OFF의 JPEG·XYZI·semantic metadata·DetectedObjects·MarkerArray
wire가 정확히 같고, 에러/overwrite/overflow/종료의 bounded sidecar 정리도 확인했다.
계속 sync가 거부되면 성공-frame JSON이 없으므로 starvation은 기존 rosout 경고와 입력/
출력 수를 함께 확인한다. live ROS/LAN 재녹화는 아직 하지 않았다.

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

현재 launch를 노드 실행 없이 resolve한 inventory는 `cpp_inventory_v1/assessment.json`과
`resolved_launch.json`에 executable hash·remap·선택된 parameter를 남겼다.
운영 tracking alias는 detector 자체를 시작하지 않고 C++ EKF→Python UNKNOWN→Python
Map-CV를 연결한다. 별도 CTRV/fixed-rate adapter와 FastBEV/PointPainting은 활성 경로가
아니다. static TF·route/map/mode helper는 Python이지만 저빈도 상태 작업으로,
우선 포팅할 병목 근거는 없다. launch에 들어 있는 worker 수를 실제 사용 thread 수와 혼동하지 않는다.

또한 내부 30Hz 출력의 최대 허용 age와 개별 track coasting을 구분한다.
`maximum_fixed_output_prediction_age_sec`는 최신 코어 상태가 너무 오래되면 새 30Hz copy
발행을 멈추는 node gate다. 실제 최신 bag/운영 alias는 0.5초이나 planning-stack의
내부 기동 기본 인자는 0.25초다. 개별 actor의 마지막 검출 이후 생존은 별도의
`confirmed_track_max_coast_time_sec=0.5`와 정지 track 1.0초 규칙이다. 이번 작업은
이를 통일하거나 자동 변경하지 않았고, 기동 경로별 resolved 값을 남겨 다음 주행에서 대조한다.

C++ tracker의 marker 생성을 subscriber가 없을 때 생략하는 복사본도 측정했다.
공개된 track을 복원하고 실제 500-slot 순회를 반영한 정상 bag은 약 0.035→0.017ms,
500개 retired marker 스트레스는 약 0.64→0.026ms다. 숨은 tentative/retired 상태까지
원본 bag에서 재현한 것은 아니며 실전 주 병목이라고 할 근거가 없다. 원 tracker는 미변경이다.

BEVFusion bridge의 Python rotated NMS도 계산 위치를 의심해 실제 4,985프레임의
server floor 0.1/top100 입력에서 박스 기하 캐시를 비교했다. NMS 입력 수는 평균 약
2개, 최대 7~10개였다. 기존 평균 0.005~0.007ms가 캐시 후보 0.006~0.008ms로
오히려 늘었다. retained index·객체 identity·순서·입력 무변경과 경계 회귀는 같지만
이득이 없어 기각했다. 최대 100개 설정이 실제로 매 프레임 100개를 처리한다는 뜻은 아니다.
운영 NMS는 미변경이며 `rotated_nms_cache_v1/report.json`에 근거를 보존했다.

이미 C++인 EKF의 cost loop도 독립 사본에서 확인했다. 원형은 비활성 slot에도 거리와
2×2 Mahalanobis 계산을 수행한 뒤 비용을 1000으로 덮어쓴다. 후보는 그 slot만 동일한
1000을 먼저 쓰고 건너뛴다. 순수 함수임과 active cost·gate·tie 순서 불변을 확인하고
Release 동일 `-O3 -DNDEBUG -std=c++14`의 실제 코어를 비교했다.
4개 dev 1,570 + 최신 완주 1,825 + packet 4,030 + 합성 12, 총 7,437프레임에서
active 상태·8×8 covariance·class vote·history·ID·생존 이력이 정확히 같았다.

| RunUpdate 단독 평균 ms | 원형 | 후보 |
|---|---:|---:|
| 최신 완주 | 0.01284 | 0.00368 |
| O02 장애물 | 0.03451 | 0.00606 |
| packet 고주로 | 0.03995 | 0.00753 |

현재 분포의 절약은 9~32µs이며 전체 ROS publisher나 GPU 지연이 아니다.
500개 slot 모두 active·측정 100개 스트레스에서는 평균 1.292→1.392ms로 약 7.7%
악화했다. 보통 slot이 드문 장면에서 큰 비율 이득이 보여도 절대 비용과 반례를 함께
보고, 운영 코어는 바꾸지 않았다. `cpp_fastpath_v1/fastpath_report.json`과 재사용
`audit_tracking_continuity_fastpath.py`에 연구 후보를 보존한다.

## 다음 검증 순서

1. Depth hint 안정화 후보에서 compile OFF/ON과 A/A를 다시 분리하고 독립 입력으로 회귀를 확인한다. 현재 E24의 proposal peak 변경은 기각했다.
2. 출력 동일 CPU 개선·단계 진단의 로컬 설치와 private ROS 전달 검증을 완료했다. 팀장님 배포와 실제 풀스택 age/drop을 확인한다.
3. packet/camera/pose reference는 shadow A/B로 비교한다. LAN clock·실제 ray time 불확실성을 해결하기 전 production stamp를 바꾸지 않는다.
4. 보행자 association 후보를 실제 근접 두 사람 교차·가림·새 약한 관측에서 검증한다. 중복 박스를 없애려고 가까운 별개 사람을 합치지 않는다.
5. 완료 5K/v150을 보존한 채 전방 근접·얇은 barrier·희귀 asset support를 새 배치 실험으로 보강한다. 중거리 지원량만 늘리면 해결된다고 가정하지 않는다.
6. 독립 평가 split과 짧은 학습 비교 뒤 최종 학습한다. 학습되는 query 구성과 추론 구성을 일치시키고 개발 bag을 train으로 돌려쓰지 않는다.

단일 bag에 맞춰 threshold·거리·기억을 반복 조절하는 방식은 피한다.
검증을 기다리는 후보와 출력 동일 소스 개선, 실제 배포 완료 상태를 문서에서 구분한다.
