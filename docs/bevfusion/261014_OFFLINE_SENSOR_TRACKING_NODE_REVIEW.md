# 센서 시간축·중복 검출·추적 연속성·노드 비용 검토

> 2026-10-15 읽기 안내: 아래 ‘기본값 미변경’과 후보 보류는 당시 오프라인/1차 live 검토 이력이다.
> 이후 LiDAR 기본값은 실제 고속/회전 검증과 사용자 승인으로 50에 전환했다.
> 원본 51 bag·가상 재조립·당시 수치와 결론은 보존한다. 현재 채택 상태는 [CURRENT](CURRENT.md#next),
> 전환 근거는 [독립 검증](261014_INDEPENDENT_VEHICLE_AND_LIDAR_50_51_REVIEW.md#같은-날-lidar-50-운영-전환),
> 실행·복귀는 [handoff](BASE5_BEVFUSION_PRODUCTION_HANDOFF.md#lidar-50-적용과-51-복귀)를 따른다.

문서 상태: EVIDENCE와 후속 검증 계획. 기준일: 2026-10-14.
MORAI를 끈 상태에서 저장된 입력으로 원인을 분리했다. 출력이 정확히 같은 후단 계산 최적화는
소스에 반영하고, 검출·association·센서 시각 후보는 운영 기본값과 분리한다.
E24 가중치, threshold, coasting, 지도 경로 선택, 기존 5K와 v150 release는 유지한다.
이 문서의 미적용 상태는 오프라인 완료 시점 이력이다. 후속
[Live 검토](261014_LIVE_PERCEPTION_CANDIDATE_REVIEW.md)에서 nearest hint와 E24 현재 박스 중심은
반영했고, 사람 ranking·50/cut0는 보류했다. 현재 실행은 [CURRENT](CURRENT.md)를 따른다.
실제 배포 기준은 [모델 CURRENT](CURRENT.md)와
[후단 CURRENT](../../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md)가 관리한다.

이번에 적용한 것은 출력 동일 후단 CPU 개선·기본 OFF 단계 진단과 시험용 adapter/분석 경로다.
박스 원점·nearest depth hint·compile·association·packet 경계 후보는 자동 배포하지 않는다.

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

### 안정화한 depth hint에서 compile 재검증

Native 745장과 dev 200장의 945장 비교 뒤, 원래 연속 dev 입력 1,570장을 다시 추론했다.
nearest eager의 독립 A/A는 **1,570/1,570 raw 출력이 정확히 같았고** 원형 C++ tracker
상태도 네 bag 모두 정확히 같았다. 원래 stamp·JPEG/XYZI hash·GT·검사 제외 공백을
유지했다. H03의 10.322초 입력 공백을 채우거나 다른 pose로 대체하지 않았다.

Compile의 radial 30m GT 대응은 V/P/O 433/112/807개로 유지됐다. 그러나 threshold 아래로
빠진 출력 2개와 yaw 예외가 남았다. 최종 detector 중심 변화 p95는 약 0.21~0.31mm,
최대 1.23mm지만 yaw 축 변화 최대는 차량 14.72°·장애물 9.85°였다. 같은 query cell의
회전 벡터 `(sin, cos)` norm이 각각 약 0.08/0.015로 작아 `atan2`가 작은 수치 변화에
민감해지는 사례다. 낮은 score만의 문제는 아니며 차량 예외의 score는 약 0.783이었다.

원형 tracker의 전체 GT 대응 수는 1,706개로 같고 평균 중심 오차는 30.300→30.292cm로
거의 같다. 하지만 아래 일부 association·잔류 예외는 전체 평균에 숨겨진다.

| 같은-class tracker 출력의 XY 차이 | 차량 | 보행자 | 장애물 |
|---|---:|---:|---:|
| 평균 | 0.273mm | 1.731mm | 2.095mm |
| p95 | 0.472mm | 0.282mm | 0.843mm |
| 최대 | 4.27cm | 8.43cm | 81.96cm |

O02에서는 score 0.100135의 장애물 검출 하나가 빠져 가까운 기존 ID 두 개의 관측 소유가
달라졌다. 81.96cm는 중복 출력을 포함한 **출력끼리의 최대 차이**이지 GT 오차가 그만큼
나빠졌다는 뜻이 아니다. 해당 primary GT 오차 악화는 13.76cm다. P01에서는 detector XY가
거의 같은데 두 track의 Mahalanobis 비용 순서가 뒤집혀 마지막 관측과 0.5초 만료가 한
프레임 늦어졌고, 중복 출력이 6→8프레임으로 늘었다. 장애물 코어의 최대 속도 차이
1.406m/s는 실제 UNKNOWN 정책이 속도를 0으로 만드는 경로이므로 planner 속도 회귀로
주장하지 않는다. 전체 성능 악화라고 단정할 근거는 아니지만 기본 ON의 동등성 인증은 실패했다.

LayerNorm/GELU를 eager로 남긴 23장 후보도 반복 내부 A/A는 같지만 yaw 차이가 남았다.
로컬 5060Ti steady 평균은 124.6→111.0ms였고 첫 compile pass에는 warmup 뒤에도
710.8ms 지연이 있었다. 새 프로세스의 eager 출력도 이전 프로세스와 조금 달라,
14.72°→3.33°를 보장된 개선율로 비교하지 않는다. 앱의 `cudnn.benchmark=True`와
알고리즘 선택은 추가 원인 가설이며, 이번 trace로 확정하지 않았다.
[PyTorch 2.7의 convolution benchmarking 설명](https://docs.pytorch.org/docs/2.7/notes/randomness.html#cuda-convolution-benchmarking)을 참고한다.

**현재 결론은 compile 기본 OFF 유지**다. 로컬 steady 비용 감소를 실제 4090 ROS 배포
인증으로 쓰지 않는다. 근거는 `nearest_compile_full_v1/summary.json`, 최종 연속 요약
`nearest_compile_continuous_summary_v1.json`, `nearest_compile_guarded_v1/summary.json`과
tracking 루트의 `nearest_variant_review_v1/`이다. CPU 요약 예외 이전의
`nearest_compile_continuous_v1/summary.json`은 최종 요약으로 사용하지 않는다.

<a id="box-origin"></a>

## 박스 바닥 좌표와 ROS 중심 좌표의 계약

새로 확인한 문제는 수집 높이나 학습 GT가 아니라 **모델 출력→ROS 전달의 기준점**이다.
MORAI 객체 자체가 공중에 뜨던 배치 문제와는 별개의 출력 계약 오류다.
MMDetection3D decoder는 geometric center에서 높이 절반을 빼 LiDAR box의 bottom center를
만든다. `bboxes_3d.tensor`의 XYZ가 이 바닥 기준인데, 앱이 이를 `center`로 내보내고
ROS DetectedObjects pose와 Marker.CUBE가 물리적 중심으로 사용했다.

Native Instance annotation 1,702 GT의 geometric center를 infos와 직접 대조했다.
같은 XY/class로 대응한 1,338쌍에서 높이 절반을 한 번 더한 결과는 다음과 같다.
검출 개수·XY 대응·yaw·shape를 다시 학습하거나 변경한 결과가 아니다.

| Z 평균 절대 오차 | 기존 바닥 XYZ를 중심으로 사용 | `z + height/2` |
|---|---:|---:|
| 차량 281쌍 | 0.940m | 0.053m |
| 보행자 578쌍 | 0.830m | 0.058m |
| 장애물 479쌍 | 0.467m | 0.060m |

Raw 모델 tensor·평가 export·기존 capture/GT·checkpoint는 보존한다. 새 서버는 원점
`lidar_bottom_v1`을 response와 runtime identity에 명시하며 숫자 자체는 바꾸지 않는다.
ROS adapter의 기본은 **`legacy_passthrough`**로 이전 DetectedObjects/Marker wire와 같다.
시험 launch에서만 `box_center_output_origin:=gravity_center_v1`을 선택할 수 있다.
태그가 없는 구 서버는 legacy에서 호환되지만 시험 변환은 추측하지 않고 실패한다.
이미 `gravity_center_v1`인 응답은 재변환하지 않아 `+h/2` 중복을 막는다. 유한한 center와
양의 height·변환 후 유한성을 검사하고 입력 response를 수정하지 않는다.

Canonical production wrapper와 `base5_production.launch`는 이 시험 인자를 노출하지 않아
기본 계약을 유지한다. 시험은 새 metadata를 제공하는 server 소스/이미지가 필요하며
로컬 ROS 설치 완료가 GPU 이미지 배포 완료를 뜻하지 않는다. 실제 bridge publication과
server response/healthcheck의 CPU 검사에서 기본 wire 불변과 한 번만 변환함을 확인했다.

실제 tracker는 yaw만이 아닌 **전체 3D TF**를 사용한다. 따라서 local Z 수정이 roll/pitch
상황에서 map XY까지 바꿀 수 있다. tracker의 Z·height EMA는 α=0.3이라 hot switch하면
이전 원점 이력이 남는다. 첫 적용은 정지 경계에서 tracker를 reset/restart하고 planner가
저장한 이전 객체가 남지 않는지 확인한다. Map-CV 차량의 future Z가 약 1.5초 후 도로
centerline으로 수렴하는 별도 정책도 있어 단순히 RViz만 고치는 문제로 설명하지 않는다.
현재 활성 planner에는 해당 standalone obstacle height gate가 없지만 활성 구성은 다시
resolve해서 검증한다. 이 후단 경계를 검증하기 전 production Z 계약은 자동 승격하지 않는다.

추가로 실제 generated ROS 메시지·`tf2_geometry_msgs.do_transform_pose`의 합성 24조건에서
변환 차이 `R[:,2] × height/2`를 검증했다. oracle 최대 오차는 3.46e-14m다. 높이 1.8m의
yaw-only 조건은 XY 변화 0이지만 roll 3°·pitch −5° 조건은 9.14cm로, 실제 주행의 측정값은
아니다. 원형 core의 planar fresh 비교는 Z 이외 기록 상태가 같았다. 중간 전환의 0.9m
원점 차이는 1/3/5/10회 관측 후 63/30.87/15.13/2.54cm 남아 `.7^n` EMA와 일치했다.
fresh reset은 첫 track부터 Z를 맞추지만 confirmation/age 이력도 초기화한다.

실제 pure Map-CV의 평평한 도로 합성 조건에서도 0.9m 초기 높이 차이가 0.5/1/1.5/2초
horizon에서 0.6/0.3/0/0m로 사라지는 현재 정책을 확인했다. TF buffer의 실제 도착·stamp
lookup, 30Hz 복제 일정, planner 저장 객체 초기화와 폐루프는 이 CPU 시험에 포함되지 않는다.
근거는 tracking 루트 `actual_tf2_core_origin_v2/report.json`과 재사용 감사
`audit_tracking_continuity_box_origin_tf.py`다.

설치된 C++ EKF 노드도 private master에서 새 process로 legacy/gravity 각 30입력을 보냈다.
현재 TF와 header 시각 TF를 서로 다르게 만들어 실제 header lookup·full-RPY·typed
world center·안정 UUID 한 개와 30Hz copy를 확인했다. copy 간격은 모두 33.333333ms였다.
원점 변환 뒤 center의 typed oracle 최대 오차는 3.55e-15m다. node의 중간 XYZ가 float32라
unquantized double과 약 14~15µm 차이가 나는 것을 타입 축소로 설명하며 넓은 tolerance로
덮지 않았다. 운영 core·node·설치본·시간 정책은 변경하지 않았다.

다만 legacy/gravity 출력 수는 각각 100/94개이며 gravity 시작에 TF past-extrapolation 경고
1개와 초기 확인 2입력 지연이 남았다. clock latch 정리 뒤에도 재현되어 startup negotiation/
clock-clear 원인을 이 bounded gate로 확정하지 못했다. **전체 30입력 소비·TF 누락 0·안전한
planner handoff 인증은 아니다.** 모든 소유 master/node process group은 종료했다.
음성 근거를 포함한 최종 결과는 `private_cpp_origin_v3/report.json`과 두 node log이며,
재사용 도구는 `audit_tracking_continuity_box_origin_private_ros.py`다.

근거는 `box_origin_native_v2.json`, tracking 루트의 `z_origin_downstream_v1/assessment.json`,
재사용 감사 `audit_base5_offline_box_center_origin.py`다. 이 수정 때문에 데이터 재수집이나
재학습이 필요하다는 근거는 없다. 실제 3D TF·Map-CV·planner 상태 handoff 검증은 남아 있다.

## 다음 생성·학습 실험

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
해당 spike의 generation-2 GC는 85.285ms다. 다만 이 시험은 bag과 출력이 미리 로드된
heap을 사용했다. 이것이 실제 ROS 노드에서도 발생하는 tail인지 별도로 확인했다.
근거는 `final_safe_v7/report.json`, `isolated_safe_candidate_v8/`에 있다.

### 실제 설치 노드의 연속 입력·자연 GC 대조

최신 bag의 연속 5,461프레임을 원래 header cadence로 스트리밍하고 실제 설치된 Map-CV를
private master에서 실행했다. child에 입력/출력 전체를 쌓지 않고 원래 GC 설정을 유지했다.
**5,461/5,461 출력 wire가 기준과 같고**, nonempty 2,406프레임도 전부 전달됐다.
자연 generation-2 GC 두 번의 최대는 9.389ms, callback wall p99/최대는
5.156/13.135ms로 20ms 초과 callback은 없었다. child RSS는 첫 callback 약
383,620KiB→종료 389,580KiB였다. 미리 로드한 heap의 87ms tail을 재현하지 못했으며
모든 풀스택 GC tail이 해결됐다는 결론은 아니다.

3배속에서는 5,461개 중 callback 4,469·출력 4,462개로 queue 포화가 생겼지만 받은
출력은 정확한 subset이다. callback 최대 26.921ms·20ms 초과 4건은 generation-2 GC와
겹치지 않았다. 서로 다른 master/관측자 부하를 포함하므로 실제 대회 drop 수치로 쓰지 않는다.

시험 도구의 `driver_delivery_wall_ms`는 publish 시작부터 **부모의 수신 wire 검증 완료**까지다.
평균 2.178ms·p95 8.698ms·최대 98.895ms는 관측자 scheduling/직렬화 검증까지 포함하며
callback 계산 또는 downstream 전달 완료 시간만이 아니다. 최종 보고서에 token별 쌍이
남지 않아 이 최대 원인이나 quantile 간 차이를 분해하지 못한다. TCP/Nagle·네트워크 원인으로
단정하거나 transport 설정을 바꾸지 않는다. 다음 시험은 bounded token ledger에 publish
시작/반환·observer 진입/검증 끝·child callback 경계를 기록한다.
근거는 `live_heap_gc_full_v3/report.json`이며 초기 harness의 pacing이 잘못된 v2는 사용하지 않는다.

이 한계를 보완한 다음 실행은 bounded token ledger로 publish 시작/반환·child callback
시작/objects publish 반환/끝·observer 진입/wire 검증 끝을 같은 token에 연결했다.
완료 token은 즉시 버리고 pending 512·큰 tail 4개만 유지한다. percentile용 scalar duration은
별도 집계하며 진단 누락도 실제 값으로 남긴다. 독립 full 1×의 5,461개 출력·callback·진단·
paired join 모두 일치했고 nonempty 2,406개를 전부 전달했다. wire 회귀와 token evict는 0이다.

| 같은 프레임의 큰 tail 분해, ms | callback 시작 전 | callback→objects publish 반환 | publish 반환→observer 진입 | wire 검증 | 전체 |
|---|---:|---:|---:|---:|---:|
| 가장 큰 창 | 4.557 | 3.915 | 77.331 | 0.208 | 86.012 |
| 두 번째 창 | 78.808 | 4.688 | 0.763 | 0.178 | 84.437 |

해당 창에 generation-2 GC는 없고 generation-0 GC도 각각 약 0.062/0.051ms다.
긴 구간이 어디인지는 분리했지만 receiver 역직렬화·TCP·host/observer scheduling 중
무엇인지는 확정하지 않았다. 첫 구간에 포함되는 sender publish call을 다시 더하거나
독립 p95끼리 빼지 않는다. publish 반환 전에 receiver가 움직일 수 있어 concurrent 경계의
음수 차이를 0으로 보정하지 않는다. 이 run은 이전 98.895ms 창을 소급 분해한 것이 아니다.
최종 근거는 `live_heap_gc_paired_v4/report.json`이다. 원 보고서의 동시 GPU 작업 문구는
이 run에 맞지 않아 `scope_clarification.json`으로 정정했고 원본 fingerprint는 보존했다.

### Private ROS TCP 저지연 옵션의 제한된 대조

같은 연속 중간 1,200프레임을 기본→input/output Subscriber의 `tcp_nodelay` ON→기본으로
재생했다. 세 번 모두 출력·callback·진단·paired 1,200개, nonempty 447프레임·680객체와
wire가 같고 누락은 0이다. child input의 실제 요청도 false/true/false를 trace로 확인했다.
clock·GC·운영 소스/transport는 바꾸지 않았으며 연구 도구의 옵션도 기본 false다.

| parent 송신→observer wire 검증, ms | 평균 | p50 | p95 | p99 | 최대 |
|---|---:|---:|---:|---:|---:|
| 기본 1 | 1.226 | 0.484 | 5.149 | 11.001 | 23.215 |
| private ON | 1.156 | 0.488 | 4.723 | 9.955 | 12.016 |
| 기본 2 | 1.210 | 0.485 | 5.395 | 10.243 | 20.713 |

평균 이득은 약 5%로 작고 관측 최대는 줄었지만 각 조건 1회다. output 구간의 p95는
0.488/0.476/0.428ms라 ON이 항상 우월하지 않다. 이 창은 앞의 full-bag 86ms 창과 같지
않고, Nagle이 237ms 원인이거나 모든 tail이 해결됐다고 해석하지 않는다. 실제 LAN과
planner 소비까지 확인할 연구 후보로만 보존한다. 근거는 `transport_nodelay_aba_v1/`이다.

GPU client/server의 별도 TCP protocol도 실제 `TcpTransport`와 loopback stub으로 검사했다.
현재 코드는 이미 16-byte header·JSON·모든 blob을 하나의 bytes로 합쳐 `sendall` 한 번으로
보낸다. immutable 20개 JPEG/XYZI fixture에서 request 평균 약 605KB·frozen E24 response
649B를 사용했고 초기 교환을 포함한 820개 request/blob/response 동등성을 확인했다.
실제 on-wire 녹화가 아니라 기존 prepared 입력으로 schema를 재구성한 transport 시험이다.

| stub 조건 | 현재 one-write 평균 RTT | 양쪽 NODELAY |
|---|---:|---:|
| 계산 대기 0ms | 0.216ms | 0.194ms |
| 인위적 대기 100ms | 100.704ms | 100.709ms |

일부러 header를 분리한 음성 대조는 약 43.86ms로 늘고 NODELAY에서 0.276ms로 줄었다.
그러나 그 분리 방식은 현재 구현이 아니다. 현행 framing의 40ms 대기 증거도, TCP 옵션으로
큰 GPU 왕복 비용을 줄일 근거도 이 loopback에서 얻지 못했다. NN·GPU·ROS·Docker/LAN을
포함한 시험이 아니므로 실제 237ms 원인을 확정하거나 배제하지 않는다. production protocol/
socket은 미변경이다. 모델 근거 루트 `tcp_framing_loopback_v1.json`·`v2.json`과 재사용
`audit_base5_offline_tcp_framing.py`를 따른다. v1 byte 통계의 잘못된 `over30ms` 필드는
`tcp_framing_units_correction_note_v1.json`에 정정했으며 v2는 단위를 분리한다.

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

BEVFusion bridge의 Python rotated NMS도 실제 순서인 server floor 0.1/top100→
V/P/O 0.4/0.1/0.1 필터→NMS로 4,985프레임의 박스 기하 캐시를 비교했다.
945 native/dev의 NMS 입력은 평균 2.03·최대 10개, 4,040 packet 입력은 평균 1.85·
최대 6개였다. 기존 평균 0.00624/0.00380ms가 후보 0.00743/0.00512ms로 오히려 늘었다.
retained index·객체 identity·순서·입력 무변경은 같지만 이득이 없어 기각했다.
최대 100개 설정이 실제로 매 프레임 100개를 처리한다는 뜻은 아니다.
운영 NMS는 미변경이며 `rotated_nms_actual_bridge_v2/report.json`을 최종 근거로 쓴다.
class 필터 전 비용을 본 기존 v1은 상한 참고용으로 보존하며 실제 bridge 비용과 구분한다.

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

1. Versioned ROS 중심 adapter의 CPU tf2·원형 EMA·Map-CV 높이와 설치 EKF의 TF/30Hz 제한 대조는 완료했다. startup TF 누락·실제 reset·planner 상태 handoff를 검증한다. raw export/학습 GT는 보존한다.
2. Depth hint 안정화의 반복 검증은 완료했다. compile 연속 tracker 예외 때문에 기본 OFF를 유지하고, nearest는 독립 ROS trial 뒤 승격 여부를 결정한다. 현재 E24의 proposal peak 사후 변경은 기각했다.
3. 출력 동일 CPU 개선·단계 진단의 로컬 설치와 private ROS 전달 검증을 완료했다. 팀장님 배포와 실제 풀스택 age/drop을 확인한다.
4. packet/camera/pose reference는 shadow A/B로 비교한다. LAN clock·실제 ray time 불확실성을 해결하기 전 production stamp를 바꾸지 않는다.
5. 보행자 association 후보를 실제 근접 두 사람 교차·가림·새 약한 관측에서 검증한다. 중복 박스를 없애려고 가까운 별개 사람을 합치지 않는다.
6. 완료 5K/v150을 보존한 채 전방 근접·얇은 barrier·희귀 asset support를 새 배치 실험으로 보강한다. 중거리 지원량만 늘리면 해결된다고 가정하지 않는다.
7. 독립 평가 split과 짧은 학습 비교 뒤 최종 학습한다. 학습되는 query 구성과 추론 구성을 일치시키고 개발 bag을 train으로 돌려쓰지 않는다.

단일 bag에 맞춰 threshold·거리·기억을 반복 조절하는 방식은 피한다.
검증을 기다리는 후보와 출력 동일 소스 개선, 실제 배포 완료 상태를 문서에서 구분한다.
