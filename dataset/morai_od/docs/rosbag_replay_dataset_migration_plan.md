# ROS bag 기반 MORAI 데이터셋 전환 검토

## 목적

현재 production 수집기는 MORAI `SaveSensorData`를 사용한다. 이 방식은 한 요청으로
4-camera, LiDAR와 GT 파일을 묶어 저장하기 쉽고 대규모 scenario 자동화가 이미 검증됐다는
장점이 있다. 반면 실제 대회 online 입력은 camera와 LiDAR가 서로 다른 주기로 독립 도착한다.
학습 데이터만 동기화 snapshot으로 구성하면 다음 분포 차이가 남는다.

- 학습에서는 camera와 LiDAR가 같은 capture event에 속하지만 실제 추론에서는 각 센서
  timestamp가 다르다.
- 대회에서 LiDAR는 nominal 15 Hz보다 낮은 약 11--12 Hz로 들어올 수 있고 camera는 약
  20--30 Hz로 도착한다.
- 움직이는 Ego와 NPC에 대해 수십 ms의 camera--LiDAR 차이가 image projection, painting,
  box GT와 velocity 학습 오차로 바뀔 수 있다.
- Temporal 모델은 실제 sweep 간격과 frame drop을 보지 못하면 online에서 다른
  `time_lag_sec` 분포를 만난다.

따라서 rosbag을 원본 기록 형식으로 사용하는 경로를 열어 둔다. 목표는 `SaveSensorData`를
즉시 폐기하는 것이 아니라, 동일 scenario를 재현 가능한 raw asynchronous stream으로
보존하고 필요한 학습 형식을 후처리로 만드는 것이다.

## rosbag이 필요한 이유

### 실제 센서 주기와 도착 패턴 보존

rosbag은 camera, LiDAR, Ego, Object, IMU와 GPS를 각각의 timestamp와 arrival order로
저장한다. 다음 항목을 학습·검증 데이터에 그대로 반영할 수 있다.

- 실제 camera/LiDAR Hz와 jitter.
- LiDAR keyframe에서 각 camera까지의 signed time lag.
- 누락 frame과 긴 gap.
- sweep 간 Ego 이동량과 회전량.
- NPC의 위치·yaw·속도 이력.

학습 extractor는 임의의 고정 12 Hz snapshot을 새로 만들어내는 대신 실제 LiDAR sweep을
keyframe으로 삼는다. Temporal 입력은 목표가 `0, -0.1, -0.2초`라고 하더라도 실제 선택된
sweep의 정확한 `time_lag_sec`를 함께 저장한다.

### Ego-motion alignment와 deskew

rosbag에 연속된 Ego pose가 있으면 sweep 간 정합은 방위각이나 이동속도를 역산하지 않고
직접 계산할 수 있다. 이전 sweep의 point를 현재 sweep 좌표계로 옮기는 기본식은 다음과
같다.

```text
p_ego_current
  = inverse(T_global_ego(current_lidar_time))
  * T_global_ego(source_lidar_time)
  * p_ego_source
```

이는 multi-sweep PointPainting, BEVFusion과 이후 temporal BEV 모델에 공통으로 사용할 수
있다. 학습과 online 추론에서 같은 pose interpolation과 transform 구현을 사용해야 한다.

2026-08-22 대회용 raw bag
`260822_005713_competition_full_route_raw_hz_run01.bag`을 직접 검사한 결과
`/sensor/lidar1`의 `PointCloud2`에는 `x, y, z, intensity, ring, time` field가 모두 있었다.
첫 scan의 point별 time은 `0.0--0.08253065초` 범위였다. 따라서 rosbag 경로에서는
scan header와 방위각으로 point time을 역산하지 않고 각 point 측정 시각의 Ego pose를
interpolation하는 exact Ego deskew를 구현할 수 있다.

- `point_time = lidar_header_stamp + point.time`의 의미와 기준점은 여러 scan에서 다시
  검증한다.
- 해당 시각의 Ego pose는 `/Ego_topic`, localization pose 또는 simulator pose history에서
  interpolation한다.
- 이전 100-frame 검증에서 raw보다 소폭 악화된 것은 SaveSensorData point cloud에 time
  field가 없어 방위각으로 시간을 근사한 실험이었다. rosbag의 실제 point-time 기반
  deskew와는 다른 실험이다.
- 이로써 rosbag의 1차 이점은 **센서 간 시간 보정, sweep 간 Ego 정합, scan 내부 exact
  Ego deskew**까지 확장된다. moving-object deskew는 객체별 motion을 별도로 알아야 하므로
  여전히 다른 문제다.

## nuScenes 방식에 대응하는 후처리

nuScenes는 sample data별 sensor timestamp, calibrated sensor extrinsic과 해당 시각의
Ego pose를 분리해 저장한다. LiDAR point를 다른 시각의 camera로 투영할 때 단순히 두 센서가
동시라고 가정하지 않고 global frame을 경유한다. 우리 extractor도 같은 구조를 사용한다.

LiDAR 시각 `t_L`의 point를 camera 시각 `t_C`로 투영하는 transform chain은 다음과 같다.

```text
LiDAR(t_L)
  -> Ego(t_L)       : LiDAR extrinsic
  -> Global         : Ego pose at t_L
  -> Ego(t_C)       : inverse Ego pose at t_C
  -> Camera(t_C)    : inverse camera extrinsic
  -> Image          : camera intrinsic
```

4개 camera는 각자 timestamp가 다를 수 있으므로 하나의 공통 camera timestamp로 뭉개지
않는다. frame metadata에는 최소한 다음 값을 저장한다.

- LiDAR keyframe timestamp와 bag receive timestamp.
- 각 camera의 sensor timestamp와 LiDAR 대비 signed lag.
- Ego pose at LiDAR time.
- Ego pose at each camera time.
- interpolation에 사용한 두 Ego sample과 interpolation ratio.
- sensor extrinsic, intrinsic과 calibration version.
- 선택 정책(`latest_causal`, `nearest_offline`)과 sync quality flag.
- 이전 sweep timestamp, 실제 `time_lag_sec`, relative Ego transform.
- GT object pose/velocity timestamp와 LiDAR 시각으로 보정했는지 여부.

### Causal 학습·추론 일치

offline에서 절대시간이 가장 가까운 camera를 선택하면 LiDAR보다 미래의 image가 포함될 수
있다. 실제 online inference에서는 아직 도착하지 않은 미래 image를 사용할 수 없다.
production 학습·평가 기본 정책은 다음과 같다.

- LiDAR keyframe 시각까지 도착한 camera 중 최신 frame을 선택한다.
- sensor timestamp와 arrival timestamp를 모두 보존한다.
- 허용 lag를 넘으면 frame을 억지로 복제하지 않고 missing/low-quality로 표시한다.
- 모델이 time lag를 입력받을 수 있으면 signed lag를 feature로 제공한다.
- nearest-offline 정책은 진단용 upper bound로만 별도 평가한다.

## Replay를 이용한 자동 수집 가능성

### 원본 기록 계약

`mrp`는 개인용 2026 launcher, `mr`은 대회용 2025 launcher를 뜻한다. 별도 언급이 없으면
rosbag은 `mrp`에서 녹화한다. 모든 `mrp` raw bag에는 `/ReplayInfo_topic`을 포함한다.

권장 raw bag은 다음을 함께 저장한다.

- `/ReplayInfo_topic`.
- `/Ego_topic`, `/Object_topic`.
- 4-camera compressed image.
- `/sensor/lidar1`.
- GPS, IMU, collision, traffic-light와 simulator status.
- `/clock`, `/tf`, `/tf_static`이 제공되면 함께 저장.

Replay를 위해 필요한 최소 payload는 `/ReplayInfo_topic`이지만, 나머지 raw sensor와 상태
토픽은 원본 주행을 직접 평가하고 replay 재현성을 검증하는 기준이 된다.

### Network Replay 검증

Network Replay는 ROS `rosbag play`가 `/ReplayInfo_topic`을 rosbridge를 통해 MORAI에
주입한다. 자동화는 쉽지만 Simulator의 ReplayInfo Publisher를 같은 토픽에 두면 약 50 Hz
입력과 약 50 Hz 출력이 합쳐져 약 98 Hz feedback 경로가 생겼다. 출력 토픽을
`/ReplayInfo_topic_out`으로 분리해 해결했다.

2026-08-22 temporary Ego replay 비교 결과는 다음과 같다.

| 항목 | 원본 | Direct | Network |
| --- | ---: | ---: | ---: |
| 활성구간 출력률 | 49.98 Hz | 50.00 Hz | 46.91 Hz |
| 1 ms 이내 연속 메시지 | 0.2% | 0.2% | 22.5% |
| 이전 source 상태로 후퇴 | 기준 | 0.58% | 8.41% |
| source frame 2개 초과 skip | 기준 | 5.73% | 11.25% |
| 재생 궤적 누적거리 | 540.40 m | 542.87 m | 706.86 m |

두 방식 모두 원본 공간 경로 위에 있었지만 Network는 앞뒤 상태 재출력 때문에 시각적으로
떨렸고 누적거리가 부풀었다. topic feedback 제거 후 떨림은 크게 줄었지만 temporal/velocity
학습용 production으로 즉시 채택하기에는 상태 순서 문제가 남았다.

### Direct Rosbag Replay 자동화

Direct Replay는 simulator가 bag을 직접 읽어 Network Replay보다 상태 순서가 안정적이었다.
MORAI의 `/Morai_SimProc` 서비스는 다음 기능을 제공한다.

- bag load.
- Ego/NPC/Pedestrian replay target 선택.
- start time과 replay speed 설정.
- play, pause, stop.
- `/sim/process/state/msg/MoraiSimProcStatus`를 통한 성공·종료 확인.

제약은 서비스의 replay option이 `current_mode=0x10`인 Replay Mode에서만 유효하다는 점이다.
공식 명세만 보면 최초 1회 Replay Mode 진입 이후 bag load부터 batch 반복까지 자동화할 수
있어 보이지만, 아래 live smoke에서 개인용 2026 launcher의 실제 동작이 명세와 일치하지
않았다. 따라서 가능하다고 가정하지 않고 launcher-version compatibility 문제로 보류한다.

자동 runner의 상태 기계는 다음과 같다.

```text
verify rosbridge/service/status
  -> verify current_mode == Replay
  -> stop previous replay
  -> load bag + select targets + speed/start time
  -> wait for load success
  -> play
  -> capture at configured policy
  -> detect replay EOF
  -> validate artifacts and metadata
  -> checkpoint
  -> next bag
```

Direct Replay의 built-in `Sensor Frame Rate` 저장과 기존 `/SaveSensorData` trigger 방식은 작은
smoke에서 비교한다. custom filename, frame-level sidecar와 실패 복구가 필요한 production은
제어 가능성이 더 높은 방식을 선택한다.

### 2026-08-22 `/Morai_SimProc` live 시행착오

개인용 2026 launcher(`mrp`)에서 rosbag 파일을 launcher의
`SaveFile/Rosbag`에 설치하고, UI에서 compatible ReplayInfo-only bag을 직접 load한 뒤
Replayer Control Panel을 paused 상태로 열어 시험했다. rosbridge와 다음 endpoint는 모두
ROS graph에 보였다.

- service `/Morai_SimProc`.
- status `/sim/process/state/msg/MoraiSimProcStatus`.
- Direct Replay output `/ReplayInfo_topic_out`.

하지만 status publisher는 seq와 header stamp만 계속 갱신하면서 다음 값을 항상 0으로
보냈다.

```text
current_mode=0
current_status=0
command_result=0
latest_command_time=0
```

UI에서 실제 Direct Replay mode와 paused control panel을 확인한 상태에서도 같았다. 따라서
이 launcher build에서는 status topic을 mode/EOF gate로 사용할 수 없다.

공식 bitmask를 사용해 `load + target + play`를 요청했다.

```text
sim_process_status = 0x01
replay_option = 0x0011
replay_target_option = 0x0111  # Ego + NPC + pedestrian
replay_speed = 1
start_time = 0
```

ROS TCP service 연결은 성립했지만 MORAI는 5초, 60초, 이후 사용자가 시험한 120초
timeout에서도 service response를 반환하지 않았다. 단순 pause 요청도 즉시 응답하지 않았다.
그러나 timeout을 실패로 오인해 같은 load 명령을 여러 번 보낸 뒤 약 30초 이상 지나자,
MORAI Replay 창이 여러 차례 연속 새로고침되며 각 load 요청을 늦게 처리했다. 즉 다음 두
현상이 동시에 있었다.

- ROS service response는 돌아오지 않거나 비정상적으로 오래 지연됐다.
- request side effect는 MORAI 내부 queue에 남아 뒤늦게 순차 실행됐다.

따라서 service timeout은 명령 미전달을 의미하지 않는다. timeout 직후 재시도하면 duplicate
load가 queue에 쌓인다. 이미 전달된 queue는 rosbridge client를 닫아도 MORAI 내부에 남을 수
있으므로, 반복 새로고침이 멈추지 않으면 MORAI 재시작이 가장 확실한 초기화다.

또한 combined `load + play` 요청은 파일 load/UI refresh만 일으키고 자동 재생으로 이어지지
않았다. load 완료 후 별도 resume를 보내는 2-stage runner도 구현했지만 load service가
120초 내에 정상 response를 주지 않아 안전하게 다음 명령을 보낼 completion signal이 없다.
status 값도 0이라 EOF와 load completion을 대체 판단할 수 없었다.

실험용 `morai_direct_replay_runner.py`에는 다음 safeguard를 넣었다.

- status 0 build를 명시적으로만 허용하는 `--allow-unreported-mode`.
- 장시간 service timeout.
- timeout 시 즉시 재시도 금지 경고.
- 정상 load response 뒤에만 resume를 한 번 보내는 2-stage flow.
- bag 존재와 target bitmask 검증.

현재 결론은 다음과 같다.

- Direct Replay 수동 재생은 가능하고 Network보다 상태 순서가 안정적이다.
- `mrp`의 `/Morai_SimProc` 기반 unattended batch 자동화는 현재 검증 실패다.
- status와 service response가 정상인 다른 MORAI build에서만 runner를 다시 시험한다.
- GUI 자동조작은 취약성과 복구 난이도 때문에 production 대안으로 채택하지 않는다.
- Network Replay는 자동화 가능하지만 state backtracking 때문에 temporal production으로
  채택하지 않는다.
- 당분간 production 수집은 검증된 ScenarioLoad + SaveSensorData Temporal V2를 유지한다.

## 제안하는 migration 단계

1. 현재 `mrp`에서는 `/Morai_SimProc` 자동화를 보류하고 SaveSensorData Temporal V2를
   계속 수집한다.
2. 다른 MORAI build 또는 공식 지원 확인 시 service response, status mode와 EOF를 가장
   먼저 재검증한다. timeout된 load는 절대 즉시 재시도하지 않는다.
3. Direct 수동 Replay에서 8-frame sensor capture를 실행하고 4-camera, LiDAR, GT 완전성을
   검사한다.
4. 원본, Direct, Network를 Ego/NPC 위치·yaw·속도와 상태 순서로 반복 비교한다.
5. raw bag extractor를 구현해 LiDAR keyframe, causal camera association와 pose
   interpolation metadata를 만든다.
6. 기존 `SaveSensorData` dataset과 동일 model/config로 A/B한다.
7. online ROS adapter에도 같은 causal association과 Ego-motion transform을 적용한다.
8. detection AP뿐 아니라 velocity error, tracking error와 1/2/3초 prediction FDE를
   비교한다.
9. 성능과 재현성이 확인된 뒤에만 production 수집을 rosbag-first로 전환한다.

## Production 결정 기준

rosbag 전환은 다음 조건을 모두 만족할 때 수행한다.

- 반복 replay에서 Ego/NPC pose와 timestamp 순서가 안정적이다.
- camera/LiDAR association의 lag 분포가 실제 대회 raw bag과 유사하다.
- sensor/GT 누락률과 자동 복구율이 기존 SaveSensorData보다 나쁘지 않다.
- temporal velocity와 downstream tracking/prediction이 유의미하게 개선된다.
- raw bag 용량, extraction 시간과 보관 정책을 감당할 수 있다.

그 전까지는 `SaveSensorData Temporal V2`가 production이고 rosbag 경로는 실험·독립 test
set·차세대 temporal model을 위한 병행 경로다.

## JPEG train--inference A/B gate

실제 대회 raw bag의 네 `sensor_msgs/CompressedImage`를 검사한 결과 MORAI image payload는
JPEG quality 90, 4:2:0 chroma subsampling이었다. rosbag은 이 byte payload를 재압축하지
않고 그대로 저장한다. 반면 현재 SaveSensorData의 RGB `Intensity.png`는 lossless PNG다.
따라서 rosbag-first 전환 전에 다음 고정 split·seed 실험을 수행한다.

| Train | Test | 목적 |
| --- | --- | --- |
| PNG | PNG | 기존 offline 기준. |
| PNG | JPEG Q90 | 현재 모델의 compression distribution shift 측정. |
| JPEG Q90 | JPEG Q90 | production-matched 학습 효과. |
| JPEG quality 85--95 random | 실제 ROS JPEG | 제한적 compression robustness 검증. |

동일 PNG를 한 번만 Q90/4:2:0으로 encode/decode하여 compression만 격리한다. 이미 JPEG인
ROS frame을 다시 encode하는 double compression은 이 실험에 사용하지 않는다. 전체 mAP
외에 20--30 m pedestrian/obstacle AP, LRASPP mIoU와 boundary quality를 별도로 본다.
PointPainting은 LiDAR geometry가 있어 영향이 제한적일 가능성이 높고 camera-only Fast-BEV는
상대적으로 더 민감할 수 있다. 최종 결정은 실제 ROS JPEG test 결과로 내린다.
