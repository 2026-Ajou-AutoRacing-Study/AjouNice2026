# MORAI Sensor_4 기반 4-camera BEVFusion zero-shot 연동

> **문서 상태: HISTORICAL.** 초기 feasibility 기록이며 현재 E25 실행 계약이 아니다.

## 1. 목표와 범위

이번 작업의 목표는 MORAI `Sensor_4.json`의 카메라 4개와 VLP-16 LiDAR를
공식 NuScenes BEVFusion checkpoint에 입력하고, zero-shot 3D detection을
RViz에서 차량 이동 중 확인할 수 있게 만드는 것이다.

이번 단계는 다음을 포함한다.

- Sensor JSON 기반 topic 및 calibration 구성
- LiDAR 기준 근접 timestamp 동기화
- 호스트 ROS와 Docker 추론 서버 사이의 localhost TCP protocol
- calibration projection debug 영상
- 4-camera BEVFusion inference
- RViz용 3D box·class·score MarkerArray
- 서버 상태와 지연시간 topic

다음은 포함하지 않는다.

- MORAI 데이터 파인튜닝과 mAP/NDS 정량 평가
- multi-object tracking과 안정적인 객체 속도
- Planning용 최종 perception message 계약
- 기존 localization·planning·control 코드 변경
- MMDetection3D 서브모듈 추가 변경

## 2. 전체 구조

```text
MORAI 운영 PC
  └─ 대회 규정의 UDP 센서 인터페이스
       ↓
참가자 PC / ROS Noetic
  ├─ camera CompressedImage × 4
  ├─ lidar PointCloud2 × 1
  └─ Sensor_4 static TF
       ↓
bevfusion_ros_bridge.py
  ├─ LiDAR stamp 기준 최근접 camera 선택
  ├─ 최신 complete frame 하나만 유지
  ├─ JPEG와 float32 point cloud 직렬화
  └─ 127.0.0.1 TCP client
       ↓
Docker inference_server.py
  ├─ Sensor_4 calibration
  ├─ projection 또는 BEVFusion inference
  └─ detection 및 debug image 반환
       ↓
ROS bridge
  ├─ MarkerArray
  ├─ projection CompressedImage
  ├─ latency
  └─ status
       ↓
RViz
```

ROS Noetic의 Python 환경과 최신 PyTorch·CUDA·MMDetection3D 환경을 한
프로세스에 섞지 않는다. ROS message 처리는 호스트가 담당하고 GPU 추론은
재현 가능한 Docker 이미지가 담당한다.

## 3. 생성·수정·제거 파일

### 생성

```text
docker/bevfusion/app/
├── __init__.py
├── protocol.py
├── sensor4_calibration.py
├── runtime_sample.py
├── bevfusion_model.py
└── inference_server.py

docker/bevfusion/scripts/
├── run_inference_server.sh
└── healthcheck.py

docker/bevfusion/tests/
├── test_protocol.py
├── test_sensor4_calibration.py
└── test_tcp_integration.py

docker/bevfusion/
├── compose.yaml
├── .env.example
├── .gitignore
└── README.md

configs/perception/bevfusion_sensor4.yaml

base_ws/src/perception/bevfusion_ros/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── src/bevfusion_ros/{__init__.py,protocol.py,transport.py}
├── scripts/bevfusion_ros_bridge.py
├── launch/{projection_check.launch,zero_shot.launch}
├── rviz/bevfusion_zero_shot.rviz
└── README.md

base_ws/src/bringup/launch/perception_bevfusion.launch
docs/bevfusion/260720_morai_four_camera_zero_shot_{full,short}.md
```

### 수정

- `base_ws/src/bringup/package.xml`: `bevfusion_ros` 실행 의존성 추가

### 제거

- 없음

기존 팀원 파일과 추적되지 않은 `docs/.gitignore`는 변경하지 않았다.

## 4. Sensor_4 입력 계약

센서 수치의 단일 기준은 `configs/sensors/Sensor_4.json`이다. perception YAML에
topic·위치·회전·해상도·FOV를 다시 하드코딩하지 않는다.

| 순서 | frame | topic | 해상도 | 수평 FOV |
|---:|---|---|---:|---:|
| 1 | `camera_front` | `/sensor/camera/front/compressed` | 1280×720 | 90° |
| 2 | `camera_front_left` | `/sensor/camera/front_left/compressed` | 640×480 | 130° |
| 3 | `camera_front_right` | `/sensor/camera/front_right/compressed` | 640×480 | 130° |
| 4 | `camera_back` | `/sensor/camera/back/compressed` | 1280×720 | 90° |
| LiDAR | `lidar1` | `/sensor/lidar1` | PointCloud2 | VLP-16 |

YAML은 score threshold, camera semantic order, 동기화 허용치, 서버 주소처럼
알고리즘 실행 정책만 가진다.

## 5. Calibration

### 5.1 Extrinsic

각 센서 JSON의 `pos`와 `rot`를 `base_link ← sensor` 강체 변환으로 해석한다.
LiDAR와 카메라가 동일한 `base_link` 부모를 사용하므로 다음처럼 계산한다.

```text
camera_ros ← lidar
= inverse(base_link ← camera_ros) × (base_link ← lidar)
```

그 후 ROS 센서 좌표를 OpenCV optical 좌표로 바꾼다.

```text
ROS:     x=forward, y=left, z=up
OpenCV:  x=right,   y=down, z=forward

[x_cv, y_cv, z_cv] = [-y_ros, -z_ros, x_ros]
```

최종적으로 `lidar2cam`, `cam2lidar`, `lidar2img`를 만들고 단위 테스트에서
`lidar2cam × cam2lidar ≈ I`를 검사한다.

### 5.2 Intrinsic

현재 JSON의 `focalLengthpixel=320`은 일부 해상도·FOV 조합과 일치하지 않는다.
기본값은 JSON의 해상도와 수평 FOV로 계산한다.

```text
fx = width / (2 × tan(horizontal_FOV / 2))
fy = fx
cx, cy = JSON principalPoint
```

이는 잠정적인 pinhole 모델이다. 렌즈 왜곡과 MORAI 내부 projection 정의가
다르면 projection 영상을 기준으로 수정해야 한다. 그래서 모델 추론 전에
`projection_check.launch`를 반드시 실행한다.

## 6. 센서 동기화와 부하 제어

LiDAR callback이 한 inference frame의 기준 timestamp를 정한다. 카메라별
ring buffer에서 가장 가까운 이미지를 선택하고 차이가 기본 0.10초보다 크면
그 frame은 버린다.

BEVFusion 처리율은 센서 주기보다 느리므로 FIFO queue를 만들지 않는다.
worker가 추론 중일 때 새 complete frame이 도착하면 아직 처리하지 않은 이전
frame을 교체한다. 이 `latest-frame-only` 방식은 throughput을 늘리지는 않지만
차량 위치보다 수 초 뒤처진 box가 나타나는 것을 방지한다.

PointCloud2는 Docker에 다음 순서의 little-endian float32 배열로 전달한다.

```text
x, y, z, intensity, relative_time
```

필드가 없으면 intensity 또는 relative time은 0으로 채운다. 첫 실시간 테스트 후
PointCloud 필드 목록과 intensity 범위를 반드시 확인해야 한다.

## 7. TCP 설계

### 7.1 TCP가 필요한 이유

ROS bridge와 Docker inference server는 서로 다른 프로세스이므로 IPC가
필요하다. Docker에는 PyTorch 2.7.1, CUDA 12.8, MMDetection3D가 있고 호스트에는
ROS Noetic Python 환경이 있다. 한쪽 환경에 다른 쪽 stack을 강제로 합치면
Python·CUDA·ROS 의존성 충돌과 배포 재현성 문제가 커진다.

TCP를 선택한 이유는 다음과 같다.

- Python 표준 라이브러리만으로 구현 가능
- 연결·timeout·오류·재연결 동작이 명확함
- Docker port publishing이 단순함
- 향후 참가자 PC 내부의 별도 GPU process로 분리하기 쉬움
- binary JPEG와 point cloud를 persistent connection으로 전송 가능

프레임마다 TCP 연결을 새로 만들지 않고 하나의 연결을 재사용한다. 오류나
timeout이 발생하면 연결을 닫고 다음 요청에서 다시 연결한다.

### 7.2 IP와 포트

호스트가 외부 LAN에 포트를 열지 않도록 Compose에서 다음처럼 publish한다.

```yaml
ports:
  - "127.0.0.1:29500:29500"
```

- 호스트 client: `127.0.0.1:29500`
- 컨테이너 server: 컨테이너 내부 `0.0.0.0:29500`
- MORAI 운영 PC 주소로 TCP 연결하지 않음
- `0.0.0.0:29500:29500` 형태의 호스트 publish를 사용하지 않음
- `network_mode: host`를 사용하지 않음

컨테이너 내부의 `0.0.0.0` listen은 Docker port forwarding을 받기 위한 것이고,
외부 노출 범위는 호스트 쪽 `127.0.0.1` binding이 제한한다.

### 7.3 패킷 구조

각 message는 network byte order의 16-byte header를 사용한다.

```text
magic[4] = AJBF
version[1]
message_type[1]
flags[2]
metadata_size[4]
payload_size[4]
JSON metadata
binary payload
```

JSON에는 blob 이름·offset·size만 넣고 실제 데이터는 binary payload에 연속으로
담는다. JPEG나 point cloud를 Base64 JSON으로 변환하지 않아 크기와 CPU 낭비를
줄인다.

ROS와 Docker가 서로 다른 센서 파일을 mount하는 실수를 막기 위해 ROS request에
`Sensor_4.json` SHA256, camera order, intrinsic source를 담는다. 서버가 시작할 때
읽은 값과 하나라도 다르면 해당 request를 거부한다.

요청 payload는 JPEG 4개와 float32 PointCloud이고, 응답은 JSON detection 또는
projection JPEG 4개다. metadata는 1 MiB, binary payload는 128 MiB로 제한한다.
protocol magic·version·blob 경계·중복·overlap을 검사한다.

네트워크에서 받은 pickle을 역직렬화하지 않는다. MMDetection3D용 임시 pkl은
검증된 JSON metadata를 바탕으로 서버가 `/dev/shm/bevfusion_runtime` 안에서
직접 생성한다. checkpoint는 팀이 신뢰하고 SHA256을 확인한 로컬 파일만 쓴다.

### 7.4 timeout과 실패 처리

- connect timeout 기본 2초
- inference request timeout 기본 30초
- 서버는 한 client의 요청을 순차 처리
- 연결 단절 시 client socket 폐기 후 다음 frame에서 재접속
- 처리 중 들어온 센서 frame은 하나만 최신 값으로 교체
- 서버 예외는 protocol error response와 Docker log에 모두 기록
- `/perception/bevfusion/status`에서 ROS 측 상태 확인

### 7.5 대회 규정과 겹칠 수 있는 부분

최신 규정은 MORAI 운영 PC와 참가자 PC 사이의 허용 센서·제어 인터페이스를
UDP로 열거하며, 허용 네트워크 외 사용 시 실격될 수 있다고 적고 있다. 이번
TCP는 두 PC 사이 통신이 아니라 **참가자 PC 한 대 내부의 ROS process와 Docker
process 사이 IPC**다. MORAI 경계에는 계속 규정상 UDP만 사용한다.

ROS1도 node 사이 message transport에 TCPROS, ROS master 관련 처리에 XML-RPC
over TCP를 사용한다. 따라서 내부 localhost TCP를 외부 허용 네트워크와 동일하게
보는 것은 기술적으로 자연스럽지 않다. 그러나 이것만으로 규정 허용이 확정되는
것은 아니며, 최종 해석 권한은 운영진에 있다.

운영진 문의 권장 문구는 다음과 같다.

> 참가자 PC 한 대 내부에서만 실행되는 ROS 프로세스와 Docker 컨테이너 사이의
> IPC 목적으로, 외부 LAN에 노출되지 않는 127.0.0.1 TCP 연결을 사용해도 되는지
> 문의드립니다. MORAI 운영 PC와 참가자 PC 사이의 센서·제어 통신은 규정에
> 지정된 UDP 인터페이스만 사용합니다. 또한 외부 서버 없이 참가자 PC의 로컬
> GPU에서 추론하는 Docker 컨테이너 사용이 허용되는지도 확인 부탁드립니다.

### 7.6 내부 TCP도 금지될 때의 pivot

운영진이 localhost TCP까지 금지하면 Unix Domain Socket(UDS)으로 바꾼다.

```text
현재: AF_INET + SOCK_STREAM + 127.0.0.1:29500
변경: AF_UNIX + SOCK_STREAM + /tmp/ajounice_bevfusion/inference.sock
```

유지되는 부분:

- 16-byte header, JSON metadata, binary payload protocol
- sensor synchronization과 latest-frame-only worker
- calibration, runtime sample, model inference
- detection 및 RViz publisher
- timeout·오류·재연결 정책

교체되는 부분:

- ROS `TcpTransport`의 socket 생성과 connect 주소
- Docker server의 bind/listen 주소
- Compose port mapping 제거
- 공유 socket directory bind mount 추가

UDS에서는 호스트와 컨테이너에 `/tmp/ajounice_bevfusion`을 동일하게 mount하고
`inference.sock` 파일로 연결한다. IP·port·LAN packet을 사용하지 않으므로 규정
해석상 더 보수적이다. 현재 `protocol.py`와 `transport.py`를 분리한 이유가 바로
이 변경을 국소화하기 위해서다.

## 8. Docker runtime

개발 중에는 `docker/bevfusion/app`과 `scripts`를 read-only bind mount한다.
Python 코드를 수정할 때 이미지를 다시 빌드하지 않으면서, 검증된 CUDA stack은
`ajounice/bevfusion:dev-cu128` 이미지에 고정한다.

체크포인트와 Sensor JSON도 read-only mount하며 git에 checkpoint를 넣지 않는다.
실시간 연동이 검증된 후 Dockerfile에 app을 `COPY`하고 최종 runtime tag를 만드는
것이 배포 단계다.

PyTorch 2.6 이후 checkpoint의 `weights_only` 기본값 변화 때문에 신뢰한 공식
checkpoint에 한해 `TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1`을 server process에만
설정한다. 출처를 모르는 checkpoint에는 적용하면 안 된다.

## 9. ROS 출력 계약

| topic | type | 목적 |
|---|---|---|
| `/perception/bevfusion/markers` | `visualization_msgs/MarkerArray` | 3D box·class·score RViz 표시 |
| `/perception/bevfusion/latency_ms` | `std_msgs/Float32` | frame 수신부터 응답까지 지연 |
| `/perception/bevfusion/status` | `std_msgs/String` | 동기화·연결·추론 상태 |
| `/perception/bevfusion/projection/<camera>/compressed` | `sensor_msgs/CompressedImage` | calibration 확인 |

box 결과는 checkpoint와 MMDetection3D가 사용하는 LiDAR frame 좌표로 반환하고
Marker의 `frame_id`를 `lidar1`로 둔다. RViz는 기존 `base_link → lidar1` TF로
변환한다.

MarkerArray는 사람이 보는 디버그 출력이지 Planning 최종 계약이 아니다.
Planning 연결 전에는 기준 frame, tracking ID, velocity 신뢰도, class mapping,
covariance, stale timeout을 합의하고 typed detection message를 별도로 발행한다.

## 10. 검증 순서

1. Python compile, YAML/XML parse
2. protocol fragmented-read round trip
3. ROS client와 Docker protocol 사이 localhost TCP round trip
4. calibration 행렬 finite/inverse 검사
5. `catkin build bevfusion_ros bringup`
6. `docker compose config`
7. Sensor_4 topic 주기와 TF 확인
8. projection-only 네 카메라 확인
9. 모델 server healthcheck
10. zero-shot RViz box 방향·위치·지연 확인
11. 장시간 실행에서 memory 증가와 stale frame 여부 확인

실행 명령은
`docs/bevfusion/260720_morai_four_camera_zero_shot_short.md`에 터미널별로 정리했다.

## 11. 알려진 한계와 다음 단계

- 공식 checkpoint는 NuScenes 6-camera 학습 결과이고 현재 입력은 MORAI 4-camera다.
- zero-shot detection 수와 score는 정확도 metric이 아니다.
- intrinsic은 JSON FOV 기반 잠정 pinhole 계산이다.
- Python `read_points` 변환은 CPU 병목이 될 수 있다.
- 센서 입력은 15~20 Hz여도 현재 GPU zero-shot inference는 약 3 FPS 수준이다.
- tracking과 Planning용 detection topic이 아직 없다.

projection과 실시간 box가 정상이라면 다음 순서는 MORAI 데이터 수집·annotation
설계, 4-camera dataset adapter, 파인튜닝, 정량 평가, Planning message 계약,
필요할 경우 PointCloud 변환 최적화다.
