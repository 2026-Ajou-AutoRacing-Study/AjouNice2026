# Base5 E24 BEVFusion 배포와 실행

문서 상태: CURRENT. 2026-10-13 사용자 요청으로 E24를 production 기본 모델로 승격했다.
이는 실제 대회용 4090 24GB의 ROS 입력·LAN·풀스택 안전 검증 완료를 뜻하지 않는다.
판단팀 브랜치 머지는 사용자가 수행하며 여기서는 해당 브랜치를 변경하지 않는다.

## 고정 실행 계약

| 항목 | 값 |
|---|---|
| 기본 진입점 | `bevfusion` 또는 `bash scripts/runtime/run_bevfusion.sh` |
| 가중치 | `~/bevfusion_artifacts/checkpoints/base5_e24_inference.pth` |
| SHA256 | `84c7a849406a37f05250f872d4d0dfa3ebcffdb66b48e47bdfcb3209efecc746` |
| 클래스와 threshold | Vehicle/Pedestrian/Obstacle, 0.40/0.10/0.10 |
| 모델 | 512×1056, 5cm/20cm, depth 0.25m, single-frame 7D |
| 입력 | 원본 JPEG, 원본 XYZI 0–255를 서버에서 정확히 한 번 ÷255 |
| 추론 | AMP·camera 병렬화 ON, compile OFF |
| 센서 기본 | `Sensor_competition_lan.json`, 실행 인자로 local/다른 실제 센서 JSON 지정 가능 |
| 출력과 후처리 | canonical detection topic, class-aware rotated BEV NMS 0.50 |
| 후단 | 기존 tracking → UNKNOWN stationary policy → Map-CV. 이번 승격만으로 tracker/planner 제약을 완화하지 않음 |

`bevfusion_production_profile.py`가 checkpoint·모델 config·bridge config·launch를 함께 선택한다.
서버 health는 weights·모델 config·센서 SHA와 Base5·AMP·병렬화·compile·클래스·입력 계약을 검사한다.
기존 서비스나 canonical detector publisher가 있으면 실행을 거부한다. launch만으로 GPU 서버가 준비되는 것은 아니다.

## 팀원 최초 준비

팀원이 판단팀 브랜치에 머지된 코드를 pull한 뒤 수행한다. 기존 팀 Docker 이미지와 NVIDIA
Container Toolkit은 계속 사용한다. 이미지에 들어 있는 upstream BEVFusion 세 핵심 Python 파일은
로컬 checkout과 SHA가 같았으며, 현재 Base5 custom 구현은 repository mount에서 가져온다.
다른 이미지·새 GPU의 실제 실행은 별도 점검해야 한다.

```bash
cd "$HOME/AjouNice2026"
python3 scripts/runtime/prepare_bevfusion_production.py
```

`ajou_drive` remote와 팀 공유 폴더 접근 권한이 필요하다. 원격 파일은
`model/bevfusion/production/base5_e24_261013/E24.pth`이며 약 501MiB다.
전용 Drive client가 다른 remote 이름이면 `--remote 이름`을 추가한다.
기존 파일의 SHA가 맞으면 다시 다운로드하지 않는다. 다른 파일이 있으면 자동 덮어쓰기하지 않는다.
토큰은 각 PC의 rclone 설정에만 두며 문서·명령·Git에 넣지 않는다.

로컬에 E24 원본이 있다면 다운로드 대신 다음을 사용한다.

```bash
python3 scripts/runtime/prepare_bevfusion_production.py --source-checkpoint /전달받은/E24.pth
```

가중치 전달과 ROS 설치 공간 갱신은 별개다. 새 launch와 최신 tracker 소스를 다음처럼 반영한다.
이번 조사에서 로컬 설치 tracker에는 최신 소스의 일부 옵션이 빠져 있었으므로 Git pull만으로 최신 바이너리라 가정하지 않는다.

```bash
source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash
catkin_make_isolated --install \
  --source "$PWD/base_ws/src" --build "$PWD/base_ws/build_aw149" \
  --devel "$PWD/base_ws/devel_aw149" --install-space "$PWD/base_ws/install_aw149" \
  --pkg bevfusion_ros ekf_multi_object_tracking perception_object_pipeline \
  -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  --make-args -j4 -l4
```

초기 ROS workspace나 Docker가 없는 PC는 기존 설치 절차로 의존성을 준비한 뒤 위 E24 준비를 수행한다.
기존 `setup_bevfusion_inference.sh`는 E25 archive도 준비하는 호환 설치기이며 E24 설치 완료로 해석하지 않는다.

## 실행과 복귀

기존 sensor/TF/localization 터미널을 준비하고 실행한다. 동일 canonical topic에 detector를 두 개 띄우지 않는다.

```bash
ajou
bevfusion
```

개인 PC의 loopback 센서 설정을 사용할 때는 다음과 같다.

```bash
bevfusion "$AJOUNICE_ROOT/configs/sensors/Sensor_competition_local.json"
```

sensor JSON은 서버와 bridge에 같은 파일로 전달된다. 기본값이 LAN이라도 시뮬레이터 UI의 IP를
자동으로 바꾸지는 않는다. `tracking`·planning은 기존 순서로 실행하며 `start_detector:=false`를 유지한다.

E25로 복귀하려면 현재 detector 터미널을 먼저 종료한 뒤 다음을 실행한다.

```bash
bevfusion --model e25
```

E25 가중치는 기존 경로에 보존한다. 없으면 [E25 복귀 계약](BASE3_BEVFUSION_PRODUCTION_HANDOFF.md)으로 준비한다.
옛 `.env.bevfusion.local`의 `BEVFUSION_INFERENCE_CHECKPOINT`는 E25 복귀에서만 사용한다.
명시적인 `BEVFUSION_CHECKPOINT` override는 두 모델에서 사용하되 SHA가 다르면 거부한다.
기존 .env는 수정하지 않으며 단순 config override로 E24를 E25 입력에 연결하지 않는다.
Validator GUI에서 명시적으로 E25 preset을 고르면 여전히 E25다. E24 preset·파일 경로를 확인하고
전용 detector와 GUI 자동 실행을 중복하지 않는다. 이번 작업은 validator preset을 변경하지 않는다.

## 팀장님 PC에서 확인할 항목

- 실제 체크포인트·옵션·센서가 일치하고 canonical publisher가 하나인지 확인한다.
- LiDAR 주기 대비 latency p50/p95, 입력 age·drop·queue, 실제 24GB VRAM·CPU 부담을 기록한다.
- 자차 근처 회신점과 자차 위치 Vehicle 출력이 있는지 확인한다. 구 Base3 입력에서는 E24가 자차를 검출한 사례가 있어 무조건 안전한 자차 mask를 적용하지 않는다.
- 근접 장애물의 ID·경계·누락·중복과 고주로 전후방 차량의 클래스·속도·연속 전달을 확인한다.
- 유효 trajectory·제어·CollisionData까지 확인한다. 오프라인 detector/tracker 비교는 폐루프 인증이 아니다.

E24 선택 근거와 알려진 한계는 [모델 CURRENT](CURRENT.md)와
[43절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-e9-e24-selection-261013),
[44절](261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-on-base3-val-261013)을 따른다.
