# 팀원용 MR 사람 주행 raw rosbag 수집 handoff

- 상태: **CURRENT HANDOFF**
- 기준 branch: `dataset/human-driven-eval-capture`
- 기준 commit: 이 문서가 포함된 branch HEAD
- 모델 추론: **사용하지 않음**
- Google Drive 사전 다운로드: **없음**

## 1. 목적과 절대 규칙

이 수집은 사람이 MR 차량을 직접 운전하여 고주로, 회전교차로, 보행자 횡단, 정적
장애물의 raw 4-camera + LiDAR rosbag을 만드는 작업이다. BEVFusion, PointPainting,
tracking, prediction, planning, control은 실행하지 않는다. 따라서 Docker, checkpoint,
dataset을 받을 필요가 없고 GPU 종류도 무관하다.

run의 역할은 녹화 전에 고정되어 있다.

- `development`: 모델·tracker·평가기 개발 중 열어봐도 된다.
- `evaluation_locked`: 학습에 절대 넣지 않고 개별 frame에 맞춰 threshold를 조절하지 않는다.
- `training_candidate`: 별도 결정 전까지 학습과 평가 양쪽에서 봉인한다.

중간에 충돌, 잘못된 scenario load, 센서 끊김이 생기면 같은 run ID 결과를 덮어쓰지 말고
새로 다시 녹화한다. 실패 폴더도 원인 확인 전 삭제하지 않는다.

## 2. 새 PC에서 최초 1회

필수 조건:

- Ubuntu 20.04, ROS Noetic
- 대회용 MR과 K-City 2025 map
- 최소 100 GiB 여유 공간 권장
- GitHub SSH 접근

Home에서 새로 clone한다.

```bash
cd ~
git clone --single-branch \
  -b dataset/human-driven-eval-capture \
  git@github.com:2026-Ajou-AutoRacing-Study/AjouNice2026.git

cd ~/AjouNice2026
```

`catkin` 명령이 없다면 한 번만 설치한다.

```bash
sudo apt update
sudo apt install python3-catkin-tools ros-noetic-velodyne-pointcloud ros-noetic-nodelet
```

최소 acquisition 패키지만 빌드한다.

```bash
cd ~/AjouNice2026
dataset/morai_od/human_eval_capture/setup_capture_environment.sh
```

이 스크립트가 `base_ws/src/morai/morai_msgs` submodule 하나만 초기화한다. 저장소의 다른
대형 submodule은 이 수집에 필요하지 않으므로 `--recurse-submodules`를 사용하지 않는다.

빌드 대상은 `morai_msgs`, `sensor_interface`뿐이다. `planning_ws`, localization,
BEVFusion, acados 및 `base_ws/install_aw149`는 만들거나 source하지 않는다. 결과는
`base_ws/devel/setup.bash`다.

## 3. 시나리오를 MR에 설치

MR 설치 경로가 Home 아래 하나뿐이면 자동으로 찾는다.

```bash
cd ~/AjouNice2026
dataset/morai_od/human_eval_capture/install_scenarios.sh
```

자동 탐색 결과가 0개 또는 여러 개면 MR root를 직접 준다.

```bash
dataset/morai_od/human_eval_capture/install_scenarios.sh \
  --mr-root "$HOME/실제_MoraiLauncher_Lin_폴더"
```

설치 후 MR의 Scenario Load에서 `ajounice_eval_`을 검색하면 9개 파일이 보여야 한다.
시나리오 파일 설명은
[`configs/scenario/human_driving_evaluation/README.md`](../../../configs/scenario/human_driving_evaluation/README.md)에 있다.

## 4. MR Network 설정

MR에서 아래 Git 파일을 직접 Load하고 Connect한다.

```text
Sensor Network:
  ~/AjouNice2026/configs/sensors/Sensor_competition_local.json

Ego Network:
  ~/AjouNice2026/configs/network/ego_network_competition_local.json
```

이 수집은 sensor와 Ego/Object를 모두 UDP로 받는다. ROSBridge와
`morai_personal_ws`를 실행하지 않는다. 같은 토픽의 publisher를 중복으로 켜지 않는다.

## 5. 매번 실행할 터미널

### Terminal 1 — ROS master

```bash
source /opt/ros/noetic/setup.bash
roscore
```

### Terminal 2 — UDP sensor + Ego/Object gateway

```bash
cd ~/AjouNice2026
dataset/morai_od/human_eval_capture/start_udp_runtime.sh
```

MR에서 Sensor/Ego Network 두 profile을 Connect한다. 다음 토픽에 메시지가 나오는지 한 번
확인한다.

```bash
rostopic hz /sensor/lidar1
rostopic hz /sensor/camera/front/compressed
rostopic hz /Ego_topic
rostopic hz /Object_topic
```

정상 기대치는 LiDAR 약 12--15 Hz, camera 약 30 Hz, Ego/Object 약 40--50 Hz다.
LiDAR가 지속해서 8 Hz 아래라면 녹화하지 말고 MR과 UDP gateway를 재시작한다.

### Terminal 3 — run 선택, 녹화 시작·종료

가능한 run을 본다.

```bash
cd ~/AjouNice2026
dataset/morai_od/human_eval_capture/capture.sh list
```

MR에서 해당 run의 scenario를 먼저 Load한다. 예를 들어 H01이면
`ajounice_eval_highway_balanced.json`을 Load한다. 시나리오가 Keyboard mode, P, 정지
상태인지 확인한 뒤 녹화를 시작한다.

```bash
dataset/morai_od/human_eval_capture/capture.sh start H01 --driver <이름>
```

스크립트가 `CAPTURE STARTED`를 출력한 다음:

1. MR에서 gear를 D로 변경한다.
2. 화면에 출력된 주행 지시대로 직접 운전한다.
3. 구간을 완전히 통과한 뒤 최소 5초 더 주행한다.
4. 아래 명령으로 종료한다.

```bash
dataset/morai_od/human_eval_capture/capture.sh stop
```

`stop`은 rosbag에 SIGINT를 보내 index와 split bag을 정상 종료한 후 자동 audit한다.
중간에 `kill -9`, PC 종료, MR 강제 종료를 하지 않는다.

현재 상태는 언제든 확인할 수 있다.

```bash
dataset/morai_od/human_eval_capture/capture.sh status
```

## 6. 우선순위와 사람 주행 방법

### 최우선: H01--H04

고주로는 각각 한 번에 끝까지 주행한다.

```text
정지 출발
→ ramp 접근
→ 합류 완료 전 59 km/h 이하
→ 본선 합류
→ 80~100 km/h 순항
→ 주변 차량을 충분히 관측
→ 안전할 때 차선 변경·추월
→ 차선 감소 bottleneck 통과
→ 톨게이트/GPS 음영 구간 직전까지
```

- 차선 중앙을 기계적으로 고정하지 말고 차선 안에서 자연스럽게 운전한다.
- 급조향으로 억지 추월 장면을 만들지 않는다.
- 측후방 차량이 보이면 바로 차선을 바꾸지 말고 상대속도를 관측할 시간을 둔다.
- 충돌하거나 차선을 크게 이탈하면 해당 run은 실패로 표시하고 다시 녹화한다.
- H02--H04는 `evaluation_locked`이므로 이후 학습에 사용하지 않는다.

### R01--R03

회전교차로 접근 전 5초, 진입·양보·회전, 출구 통과 후 5초를 한 bag에 담는다. NPC를
만나려고 위험하게 타이밍을 맞추지 않는다. 정상적인 양보 판단을 유지한다.

### P01--P02

보행자가 움직이기 전부터 녹화한 상태여야 한다. 자연스럽게 접근하고 횡단 중에는 양보한
뒤 완전히 벗어나면 통과한다.

### O01--O02

6개 scale-1 장애물을 저속으로 지나가며 정면 사선과 측면 관측을 남긴다. 충돌하지 않는다.
WoodBox는 이 묶음에 없다.

세부 run별 지시는 `capture.sh start`가 다시 출력한다.

## 7. 저장 위치와 자동 검사

기본 저장 경로:

```text
~/ajounice_human_eval_bags/<mission>/<timestamp>_<run_id>_<role>/
```

각 run에는 다음이 생긴다.

- LZ4, 4 GiB 단위 split rosbag
- Git SHA, scenario SHA-256, driver, 역할이 기록된 metadata JSON
- topic count/rate와 PASS/FAIL이 기록된 audit JSON
- rosbag recorder log

자동 PASS gate:

| 토픽 | 최소 평균 주기 |
|---|---:|
| LiDAR | 8 Hz |
| 각 camera | 19 Hz (현재 profile 정상 기대값은 약 30 Hz) |
| Ego/Object | 15 Hz |
| GPS | 10 Hz |
| IMU | 20 Hz |

FAIL 결과는 전달하지 말고 원인을 확인한 뒤 다시 녹화한다. 기존 bag을 수동으로 다시
검사하려면 split part를 모두 넘긴다.

```bash
dataset/morai_od/human_eval_capture/capture.sh verify \
  ~/ajounice_human_eval_bags/highway/<run-folder>/*.bag
```

## 8. 전달

수집이 끝나면 metadata와 audit을 포함한 상위 폴더를 그대로 묶는다. 원본 bag 이름이나
폴더 역할을 변경하지 않는다.

```bash
cd ~
tar -I 'zstd -T0 -3' -cf ajounice_human_eval_bags_<날짜>.tar.zst \
  ajounice_human_eval_bags
sha256sum ajounice_human_eval_bags_<날짜>.tar.zst \
  > ajounice_human_eval_bags_<날짜>.tar.zst.sha256
```

이 결과물만 공유 드라이브에 업로드한다. 수집 전 Google Drive에서 받을 파일은 없다.

## 9. 문제 발생 시 보내줄 정보

아래 결과를 함께 전달한다.

```bash
cd ~/AjouNice2026
git rev-parse HEAD
git status --short
dataset/morai_od/human_eval_capture/capture.sh status
rostopic list | sort
rostopic info /sensor/lidar1
rostopic info /Ego_topic
```

그리고 해당 run 폴더의 `*_rosbag.log`, `*_metadata.json`, `*_audit.json`을 전달한다.
