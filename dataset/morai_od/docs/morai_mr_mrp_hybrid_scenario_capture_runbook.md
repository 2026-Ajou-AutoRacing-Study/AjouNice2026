# MORAI MR/MRP hybrid ROS+UDP 시나리오 수집 런북

- 상태: **CURRENT**
- 마지막 검증: 2026-09-20
- 목적: 자동 `ScenarioLoad`, ROS 제어, UDP 센서 rosbag 수집

이 문서는 MR과 MRP에서 생성 시나리오를 순서대로 주입하고 Ego를 움직이며 실제
온라인 센서 토픽을 rosbag으로 기록하는 공통 실행 계약이다. 일반 주행용 full-UDP와
이 자동 수집용 hybrid 구성을 섞지 않는다.

## 1. 공통 전송 계약

| 데이터/명령 | 전송 | 단일 publisher/subscriber 계약 |
|---|---|---|
| Camera, LiDAR, GPS, IMU | UDP | `morai_sensor_udp_receiver` |
| `/Ego_topic`, `/Object_topic` | rosbridge | MORAI ROS Ego Network |
| `/ScenarioLoad` | rosbridge | MORAI ROS Ego Network |
| `/ctrl_cmd`, control-mode service | rosbridge | MORAI ROS Ego Network |
| `/dataset/scenario_marker` | ROS 내부 | scenario runner |

따라서 UDP gateway는 반드시 sensor-only로 실행한다. `/SaveSensorData`는 이 계약에
포함하지 않는다. 원본은 continuous rosbag이며 필요한 LiDAR frame을 marker window
안에서 나중에 고른다.

```bash
roslaunch sensor_interface morai_udp_interface.launch \
  sensor_config:="$AJOUNICE_ROOT/configs/sensors/Sensor_competition_local.json" \
  enable_ego_network:=false \
  enable_control:=false
```

`udp` alias는 Ego Network와 control UDP까지 켜므로 이 수집 모드에서는 사용하지 않는다.
rosbridge와 UDP가 동시에 `/Ego_topic` 또는 `/Object_topic`을 발행하면 그 run은 무효다.

Scenario sidecar는 다음 다섯 interface만 갖는다.

```text
/Ego_topic
/Object_topic
/ScenarioLoad
/ctrl_cmd
/Service_MoraiEventCmd
```

MR의 full 23-entry ROS profile은 이 launcher에서 시간이 갈수록 sensor rate를 떨어뜨린
실측 이력이 있으므로 자동 수집에 사용하지 않는다.

## 2. MR과 MRP의 차이

| 항목 | MR | MRP |
|---|---|---|
| launcher | `~/MoraiLauncher_Lin` | `~/MoraiLauncher_Lin_per` |
| bundle 계열 | `25.S4.MolitComp03` | `26.R1.H3` |
| Sensor GUI | `Sensor_competition_local.json`, UDP | 동일, UDP |
| ROS Ego Network 원본 | `ss_ego_only.json` | `ros.json` |
| 수집 profile | `ego_network_mr_minimal_hybrid.json` | `ego_network_mrp_minimal_hybrid.json` |
| extra workspace | 사용하지 않음 | `~/morai_personal_ws/devel/setup.bash` 사용 |
| scenario directory | `~/MoraiLauncher_Lin/.../Scenario/R_KR_PR_K-city_2025` | `~/MoraiLauncher_Lin_per/.../Scenario/R_KR_PR_K-city_2025` |

MR의 `ss.json`에는 Ego `UNIQUEID=0`과 Simulator `UNIQUEID=-100`이 함께 있다. scenario
sidecar에는 Ego 항목만 넣는다. MORAI GUI는 불러온 Network JSON에 Simulator 항목을
다시 써 넣을 수 있으므로, GUI에서 한 번 로드한 파생 파일을 다음 sidecar 생성의 원본으로
재사용하지 않는다. 원본 `ss.json`에서 `UNIQUEID=0`을 다시 추출하거나 이미 검증된
scenario sidecar를 사용한다. 추적되는 최소 profile과 launcher 설치본은 다음과 같다.

```text
configs/network/ego_network_mr_minimal_hybrid.json
configs/network/ego_network_mrp_minimal_hybrid.json

~/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Network/
25.S4.MolitComp03/ss_ego_minimal_hybrid.json

~/MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Network/
26.R1.H3/ros_base4_minimal_hybrid.json
```

원본 `ss.json`은 수정하지 않는다.

## 3. 공통 터미널 순서

### Terminal 1 — ROS master

```bash
source /opt/ros/noetic/setup.bash
source ~/AjouNice2026/base_ws/install_aw149/setup.bash
roscore
```

### Terminal 2 — rosbridge

MR:

```bash
source /opt/ros/noetic/setup.bash
source ~/AjouNice2026/base_ws/install_aw149/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

MRP:

```bash
source /opt/ros/noetic/setup.bash
source ~/AjouNice2026/base_ws/install_aw149/setup.bash
source ~/morai_personal_ws/devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

### Terminal 3 — sensor-only UDP gateway

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash
export AJOUNICE_ROOT="$PWD"
rosparam set /use_sim_time false

roslaunch sensor_interface morai_udp_interface.launch \
  sensor_config:="$PWD/configs/sensors/Sensor_competition_local.json" \
  enable_ego_network:=false \
  enable_control:=false
```

### Terminal 4 — simulator

MR:

```bash
cd ~/MoraiLauncher_Lin
bash ./MORAISim.sh
```

MRP:

```bash
cd ~/MoraiLauncher_Lin_per
bash ./MORAISim.sh
```

MORAI GUI에서 Sensor Network는 `Sensor_competition_local.json`을 UDP로 연결하고,
Ego Network는 위의 launcher별 minimal profile을 연결한다. Simulator Network는 별도로
연결하지 않는다. Runner를 쓸 때 대상 scenario를 GUI에서 먼저 직접 로드하지 않는다.

### Terminal 5 — 사전 검사

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash

python3 dataset/morai_od/scripts/check_base3_online_runtime.py \
  --gt-transport ros
```

추가로 다음 계약을 확인한다.

```bash
rostopic info /sensor/lidar1
rostopic info /Ego_topic
rostopic info /Object_topic
rostopic info /ScenarioLoad
rostopic info /ctrl_cmd
```

- LiDAR publisher: UDP receiver 하나.
- Ego/Object publisher: rosbridge 하나.
- ScenarioLoad/ctrl_cmd: MORAI rosbridge client subscriber 존재.

### Terminal 6 — rosbag recorder

Runner보다 먼저 시작한다. MR에서도 `morai_personal_ws`가 암묵적으로 source되지 않도록
공통 런북에서는 명시적인 recorder를 사용한다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash

RUN_TAG="$(date +%y%m%d_%H%M%S)_<campaign>" \
dataset/morai_od/scripts/record_base4_temporal_bag.sh \
  "$PWD/rosbags/datasets/base4_temporal/<campaign>"
```

이 recorder는 필수 11개 topic과 진단용 optional topic을 구분하고 sensor/network
config SHA-256, Git SHA, 실행 시각을 `*_recording_contract.txt`에 남긴다. optional topic이
advertise되지 않아도 rosbag은 계속 기록한다.

### Terminal 7 — scenario runner

campaign의 runtime manifest와 launcher별 scenario directory를 사용한다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash
export AJOUNICE_ROOT="$PWD"

python3 -u dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest <runtime-manifest.json> \
  --capture-dir <runtime-metadata-dir> \
  --no-timestamp-prefix \
  --morai-scenario-dir <launcher-scenario-dir> \
  --motion-mode rolling-grouped \
  --online-bag-only \
  --scenario-load-attempts 3 \
  --capture-timeout-sec 30 \
  --continue-on-scene-error
```

Runner가 `auto-capture complete`를 출력한 뒤에만 recorder를 `Ctrl-C`로 종료한다.

## 4. MR WoodBox/CargoBox rolling A/B

2026-09-20 준비된 MR 전용 2-scene runtime manifest:

```text
dataset/morai_od/manifests/woodbox_alignment_mr_pair_260920_minimal_runtime.json
```

- `wba015`: CargoBox control, 10 m, yaw 45도.
- `wba016`: WoodBox probe, 같은 Ego/object pose.
- scene당 3개 marker, 0.75 m 간격.
- Ego 5 km/h, 최대 이동 4 m.
- rolling/route-follow runner용 각 scene에는 route의 `anchor_index`가 반드시 있어야 한다.
  이 pair의 anchor는 `619`이며 시작 pose와 route point의 XY 오차는 `0 m`다.

Runner 명령:

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/install_aw149/setup.bash
export AJOUNICE_ROOT="$PWD"

RUN_DIR="/home/luv/morai_cap/$(date +%y%m%d_%H%M%S)_woodbox_mr_pair"

python3 -u dataset/morai_od/scripts/ros_scenario_autocapture_poc.py \
  --manifest dataset/morai_od/manifests/woodbox_alignment_mr_pair_260920_minimal_runtime.json \
  --capture-dir "$RUN_DIR" \
  --no-timestamp-prefix \
  --morai-scenario-dir /home/luv/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025 \
  --motion-mode rolling-grouped \
  --captures-per-scene 3 \
  --capture-spacing-m 0.75 \
  --speed-kph 5 \
  --maximum-motion-m 4 \
  --online-bag-only \
  --scenario-load-attempts 3 \
  --capture-timeout-sec 30 \
  --continue-on-scene-error
```

분석용 bag에는 최소한 다음 토픽이 있어야 한다.

```text
/sensor/lidar1
/sensor/camera/front/compressed
/Ego_topic
/Object_topic
/dataset/scenario_marker
```

2026-09-20 MR 실측에서는 두 scene 모두 자동 load, AV External Control, 2.49 m 주행,
scene당 3 trigger를 완료했다. 확정 근거는 다음과 같다.

```text
rosbags/datasets/woodbox_alignment_mr/
260920_164344_mr_cargobox_woodbox_rolling.bag

/home/luv/morai_cap/260920_164358_woodbox_mr_pair_retry/

artifacts/datasets/woodbox_alignment_260920/mr_rolling_pair/
```

### 4.1 MR stationary 10 m yaw grid

MR에서도 MRP의 exact-pose 실험과 같은 `10 m x yaw 0/30/45/60/90 deg` grid를
추가로 기록했다. CargoBox와 WoodBox 각 5 scene은 모두 첫 시도에 load되었고 실패는
없었다.

```text
rosbags/datasets/woodbox_alignment_mr/
260920_164959_mr_cargobox_woodbox_10m_yaw_grid.bag

/home/luv/morai_cap/260920_165008_woodbox_mr_10m_yaw_grid/

artifacts/datasets/woodbox_alignment_260920/mr_10m_yaw_grid_final/
```

이 run은 bag 전체 기준 LiDAR 약 `1.77 Hz`, camera 약 `3.60 Hz`, Ego 약 `7.16 Hz`로
MORAI가 심하게 느려진 상태였다. 따라서 아래 point audit 수치는 진단 흔적으로만 남기며
**MR WoodBox 정합의 유효 근거로 사용하지 않는다.**

| Asset | Raw GT 내부 points | +3 cm 내부 points | +5 cm 내부 points |
|---|---:|---:|---:|
| CargoBox | 30 | 104 | 143 |
| WoodBox | 158 | 194 | 380 |

정상 주기에서 같은 grid를 재취득하기 전까지 MR에서 WoodBox 고유 문제가 있는지 여부는
미확정이다.

이 audit는 box 주위의 배경점을 instance 분리하지 않으므로 절대 point 수나 dilation
증가율을 GT 오차 크기로 해석하지 않는다. 같은 pose의 paired 비교와 BEV 그림을 함께
사용한다.

### 4.2 MR oblique rolling A/B 시도와 무효 판정

정면 stationary probe의 한계를 제거하려고 Ego 기준 다음 네 placement를 CargoBox와
WoodBox pair로 생성했다.

| 시작 종/횡 위치 | asset yaw | 주행 |
|---|---:|---:|
| `+14 / +3.0 m` | `0 deg` | 약 11 m |
| `+14 / -3.0 m` | `30 deg` | 약 11 m |
| `+18 / +3.5 m` | `60 deg` | 약 11 m |
| `+18 / -3.5 m` | `90 deg` | 약 11 m |

8개 scene 모두 AV External Control, `8 km/h`, 5 trigger를 완료했다. bag에서 확인한 실제
상대 yaw는 약 `-1/27/59/89 deg`, Ego 이동은 `10.7--11.1 m`였으므로 제어와 geometry
적용 자체는 정상이다.

그러나 capture window별 주기는 LiDAR `3.09--4.30 Hz`, camera `6.16--8.81 Hz`,
Ego/Object `12.39--17.72 Hz`로 떨어졌다. 이 run도 **geometry/control smoke evidence일
뿐 정합 판정에는 무효**다.

```text
rosbags/datasets/woodbox_alignment_mr/
260920_mr_cargobox_woodbox_rolling_oblique.bag

/home/luv/morai_cap/260920_170020_woodbox_mr_rolling_oblique/

artifacts/datasets/woodbox_alignment_260920/mr_rolling_oblique/
```

MORAI가 LiDAR 약 `3 Hz`, Ego 약 `15 Hz` 이하로 떨어지거나 정상 상태 대비 주기가 크게
하락하면 결과를 믿지 않는다. simulator/PC를 재시작하고 정상 주기를 확인한 뒤 campaign을
처음부터 다시 기록한다.

### 4.3 재부팅 후 사용할 clean MR retry set

MRP 유래 base scenario에서 geometry probe에 불필요한 다음 필드를 명시적으로 비웠다.

```text
tlList
shadedAreaList
frictionChangeMaterialList
waypointDataList
```

clean scene은 `wbmrc001--008`이며 CargoBox/WoodBox pair, 좌우 위치, yaw, rolling route는
4.2절과 같다.

```text
source manifest:
dataset/morai_od/manifests/
woodbox_alignment_mr_rolling_oblique_clean_260920.json

runtime manifest:
dataset/morai_od/manifests/
woodbox_alignment_mr_rolling_oblique_clean_260920_runtime.json

source scenarios:
dataset/morai_od/scenarios/
woodbox_alignment_mr_rolling_oblique_clean_260920/
```

MR Scenario 폴더와 `EgoNetwork`에는 clean 8개가 설치되어 있다. 저주기 run에 사용한
`wba011--020`, `wbmr001--008`은 다음으로 옮겼다.

```text
/home/luv/morai_scenario_archive/260920_woodbox_invalid_lowhz/
```

재부팅 후 첫 scene load 전후로 `Player.log`의 `IntTL` 증가 여부와 topic rate를 먼저
확인한다. 2026-09-20 single-scene gate에서 빈 `tlList`인데도 `IntTL` 로그가 계속
증가했으므로, 이 로그는 scene의 `tlList`가 아니라 MR/map runtime에서 생성되는 것으로
판단한다. 따라서 `IntTL` 유무만으로 scene을 판정하지 않고 topic rate를 최종 health
gate로 사용한다.

같은 gate에서 MORAI 프로세스만 재시작한 뒤 baseline은 LiDAR/camera/Ego가 각각
`12.9/24.9/41.2 Hz`였지만, clean scene 하나를 load한 뒤 `8.4/16.7/31.3 Hz`, 잠시 뒤
`6.6/13.4/25.3 Hz`로 계속 하락했다. 이 상태에서는 전체 8-scene run을 시작하지 않는다.
OS uptime과 장기 desktop process가 유지되고 있었으므로 simulator 재시작과 PC 전체
재부팅을 구분한다. 전체 재부팅 후에도 같은 현상이 재현되면 scene 위치/렌더링·sensor
CPU cost 및 ScenarioLoad 자체를 별도 A/B한다.

### 4.4 원인 분리와 minimal hybrid 최종 판정

MR과 MRP의 full ROS Ego profile을 정규화해 비교하면 Ego `UNIQUEID=0`의 23개 interface는
동일했다. 하지만 launcher generation은 MR `S4.251001.MolitComp03`, MRP
`R1.260701.H3`로 다르고 `morai_msgs/EgoVehicleStatus`, `CtrlCmd` MD5도 서로 다르다.
따라서 launcher에 맞는 overlay를 사용하되, MR에는 full 23-entry profile을 넣지 않는다.

MR minimal five-interface profile을 사용한 최종 2-scene pilot 결과:

| Scene | Asset | LiDAR | 4-camera | Ego/Object | 결과 |
|---|---|---:|---:|---:|---|
| `wba015` | CargoBox | 11.90 Hz | 29.93 Hz | 47.96/47.60 Hz | PASS |
| `wba016` | WoodBox | 12.63 Hz | 29.60 Hz | 48.01/47.65 Hz | PASS |

- 두 scene 모두 ScenarioLoad, sidecar reconnect, AV external control, 약 2.3 m 이동,
  scene당 3 trigger를 완료했다.
- camera--LiDAR nearest age p50은 camera별 `2.0--3.7 ms`, p95는 약 `26 ms`였다.
- 작은 Ego header reversal 1회(`2.0 ms`)는 timestamp sort로 처리하며 audit warning이다.
- raw bag audit error는 0이고 PASS다.

근거:

```text
rosbags/datasets/base4_temporal/mr_minimal_hybrid_pilot/
260920_202943_mr_minimal_hybrid_2scene_0.bag

/home/luv/morai_cap/260920_202957_base4_mr_minimal_2scene/

artifacts/datasets/base4_mr_minimal_hybrid_pilot_bag_audit.json
```

### 4.5 MRP 실패 asset의 MR 복구 시험

MRP에서 direct ScenarioLoad가 실패했던 네 candidate를 같은 class control과 두 위치에서
다시 검사했다. MR minimal profile에서는 control 8개와 probe 8개, 총 16/16 scene이
모두 첫 시도에 정상 load됐고 `/Object_topic` 이름까지 일치했다.

| Asset | MRP direct-load | MR minimal direct-load | MR 정책 |
|---|---:|---:|---|
| `2017_Hyundai_Grandeur` | 0/2, 별도 0/5 | 2/2 | 복구 후보 |
| `police_f_1` | 0/2 | 2/2 | 복구 후보 |
| `Boy2` | 0/2 | 2/2 | 복구 후보 |
| `Scooter_01` | 0/2 | 2/2 | 복구 후보 |

따라서 이 네 자산을 전역 미지원으로 부르면 안 된다. MRP catalog/template 호환성
문제이며 대회 계열 MR에서는 생성 후보로 복구할 수 있다. 다만 실제 Base4 positive
pool 편입은 sensor visibility와 GT 정합 audit를 별도로 통과해야 한다.

```text
/home/luv/morai_cap/260920_asset_revalidation_mr_minimal/
```

## 5. 혼선 방지 규칙

1. 일반 주행 full-UDP와 자동 dataset hybrid mode를 같은 실행으로 섞지 않는다.
2. MR에는 MRP GUI 원본을 복사하지 않고 tracked MR minimal profile을 쓴다.
3. MRP에는 tracked MRP minimal profile을 쓴다. 현재 payload가 같아도 provenance와
   설치 경로는 launcher별로 유지한다.
4. sidecar는 scenario load 뒤 ROS 연결을 복구하는 실행 계약의 일부다.
5. `rostopic list`만 보고 정상이라 판단하지 않고 publisher/subscriber 주체를 확인한다.
6. rosbridge reconnect가 실패하면 검증 gate를 추가하기 전에 MORAI Ego Network를 먼저
   재연결한다.
