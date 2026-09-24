# E25 rosbag Tracking/Prediction RViz runbook

> **문서 상태: CURRENT.** Production `B3-BF0-F40 E25`의 저장된 detection,
> tracking, Map-CV prediction을 mission rosbag과 함께 재생하는 표준 절차다.

## 목적과 기본 계약

- 기본 재생 속도는 `1.0x`다.
- `0.25x` 또는 `0.5x`는 BBox 정합이나 순간 오검출을 정밀 검사할 때만 쓴다.
- 분홍색은 asset catalog로 center/size를 보정한 MORAI GT다.
- 입력은 raw sensor bag과 이미 계산해 둔 E25 `pipeline_outputs.bag`이다. 이 절차는
  detector를 다시 추론하지 않는다.
- `pipeline_prediction_rviz.launch`는 canonical detection, tracking, prediction을
  RViz Marker로 변환하는 `object_pipeline_visualization`을 포함한다.

## Terminal 1: ROS master

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source scripts/workspace/source_aw149.sh
source base_ws/devel/setup.bash

roscore
```

## Terminal 2: 지도, 보정 GT, object visualization, RViz

`source base_ws/devel/setup.bash`는 `PYTHONPATH`를 ROS 1 devel 기준으로 다시 쓴다.
그 전에 AW149 Python/Library 경로를 반드시 보존하고, `roslaunch`에 명시적으로
전달한다. 이를 생략하면 visualization node가 ROS 2 compatibility message인
`builtin_interfaces`를 import하지 못하고 GT만 보일 수 있다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source scripts/workspace/source_aw149.sh

AW149_PYTHONPATH="$PYTHONPATH"
AW149_LD_LIBRARY_PATH="$LD_LIBRARY_PATH"

source base_ws/devel/setup.bash
export AJOUNICE_ROOT="$PWD"
rosparam set /use_sim_time true

env \
  PYTHONPATH="$AW149_PYTHONPATH" \
  LD_LIBRARY_PATH="$AW149_LD_LIBRARY_PATH" \
  PYTHONNOUSERSITE=1 \
  /usr/bin/python3 /opt/ros/noetic/bin/roslaunch \
  tracking_prediction_evaluation pipeline_prediction_rviz.launch \
  use_sim_time:=true \
  start_rviz:=true \
  show_gt_boxes:=true \
  use_heading_correction:=true \
  morai_heading_correction_deg:=-1.347 \
  asset_bbox_catalog:="$PWD/dataset/morai_od/configs/morai_asset_bbox_offsets_base3_v1.json"
```

## Terminal 3: 고주로1 1.0x 재생

실행 후 `Space`로 시작/일시정지하고 `Ctrl-C`로 종료한다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source scripts/workspace/source_aw149.sh
source base_ws/devel/setup.bash

rosbag play \
  --clock \
  --pause \
  --rate=1.0 \
  rosbags/raw_sensor_w_cam/sensor_cam_raw_highway1.bag \
  artifacts/evaluation/bevfusion/bf_e40fresh_e25_tracking_mapcv_260918/vehicle_highway1/pipeline_outputs.bag
```

## 다른 Vehicle 구간

Terminal 1과 2는 유지하고 Terminal 3의 bag만 바꾼다.

### 고주로 합류

```bash
rosbag play --clock --pause --rate=1.0 \
  rosbags/raw_sensor_w_cam/perception_benchmark/260821_235409_highway_merge_dynamic_competition_raw.bag \
  artifacts/evaluation/bevfusion/bf_e40fresh_e25_tracking_mapcv_260918/vehicle_highway_merge/pipeline_outputs.bag
```

### 로터리

```bash
rosbag play --clock --pause --rate=1.0 \
  rosbags/raw_sensor_w_cam/perception_benchmark/260821_235243_roundabout_dynamic_competition_raw.bag \
  artifacts/evaluation/bevfusion/bf_e40fresh_e25_tracking_mapcv_260918/vehicle_roundabout_new/pipeline_outputs.bag
```

## GT만 보일 때

먼저 node와 subscriber를 확인한다.

```bash
rosnode list | grep object_pipeline_visualization
rostopic info /perception/object_recognition/tracking/objects
rostopic info /perception/object_recognition/objects
```

정상이라면 `/object_pipeline_visualization`이 존재하고 두 object topic에 해당 node가
subscriber로 표시된다. Marker 출력은 다음과 같이 확인한다.

```bash
rostopic hz /perception/object_recognition/visualization/tracks
rostopic hz /perception/object_recognition/visualization/predictions
```

node가 없고 Terminal 2에 아래 오류가 있으면 AW149 Python 환경이 유실된 것이다.

```text
ModuleNotFoundError: No module named 'builtin_interfaces'
```

Terminal 2를 위의 표준 명령으로 다시 시작한다. 현재 launch를 유지한 채 node만 복구해야
한다면 새 터미널에서 아래를 실행할 수 있다.

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source scripts/workspace/source_aw149.sh
AW149_PYTHONPATH="$PYTHONPATH"
AW149_LD_LIBRARY_PATH="$LD_LIBRARY_PATH"
source base_ws/devel/setup.bash

env \
  PYTHONPATH="$AW149_PYTHONPATH" \
  LD_LIBRARY_PATH="$AW149_LD_LIBRARY_PATH" \
  PYTHONNOUSERSITE=1 \
  rosrun perception_object_pipeline \
  object_pipeline_visualization_node.py \
  __name:=object_pipeline_visualization \
  '~output/detections:=/perception/object_recognition/visualization/detections' \
  '~output/tracks:=/perception/object_recognition/visualization/tracks' \
  '~output/predictions:=/perception/object_recognition/visualization/predictions'
```

복구 명령에서 위 세 remap을 빼면 node는 살아 있지만 Marker를
`/object_pipeline_visualization/output/*`에 발행한다. RViz는
`/perception/object_recognition/visualization/*`를 구독하므로 GT만 보이게 된다.
`rosnode info /object_pipeline_visualization`의 Publications가 반드시 아래 세 canonical
topic이어야 한다.

```text
/perception/object_recognition/visualization/detections
/perception/object_recognition/visualization/tracks
/perception/object_recognition/visualization/predictions
```

중요: `/opt/ros/noetic/bin/roslaunch`는 Python script라 위 표준 명령처럼
`/usr/bin/python3`로 실행할 수 있지만, `/opt/ros/noetic/bin/rosrun`은 Bash script다.
따라서 `python3 /opt/ros/noetic/bin/rosrun ...`처럼 실행하면 안 된다. 그렇게 하면
`function usage()`에서 `SyntaxError`가 발생한다.

`--keep-alive`는 표준 재생 명령에 사용하지 않는다. 재생이 끝난 뒤 player와 `/clock`만
살아 있어 graph가 정상 동작 중인 것처럼 보이지만 object/Marker 메시지는 더 이상 나오지
않아 진단을 혼동시킬 수 있다.
