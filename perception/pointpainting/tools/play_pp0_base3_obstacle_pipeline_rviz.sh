#!/usr/bin/env bash
set -euo pipefail

RATE="${1:-1.0}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
CASE_DIR="$ROOT/artifacts/qualitative/pp0_base3_obstacle_benchmark_260912"
RAW_BAG="$ROOT/artifacts/evaluation/pointpainting/260912_base3_obstacle_benchmark/input_trim6s.bag"
PIPELINE_BAG="$CASE_DIR/pipeline_outputs.bag"
GT_PATH_BAG="$CASE_DIR/gt_future_paths.bag"
ROS_PORT="${PP0_BASE3_OBSTACLE_RVIZ_ROS_PORT:-11353}"

for path in "$RAW_BAG" "$PIPELINE_BAG" "$GT_PATH_BAG"; do
    if [[ ! -f "$path" ]]; then
        echo "Missing required bag: $path"
        exit 1
    fi
done

source /opt/ros/noetic/setup.bash
source "$ROOT/base_ws/devel/setup.bash"
export AJOUNICE_ROOT="$ROOT"
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export ROS_IP=127.0.0.1
export ROS_HOME="/tmp/ajou_pp0_base3_obstacle_rviz_${ROS_PORT}"
mkdir -p "$ROS_HOME"

ROSCORE_PID=""
RVIZ_LAUNCH_PID=""
cleanup() {
    set +e
    for pid in "$RVIZ_LAUNCH_PID" "$ROSCORE_PID"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null
            wait "$pid" 2>/dev/null
        fi
    done
}
trap cleanup EXIT INT TERM

roscore -p "$ROS_PORT" >"$CASE_DIR/rviz_roscore.log" 2>&1 &
ROSCORE_PID=$!
for _ in $(seq 1 100); do
    rosparam list >/dev/null 2>&1 && break
    sleep 0.1
done

roslaunch tracking_prediction_evaluation pipeline_prediction_rviz.launch \
    use_sim_time:=true \
    start_rviz:=true \
    show_gt_boxes:=true \
    >"$CASE_DIR/rviz_launch.log" 2>&1 &
RVIZ_LAUNCH_PID=$!

for _ in $(seq 1 200); do
    if ! kill -0 "$RVIZ_LAUNCH_PID" 2>/dev/null; then
        echo "RViz launch failed:"
        tail -n 120 "$CASE_DIR/rviz_launch.log"
        exit 1
    fi
    if rosnode list 2>/dev/null | grep -q '^/object_pipeline_visualization$'; then
        break
    fi
    sleep 0.1
done

echo "============================================================"
echo "PP0 Base3 obstacle pipeline RViz"
echo "Thresholds: Vehicle=0.40 Pedestrian=0.15 Obstacle=0.10"
echo "Inputs: PP0 detection -> EKF tracking -> map-CV prediction"
echo "Green translucent boxes: detector output"
echo "Class-colored boxes and labels: tracker output"
echo "Colored paths: map-CV prediction"
echo "Magenta boxes: MORAI GT"
echo "Purple paths: matched GT future paths"
echo "Playback rate: ${RATE}x"
echo "RViz가 뜨면 SPACE를 눌러 재생. 다시 SPACE는 일시정지."
echo "Ctrl-C로 rosbag과 RViz를 함께 종료."
echo "============================================================"

rosbag play --clock --pause --keep-alive --rate="$RATE" \
    "$RAW_BAG" \
    "$PIPELINE_BAG" \
    "$GT_PATH_BAG"
