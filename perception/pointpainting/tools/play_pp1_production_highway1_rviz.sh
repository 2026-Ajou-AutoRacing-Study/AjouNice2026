#!/usr/bin/env bash
set -euo pipefail

RATE="${1:-1.0}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
CASE_DIR="$ROOT/artifacts/qualitative/pp1_production_contract_260913/highway1_fixed30_simtime_fix"
RAW_BAG="$ROOT/rosbags/raw_sensor_w_cam/sensor_cam_raw_highway1.bag"
PIPELINE_BAG="$CASE_DIR/pipeline_outputs.bag"
GT_PATH_BAG="$CASE_DIR/gt_future_paths.bag"
ROS_PORT="${PP1_HIGHWAY_RVIZ_ROS_PORT:-$((16000 + ($$ % 10000)))}"

for path in "$RAW_BAG" "$PIPELINE_BAG" "$GT_PATH_BAG"; do
    if [[ ! -f "$path" ]]; then
        echo "Missing required bag: $path" >&2
        exit 1
    fi
done

export AJOUNICE_ROOT="$ROOT"
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export ROS_IP=127.0.0.1
export ROS_HOME="/tmp/ajou_pp1_highway1_rviz_${ROS_PORT}"
mkdir -p "$ROS_HOME"
source /opt/ros/noetic/setup.bash
source "$ROOT/base_ws/devel/setup.bash"
SYSTEM_PYTHONPATH="$ROOT/base_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages"

ROSCORE_PID=""
RVIZ_PID=""
cleanup() {
    set +e
    for pid in "$RVIZ_PID" "$ROSCORE_PID"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null
            wait "$pid" 2>/dev/null
        fi
    done
}
trap cleanup EXIT INT TERM

env PYTHONPATH="$SYSTEM_PYTHONPATH" PYTHONNOUSERSITE=1 \
/usr/bin/python3 /opt/ros/noetic/bin/roscore -p "$ROS_PORT" \
    >"$CASE_DIR/rviz_roscore.log" 2>&1 &
ROSCORE_PID=$!
for _ in $(seq 1 100); do
    rosparam list >/dev/null 2>&1 && break
    sleep 0.1
done
if ! kill -0 "$ROSCORE_PID" 2>/dev/null || ! rosparam list >/dev/null 2>&1; then
    echo "RViz roscore failed on port $ROS_PORT" >&2
    cat "$CASE_DIR/rviz_roscore.log" >&2
    exit 1
fi

env PYTHONPATH="$SYSTEM_PYTHONPATH" PYTHONNOUSERSITE=1 \
/usr/bin/python3 /opt/ros/noetic/bin/roslaunch \
    tracking_prediction_evaluation pipeline_prediction_rviz.launch \
    use_sim_time:=true \
    start_rviz:=true \
    show_gt_boxes:=true \
    map_file:="$ROOT/kcity_map/lanelet_map/lanelet2_map_v69.osm" \
    projector_file:="$ROOT/kcity_map/lanelet_map/map_projector_info_v1.yaml" \
    >"$CASE_DIR/rviz_launch.log" 2>&1 &
RVIZ_PID=$!

for _ in $(seq 1 200); do
    if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
        echo "RViz launch failed:" >&2
        tail -n 120 "$CASE_DIR/rviz_launch.log" >&2
        exit 1
    fi
    rosnode list 2>/dev/null | grep -q '^/object_pipeline_visualization$' && break
    sleep 0.1
done

echo "============================================================"
echo "B3S-PP1 E34 production contract / Highway 1"
echo "Detector: PP1 + Base3 LRASPP, single-frame, no OD velocity"
echo "Scores: Vehicle=0.40 Pedestrian=0.15 Obstacle=0.10"
echo "Pipeline: EKF internal 30 Hz + unknown stationary + v66 Map-CV"
echo "Colors: detection green, tracks class color, prediction paths,"
echo "        GT boxes magenta, matched GT future paths purple"
echo "Playback: ${RATE}x. Press SPACE to play/pause; Ctrl-C to close."
echo "============================================================"

rosbag play --clock --pause --keep-alive --rate="$RATE" \
    "$RAW_BAG" "$PIPELINE_BAG" "$GT_PATH_BAG"
