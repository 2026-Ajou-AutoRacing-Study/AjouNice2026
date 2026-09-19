#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Usage: $0 {obstacle|roundabout|highway_merge|highway1|pedestrian_crossing|pedestrian_distance} [RATE]"
    exit 2
fi

CASE="$1"
RATE="${2:-1.0}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
OUTPUT_ROOT="$ROOT/artifacts/qualitative/pp1_class_thresholds_260912"
ROS_PORT="${PP1_RVIZ_ROS_PORT:-11352}"

case "$CASE" in
    obstacle|obstacle_backforth)
        CASE="obstacle_backforth"
        RAW_BAG="$ROOT/rosbags/raw_sensor_w_cam/perception_benchmark/260912_165936_obstacle_backforth_mr.bag"
        ;;
    roundabout)
        RAW_BAG="$ROOT/rosbags/raw_sensor_w_cam/perception_benchmark/260821_235243_roundabout_dynamic_competition_raw.bag"
        ;;
    highway_merge|merge)
        CASE="highway_merge"
        RAW_BAG="$ROOT/rosbags/raw_sensor_w_cam/perception_benchmark/260821_235409_highway_merge_dynamic_competition_raw.bag"
        ;;
    highway1)
        RAW_BAG="$ROOT/rosbags/raw_sensor_w_cam/sensor_cam_raw_highway1.bag"
        ;;
    pedestrian_crossing|ped_crossing)
        CASE="pedestrian_crossing"
        RAW_BAG="$ROOT/rosbags/raw_sensor_w_cam/pointpainting_eval_260730/pointpainting_pedestrian_crossing_raw.bag"
        ;;
    pedestrian_distance|ped_distance)
        CASE="pedestrian_distance"
        RAW_BAG="$ROOT/rosbags/raw_sensor_w_cam/pointpainting_eval_260730/pointpainting_pedestrian_distance_5_10_15_18m_raw.bag"
        ;;
    *)
        echo "Unknown case: $CASE"
        echo "Choose obstacle, roundabout, highway_merge, highway1, pedestrian_crossing, or pedestrian_distance."
        exit 2
        ;;
esac

CASE_DIR="$OUTPUT_ROOT/$CASE"
PIPELINE_BAG="$CASE_DIR/pipeline_outputs.bag"
GT_PATH_BAG="$CASE_DIR/gt_future_paths.bag"
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
export ROS_HOME="/tmp/ajou_pp1_qualitative_rviz_${ROS_PORT}"
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
echo "PP1 qualitative RViz: $CASE"
echo "Vehicle=0.40 Pedestrian=0.15 Obstacle=0.10"
echo "Playback rate: ${RATE}x"
echo "RViz is starting. Begin screen recording, then press SPACE."
echo "Press SPACE again to pause; Ctrl-C closes playback and RViz."
echo "============================================================"

rosbag play --clock --pause --keep-alive --rate="$RATE" \
    "$RAW_BAG" \
    "$PIPELINE_BAG" \
    "$GT_PATH_BAG"
