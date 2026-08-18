import json
import math

from tracking_prediction_evaluation.bbox_reference import (
    audit_capture_dataset,
    oriented_bev_iou,
)


def write_jsonl(path, items):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(item) + "\n" for item in items))


def test_capture_audit_recovers_scenario_to_box_center_offset(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    root = tmp_path / "dataset"
    scenario_path = tmp_path / "scenario.json"
    scenario_path.write_text(
        json.dumps(
            {
                "vehicleList": [
                    {
                        "DataID": 301,
                        "UNIQUEID": 1001,
                        "initPosition": {
                            "pos": {"x": 10.0, "y": 2.0, "z": 0.0},
                            "rot": {"yaw": "0.0"},
                        },
                    }
                ]
            }
        )
    )
    frames = []
    annotations = []
    for index, ego_x in enumerate((0.0, 1.0)):
        frame_id = f"scene_f{index}"
        frames.append(
            {
                "frame_id": frame_id,
                "scene_id": "scene",
                "frame_index": index,
                "trigger_ego_position": {"x": ego_x, "y": 2.0, "z": 0.0},
            }
        )
        # center_map = ego_x + 0.855 + box_x = actor_x + 1.3
        annotations.append(
            {
                "frame_id": frame_id,
                "boxes": [
                    {
                        "instance_id": 1001,
                        "class": "vehicle",
                        "box_lidar": [10.445 - ego_x, 0.0, -0.9, 4.8, 2.0, 1.5, 0.0],
                    }
                ],
            }
        )
    write_jsonl(root / "metadata" / "frames.jsonl", frames)
    write_jsonl(
        root / "metadata" / "scenes.jsonl",
        [{"scene_id": "scene", "scenario": str(scenario_path)}],
    )
    write_jsonl(root / "annotations" / "boxes_3d.jsonl", annotations)

    result = audit_capture_dataset(root)

    assert result["vehicle_box_count"] == 2
    assert math.isclose(
        result["passenger_sized_under_5_5m"]["longitudinal_offset_m"]["p50"],
        1.3,
        abs_tol=1.0e-9,
    )
    assert math.isclose(
        result["all_assets"]["lateral_offset_m"]["p50"], 0.0, abs_tol=1.0e-9
    )


def test_oriented_bev_iou_handles_identical_and_shifted_boxes():
    box = (0.0, 0.0, 4.0, 2.0, 0.0)
    assert math.isclose(oriented_bev_iou(box, box), 1.0, abs_tol=1.0e-9)
    shifted = (2.0, 0.0, 4.0, 2.0, 0.0)
    assert math.isclose(oriented_bev_iou(box, shifted), 1.0 / 3.0, abs_tol=1.0e-9)
