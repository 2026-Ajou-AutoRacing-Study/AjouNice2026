import json
from pathlib import Path

import numpy as np
from PIL import Image


def _load_module():
    import importlib.util
    import sys

    tools = Path(__file__).parents[3] / "tools" / "segmentation"
    sys.path.insert(0, str(tools))
    path = tools / "prepare_base3_lraspp_4class.py"
    spec = importlib.util.spec_from_file_location("prepare_base3_lraspp", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_training_module():
    import importlib.util

    path = Path(__file__).parents[3] / "tools" / "segmentation" / (
        "train_lraspp_morai_4class_v2.py"
    )
    spec = importlib.util.spec_from_file_location("train_lraspp", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_selected_clips_uses_completed_and_exclusions(tmp_path):
    module = _load_module()
    run = tmp_path / "primary"
    metadata = run / "dynamic_metadata"
    metadata.mkdir(parents=True)
    (run / "summary.json").write_text(json.dumps({
        "completed_scene_ids": ["keep", "drop"],
    }))
    for scene in ("keep", "drop"):
        frames_path = metadata / f"{scene}.frames.jsonl"
        frames_path.write_text("\n".join(json.dumps({
            "frame_index": index,
            "verified": True,
            "new_files": [],
        }) for index in range(3)) + "\n")
        (metadata / f"{scene}.clip.json").write_text(json.dumps({
            "split": "train",
            "frame_count": 3,
            "mission_family": "stopped_vehicle",
            "frames_jsonl": f"dynamic_metadata/{scene}.frames.jsonl",
        }))
    selection = {
        "runs": [{"name": "primary", "exclude_scene_ids": ["drop"]}],
    }
    clips = module.selected_clips(tmp_path, selection)
    assert [clip["scene_id"] for clip in clips] == ["keep"]


def test_scene_camera_files_uses_recorded_artifacts_not_stale_glob(tmp_path):
    module = _load_module()
    camera = tmp_path / "CAMERA_1"
    metadata = tmp_path / "dynamic_metadata"
    camera.mkdir()
    metadata.mkdir()
    records = []
    for index in range(3):
        relative = f"CAMERA_1/current_scene_f{index:02d}_Semantic.png"
        (tmp_path / relative).write_bytes(b"semantic")
        records.append({
            "frame_index": index,
            "verified": True,
            "new_files": [relative],
        })
    (camera / "stale_scene_f00_Semantic.png").write_bytes(b"stale")
    frames = metadata / "scene.frames.jsonl"
    frames.write_text("\n".join(json.dumps(row) for row in records) + "\n")
    paths = module.scene_camera_files({
        "run_root": tmp_path,
        "scene_id": "scene",
        "frames_path": frames,
    }, "CAMERA_1")
    assert [path.name for path in paths] == [
        "current_scene_f00_Semantic.png",
        "current_scene_f01_Semantic.png",
        "current_scene_f02_Semantic.png",
    ]


def test_semantic_palette_is_converted_to_four_class_mask(tmp_path):
    module = _load_module()
    rgba = np.array([[[0, 0, 0, 255], [98, 2, 255, 255],
                      [255, 2, 2, 255], [255, 60, 60, 255],
                      [255, 75, 75, 255], [255, 90, 90, 255],
                      [255, 105, 105, 255], [255, 135, 135, 255],
                      [255, 150, 150, 255], [236, 255, 2, 255]]],
                    dtype=np.uint8)
    path = tmp_path / "semantic.png"
    Image.fromarray(rgba, mode="RGBA").save(path)
    assert module.semantic_to_mask(path).tolist() == [
        [0, 1, 2, 2, 2, 2, 2, 2, 2, 3]
    ]


def test_segmentation_metrics_include_foreground_quality(tmp_path):
    del tmp_path
    import torch

    module = _load_training_module()
    confusion = torch.tensor([
        [8, 2, 0, 0],
        [1, 7, 2, 0],
        [0, 1, 8, 1],
        [0, 0, 2, 8],
    ])
    metrics = module.metrics_from_confusion(confusion)
    assert 0.0 < metrics["foreground_miou"] < 1.0
    assert set(metrics["class_precision"]) == {
        "background", "pedestrian", "vehicle", "obstacle"
    }
    assert abs(metrics["class_recall"]["pedestrian"] - 0.7) < 1e-9
    assert abs(metrics["class_precision"]["obstacle"] - 0.8 / 0.9) < 1e-9
    assert len(metrics["confusion_matrix"]) == 4
    assert len(metrics["confusion_matrix_row_normalized"]) == 4
