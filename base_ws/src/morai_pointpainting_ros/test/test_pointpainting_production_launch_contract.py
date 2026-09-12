from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _launch():
    return ET.parse(PACKAGE_ROOT / "launch" / "pointpainting_production.launch").getroot()


def test_environment_is_explicit_and_machine_independent():
    args = {item.attrib["name"]: item.attrib for item in _launch().findall("arg")}
    serialized = ET.tostring(_launch(), encoding="unicode")
    assert "default" not in args["python_executable"]
    assert "openpcdet_rtx50" not in serialized
    assert "/home/luv" not in serialized


def test_pp1_production_contract_is_explicit():
    include = _launch().find("include")
    assert include is not None
    values = {item.attrib["name"]: item.attrib["value"] for item in include.findall("arg")}
    expected = {
        "segmentation_preprocess_backend": "opencv",
        "camera_cpu_workers": "4",
        "model_score_threshold": "0.10",
        "vehicle_score_threshold": "0.40",
        "pedestrian_score_threshold": "0.15",
        "obstacle_score_threshold": "0.10",
        "publish_detector_velocity": "false",
        "lidar_measurement_stamp_mode": "object_points_median",
        "lidar_deskew_mode": "off",
        "temporal_num_sweeps": "0",
        "warmup_segmentation": "$(arg warmup_segmentation)",
        "warmup_full_pipeline": "$(arg warmup_full_pipeline)",
    }
    for name, value in expected.items():
        assert values.get(name) == value, (name, values.get(name))
