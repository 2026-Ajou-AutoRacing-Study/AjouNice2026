#!/usr/bin/env python3
"""No ROS/network/model execution: projection, identity, freshness and uncertainty."""
import copy
import json
import math
import sys
import unittest
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[3]
sys.path.insert(0, str(PACKAGE / "scripts"))
from traffic_light_map_selector import (CameraModel, RigidTransform, associate_boxes, covariance_sigmas,
    load_camera, load_geometry, pose_tf_consistency, project_heads, quaternion_matrix,
    rotate, select_packet, uncertainty_padding)
from traffic_light_map_selector import selector_config, DEFAULT_SELECTOR_CONFIG


def box(xyxy, confidence=.9):
    x1, y1, x2, y2 = xyxy
    return ((x1+x2)/2560., (y1+y2)/1440., (x2-x1)/1280., (y2-y1)/720.,
            (x1, y1, x2, y2, confidence))


def camera_transform(position=(0., 0., 0.), corrected_yaw_degrees=0., camera=None):
    """Offline fixture ONLY: input yaw is already corrected, no hidden offset."""
    yaw = math.radians(corrected_yaw_degrees)
    world_to_body = quaternion_matrix((0, 0, -math.sin(yaw/2), math.cos(yaw/2)))
    body_to_camera = quaternion_matrix(tuple(-v for v in camera.quaternion[:3]) + (camera.quaternion[3],))
    rotation = tuple(tuple(sum(body_to_camera[i][k]*world_to_body[k][j] for k in range(3))
                           for j in range(3)) for i in range(3))
    body_translation = rotate(world_to_body, tuple(-v for v in position))
    translation = rotate(body_to_camera, tuple(body_translation[i]-camera.position[i] for i in range(3)))
    return RigidTransform(rotation, translation)


def simple_head(identifier="target", group=10, center=(20., 0., 2.), heading=180.):
    return {"id": identifier, "type": "car", "group_id": group, "point": list(center),
            "heading": heading, "width": 1.4, "height": .355}


class MapSelectorTest(unittest.TestCase):
    def setUp(self):
        self.camera = CameraModel(1280, 720, 640., 640., 640., 360., "camera_front",
                                  (0., 0., 0.), (0., 0., 0., 1.))
        self.identity = RigidTransform.from_quaternion((0, 0, 0), (0, 0, 0, 1))

    def project(self, heads, group=10, **options):
        return project_heads({"heads": heads}, self.camera, self.identity, group, **options)

    def test_projection_axes_and_center_no_second_z_offset(self):
        head = simple_head(center=(20, 2, 3))
        head["z_offset"] = 4.5
        rois, _ = self.project([head])
        bbox = rois[0]["bbox_xyxy"]
        self.assertAlmostEqual((bbox[0]+bbox[2])/2, 576.)
        self.assertAlmostEqual((bbox[1]+bbox[3])/2, 264.)

    def test_front_facing_not_merely_positive_dot(self):
        heads = [simple_head("front"), simple_head("side", heading=100.),
                 simple_head("back", heading=0.)]
        rois, rejected = self.project(heads)
        self.assertEqual([r["head_id"] for r in rois], ["front"])
        self.assertEqual({r["head_id"] for r in rejected}, {"side", "back"})
        self.assertTrue(all(r["reason"] == "not_front_facing" for r in rejected))

    def test_behind_outside_tiny_and_clipped_are_not_eligible(self):
        heads = [simple_head("behind", center=(-20, 0, 2)),
                 simple_head("outside", center=(20, 100, 2)),
                 simple_head("tiny", center=(170, 0, 2)),
                 simple_head("clipped", center=(20, 0, 11.2))]
        rois, rejected = self.project(heads)
        self.assertFalse(rois)
        self.assertEqual(len(rejected), 4)

    def test_different_route_never_used_as_center_fallback(self):
        rois, _ = self.project([simple_head("other", group=20)])
        best, reason = associate_boxes([box(rois[0]["bbox_xyxy"])], rois, 10)
        self.assertIsNone(best)
        self.assertEqual(reason["status"], "missing")

    def test_same_program_both_heads_permitted_larger_quality_preferred(self):
        rois, _ = self.project([simple_head("near"), simple_head("far", center=(32, 2, 2))])
        boxes = [box(r["bbox_xyxy"]) for r in rois]
        best, result = associate_boxes(boxes, rois, 10, previous_head_id="far")
        self.assertIs(best, boxes[0])
        self.assertEqual(result["head_id"], "near")

    def test_other_head_ambiguity_vetoes_allowed_identity(self):
        rois, _ = self.project([simple_head("allowed"), simple_head("other", group=20)])
        best, result = associate_boxes([box(rois[0]["bbox_xyxy"])], rois, 10)
        self.assertIsNone(best)
        self.assertEqual(result["rejected"][0]["reason"], "other_head_identity_ambiguous")

    def test_small_pitch_or_pose_residual_tolerated_without_relabel(self):
        rois, _ = self.project([simple_head()])
        bbox = rois[0]["bbox_xyxy"]
        observed = [bbox[0]+7, bbox[1]-12, bbox[2]+7, bbox[3]-12]
        best, _ = associate_boxes([box(observed)], rois, 10)
        self.assertIsNotNone(best)

    def test_large_displacement_or_implausible_box_size_rejected(self):
        rois, _ = self.project([simple_head()])
        bbox = rois[0]["bbox_xyxy"]
        self.assertIsNone(associate_boxes([box([v+200 for v in bbox])], rois, 10)[0])
        self.assertIsNone(associate_boxes([box([0, 0, 1280, 720])], rois, 10)[0])

    def test_increasing_uncertainty_is_bounded_not_arbitrary_fallback(self):
        p1 = uncertainty_padding(self.camera, 20, .2, math.radians(.35))
        p2 = uncertainty_padding(self.camera, 20, .5, math.radians(1.))
        self.assertGreater(p2[0], p1[0])
        rois, rejected = self.project([simple_head()], position_sigma_m=2.)
        self.assertFalse(rois)
        self.assertEqual(rejected[0]["reason"], "projection_uncertainty_exceeds_bound")

    def test_bad_threshold_cannot_admit_side_back_housing(self):
        with self.assertRaises(ValueError):
            self.project([simple_head()], max_view_angle_degrees=89.)

    def test_invalid_covariance_and_pose_tf_mismatch(self):
        covariance = [0.]*36
        self.assertEqual(covariance_sigmas(covariance), (0., 0., 0.))
        for value in (-1., float("nan"), 4.):
            covariance[0] = value
            with self.assertRaises(ValueError):
                covariance_sigmas(covariance)
        with self.assertRaises(ValueError):
            pose_tf_consistency((0, 0, 0), (0, 0, 0, 1), (4, 0, 0), (0, 0, 0, 1), .02)
        with self.assertRaises(ValueError):
            pose_tf_consistency((0, 0, 0), (0, 0, 0, 0), (0, 0, 0), (0, 0, 0, 1), .02)
        distance, _ = pose_tf_consistency((0, 0, 0), (0, 0, 0, 1), (.6, 0, 0), (0, 0, 0, 1), .1, 2.)
        self.assertAlmostEqual(distance, .6)

    def test_live_GNSS_z_variance_never_inflates_horizontal_padding(self):
        covariance = [0.]*36
        covariance[0], covariance[7], covariance[14], covariance[35] = .0007363848, .0004497517, 1., .0000522796
        xy, z, yaw = covariance_sigmas(covariance)
        self.assertLess(xy, .03)
        self.assertEqual(z, 1.)
        padding = uncertainty_padding(self.camera, 30., xy, .02905, vertical_sigma_m=z, camera_up_m=5.)
        self.assertLess(padding[0], 60.)
        self.assertGreater(padding[1], 65.)
        covariance[14] = .25
        new_xy, new_z, new_yaw = covariance_sigmas(covariance)
        self.assertEqual((new_xy, new_yaw), (xy, yaw))
        self.assertEqual(new_z, .5)
        lower_z = uncertainty_padding(self.camera, 30., xy, .02905, vertical_sigma_m=new_z, camera_up_m=5.)
        self.assertEqual(padding[0], lower_z[0])
        self.assertGreater(padding[1], lower_z[1])

    def test_independent_vertical_bound_keeps_reported_z_uncertainty_and_X90(self):
        with self.assertRaises(ValueError) as failure:
            uncertainty_padding(self.camera, 18., .03, math.radians(.5), vertical_sigma_m=1., camera_up_m=5.,
                                max_vertical_padding_px=90.)
        self.assertEqual(failure.exception.bounds_xy_px, (90., 90.))
        padding = uncertainty_padding(self.camera, 18., .03, math.radians(.5), vertical_sigma_m=1.,
                                      camera_up_m=5., max_vertical_padding_px=180.)
        self.assertLess(padding[0], 90.)
        self.assertGreater(padding[1], 90.)
        self.assertLess(padding[1], 180.)
        with self.assertRaises(ValueError):
            uncertainty_padding(self.camera, 18., 1.5, math.radians(.5), vertical_sigma_m=1.,
                                max_vertical_padding_px=180.)

    def test_XY_covariance_uses_principal_axis_and_separate_z_limit(self):
        covariance = [0.]*36
        covariance[0] = covariance[7] = .01
        covariance[1] = covariance[6] = .009
        self.assertAlmostEqual(covariance_sigmas(covariance)[0], math.sqrt(.019))
        covariance[14] = 4.
        with self.assertRaisesRegex(ValueError, "vertical"):
            covariance_sigmas(covariance)
        covariance[14] = 0.
        covariance[1] = covariance[6] = .02
        with self.assertRaisesRegex(ValueError, "semidefinite"):
            covariance_sigmas(covariance)

    def test_single_configuration_contract_and_horizontal_housing_aspect_guard(self):
        config = selector_config()
        self.assertEqual(config.max_padding_px, 90.)
        self.assertEqual(config.max_vertical_padding_px, 180.)
        self.assertEqual(config.min_aspect_ratio_factor, .4)
        for bad in ({"unknown": 1}, {"max_padding_px": -1}, {"min_aspect_ratio_factor": 3., "max_aspect_ratio_factor": 2.}):
            with self.assertRaises(ValueError): selector_config(bad)
        rois, _ = self.project([simple_head()], vertical_sigma_m=1., max_vertical_padding_px=180.)
        bbox = rois[0]["bbox_xyxy"]
        width, height = bbox[2]-bbox[0], bbox[3]-bbox[1]
        cx, cy = (bbox[0]+bbox[2])/2, (bbox[1]+bbox[3])/2+2.8*640/20.
        pedestrian_shaped = box((cx-.55*width/2, cy-2.8*height/2, cx+.55*width/2, cy+2.8*height/2))
        self.assertIsNone(associate_boxes([pedestrian_shaped], rois, 10)[0])
        unguarded = selector_config({"min_aspect_ratio_factor": .01, "max_aspect_ratio_factor": 100.})
        self.assertIsNotNone(associate_boxes([pedestrian_shaped], rois, 10, config=unguarded)[0])

    def test_launch_gate_defaults_match_the_shared_configuration(self):
        import xml.etree.ElementTree as ET
        args = {a.attrib["name"]: a.attrib.get("default") for a in
                ET.parse(PACKAGE/"launch/traffic_light_node.launch").getroot().findall("arg")}
        for field in ("max_padding_px", "max_vertical_padding_px", "min_aspect_ratio_factor", "max_aspect_ratio_factor",
                      "max_position_sigma_m", "max_vertical_sigma_m", "max_yaw_sigma_degrees",
                      "max_view_angle_degrees", "min_height_px", "base_margin_px", "pitch_tolerance_degrees"):
            self.assertEqual(float(args["map_"+field]), getattr(DEFAULT_SELECTOR_CONFIG, field))

    def test_packet_group_time_resolution_frame_and_zero_stamp_gates(self):
        packet = {"schema_version": 1, "source_stamp_ns": 1000000000,
                  "generated_stamp_ns": 1020000000, "group_id": 10,
                  "image_width": 1280, "image_height": 720, "camera_frame": "camera_front"}
        def select(p=packet, source=1000000000, group=10, width=1280, now=1100000000, frame="camera_front"):
            return select_packet([p], source, group, width, 720, now, camera_frame=frame)
        self.assertIs(select(), packet)
        self.assertIsNone(select(group=20))
        self.assertIsNone(select(group=0))
        self.assertIsNone(select(source=0))
        self.assertIsNone(select(source=1100000000))
        self.assertIsNone(select(width=640))
        self.assertIsNone(select(now=2000000000))
        self.assertIsNone(select(now=1000000000))
        self.assertIsNone(select(frame="camera_right"))
        self.assertIsNone(select(p={"bad": "packet"}))

    def test_nearest_packet_not_latest_wrong_frame(self):
        base = {"schema_version": 1, "source_stamp_ns": 1000000000,
                "generated_stamp_ns": 1020000000, "group_id": 10,
                "image_width": 1280, "image_height": 720}
        newer = dict(base, source_stamp_ns=1030000000)
        self.assertIs(select_packet([newer, base], 1000000000, 10, 1280, 720, 1100000000), base)

    def test_static_profile_vehicle_whitelist_same_group_and_no_D1_guess(self):
        document = load_geometry(PACKAGE / "config/traffic_light_map_geometry.json")
        ids = {h["id"]: h["group_id"] for h in document["heads"]}
        self.assertEqual(ids["C1256W000016"], 201290)
        self.assertEqual(ids["C1256W000018"], 201290)
        self.assertIsNone(ids["C1256W000047"])
        self.assertNotIn(201397, ids.values())
        self.assertTrue(all(h["type"] == "car" for h in document["heads"]))

    def test_camera_profile_uses_HFOV_not_inconsistent_stored_focal(self):
        camera = load_camera(ROOT / "configs/sensors/Sensor_competition_local.json")
        self.assertEqual((camera.width, camera.height, camera.frame), (1280, 720, "camera_front"))
        self.assertAlmostEqual(camera.fx, 640.)
        self.assertEqual(camera.position, (1.9, 0., 1.2))

    def test_archived_180_pose_projection_matches_original_equations(self):
        dataset = ROOT / "artifacts/evaluation/traffic_light_robustness_261025/crop_probe_dataset_v1.json"
        if not dataset.exists():
            self.skipTest("optional immutable offline evidence is not installed")
        records = json.loads(dataset.read_text())["records"]
        document = load_geometry(PACKAGE / "config/traffic_light_map_geometry.json")
        camera = load_camera(ROOT / "configs/sensors/Sensor_competition_local.json")
        groups = {h["id"]: h["group_id"] for h in document["heads"]}
        compared = 0
        for record in records:
            provenance = record["provenance"]
            ego, oracle = provenance["ego"], provenance["oracle_projection"]
            head_id = provenance["requested_native_id"]
            # This fixture converts RAW archived Ego exactly once. Runtime never does this.
            transform = camera_transform(ego["position_xyz_m"], ego["heading_degrees"]-1.347, camera)
            rois, _ = project_heads(document, camera, transform, groups[head_id])
            roi = next(r for r in rois if r["head_id"] == head_id)
            for actual, expected in zip(roi["bbox_xyxy"], oracle["bbox_xyxy_float_no_margin"]):
                self.assertAlmostEqual(actual, expected, places=7)
            selected, result = associate_boxes([box(roi["bbox_xyxy"])], rois, groups[head_id])
            self.assertIsNotNone(selected)
            self.assertEqual(result["head_id"], head_id)
            compared += 1
        self.assertEqual(compared, 180)

    def test_archived_pose_residual_sweep_and_C_side_head_rejection(self):
        dataset = ROOT / "artifacts/evaluation/traffic_light_robustness_261025/crop_probe_dataset_v1.json"
        if not dataset.exists():
            self.skipTest("optional immutable offline evidence is not installed")
        records = json.loads(dataset.read_text())["records"]
        document = load_geometry(PACKAGE / "config/traffic_light_map_geometry.json")
        camera = load_camera(ROOT / "configs/sensors/Sensor_competition_local.json")
        groups = {h["id"]: h["group_id"] for h in document["heads"]}
        seen, compared = set(), 0
        for record in records:
            if record["group_id"] in seen:
                continue
            seen.add(record["group_id"])
            provenance = record["provenance"]
            ego, identifier = provenance["ego"], provenance["requested_native_id"]
            yaw, position = ego["heading_degrees"]-1.347, ego["position_xyz_m"]
            rois, rejected = project_heads(document, camera, camera_transform(position, yaw, camera), groups[identifier])
            roi = next(r for r in rois if r["head_id"] == identifier)
            if record["group_id"] == "C20_center":
                reasons = {r["head_id"]: r["reason"] for r in rejected}
                for side in ("C1256W000017", "C1256W000074"):
                    self.assertEqual(reasons[side], "not_front_facing")
                for tiny in ("C1256W000015", "C1256W000123"):
                    self.assertEqual(reasons[tiny], "housing_too_small")
            for dx, dy, dz, dyaw in ((.3, 0, 0, 0), (-.3, 0, 0, 0), (0, .3, 0, 0), (0, -.3, 0, 0),
                                    (0, 0, .2, 0), (0, 0, -.2, 0), (0, 0, 0, 1), (0, 0, 0, -1)):
                perturbed = [position[i]+offset for i, offset in enumerate((dx, dy, dz))]
                changed, _ = project_heads(document, camera, camera_transform(perturbed, yaw+dyaw, camera), groups[identifier])
                selected, result = associate_boxes([box(roi["bbox_xyxy"])], changed, groups[identifier])
                self.assertIsNotNone(selected)
                self.assertEqual(result["head_id"], identifier)
                compared += 1
        self.assertEqual(compared, 64)

    def test_old_development_val_saved_detections_asymmetric_gate_regression(self):
        directory = ROOT/"artifacts/evaluation/traffic_light_robustness_261025"
        paths = [directory/name for name in ("housing_detector_development_curated_v1.json",
                 "full_cnn_raw_dataset_v3.json", "pipeline_baseline640_development348_v1.json")]
        if not all(path.exists() for path in paths):
            self.skipTest("optional previously observed development replay evidence not installed")
        sys.path.insert(0, str(ROOT/"scripts/evaluation"))
        import evaluate_traffic_light_pipeline as evaluator
        source = evaluator.load_rows(paths[0])
        evidence = evaluator.index_rows(evaluator.load_rows(paths[1]))
        predictions = {row["image_sha256"]: row["detections"] for row in json.loads(paths[2].read_text())["records"]}
        geometry = load_geometry(PACKAGE/"config/traffic_light_map_geometry.json")
        camera = load_camera(ROOT/"configs/sensors/Sensor_competition_local.json")
        id_groups = {head["id"]: head.get("group_id") for head in geometry["heads"]}
        val = [row for row in source if row.get("split") == "val"]
        self.assertEqual(len(val), 66)
        for vertical_bound, expected_selected in ((90., 24), (180., 66)):
            config = selector_config({"max_vertical_padding_px": vertical_bound})
            selected_count = wrong = 0
            for row in val:
                group = evaluator.frame_group(row, geometry, evidence[row["image_sha256"]])
                transform = evaluator.development_camera_transform(row["ego"], camera)
                rois, _ = project_heads(geometry, camera, transform, group, math.sqrt(.0007363848), .02905,
                    vertical_sigma_m=1., **config.projection_kwargs())
                boxes = [evaluator.make_box(box[:4], box[4], 1280, 720) for box in predictions[row["image_sha256"]]]
                best, _ = associate_boxes(boxes, rois, group, config=config)
                identity = evaluator.manual_identity(best[4][:4] if best else None, row["boxes"])
                selected_count += best is not None
                wrong += identity["known"] and id_groups.get(identity.get("native_id")) != group
            self.assertEqual(selected_count, expected_selected)
            self.assertEqual(wrong, 0)


if __name__ == "__main__":
    unittest.main()
