import json
import math
import tempfile
import unittest
from pathlib import Path

from tracking_prediction_evaluation.bbox_reference import (
    transform_segments_to_asset_catalog_bbox_center,
)
from tracking_prediction_evaluation.trajectory import TrajectoryPoint, TrajectorySegment


def point(*, name, size, yaw=0.0):
    return TrajectoryPoint(
        timestamp=0.0,
        x=10.0,
        y=20.0,
        z=1.0,
        yaw=yaw,
        longitudinal_velocity=0.0,
        lateral_velocity=0.0,
        longitudinal_acceleration=0.0,
        lateral_acceleration=0.0,
        class_name="vehicle",
        object_id=1,
        asset_name=name,
        length=size[0],
        width=size[1],
        height=size[2],
    )


class AssetCatalogBboxReferenceTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.catalog = Path(self.temp.name) / "catalog.json"
        self.catalog.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "assets": {
                        "CarA": {
                            "class": "vehicle",
                            "pivot_to_bbox_center_local_m": [1.0, 0.2, 0.5],
                            "bbox_size_m": [4.5, 1.8, 1.5],
                        },
                        "CarB": {
                            "class": "vehicle",
                            "pivot_to_bbox_center_local_m": [1.3, 0.0, 0.6],
                            "bbox_size_m": [5.1, 2.1, 1.8],
                        },
                    },
                }
            ),
            encoding="utf-8",
        )

    def tearDown(self):
        self.temp.cleanup()

    def segment(self, name, size, yaw=0.0):
        sample = point(name=name, size=size, yaw=yaw)
        return TrajectorySegment(
            key="vehicle:1",
            class_name="vehicle",
            object_id=1,
            asset_name=name,
            points=(sample,),
        )

    def test_exact_name_uses_rotated_offset_and_catalog_size(self):
        segment = self.segment("CarA", (9.0, 9.0, 9.0), yaw=math.pi / 2.0)
        transformed, report = transform_segments_to_asset_catalog_bbox_center(
            (segment,), self.catalog
        )
        sample = transformed[0].points[0]
        self.assertAlmostEqual(sample.x, 9.8)
        self.assertAlmostEqual(sample.y, 21.0)
        self.assertAlmostEqual(sample.z, 1.5)
        self.assertEqual((sample.length, sample.width, sample.height), (4.5, 1.8, 1.5))
        self.assertEqual(report["resolution_counts"], {"exact-name": 1})

    def test_legacy_link_name_is_resolved_from_vehicle_size(self):
        segment = self.segment("A2256W000406", (4.51, 1.80, 1.50))
        transformed, report = transform_segments_to_asset_catalog_bbox_center(
            (segment,), self.catalog
        )
        self.assertEqual(transformed[0].asset_name, "CarA")
        self.assertEqual(report["resolution_counts"], {"vehicle-size": 1})
        self.assertEqual(report["unresolved_segment_count"], 0)

    def test_unknown_size_is_excluded_instead_of_guessed(self):
        segment = self.segment("unknown", (7.0, 3.0, 3.0))
        transformed, report = transform_segments_to_asset_catalog_bbox_center(
            (segment,), self.catalog
        )
        self.assertEqual(transformed, ())
        self.assertEqual(report["unresolved_reasons"], {"vehicle-size-no-match": 1})

    def test_track_with_multiple_exact_asset_names_is_rejected(self):
        first = point(name="CarA", size=(4.5, 1.8, 1.5))
        second = point(name="CarB", size=(5.1, 2.1, 1.8))
        segment = TrajectorySegment(
            key="vehicle:mixed",
            class_name="vehicle",
            object_id=1,
            asset_name="CarA",
            points=(first, second),
        )
        transformed, report = transform_segments_to_asset_catalog_bbox_center(
            (segment,), self.catalog
        )
        self.assertEqual(transformed, ())
        self.assertEqual(report["unresolved_reasons"], {"multiple-asset-names": 1})


if __name__ == "__main__":
    unittest.main()
