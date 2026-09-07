#!/usr/bin/env python3

import importlib.util
import unittest
from pathlib import Path

import torch


ROOT = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


builder = load_module(
    "velocity_builder", ROOT / "tools/build_pointpainting_velocity_dataset.py"
)
migrator = load_module(
    "velocity_migrator", ROOT / "tools/expand_pointpillar_velocity_checkpoint.py"
)


class VelocityDatasetToolsTest(unittest.TestCase):
    def test_actor_velocity_uses_absolute_velocity_rotated_to_ego_axes(self):
        actor = {
            "simulation_velocity_lidar_mps": {"x": 2.0, "y": -0.5}
        }
        self.assertEqual(builder.actor_velocity_lidar(actor), (2.0, -0.5))

    def test_label_layout_is_9d_plus_class(self):
        line = builder.format_label(
            [1.0, 2.0, 3.0, 4.0, 2.0, 1.5, 0.1],
            (3.0, -1.0),
            "Vehicle",
        )
        self.assertEqual(len(line.split()), 10)
        self.assertEqual(line.split()[-1], "Vehicle")

    def test_checkpoint_expansion_preserves_each_anchor_geometry(self):
        old = torch.arange(2 * 7 * 3, dtype=torch.float32).reshape(14, 3, 1, 1)
        new = migrator.expand_anchor_channels(old, 7, 9).reshape(2, 9, 3, 1, 1)
        torch.testing.assert_close(new[:, :7], old.reshape(2, 7, 3, 1, 1))
        torch.testing.assert_close(new[:, 7:], torch.zeros_like(new[:, 7:]))

    def test_checkpoint_optimizer_is_removed(self):
        checkpoint = {
            "model_state": {
                migrator.WEIGHT_KEY: torch.ones(14, 2, 1, 1),
                migrator.BIAS_KEY: torch.ones(14),
            },
            "optimizer_state": {"old": True},
            "epoch": 65,
            "it": 100,
        }
        migrated = migrator.migrate_checkpoint(checkpoint)
        self.assertNotIn("optimizer_state", migrated)
        self.assertEqual(migrated["model_state"][migrator.WEIGHT_KEY].shape[0], 18)
        self.assertEqual(migrated["epoch"], -1)

    def test_temporal_feature_is_inserted_before_vfe_offsets(self):
        old = torch.arange(2 * 15, dtype=torch.float32).reshape(2, 15)
        new = migrator.insert_vfe_raw_feature(old, old_raw_feature_count=9)
        self.assertEqual(tuple(new.shape), (2, 16))
        torch.testing.assert_close(new[:, :9], old[:, :9])
        torch.testing.assert_close(new[:, 9], torch.zeros(2))
        torch.testing.assert_close(new[:, 10:], old[:, 9:])


if __name__ == "__main__":
    unittest.main()
