"""Optional crop evidence must not block startup or current signal inference.

Fake ROS/model only: no ROS master, model load, CUDA or MORAI access.
"""
from pathlib import Path
import tempfile
import types
from unittest.mock import Mock, patch

import cv2
import numpy as np
import pytest

from test_traffic_light_candidate_node import PACKAGE, load_node


@pytest.mark.parametrize('error', [PermissionError('read-only'), OSError('disk full'), ValueError('bad path')])
def test_unavailable_debug_directory_does_not_prevent_node_start(error):
    module = load_node()
    with tempfile.TemporaryDirectory() as directory:
        weights = Path(directory) / 'fixture.pt'
        weights.write_bytes(b'fake local weights; no model is loaded')
        params = {'~weights_path': str(weights), '~debug_save_crop_dir': '/debug-only',
                  '~signal_profile_path': str(PACKAGE / 'config/traffic_signal_profiles.yaml')}
        model = types.SimpleNamespace(device='cpu', predict=Mock())
        with patch.object(module.rospy, 'get_param', side_effect=lambda key, default=None:
                          params.get(key, default), create=True), \
                patch.object(module.rospy, 'Publisher', side_effect=lambda *a, **k: Mock(), create=True), \
                patch.object(module.rospy, 'Subscriber', create=True), \
                patch.object(module.os, 'makedirs', side_effect=error), \
                patch.object(module, 'YOLO', return_value=model):
            node = module.TrafficLightNode()
        assert node.debug_save_crop_dir == ''
        assert node.model is model
        assert node.classifier_mode == 'hsv'  # Preserve this fixture's explicit original mode.
        model.predict.assert_not_called()
        module.rospy.logwarn.assert_called_once()


@pytest.mark.parametrize('write_result', [False, OSError('disk full'), cv2.error('write failed')])
def test_crop_write_failure_disables_only_debug_capture(write_result):
    module = load_node()
    node = object.__new__(module.TrafficLightNode)
    node.debug_save_crop_dir, node._save_crop_count = '/debug-only', 0
    sentinel = node.model = object()
    raw = [('red', 'circle', 1.)]
    options = {'side_effect': write_result} if isinstance(write_result, Exception) else {'return_value': write_result}
    with patch.object(module.cv2, 'imwrite', **options) as writer:
        node._maybe_save_conflict_crop(np.zeros((4, 4, 3), np.uint8), raw)
        node._maybe_save_conflict_crop(np.zeros((4, 4, 3), np.uint8), raw)
    assert writer.call_count == 1
    assert node.debug_save_crop_dir == ''
    assert node.model is sentinel
    module.rospy.logwarn.assert_called_once()


def test_successful_optional_crop_write_is_preserved():
    module = load_node()
    node = object.__new__(module.TrafficLightNode)
    node.debug_save_crop_dir, node._save_crop_count = '/debug-only', 0
    with patch.object(module.cv2, 'imwrite', return_value=True) as writer:
        node._maybe_save_conflict_crop(np.zeros((4, 4, 3), np.uint8), [('red', 'circle', 1.)])
    assert writer.call_count == 1
    assert node.debug_save_crop_dir == '/debug-only'
    module.rospy.logwarn.assert_not_called()
