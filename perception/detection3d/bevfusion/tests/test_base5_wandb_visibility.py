"""Animal charts visible; existing E25 metric names/epoch axes unchanged."""
import unittest
from unittest.mock import Mock, patch

from mmengine.visualization import WandbVisBackend
from perception.detection3d.bevfusion.plugin.morai_wandb import MoraiWandbVisBackend


class WandbVisibilityTest(unittest.TestCase):
    def test_four_class_ap_is_visible_without_renaming_existing_metrics(self):
        backend = object.__new__(MoraiWandbVisBackend)
        backend._wandb = Mock()
        with patch.object(WandbVisBackend, '_init_env'):
            backend._init_env()
        definitions = {call.args[0]: call.kwargs
                       for call in backend._wandb.define_metric.call_args_list}
        for name in ('vehicle', 'pedestrian', 'obstacle', 'animal'):
            for key in (f'val/center/{name}_mean_AP', f'val/center/{name}_AP_1m',
                        f'val/operating/{name}_1m_recall'):
                self.assertFalse(definitions[key]['hidden'])
                self.assertEqual(definitions[key]['step_metric'], 'epoch')
        self.assertEqual(definitions['val/center/mAP']['summary'], 'max')

    def test_canonical_names_epoch_and_step_remain_unchanged(self):
        backend = object.__new__(MoraiWandbVisBackend)
        backend._current_epoch = 0
        values = backend._canonicalize(dict(epoch=2, loss=1.5, lr=.001), step=40)
        self.assertEqual(values, dict(epoch=2, global_step=40, **{'train/loss':1.5,'train/lr':.001}))
        values = backend._canonicalize({'val/center/animal_mean_AP':.2}, step=40)
        self.assertEqual(values['epoch'],2)
        self.assertEqual(values['val/center/animal_mean_AP'],.2)


if __name__ == '__main__':
    unittest.main()
