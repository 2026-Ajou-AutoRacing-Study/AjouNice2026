import sys
import unittest
import tempfile
from unittest.mock import patch
import numpy as np
from PIL import Image
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from base5_capture_timing import StageTiming, preflight_stationary
from base5_capture_audit import camera_evidence


class TimingTest(unittest.TestCase):
    def test_stages_partition_elapsed_without_altering_clock(self):
        ticks = iter([10., 11., 13., 14.])
        timer = StageTiming(lambda: next(ticks))
        timer.mark('preflight')
        timer.mark('capture')
        timer.mark('preflight')
        result = timer.snapshot()
        self.assertEqual(result['total_sec'], 4.)
        self.assertEqual(result['stages_sec'], dict(preflight=2., capture=2.))
        self.assertEqual(sum(result['stages_sec'].values()), result['total_sec'])

    def test_stationary_witness_rejects_stale_short_moving_or_unstable(self):
        rows=[dict(t=t,x=0.,y=0.,z=1.,vx=0.,vy=0.) for t in (.61,.71,.81,.91,1.)]
        self.assertTrue(preflight_stationary(rows,1.))
        self.assertFalse(preflight_stationary(rows,1.2))
        self.assertFalse(preflight_stationary(rows[-2:],1.))
        for key,value in [('vx',.1),('x',.1),('z',.1),('vy',float('nan'))]:
            changed=[dict(r) for r in rows];changed[-1][key]=value
            self.assertFalse(preflight_stationary(changed,1.),key)

    def test_image_cache_reuses_decode_without_changing_evidence(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);folder=root/'CAMERA_1';folder.mkdir()
            pixels=np.full((20,40,4),[1,2,3,255],dtype=np.uint8)
            image=folder/'x_Instance.png';Image.fromarray(pixels).save(image)
            (folder/'x_BBox2D.txt').write_text('Vehicle 0 0 40 20 7 [1, 2, 3, 255]\n')
            expected=camera_evidence(root,7,'Vehicle')
            cache={}
            with patch('base5_capture_audit.Image.open',wraps=Image.open) as opened:
                self.assertEqual(camera_evidence(root,7,'Vehicle',cache),expected)
                self.assertEqual(camera_evidence(root,7,'Vehicle',cache),expected)
                self.assertEqual(opened.call_count,1)
            self.assertEqual(len(cache),1)
