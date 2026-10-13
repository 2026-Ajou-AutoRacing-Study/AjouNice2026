import sys
import unittest
import tempfile
import json
import subprocess
from unittest.mock import patch
import numpy as np
from PIL import Image
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from base5_capture_timing import StageTiming, preflight_stationary, stationary_witness, post_load_topics_ready, deferred_static_secondary, bounded_stationary_recheck
from base5_capture_audit import camera_evidence


class TimingTest(unittest.TestCase):
    def test_bounded_recheck_only_delays_failed_static_witness(self):
        ticks=[0.]
        def sleep(dt):ticks[0]+=dt
        for static in (False,True):
            with patch('time.sleep') as unused:
                good,info=bounded_stationary_recheck(lambda:dict(verified=True),lambda:None,static,
                    clock=lambda:ticks[0],sleep=sleep)
                self.assertTrue(good['verified']);self.assertEqual(info['checks'],1)
                self.assertEqual(info['elapsed_sec'],0.)
        reads=iter([dict(verified=False),dict(verified=True)])
        good,info=bounded_stationary_recheck(lambda:next(reads),lambda:None,True,clock=lambda:ticks[0],sleep=sleep)
        self.assertTrue(good['verified']);self.assertEqual(info['checks'],2)
        bad,info=bounded_stationary_recheck(lambda:dict(verified=False),lambda:None,True,
            clock=lambda:ticks[0],sleep=sleep,timeout=.10)
        self.assertFalse(bad['verified']);self.assertAlmostEqual(info['elapsed_sec'],.10)
        bad,info=bounded_stationary_recheck(lambda:dict(verified=False),lambda:None,False,
            clock=lambda:ticks[0],sleep=sleep)
        self.assertEqual(info['checks'],1);self.assertEqual(info['elapsed_sec'],0.)
        with self.assertRaisesRegex(RuntimeError,'stale'):
            bounded_stationary_recheck(lambda:dict(verified=False),lambda:(_ for _ in ()).throw(RuntimeError('stale')),True)

    def test_network_reuse_bulk_refused_before_output_or_ros(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);manifest=root/'manifest.json'
            manifest.write_text(json.dumps(dict(scenes=[dict(id=str(i)) for i in range(17)])))
            result=subprocess.run([sys.executable,str(Path(__file__).resolve().parents[1]/'scripts/run_base5_capture.py'),
                '--manifest',str(manifest),'--output',str(root/'out'),'--limit','17',
                '--batch-reuse-network','--prepare-only'],capture_output=True,text=True)
            self.assertEqual(result.returncode,2)
            self.assertIn('Network reuse disabled',result.stderr)
            self.assertFalse((root/'out').exists())
            result=subprocess.run([sys.executable,str(Path(__file__).resolve().parents[1]/'scripts/run_base5_capture.py'),
                '--manifest',str(manifest),'--output',str(root/'out'),'--limit','17',
                '--shared-ros-batch','--prepare-only'],capture_output=True,text=True)
            self.assertEqual(result.returncode,2)
            self.assertIn('limited to 16',result.stderr)
            self.assertFalse((root/'out').exists())

    def test_batch_network_first_load_and_post_load_witness(self):
        actors=[dict(unique_id=1,source_list='vehicleList'),dict(unique_id=2,source_list='aiPedstrianList')]
        state=dict(ego_t=1.7,object_t=1.7,collision_t=1.7,objects=dict(npc_list=[dict(unique_id=1)]))
        self.assertTrue(post_load_topics_ready(state,actors,1.,1.8))
        self.assertFalse(post_load_topics_ready(dict(state,collision_t=1.3),actors,1.,1.8))
        self.assertFalse(post_load_topics_ready(state,actors,1.,3.))
        self.assertFalse(post_load_topics_ready(dict(state,objects={}),actors,1.,1.8))

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

    def test_missing_secondary_is_only_deferred_with_fresh_transport_and_static_actor(self):
        actors=[dict(unique_id=1,source_list='vehicleList',motion='static'),
                dict(unique_id=2,source_list='pedestrianList',motion='standing')]
        state=dict(ego_t=1.7,object_t=1.7,collision_t=1.7,objects=dict(npc_list=[dict(unique_id=1)]))
        self.assertEqual(deferred_static_secondary(state,actors,1.,1.8),[2])
        self.assertEqual(deferred_static_secondary(dict(state,collision_t=0),actors,1.,1.8),[])
        self.assertEqual(deferred_static_secondary(dict(state,objects={}),actors,1.,1.8),[])
        actors[1]['motion']='walking'
        self.assertEqual(deferred_static_secondary(state,actors,1.,1.8),[])

    def test_stationary_witness_rejects_stale_short_moving_or_unstable(self):
        rows=[dict(t=t,x=0.,y=0.,z=1.,vx=0.,vy=0.) for t in (.61,.71,.81,.91,1.)]
        self.assertTrue(preflight_stationary(rows,1.))
        self.assertFalse(preflight_stationary(rows,1.2))
        self.assertFalse(preflight_stationary(rows[-2:],1.))
        for key,value in [('vx',.1),('x',.1),('z',.1),('vy',float('nan'))]:
            changed=[dict(r) for r in rows];changed[-1][key]=value
            self.assertFalse(preflight_stationary(changed,1.),key)

    def test_missing_static_primary_requires_explicit_diagnostic_opt_in(self):
        actors=[dict(unique_id=1,source_list='pedestrianList',motion='static')]
        state=dict(ego_t=1.7,object_t=1.7,collision_t=1.7,objects={})
        self.assertEqual(deferred_static_secondary(state,actors,1.,1.8),[])
        self.assertEqual(deferred_static_secondary(state,actors,1.,1.8,allow_missing_primary=True),[1])
        self.assertEqual(deferred_static_secondary(dict(state,collision_t=0),actors,1.,1.8,allow_missing_primary=True),[])
        actors[0]['motion']='walking'
        self.assertEqual(deferred_static_secondary(state,actors,1.,1.8,allow_missing_primary=True),[])

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

    def test_post_park_witness_excludes_pre_park_samples_and_checks_vz(self):
        rows=[dict(t=t,x=0.,y=0.,z=1.,vx=0.,vy=0.,vz=0.) for t in (.61,.71,.81,.91,1.)]
        self.assertTrue(stationary_witness(rows,1.,.60)['verified'])
        self.assertFalse(stationary_witness(rows,1.,.81)['verified'])
        for key,value in [('vx',.021),('vz',.021),('z',1.021),('x',.021),('vz',float('nan'))]:
            changed=[dict(r) for r in rows];changed[-1][key]=value
            self.assertFalse(stationary_witness(changed,1.,.60)['verified'],key)
        del rows[-1]['vz']
        self.assertFalse(stationary_witness(rows,1.,.60)['verified'])
