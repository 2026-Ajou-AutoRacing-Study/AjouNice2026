import ast
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_spatial_diversity as spatial
import report_base5_postcollection as report
import run_base5_capture_optimized as optimized
import run_base5_batch_optimized as batch
import run_base5_capture_overlap as overlap
from argparse import Namespace


class SpatialTests(unittest.TestCase):
    def test_seed_repeatable_and_environment_specific(self):
        self.assertEqual(spatial.seed_for('foggy11','slot',1),spatial.seed_for('foggy11','slot',1))
        self.assertNotEqual(spatial.seed_for('foggy11','slot',1),spatial.seed_for('foggy13','slot',1))
    def test_excluded_gaps_are_not_sampled(self):
        self.assertEqual(spatial.slice_intervals([[0,10],[100,110]],5,15),[[5,10],[100,105]])
    def test_invalid_stratum(self):
        for a,b in [(-1,1),(5,4),(0,21)]:
            with self.assertRaises(ValueError):spatial.slice_intervals([[0,20]],a,b)
    def test_partition_quota_and_original_rows_not_reordered(self):
        items=[(str(i),dict(mission=False,family='obstacle',topology='route_general',role='front',distance_bounds=[10,30]),'train') for i in range(8)]
        spatial.assign_plans(items,{('other','train'):[[0,10],[100,110]]})
        self.assertEqual([v[0] for v in items],[str(i) for i in range(8)])
        plans=[s['spatial_sampling_plan'] for _,s,_ in items]
        self.assertEqual({p['stratum_index'] for p in plans},set(range(8)))
        self.assertAlmostEqual(sum(b-a for p in plans for a,b in p['quota_intervals_m']),20)
        self.assertTrue(all(len(p['local_retry_windows_m'])==3 for p in plans))
    def test_mission_and_traffic_paths_not_perturbed(self):
        for spec in [dict(mission=True,family='obstacle',topology='route_general'),
                     dict(mission=False,family='highway',topology='route_general'),
                     dict(mission=False,family='pedestrian_related',topology='route_general',shadow_phase='inside')]:
            self.assertFalse(spatial.eligible(spec))
    def test_adapter_restored_on_error(self):
        original=spatial.gen.build
        try:
            with spatial.installed():
                self.assertIs(spatial.gen.build,spatial.build)
                raise RuntimeError('injected')
        except RuntimeError:pass
        self.assertIs(spatial.gen.build,original)


class RuntimeTests(unittest.TestCase):
    def test_overlap_adapter_compiles_without_touching_source(self):
        before=optimized.SOURCE.read_bytes()
        self.assertIsNotNone(overlap.compiled_source())
        self.assertEqual(optimized.SOURCE.read_bytes(),before)
    def test_service_probe_is_readonly_and_failed_probe_rejected(self):
        m={'runtime_actors':[]};calls=[]
        def fail(name,timeout):
            calls.append((name,timeout));raise RuntimeError('service absent')
        probe=overlap.start_probe(m,fail)
        with self.assertRaises(RuntimeError):overlap.finish_probe(probe,m,fail)
        self.assertEqual(calls,[('/Service_MoraiEventCmd',4.)])
    def test_readonly_probe_success_records_evidence(self):
        m={'runtime_actors':[]};probe=overlap.start_probe(m,lambda *a,**kw:None)
        overlap.finish_probe(probe,m,lambda *a,**kw:None)
        self.assertTrue(m['service_preparation_overlap']['ok'])
    def test_animated_service_preparation_stays_legacy(self):
        m={'runtime_actors':[{'motion':'walking'}]}
        with patch.object(overlap,'lightweight_static',return_value=False):
            self.assertIsNone(overlap.start_probe(m,lambda *a,**kw:None))
    def test_frozen_source_compiles_profiles(self):
        for profile in ('baseline','static-ready'):self.assertIsNotNone(optimized.compiled_source(profile))
        with self.assertRaises(ValueError):optimized.compiled_source('unknown')
    def test_refuse_changed_source_structure(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'x.py';path.write_text('runner_options = {}\n')
            with patch.object(optimized,'SOURCE',path):
                with self.assertRaises(ValueError):optimized.compiled_source('baseline')
    def test_stale_transport_never_skips_delay(self):
        m={'runtime_actors':[]};state=dict(ego_t=1.,object_t=1.,collision_t=1.,objects={})
        with patch('time.monotonic',return_value=5.):self.assertFalse(optimized.static_fast_ready(m,state,0))
    def test_fresh_background_can_skip_delay(self):
        m={'runtime_actors':[]};state=dict(ego_t=5.,object_t=5.,collision_t=5.,objects={})
        with patch('time.monotonic',return_value=5.):self.assertTrue(optimized.static_fast_ready(m,state,4))
    def test_manifest_change_blocks_resume_before_native_execute(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);manifest=root/'m.json';manifest.write_text(json.dumps({'scenes':[{}]}))
            output=root/'output';output.mkdir();(output/'optimized_contract.json').write_text('{}')
            a=Namespace(manifest=manifest,output=output,resume=True,limit=1,chunk_size=4,setup_profile='baseline')
            with patch.object(batch.batch.campaign,'contract',return_value={}),patch.object(batch.batch,'execute') as execute:
                with self.assertRaises(ValueError):batch.run(a)
                execute.assert_not_called()
    def test_legacy_output_cannot_be_adapted(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);manifest=root/'m.json';manifest.write_text(json.dumps({'scenes':[{}]}))
            output=root/'output';output.mkdir();(output/'batch_contract.json').write_text('{}')
            a=Namespace(manifest=manifest,output=output,resume=True,limit=1,chunk_size=4,setup_profile='baseline')
            with patch.object(batch.batch.campaign,'contract',return_value={}):
                with self.assertRaises(ValueError):batch.run(a)


class ReportingTests(unittest.TestCase):
    def test_hardlink_and_overlapping_roots_count_once(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);p=root/'a'/'timing.json';p.parent.mkdir()
            p.write_text(json.dumps(dict(stages_sec={'scenario_install':2},total_sec=2,
                intervals=[dict(stage='scenario_install',seconds=2)])))
            q=root/'b'/'timing.json';q.parent.mkdir();os.link(p,q)
            result=report.timing_report([root,root/'a'])
            self.assertEqual(result['unique_sessions'],1);self.assertEqual(result['measured_native_sec'],2)
            self.assertEqual(result['timed_scenario_loads'],1)
    def test_invalid_stage_partition_reported(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'timing.json';p.write_text(json.dumps(dict(stages_sec={'setup':2},total_sec=3,intervals=[])))
            self.assertTrue(report.timing_report([Path(d)])['errors'])


if __name__=='__main__':unittest.main()
