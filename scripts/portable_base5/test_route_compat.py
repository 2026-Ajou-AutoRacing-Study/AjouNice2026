"""CPU-only canonical arc compatibility; no ROS, simulator or remote access."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import route_compat as compat


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True)+'\n')
    return path


class RouteCompatibilityTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.route = [dict(x=float(i)*.5,y=0.,z=1.,link_idx='lane') for i in range(4)]
        self.arc = [0.,.5,1.,1.5]
        self.route_sha = self.extent_digest(dict(route=self.route,arc=self.arc))
        manifest = dict(count=5000,scenes=[None]*5000,
            config=dict(static_mission_extent=dict(route_sha256=self.route_sha)))
        self.manifest = write(self.root/'manifest.json',manifest)
        self.map = self.root/'map';write(self.map/'link_set.json',[])
        self.guide = write(self.root/'guide.txt',[])
        geo = SimpleNamespace(__file__=str(write(self.root/'geometry.py',{})),
            DEFAULT_MGEO=self.map,DEFAULT_ROUTE=self.guide,
            load_route=lambda path:([],[]),load_json=lambda path:[],
            build_mgeo_guided_competition_route=lambda guide,links:(copy.deepcopy(self.route),{}),
            route_arc_coordinates=lambda route:list(self.arc))
        self.gen = SimpleNamespace(__file__=str(write(self.root/'generator.py',{})),geo=geo,
            context=lambda runtime,mgeo:dict(route=copy.deepcopy(self.route),arc=list(self.arc),
                unrelated=dict(qa='unchanged')))
        self.extent = SimpleNamespace(__file__=str(write(self.root/'extent.py',{})),
            route_sha256=self.extent_digest)
        for name,value in [('CANONICAL_POINTS',4),('CANONICAL_ROUTE_SHA256',self.route_sha),
                ('CANONICAL_MANIFEST_SHA256',compat.sha256(self.manifest))]:
            guard = patch.object(compat,name,value);guard.start();self.addCleanup(guard.stop)
        self.witness = self.root/'witness.json'
        with patch.object(compat.sys,'version_info',(3,8,10)):
            compat.export_witness(self.manifest,self.witness,generator=self.gen,extent=self.extent)

    @staticmethod
    def extent_digest(ctx):
        return compat.digest([[p[k] for k in compat.AXES]+[float(a)]
            for p,a in zip(ctx['route'],ctx['arc'])])

    def options(self, **extra):
        return dict(generator=self.gen,extent=self.extent,manifest=self.manifest,
            expected_witness_sha256=compat.sha256(self.witness),**extra)

    def context(self):
        return self.gen.context(None,self.map)

    def test_export_is_exact_new_evidence_not_native_or_bulk_authority(self):
        doc = compat.read(self.witness)
        self.assertEqual(doc['canonical_arc_m'],self.arc)
        self.assertEqual(doc['route_points'],self.route)
        self.assertFalse(doc['native_capture_executed'])
        self.assertFalse(doc['bulk_authorized'])
        self.assertEqual(set(doc['source_files']),{'manifest','generator','geometry','extent','map_link','guide'})
        before = self.witness.read_bytes()
        with self.assertRaisesRegex(compat.RouteCompatibilityError,'Fresh'):
            compat.export_witness(self.manifest,self.witness,generator=self.gen,extent=self.extent)
        self.assertEqual(before,self.witness.read_bytes())

    def test_non38_export_is_forbidden_before_output(self):
        other = self.root/'forbidden.json'
        with patch.object(compat.sys,'version_info',(3,11,1)):
            with self.assertRaisesRegex(compat.RouteCompatibilityError,'Python 3.8'):
                compat.export_witness(self.manifest,other,generator=self.gen,extent=self.extent)
        self.assertFalse(other.exists())

    def test_readonly_verifier_accepts_tiny_arc_difference_without_mutation(self):
        ctx = self.context();ctx['arc'][1] += 1e-13
        before = copy.deepcopy(ctx)
        report = compat.verify_context(ctx,self.witness,**self.options())
        self.assertEqual(ctx,before)
        self.assertTrue(report['passed'] and report['coordinates_exact'])
        self.assertEqual(report['changed_arc_entries'],1)
        self.assertLess(report['maximum_arc_difference_m'],1e-9)
        self.assertFalse(report['context_modified'] or report['bulk_authorized'])

    def test_restore_changes_only_arc_and_keeps_original_route_qa_identity(self):
        ctx = self.context();ctx['arc'][1] += 1e-13
        route,other = ctx['route'],ctx['unrelated']
        result = compat.restore_context(ctx,self.witness,**self.options())
        self.assertIs(result,ctx);self.assertIs(ctx['route'],route)
        self.assertIs(ctx['unrelated'],other)
        self.assertEqual(ctx['arc'],self.arc)
        self.assertEqual(self.extent.route_sha256(ctx),self.route_sha)

    def test_restore_is_idempotent_and_failure_keeps_context_unmodified(self):
        ctx = self.context()
        compat.restore_context(ctx,self.witness,**self.options())
        compat.restore_context(ctx,self.witness,**self.options())
        ctx['route'][1]['x'] += .001;before = copy.deepcopy(ctx)
        with self.assertRaises(compat.RouteCompatibilityError):
            compat.restore_context(ctx,self.witness,**self.options())
        self.assertEqual(ctx,before)

    def test_only_exact_coordinate_order_and_link_identity_are_allowed(self):
        for change in ('coordinate','link','order','count'):
            ctx = self.context()
            if change == 'coordinate':ctx['route'][1]['x'] += 1e-13
            if change == 'link':ctx['route'][1]['link_idx'] = 'other'
            if change == 'order':ctx['route'][1:3] = ctx['route'][1:3][::-1]
            if change == 'count':ctx['route'].pop()
            with self.subTest(change=change),self.assertRaises(compat.RouteCompatibilityError):
                compat.verify_context(ctx,self.witness,**self.options())

    def test_arc_bound_finiteness_monotonicity_and_shape_are_not_waived(self):
        for value in ([0.,.5+2e-9,1.,1.5],[0.,float('nan'),1.,1.5],
                [0.,float('inf'),1.,1.5],[0.,.5,.5,1.5],[0.,.5,1.], [True,.5,1.,1.5]):
            ctx = self.context();ctx['arc'] = value
            with self.subTest(value=value),self.assertRaises(compat.RouteCompatibilityError):
                compat.verify_context(ctx,self.witness,**self.options())

    def test_explicit_witness_sha_and_current_helper_source_are_required(self):
        with self.assertRaisesRegex(compat.RouteCompatibilityError,'pinned'):
            compat.verify_context(self.context(),self.witness,**dict(self.options(),expected_witness_sha256=None))
        doc = compat.read(self.witness);doc['helper_sha256'] = '0'*64;write(self.witness,doc)
        with self.assertRaisesRegex(compat.RouteCompatibilityError,'profile'):
            compat.verify_context(self.context(),self.witness,**self.options())

    def test_all_source_map_guide_generator_pins_are_enforced(self):
        files = compat.read(self.witness)['source_files']
        for name,row in files.items():
            path = Path(row['logical_path']);before = path.read_bytes()
            try:
                path.write_bytes(before+b' ')
                with self.subTest(source=name),self.assertRaises(compat.RouteCompatibilityError):
                    compat.verify_context(self.context(),self.witness,**self.options())
            finally:path.write_bytes(before)

    def test_witness_cannot_self_declare_new_arc_or_coordinate_contract(self):
        original = self.witness.read_bytes()
        for change in ('coordinates','arc','tolerance','producer','bulk','capture'):
            doc = json.loads(original)
            if change == 'coordinates':doc['route_points'][1]['x'] += .1
            if change == 'arc':doc['canonical_arc_m'][1] += 1e-13
            if change == 'tolerance':doc['maximum_arc_difference_m'] = .01
            if change == 'producer':doc['producer_python_major_minor'] = [3,11]
            if change == 'bulk':doc['bulk_authorized'] = True
            if change == 'capture':doc['native_capture_executed'] = True
            write(self.witness,doc)
            with self.subTest(change=change),self.assertRaises(compat.RouteCompatibilityError):
                compat.verify_context(self.context(),self.witness,**self.options())

    def test_scoped_install_restores_generator_even_on_failure_without_math_patch(self):
        original = self.gen.context;hypot = math.hypot
        def floating(runtime,mgeo):
            ctx = original(runtime,mgeo);ctx['arc'][1] += 1e-13;return ctx
        self.gen.context = floating
        with self.assertRaisesRegex(ValueError,'test failure'):
            with compat.installed(self.witness,**self.options()):
                ctx = self.context()
                self.assertEqual(ctx['arc'],self.arc)
                self.assertIs(math.hypot,hypot)
                raise ValueError('test failure')
        self.assertIs(self.gen.context,floating);self.assertIs(math.hypot,hypot)

    def test_context_alternate_mgeo_must_match_original_bytes(self):
        other = self.root/'other-map';write(other/'link_set.json',dict(changed=True))
        original = self.gen.context
        with compat.installed(self.witness,**self.options()):
            with self.assertRaisesRegex(compat.RouteCompatibilityError,'bytes changed'):
                self.gen.context(None,other)
        self.assertIs(self.gen.context,original)

    def test_partial_or_symlink_witness_is_not_repaired(self):
        alias = self.root/'alias.json';alias.symlink_to(self.witness)
        with self.assertRaisesRegex(compat.RouteCompatibilityError,'symlink'):
            compat.verify_context(self.context(),alias,**self.options())
        self.witness.write_text('{')
        with self.assertRaisesRegex(compat.RouteCompatibilityError,'malformed'):
            compat.verify_context(self.context(),self.witness,**self.options())


class RealCrossVersionTests(unittest.TestCase):
    def test_frozen_4392_route_38_witness_restores_actual_311_arc(self):
        repo = Path(__file__).resolve().parents[2]
        manifest = repo/'artifacts/evaluation/base5_unit_release_261017/close_primary_fullsource_v1/candidate_v158a/manifest.json'
        python38 = Path('/usr/bin/python3.8')
        python311 = Path('/home/luv/miniconda3/envs/fastbev_rtx50/bin/python')
        if not all(p.is_file() for p in (manifest,python38,python311)):
            self.skipTest('Local frozen source and independent Python 3.8/3.11 interpreters required')
        module = Path(compat.__file__).resolve();scripts = repo/'dataset/morai_od/scripts'
        env = dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1')
        with tempfile.TemporaryDirectory() as temp:
            witness = Path(temp)/'canonical.json'
            exported = subprocess.run([str(python38),'-B',str(module),'export','--manifest',str(manifest),
                '--witness',str(witness),'--scripts',str(scripts)],env=env,capture_output=True,text=True,timeout=30)
            self.assertEqual(exported.returncode,0,exported.stderr)
            code = '''import sys,json
from pathlib import Path
sys.path.insert(0,sys.argv[1]);sys.path.insert(0,sys.argv[2])
import route_compat as r
import generate_base5_independent as g
import base5_static_mission_extent as e
guide,_=g.geo.load_route(g.geo.DEFAULT_ROUTE)
route,_=g.geo.build_mgeo_guided_competition_route(guide,g.geo.load_json(g.geo.DEFAULT_MGEO/'link_set.json'))
ctx=dict(route=route,arc=g.geo.route_arc_coordinates(route),unrelated=dict(qa='unchanged'))
options=dict(generator=g,extent=e,manifest=Path(sys.argv[3]),expected_witness_sha256=r.sha256(sys.argv[4]))
report=r.verify_context(ctx,Path(sys.argv[4]),**options)
before=ctx['route'];other=ctx['unrelated'];r.restore_context(ctx,Path(sys.argv[4]),**options)
assert e.route_sha256(ctx)==r.CANONICAL_ROUTE_SHA256 and ctx['route'] is before and ctx['unrelated'] is other
factory=g.context
with r.installed(Path(sys.argv[4]),**options):
    actual=g.context(g.geo.DEFAULT_RUNTIME_ASSET_DIR,g.geo.DEFAULT_MGEO)
    assert e.route_sha256(actual)==r.CANONICAL_ROUTE_SHA256
assert g.context is factory
report['actual_generator_factory_restored']=True
print(json.dumps(report))
'''
            verified = subprocess.run([str(python311),'-B','-c',code,str(module.parent),str(scripts),
                str(manifest),str(witness)],env=env,capture_output=True,text=True,timeout=30)
            self.assertEqual(verified.returncode,0,verified.stderr)
            report = json.loads(verified.stdout)
            self.assertEqual(report['route_points'],4392)
            self.assertTrue(report['passed'] and report['coordinates_exact'])
            self.assertTrue(report['actual_generator_factory_restored'])
            self.assertGreater(report['changed_arc_entries'],0)
            self.assertLess(report['maximum_arc_difference_m'],1e-9)
            self.assertEqual(report['canonical_route_sha256'],compat.CANONICAL_ROUTE_SHA256)
            self.assertFalse(report['native_capture_executed'] or report['bulk_authorized'])


if __name__ == '__main__':
    unittest.main()
