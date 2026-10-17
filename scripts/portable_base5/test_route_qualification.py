"""CPU qualification orchestration tests; fake replacements never import ROS."""
from contextlib import contextmanager
import copy
import importlib
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import route_qualification as qualification


def write(path, value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value)+'\n')
    return path


class QualificationTests(unittest.TestCase):
    def fixture(self, folder):
        workspace = Path(folder)/'ajounice_base5_capture'
        intent = write(workspace/'checks/route_compat_intent.json',dict(test=True))
        metas = []
        for stratum in ('before','intersection','after'):
            for split in ('train','val'):
                metas.append(write(workspace/'source'/('%s_%s.json' % (stratum,split)),
                    dict(spec=dict(static_mission_extent=dict(stratum=stratum),avoidance_phase=stratum),
                        split=split,runtime_actors=[dict(data_id=1)])))
        scenario = write(workspace/'source/scenario.json',dict(original=True))
        entries = [dict(id=qualification.PROBLEM_ID if i == 76 else 'scene_%05d' % i,
            metadata=str(metas[i % len(metas)]),scenario=str(scenario),
            scenario_sha256=qualification.compat.sha256(scenario),split=('train','val')[i % 2])
            for i in range(5000)]
        manifest = write(workspace/'source/manifest.json',dict(count=5000,scenes=entries,config={}))
        document = dict(workspace=str(workspace),manifest=str(manifest),witness='test-witness',
            witness_sha256='1'*64,files={str(Path(qualification.__file__).absolute()):qualification.compat.sha256(qualification.__file__)})
        return workspace,intent,document

    def test_plan_is_readonly_and_cannot_escape_new_work_output(self):
        with tempfile.TemporaryDirectory() as folder:
            workspace,intent,document = self.fixture(folder)
            output = workspace/'work/route-compat-validation'
            with patch.object(qualification.route_dispatch,'validate_intent',return_value=document):
                plan = qualification.run(intent,output)
                self.assertEqual(plan['captures_requested'],0)
                self.assertFalse(plan['files_written'] or output.exists())
                with self.assertRaisesRegex(RuntimeError,'within'):
                    qualification.run(intent,workspace/'collections/forbidden')
                output.mkdir(parents=True)
                with self.assertRaisesRegex(RuntimeError,'Fresh'):
                    qualification.run(intent,output)

    def test_preregistered_selection_uses_actual_metadata_all_strata_and_splits(self):
        with tempfile.TemporaryDirectory() as folder:
            _,_,document = self.fixture(folder)
            deck = qualification.compat.read(document['manifest'])
            selected = qualification.select_cases(deck,SimpleNamespace())
            self.assertEqual(len(selected),7)
            self.assertEqual(selected[0]['entry']['id'],qualification.PROBLEM_ID)
            self.assertEqual({r['metadata']['spec']['static_mission_extent']['stratum'] for r in selected},
                {'before','intersection','after'})
            self.assertEqual({r['metadata']['split'] for r in selected},{'train','val'})
            self.assertEqual(selected,qualification.select_cases(deck,SimpleNamespace()))

    def test_cpu_scope_blocks_ros_network_and_child_process_then_restores(self):
        popen,sock = subprocess.Popen,socket.socket
        with qualification.cpu_only():
            for operation in (lambda:importlib.import_module('rospy'),
                    lambda:subprocess.Popen(['anything']),lambda:socket.socket()):
                with self.assertRaises(RuntimeError):operation()
        self.assertIs(subprocess.Popen,popen);self.assertIs(socket.socket,sock)

    def mock_modules(self, folder):
        active = dict(route=False,spatial=False)
        @contextmanager
        def installed(name):
            active[name] = True
            try:yield
            finally:active[name] = False
        gen = SimpleNamespace(context=lambda *a:dict(route=[],arc=[]),
            geo=SimpleNamespace(DEFAULT_RUNTIME_ASSET_DIR='/frozen-readonly',DEFAULT_MGEO='/frozen-map'))
        def replacement(source,output,**kwargs):
            self.assertTrue(active['route'] and active['spatial'])
            self.assertEqual(len(kwargs['known_payloads']),1)
            return write(Path(output)/'manifest.json',dict(count=1,scenes=[dict(test=True)]))
        quality = SimpleNamespace(gen=gen,replacement=replacement)
        campaign = SimpleNamespace(subset_deck=lambda deck,entries:dict(deck,count=1,scenes=entries))
        spatial = SimpleNamespace(gen=gen,installed=lambda:installed('spatial'))
        modules = dict(generate_base5_independent=gen,base5_static_mission_extent=SimpleNamespace(
            route_sha256=lambda ctx:qualification.compat.CANONICAL_ROUTE_SHA256),
            run_base5_quality_loop=quality,run_base5_capture_campaign=campaign,
            base5_spatial_diversity=spatial,base5_layout_split=SimpleNamespace(),base5_unit_geometry_contract=SimpleNamespace())
        return active,installed,modules

    def test_cpu_run_executes_at_most_seven_with_both_adapters_and_no_native(self):
        with tempfile.TemporaryDirectory() as folder:
            workspace,intent,document = self.fixture(folder)
            output = workspace/'work/route-compat-validation/v1'
            active,installed,modules = self.mock_modules(folder)
            with patch.object(qualification.route_dispatch,'validate_intent',return_value=document), \
                    patch.dict(sys.modules,modules), \
                    patch.object(qualification.compat,'verify_context',return_value=dict(passed=True,maximum_arc_difference_m=1e-13)), \
                    patch.object(qualification.compat,'restore_context',side_effect=lambda ctx,*a,**kw:ctx), \
                    patch.object(qualification.compat,'installed',side_effect=lambda *a,**kw:installed('route')), \
                    patch.object(qualification,'verify_replacement',side_effect=lambda deck,row,path,**kw:dict(passed=True,source_id=row['entry']['id'])):
                result = qualification.run(intent,output,True)
            receipt = qualification.compat.read(result)
            self.assertTrue(receipt['passed'] and receipt['source_inputs_unchanged'])
            self.assertEqual(receipt['replacement_count'],7)
            self.assertEqual(receipt['manifest_sha256'],qualification.compat.CANONICAL_MANIFEST_SHA256)
            self.assertEqual(receipt['captures_requested'],0)
            self.assertFalse(receipt['native_capture_executed'] or receipt['qa_changed'] or receipt['bulk_authorized'])
            self.assertEqual(active,dict(route=False,spatial=False))
            recipe = qualification.compat.read(output/'recipe.json')
            self.assertTrue(recipe['selection_before_outcomes'])
            self.assertEqual(recipe['selected'][0]['id'],qualification.PROBLEM_ID)
            self.assertTrue(all(Path(record['path']).is_file() for record in receipt['replacements']))

    def test_failed_candidate_is_reported_without_substituting_another_cell(self):
        with tempfile.TemporaryDirectory() as folder:
            workspace,intent,document = self.fixture(folder)
            output = workspace/'work/route-compat-validation/v1'
            active,installed,modules = self.mock_modules(folder)
            calls = []
            def failure(source,output,**kwargs):
                calls.append(source)
                raise RuntimeError('Actual frozen contract failed')
            modules['run_base5_quality_loop'].replacement = failure
            with patch.object(qualification.route_dispatch,'validate_intent',return_value=document), \
                    patch.dict(sys.modules,modules), \
                    patch.object(qualification.compat,'verify_context',return_value=dict(passed=True)), \
                    patch.object(qualification.compat,'restore_context',side_effect=lambda ctx,*a,**kw:ctx), \
                    patch.object(qualification.compat,'installed',side_effect=lambda *a,**kw:installed('route')):
                with self.assertRaisesRegex(RuntimeError,'Actual frozen'):
                    qualification.run(intent,output,True)
            self.assertEqual(len(calls),1)
            self.assertFalse((output/'qualification.json').exists())
            failed = qualification.compat.read(output/'qualification_failed.json')
            self.assertFalse(failed['passed'] or failed['native_capture_executed'])
            self.assertEqual(active,dict(route=False,spatial=False))


if __name__ == '__main__':unittest.main()
