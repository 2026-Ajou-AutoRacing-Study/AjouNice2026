"""Explicit candidate integration only; no GPU, ROS master, model or drive."""
import copy
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from planning_scenario_validator import gui, runner
from planning_scenario_validator.traffic_light import (
    PROFILE_LABELS, DEFAULT_MODELS_ROOT, profile_name, runtime_settings,
    candidate_assets, profile_check_arguments)


class CandidateProfileTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.example=gui.load_document(gui.EXAMPLES/'static_pair_stress_gt.yaml')

    def test_old_yaml_roundtrip_adds_no_candidate_defaults(self):
        result=gui.build_document(self.example,gui.form_values(self.example))
        self.assertEqual(result,self.example)
        self.assertNotIn('traffic_light_profile',result['stack'])
        self.assertIsNone(candidate_assets({},self.root))
        self.assertEqual(profile_check_arguments({},self.root),[])

    def test_gui_explicit_choice_is_preserved_with_custom_model_path(self):
        values=gui.form_values(self.example)
        values.update(traffic_light_profile=PROFILE_LABELS['v5_candidate'],
                      traffic_light_models_root='/my/model cache')
        result=gui.build_document(self.example,values)
        self.assertEqual(result['stack']['traffic_light_profile'],'v5_candidate')
        self.assertEqual(result['stack']['traffic_light_models_root'],'/my/model cache')
        self.assertEqual(gui.build_document(result,gui.form_values(result)),result)
        self.assertEqual(profile_name({}),'legacy')

    def test_actual_operator_save_not_just_form_builder_persists_candidate(self):
        example=copy.deepcopy(self.example)
        example['stack'].update(traffic_light_runtime='system',traffic_light_python='/usr/bin/python3')
        values=gui.form_values(example)
        self.assertEqual(gui.operator_document(example,example,values,example['scenarios'].get('include')),example)
        values.update(traffic_light_profile=PROFILE_LABELS['v5_candidate'],traffic_light_models_root='/my/models')
        saved=gui.operator_document(example,example,values,example['scenarios'].get('include'))
        self.assertEqual(saved['stack']['traffic_light_profile'],'v5_candidate')
        self.assertEqual(saved['stack']['traffic_light_models_root'],'/my/models')
        self.assertEqual(saved['stack']['traffic_light_runtime'],'virtualenv')
        self.assertEqual(saved['stack']['traffic_light_python'],gui.DEFAULT_PYTHON)
        # Concurrent VSCode edits are still protected by the same merge contract.
        changed=copy.deepcopy(example);changed['stack']['traffic_light_profile']='v5_candidate'
        changed['stack']['traffic_light_python']='/someone/else/python'
        with self.assertRaisesRegex(ValueError,'양쪽'):
            gui.operator_document(example,changed,values,example['scenarios'].get('include'))

    @unittest.skipUnless(os.environ.get('DISPLAY'),'requires Xvfb')
    def test_candidate_choice_is_editable_and_locked_during_a_run(self):
        failures=[]
        with patch.dict(os.environ,{'XDG_CONFIG_HOME':str(self.root/'gui')}):
            def ready(ui):
                def exercise():
                    try:
                        widget=ui['field_widgets']['traffic_light_profile']
                        self.assertEqual(str(widget.cget('state')),'readonly')
                        ui['fields']['traffic_light_profile'].set(PROFILE_LABELS['v5_candidate'])
                        widget.event_generate('<<ComboboxSelected>>')
                        self.assertEqual(ui['fields']['traffic_light_runtime'].get(),gui.MODE_LABELS['virtualenv'])
                        ui['actions']['set_running_ui'](True)
                        self.assertEqual(str(widget.cget('state')),'disabled')
                    except Exception as error:failures.append(error)
                    finally:ui['root'].destroy()
                ui['root'].after(30,exercise)
            gui.main(on_ready=ready)
        if failures:raise failures[0]

    def test_invalid_profile_never_falls_back_to_legacy(self):
        for value in ('production','auto',[],{},None,True):
            with self.subTest(value=value),self.assertRaises(ValueError):
                runtime_settings({'traffic_light_profile':value},self.root)

    def test_candidate_assets_absent_is_clear_before_any_drive(self):
        with self.assertRaisesRegex(ValueError,'install_traffic_light_candidate'):
            candidate_assets({'traffic_light_profile':'v5_candidate'},self.root,check_exists=True)

    def test_candidate_launch_uses_real_route_and_actual_sensor_profile(self):
        cfg={'stack':{'pose_source':'maptf_morai','detector':'od_gt','traffic_light':True,
                      'traffic_light_profile':'v5_candidate'},
             'network':{'sensor_json':'/profiles/competition_lan.json'},'document':{}}
        processes,monitor=Mock(),Mock()
        with patch.object(runner,'runtime_settings',return_value=('/my/env/bin/python','0')), patch.object(runner,'candidate_assets',return_value={'root':'/my/model cache'}):
            runner._start_case_stack(cfg,processes,monitor)
        calls={c.args[0]:c.args[1] for c in processes.start.call_args_list}
        command=calls['traffic_light']
        self.assertIn('traffic_light_v5_candidate.launch',command)
        self.assertIn('models_root:=/my/model cache',command)
        self.assertIn('sensor_config:=/profiles/competition_lan.json',command)
        self.assertFalse(any('enable_route_matching:=false' in str(v) for v in command))
        self.assertIn('enable_traffic_light_planner:=true',calls['planning'])

    def test_preflight_checks_selected_models_not_the_old_default(self):
        cfg={'output':self.root,'stack':{'traffic_light':True,'traffic_light_profile':'v5_candidate'}}
        extra=['--weights','/models/yolo.pt','--cnn-checkpoint','/models/cnn.pt','--imgsz','960']
        with patch.object(runner,'runtime_settings',return_value=('/my/env/bin/python','0')), patch.object(runner,'profile_check_arguments',return_value=extra), patch.object(runner.subprocess,'run',return_value=Mock(returncode=0,stdout='{"status":"PASS"}')) as checked:
            runner._check_traffic_light(cfg)
        self.assertEqual(checked.call_args.args[0][-len(extra):],extra)
        self.assertTrue((self.root/'stack_logs/traffic_light_preflight.json').is_file())


if __name__=='__main__':unittest.main()
