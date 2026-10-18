from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import copy
from types import SimpleNamespace
import pytest
import tempfile

sys.path.insert(0,str(Path(__file__).resolve().parent))
import reinforcement_qualification as q
import unified_capture as common
from test_unified_capture import fresh_source


class ReinforcementQualificationTests(unittest.TestCase):
    def test_distinct_schema_and_original_six_gates(self):
        self.assertNotEqual(q.QUALIFICATION_SCHEMA,q.fresh.QUALIFICATION_SCHEMA)
        self.assertEqual(q.GATES,q.original.GATES)
        self.assertEqual(q.adapter_pin()['registration_schema'],q.unit_smoke.SCHEMA)

    def test_scoped_adapter_restores_after_success_and_exception(self):
        before=(q.unit_smoke.inspect_source,q.fresh.QUALIFICATION_SCHEMA,q.fresh.POLICY,q.fresh.adapter_pin)
        with q.installed():
            self.assertIs(q.unit_smoke.inspect_source,q.inspect_source)
            self.assertEqual(q.fresh.QUALIFICATION_SCHEMA,q.QUALIFICATION_SCHEMA)
            with self.assertRaisesRegex(ValueError,'Concurrent/nested'):
                with q.installed():pass
        self.assertEqual(before,(q.unit_smoke.inspect_source,q.fresh.QUALIFICATION_SCHEMA,q.fresh.POLICY,q.fresh.adapter_pin))
        with self.assertRaisesRegex(RuntimeError,'intentional'):
            with q.installed():raise RuntimeError('intentional')
        self.assertEqual(before,(q.unit_smoke.inspect_source,q.fresh.QUALIFICATION_SCHEMA,q.fresh.POLICY,q.fresh.adapter_pin))

    def test_six_gate_functions_are_reused_inside_explicit_scope(self):
        with patch.object(q.fresh,'freeze',return_value='release') as freeze:
            self.assertEqual(q.freeze({'real':'inputs'},'new'),'release')
            freeze.assert_called_once_with({'real':'inputs'},'new')
        with patch.object(q.fresh,'qualified_release_context',return_value={'passed':True}) as qualify:
            self.assertEqual(q.qualified_release_context('release','foggy13','manifest'),{'passed':True})
            qualify.assert_called_once_with('release','foggy13','manifest')

    def test_dispatch_is_schema_only_and_rejects_unknown(self):
        for schema,name in (
            ('base5_new_environment_release_qualification_v1','finalize_base5_environment_release'),
            ('base5_fresh_environment_release_qualification_v1','fresh_environment_qualification'),
            (q.QUALIFICATION_SCHEMA,'reinforcement_qualification')):
            with patch.object(common.importlib,'import_module',return_value=name) as imported:
                self.assertEqual(common.fresh_release_verifier({'qualification_schema':schema,'environment':'foggy13'}),name)
                imported.assert_called_once_with(name)
        for schema in (None,'unknown','base5_reinforced_environment_release_qualification_v0'):
            with self.assertRaisesRegex(RuntimeError,'Unknown fresh source qualification schema'):
                common.fresh_release_verifier({'qualification_schema':schema})

    def test_missing_diagnostic_cannot_release_full_profile(self):
        with self.assertRaisesRegex(ValueError,'diagnostic receipt is required'):
            q.verify_required_diagnostic({'manifest':'unused'})

    def test_diagnostic_registration_timestamp_cannot_be_missing_nonfinite_or_future(self):
        declaration=dict(schema='base5_reinforcement_changed_row_pilot_registration_v1',
            helper=q.profile.pin(q.__file__),selected_before_capture=True,
            native_outcomes_used_for_selection=False)
        for value in (None,True,float('nan'),float('inf'),-1,q.time.time()+3600):
            changed=dict(declaration,registered_at_unix=value)
            with self.subTest(value=value),patch.object(q.profile,'read',return_value=changed):
                with self.assertRaisesRegex(ValueError,'preregistration timestamp'):
                    q.verify_pilot('/tmp/registration-does-not-exist','/tmp/native-does-not-exist')

    def diagnostic_fixture(self):
        receipt=dict(passed=True,registration=dict(path='registration'),capture_root='native',
            profile=dict(path='pilot-profile',sha256='p'*64),
            allocation=dict(path='pilot-allocation',sha256='a'*64),
            source=dict(path='source',sha256='s'*64),
            observations=[dict(id='one',scenario_sha256='x'*64)])
        full=dict(config={q.profile.CONFIG_KEY:dict(phase='full',
            profile=dict(path='full-profile',sha256='p'*64),
            allocation=dict(path='full-allocation',sha256='a'*64),source=receipt['source'])},
            scenes=[dict(id='one',scenario_sha256='x'*64)])
        return receipt,full

    def test_actual_event_jsonl_schema_has_scene_phase_time_not_ledger_id(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'event_cmd_requests.jsonl'
            path.write_text('{"phase":"park","scene":"slot1","recorded_at":123.5}\n')
            self.assertEqual(q.verify_event_registration(path,123)[0]['scene'],'slot1')
            with self.assertRaisesRegex(ValueError,'predate preregistration'):
                q.verify_event_registration(path,124)

    def test_diagnostic_tamper_and_full_geometry_or_profile_change_rejected(self):
        receipt,full=self.diagnostic_fixture()
        def read(value):return copy.deepcopy(receipt if value=='receipt' else full)
        with patch.object(q.profile,'read',side_effect=read),patch.object(q,'verify_pilot',return_value=receipt):
            self.assertEqual(q.verify_required_diagnostic(dict(reinforcement_pilot_receipt='receipt',manifest='full')),receipt)
            altered=copy.deepcopy(receipt);altered['extra']='forged'
            with patch.object(q,'verify_pilot',return_value=altered):
                with self.assertRaisesRegex(ValueError,'differs from actual'):q.verify_required_diagnostic(dict(reinforcement_pilot_receipt='receipt',manifest='full'))
            full['scenes'][0]['scenario_sha256']='changed'
            with self.assertRaisesRegex(ValueError,'Full geometry changed'):q.verify_required_diagnostic(dict(reinforcement_pilot_receipt='receipt',manifest='full'))
            full['scenes'][0]['scenario_sha256']='x'*64
            full['config'][q.profile.CONFIG_KEY]['allocation']['sha256']='changed'
            with self.assertRaisesRegex(ValueError,'Full profile/allocation/source differs'):q.verify_required_diagnostic(dict(reinforcement_pilot_receipt='receipt',manifest='full'))

    def test_all_diagnostic_raw_registration_and_receipt_bytes_enter_release_inventory(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);pins={}
            for name in ('raw.bin','registration.json','native-session.json'):
                path=root/name;path.write_text('actual '+name);pins[str(path)]=q.profile.sha256(path)
            receipt=root/'diagnostic-receipt.json';receipt.write_text('{"passed":true}')
            arguments=dict(reinforcement_pilot_receipt=str(receipt))
            result=dict(input_hashes={})
            with patch.object(q,'verify_required_diagnostic',return_value={'evidence_hashes':pins}),\
                    patch.object(q,'BASE_QUALIFY',return_value=result):
                actual=q.qualify_with_diagnostic(arguments)
            self.assertEqual(actual['input_hashes'],dict(pins,**{str(receipt):q.profile.sha256(receipt)}))
            (root/'raw.bin').write_text('tampered')
            with self.assertRaisesRegex(ValueError,'changed'):
                q.profile.inputs.verify_hashes(actual['input_hashes'])


@pytest.mark.parametrize('missing',('fixed_implementation','release_implementation','release_adapter'))
def test_common_release_requires_profile_and_adapter_pins(fresh_source,tmp_path,monkeypatch,missing):
    doc,values,_,adapter,_=fresh_source
    values=copy.deepcopy(values);doc=copy.deepcopy(doc)
    values['qualification_schema']=q.QUALIFICATION_SCHEMA
    reinforced=SimpleNamespace(__file__=adapter.__file__,QUALIFICATION_SCHEMA=q.QUALIFICATION_SCHEMA,
        POLICY=values['release_policy'])
    implementation=tmp_path/'reinforcement_profile.py';implementation.write_text('# common profile\n')
    implementation_sha=common.sha(implementation)
    monkeypatch.setitem(common.sys.modules,'reinforcement_qualification',reinforced)
    monkeypatch.setitem(common.sys.modules,'reinforcement_profile',SimpleNamespace(__file__=str(implementation)))
    doc['fixed_pins'][str(implementation)]=implementation_sha
    values['input_hashes'][str(implementation)]=implementation_sha
    common.validate_fresh_release(doc,values)
    if missing=='fixed_implementation':doc['fixed_pins'].pop(str(implementation))
    elif missing=='release_implementation':values['input_hashes'].pop(str(implementation))
    else:values['input_hashes'].pop(adapter.__file__)
    with pytest.raises(RuntimeError,match='profile/qualification implementation'):
        common.validate_fresh_release(doc,values)


if __name__=='__main__':unittest.main()
