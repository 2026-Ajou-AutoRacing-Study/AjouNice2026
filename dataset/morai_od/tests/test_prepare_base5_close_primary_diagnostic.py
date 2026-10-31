import copy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_close_primary_observation as close
import prepare_base5_close_primary_diagnostic as prepare


class DiagnosticTests(unittest.TestCase):
    def test_unit_payload_never_rescales_an_existing_object(self):
        scenario=dict(objectList=[dict(scale=dict(x=1.,y=1.,z=1.))])
        snapshot=copy.deepcopy(scenario)
        prepare.unit_payload(scenario);self.assertEqual(scenario,snapshot)
        for scale in ({},dict(x=1.,y=1.25,z=1.),dict(x=1.,y=1.,z=1.,_x='1.250'),
                      dict(x=True,y=1.,z=1.)):
            with self.assertRaises(close.ClosePrimaryContractError):
                prepare.unit_payload(dict(objectList=[dict(scale=scale)]))

    def test_roster_includes_order_uid_dataid_name_container_motion_and_scale(self):
        actor=dict(unique_id=1,data_id=2,name='physical',source_list='objectList',motion='static')
        base=dict(runtime_actors=[actor])
        for key,value in (('unique_id',3),('data_id',4),('name','other'),('source_list','vehicleList'),
                          ('motion','walking'),('asset_scale',dict(scale_xyz=[1.25]*3))):
            changed=copy.deepcopy(base);changed['runtime_actors'][0][key]=value
            self.assertNotEqual(prepare.roster(base),prepare.roster(changed))

    def test_subset_manifest_has_no_false_completion_or_release_claim(self):
        deck=dict(config=dict(original=True),scenes=[dict(id='a'),dict(id='b')],count=5000,bulk_ready=True)
        before=copy.deepcopy(deck)
        result=prepare._entry_deck(deck,deck['scenes'][0],dict(explicit=True))
        self.assertEqual(deck,before)
        self.assertEqual(result['count'],1)
        self.assertFalse(result['bulk_ready']);self.assertFalse(result['completion_live_qualified'])

    def test_two_distinct_control_sources_required_before_runtime_access(self):
        with self.assertRaises(ValueError):
            prepare.prepare('/does-not-exist','/tmp/nonexistent-close-primary-output',['same','same'])


if __name__=='__main__':unittest.main()
