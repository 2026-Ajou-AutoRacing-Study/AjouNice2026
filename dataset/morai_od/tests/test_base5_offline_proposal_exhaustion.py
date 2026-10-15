import json
import sys
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_base5_capture_campaign as campaign


class OfflineProposalTests(unittest.TestCase):
    def test_empty_previous_report_is_not_safety_proof(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'prior.json'
            prior=dict(stage='complete',selected=[],attempts=[],unresolved=['no support'])
            campaign.atomic_json(path,prior)
            report=dict(prior,offline_proposal_failure=dict(previous_report=str(path),
                previous_report_sha256=campaign.sha256(path),no_new_capture=True))
            with self.assertRaises(ValueError):campaign.validate_selected(report)

    def test_actual_prior_park_and_session_are_rechecked(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);child=root/'child';manifest=root/'manifest.json'
            campaign.atomic_json(manifest,{'scene':'fixture'})
            campaign.atomic_json(child/'shutdown_safety.json',{'park_verified':True})
            campaign.atomic_json(child/'session_contract.json',{'manifest_sha256':campaign.sha256(manifest)})
            prior=dict(stage='complete',selected=[],attempts=[dict(kind='initial',capture_root=str(child),
                       shutdown=dict(park_verified=True))],unresolved=['no support'])
            path=root/'prior.json';campaign.atomic_json(path,prior)
            report=dict(prior,offline_proposal_failure=dict(previous_report=str(path),
                previous_report_sha256=campaign.sha256(path),no_new_capture=True))
            with patch.object(campaign,'prepare_session') as check:
                campaign.validate_selected(report);check.assert_called_once()
                campaign.atomic_json(child/'shutdown_safety.json',{'park_verified':False})
                with self.assertRaises(ValueError):campaign.validate_selected(report)

    def test_exhaustion_leaves_deficit_and_continues_without_repeated_generation_on_resume(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'source.json'
            campaign.atomic_json(source,dict(config={},scenes=[dict(id='a',family='obstacle'),dict(id='b',family='obstacle')]))
            def child(manifest,indices,output,**kw):
                ident=json.loads(Path(manifest).read_text())['scenes'][0]['id']
                campaign.atomic_json(output/'report.json',dict(stage='complete',attempts=[],
                    selected=[] if ident=='a' else [dict(id='b',status='accepted_candidate')],
                    unresolved=['native rejected'] if ident=='a' else []))
                return 0
            with patch.object(campaign,'contract',return_value={}),patch.object(campaign,'validate_selected'), \
                 patch.object(campaign.loop,'execute',side_effect=child) as children, \
                 patch.object(campaign.loop,'replacement',side_effect=campaign.loop.PlacementProposalExhausted('512 proposals')) as proposals:
                output=root/'output'
                self.assertEqual(campaign.execute(source,[0,1],output),0)
                self.assertEqual(campaign.execute(source,[0,1],output,resume=True),0)
                self.assertEqual(children.call_count,2);self.assertEqual(proposals.call_count,1)
            self.assertEqual(json.loads((output/'quota.json').read_text())['deficit'],1)
            failure=json.loads((output/'a/round_1/report.json').read_text())
            self.assertTrue(failure['offline_proposal_failure']['no_new_capture'])

    def test_other_value_errors_still_halt(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'source.json'
            campaign.atomic_json(source,dict(config={},scenes=[dict(id='a',family='obstacle')]))
            def child(manifest,indices,output,**kw):
                campaign.atomic_json(output/'report.json',dict(stage='complete',attempts=[],selected=[],unresolved=['native rejected']))
                return 0
            with patch.object(campaign,'contract',return_value={}),patch.object(campaign,'validate_selected'), \
                 patch.object(campaign.loop,'execute',side_effect=child), \
                 patch.object(campaign.loop,'replacement',side_effect=ValueError('hash mismatch')):
                self.assertEqual(campaign.execute(source,[0],root/'output'),1)


if __name__=='__main__':unittest.main()
