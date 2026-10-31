import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parent))
import sunny15_backup_qualification as qualification
import sunny15_vast_training as launcher


class ReusedBackupQualificationTests(unittest.TestCase):
    def fixture(self, root):
        uploader=root/'backup_base5_checkpoints.py';parts=root/'base5_checkpoint_parts.py'
        uploader.write_bytes((qualification.ROOT/qualification.HELPERS[0]).read_bytes())
        parts.write_bytes((qualification.ROOT/qualification.HELPERS[1]).read_bytes())
        helpers={name:qualification.sha(qualification.ROOT/name) for name in qualification.HELPERS}
        c=dict(host='root@host',port=19463,drive_remote='ajou_drive_private',drive_root='root',
               destination='backup:new-destination',backup_script=str(uploader))
        record=dict(sha256=qualification.E24_SHA,bytes=524552773,resume_capable=True,
                    contract_sha256=qualification.E24_CONTRACT,readback_location='local_credential_owner')
        log=root/'old.jsonl';manifest=root/'manifest.json'
        log.write_text(json.dumps(dict(event='local_readback_verified',verified=True,
                       sha256=qualification.E24_SHA,bytes=524552773))+'\n'+json.dumps(dict(valid=True,
                       status=dict(uploaded={'epoch_24.pth':record},errors=[],destination='backup:old'),
                       pending_checkpoints=[]))+'\n')
        manifest.write_text(json.dumps(dict(contract_sha256=qualification.E24_CONTRACT,
                                            identity=dict(source_hashes=helpers))))
        proof=qualification.prior_full_evidence(log,manifest)
        receipt=dict(qualification_kind=qualification.KIND,new_checkpoint_uploaded=False,
                     first_new_epoch_backup_requires_actual_verification=True,
                     transport_identity=qualification.identity(c),prior_full_evidence=proof,
                     local_helpers=helpers,remote_helpers={str(uploader):helpers[qualification.HELPERS[0]],
                                                          str(parts):helpers[qualification.HELPERS[1]]})
        return c,receipt

    def test_explicit_reused_qualification_accepts_same_remote_mechanics(self):
        with tempfile.TemporaryDirectory() as temp:
            c,r=self.fixture(Path(temp));qualification.validate(r,c)
            self.assertFalse(r['new_checkpoint_uploaded'])

    def test_small_probe_alone_is_not_reused_full_qualification(self):
        with tempfile.TemporaryDirectory() as temp:
            c,_=self.fixture(Path(temp))
            with self.assertRaisesRegex(ValueError,'explicit reused'):
                qualification.validate(dict(valid=True,write_readback=True),c)

    def test_old_helper_hash_mismatch_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            c,r=self.fixture(Path(temp));r['prior_full_evidence']['frozen_helpers'][qualification.HELPERS[0]]='changed'
            with self.assertRaisesRegex(ValueError,'helper hashes differ'):qualification.validate(r,c)

    def test_actual_remote_helper_drift_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            c,r=self.fixture(Path(temp));Path(c['backup_script']).write_text('changed')
            with self.assertRaisesRegex(ValueError,'Remote helper changed'):qualification.validate(r,c)

    def test_changed_endpoint_or_destination_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            c,r=self.fixture(Path(temp));c['destination']='backup:elsewhere'
            with self.assertRaisesRegex(ValueError,'identity differs'):qualification.validate(r,c)

    def test_reuse_cannot_claim_fresh_upload(self):
        with tempfile.TemporaryDirectory() as temp:
            c,r=self.fixture(Path(temp));r['new_checkpoint_uploaded']=True
            with self.assertRaisesRegex(ValueError,'new-payload claim'):qualification.validate(r,c)

    def test_prior_log_requires_actual_full_readback_not_just_valid_marker(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);self.fixture(root);log=root/'old.jsonl'
            log.write_text(log.read_text().splitlines()[-1]+'\n')
            with self.assertRaisesRegex(ValueError,'full E24 upload/readback evidence missing'):
                qualification.prior_full_evidence(log,root/'manifest.json')

    def test_remote_gate_distinguishes_reused_receipt_from_full_new_report(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);c,r=self.fixture(root);path=root/'reused.json'
            path.write_text(json.dumps(r));c.update(backup_qualification_report=str(root/'new_missing.json'),
                backup_reuse_receipt_remote=str(path),backup_reuse_receipt_sha256=qualification.sha(path))
            self.assertEqual(launcher.backup_gate(c)['qualification_kind'],qualification.KIND)

    def test_local_evidence_mutation_is_rejected_before_start(self):
        with tempfile.TemporaryDirectory() as temp, patch.object(qualification,'credential_profile',return_value={'test':'profile'}):
            root=Path(temp);c,r=self.fixture(root)
            probe=root/'probe.json';probe.write_text(json.dumps(dict(write_readback=True,training_started=False)))
            config=root/'probe_config.json';config.write_text(json.dumps(c))
            r['credential_profile']={'test':'profile'}
            paths=dict(prior_log=root/'old.jsonl',prior_manifest=root/'manifest.json',
                       current_probe=probe,current_probe_config=config)
            r['local_evidence']={name:dict(path=str(path),sha256=qualification.sha(path)) for name,path in paths.items()}
            qualification.validate(r,c,local=True)
            probe.write_text('{}')
            with self.assertRaisesRegex(ValueError,'evidence changed'):qualification.validate(r,c,local=True)


if __name__=='__main__':unittest.main()
