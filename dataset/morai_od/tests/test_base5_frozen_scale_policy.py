import copy
import gc
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_asset_scale as scale


class FrozenPolicyTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.evidence = Path(self.temp.name)/'native.json'
        self.evidence.write_text('{"fixture":true}\n')
        self.policy = scale.candidate_policy([2, 0, 1])
        self.profile = self.policy['assets']['CargoBox']['profiles']['iso125']
        self.profile.update(status='qualified', native_geometry=dict(bbox_size_m=[1., 2., .5],
            pivot_to_bbox_center_local_m=[.01, .02, .25], pivot_height_above_ground_m=0.),
            native_qualification=dict(observations=2, semantic_visual_review='accepted', contact_review='passed',
                sensor_gt_review='passed', axis_mapping_verified=True, single_component_obstacle_verified=True,
                evidence=[dict(path=str(self.evidence), sha256=scale.hashlib.sha256(self.evidence.read_bytes()).hexdigest())]))
        self.asset = dict(name='CargoBox', data_id=40100001, source_list='objectList', semantic_class='obstacle')
        self.row = dict(seed=10)

    def snapshot(self):
        return scale.freeze_policy(self.policy)

    def test_digest_and_selection_equal_mutable_policy_without_reencoding_or_pin_hot_calls(self):
        expected = scale.select(self.asset, self.row, 0, 'static', self.policy, profile_id='iso125')
        snapshot = self.snapshot()
        original_digest = scale.digest(self.policy)
        with patch.object(scale.json, 'dumps', side_effect=AssertionError('cached digest must not serialize')):
            for _ in range(100):
                self.assertEqual(scale.digest(snapshot), original_digest)
        with patch.object(scale, '_pins', side_effect=AssertionError('hot validation must not stat evidence')):
            for _ in range(20):
                self.assertIs(scale.validate_policy(snapshot), snapshot)
                actual = scale.select(self.asset, self.row, 0, 'static', snapshot, profile_id='iso125')
                self.assertEqual(actual, expected)
                scale.validate_assignment(actual, snapshot)

    def test_mapping_list_mutators_and_direct_base_methods_cannot_bypass_freeze(self):
        snapshot = self.snapshot()
        mapping = snapshot['assets']['CargoBox']
        values = mapping['bbox_axis_to_scale_axis']
        for operation in (lambda:snapshot.update({}), lambda:mapping.clear(), lambda:mapping.pop('group'),
                lambda:mapping.setdefault('x', 1), lambda:mapping.__setitem__('data_id', 0),
                lambda:dict.__setitem__(snapshot, 'assets', {}),
                lambda:values.append(9), lambda:values.__setitem__(0, 9),
                lambda:list.__setitem__(values, 0, 9), lambda:values.__iadd__([9])):
            with self.subTest(operation=operation), self.assertRaises(TypeError):
                operation()
        self.assertEqual(scale.thaw_policy(snapshot), self.policy)

    def test_snapshot_attributes_and_mappingproxy_are_read_only_even_through_object_setattr(self):
        snapshot = self.snapshot()
        for key, value in (('_data', {}), ('_policy_sha256', 'forged'), ('_evidence', ())):
            with self.assertRaises((AttributeError, TypeError)):
                object.__setattr__(snapshot, key, value)
        with self.assertRaises(TypeError):
            snapshot._data['assets'] = {}
        with self.assertRaises(TypeError):
            snapshot._evidence_stamps['forged'] = ()

    def test_constructor_and_uninitialized_forgery_are_rejected(self):
        with self.assertRaises(scale.ScaleContractError):
            scale.FrozenPolicy(self.policy)
        forged = object.__new__(scale.FrozenPolicy)
        for operation in (lambda:scale.validate_policy(forged), lambda:scale.digest(forged),
                          lambda:scale.verify_frozen_policy(forged)):
            with self.assertRaises(scale.ScaleContractError):
                operation()

    def test_deepcopy_self_plain_config_json_roundtrip_and_sequence_equality(self):
        snapshot = self.snapshot()
        self.assertIs(copy.deepcopy(snapshot), snapshot)
        self.assertIs(copy.deepcopy(snapshot['assets']), snapshot['assets'])
        values = snapshot['assets']['CargoBox']['profiles']['iso125']['native_geometry']['bbox_size_m']
        self.assertEqual(values, [1., 2., .5])
        self.assertEqual([1., 2., .5], values)
        self.assertFalse(values != [1., 2., .5])
        self.assertFalse([1., 2., .5] != values)
        plain = scale.plain_config({scale.CONFIG_KEY:snapshot})
        self.assertEqual(json.loads(json.dumps(plain))[scale.CONFIG_KEY], self.policy)
        self.assertEqual(scale.digest(snapshot['assets']), scale.digest(self.policy['assets']))

    def test_source_and_thawed_mutation_do_not_change_snapshot(self):
        snapshot = self.snapshot()
        digest = scale.digest(snapshot)
        self.policy['assets']['CargoBox']['variation_fraction'] = .1
        plain = scale.thaw_policy(snapshot)
        plain['assets']['CargoBox']['profiles']['iso125']['scale_xyz'][0] = 9.
        self.assertEqual(scale.digest(snapshot), digest)
        self.assertNotEqual(scale.digest(self.policy), digest)
        self.assertNotEqual(scale.digest(plain), digest)
        self.assertEqual(snapshot['assets']['CargoBox']['profiles']['iso125']['scale_xyz'], [1.25]*3)

    def test_bound_config_leaves_input_untouched_and_reuses_snapshot_explicitly(self):
        config = {scale.CONFIG_KEY:self.policy, 'capture':{'static':True}}
        bound = scale.bind_config(config)
        self.assertIsNot(config, bound)
        self.assertIs(config[scale.CONFIG_KEY], self.policy)
        self.assertIsInstance(bound[scale.CONFIG_KEY], scale.FrozenPolicy)
        self.assertIs(scale.bind_config(bound)[scale.CONFIG_KEY], bound[scale.CONFIG_KEY])
        self.assertEqual(scale.plain_config(bound), config)

    def test_mutable_policies_still_validate_every_call_and_cannot_reuse_old_assignments(self):
        with patch.object(scale, '_pins', wraps=scale._pins) as calls:
            scale.validate_policy(self.policy)
            first = calls.call_count
            scale.validate_policy(self.policy)
            self.assertGreater(calls.call_count, first)
        assignment = scale.select(self.asset, self.row, 0, 'static', self.policy, profile_id='iso125')
        self.policy['assets']['CargoBox']['variation_fraction'] = .2
        with self.assertRaises(scale.ScaleContractError):
            scale.validate_assignment(assignment, self.policy)

    def test_changed_evidence_fails_at_explicit_end_boundary(self):
        snapshot = self.snapshot()
        self.evidence.write_text('changed\n')
        # The immutable in-memory policy is unchanged; the boundary refuses to
        # approve the operation after evidence mutation, without actor hot IO.
        self.assertIs(scale.validate_policy(snapshot), snapshot)
        with self.assertRaises(scale.ScaleContractError):
            scale.verify_frozen_policy(snapshot)
        with self.assertRaises(scale.ScaleContractError):
            scale.freeze_policy(snapshot)

    def test_same_bytes_with_restored_mtime_still_detects_ctime_or_inode_drift(self):
        snapshot = self.snapshot()
        original, stat = self.evidence.read_bytes(), self.evidence.stat()
        # Allocate the replacement while the original inode still exists.
        # In-place writes can share one coarse filesystem ctime tick, even
        # when the bytes and mtime are restored, so they are not a reliable
        # way to exercise stamp drift without sleeps.
        replacement = self.evidence.with_name('replacement.json')
        replacement.write_bytes(original)
        self.assertNotEqual(replacement.stat().st_ino, stat.st_ino)
        os.utime(replacement, ns=(stat.st_atime_ns, stat.st_mtime_ns))
        replacement.replace(self.evidence)
        self.assertEqual(self.evidence.read_bytes(), original)
        self.assertEqual(self.evidence.stat().st_mtime_ns, stat.st_mtime_ns)
        with self.assertRaisesRegex(scale.ScaleContractError, 'stamp'):
            scale.verify_frozen_policy(snapshot)

    def test_each_evidence_stamp_field_is_checked_independently_of_clock_resolution(self):
        snapshot = self.snapshot()
        stamp = scale._file_stamp(self.evidence)
        self.assertIs(scale.verify_frozen_policy(snapshot), snapshot)
        for index, field in enumerate(('device', 'inode', 'size', 'mtime_ns', 'ctime_ns')):
            changed = tuple(value + (offset == index) for offset, value in enumerate(stamp))
            # Real content/hash validation remains active; only the stat
            # observation is controlled, including a ctime-only change.
            with self.subTest(field=field), patch.object(scale, '_file_stamp', return_value=changed):
                with self.assertRaisesRegex(scale.ScaleContractError, 'stamp'):
                    scale.verify_frozen_policy(snapshot)

    def test_frozen_nodes_are_weakly_owned_and_do_not_leak_each_bound_policy(self):
        gc.collect()
        before = len(scale._FROZEN_NODES)
        snapshot = self.snapshot()
        self.assertGreater(len(scale._FROZEN_NODES), before)
        del snapshot
        gc.collect()
        self.assertEqual(len(scale._FROZEN_NODES), before)

    def test_unvalidated_profile_like_extra_mapping_cannot_use_qualified_fastpath(self):
        self.policy['note'] = {'status':'qualified'}
        snapshot = self.snapshot()
        with self.assertRaises(scale.ScaleContractError):
            scale._qualified(snapshot['note'])


if __name__ == '__main__':
    unittest.main()
