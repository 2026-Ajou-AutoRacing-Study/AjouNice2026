import copy
import unittest

from control_traffic_light_short_cycles import (GROUP_IDS, NATURAL_CYCLES, MismatchGuard,
                                               TrialError, input_gate, scheduled_phase, validate_plan,
                                               validate_max_seconds, paced_native_cycle,
                                               persistent_request, persistent_result, PersistentNativeSender,
                                               PERSISTENT_SET_METHOD)
from probe_morai_traffic_light_gt import _fields
from control_traffic_light_policy_trial import coordinated_native_cycle


def plan():
    return dict(schema_version=1, name='short', groups=[dict(
        group_id=group_id, native_ids=list(GROUP_IDS[group_id]), phase_offset_sec=0,
        phases=[dict(color=color, seconds=2 if color in (4, 5) else 10) for color in colors])
        for group_id, colors in NATURAL_CYCLES.items()])


class ShortCyclesTest(unittest.TestCase):
    def test_persistent_request_pins_physical_head_and_false_modes(self):
        for group_id, ids in GROUP_IDS.items():
            for native_id in ids:
                for color in NATURAL_CYCLES[group_id]:
                    fields = list(_fields(persistent_request(native_id, color)))
                    self.assertEqual(fields[1:], [(2, 0, 0), (3, 0, 0)])
                    info = list(_fields(fields[0][2]))
                    self.assertEqual(info[0][0:2], (2, 2))
                    self.assertEqual(list(_fields(info[0][2])), [(1, 2, native_id.encode())])
                    self.assertEqual(info[1], (3, 0, color))
        for native_id, color in [('C1256W000053', 16), ('C1256W000077', 33),
                                  ('C1256W000052', -1), ('C1256W000052', True)]:
            with self.assertRaises(ValueError): persistent_request(native_id, color)

    def test_persistent_result_requires_explicit_success(self):
        self.assertEqual(persistent_result(b'\x08\x01')['status'], 1)
        for response in (b'', b'\x08\x00', b'\x08\x02', b'\x0a\x01x', b'\x12\x02\xff\xff'):
            with self.subTest(response=response), self.assertRaises((ValueError, TrialError)):
                persistent_result(response)

    def test_sender_only_uses_inspected_rpc_and_rejects_before_transport(self):
        calls = []
        sender = PersistentNativeSender(rpc_call=lambda path, data, timeout:
            calls.append((path, data, timeout)) or b'\x08\x01')
        result = sender('C1256W000052', 33)
        self.assertEqual(calls[0][0], PERSISTENT_SET_METHOD)
        self.assertEqual(calls[0][1], persistent_request('C1256W000052', 33))
        self.assertFalse(result['is_impulse'])
        self.assertFalse(result['set_sibling'])
        self.assertTrue(result['rpc_set_success'])
        with self.assertRaises(ValueError): sender('C1256W000053', 33)
        self.assertEqual(len(calls), 1)
        sender.close()

    def test_owner_preparation_plus_case_duration_matches_wrapper_bound(self):
        self.assertEqual(validate_max_seconds(600), 600)
        self.assertEqual(validate_max_seconds(600 + 300), 900)
        self.assertEqual(validate_max_seconds(1800), 1800)
        for value in (0, -1, 1800.01, True, float('nan'), float('inf'), '900'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_max_seconds(value)

    def test_exact_all_groups_and_colors(self):
        source = plan()
        self.assertEqual(validate_plan(source), source)
        self.assertIsNot(validate_plan(source), source)
        for mutate in ('missing', 'duplicate', 'D1', 'C_single', 'fake_color', 'fake_order'):
            source = plan()
            if mutate == 'missing': source['groups'].pop()
            if mutate == 'duplicate': source['groups'][1] = copy.deepcopy(source['groups'][0])
            if mutate == 'D1': source['groups'][3]['native_ids'] = ['C1256W000053']
            if mutate == 'C_single': source['groups'][2]['native_ids'].pop()
            if mutate == 'fake_color': source['groups'][2]['phases'][0]['color'] = 16
            if mutate == 'fake_order': source['groups'][3]['phases'].reverse()
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                validate_plan(source)

    def test_phase_durations_finite_and_bounded(self):
        for value in (True, -1, 0, 31, float('nan'), '2'):
            source = plan()
            source['groups'][0]['phases'][0]['seconds'] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_plan(source)
        source = plan()
        source['groups'][0]['phases'][1]['seconds'] = 6
        with self.assertRaises(ValueError): validate_plan(source)

    def test_offsets_and_rotation_supported_without_inventing_combination(self):
        source = plan()
        source['groups'][3]['phases'] = source['groups'][3]['phases'][2:] + source['groups'][3]['phases'][:2]
        source['groups'][0]['phase_offset_sec'] = 7
        validate_plan(source)
        for bad in (-1, True, 121, float('inf')):
            source['groups'][0]['phase_offset_sec'] = bad
            with self.assertRaises(ValueError): validate_plan(source)

    def test_schedule_uses_single_anchor_and_wraps(self):
        group = plan()['groups'][0]
        self.assertEqual(scheduled_phase(group, 0)['color'], 16)
        self.assertEqual(scheduled_phase(group, 9.99)['color'], 16)
        self.assertEqual(scheduled_phase(group, 10)['color'], 4)
        self.assertEqual(scheduled_phase(group, 12)['color'], 1)
        self.assertEqual(scheduled_phase(group, 22)['cycle_index'], 1)
        self.assertEqual(scheduled_phase(group, 22)['color'], 16)
        self.assertEqual(scheduled_phase(group, 32)['color'], 4)
        group['phase_offset_sec'] = 11
        self.assertEqual(scheduled_phase(group, 0)['color'], 4)
        self.assertEqual(scheduled_phase(group, 0)['phase_remaining_sec'], 1)
        self.assertEqual(scheduled_phase(group, 1)['color'], 1)
        with self.assertRaises(ValueError): scheduled_phase(group, -1)

    def test_persistent_mismatch_fatal_and_phase_change_resets_only_that_budget(self):
        guard = MismatchGuard()
        guard.update(1, (0, 0), False)
        guard.update(2.9, (0, 0), False)
        with self.assertRaises(TrialError): guard.update(3, (0, 0), False)
        guard = MismatchGuard()
        guard.update(1, (0, 0), False)
        guard.update(2, (0, 0), True)
        guard.update(3, (0, 0), False)
        guard.update(4.9, (0, 1), False)
        guard.update(6.8, (0, 1), False)
        with self.assertRaises(TrialError): guard.update(6.9, (0, 1), False)

    def test_c_sends_each_head_with_frame_wait_then_reads_separately(self):
        events = []
        ids = GROUP_IDS[201290]
        result = coordinated_native_cycle(ids, 48,
            lambda native_id, color: events.append(('send', native_id, color)),
            lambda native_id: events.append(('read', native_id)) or 48,
            lambda seconds: events.append(('wait', seconds)))
        self.assertEqual(result, {ids[0]: 48, ids[1]: 48})
        self.assertEqual(events, [('send', ids[0], 48), ('wait', .1),
                                  ('send', ids[1], 48), ('wait', .1),
                                  ('read', ids[0]), ('read', ids[1])])

    def test_all_groups_wait_after_each_physical_set_without_double_wait_for_c(self):
        events = []
        for group_id, colors in NATURAL_CYCLES.items():
            color = colors[0]
            paced_native_cycle(GROUP_IDS[group_id], color,
                lambda native_id, state: events.append(('send', native_id, state)),
                lambda native_id: events.append(('read', native_id)) or color,
                lambda seconds: events.append(('wait', seconds)))
        expected = []
        for group_id, colors in NATURAL_CYCLES.items():
            for native_id in GROUP_IDS[group_id]:
                expected.extend([('send', native_id, colors[0]), ('wait', .1)])
            expected.extend(('read', native_id) for native_id in GROUP_IDS[group_id])
        self.assertEqual(events, expected)
        self.assertEqual(sum(event[0] == 'send' for event in events), 5)
        self.assertEqual(sum(event[0] == 'wait' for event in events), 5)

    def test_fresh_d4_gate_allows_moving_or_stationary_without_signal(self):
        latest = {name: dict(receipt_monotonic=10, source_stamp_ns=10_000_000_000)
                  for name in ('odom', 'gear', 'ctrl')}
        latest['odom'].update(position_xyz=[0, 0, 0], velocity_xyz=[10, 0, 0],
                              quaternion_xyzw=[0, 0, 0, 1], frame='map')
        latest['gear']['value'] = 4
        self.assertTrue(input_gate(latest, 10.2, 10_200_000_000)[0])
        latest['odom']['velocity_xyz'] = [0, 0, 0]
        self.assertTrue(input_gate(latest, 10.2, 10_200_000_000)[0])
        latest['gear']['value'] = 1
        self.assertFalse(input_gate(latest, 10.2, 10_200_000_000)[0])
        latest['gear']['value'] = 4
        self.assertFalse(input_gate(latest, 11.1, 11_100_000_000)[0])


if __name__ == '__main__': unittest.main()
