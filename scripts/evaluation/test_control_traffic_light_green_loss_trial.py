"""Pure green-loss sequence tests; no ROS, native API, UDP or controls start."""
import math
import ast
from pathlib import Path
import signal
import unittest

import control_traffic_light_green_loss_trial as trial

GROUP = dict(group_id=201351, native_ids=['A_head'], stop_line_xy=[[10., -2.], [10., 2.]])


def inputs(now=10., speed=0., x=6., gear=4, forward=False):
    rows = {name: dict(receipt_monotonic=now, source_stamp_ns=None) for name in ('odom', 'gear', 'ctrl')}
    rows['odom'].update(source_stamp_ns=int(now*1e9), position_xyz=[x, 0., 0.],
        quaternion_xyzw=[0., 0., 0., 1.], velocity_xyz=[speed, 0., 0.], frame='map')
    rows['gear']['value'] = gear
    rows['ctrl']['control_message'] = dict(longlCmdType=1, accel=.2 if forward else 0.,
                                          brake=0. if forward else 1.)
    return rows


class FakeTrial:
    def __init__(self, departure_speed=.2, departure_x=6., native_color=None, group=GROUP):
        self.group = group
        native_color = trial.controlled_go_state(group['group_id']) if native_color is None else native_color
        self.now, self.controlled_go = 10., False
        self.departure_speed, self.departure_x, self.native_color = departure_speed, departure_x, native_color
        self.events, self.actions = [], []
        self.signal, self.terminal_calls = None, 0

    def snapshot(self):
        return inputs(self.now, speed=self.departure_speed if self.controlled_go else 0.,
                      x=self.departure_x if self.controlled_go else 6., forward=self.controlled_go)

    def wait(self, duration): self.now += duration
    def emit(self, event, **fields): self.events.append(dict(event=event, **fields))
    def fault(self, mode, stage, immediate): self.actions.append(('fault', mode, stage, immediate, self.now))
    def red(self, stage):
        self.actions.append(('red', stage, self.now))
        return True
    def actual(self, stage):
        self.actions.append(('go_set_get', stage, self.now))
        self.controlled_go = self.native_color == trial.controlled_go_state(self.group['group_id'])
        if stage == 'terminal_controlled_go':
            self.terminal_calls += 1
            if self.terminal_calls >= 2:
                self.signal = signal.SIGTERM
        return {native_id:self.native_color for native_id in self.group['native_ids']}

    def execute(self, seconds=30.):
        return trial.execute_sequence(self.group, self.snapshot, lambda: self.now, lambda: int(self.now*1e9),
            self.wait, lambda: self.signal is not None, self.now+seconds, self.fault, self.red, self.actual,
            self.emit, termination_signal=lambda: self.signal)


class GreenLossTrialTest(unittest.TestCase):
    def test_native_permission_is_route_specific_not_yellow_or_forced_green(self):
        self.assertTrue(trial.native_route_permission(201351, {'A':16}, ['A']))
        self.assertFalse(trial.native_route_permission(201351, {'A':4}, ['A']))
        self.assertFalse(trial.native_route_permission(201351, {'A':33}, ['A']))
        self.assertTrue(trial.native_route_permission(201365, {'B':16}, ['B']))
        self.assertTrue(trial.native_route_permission(201290, {'C1':48,'C2':48}, ['C1','C2']))
        self.assertFalse(trial.native_route_permission(201290, {'C1':48,'C2':1}, ['C1','C2']))
        with self.assertRaises(trial.TrialError):
            trial.native_route_permission(201290, {'C1':48}, ['C1','C2'])
        for color in (32, 33):
            self.assertTrue(trial.native_route_permission(201326, {'D':color}, ['D']))
        for color in (1, 4, 5, 16, 48):
            self.assertFalse(trial.native_route_permission(201326, {'D':color}, ['D']))
        for bad in ({'wrong':16}, {'A':0}, {'A':True}):
            with self.assertRaises(trial.TrialError):
                trial.native_route_permission(201351, bad, ['A'])

    def test_departure_needs_real_forward_control_fresh_D4_and_narrow_speed_window(self):
        self.assertTrue(trial.departure_gate(inputs(speed=.2, forward=True), GROUP, 10., 10**10)['ready'])
        for speed in (.15, .4):
            self.assertTrue(trial.departure_gate(inputs(speed=speed, forward=True), GROUP, 10., 10**10)['ready'])
        for speed in (0., .149):
            self.assertFalse(trial.departure_gate(inputs(speed=speed, forward=True), GROUP, 10., 10**10)['ready'])
        self.assertFalse(trial.departure_gate(inputs(speed=.2, forward=False), GROUP, 10., 10**10)['ready'])
        self.assertTrue(trial.departure_gate(inputs(speed=.401, forward=True), GROUP, 10., 10**10)['not_applicable'])
        self.assertTrue(trial.departure_gate(inputs(speed=.2, x=8., forward=True), GROUP, 10., 10**10)['not_applicable'])
        with self.assertRaises(trial.TrialError):
            trial.departure_gate(inputs(gear=1), GROUP, 10., 10**10)
        with self.assertRaises(trial.TrialError):
            trial.departure_gate(inputs(), GROUP, 11., 11*10**9)

    def test_front_distance_must_exceed_point_four_and_no_GT_heading_adjustment(self):
        allowed = trial.departure_gate(inputs(speed=.2, x=6.59, forward=True), GROUP, 10., 10**10)
        self.assertTrue(allowed['ready'])
        self.assertAlmostEqual(allowed['front_distance_m'], .41)
        rejected = trial.departure_gate(inputs(speed=.2, x=6.61, forward=True), GROUP, 10., 10**10)
        self.assertTrue(rejected['not_applicable'])
        self.assertEqual(allowed['geometry']['matches'][0]['front_point_map_xy_m'][1], 0.)

    def test_sequence_arms_red_conditional_go_then_holds_positive_terminal_until_owner_sigterm(self):
        fake = FakeTrial()
        fake.execute()
        self.assertTrue(any(row['event']=='experiment_start_gate_confirmed' for row in fake.events))
        first_go = next(i for i, row in enumerate(fake.actions) if row[0]=='go_set_get')
        first_drop = next(i for i, row in enumerate(fake.actions) if row[0]=='fault' and row[1]=='drop')
        first_postdrop_red = next(i for i, row in enumerate(fake.actions)
                                 if row[:2]==('red', 'red_drop_after_controlled_departure'))
        self.assertLess(first_go, first_drop)
        self.assertLess(first_drop, first_postdrop_red)
        self.assertTrue(fake.actions[first_drop][3])  # synchronous request, not just future heartbeat
        self.assertEqual(fake.actions[-1][:2], ('go_set_get', 'terminal_controlled_go'))
        self.assertEqual(fake.terminal_calls, 2)
        self.assertFalse(any(row[0]=='release' for row in fake.actions))
        completed = [row for row in fake.events if row['event']=='stage_complete']
        timed = [row for row in completed if 'seconds' in row['stage']]
        self.assertEqual([row['stage']['seconds'] for row in timed], [3., 2.])
        self.assertTrue(all(row['confirmed_elapsed_sec']>=row['stage']['seconds'] for row in timed))
        self.assertEqual(completed[-1]['owner_teardown_signal'], signal.SIGTERM)
        final = fake.events[-1]
        self.assertFalse(final['official_violation_certified'])
        self.assertFalse(final['natural_mode_restoration_certified'])
        self.assertTrue(final['restore_required'])

    def test_late_or_crossed_departure_is_not_applicable_without_drop(self):
        for speed, x in ((.5, 6.), (.2, 8.)):
            fake = FakeTrial(departure_speed=speed, departure_x=x)
            with self.assertRaises(trial.TrialNotApplicable):
                fake.execute()
            self.assertFalse(any(row[0]=='fault' and row[1]=='drop' for row in fake.actions))

    def test_controlled_go_wait_is_globally_bounded_and_transport_errors_propagate(self):
        fake = FakeTrial(departure_speed=0., native_color=1)
        with self.assertRaises(trial.TrialNotApplicable):
            fake.execute(seconds=2.)
        self.assertFalse(any(row[0]=='fault' and row[1]=='drop' for row in fake.actions))
        fake = FakeTrial()
        def failed_read(stage): raise trial.TrialError('native error')
        fake.actual = failed_read
        with self.assertRaisesRegex(trial.TrialError, 'native error'):
            fake.execute()
        self.assertFalse(any(row[0]=='fault' and row[1]=='drop' for row in fake.actions))

    def test_control_modes_and_fault_payload_never_contain_native_color_or_GO(self):
        for kind, field in ((1,'accel'), (2,'velocity'), (3,'acceleration')):
            self.assertTrue(trial.forward_control(dict(longlCmdType=kind, brake=0., **{field:.2})))
            self.assertFalse(trial.forward_control(dict(longlCmdType=kind, brake=1., **{field:.2})))
        with self.assertRaises((ValueError, trial.TrialError)):
            trial.forward_control(dict(longlCmdType=1, brake=0., accel=math.nan))
        payload = trial.fault_payload('drop', 201326, trial.EXPERIMENT, 'red_drop_after_controlled_departure')
        self.assertEqual(set(payload), {'mode','group_id','experiment','stage'})

    def test_controlled_go_requires_exact_requested_positive_color_and_reacquires_mismatch(self):
        self.assertEqual(trial.controlled_go_state(201351), 16)
        self.assertEqual(trial.controlled_go_state(201365), 16)
        self.assertEqual(trial.controlled_go_state(201290), 48)
        self.assertEqual(trial.controlled_go_state(201326), 33)
        self.assertFalse(trial.controlled_go_matches(201326, {'D':32}, ['D']))
        self.assertFalse(trial.controlled_go_matches(201351, {'A':1}, ['A']))
        self.assertTrue(trial.controlled_go_matches(201326, {'D':33}, ['D']))
        fake = FakeTrial()
        calls = [0]
        def transient_go(stage):
            calls[0] += 1
            fake.native_color = 1 if calls[0] == 1 else 16
            return FakeTrial.actual(fake, stage)
        fake.actual = transient_go
        fake.execute()
        observations = [row for row in fake.events if row['event']=='controlled_departure_observation']
        self.assertEqual([row['exact_controlled_go_confirmed'] for row in observations], [False, True])
        with self.assertRaises(trial.TrialError):
            trial.controlled_go_matches(201351, {'A':-1}, ['A'])

    def test_positive_only_sender_blocks_negative_before_wire_and_finally_has_no_native_writes(self):
        sent = []
        send = lambda native_id, state: sent.append((native_id, state))
        for state in (1, 4, 5, 16, 33, 48):
            trial.send_positive_signal(send, 'A_head', state)
        self.assertEqual([state for _, state in sent], [1, 4, 5, 16, 33, 48])
        for state in (-1, 0, True, None, 32):
            with self.assertRaises((ValueError, trial.TrialError)):
                trial.send_positive_signal(send, 'A_head', state)
        self.assertEqual(len(sent), 6)
        tree = ast.parse(Path(trial.__file__).read_text())
        run = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name=='run')
        final_blocks = [node.finalbody for node in ast.walk(run) if isinstance(node, ast.Try) and node.finalbody]
        self.assertTrue(final_blocks)
        prohibited = {'sender','positive_request','send_positive_signal','apply_red','apply_go','release'}
        for body in final_blocks:
            for statement in body:
                calls = [node for node in ast.walk(statement) if isinstance(node, ast.Call)]
                self.assertFalse(any(isinstance(node.func, ast.Name) and node.func.id in prohibited for node in calls))

    def test_abort_or_sigterm_before_terminal_confirmation_stays_error_and_adds_no_final_override(self):
        fake = FakeTrial()
        fake.signal = signal.SIGINT
        with self.assertRaises(trial.TrialError):
            fake.execute()
        self.assertFalse(any(row[0] in ('red','go_set_get','release') for row in fake.actions))
        fake = FakeTrial()
        original = fake.actual
        def premature_sigterm(stage):
            result = original(stage)
            if stage == 'terminal_controlled_go': fake.signal = signal.SIGTERM
            return result
        fake.actual = premature_sigterm
        with self.assertRaises(trial.TrialError):
            fake.execute()
        self.assertEqual(fake.terminal_calls, 1)
        self.assertFalse(any(row['event']=='experiment_complete' for row in fake.events))
        self.assertFalse(any(row[0]=='release' for row in fake.actions))

    def test_terminal_color_change_after_confirmation_is_error_not_natural_restoration(self):
        fake = FakeTrial()
        original = fake.actual
        def changed_terminal(stage):
            if stage == 'terminal_controlled_go' and fake.terminal_calls >= 1:
                fake.native_color = 1
            result = original(stage)
            fake.signal = None  # Do not mask the mismatching GET with owner teardown.
            return result
        fake.actual = changed_terminal
        with self.assertRaisesRegex(trial.TrialError, 'changed after confirmation'):
            fake.execute()
        self.assertTrue(any(row['event']=='terminal_controlled_go_confirmed' for row in fake.events))
        self.assertFalse(any(row['event']=='experiment_complete' for row in fake.events))
        self.assertFalse(any(row[0]=='release' for row in fake.actions))

    def test_B_and_two_head_C_complete_with_exact_route_positive_colors(self):
        for group_id, native_ids in ((201365,['B_head']), (201290,['C_head1','C_head2'])):
            group = dict(GROUP, group_id=group_id, native_ids=native_ids)
            fake = FakeTrial(group=group)
            fake.execute()
            observed = next(row for row in fake.events if row['event']=='controlled_departure_observation')
            self.assertEqual(set(observed['native_states']), set(native_ids))
            self.assertTrue(all(value==trial.controlled_go_state(group_id)
                                for value in observed['native_states'].values()))
            self.assertTrue(observed['exact_controlled_go_confirmed'])
            self.assertTrue(fake.events[-1]['restore_required'])
        for group_id in (0,201397,999):
            with self.assertRaises(ValueError): trial.controlled_go_state(group_id)


if __name__ == '__main__': unittest.main()
