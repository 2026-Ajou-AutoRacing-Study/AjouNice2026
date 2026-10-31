#!/usr/bin/env python3
"""Deterministic checks of operator-visible formatting and transition timing."""
import importlib.util
import math
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location(
    'control_terminal', Path(__file__).resolve().parents[1] / 'scripts/control_terminal.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class TerminalTest(unittest.TestCase):
    def setUp(self):
        self.lines = []
        self.terminal = MODULE.Terminal(lambda line, level: self.lines.append((line, level)))

    def feed(self, now=0., vx=23.45, ay=6.55, steer=12.3, state='OK', reason=''):
        self.terminal.sample('vx', vx, now)
        self.terminal.sample('ay', ay, now)
        self.terminal.state(state, reason, math.radians(steer), now)
        self.terminal.tick(now, '00.0')

    def test_fixed_columns_and_signed_steering(self):
        self.feed()
        self.feed(1., vx=134.55, ay=.25, steer=2.1)
        self.feed(2., vx=0., ay=-6.55, steer=-40.)
        self.assertEqual([line for line, _ in self.lines], [
            '00.0 CTRL  23.45kph  6.55mps2 +12.3°',
            '00.0 CTRL 134.55kph  0.25mps2  +2.1°',
            '00.0 CTRL   0.00kph -6.55mps2 -40.0°'])
        for unit in ('kph', 'mps2', '°'):
            self.assertEqual(len({line.index(unit) for line, _ in self.lines}), 1)

    def test_normal_one_second(self):
        for index in range(101):
            self.feed(index / 100.)
        self.assertEqual(len(self.lines), 2)
        self.assertTrue(all(level == 'OK' for _, level in self.lines))

    def test_saturation_immediate_then_point_two_seconds(self):
        self.feed()
        self.feed(.03, steer=40.)
        self.assertEqual(len(self.lines), 2)
        self.assertEqual(self.lines[-1][1], 'WARN')
        self.assertNotIn('WARN', self.lines[-1][0])
        self.feed(.22, steer=40.)
        self.assertEqual(len(self.lines), 2)
        self.feed(.23, steer=-40.)
        self.assertEqual(len(self.lines), 3)

    def test_first_recovery_does_not_wait_for_timer(self):
        self.feed(0., steer=40.)
        self.feed(.01, steer=39.)
        self.assertEqual([level for _, level in self.lines], ['WARN', 'OK'])
        self.feed(.99)
        self.assertEqual(len(self.lines), 2)
        self.feed(1.01)
        self.assertEqual(len(self.lines), 3)

    def test_error_recovery_immediate(self):
        self.feed()
        self.feed(.01, state='ERROR', reason='state rx stale 0.51s')
        self.feed(.02, state='ERROR', reason='state rx stale 0.52s')
        self.assertEqual(len(self.lines), 2)  # changing age must not flood the display
        self.feed(.03)
        self.assertEqual([level for _, level in self.lines], ['OK', 'ERROR', 'OK'])

    def test_missing_measurement_is_not_zero_or_cached(self):
        self.feed()
        self.terminal.state('OK', '', 0., .51)
        self.terminal.tick(.51, '00.5')
        self.assertEqual(self.lines[-1], ('00.5 CTRL     --kph    --mps2  +0.0°', 'WARN'))
        self.feed(.52)
        self.assertEqual(self.lines[-1][1], 'OK')

    def test_controller_timeout_and_wait(self):
        self.terminal.tick(0., '00.0')
        self.assertEqual(self.lines[-1], ('00.0 CTRL WAIT controller', 'WARN'))
        self.feed(.01)
        self.terminal.tick(.52, '00.5')
        self.assertEqual(self.lines[-1], ('00.5 CTRL ERROR controller timeout', 'ERROR'))

    def test_nonfinite_values_never_appear_as_normal(self):
        self.feed(vx=math.nan, ay=math.inf)
        self.assertEqual(self.lines[-1][1], 'WARN')
        self.assertNotIn('nan', self.lines[-1][0])
        self.feed(.01, steer=math.nan)
        self.assertEqual(self.lines[-1][1], 'ERROR')


if __name__ == '__main__':
    unittest.main()
