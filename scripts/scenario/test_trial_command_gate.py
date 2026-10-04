import unittest
import threading
from types import SimpleNamespace
from trial_command_gate import TrialCommandGate


def command():
    return SimpleNamespace(longlCmdType=2, accel=.8, brake=0., steering=.12,
                           velocity=70., acceleration=3.)


class GateTest(unittest.TestCase):
    def setUp(self):
        self.sent = []
        self.gate = TrialCommandGate(self.sent.append, command)

    def assert_brake(self, value):
        self.assertEqual((value.longlCmdType, value.accel, value.brake,
                          value.velocity, value.acceleration), (1, 0., 1., 0., 0.))

    def test_normal_exact_passthrough_and_no_timer_duplicate(self):
        original = command()
        self.gate.receive(original)
        self.gate.tick()
        self.assertEqual(len(self.sent), 1)
        self.assertIs(self.sent[0], original)

    def test_stop_is_immediate_and_late_acceleration_cannot_override(self):
        original = command()
        self.gate.receive(original)
        self.gate.stop()
        self.assert_brake(self.sent[-1])
        self.assertEqual(original.accel, .8)
        original.steering = -.2
        self.gate.receive(original)
        self.assert_brake(self.sent[-1])
        self.assertEqual(self.sent[-1].steering, -.2)
        self.gate.tick()
        self.assert_brake(self.sent[-1])

    def test_stop_without_controller_and_repeated_stop(self):
        self.gate.stop()
        self.gate.stop()
        self.gate.tick()
        for value in self.sent:
            self.assert_brake(value)

    def test_nonfinite_stop_steering_is_not_forwarded(self):
        self.gate.receive(command())
        self.gate.stop()
        raw = command()
        raw.steering = float('nan')
        self.gate.receive(raw)
        self.assertEqual(self.sent[-1].steering, .12)
        self.assertEqual(self.gate.metrics['post_latch_accel_nonzero'], 0)

    def test_inflight_publish_cannot_overtake_latched_brake(self):
        entered, release, stop_called = threading.Event(), threading.Event(), threading.Event()
        sent = []
        def publish(value):
            if value.accel > 0:
                entered.set()
                if not release.wait(2):
                    raise RuntimeError('test publication release timeout')
            sent.append(value)
        gate = TrialCommandGate(publish, command)
        receive = threading.Thread(target=gate.receive, args=(command(),))
        receive.start()
        self.assertTrue(entered.wait(1))
        def stopping():
            stop_called.set()
            gate.stop()
        stop = threading.Thread(target=stopping)
        stop.start()
        self.assertTrue(stop_called.wait(1))
        release.set()
        receive.join(2)
        stop.join(2)
        self.assertFalse(receive.is_alive() or stop.is_alive())
        self.assertEqual(len(sent), 2)
        self.assertEqual(sent[0].accel, .8)
        self.assert_brake(sent[1])
        gate.receive(command())
        self.assert_brake(sent[-1])

    def test_tick_continues_during_blocked_shutdown(self):
        entered, finish = threading.Event(), threading.Event()
        def shutdown():
            entered.set()
            finish.wait(2)
        self.gate.stop()
        waiter = threading.Thread(target=shutdown)
        waiter.start()
        self.assertTrue(entered.wait(1))
        for _ in range(5):
            self.gate.tick()
        finish.set()
        waiter.join(2)
        self.assertEqual(len(self.sent), 6)
        for value in self.sent:
            self.assert_brake(value)


if __name__ == '__main__':
    unittest.main()
