#!/usr/bin/env python3
"""Exercise the installed display over ROS and a color-capable terminal."""
import math
import os
import pty
import select
import subprocess
import threading
import time
import unittest

import rospy
import rostest
from morai_msgs.msg import EgoVehicleStatus
from sensor_msgs.msg import Imu
from trajectory_follower.msg import TerminalState


class TransportTest(unittest.TestCase):
    def test_sources_colors_cadence_and_immediate_recovery(self):
        vx = rospy.Publisher('/Competition_topic', EgoVehicleStatus, queue_size=1)
        imu = rospy.Publisher('/sensor/imu', Imu, queue_size=1)
        status = rospy.Publisher('/control/terminal_state', TerminalState, queue_size=1)
        ego = rospy.Publisher('/Ego_topic', EgoVehicleStatus, queue_size=1)
        master, slave = pty.openpty()
        env = dict(os.environ, TERM='xterm')
        env.pop('NO_COLOR', None)
        executable = os.environ.get('CTRL_TERMINAL_EXECUTABLE')
        command = [executable] if executable else ['rosrun', 'trajectory_follower', 'control_terminal.py']
        child = subprocess.Popen(command,
                                 stdout=slave, stderr=slave, env=env)
        os.close(slave)
        lock = threading.Lock()
        mode = ['OK', '', math.radians(12.3)]
        stop = threading.Event()

        def publish():
            while not stop.wait(.02):
                competition = EgoVehicleStatus()
                competition.velocity.x = 23.45 / 3.6
                vx.publish(competition)
                wrong = EgoVehicleStatus()
                wrong.velocity.x = 999.
                ego.publish(wrong)
                measurement = Imu()
                measurement.linear_acceleration.y = -6.55
                imu.publish(measurement)
                state = TerminalState()
                with lock:
                    state.level, state.reason, state.steering_rad = mode
                status.publish(state)

        worker = threading.Thread(target=publish, daemon=True)
        worker.start()
        pending = b''

        def read_for(duration):
            nonlocal pending
            lines = []
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                if select.select([master], [], [], min(.02, max(0., deadline - time.monotonic())))[0]:
                    pending += os.read(master, 65536)
                    while b'\n' in pending:
                        line, pending = pending.split(b'\n', 1)
                        text = line.decode('utf-8', errors='replace').strip()
                        if ' CTRL ' in text:
                            lines.append((time.monotonic(), text))
            return lines

        try:
            deadline = time.monotonic() + 10.
            while time.monotonic() < deadline and status.get_num_connections() == 0:
                read_for(.05)
            self.assertGreater(status.get_num_connections(), 0)
            normal = read_for(2.2)
            green = [row for row in normal if '\033[92m' in row[1]]
            self.assertGreaterEqual(len(green), 2)
            self.assertIn(' 23.45kph -6.55mps2 +12.3°', green[-1][1])
            self.assertNotIn('3596.40', green[-1][1])
            self.assertAlmostEqual(green[-1][0] - green[-2][0], 1., delta=.10)
            read_for(.03)
            changed = time.monotonic()
            with lock:
                mode[:] = ['OK', '', math.radians(40.)]
            yellow = [row for row in read_for(.68) if '\033[93m' in row[1]]
            self.assertGreaterEqual(len(yellow), 3)
            self.assertLess(yellow[0][0] - changed, .15)
            self.assertAlmostEqual(yellow[1][0] - yellow[0][0], .2, delta=.07)
            self.assertTrue(all('WARN' not in text and '+40.0°' in text for _, text in yellow))
            changed = time.monotonic()
            with lock:
                mode[:] = ['OK', '', math.radians(2.1)]
            recovered = [row for row in read_for(.15) if '\033[92m' in row[1]]
            self.assertEqual(len(recovered), 1)
            self.assertLess(recovered[0][0] - changed, .15)
            with lock:
                mode[:] = ['ERROR', 'state rx stale 0.73s', 0.]
            red = [row for row in read_for(.15) if '\033[91m' in row[1]]
            self.assertTrue(red)
            self.assertIn('ERROR state rx stale', red[0][1])
            with lock:
                mode[:] = ['OK', '', 0.]
            recovered = [row for row in read_for(.15) if '\033[92m' in row[1]]
            self.assertTrue(recovered)
            self.assertEqual(ego.get_num_connections(), 0)
        finally:
            stop.set()
            worker.join(1.)
            child.terminate()
            try:
                child.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
            os.close(master)


if __name__ == '__main__':
    rospy.init_node('control_terminal_transport_test')
    rostest.rosrun('trajectory_follower', 'control_terminal_transport', TransportTest)
