#!/usr/bin/env python3
"""Compact Qt dashboard for the competition rule monitor."""

import json
import sys

import rospy
from PyQt5 import QtCore, QtGui, QtWidgets
from std_msgs.msg import String


class Bridge(QtCore.QObject):
    status = QtCore.pyqtSignal(dict)
    event = QtCore.pyqtSignal(dict)


class Window(QtWidgets.QWidget):
    def __init__(self, bridge):
        super().__init__()
        self.setWindowTitle("AjouNice Competition Rule Monitor")
        self.resize(760, 520)
        self.setStyleSheet(
            "QWidget { background: #17191d; color: #eeeeee; font-size: 14px; }"
            "QLabel#state { font-size: 30px; font-weight: bold; padding: 8px; }"
            "QTextEdit { background: #0e0f12; font-family: monospace; }"
            "QGroupBox { border: 1px solid #3d4148; margin-top: 8px; padding-top: 8px; }"
        )
        self.state = QtWidgets.QLabel("READY")
        self.state.setObjectName("state")
        self.speed = QtWidgets.QLabel("Speed: -- km/h")
        self.time = QtWidgets.QLabel("Time: 0.00 s")
        self.cp = QtWidgets.QLabel("Checkpoint: 0/15")
        self.next_cp = QtWidgets.QLabel("Next: CP1")
        self.region = QtWidgets.QLabel("Region: normal")
        self.penalty = QtWidgets.QLabel("Penalty: +0 s")
        self.total = QtWidgets.QLabel("Total: 0.00 s")
        self.run_dir = QtWidgets.QLabel("Run: --")
        self.run_dir.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.log = QtWidgets.QTextEdit()
        self.log.setReadOnly(True)
        self.log.document().setMaximumBlockCount(1000)

        grid = QtWidgets.QGridLayout()
        grid.addWidget(self.speed, 0, 0)
        grid.addWidget(self.time, 0, 1)
        grid.addWidget(self.cp, 1, 0)
        grid.addWidget(self.next_cp, 1, 1)
        grid.addWidget(self.region, 2, 0)
        grid.addWidget(self.penalty, 2, 1)
        grid.addWidget(self.total, 3, 0, 1, 2)
        box = QtWidgets.QGroupBox("Live")
        box.setLayout(grid)
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.state)
        layout.addWidget(box)
        layout.addWidget(self.run_dir)
        layout.addWidget(QtWidgets.QLabel("Penalty log"))
        layout.addWidget(self.log, 1)
        bridge.status.connect(self.update_status)
        bridge.event.connect(self.add_event)

    @QtCore.pyqtSlot(dict)
    def update_status(self, data):
        state = data.get("state", "ERROR")
        self.state.setText(state)
        colors = {"READY": "#b8bec7", "RUNNING": "#56d364", "FINISHED": "#58a6ff", "ERROR": "#ff5f56"}
        self.state.setStyleSheet("color: {};".format(colors.get(state, "#ff5f56")))
        speed = data.get("speed_kph")
        self.speed.setText("Speed: {} km/h".format("--" if speed is None else "{:.3f}".format(speed)))
        elapsed = float(data.get("elapsed_sec", 0.0))
        penalty = int(data.get("total_penalty_sec", 0))
        self.time.setText("Time: {:.2f} s".format(elapsed))
        self.cp.setText("Checkpoint: {}/{}".format(data.get("checkpoints_passed", 0), data.get("checkpoints_total", 15)))
        next_cp = data.get("next_checkpoint")
        self.next_cp.setText("Next: {}".format("FINISH" if next_cp is None else "CP{}".format(next_cp)))
        regions = []
        if data.get("gps_blackout"):
            regions.append("GPS BLACKOUT (lane penalty disabled)")
        if data.get("speed_exception"):
            regions.append("HIGH-SPEED EXCEPTION")
        self.region.setText("Region: " + (", ".join(regions) if regions else "normal"))
        self.penalty.setText("Penalty: +{} s".format(penalty))
        self.total.setText("Total: {:.2f} s".format(elapsed + penalty))
        self.run_dir.setText("Run: " + (data.get("run_directory") or "--"))

    @QtCore.pyqtSlot(dict)
    def add_event(self, data):
        display = data.get("display")
        if display:
            # Fixed-width elapsed time keeps every event message aligned even
            # when the run crosses from tens into hundreds of seconds.
            self.log.append("[{:7.3f}] {}".format(float(data.get("elapsed_sec", 0.0)), display))


def main():
    rospy.init_node("competition_rule_monitor_gui", disable_signals=True)
    app = QtWidgets.QApplication(sys.argv)
    bridge = Bridge()

    def decode_status(msg):
        try:
            bridge.status.emit(json.loads(msg.data))
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "invalid monitor status JSON: %s", exc)

    def decode_event(msg):
        try:
            bridge.event.emit(json.loads(msg.data))
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "invalid monitor event JSON: %s", exc)

    rospy.Subscriber("/competition/rule_monitor/status", String, decode_status, queue_size=20)
    rospy.Subscriber("/competition/rule_monitor/event", String, decode_event, queue_size=100)
    window = Window(bridge)
    window.show()
    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: app.quit() if rospy.is_shutdown() else None)
    timer.start(200)
    code = app.exec_()
    rospy.signal_shutdown("Qt GUI closed")
    return code
