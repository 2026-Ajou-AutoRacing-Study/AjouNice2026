"""Small rqt checkbox panel: index.csv is the source of selectable run history."""

import csv
import os

import rospy
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QCheckBox, QHBoxLayout, QLabel, QPushButton, QScrollArea, QTabWidget, QVBoxLayout, QWidget
from qt_gui.plugin import Plugin
from rqt_plot.data_plot import DataPlot
from rqt_plot.plot_widget import PlotWidget

from trajectory_validator.srv import SetHistoryVisibility


class HistoryPanel(Plugin):
    def __init__(self, context):
        super(HistoryPanel, self).__init__(context)
        self.setObjectName("TrajectoryValidatorHistoryPanel")
        self.package_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        self.index = os.path.join(self.package_dir, "history", "pp_pi", "index.csv")
        self.service = rospy.ServiceProxy("/trajectory_validator/history/set_visible", SetHistoryVisibility)
        self.widget = QWidget()
        self.layout = QVBoxLayout(self.widget)
        header = QHBoxLayout()
        header.addWidget(QLabel("Trajectory Validator History (default: none selected)"))
        reload_button = QPushButton("Reload")
        reload_button.clicked.connect(self.reload)
        header.addWidget(reload_button)
        self.layout.addLayout(header)
        self.scroll = QScrollArea(); self.scroll.setWidgetResizable(True)
        self.rows = QWidget(); self.rows_layout = QVBoxLayout(self.rows); self.rows_layout.setAlignment(Qt.AlignTop)
        self.scroll.setWidget(self.rows); self.layout.addWidget(self.scroll)
        context.add_widget(self.widget)
        self.reload()

    def reload(self):
        while self.rows_layout.count():
            child = self.rows_layout.takeAt(0)
            if child.widget(): child.widget().deleteLater()
        if not os.path.isfile(self.index):
            self.rows_layout.addWidget(QLabel("No completed run yet. index.csv is created after finish/abort."))
            return
        try:
            with open(self.index, newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            for row in reversed(rows):
                label = "%s  %s  status=%s  lat_rms=%s m" % (row.get("run_label", "?"), row.get("run_id", "?"), row.get("status", "?"), row.get("lateral_rms_m", ""))
                box = QCheckBox(label)
                box.toggled.connect(lambda checked, run_id=row.get("run_id", ""): self.set_visible(run_id, checked))
                self.rows_layout.addWidget(box)
        except (OSError, csv.Error) as error:
            self.rows_layout.addWidget(QLabel("Could not read index.csv: %s" % error))

    def set_visible(self, run_id, visible):
        try:
            response = self.service(run_id=run_id, visible=visible)
            if not response.success:
                rospy.logwarn("history visibility request failed: %s", response.message)
        except rospy.ServiceException as error:
            rospy.logwarn("history service unavailable: %s", error)


class TuningPanel(HistoryPanel):
    """Fixed lateral/longitudinal plot tabs plus selectable history overlays.

    Plot selection is a visualization concern, never a controller YAML setting.
    rqt_multiplot is unavailable in this ROS image, so this panel embeds the
    installed rqt_plot widget while preserving the same fixed topic groups.
    """
    TOPICS = {
        "longitudinal": [
            "/validation/longitudinal/target_vel", "/validation/longitudinal/long_vel",
            "/validation/longitudinal/long_vel_err", "/validation/longitudinal/long_acc",
            "/validation/longitudinal/accel_cmd", "/validation/longitudinal/brake_cmd",
        ],
        "lateral": [
            "/validation/lateral/lat_err", "/validation/lateral/heading_err",
            "/validation/lateral/steering_angle_cmd", "/validation/lateral/steering_rate_cmd",
            "/validation/lateral/lat_vel", "/validation/lateral/lat_acc",
            "/validation/lateral/lookahead_distance",
        ],
        "coupled": [
            "/validation/lateral/lat_err", "/validation/lateral/steering_angle_cmd",
            "/validation/longitudinal/target_vel", "/validation/longitudinal/long_vel",
            "/validation/longitudinal/long_acc", "/validation/longitudinal/accel_cmd",
            "/validation/longitudinal/brake_cmd",
        ],
    }

    def __init__(self, context):
        super(TuningPanel, self).__init__(context)
        self.setObjectName("TrajectoryValidatorTuningPanel")
        self.widget.setWindowTitle("Trajectory Validator: time series and history")
        self.plot_widgets = []
        self.data_plots = []
        tabs = QTabWidget()
        for scope, title in (("longitudinal", "종방향"), ("lateral", "횡방향"), ("coupled", "통합")):
            page = QWidget()
            page_layout = QVBoxLayout(page)
            page_layout.addWidget(QLabel("고정 %s 검증 시계열" % title))
            plot_widget = PlotWidget(initial_topics=self.TOPICS[scope])
            data_plot = DataPlot(plot_widget)
            # ROS timestamps are epoch seconds, so a fixed 0..10 x-axis made the
            # plots appear empty.  Keep both axes following incoming samples.
            data_plot.set_autoscale(x=DataPlot.SCALE_EXTEND | DataPlot.SCALE_VISIBLE)
            data_plot.set_autoscale(y=DataPlot.SCALE_EXTEND | DataPlot.SCALE_VISIBLE)
            plot_widget.switch_data_plot_widget(data_plot)
            page_layout.addWidget(plot_widget)
            tabs.addTab(page, title)
            self.plot_widgets.append(plot_widget)
            self.data_plots.append(data_plot)
        self.layout.addWidget(QLabel("주행에는 영향을 주지 않습니다. 필요한 횡/종 탭만 선택해 확인합니다."))
        self.layout.addWidget(tabs)
        self.layout.setStretch(1, 1)
        self.layout.setStretch(3, 3)

    def shutdown_plugin(self):
        for plot_widget in self.plot_widgets:
            plot_widget.clean_up_subscribers()
