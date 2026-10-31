"""Window-selection/raising contracts without GTK, X11, ROS, or a simulator."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call


SCRIPT = (Path(__file__).resolve().parents[1] / "scripts" /
          "arrange_split_windows.py")
SPEC = importlib.util.spec_from_file_location("split_foreground_helper", SCRIPT)
HELPER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HELPER)


class SplitWindowForegroundTest(unittest.TestCase):
    @staticmethod
    def window(name, class_name, recorder, label):
        return SimpleNamespace(
            get_name=lambda: name,
            get_class_group_name=lambda: class_name,
            activate=lambda timestamp: recorder(label, timestamp),
        )

    def test_only_selected_simulator_then_rviz_are_raised(self):
        recorder = Mock()
        terminal = self.window("~/MoraiLauncher_Lin", "Terminator", recorder,
                               "terminal")
        editor = self.window("Simulator notes - Visual Studio Code", "Code",
                             recorder, "editor")
        morai = self.window("Simulator", "Simulator.x86_64", recorder, "morai")
        rviz = self.window("kcity_aw149_planning - RViz", "Rviz", recorder,
                           "rviz")
        selected_rviz, selected_morai = HELPER.choose_windows(
            [terminal, editor, morai, rviz])
        HELPER.raise_split_windows(selected_rviz, selected_morai, 194837265)
        self.assertEqual(recorder.mock_calls,
                         [call("morai", 194837265), call("rviz", 194837265)])

    def test_raise_does_not_change_geometry_maximization_or_always_on_top(self):
        morai, rviz = Mock(), Mock()
        HELPER.raise_split_windows(rviz, morai, 4294967290)
        self.assertEqual(morai.mock_calls, [call.activate(4294967290)])
        self.assertEqual(rviz.mock_calls, [call.activate(4294967290)])

    def test_regular_rviz_title_fallback_is_still_selected(self):
        recorder = Mock()
        morai = self.window("MORAI Simulator", "Simulator", recorder, "morai")
        rviz = self.window("RViz", "Rviz", recorder, "rviz")
        self.assertEqual(HELPER.choose_windows([rviz, morai]), (rviz, morai))


if __name__ == "__main__":
    unittest.main()
