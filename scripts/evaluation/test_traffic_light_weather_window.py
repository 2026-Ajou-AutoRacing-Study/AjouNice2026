"""Pure tests only: no display connection, screenshot, focus, or mouse input."""

import contextlib
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import traffic_light_weather_window as weather


class WindowScopeTest(unittest.TestCase):
    def test_exact_unique_official_title_only(self):
        title = weather.TITLE
        listing = '  0x123 "{}": ("weather" "weather") 800x600+0+0\n'.format(title)
        self.assertEqual(weather.parse_official_window_id(listing), 0x123)
        with self.assertRaises(RuntimeError):
            weather.parse_official_window_id(listing + listing.replace("0x123", "0x456"))
        with self.assertRaises(RuntimeError):
            weather.parse_official_window_id(listing.replace(title, title + " - extra"))
        with self.assertRaises(RuntimeError):
            weather.parse_official_window_id('0x123 "Desktop":\n')

    def test_net_wm_name_must_match_exactly(self):
        exact = '_NET_WM_NAME(UTF8_STRING) = "{}"\n'.format(weather.TITLE)
        weather.validate_window_title(exact)
        for altered in (exact.replace("UTF8_STRING", "STRING"), exact.replace("Changer", "Changer 2"), ""):
            with self.assertRaises(RuntimeError):
                weather.validate_window_title(altered)

    def test_click_bounds_and_nonpositive_window(self):
        weather.validate_click((0, 0), 100, 50)
        weather.validate_click((99, 49), 100, 50)
        weather.validate_click(None, 100, 50)
        for click in ((-1, 0), (0, -1), (100, 0), (0, 50)):
            with self.assertRaises(ValueError):
                weather.validate_click(click, 100, 50)
        with self.assertRaises(RuntimeError):
            weather.validate_click(None, 0, 50)

    def test_discovery_runs_only_fixed_read_only_commands(self):
        listing = ' 0x123 "{}":\n'.format(weather.TITLE)
        title = '_NET_WM_NAME(UTF8_STRING) = "{}"'.format(weather.TITLE)
        with patch.object(weather, "_run_read_only", side_effect=[listing, title]) as run:
            self.assertEqual(weather.find_official_window_id(), 0x123)
        self.assertEqual(run.call_args_list[0].args[0], ["xwininfo", "-root", "-tree"])
        self.assertEqual(run.call_args_list[1].args[0], ["xprop", "-id", "0x123", "_NET_WM_NAME"])

    def test_scoped_capture_never_uses_root_window(self):
        window = Mock()
        window.get_width.return_value = 100
        window.get_height.return_value = 50
        window.get_geometry.return_value = (0, 0, 100, 50)
        gdk = Mock()
        gdk.pixbuf_get_from_window.return_value.save_to_bufferv.return_value = (True, weather.PNG_SIGNATURE + b"test")
        with patch.object(weather, "validate_output_path", return_value=Path("ignored.png")), \
                patch.object(weather, "activate_official_window") as activate, \
                patch.object(weather, "verify_active_window"), \
                patch.object(weather, "verify_official_title"), patch.object(weather, "write_png_exclusive") as save:
            weather.capture_official_window(gdk, 0x123, window, Path("ignored.png"))
        gdk.pixbuf_get_from_window.assert_called_once_with(window, 0, 0, 100, 50)
        save.assert_called_once_with(Path("ignored.png"), weather.PNG_SIGNATURE + b"test")
        activate.assert_called_once_with(0x123, window)

    def test_direct_capture_rejects_outside_output_before_any_gui_operation(self):
        gdk, window = Mock(), Mock()
        with patch.object(weather, "verify_official_title") as verify:
            with self.assertRaises(ValueError):
                weather.capture_official_window(gdk, 0x123, window, Path("/tmp/outside_weather.png"))
        verify.assert_not_called()
        gdk.pixbuf_get_from_window.assert_not_called()

    def test_click_restores_pointer_without_real_input(self):
        window = Mock()
        window.get_width.return_value = 100
        window.get_height.return_value = 50
        window.get_geometry.return_value = (0, 0, 100, 50)
        device = Mock()
        device.wait_for_clear_input.return_value = {'root_window':999, 'root_x':123, 'root_y':456, 'mask':0}
        device.query_pointer.return_value = {'window_x':10, 'window_y':20, 'mask':0}
        manager = Mock()
        manager.__enter__ = Mock(return_value=device)
        manager.__exit__ = Mock(return_value=False)
        with patch.object(weather, "_X11Input", return_value=manager), \
                patch.object(weather, "activate_official_window") as activate, \
                patch.object(weather, "verify_active_window"), \
                patch.object(weather, "verify_official_title"), patch.object(weather.time, "sleep"):
            weather.click_official_window(0x123, window, (10, 20), restore_pointer=True)
        activate.assert_called_once_with(0x123, window)
        self.assertEqual(device.move_to_window.call_args_list[0].args, (0x123, 10, 20))
        self.assertEqual(device.move_to_window.call_args_list[-1].args, (999, 123, 456))
        self.assertEqual([call.args[0] for call in device.button.call_args_list], [True, False])
        device.confirm_button_released.assert_called_once()


class ManagedActivationTest(unittest.TestCase):
    def test_active_window_property_requires_typed_xid(self):
        self.assertEqual(weather.parse_active_window_id("_NET_ACTIVE_WINDOW(WINDOW): window id # 0x123\n"), 0x123)
        for malformed in ("", "_NET_ACTIVE_WINDOW: not found.", "_NET_ACTIVE_WINDOW(CARDINAL) = 291"):
            with self.assertRaises(RuntimeError):
                weather.parse_active_window_id(malformed)

    def _fake_runtime(self):
        gdk, gdk_x11, glib, wnck = (Mock() for _ in range(4))
        gdk.EventMask.PROPERTY_CHANGE_MASK = 1
        gdk_x11.x11_get_server_time.return_value = 2468
        glib.MainContext.default.return_value.pending.return_value = False
        target = Mock()
        target.get_xid.return_value = 0x123
        target.get_name.return_value = weather.TITLE
        screen = wnck.Screen.get_default.return_value
        screen.get_windows.return_value = [target]
        screen.get_active_window.return_value = target
        foreign = Mock()
        foreign.get_events.return_value = 0
        return (gdk, gdk_x11, glib, wnck), target, foreign

    def test_real_server_timestamp_and_both_active_readbacks_required(self):
        runtime, target, foreign = self._fake_runtime()
        with patch.object(weather, "_activation_dependencies", return_value=runtime), \
                patch.object(weather, "verify_official_title"), \
                patch.object(weather, "read_active_window_id", return_value=0x123):
            weather.activate_official_window(0x123, foreign)
        runtime[1].x11_get_server_time.assert_called_once_with(foreign)
        target.activate.assert_called_once_with(2468)
        self.assertEqual([call.args[0] for call in foreign.set_events.call_args_list], [1, 0])

    def test_mismatched_active_readback_times_out_without_success(self):
        runtime, target, foreign = self._fake_runtime()
        with patch.object(weather, "_activation_dependencies", return_value=runtime), \
                patch.object(weather, "verify_official_title"), \
                patch.object(weather, "read_active_window_id", return_value=0x456), \
                patch.object(weather.time, "monotonic", side_effect=[0, 0.1, 1]), \
                patch.object(weather.time, "sleep"):
            with self.assertRaises(RuntimeError):
                weather.activate_official_window(0x123, foreign, timeout_sec=0.5)
        target.activate.assert_called_once_with(2468)

    def test_zero_server_timestamp_is_rejected(self):
        runtime, target, foreign = self._fake_runtime()
        runtime[1].x11_get_server_time.return_value = 0
        with patch.object(weather, "_activation_dependencies", return_value=runtime), \
                patch.object(weather, "verify_official_title"):
            with self.assertRaises(RuntimeError):
                weather.activate_official_window(0x123, foreign)
        target.activate.assert_not_called()

    def test_failed_activation_cannot_open_input_or_click(self):
        window = Mock()
        window.get_width.return_value = 100
        window.get_height.return_value = 50
        window.get_geometry.return_value = (0, 0, 100, 50)
        with patch.object(weather, "activate_official_window", side_effect=RuntimeError("not active")), \
                patch.object(weather, "verify_official_title"), patch.object(weather, "_X11Input") as input_type:
            with self.assertRaises(RuntimeError):
                weather.click_official_window(0x123, window, (10, 20))
        input_type.assert_not_called()

    def test_lost_focus_before_button_is_rejected_without_button_event(self):
        window = Mock()
        window.get_width.return_value = 100
        window.get_height.return_value = 50
        window.get_geometry.return_value = (0, 0, 100, 50)
        device = Mock()
        manager = Mock()
        manager.__enter__ = Mock(return_value=device)
        manager.__exit__ = Mock(return_value=False)
        with patch.object(weather, "_X11Input", return_value=manager), \
                patch.object(weather, "activate_official_window"), \
                patch.object(weather, "verify_official_title"), \
                patch.object(weather, "verify_active_window", side_effect=RuntimeError("focus lost")), \
                patch.object(weather.time, "sleep"):
            with self.assertRaises(RuntimeError):
                weather.click_official_window(0x123, window, (10, 20))
        device.button.assert_not_called()


class InputSafetyTest(unittest.TestCase):
    def test_buttons_and_gesture_modifiers_are_rejected_before_button(self):
        for bit in (0, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12):
            with self.subTest(bit=bit), self.assertRaises(RuntimeError):
                weather.validate_pointer_before_button({'mask':1 << bit, 'window_x':10, 'window_y':20}, (10, 20))
        weather.validate_pointer_before_button({'mask':(1 << 1) | (1 << 4), 'window_x':10, 'window_y':20}, (10, 20))

    def test_coordinate_readback_requires_two_pixel_tolerance(self):
        weather.validate_pointer_before_button({'mask':0, 'window_x':12, 'window_y':18}, (10, 20))
        with self.assertRaises(RuntimeError):
            weather.validate_pointer_before_button({'mask':0, 'window_x':13, 'window_y':20}, (10, 20))

    def test_preflight_waits_for_clear_input_but_times_out_within_two_seconds(self):
        device = weather._X11Input.__new__(weather._X11Input)
        device.query_pointer = Mock(side_effect=[{'mask':weather.BUTTON1_MASK}, {'mask':0}])
        with patch.object(weather.time, 'sleep'):
            self.assertEqual(device.wait_for_clear_input()['mask'], 0)
        device.query_pointer = Mock(return_value={'mask':weather.BUTTON1_MASK})
        with patch.object(weather.time, 'monotonic', side_effect=[0, 2]), patch.object(weather.time, 'sleep'):
            with self.assertRaises(RuntimeError):
                device.wait_for_clear_input()

    def test_release_uses_xsync_and_requires_zero_button1_mask(self):
        device = weather._X11Input.__new__(weather._X11Input)
        device.x, device.display = Mock(), 123
        device.query_pointer = Mock(side_effect=[{'mask':weather.BUTTON1_MASK}, {'mask':0}])
        with patch.object(weather.time, 'sleep'):
            device.confirm_button_released()
        device.x.XSync.assert_called_once_with(123, False)
        device.query_pointer = Mock(return_value={'mask':weather.BUTTON1_MASK})
        with patch.object(weather.time, 'monotonic', side_effect=[0, 2]), patch.object(weather.time, 'sleep'):
            with self.assertRaises(RuntimeError):
                device.confirm_button_released()

    def test_held_input_preflight_cannot_warp_or_click(self):
        window, device, manager = Mock(), Mock(), Mock()
        window.get_geometry.return_value = (0, 0, 100, 50)
        device.wait_for_clear_input.side_effect = RuntimeError('held button')
        manager.__enter__ = Mock(return_value=device)
        manager.__exit__ = Mock(return_value=False)
        with patch.object(weather, '_X11Input', return_value=manager), \
                patch.object(weather, 'activate_official_window'), patch.object(weather, 'verify_official_title'):
            with self.assertRaises(RuntimeError):
                weather.click_official_window(0x123, window, (10, 20))
        device.move_to_window.assert_not_called()
        device.button.assert_not_called()

    def test_resize_after_activation_is_rechecked_before_button(self):
        window, device, manager = Mock(), Mock(), Mock()
        window.get_geometry.side_effect = [(0, 0, 514, 539), (0, 0, 411, 539)]
        manager.__enter__ = Mock(return_value=device)
        manager.__exit__ = Mock(return_value=False)
        with patch.object(weather, '_X11Input', return_value=manager), \
                patch.object(weather, 'activate_official_window'), patch.object(weather, 'verify_official_title'), \
                patch.object(weather.time, 'sleep'):
            with self.assertRaises(ValueError):
                weather.click_official_window(0x123, window, (500, 20))
        device.button.assert_not_called()

    def test_screenshot_only_activation_failure_does_not_capture(self):
        gdk, window = Mock(), Mock()
        with patch.object(weather, 'validate_output_path', return_value=Path('ignored.png')), \
                patch.object(weather, 'verify_official_title'), \
                patch.object(weather, 'activate_official_window', side_effect=RuntimeError('not active')):
            with self.assertRaises(RuntimeError):
                weather.capture_official_window(gdk, 0x123, window, Path('ignored.png'))
        gdk.pixbuf_get_from_window.assert_not_called()


class OutputScopeTest(unittest.TestCase):
    def test_nested_new_png_is_allowed_but_other_outputs_are_not(self):
        with tempfile.TemporaryDirectory() as directory:
            allowed = Path(directory) / "artifacts/evaluation"
            nested = allowed / "traffic_light_robustness_261025"
            nested.mkdir(parents=True)
            candidate = nested / "new.png"
            self.assertEqual(weather.validate_output_path(candidate, allowed), candidate)
            for outside in (Path(directory) / "outside.png", allowed / "../outside.png", nested / "new.jpg"):
                with self.assertRaises(ValueError):
                    weather.validate_output_path(outside, allowed)
            with self.assertRaises(ValueError):
                weather.validate_output_path(nested / "missing_parent/new.png", allowed)

    def test_symlink_escape_and_existing_outputs_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            allowed = root / "allowed"
            allowed.mkdir()
            outside = root / "outside"
            outside.mkdir()
            (allowed / "escape").symlink_to(outside, target_is_directory=True)
            with self.assertRaises(ValueError):
                weather.validate_output_path(allowed / "escape/new.png", allowed)
            (allowed / "dangling.png").symlink_to(allowed / "missing.png")
            with self.assertRaises(FileExistsError):
                weather.validate_output_path(allowed / "dangling.png", allowed)
            existing = allowed / "existing.png"
            weather.write_png_exclusive(existing, weather.PNG_SIGNATURE + b"original")
            with self.assertRaises(FileExistsError):
                weather.validate_output_path(existing, allowed)
            with self.assertRaises(FileExistsError):
                weather.write_png_exclusive(existing, weather.PNG_SIGNATURE + b"replacement")
            self.assertEqual(existing.read_bytes(), weather.PNG_SIGNATURE + b"original")

    def test_non_png_encoder_output_does_not_create_file(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "new.png"
            with self.assertRaises(RuntimeError):
                weather.write_png_exclusive(output, b"not_png")
            self.assertFalse(output.exists())

    def test_output_rejection_occurs_before_any_gui_discovery(self):
        with patch.object(weather, "find_official_window_id") as find, contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as stopped:
                weather.main(["--output", "/tmp/outside_weather.png"])
        self.assertEqual(stopped.exception.code, 2)
        find.assert_not_called()

    def test_arbitrary_exec_title_display_and_abbreviations_are_not_options(self):
        for argument in ("--exec", "--title", "--display", "--out"):
            with contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as stopped:
                    weather.main(["--output", "/tmp/unused.png", argument, "anything"])
            self.assertEqual(stopped.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
