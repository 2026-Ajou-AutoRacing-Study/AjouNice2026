#!/usr/bin/env python3
"""Tile the MORAI game and Planviz above the validator GUI's bottom monitor bar.

This is a best-effort X11 helper. The caller records the full screen even if a
window refuses resizing, so a layout failure must not be mistaken for a failed
scenario or sensor connection.
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from planning_scenario_validator.layout import resolve_bar_height


def choose_windows(windows):
    """Do not mistake a terminal opened in ~/MoraiLauncher_Lin for the game."""
    names = [(window, (window.get_name() or "").lower(),
              (window.get_class_group_name() or "").lower()) for window in windows]
    rviz = [window for window, name, _ in names if "kcity_aw149_planning" in name]
    if not rviz:
        rviz = [window for window, name, _ in names if "rviz" in name]
    morai = [window for window, name, class_name in names
             if ("morai" in name or "simulator" in name)
             and not any(term in class_name for term in
                         ("terminal", "terminator", "code", "firefox", "chrome"))
             and "rviz" not in name]
    return (rviz[-1] if rviz else None, morai[-1] if morai else None)


def split_rectangles(area, bottom_bar_px):
    """Reserve the monitor bar only below RViz; MORAI fills the right side."""
    top_height = area.height - bottom_bar_px
    left_width = area.width // 2
    return ((area.x, area.y, left_width, top_height),
            (area.x + left_width, area.y, area.width - left_width, area.height))


def clear_maximized_axes(window):
    """A single maximized axis also makes the WM reject resize requests."""
    if window.is_maximized_horizontally():
        window.unmaximize_horizontally()
    if window.is_maximized_vertically():
        window.unmaximize_vertically()
    if window.is_fullscreen():
        window.set_fullscreen(False)


def main(argv=None):
    import gi
    gi.require_version("Gdk", "3.0")
    gi.require_version("Wnck", "3.0")
    from gi.repository import Gdk, GLib, Wnck

    args = sys.argv[1:] if argv is None else argv
    bar_setting = args[0] if args else "auto"
    startup_grace_sec = max(0.0, float(args[1])) if len(args) > 1 else 0.0
    screen = Wnck.Screen.get_default()
    if screen is None:
        print("X11 screen unavailable", file=sys.stderr)
        return 1
    context = GLib.MainContext.default()
    deadline = time.monotonic() + 3.0
    rviz = morai = None
    while time.monotonic() < deadline:
        while context.pending():
            context.iteration(False)
        screen.force_update()
        rviz, morai = choose_windows(screen.get_windows())
        if rviz is not None and morai is not None:
            break
        time.sleep(0.2)
    if rviz is None or morai is None:
        print("필요한 창을 찾지 못했습니다: RViz={}, MORAI={}. 수동 반반 배치하세요".format(
            bool(rviz), bool(morai)), file=sys.stderr)
        return 1

    # The window exists before RViz restores its saved QMainWindow geometry.
    # An immediate resize briefly works, then RViz puts its old size back.
    # Only the first case after launching Planviz needs this startup grace.
    if startup_grace_sec:
        time.sleep(startup_grace_sec)
        while context.pending():
            context.iteration(False)
        screen.force_update()
        rviz, morai = choose_windows(screen.get_windows())
        if rviz is None or morai is None:
            print("RViz/MORAI 창이 초기화 중 사라졌습니다", file=sys.stderr)
            return 1

    gdk_display = Gdk.Display.get_default()
    morai_x, morai_y, morai_width, morai_height = morai.get_geometry()
    monitor = gdk_display.get_monitor_at_point(morai_x + morai_width // 2,
                                               morai_y + morai_height // 2)
    area = monitor.get_workarea()
    try:
        bottom_bar_px = resolve_bar_height(area.height, bar_setting)
    except ValueError as error:
        print(error, file=sys.stderr)
        return 1
    top_height = area.height - bottom_bar_px
    if area.width < 900 or top_height < 400:
        print("화면 작업 영역이 반반 배치에 너무 작습니다: {}x{}".format(
            area.width, area.height), file=sys.stderr)
        return 1
    mask = (Wnck.WindowMoveResizeMask.X | Wnck.WindowMoveResizeMask.Y |
            Wnck.WindowMoveResizeMask.WIDTH | Wnck.WindowMoveResizeMask.HEIGHT)
    left, right = split_rectangles(area, bottom_bar_px)
    targets = ((rviz, left), (morai, right))
    for window, _ in targets:
        clear_maximized_axes(window)
    for _ in range(2):
        while context.pending():
            context.iteration(False)
        screen.force_update()
        for window, (x, y, target_width, target_height) in targets:
            window.set_geometry(Wnck.WindowGravity.STATIC, mask,
                                x, y, target_width, target_height)
        time.sleep(0.35)
    # Catch a late RViz config/Qt layout restore before recording starts.
    for _ in range(3):
        time.sleep(0.5)
        while context.pending():
            context.iteration(False)
        screen.force_update()
        if all(all(abs(actual - expected) <= 16 for actual, expected in zip(
                window.get_geometry(), rect)) for window, rect in targets):
            continue
        for window, (x, y, width, height) in targets:
            window.set_geometry(Wnck.WindowGravity.STATIC, mask, x, y, width, height)
    screen.force_update()
    refused = []
    for window, (x, y, target_width, target_height) in targets:
        actual_x, actual_y, actual_width, actual_height = window.get_geometry()
        if (abs(actual_x - x) > 16 or abs(actual_y - y) > 16 or
                abs(actual_width - target_width) > 16 or
                abs(actual_height - target_height) > 16):
            refused.append("{}: target={}, actual={}".format(
                window.get_name() or "이름 없는 창",
                (x, y, target_width, target_height),
                (actual_x, actual_y, actual_width, actual_height)))
    if refused:
        print("창 크기 조절을 거부한 창: {}. 수동 반반 배치하세요".format(", ".join(refused)),
              file=sys.stderr)
        return 1
    print("split layout: RViz={} MORAI={} bar={}px".format(
        rviz.get_geometry(), morai.get_geometry(), bottom_bar_px))
    return 0


if __name__ == "__main__":
    sys.exit(main())
