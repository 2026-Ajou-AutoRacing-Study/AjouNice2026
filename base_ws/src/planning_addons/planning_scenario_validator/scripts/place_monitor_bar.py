#!/usr/bin/env python3
"""Set the validator GUI's *outer* X11 rectangle, including window decorations.

Run separately from Tk: libwnck has occasionally crashed while another window
was being replaced, and a failed layout must never terminate the campaign GUI.
"""
import sys
import time


def find_gui_window(windows, title):
    matches = [window for window in windows
               if window.get_name() == title]
    return matches[-1] if matches else None


def main(argv=None):
    import gi
    gi.require_version("Wnck", "3.0")
    from gi.repository import GLib, Wnck

    args = sys.argv[1:] if argv is None else argv
    if len(args) != 5:
        print("usage: place_monitor_bar.py TITLE X Y WIDTH HEIGHT", file=sys.stderr)
        return 2
    title = args[0]
    x, y, width, height = map(int, args[1:])
    screen = Wnck.Screen.get_default()
    if screen is None:
        return 1
    context = GLib.MainContext.default()
    deadline = time.monotonic() + 2.0
    window = None
    while time.monotonic() < deadline:
        while context.pending():
            context.iteration(False)
        screen.force_update()
        window = find_gui_window(screen.get_windows(), title)
        if window is not None:
            break
        time.sleep(0.1)
    if window is None:
        print("GUI 창을 찾지 못했습니다", file=sys.stderr)
        return 1
    mask = (Wnck.WindowMoveResizeMask.X | Wnck.WindowMoveResizeMask.Y |
            Wnck.WindowMoveResizeMask.WIDTH | Wnck.WindowMoveResizeMask.HEIGHT)
    for _ in range(2):
        window.set_geometry(Wnck.WindowGravity.STATIC, mask, x, y, width, height)
        time.sleep(0.15)
        while context.pending():
            context.iteration(False)
    screen.force_update()
    actual = window.get_geometry()
    if any(abs(a - b) > 8 for a, b in zip(actual, (x, y, width, height))):
        print("GUI 창 배치 불일치: 요청 {}, 실제 {}".format(
            (x, y, width, height), actual), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
