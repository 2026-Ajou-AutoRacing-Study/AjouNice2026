#!/usr/bin/env python3
"""Raise/maximize Planviz in a fresh GTK process before X11 recording."""
import sys
import time


def main():
    import gi
    gi.require_version("Wnck", "3.0")
    from gi.repository import GLib, Wnck

    screen = Wnck.Screen.get_default()
    if screen is None:
        print("X11 screen unavailable", file=sys.stderr)
        return 1
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
    deadline = time.monotonic() + timeout
    context = GLib.MainContext.default()
    while time.monotonic() < deadline:
        while context.pending():
            context.iteration(False)
        screen.force_update()
        windows = [window for window in screen.get_windows()
                   if "kcity_aw149_planning" in (window.get_name() or "").lower()]
        if not windows:
            windows = [window for window in screen.get_windows()
                       if "rviz" in (window.get_name() or "").lower()]
        if windows:
            window = windows[-1]
            window.maximize()
            window.activate(int(time.monotonic() * 1000) & 0xffffffff)
            for _ in range(10):
                while context.pending():
                    context.iteration(False)
                time.sleep(0.2)
                screen.force_update()
                active = screen.get_active_window()
                if (window.is_maximized() and active is not None
                        and active.get_xid() == window.get_xid()):
                    return 0
        time.sleep(0.2)
    print("Planviz RViz window not found/maximized/active", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
