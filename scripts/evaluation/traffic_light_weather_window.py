#!/usr/bin/env python3
"""Screenshot/click only the official MORAI weather window on current DISPLAY.

Adapted, without modifying it, from the frozen helper
artifacts/evaluation/base5_sunny11_training_export_261019_v1/remote_weather_ui.py.
There is no desktop capture, arbitrary title, display, remote host, command, or
ROS-node option. A click is relative to the one exact-title window. Output must
be a new .png below this repository's artifacts/evaluation directory.

Example (the output parent directory must already exist)::

    /usr/bin/python3 scripts/evaluation/traffic_light_weather_window.py \\
        --output artifacts/evaluation/traffic_light_robustness_261025/weather.png

Add ``--click X Y --restore-pointer`` only when a weather-tool click is intended.
Importing this module or running --help does not connect to a display.
"""

import argparse
import ctypes
import ctypes.util
from pathlib import Path
import re
import subprocess
import time


TITLE = "MORAI Simulator - Time / Weather Changer"
REPO_ROOT = Path(__file__).resolve().parents[2]
ALLOWED_OUTPUT_ROOT = REPO_ROOT / "artifacts/evaluation"
DEFAULT_OUTPUT_DIRECTORY = ALLOWED_OUTPUT_ROOT / "traffic_light_robustness_261025"
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
BUTTON1_MASK = 1 << 8
BUTTON_MASKS = sum(1 << bit for bit in range(8, 13))
# Shift, Control, Alt/Mod1, Mod3, Super/Mod4 and Mod5. Ignore Caps/Num lock.
GESTURE_MODIFIER_MASKS = (1 << 0) | (1 << 2) | (1 << 3) | (1 << 5) | (1 << 6) | (1 << 7)
UNSAFE_INPUT_MASKS = BUTTON_MASKS | GESTURE_MODIFIER_MASKS


def validate_output_path(output, allowed_root=ALLOWED_OUTPUT_ROOT):
    """Check canonical containment and nonexistence before any GUI operation."""
    requested = Path(output).expanduser()
    if requested.suffix != ".png":
        raise ValueError("output must have the .png suffix")
    # Do not resolve the allowed root through a symlink escaping the repository.
    root = Path(allowed_root).absolute()
    resolved = requested.resolve()
    try:
        resolved.relative_to(root)
    except ValueError:
        raise ValueError("output must remain below {}".format(root))
    if requested.exists() or requested.is_symlink() or resolved.exists():
        raise FileExistsError("output already exists: {}".format(requested))
    if not resolved.parent.is_dir():
        raise ValueError("output parent directory must already exist")
    return resolved


def parse_official_window_id(listing):
    pattern = r'^\s*(0x[0-9a-fA-F]+) "' + re.escape(TITLE) + r'":'
    matches = re.findall(pattern, listing, re.MULTILINE)
    if len(matches) != 1:
        raise RuntimeError("exactly one official weather tool window is required")
    return int(matches[0], 16)


def validate_window_title(property_text):
    if property_text.strip() != '_NET_WM_NAME(UTF8_STRING) = "' + TITLE + '"':
        raise RuntimeError("official weather window title changed or is not exact")


def validate_click(click, width, height):
    if width <= 0 or height <= 0:
        raise RuntimeError("official weather window has no positive drawable size")
    if click is not None:
        x, y = click
        if not (0 <= x < width and 0 <= y < height):
            raise ValueError("click must remain inside the official weather window")


def current_window_dimensions(window):
    """Use current server geometry after activation, not pre-activation bounds."""
    window.get_display().sync()
    geometry = window.get_geometry()  # Gdk3: x, y, width, height.
    width, height = int(geometry[2]), int(geometry[3])
    validate_click(None, width, height)
    return width, height


def validate_pointer_before_button(state, click, tolerance=2):
    if state['mask'] & UNSAFE_INPUT_MASKS:
        raise RuntimeError("a mouse button or gesture modifier is held; refusing click")
    if max(abs(state['window_x'] - click[0]), abs(state['window_y'] - click[1])) > tolerance:
        raise RuntimeError("pointer did not reach the requested weather-window coordinates; refusing click")


def _run_read_only(arguments):
    return subprocess.run(
        arguments, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, timeout=5,
    ).stdout


def verify_official_title(window_id):
    # The ID comes only from the exact, unique fixed-title listing above.
    validate_window_title(_run_read_only(["xprop", "-id", hex(window_id), "_NET_WM_NAME"]))


def find_official_window_id():
    window_id = parse_official_window_id(_run_read_only(["xwininfo", "-root", "-tree"]))
    verify_official_title(window_id)
    return window_id


def load_official_window(window_id):
    """Lazy GUI imports: this is never called by pure validation or --help."""
    import gi

    gi.require_version("Gdk", "3.0")
    gi.require_version("GdkX11", "3.0")
    from gi.repository import Gdk, GdkX11

    display = Gdk.Display.get_default()
    if display is None or not isinstance(display, GdkX11.X11Display):
        raise RuntimeError("the current default DISPLAY must be an available X11 display")
    window = GdkX11.X11Window.foreign_new_for_display(display, window_id)
    if window is None:
        raise RuntimeError("official weather window is unavailable on the current display")
    return Gdk, window


def parse_active_window_id(property_text):
    match = re.fullmatch(
        r"_NET_ACTIVE_WINDOW\(WINDOW\): window id # (0x[0-9a-fA-F]+)\s*",
        property_text.strip(),
    )
    if match is None:
        raise RuntimeError("cannot read back the current _NET_ACTIVE_WINDOW")
    return int(match.group(1), 16)


def read_active_window_id():
    return parse_active_window_id(_run_read_only(["xprop", "-root", "_NET_ACTIVE_WINDOW"]))


def verify_active_window(window_id):
    if read_active_window_id() != window_id:
        raise RuntimeError("official weather window is not the active window; refusing click")


def _activation_dependencies():
    import gi

    gi.require_version("Gdk", "3.0")
    gi.require_version("GdkX11", "3.0")
    gi.require_version("Wnck", "3.0")
    from gi.repository import Gdk, GdkX11, GLib, Wnck

    return Gdk, GdkX11, GLib, Wnck


def activate_official_window(window_id, foreign_window, timeout_sec=3.0):
    """Request managed activation, requiring exact active-XID readback.

    A real X server timestamp is required: a monotonic-clock approximation can
    be rejected by GNOME's focus-stealing protection. This never maximizes,
    searches for another application, or falls back to raw XSetInputFocus.
    """
    verify_official_title(window_id)
    Gdk, GdkX11, GLib, Wnck = _activation_dependencies()
    screen = Wnck.Screen.get_default()
    if screen is None:
        raise RuntimeError("current X11 Wnck screen is unavailable; refusing click")
    context = GLib.MainContext.default()
    # Drain bounded batches so a busy event queue cannot defeat the timeout.
    for _ in range(128):
        if not context.pending():
            break
        context.iteration(False)
    screen.force_update()
    targets = [
        candidate for candidate in screen.get_windows()
        if candidate.get_xid() == window_id and candidate.get_name() == TITLE
    ]
    if len(targets) != 1:
        raise RuntimeError("exact official weather window is not uniquely managed; refusing click")

    original_events = foreign_window.get_events()
    try:
        # x11_get_server_time requires PropertyChangeMask on the foreign window.
        foreign_window.set_events(original_events | Gdk.EventMask.PROPERTY_CHANGE_MASK)
        timestamp = int(GdkX11.x11_get_server_time(foreign_window))
    finally:
        foreign_window.set_events(original_events)
    if not 0 < timestamp <= 0xFFFFFFFF:
        raise RuntimeError("a real nonzero X11 server timestamp is required; refusing click")
    targets[0].activate(timestamp)
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        for _ in range(128):
            if not context.pending():
                break
            context.iteration(False)
        screen.force_update()
        active = screen.get_active_window()
        if active is not None and active.get_xid() == window_id:
            # Both the managed-window view and the WM's root property must agree.
            if read_active_window_id() == window_id:
                verify_official_title(window_id)
                return
        time.sleep(0.05)
    raise RuntimeError("official weather window activation was not confirmed; refusing click")


class _X11Input:
    """Minimal current-display X11 input; no command or display selection API."""

    def __enter__(self):
        x11_path = ctypes.util.find_library("X11")
        xtst_path = ctypes.util.find_library("Xtst")
        if not x11_path or not xtst_path:
            raise RuntimeError("X11 and Xtst libraries are required for a click")
        self.x = ctypes.CDLL(x11_path)
        self.t = ctypes.CDLL(xtst_path)
        pointer, ulong, integer = ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int
        self.x.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x.XOpenDisplay.restype = pointer
        self.x.XCloseDisplay.argtypes = [pointer]
        self.x.XDefaultRootWindow.argtypes = [pointer]
        self.x.XDefaultRootWindow.restype = ulong
        self.x.XWarpPointer.argtypes = [
            pointer, ulong, ulong, integer, integer, ctypes.c_uint, ctypes.c_uint, integer, integer,
        ]
        self.x.XQueryPointer.argtypes = [
            pointer, ulong, ctypes.POINTER(ulong), ctypes.POINTER(ulong),
            ctypes.POINTER(integer), ctypes.POINTER(integer), ctypes.POINTER(integer),
            ctypes.POINTER(integer), ctypes.POINTER(ctypes.c_uint),
        ]
        self.x.XQueryPointer.restype = integer
        self.x.XFlush.argtypes = [pointer]
        self.x.XSync.argtypes = [pointer, integer]
        self.t.XTestFakeButtonEvent.argtypes = [pointer, ctypes.c_uint, integer, ulong]
        self.t.XTestFakeButtonEvent.restype = integer
        self.display = self.x.XOpenDisplay(None)  # Current process DISPLAY only.
        if not self.display:
            raise RuntimeError("cannot open the current X11 display")
        return self

    def __exit__(self, *_args):
        self.x.XCloseDisplay(self.display)

    def query_pointer(self, window_id=None):
        root = self.x.XDefaultRootWindow(self.display)
        queried_window = root if window_id is None else window_id
        root_return, child_return = ctypes.c_ulong(), ctypes.c_ulong()
        root_x, root_y, window_x, window_y = (ctypes.c_int() for _ in range(4))
        mask = ctypes.c_uint()
        success = self.x.XQueryPointer(
            self.display, queried_window, ctypes.byref(root_return), ctypes.byref(child_return),
            ctypes.byref(root_x), ctypes.byref(root_y), ctypes.byref(window_x),
            ctypes.byref(window_y), ctypes.byref(mask),
        )
        if not success:
            raise RuntimeError("cannot query pointer on the current X11 screen")
        return {'root_window': root_return.value, 'root_x': root_x.value, 'root_y': root_y.value,
                'window_x': window_x.value, 'window_y': window_y.value, 'mask': mask.value}

    def wait_for_clear_input(self, timeout_sec=2.0):
        deadline = time.monotonic() + timeout_sec
        while True:
            state = self.query_pointer()
            if not state['mask'] & UNSAFE_INPUT_MASKS:
                return state
            if time.monotonic() >= deadline:
                raise RuntimeError("mouse buttons/modifiers did not clear within 2s; refusing pointer movement")
            time.sleep(0.05)

    def confirm_button_released(self, timeout_sec=2.0):
        self.x.XSync(self.display, False)
        deadline = time.monotonic() + timeout_sec
        while True:
            if not self.query_pointer()['mask'] & BUTTON1_MASK:
                return
            if time.monotonic() >= deadline:
                raise RuntimeError("Button1 release was not confirmed within 2s")
            time.sleep(0.05)

    def move_to_window(self, window_id, x, y):
        self.x.XWarpPointer(self.display, 0, window_id, 0, 0, 0, 0, x, y)
        self.x.XFlush(self.display)

    def button(self, pressed):
        if not self.t.XTestFakeButtonEvent(self.display, 1, int(pressed), 0):
            raise RuntimeError("X11 did not accept the scoped weather-window button event")
        self.x.XFlush(self.display)


def click_official_window(window_id, window, click, restore_pointer=False):
    """Activate and left-click only after managed focus readback is confirmed."""
    verify_official_title(window_id)
    # Fail before even opening the input connection or moving the pointer.
    activate_official_window(window_id, window)
    validate_click(click, *current_window_dimensions(window))
    with _X11Input() as input_device:
        clear_state = input_device.wait_for_clear_input()
        original = ((clear_state['root_window'], clear_state['root_x'], clear_state['root_y'])
                    if restore_pointer else None)
        pressed = False
        try:
            input_device.move_to_window(window_id, *click)
            time.sleep(0.2)
            validate_click(click, *current_window_dimensions(window))
            verify_official_title(window_id)
            verify_active_window(window_id)
            validate_pointer_before_button(input_device.query_pointer(window_id), click)
            input_device.button(True)
            pressed = True
            time.sleep(0.12)
            input_device.button(False)
            pressed = False
            input_device.confirm_button_released()
        finally:
            try:
                if pressed:
                    input_device.button(False)
                    input_device.confirm_button_released()
            finally:
                if original is not None:
                    # Restoring while a physical button/modifier is held could
                    # drag/resize a managed window just like the initial warp.
                    input_device.wait_for_clear_input()
                    input_device.move_to_window(*original)
    time.sleep(0.8)


def write_png_exclusive(output, png_bytes):
    """Never replace existing output, including a symlink created after checks."""
    payload = bytes(png_bytes)
    if not payload.startswith(PNG_SIGNATURE):
        raise RuntimeError("window encoder did not return PNG data")
    with Path(output).open("xb") as stream:
        stream.write(payload)


def capture_official_window(gdk, window_id, window, output):
    # Enforce the same output scope even when this function is called directly.
    output = validate_output_path(output)
    verify_official_title(window_id)
    # Screenshot-only invocations also need an unobscured active managed app.
    activate_official_window(window_id, window)
    width, height = current_window_dimensions(window)
    verify_active_window(window_id)
    # Deliberately use only the foreign app window, never a root/desktop window.
    pixels = gdk.pixbuf_get_from_window(window, 0, 0, width, height)
    if pixels is None:
        raise RuntimeError("official weather app window is not viewable")
    success, encoded = pixels.save_to_bufferv("png", [], [])
    if not success:
        raise RuntimeError("cannot encode the official weather window as PNG")
    write_png_exclusive(output, encoded)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", required=True, type=Path, help="new .png below artifacts/evaluation")
    parser.add_argument("--click", nargs=2, type=int, metavar=("X", "Y"), help="optional window-relative left click")
    parser.add_argument("--restore-pointer", action="store_true", help="restore mouse position after an explicit click")
    args = parser.parse_args(argv)
    if args.restore_pointer and args.click is None:
        parser.error("--restore-pointer requires --click")
    try:
        output = validate_output_path(args.output)
        window_id = find_official_window_id()
        gdk, window = load_official_window(window_id)
        validate_click(args.click, *current_window_dimensions(window))
        if args.click is not None:
            click_official_window(window_id, window, args.click, args.restore_pointer)
        capture_official_window(gdk, window_id, window, output)
        print(output)
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
