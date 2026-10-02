"""Shared monitor-bar sizing for the GUI and X11 window arranger."""
import math
import re


def normalize_bar_setting(value):
    """Accept auto, a legacy pixel count, or an explicit percentage."""
    if isinstance(value, str):
        value = value.strip().lower()
        if value == "auto":
            return "auto"
        match = re.fullmatch(r"(\d+(?:\.\d+)?)%", value)
        if match:
            percent = float(match.group(1))
            if not 10 <= percent <= 30:
                raise ValueError("GUI 하단 높이 비율은 10~30%여야 합니다")
            return "{}%".format(format(percent, "g"))
    try:
        pixels = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError("GUI 하단 높이는 auto, 100~400px 또는 10~30%여야 합니다") from error
    if not math.isfinite(pixels) or not pixels.is_integer() or not 100 <= pixels <= 400:
        raise ValueError("GUI 하단 높이 px 값은 100~400 정수여야 합니다")
    return int(pixels)


def resolve_bar_height(workarea_height, setting="auto"):
    """Return physical pixels; keep room for the decorated GUI and its controls."""
    setting = normalize_bar_setting(setting)
    if setting == "auto":
        return max(170, min(275, round(workarea_height * 0.17)))
    if isinstance(setting, str):
        return max(140, min(400, round(workarea_height * float(setting[:-1]) / 100)))
    return max(140, setting)
