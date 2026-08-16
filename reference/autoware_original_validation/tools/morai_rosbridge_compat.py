#!/usr/bin/env python3
"""Launch rosbridge with MORAI message compatibility for ROS 2.

MORAI MolitComp03 sends ROS1-style ``std_msgs/Header`` objects containing
``seq``. ROS 2 removed that field, and rosbridge 2.x otherwise rejects the
entire incoming message. The competition build also appends fields such as
``lateral_offset`` that are absent from the public ROS 2 ``morai_msgs``.

The competition build's ROS control input is ROS1-shaped as well. In
particular, ``CtrlCmd`` expects ``longlCmdType``, ``front_steer``, and
``rear_steer``, while the pinned ROS 2 message used by this workspace exposes
``longl_cmd_type`` and ``steering``. Add the ROS1 names only at the websocket
boundary so the ROS 2 adapter and the pinned Autoware source tree remain
unchanged.

This launcher drops only fields that the installed ROS 2 message class does
not define, while preserving and type-checking every common field before
running the stock ROS 2 rosbridge server.
"""

from __future__ import annotations

import runpy
from typing import Any

from rosbridge_library.internal import message_conversion


_original_to_object_inst = message_conversion._to_object_inst
_original_from_object_inst = message_conversion._from_object_inst
_reported_ignored_fields: set[tuple[str, str]] = set()
_reported_output_aliases: set[str] = set()


def _to_object_inst_with_ros1_header_compat(
    msg: dict[str, Any],
    rostype: str,
    roottype: str,
    clock: Any,
    inst: Any,
    stack: list[str],
) -> Any:
    installed_fields = inst.get_fields_and_field_types()
    unknown_fields = set(msg).difference(installed_fields)
    if unknown_fields:
        msg = dict(msg)
        for field_name in sorted(unknown_fields):
            msg.pop(field_name)
            field_key = (rostype, field_name)
            if field_key not in _reported_ignored_fields:
                print(
                    "[MORAI ROS 2 compatibility] ignoring unsupported field "
                    f"{rostype}.{field_name}",
                    flush=True,
                )
                _reported_ignored_fields.add(field_key)

    return _original_to_object_inst(msg, rostype, roottype, clock, inst, stack)


message_conversion._to_object_inst = _to_object_inst_with_ros1_header_compat


def _from_object_inst_with_morai_ros1_aliases(
    inst: Any,
    rostype: str,
) -> dict[str, Any]:
    """Return a ROS2 message dictionary with MORAI's ROS1 control aliases."""
    msg = _original_from_object_inst(inst, rostype)
    if rostype.endswith("/CtrlCmd"):
        if "longl_cmd_type" in msg:
            msg["longlCmdType"] = msg["longl_cmd_type"]
        if "steering" in msg:
            msg["front_steer"] = msg["steering"]
            msg["rear_steer"] = 0.0
        if rostype not in _reported_output_aliases:
            print(
                "[MORAI ROS 2 compatibility] adding ROS1 CtrlCmd aliases "
                "longlCmdType/front_steer/rear_steer",
                flush=True,
            )
            _reported_output_aliases.add(rostype)
    return msg


message_conversion._from_object_inst = _from_object_inst_with_morai_ros1_aliases

runpy.run_path(
    "/opt/ros/humble/lib/rosbridge_server/rosbridge_websocket",
    run_name="__main__",
)
