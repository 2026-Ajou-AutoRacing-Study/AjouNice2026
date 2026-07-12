#!/usr/bin/env python3

import threading
import uuid as uuid_library
from copy import deepcopy

import rospy
from tier4_rtc_msgs.msg import (
    Command,
    CooperateResponse,
    CooperateStatus,
    CooperateStatusArray,
    State,
)
from tier4_rtc_msgs.srv import (
    AutoMode,
    AutoModeResponse,
    CooperateCommands,
    CooperateCommandsResponse,
)
from unique_identifier_msgs.msg import UUID


def new_uuid():
    message = UUID()
    message.uuid = list(uuid_library.uuid4().bytes)
    return message


def uuid_key(value):
    return tuple(int(item) for item in value.uuid)


class RTCInterfaceROS1:
    """ROS 1 implementation of Autoware's per-module RTC communication surface."""

    def __init__(
        self,
        module_type,
        status_topic,
        command_service,
        auto_mode_service,
        enable_rtc=False,
    ):
        self.module_type = int(module_type)
        self.status_topic = str(status_topic)
        self.command_service_name = str(command_service)
        self.auto_mode_service_name = str(auto_mode_service)
        self._auto_mode = not bool(enable_rtc)
        self._statuses = {}
        self._lock = threading.RLock()

        self._status_pub = rospy.Publisher(
            self.status_topic, CooperateStatusArray, queue_size=1
        )
        self._command_server = rospy.Service(
            self.command_service_name,
            CooperateCommands,
            self._on_commands,
        )
        self._auto_mode_server = rospy.Service(
            self.auto_mode_service_name,
            AutoMode,
            self._on_auto_mode,
        )

    def auto_mode_enabled(self):
        with self._lock:
            return self._auto_mode

    def is_registered(self, request_uuid):
        with self._lock:
            return uuid_key(request_uuid) in self._statuses

    def update_status(
        self,
        request_uuid,
        safe,
        state,
        start_distance,
        finish_distance,
        stamp=None,
        requested=False,
    ):
        stamp = stamp if stamp is not None else rospy.Time.now()
        key = uuid_key(request_uuid)
        with self._lock:
            status = self._statuses.get(key)
            if status is None:
                status = CooperateStatus()
                status.uuid = deepcopy(request_uuid)
                status.module.type = self.module_type
                status.command_status.type = Command.DEACTIVATE
                self._statuses[key] = status
            status.stamp = stamp
            status.safe = bool(safe)
            status.requested = bool(requested)
            status.state.type = int(state)
            status.auto_mode = self._auto_mode
            status.start_distance = float(start_distance)
            status.finish_distance = float(finish_distance)

    def is_activated(self, request_uuid):
        with self._lock:
            status = self._statuses.get(uuid_key(request_uuid))
            if status is None:
                return False
            if self._auto_mode:
                return bool(status.safe) and not bool(status.requested)
            return status.command_status.type == Command.ACTIVATE

    def remove_status(self, request_uuid):
        with self._lock:
            self._statuses.pop(uuid_key(request_uuid), None)

    def clear(self):
        with self._lock:
            self._statuses.clear()

    def publish(self, stamp=None):
        stamp = stamp if stamp is not None else rospy.Time.now()
        message = CooperateStatusArray()
        message.stamp = stamp
        with self._lock:
            for key in sorted(self._statuses):
                status = deepcopy(self._statuses[key])
                status.stamp = stamp
                status.auto_mode = self._auto_mode
                message.statuses.append(status)
        self._status_pub.publish(message)

    def _on_commands(self, request):
        response = CooperateCommandsResponse()
        with self._lock:
            for command in request.commands:
                result = CooperateResponse()
                result.uuid = deepcopy(command.uuid)
                result.module = deepcopy(command.module)
                key = uuid_key(command.uuid)
                status = self._statuses.get(key)
                result.success = bool(
                    status is not None
                    and command.module.type == self.module_type
                    and command.command.type in (Command.DEACTIVATE, Command.ACTIVATE)
                )
                if result.success:
                    status.command_status = deepcopy(command.command)
                response.responses.append(result)
        self.publish(request.stamp)
        return response

    def _on_auto_mode(self, request):
        with self._lock:
            self._auto_mode = bool(request.enable)
            for status in self._statuses.values():
                status.auto_mode = self._auto_mode
        self.publish(rospy.Time.now())
        return AutoModeResponse(success=True)


class RTCSceneController:
    """Tracks one active scene request across directional RTC interfaces."""

    def __init__(self, interfaces):
        self.interfaces = dict(interfaces)
        self.direction = None
        self.signature = None
        self.request_uuid = None
        self.was_activated = False
        self.ever_activated = False
        self.pending_removal = None
        self.start_distance = 0.0
        self.finish_distance = 0.0

    def evaluate(
        self,
        direction,
        signature,
        safe,
        start_distance,
        finish_distance,
        stamp,
        requested=False,
    ):
        self._remove_terminal(stamp)
        interface = self.interfaces[direction]
        if self.signature != signature or self.direction != direction:
            self._discard_active(stamp)
            self.direction = direction
            self.signature = signature
            self.request_uuid = new_uuid()
            self.was_activated = False
            self.ever_activated = False
        self.start_distance = float(start_distance)
        self.finish_distance = float(finish_distance)

        interface.update_status(
            self.request_uuid,
            safe,
            State.WAITING_FOR_EXECUTION,
            start_distance,
            finish_distance,
            stamp,
            requested=requested,
        )
        activated = interface.is_activated(self.request_uuid)
        if activated:
            state = State.RUNNING
            self.ever_activated = True
        elif self.was_activated:
            state = State.ABORTING
        else:
            state = State.WAITING_FOR_EXECUTION

        interface.update_status(
            self.request_uuid,
            safe,
            state,
            start_distance,
            finish_distance,
            stamp,
            requested=requested,
        )
        interface.publish(stamp)
        self.was_activated = activated
        return activated, state, deepcopy(self.request_uuid), interface.auto_mode_enabled()

    def complete(self, stamp):
        self._remove_terminal(stamp)
        if self.request_uuid is None:
            return
        interface = self.interfaces[self.direction]
        if self.ever_activated:
            interface.update_status(
                self.request_uuid,
                True,
                State.SUCCEEDED,
                self.start_distance,
                self.finish_distance,
                stamp,
            )
            interface.publish(stamp)
            self.pending_removal = (interface, deepcopy(self.request_uuid))
        else:
            interface.remove_status(self.request_uuid)
            interface.publish(stamp)
        self.direction = None
        self.signature = None
        self.request_uuid = None
        self.was_activated = False
        self.ever_activated = False

    def _discard_active(self, stamp):
        if self.request_uuid is None:
            return
        interface = self.interfaces[self.direction]
        interface.remove_status(self.request_uuid)
        interface.publish(stamp)

    def _remove_terminal(self, stamp):
        if self.pending_removal is None:
            return
        interface, request_uuid = self.pending_removal
        interface.remove_status(request_uuid)
        interface.publish(stamp)
        self.pending_removal = None
