"""Launch an adapter whose vehicle outputs are fully isolated from MORAI."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the non-driving verification adapter."""
    interface_node = Node(
        package='morai_autoware_interface',
        executable='morai_autoware_interface',
        name='morai_autoware_verification_adapter',
        output='screen',
        parameters=[
            {
                'morai.ctrl_topic': '/verification/morai_ctrl_cmd',
                'morai.event_service': '/verification/no_event_service',
                'autoware.control_topic': '/verification/autoware_control_cmd',
                'autoware.gear_topic': '/verification/autoware_gear_cmd',
                'autoware.turn_topic': '/verification/autoware_turn_cmd',
                'autoware.hazard_topic': '/verification/autoware_hazard_cmd',
                'autoware.control_mode_service': (
                    '/verification/control_mode_request'
                ),
                'initial.control_mode': 'autonomous',
                'initial.gear': 'drive',
                'initial.set_autonomous_and_drive_on_start': False,
                'command.timeout_sec': 0.5,
                'publish.tf': False,
                'publish.objects': True,
            }
        ],
        remappings=[
            (
                '/vehicle/status/velocity_status',
                '/verification/vehicle/status/velocity_status',
            ),
            (
                '/vehicle/status/steering_status',
                '/verification/vehicle/status/steering_status',
            ),
            (
                '/vehicle/status/control_mode',
                '/verification/vehicle/status/control_mode',
            ),
            (
                '/vehicle/status/gear_status',
                '/verification/vehicle/status/gear_status',
            ),
            (
                '/vehicle/status/turn_indicators_status',
                '/verification/vehicle/status/turn_indicators_status',
            ),
            (
                '/vehicle/status/hazard_lights_status',
                '/verification/vehicle/status/hazard_lights_status',
            ),
            (
                '/vehicle/status/actuation_status',
                '/verification/vehicle/status/actuation_status',
            ),
            (
                '/localization/kinematic_state',
                '/verification/localization/kinematic_state',
            ),
            (
                '/localization/acceleration',
                '/verification/localization/acceleration',
            ),
            (
                '/perception/object_recognition/objects',
                '/verification/perception/object_recognition/objects',
            ),
        ],
    )
    return LaunchDescription([interface_node])
