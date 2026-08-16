"""Launch the MORAI interface with its version-pinned default parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create the standalone MORAI vehicle-interface launch description."""
    default_config = PathJoinSubstitution(
        [FindPackageShare('morai_autoware_interface'), 'config', 'default.param.yaml']
    )
    config_argument = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='MORAI/Autoware interface parameter file',
    )
    interface_node = Node(
        package='morai_autoware_interface',
        executable='morai_autoware_interface',
        name='morai_autoware_interface',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )
    return LaunchDescription([config_argument, interface_node])
