import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    quadruped_teleop_pkg_share = FindPackageShare('quadruped_teleop').find('quadruped_teleop')

    default_joystick_config = os.path.join(quadruped_teleop_pkg_share, 'config', 'dualsense_joystick.yaml')
    joystick_config = LaunchConfiguration('joystick_config', default=default_joystick_config)

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='both',
        parameters=[{
          'deadzone': 0.05,
          'autorepeat_rate': 0.0,
          'sticky_buttons': False,
          'coalesce_interval_ms': 1
        }]
    )

    joy_teleop_node = Node(
        package='quadruped_teleop',
        executable='joy_teleop_node',
        name='joy_teleop_node',
        parameters=[joystick_config]
    )

    return LaunchDescription({
        DeclareLaunchArgument(name='joystick_config', default_value=default_joystick_config,
                              description='Absolute path to joystick config file'),
        joy_node,
        joy_teleop_node
    })
