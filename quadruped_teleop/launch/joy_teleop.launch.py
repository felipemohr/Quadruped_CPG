import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    quadruped_teleop_pkg = FindPackageShare('quadruped_teleop').find('quadruped_teleop')

    default_joystick_config = os.path.join(quadruped_teleop_pkg, 'config', 'dualsense_joystick.yaml')
    joystick_config = LaunchConfiguration('joystick_config', default=default_joystick_config)

    default_quadruped_limits_config = os.path.join(quadruped_teleop_pkg, 'config', 'go1_limits.yaml')
    quadruped_limits_config = LaunchConfiguration('quadruped_limits', default=default_quadruped_limits_config)

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
        parameters=[quadruped_limits_config, joystick_config]
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='joystick_config', default_value=default_joystick_config,
                              description='Absolute path to joystick config file'),
        DeclareLaunchArgument(name='quadruped_limits', default_value=default_quadruped_limits_config,
                              description='Absolute path to quadruped limits config file'),
        joy_node,
        joy_teleop_node
    ])
