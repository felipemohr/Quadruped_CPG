import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    quadruped_kinematics_pkg = FindPackageShare('quadruped_kinematics').find('quadruped_kinematics')

    default_quadruped_config = os.path.join(quadruped_kinematics_pkg, 'config', 'go1.yaml')
    quadruped_config = LaunchConfiguration('quadruped_config', default=default_quadruped_config)

    ik_server = Node(
        package='quadruped_kinematics',
        executable='ik_server',
        name='ik_server',
        output='both',
        parameters=[quadruped_config]
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='quadruped_config', default_value=default_quadruped_config,
                              description='Absolute path to quadruped config file'),
        ik_server        
    ])
