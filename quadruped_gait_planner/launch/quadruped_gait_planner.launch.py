import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    quadruped_gait_planner_pkg = FindPackageShare('quadruped_gait_planner').find('quadruped_gait_planner')

    default_cpg_config = os.path.join(quadruped_gait_planner_pkg, 'config', 'cpg.yaml')
    cpg_config = LaunchConfiguration('cpg_config', default=default_cpg_config)

    gait_planner_node = Node(
        package='quadruped_gait_planner',
        executable='gait_planner_node',
        name='gait_planner_node',
        output='both',
        parameters=[cpg_config]
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='cpg_config', default_value=default_cpg_config,
                              description='Absolute path to CPG config file'),
        gait_planner_node 
    ])
