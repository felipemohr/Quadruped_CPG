import os
import launch
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Quadruped packages
    quadruped_teleop_pkg = FindPackageShare('quadruped_teleop').find('quadruped_teleop')
    quadruped_kinematics_pkg = FindPackageShare('quadruped_kinematics').find('quadruped_kinematics')
    quadruped_gait_planner_pkg = FindPackageShare('quadruped_gait_planner').find('quadruped_gait_planner')

    # Quadruped Teleop configs
    default_joystick_config = os.path.join(quadruped_teleop_pkg, 'config', 'dualsense_joystick.yaml')
    joystick_config = LaunchConfiguration('joystick_config', default=default_joystick_config)

    default_quadruped_limits_config = os.path.join(quadruped_teleop_pkg, 'config', 'go1_limits.yaml')
    quadruped_limits_config = LaunchConfiguration('quadruped_limits', default=default_quadruped_limits_config)

    # Quadruped Gait Planner configs
    default_cpg_config = os.path.join(quadruped_gait_planner_pkg, 'config', 'cpg.yaml')
    cpg_config = LaunchConfiguration('cpg_config', default=default_cpg_config)

    # Quadruped Kinematics configs
    default_quadruped_config = os.path.join(quadruped_kinematics_pkg, 'config', 'go1.yaml')
    quadruped_config = LaunchConfiguration('quadruped_config', default=default_quadruped_config)

    # Quadruped launchers
    joy_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(quadruped_teleop_pkg, 'launch', 'joy_teleop.launch.py')),
        launch_arguments={'joystick_config': joystick_config,
                          'quadruped_limits': quadruped_limits_config}.items(),
    )

    gait_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(quadruped_gait_planner_pkg, 'launch', 'quadruped_gait_planner.launch.py')),
        launch_arguments={'cpg_config': cpg_config}.items(),
    )

    kinematics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(quadruped_kinematics_pkg, 'launch', 'quadruped_kinematics.launch.py')),
        launch_arguments={'quadruped_config': quadruped_config}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='quadruped_config', default_value=default_quadruped_config,
                              description='Absolute path to quadruped config file'),
        DeclareLaunchArgument(name='cpg_config', default_value=default_cpg_config,
                              description='Absolute path to CPG config file'),
        DeclareLaunchArgument(name='joystick_config', default_value=default_joystick_config,
                              description='Absolute path to joystick config file'),
        DeclareLaunchArgument(name='quadruped_limits', default_value=default_quadruped_limits_config,
                              description='Absolute path to quadruped limits config file'),
        kinematics,
        gait_planner,
        joy_teleop,
    ])
