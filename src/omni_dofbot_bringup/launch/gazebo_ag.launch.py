"""Gazebo + nodo cinemático/PID + odometría: todo lo que necesita el AG.

Equivalente en Gazebo a `omni_dofbot_mujoco/mujoco_ag.launch.py`. Después, en
otra terminal:  ros2 run omni_dofbot_bringup ag_motion_tests.py

  ros2 launch omni_dofbot_bringup gazebo_ag.launch.py
  ros2 launch omni_dofbot_bringup gazebo_ag.launch.py gui:=false rviz:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_time = {'use_sim_time': True}
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('omni_dofbot_bringup'),
            'launch', 'omni_dofbot_controller.launch.py')),
        launch_arguments={
            'world': 'arena_pid_tuning.world',
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
        }.items())
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        gazebo,
        Node(package='omni_dofbot_bringup', executable='mecanum_kinematic_node.py',
             output='screen', parameters=[sim_time]),
        Node(package='omni_dofbot_bringup', executable='Mecanum_odometry_node.py',
             output='screen', parameters=[sim_time]),
    ])
