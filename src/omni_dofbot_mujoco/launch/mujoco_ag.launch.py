"""Sustituye a omni_dofbot_controller.launch.py (Gazebo) por MuJoCo.

Lanza el puente + el nodo cinemático/PID + la odometría. Después, en otra
terminal:  ros2 run omni_dofbot_bringup ag_motion_tests.py

  ros2 launch omni_dofbot_mujoco mujoco_ag.launch.py viewer:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_time = {'use_sim_time': True}
    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='false'),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('real_time_factor', default_value='1.0'),
        # Publicar /clock cada N pasos de 1 ms. Subirlo a 2-5 si el factor de
        # tiempo real medido queda < 0.9 (el AG descarta esas evaluaciones).
        DeclareLaunchArgument('clock_decimation', default_value='1'),
        Node(package='omni_dofbot_mujoco', executable='mujoco_bridge_node.py',
             output='screen',
             parameters=[{
                 'viewer': ParameterValue(LaunchConfiguration('viewer'), value_type=bool),
                 'model_path': ParameterValue(LaunchConfiguration('model_path'),
                                              value_type=str),
                 'real_time_factor': ParameterValue(
                     LaunchConfiguration('real_time_factor'), value_type=float),
                 'clock_decimation': ParameterValue(
                     LaunchConfiguration('clock_decimation'), value_type=int),
             }]),
        Node(package='omni_dofbot_bringup', executable='mecanum_kinematic_node.py',
             output='screen', parameters=[sim_time]),
        Node(package='omni_dofbot_bringup', executable='Mecanum_odometry_node.py',
             output='screen', parameters=[sim_time]),
    ])
