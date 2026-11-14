"""Simulación para la medición de error de pose de objetos (sin Nav2).

Lanza Gazebo con el mundo de UN ensayo, el robot con el publicador de pose
real, los puentes de cámara/pose/teletransporte, los controladores y el
publicador de la verdad de referencia. No lanza Nav2, AMCL ni odometría.

  ros2 launch vla_pose_eval vla_pose_eval.launch.py trial_dir:=/ruta/S1_seed0000

Después, en otra terminal:
  ros2 run vla_pose_eval recon_pass --ros-args -p trial_dir:=/ruta/S1_seed0000
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD_NAME = 'vla_pose_arena'     # eval.yaml: world_name
ROBOT_NAME = 'omni_dofbot'        # eval.yaml: robot_name


def generate_launch_description():
    pkg_desc = get_package_share_directory('omni_dofbot_description')
    urdf = os.path.join(pkg_desc, 'urdf', 'omni_dofbot_vla.xacro')
    trial_dir = LaunchConfiguration('trial_dir')
    world = PathJoinSubstitution([trial_dir, 'scene.world'])

    robot_description = ParameterValue(Command(['xacro ', urdf]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('gz_extra_args'), ' ', world]}.items())

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', ROBOT_NAME,
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.1', '-Y', '1.570796'])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/mono_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/mono_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Pose real del robot (PosePublisher en omni_dofbot_vla.xacro).
            '/model/%s/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose' % ROBOT_NAME,
            # Teletransporte entre vistas.
            '/world/%s/set_pose@ros_gz_interfaces/srv/SetEntityPose' % WORLD_NAME,
        ])

    def spawner(name, *extra):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '30', *extra])

    # Mismo orden que omni_dofbot_controller.launch.py (evita el bloqueo por el
    # error de mimic): ruedas inactivas -> brazo -> gripper -> JSB -> activar ruedas.
    wheels = spawner('wheel_velocity_controller', '--inactive')
    arm = spawner('dofbot_trajectory_controller')
    gripper = spawner('dofbot_gripper_controller')
    jsb = spawner('joint_state_broadcaster')
    activate_wheels = ExecuteProcess(
        cmd=['ros2', 'control', 'switch_controllers', '--activate',
             'wheel_velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen')

    def after(prev, nxt):
        return RegisterEventHandler(OnProcessExit(target_action=prev, on_exit=[nxt]))

    return LaunchDescription([
        DeclareLaunchArgument('trial_dir', description='Ensayo con scene.world y scene_gt.yaml'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('gz_extra_args', default_value='',
                              description="p. ej. '-s' para Gazebo sin interfaz gráfica"),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               os.path.abspath(os.path.join(pkg_desc, '..'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
        gazebo, spawn, bridge,
        Node(package='vla_pose_eval', executable='ground_truth_publisher', output='screen',
             parameters=[{'trial_dir': trial_dir}]),
        Node(package='rviz2', executable='rviz2', parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        after(spawn, wheels), after(wheels, arm), after(arm, gripper), after(gripper, jsb),
        after(jsb, activate_wheels),
    ])
