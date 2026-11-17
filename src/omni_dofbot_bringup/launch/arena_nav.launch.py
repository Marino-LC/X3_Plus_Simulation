"""
arena_nav.launch.py — launch único del experimento de navegación.

Levanta, en una sola terminal:
  1. omni_dofbot_controller.launch.py: Gazebo con arena_world.world, robot,
     controladores, puentes ROS-Gazebo, nodo cinemático (PID con las ganancias
     del AG) y nodo de odometría.
  2. Tras `nav2_delay` segundos: nav2_arena.launch.py con la localización
     elegida (AMCL sobre el mapa de referencia o slam_toolbox).

Argumentos:
  localization:=amcl|slam        (defecto amcl)
  use_collision_monitor:=true|false (defecto true)
  gui:=true|false                ventana de Gazebo (defecto true; false para
                                 medir costo computacional sin render de GUI)
  rviz:=true|false               (defecto true)
  nav2_delay:=<s>                espera antes de Nav2, en segundos reales
                                 (defecto 15; súbelo si la PC es lenta)

Ejemplos:
  ros2 launch omni_dofbot_bringup arena_nav.launch.py
  ros2 launch omni_dofbot_bringup arena_nav.launch.py localization:=slam gui:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('omni_dofbot_bringup'), 'launch')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'omni_dofbot_controller.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui'),
                          'rviz': LaunchConfiguration('rviz')}.items())

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2_arena.launch.py')),
        launch_arguments={
            'localization': LaunchConfiguration('localization'),
            'use_collision_monitor': LaunchConfiguration('use_collision_monitor'),
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument('localization', default_value='amcl', description='amcl | slam'),
        DeclareLaunchArgument('use_collision_monitor', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('nav2_delay', default_value='15.0'),
        sim,
        # Nav2 espera a que el robot exista, los controladores estén activos y
        # la odometría publique odom -> base_footprint.
        TimerAction(period=LaunchConfiguration('nav2_delay'), actions=[nav2]),
    ])
