"""
slam_toolbox.launch.py — slam_toolbox (modo mapping asíncrono) con los
parámetros de config/slam_toolbox.yaml.

Usa el launch oficial online_async_launch.py, que en Jazzy configura y activa
el nodo (es un nodo con ciclo de vida). Publica /map y el TF map -> odom.

Solo (para construir un mapa a mano, con la simulación corriendo):
  ros2 launch omni_dofbot_bringup slam_toolbox.launch.py
  ros2 run nav2_map_server map_saver_cli -f mi_mapa --ros-args -p use_sim_time:=true

Dentro del experimento lo incluye nav2_arena.launch.py con localization:=slam.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('omni_dofbot_bringup')
    params = LaunchConfiguration('slam_params_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml'),
            description='Parámetros de slam_toolbox'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py')),
            launch_arguments={'slam_params_file': params,
                              'use_sim_time': 'true'}.items()),
    ])
