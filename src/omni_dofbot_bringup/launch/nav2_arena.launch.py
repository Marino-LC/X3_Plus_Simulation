"""
nav2_arena.launch.py — pila de navegación Nav2 para la arena.

Argumentos:
  localization:=amcl|slam   amcl (defecto): map_server con el mapa de referencia
                            + AMCL. slam: slam_toolbox construye el mapa en
                            línea (launch/slam_toolbox.launch.py) y publica
                            map -> odom; no se lanzan map_server ni AMCL.
  use_collision_monitor:=true|false
                            Inserta collision_monitor al final de la cadena de
                            velocidad (defecto true).

Cadena de velocidad (la misma que nav2_bringup en Jazzy):
  controller_server / behavior_server --cmd_vel_nav--> velocity_smoother
    --cmd_vel_smoothed--> collision_monitor --cmd_vel--> mecanum_kinematic_node
Con use_collision_monitor:=false, velocity_smoother publica directo en cmd_vel.
(Antes el controlador publicaba en cmd_vel y el monitor también, formando un ciclo
smoother -> monitor -> cmd_vel -> smoother.)

Normalmente se lanza desde arena_nav.launch.py, pero puede lanzarse solo con la
simulación ya corriendo.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    pkg_dir = get_package_share_directory('omni_dofbot_bringup')
    map_file = os.path.join(pkg_dir, 'maps', 'arena_map.yaml')
    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    loc = LaunchConfiguration('localization').perform(context).lower()
    use_cm = LaunchConfiguration('use_collision_monitor').perform(context).lower() == 'true'
    if loc not in ('amcl', 'slam'):
        raise RuntimeError(f"localization debe ser 'amcl' o 'slam', no '{loc}'")

    sim = {'use_sim_time': True}
    to_nav = [('cmd_vel', 'cmd_vel_nav')]
    actions = []

    # 1. Localización
    if loc == 'amcl':
        # map_server: sirve el mapa de referencia (marco map == mundo de Gazebo)
        actions.append(Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            # ¡IMPORTANTE! params_file va primero, el diccionario va segundo
            parameters=[params_file, {'yaml_filename': map_file, **sim}],
            output='screen'))
        # AMCL: localización Monte Carlo sobre el mapa estático
        actions.append(Node(
            package='nav2_amcl', executable='amcl', name='amcl',
            parameters=[params_file, sim], output='screen'))
    else:
        # slam_toolbox: publica /map y map -> odom; gestiona su propio ciclo de vida
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'slam_toolbox.launch.py'))))

    # 2. Planificador global. Con SLAM el mapa está incompleto al inicio: se
    #    permite planificar por celdas desconocidas.
    planner_overrides = {'GridBased.allow_unknown': True} if loc == 'slam' else {}
    actions += [
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[params_file, sim, planner_overrides], output='screen'),
        # 3. Controlador local (DWB)
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', parameters=[params_file, sim],
             remappings=to_nav, output='screen'),
        # 4. Suavizador de rutas
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server',
             parameters=[params_file, sim], output='screen'),
        # 5. Comportamientos de recuperación (spin, backup, ...)
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             parameters=[params_file, sim], remappings=to_nav, output='screen'),
        # 6. Árbol de comportamiento
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[params_file, sim], output='screen'),
        # 7. Seguidor de waypoints (no lo usa el experimento, se mantiene disponible)
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', parameters=[params_file, sim], output='screen'),
        # 8. Suavizador de velocidad
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', parameters=[params_file, sim],
             remappings=to_nav + ([] if use_cm else [('cmd_vel_smoothed', 'cmd_vel')]),
             output='screen'),
    ]
    # 9. Monitor de colisiones (último eslabón antes de cmd_vel)
    if use_cm:
        actions.append(Node(
            package='nav2_collision_monitor', executable='collision_monitor',
            name='collision_monitor', parameters=[params_file, sim], output='screen'))

    # 10. Gestor de ciclo de vida
    lifecycle_nodes = (['map_server', 'amcl'] if loc == 'amcl' else []) + [
        'planner_server', 'controller_server', 'smoother_server', 'behavior_server',
        'bt_navigator', 'waypoint_follower', 'velocity_smoother'] + (
        ['collision_monitor'] if use_cm else [])
    actions.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': lifecycle_nodes, 'bond_timeout': 10.0}],
        output='screen'))

    # NOTA: aquí NO se publica ningún TF base_link <-> base_footprint.
    # El URDF ya define base_footprint -> base_link (base_joint) y lo publica
    # robot_state_publisher; un estático adicional creaba un ciclo en el árbol TF.
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('localization', default_value='amcl',
                              description='amcl | slam'),
        DeclareLaunchArgument('use_collision_monitor', default_value='true',
                              description='Incluir nav2_collision_monitor en la cadena de cmd_vel'),
        OpaqueFunction(function=_nodes),
    ])
