import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('omni_dofbot_bringup')
    map_file = os.path.join(pkg_dir, 'maps', 'arena_map.yaml')
    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # Añadimos behavior_server a la lista oficial.
    # Se agregaron los nodos de navegación faltantes a la lista de nodos de ciclo de vida.
    lifecycle_nodes = [
                        'map_server',
                        'amcl', 
                        'planner_server',
                        'controller_server', 
                        'smoother_server',
                        'behavior_server', 
                        'bt_navigator', 
                        'waypoint_follower',
                        'velocity_smoother'
                        #'collision_monitor'
                    ]

    return LaunchDescription([

        # 1. Servidor de Mapas ('map_server')
        # Este nodo se encarga de cargar y servir el mapa del entorno al resto de 
        # nodos de navegación. Recibe un archivo .yaml que describe el mapa y lo
        # convierte para que otros nodos accedan a él.

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            # ¡IMPORTANTE! params_file va primero, el diccionario va segundo
            parameters=[params_file, {'yaml_filename': map_file, 'use_sim_time': True}],
            output='screen'
        ),

        # 2. Implementación de nodo ACML (Adaptive Monte Carlo Localization)
        # Este nodo se encarga de localizar el robot en el mapa utilizando un algoritmo 
        # de muestreo adaptativo. Recibe datos del sensor y actualiza la estimación de 
        # la posición del robot.

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[params_file, {'use_sim_time': True}],
                    output='screen'
        ),

        # 3. El Administrador de Vida
        # Este nodo se encarga de encender todos los nodos anteriores al mismo tiempo
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': lifecycle_nodes, 'bond_timeout':10.0}],
            output='screen'
        ),

        # 4. Planificador Global (planner_server)
        # Este nodo se encarga de la generación de rutas globales desde la posición
        # actual del robot hacia otra posición objetivo dentro del mapa. 

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[params_file],
            output='screen'
        ),

        # 5. Controlador Local (PID/Seguimiento - controller_server) 
        # Este nodo se encarga de generar los comandos de velocidad para el robot
        # en base a la ruta generada por el planificador global y la posición actual.

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[params_file],
            output='screen'
        ),

        # 6. (smoother server)

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            parameters=[params_file],
            output='screen'
        ),

        # 7. Comportamientos (behavior_server)
        # Este nodo se encarga de ejecutar comportamientos predefinidos, como
        # seguir una ruta, evitar obstáculos, etc.

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[params_file],
            output='screen'
        ),

        # 8. Navegador de Árbol de Comportamiento (bt_navigator)
        # Este nodo se encarga de ejecutar un árbol de comportamiento con el fin de
        #definir la lógica de navegación del robot.

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[params_file],
            output='screen'
        ),

        # 9. Seguidor de puntos de ruta (waypoint_follower)
        # Este nodo se encarga de seguir una serie de puntos de ruta predefinidos
        
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            parameters=[params_file],
            output='screen'
        ),

        # 10. Suavizador de velocidad (velocity_smoother)
        # Este nodo se encarga de suavizar los comandos de velocidad generados por el
        # controlador local para evitar movimientos bruscos del robot.

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            parameters=[params_file],
            output='screen'
        ),

        # 11. Monitor de colisiones (collision_monitor)
        # Este nodo se encarga de monitorear la distancia a los obstáculos y generar
        # alertas si el robot se acerca demasiado a ellos.

        #Node(
        #    package='nav2_collision_monitor',
        #    executable='collision_monitor',
        #    name='collision_monitor',
        #    parameters=[params_file],
        #    output='screen'
        #),

        # 12. Relacionar base_link con base_footprint
        # Este nodo se encarga de publicar una transformación estática entre los marcos
        # de referencia 'base_link' y 'base_footprint'. Esto es útil para mantener 
        # la coherencia entre los diferentes marcos de referencia del robot.
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_tf',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'base_footprint'
            ],
            output='screen'
        )
    ])