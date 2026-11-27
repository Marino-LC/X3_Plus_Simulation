# omni dofbot
## Descripción General
Este repositorio contiene la implementación de un robot manipulador móvil omnidireccional utilizando ROS2. Incluye múltiples paquetes que describen y demuestran la funcionalidad del robot tanto de manera independiente como en conjunto. El proyecto está estructurado para facilitar el desarrollo y la prueba de varios componentes del robot.

## Estructura del Repositorio
El repositorio está organizado en varios paquetes, cada uno con un propósito específico. A continuación se presenta una breve descripción de cada paquete y su contenido:

### 1. dofbot_bringup
Contiene archivos de lanzamiento y configuraciones para poner en marcha el robot dofbot.
Archivos Clave:
- **launch/dofbot_trajectory_controller.launch.py**: Archivo de lanzamiento para el controlador de trayectoria, este script lanza el dofbot en gazebo y rviz con un controlador para la trayectoria del dofbot y otro para el gripper.
- **rviz/dofbot_trajectory_rviz.rviz**: Archivo de configuración de rviz para visualizar el robot.
- **src/**: Archivos para publicar en los controladores, los scripts 'dofbot_gripper_test.py' y 'dofbot_trajectory_test.py' publican trayectorias predefinidas para el gripper y el brazo del dofbot respectivamente.
- **world/test_world.world**: Archivo de mundo para simulación en Gazebo.

### 2. dofbot_description
Proporciona la descripción del robot dofbot, incluyendo su URDF y modelos visuales.
Archivos Clave:
- **config/dofbot_trajectory_controller.yaml**: Archivo de configuración para los controladores del dofbot.
- **launch/dofbot_arm.launch.xml**: Este archivo permite lanzar el dofbot para su visualizaciion en rviz y el control de sus juntas con el 'joint_state_publisher_gui'
- **meshes/**: Contiene archivos STL para los componentes visuales del robot.
- **rviz/dofbot_arm_rviz.rviz**: Configuración de rviz para el brazo del dofbot.
- **urdf/**: Contiene archivos Xacro para generar el URDF del robot.


### 3. omni_bringup
Contiene archivos de lanzamiento y configuraciones para el robot omnidireccional.
Archivos Clave:
- **launch/simple_velocity_controller.launch.py**: Archivo de lanzamiento en gazebo y rviz para el controlador de velocidad del robot móvil.
- **rviz/omni_velocity_rviz.rviz**: Configuración de RViz para el robot Omni.
- **world/Mundo_mesa_y_cajas.world**: Archivo de mundo para simulación en Gazebo.

### 4. omni_description
Proporciona la descripción del robot Omni, incluyendo su URDF y modelos visuales.
Archivos Clave:
- **config/omni_velocity_controller.yaml**: Archivo con los parámetros de configuración del control de velocidad para el robot omnidireccional.
- **launch/omni_display.launch.xml**: Este archivo lanza una instancia en rviz para visualizar el robot omni y el joint_state_publisher_gui para controlar las juntas.
- **meshes/**: Contiene archivos STL para los componentes visuales del robot.
- **rviz/omni_rviz.rviz**: Archivo con la configuracion de rviz para su visualizacion.
- **urdf/**: Contiene archivos Xacro para generar el URDF del robot. `mecanum_wheel.xacro` define la rueda mecanum de 9 rodillos libres, con los parámetros homologados con MuJoCo.

### 5. omni_dofbot_bringup
Contiene archivos de lanzamiento y configuraciones para el robot Omni Dofbot.
Archivos Clave:
- **launch/omni_dofbot_controller.launch.py**: Archivo de lanzamiento para el controlador del omni_dofbot, lanza los controladores para el dofbot y el movil, el robot se visualiza en rviz y gazebo. Argumentos: `world`, `gui`, `rviz`.
- **launch/gazebo_ag.launch.py**: Gazebo + nodo cinemático + odometría, listo para correr el AG.
- **world/arena_pid_tuning.world**: Arena del AG, homologada con la de MuJoCo.
- **rviz/omni_dofbot_trayectory_rviz.rviz**: Configuración de rviz para el Omni Dofbot.
- **src/ag_motion_tests.py**: Sintonización de las ganancias PID de la base mecanum mediante Algoritmo Genético. Ver [omni_dofbot_bringup/README.md](omni_dofbot_bringup/README.md) para la explicación completa del experimento, las pruebas y la estructura del AG.

### 6. omni_dofbot_description
Proporciona la descripción del robot omni_dofbot, incluyendo su URDF y modelos visuales.
Archivos Clave:
- **config/omni_dofbot_controller.yaml**: Este archivo contiene los paramentros de los controladores implementados en el omni_dofbot.
- **launch/omni_dofbot_display.launch.xml**: Archivo que lanza una instancia para la visualizacion del robot en rviz y el nodo joint_state_publisher_gui.
- **meshes/**: Contiene archivos STL para los componentes visuales del Omni Dofbot.
- **urdf/**: Contiene archivos Xacro para generar el URDF del omni_dofbot.

### 7. omni_dofbot_mujoco
Modelo MuJoCo del omni_dofbot con rodillos libres y puente ROS 2 con las mismas interfaces que Gazebo, para correr el mismo AG en los dos simuladores. Ver el [README de la raíz](../README.md).
Archivos Clave:
- **launch/mujoco_ag.launch.py**: Puente MuJoCo + nodo cinemático + odometría, listo para correr el AG.
- **omni_dofbot_mujoco/params.py**: Parámetros físicos (fuente única; Gazebo los copia).
- **scripts/run_ag_mujoco.py**: AG sobre MuJoCo sin ROS, en paso fijo.

## Instalación y ejecución

Ver el [README de la raíz](../README.md): instalación con ROS 2 Jazzy + Gazebo Harmonic + MuJoCo, y cómo correr el AG en Gazebo o en MuJoCo.

Ejemplo rápido (desde la raíz del workspace):

```shell
colcon build --symlink-install
```

```shell
source install/setup.bash
```

```shell
ros2 launch omni_dofbot_bringup omni_dofbot_controller.launch.py
```

Con el robot en Gazebo, en otra terminal (con `source install/setup.bash`):

```shell
ros2 run dofbot_bringup dofbot_trajectory_test.py
```
