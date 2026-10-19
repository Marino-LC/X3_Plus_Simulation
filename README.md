# Universidad Nacional Autónoma de México

# Facultad de Ingeniería  

# Mechatronics Research Group

**Responsable del proyecto:** Marino López Cruz

**Supervisión:** Doctor Victor Javier Gonzaléz Villela y  Mtro. Erik Peña Medina

**Alumnos colaboradores:**

* José Eduardo Garduño Silverio

* Emmanuel Ricardo Oviedo Mendoza

* Diego Alejandro Segura Soriano

* Lair Abraham Santos Cruz

## Estructura de los Branches

La estructura del repositorio contempla diversas ramas derivadas de `main`, cada una enfocada en un aspecto específico del proyecto:

* **main:** Contiene el desarrollo principal y base del proyecto.

* **Nav2:** Implementación y configuración de Nav2 para la navegación autónoma.

* **MuJoCo_simulation:** Entorno de simulación utilizando MuJoCo.

* **VLA_models:** Modelos de Vision-Language-Action (VLA) integrados.

* **Ruedas_mechanum:** Ampliación de la tesis que incluye una comparativa de rendimiento y simulación entre MuJoCo y Gazebo.

## Descripción General

Este repositorio contiene la simulación de un robot manipulador móvil omnidireccional utilizando ROS 2. Incluye múltiples paquetes que describen y demuestran la funcionalidad del robot tanto de manera independiente como en conjunto. El proyecto está estructurado para facilitar el desarrollo y la prueba de varios componentes del robot. 

## Estructura del Repositorio 

El repositorio está organizado en varios paquetes, cada uno con un propósito específico. A continuación se presenta una breve descripción de cada paquete y su contenido:

### 1. dofbot_bringup

Contiene archivos de lanzamiento y configuraciones para poner en marcha el robot dofbot.
**Archivos Clave:**

* `launch/dofbot_trajectory_controller.launch.py`: Archivo de lanzamiento para el controlador de trayectoria. Este script lanza el dofbot en Gazebo y RViz con un controlador para la trayectoria del dofbot y otro para el gripper.

* `rviz/dofbot_trajectory_rviz.rviz`: Archivo de configuración de RViz para visualizar el robot.

* `src/`: Archivos para publicar en los controladores. Los scripts `dofbot_gripper_test.py` y `dofbot_trajectory_test.py` publican trayectorias predefinidas para el gripper y el brazo del dofbot respectivamente.

* `world/test_world.world`: Archivo de mundo para simulación en Gazebo.

### 2. dofbot_description

Proporciona la descripción del robot dofbot, incluyendo su URDF y modelos visuales.
**Archivos Clave:**

* `config/dofbot_trajectory_controller.yaml`: Archivo de configuración para los controladores del dofbot.

* `launch/dofbot_arm.launch.xml`: Permite lanzar el dofbot para su visualización en RViz y el control de sus juntas con el `joint_state_publisher_gui`.

* `meshes/`: Contiene archivos STL para los componentes visuales del robot.

* `rviz/dofbot_arm_rviz.rviz`: Configuración de RViz para el brazo del dofbot.

* `urdf/`: Contiene archivos Xacro para generar el URDF del robot.

### 3. omni_bringup

Contiene archivos de lanzamiento y configuraciones para el robot omnidireccional.
**Archivos Clave:**

* `launch/simple_velocity_controller.launch.py`: Archivo de lanzamiento en Gazebo y RViz para el controlador de velocidad del robot móvil.

* `rviz/omni_velocity_rviz.rviz`: Configuración de RViz para el robot Omni.

* `world/Mundo_mesa_y_cajas.world`: Archivo de mundo para simulación en Gazebo.

### 4. omni_description

Proporciona la descripción del robot Omni, incluyendo su URDF y modelos visuales.
**Archivos Clave:**

* `config/omni_velocity_controller.yaml`: Archivo con los parámetros de configuración del control de velocidad para el robot omnidireccional.

* `launch/omni_display.launch.xml`: Lanza una instancia en RViz para visualizar el robot omni y el `joint_state_publisher_gui` para controlar las juntas.

* `meshes/`: Contiene archivos STL para los componentes visuales del robot.

* `rviz/omni_rviz.rviz`: Archivo con la configuración de RViz para su visualización.

* `urdf/`: Contiene archivos Xacro para generar el URDF del robot.

### 5. omni_dofbot_bringup

Contiene archivos de lanzamiento y configuraciones para el robot Omni Dofbot.
**Archivos Clave:**

* `launch/omni_dofbot_controller.launch.py`: Archivo de lanzamiento para el controlador del omni_dofbot. Lanza los controladores para el dofbot y el móvil. El robot se visualiza en RViz y Gazebo con sus sensores.

* `rviz/omni_dofbot_trayectory_rviz.rviz`: Configuración de RViz para el Omni Dofbot.

### 6. omni_dofbot_description

Proporciona la descripción del robot omni_dofbot, incluyendo su URDF y modelos visuales.
**Archivos Clave:**

* `config/omni_dofbot_controller.yaml`: Contiene los parámetros de los controladores implementados en el omni_dofbot.

* `launch/omni_dofbot_display.launch.xml`: Archivo que lanza una instancia para la visualización del robot en RViz y el nodo `joint_state_publisher_gui`.

* `meshes/`: Contiene archivos STL para los componentes visuales del Omni Dofbot.

* `urdf/`: Contiene archivos Xacro para generar el URDF del omni_dofbot.

## ¿Cómo instalar el paquete?

**Paso 1. Agregar el repositorio:**
Crea un espacio de trabajo destinado a este repositorio dentro de tu carpeta de ROS 2, y dentro de él una carpeta `src`:

```
$ mkdir -p ~/ROS2Dev/omni_dofbot/src

```

Muévete a la carpeta `src` e inicia git para añadir el repositorio:

```
$ cd ~/ROS2Dev/omni_dofbot/src
$ git init
$ git remote add origin https://github.com/arrg-mx/ROSX3_Plus_ROSX3.git

```

Haz un pull para tener de manera local los archivos del repositorio:

```
$ git pull origin

```

Finalmente, cambia a la branch `humble` (o la branch correspondiente a tu versión de ROS 2):

```
$ git checkout humble 

```

**Paso 2. Compilar los paquetes y hacerles un source:**
Vuelve al nivel de la carpeta del espacio de trabajo:

```
$ cd ..

```

Compila todos los paquetes:

```
$ colcon build

```

Realiza un source a la carpeta de instalación:

```
$ source install/setup.bash

```

## Instrucciones Omni Dofbot (Prueba de paquetes)

Para probar la integración completa del Omni Dofbot:

```
$ ros2 launch omni_dofbot_bringup omni_dofbot_controller.launch.py 

```

Una vez spawneado el robot en Gazebo y RViz, puedes usar los scripts de los controladores en otra terminal. **Importante:** No olvides volver a hacer source del workspace en la nueva terminal:

```
$ source install/setup.bash
$ ros2 run dofbot_bringup dofbot_trajectory_test.py 

```

## Notas

### Visualización en Gazebo

Si se presentan problemas al visualizar las mallas en Gazebo, intenta una de las siguientes soluciones:

**Solución 1:**
Modifica la ruta de `gazebo_model_path` que se encuentra en los metadatos del archivo `package.xml` de cada paquete de description (debes hacerlo para los 3 paquetes de description):

```
  <export>
    <build_type>ament_cmake</build_type>
    <gazebo_ros gazebo_model_path = "ruta/del/directorio/share/del/paquete/description"/>
  </export>

```

**Solución 2:**
Copiar directamente las 3 carpetas de `description` al directorio `~/.gazebo/models`.

### Estado Actual de la Migración a Gazebo Harmonic (ROS 2 Jazzy)

**Lo que se logró:**

* **Entorno limpio:** Se creó un nuevo workspace (`tesisx3_plus`) migrando los paquetes `omni_description`, `dofbot_description` y `dofbot_bringup` a la nueva estructura de `ament_cmake`.

* **Lanzamiento base:** El archivo `gazebo_harmonic.launch.py` levanta con éxito el mundo en Gazebo (`ros_gz_sim`) y el `robot_state_publisher`.

* **Mallas y Geometría:** Se solucionaron los errores de rutas. Gazebo ya localiza y renderiza correctamente todas las mallas (`.stl`) del chasis omnidireccional y del brazo Dofbot usando el prefijo `file://$(find ...)`.

* **URDF de Control:** Se actualizó la etiqueta de hardware a `<plugin>gz_ros2_control/GazeboSimSystem</plugin>` y el plugin de simulación a `gz_ros2_control::GazeboSimROS2ControlPlugin`.

**Fallos Pendientes:**
El simulador arranca y lanza el robot, pero falla inmediatamente al intentar cargar el `controller_manager` con un `Segmentation fault`.
Traza del error principal:

```
[gazebo-2] [INFO] [gz_ros_control]: Loading joint: front_left_joint
[gazebo-2] [INFO] [gz_ros_control]: 	State:
[gazebo-2] [INFO] [gz_ros_control]: 		 position
[gazebo-2] [INFO] [gz_ros_control]: 		 position
[gazebo-2] Segmentation fault (Invalid permissions for mapped object [0x7bd4c8086010])

```
