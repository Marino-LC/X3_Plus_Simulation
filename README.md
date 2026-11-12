# X3 Plus Simulation — rama `Nav2`

Simulación del manipulador móvil omnidireccional **X3 Plus + Dofbot** en ROS 2 Jazzy y Gazebo Harmonic, con navegación autónoma (Nav2) sobre una arena de 2.7 × 2.7 m.

Esta rama prepara los experimentos del **Artículo 1 (navegación y SLAM)**: N ≥ 20 corridas por simulador sobre una ruta de 6 waypoints, comparando Gazebo contra MuJoCo.

> **Estado de validación.** Los cambios del commit `1a41b8d` (mapa, TF, verdad de terreno, colisiones y ejecutor de corridas) se revisaron solo de forma estática: sintaxis de Python, XML y YAML. **Todavía no se han ejecutado en ROS 2 / Gazebo.** La primera prueba debe seguir la sección [Verificación de la primera prueba](#5-verificación-de-la-primera-prueba).

La descripción de los paquetes de descripción del robot (`omni_description`, `dofbot_description`, etc.) está en [`src/README.md`](src/README.md); partes de ese documento describen un estado anterior (Humble, error de `controller_manager`).

---

## 1. Qué hay en la rama

Todo lo relevante para navegación está en `src/omni_dofbot_bringup`.

| Componente | Archivo | Función |
|---|---|---|
| Simulación | `launch/omni_dofbot_controller.launch.py` | Gazebo con `arena_world.world`, robot en (0, −0.91, yaw π/2), controladores de ruedas, brazo y pinza, y puentes ROS–Gazebo |
| Control de ruedas | `src/mecanum_kinematic_node.py` | `/cmd_vel` → velocidad por rueda, con PID incremental a 100 Hz que imita el firmware (PWM, ganancia de motor, retardo, umbral de arranque) |
| Odometría | `src/Mecanum_odometry_node.py` | Integra `/joint_states`; publica `/odom` y el TF `odom → base_footprint`, arrancando en (0, 0, π/2) |
| Navegación | `launch/nav2_arena.launch.py`, `config/nav2_params.yaml` | map_server, AMCL (modelo omnidireccional), NavFn, DWB con `vy = 0`, behaviors, BT navigator |
| Ruta | `config/waypoints.yaml` | WP1 (spawn) y WP2–WP6, en coordenadas del mundo |
| Mapa | `maps/generate_map.py`, `maps/arena_map.*` | Mapa de referencia alineado con el mundo (`map` = mundo de Gazebo) |
| Ejecutor de corridas | `src/waypoints_nav.py` | Repite la ruta N veces y registra los resultados |
| Verdad de terreno | `omni_dofbot_description/urdf/omni_dofbot_controllers.xacro` | Plugin `OdometryPublisher` → `/ground_truth/odom` (pose real en el marco del mundo) |
| Colisiones | `world/arena_world.world` | Sensores de contacto en las 4 paredes, las 2 cajas y el pedestal → `/contacts/<modelo>` |

### Qué hace el ejecutor de corridas

Por cada corrida, `waypoints_nav.py`:

1. Teletransporta el robot al spawn y reinicia odometría, PID, pose de AMCL (`/initialpose`) y costmaps. Comprueba con la pose real que el reset funcionó.
2. Envía WP2–WP6 uno por uno con `NavigateToPose`, de modo que el robot se detiene en cada waypoint.
3. En cada waypoint compara la **pose real** contra el objetivo (0.08 m y 0.15 rad por defecto).
4. Cuenta colisiones (solo contactos donde participa el robot) y recuperaciones de Nav2.

Una corrida es **exitosa** si Nav2 completa los cinco waypoints, la pose real queda dentro de tolerancia en cada uno y no hubo ninguna colisión.

---

## 2. Requisitos

- Ubuntu 24.04 con **ROS 2 Jazzy** y **Gazebo Harmonic**.
- Paquetes de ROS (lista derivada de lo que importa el código; ajusta si tu instalación ya los trae):

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-rviz2 ros-jazzy-tf2-ros
```

- Python: `numpy` y `pyyaml` para navegación. Solo para los scripts de sintonización del PID (`ag_motion_tests.py`): `deap`, `matplotlib` y `plotly`.

---

## 3. Inicializar el entorno

El repositorio ya tiene la carpeta `src/`, así que **la raíz del repositorio es el workspace**.

```bash
git clone -b Nav2 https://github.com/Marino-LC/X3_Plus_Simulation.git
```

```bash
cd X3_Plus_Simulation
```

```bash
source /opt/ros/jazzy/setup.bash
```

```bash
colcon build --symlink-install
```

```bash
source install/setup.bash
```

Si ya tenías el workspace compilado, vuelve a ejecutar `colcon build` después de actualizar la rama: el mapa, el mundo, los waypoints y el xacro se instalan en `install/` y los nodos leen esa copia.

---

## 4. Ejecutar una prueba

Abre cinco terminales. En **cada una** ejecuta primero `source /opt/ros/jazzy/setup.bash` y `source install/setup.bash` desde la raíz del workspace.

**Terminal 1 — simulación.** Espera a que el robot aparezca y a que termine la activación de `wheel_velocity_controller`.

```bash
ros2 launch omni_dofbot_bringup omni_dofbot_controller.launch.py
```

**Terminal 2 — control de ruedas.**

```bash
ros2 run omni_dofbot_bringup mecanum_kinematic_node.py
```

**Terminal 3 — odometría.**

```bash
ros2 run omni_dofbot_bringup Mecanum_odometry_node.py
```

**Terminal 4 — Nav2.** Espera el mensaje de que los nodos gestionados están activos.

```bash
ros2 launch omni_dofbot_bringup nav2_arena.launch.py
```

**Terminal 5 — corridas.** Empieza con dos corridas de prueba; para el experimento usa `runs:=20` o más.

```bash
ros2 run omni_dofbot_bringup waypoints_nav.py --ros-args -p runs:=2
```

### Parámetros del ejecutor

| Parámetro | Valor por defecto | Significado |
|---|---|---|
| `runs` | 20 | Número de corridas |
| `pos_tol` | 0.08 | Tolerancia de posición en cada waypoint (m), contra la pose real |
| `yaw_tol` | 0.15 | Tolerancia de orientación (rad), contra la pose real |
| `goal_timeout` | 60.0 | Límite por waypoint, en segundos simulados |
| `output_dir` | `~/x3_plus_results/<fecha>/` | Carpeta de resultados |
| `set_gains`, `kp`, `ki`, `kd` | `false` | Si `set_gains` es `true`, fija las ganancias del PID antes de empezar |

### Resultados

En la carpeta de salida quedan:

- **`results.json`**: resumen (tasa de éxito con intervalo de Wilson 95 %) y una entrada por corrida con éxito, causa de fallo, tiempo de misión simulado, tiempo real, RTF, longitud real recorrida, recuperaciones, colisiones y error de posición y orientación por waypoint.
- **`run_XX_traj.csv`**: trayectoria a 20 Hz con la pose real (`gt_*`, marco del mundo), la odometría (`odom_*`, marco `odom`, origen en el spawn) y la pose estimada por la localización (`est_*`, marco `map`).

---

## 5. Verificación de la primera prueba

Antes de lanzar las 20 corridas, comprueba estos tres puntos.

**Verdad de terreno.** Debe publicar y marcar aproximadamente (0, −0.91) con el robot en el spawn. Si no publica, el ejecutor aborta con un mensaje de error.

```bash
ros2 topic echo /ground_truth/odom --once
```

**Árbol TF.** Debe ser `map → odom → base_footprint → base_link → …`, sin ciclos.

```bash
ros2 run tf2_tools view_frames
```

**Sensores de contacto.** Empuja el robot contra una caja (por ejemplo publicando en `/cmd_vel` mientras corre el ejecutor) y confirma que aparece `COLISIÓN con obstacle_1` en su log. También puedes observar el tópico directamente:

```bash
ros2 topic echo /contacts/obstacle_1
```

---

## 6. Estado frente a las métricas del Artículo 1

| Métrica | Estado | Lo que falta |
|---|---|---|
| Tasa de éxito (SR), N ≥ 20, Wilson 95 % | Implementada en Gazebo, sin validar en ejecución | Validar; repetir en MuJoCo |
| SPL | Parcial: se registra la longitud real recorrida | Longitud de la ruta más corta de referencia y cálculo del SPL |
| ATE | Parcial: el CSV guarda pose real, odometría y pose estimada | Script de alineación de marcos y RMSE |
| Error de seguimiento (transversal y de rumbo) | No implementada | Grabar la ruta planificada (`/plan`) y calcular el error contra la pose real |
| Tiempo y robustez | Implementada en Gazebo: tiempo de misión, recuperaciones y colisiones | Validar; repetir en MuJoCo |
| Calidad del mapa (IoU, error de pared) | Solo existe el mapa de referencia y los parámetros de `slam_toolbox` | Launch de SLAM, guardado de mapa por corrida, alineación y cálculo |
| Discrepancia Gazebo vs MuJoCo (diferencia relativa, DTW) | No implementada | Todo el lado MuJoCo y el script de comparación |
| Costo computacional (RTF, CPU por segundo simulado) | Parcial: RTF por corrida | Medición de CPU; quitar el límite de RTF del mundo o reportarlo como limitado a 1.0 |

---

## 7. Problemas conocidos y decisiones pendientes

**Decisión por confirmar — posición de WP6.** Se movió de (0, 0.71, yaw 0) a (0.30, 0.81, yaw π), al este del pedestal y mirando hacia él. La meta original quedaba a 0.025 m del borde del pedestal, dentro del footprint, y el hueco de 0.41 m entre la caja O1 y el pedestal no permite girar un robot de 0.33 m de largo. Si se quiere llegar al pedestal desde el sur, hay que mover O1 o el pedestal.

**Geometría de la odometría.** Los nodos usan `lx = 0.110`, pero el URDF coloca las ruedas en x = ±0.08. Si el modelo simulado es el del URDF, la velocidad angular odométrica tiene alrededor de 16 % de error. Se puede comprobar comparando `odom_yaw` contra `gt_yaw` en el CSV.

**Tramos de la ruta que atraviesan las cajas.** Las rectas WP2→WP3 y WP4→WP5 pasan por el centro de O2 y O1; el planificador las rodea. Por eso la ruta más corta para el SPL no es la poligonal entre waypoints.

**Navegación como robot diferencial.** DWB tiene `vy = 0`, así que no se usa el desplazamiento lateral de las ruedas mecanum. Los yaw de los waypoints intermedios obligan a giros en el sitio.

**RTF limitado.** `arena_world.world` fija `real_time_factor` en 1.0, por lo que el RTF medido nunca superará 1.

**Velocity smoother fuera de la cadena.** El controlador publica directo en `/cmd_vel`; nadie consume `cmd_vel_smoothed`.

**Sin sensor IMU.** El plugin del mundo ya está bien declarado, pero el xacro del IMU sigue comentado, así que `/imu` no publica.

**MuJoCo.** `world_mujoco/arena_pid_tunning.xml` ya se puede leer, pero solo contiene la arena vacía: no hay robot, obstáculos, lidar ni puente a ROS 2.

**Empaquetado.** `package.xml` no declara dependencias y `world_mujoco` no se instala.

---

## 8. Trabajo pendiente, en orden

1. **Validar en Gazebo** lo descrito en este documento (sección 5) y corregir lo que falle.
2. **Launch único con modos** (`sim:=gazebo|mujoco`, `localization:=amcl|slam`) que incluya los nodos de cinemática y odometría, más un launch de `slam_toolbox`.
3. **Scripts de análisis offline** para SPL, ATE, error de seguimiento, calidad de mapa, DTW y costo computacional, a partir de `results.json` y los CSV.
4. **Lado MuJoCo**: modelo MJCF del robot con la misma masa, fricción y geometría de ruedas, arena completa, lidar equivalente (1667 rayos, 12 Hz, mismo ruido) y un puente a ROS 2 que exponga `/clock`, `/joint_states`, `/scan`, comandos de rueda, verdad de terreno, contactos y servicio de reset. El resto de la cadena debe quedar idéntico al de Gazebo.
