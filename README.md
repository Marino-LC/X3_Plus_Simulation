# X3 Plus + Dofbot — sintonización PID por AG en Gazebo y en MuJoCo

Rama `MuJoCo_simulation`: **la única rama de trabajo** del experimento. Reúne el
robot de Gazebo con rodillos individuales (antes en `Ruedas_mechanum`, ya
fusionada aquí) y el modelo de MuJoCo con rodillos de junta libre. Un **puente
ROS 2** da a MuJoCo las mismas interfaces que Gazebo, así que el AG
(`ag_motion_tests.py`) corre **sin modificar** en cualquiera de los dos
simuladores. Los parámetros físicos de Gazebo están homologados con los de MuJoCo
(sección [Homologación Gazebo ↔ MuJoCo](#homologación-gazebo--mujoco)) para que
los dos entornos se puedan comparar.

Para correr el AG, ir directo a
[Correr el AG en Gazebo o en MuJoCo](#correr-el-ag-en-gazebo-o-en-mujoco).

La especificación de partida es la guía
[docs/Guia_friccion_mecanum_MuJoCo.md](docs/Guia_friccion_mecanum_MuJoCo.md)
(fricción de rodillos, parámetros, arena, puente y pruebas T1–T10). Este README
resume esas especificaciones tal como quedaron implementadas y da los protocolos
para ejecutarlas.

## Estado (27 nov 2026, MuJoCo 4.0.1, ROS 2 Jazzy, Gazebo Harmonic)

| Parte | Estado |
|---|---|
| Modelo con rodillos libres, actuadores, masa 4.8 kg | Implementado; chasis visible y rodillos alineados con el CAD (ver notas) |
| Arena de 2.42 × 2.42 m, igual en MuJoCo y en `arena_pid_tuning.world` | Implementada |
| T1–T5, T7, T8, T9 (sin ROS) | **Pasan** con los parámetros de partida (resultados abajo) |
| AG de Gazebo (`ag_motion_tests.py` sin modificar) sobre MuJoCo, sin ROS | **Funciona**: batería P1–P4 y AG de humo probados (sección [AG en MuJoCo](#ag-en-mujoco-sin-ros-misma-lógica-que-gazebo)) |
| Gazebo con rodillos individuales, homologado con MuJoCo (27 nov) | **Funciona**: misma batería P1–P4 con fitness a 2 % del de MuJoCo; RTF ≈ 0.7 (ver [comparación](#comparación-de-la-batería-p1p4)) |
| Puente ROS 2 (`mujoco_bridge_node.py`) | Probado con ROS 2 Jazzy en el reporte semanal 5 (/clock, /joint_states, cmd_vel, set_entity_pose) |
| T6, T10, AG completo, AG con ROS real | Pendientes |
| Calibración con hardware (sección 9 de la guía) | Pendiente; hay herramientas listas |

## Estructura del repositorio

```
README.md                         este archivo
docs/
  Guia_friccion_mecanum_MuJoCo.md   guía/especificación (convertida del .docx)
  Conversion_URDF_a_MuJoCo.md       proceso de conversión URDF -> MJCF
src/                              workspace ROS 2 (colcon build desde la raíz)
  omni_dofbot_mujoco/             NUEVO: todo lo de MuJoCo
    omni_dofbot_mujoco/             biblioteca Python sin ROS
      params.py                       parámetros (única fuente de verdad)
      model_builder.py                generador del MJCF
      sim.py                          núcleo: paso, comandos, teletransporte, ground truth
      fake_ros.py                     ROS 2 mínimo en proceso (para correr los nodos reales sin ROS)
      lockstep.py                     AG + nodos reales + puente sobre MuJoCo en paso fijo
    models/
      assets/                         mallas STL
      reference/omni_mujoco2_cad.xml  ensamble CAD de la rueda (fases de los rodillos; lo usa una prueba)
      x3plus_rodillos_fijos.xml       conversión directa del URDF (base del generador)
      omni_dofbot_arena.xml           GENERADO: robot + arena
      omni_dofbot_vacio.xml           GENERADO: robot + piso vacío
    scripts/
      build_model.py                  regenera los XML con otros parámetros
      run_acceptance_tests.py         pruebas T1–T9
      friction_sweep.py               tabla mu x amortiguamiento del rodillo
      plano_inclinado.py              plano inclinado virtual (calibración)
      view.py                         visor interactivo sin ROS
      run_ag_mujoco.py                AG de Gazebo sobre MuJoCo, sin ROS
    src/mujoco_bridge_node.py       puente ROS 2
    launch/mujoco_ag.launch.py      puente + nodo cinemático + odometría (para el AG)
    test/test_acceptance.py         T1–T9 + regresiones (chasis visible, rodillos vs CAD)
    test/test_ag_lockstep.py        AG sobre MuJoCo: batería, determinismo, teletransporte
  omni_dofbot_bringup/            AG, ZN, nodo cinemático, odometría, arena de Gazebo
    launch/gazebo_ag.launch.py      Gazebo + nodo cinemático + odometría (para el AG)
    launch/omni_dofbot_controller.launch.py   Gazebo + robot + controladores
    world/arena_pid_tuning.world    arena del AG (homologada con MuJoCo)
  omni_description/               base móvil: URDF con ruedas de 9 rodillos (mecanum_wheel.xacro)
  omni_dofbot_description/        robot completo (base + brazo) y ros2_control para Gazebo
  dofbot_description/, dofbot_bringup/, omni_bringup/   brazo y base por separado
```

Se quitaron del repositorio para no confundir: el stack de Nav2 (parámetros,
mapas, `nav2_arena.launch.py`, `arena_world.world`, `waypoints_nav.py`) y el AG
anterior basado en Nav2 (`ag_tuner.py`); los intentos antiguos de conversión
URDF → MJCF dentro de los paquetes de descripción (`urdf_a_mjcf.py`,
`omni_mujoco*.xml`, `compiled.urdf`), sustituidos por `omni_dofbot_mujoco`; el
paquete duplicado `omni_description_mujoco` (del que solo se conservó el XML del
CAD en `models/reference/`); modelos MuJoCo duplicados (`*_omni_ellipsoid.xml`,
iguales a los de por defecto); un `.zip` viejo de la biblioteca; el launch de
diagnóstico y su xacro sin control; y el submódulo vacío `src/robot-x-main3`.
Todo sigue en el historial de git.

## Especificaciones

### Modelo (`params.py`)

No se usa fricción anisotrópica: MuJoCo no la tiene. Cada rueda lleva 9 rodillos
(elipsoide + junta de revolución a 45°); la anisotropía sale de que el rodillo
gira libre.

| Parámetro | Valor de partida | Origen |
|---|---|---|
| Fricción rodillo y piso | `0.68 0.005 0.0001` | guía §3 (μ del MDF) |
| Amortiguamiento del rodillo | `1e-4` | guía §3 |
| Armadura del rodillo | `1e-6` | guía §3 |
| Junta de rueda: damping, armature | `0.002`, `5e-4` | guía §3 |
| `<option>` | `timestep=0.001 integrator=implicitfast cone=elliptic impratio=10` | guía §3 |
| Servo de rueda | `<velocity kv=2>`, ±25 rad/s | guía §4 y §6 |
| Par máximo de rueda | 1.0 N·m | **supuesto** (no está en la guía) |
| Rodillo: radio al centro / radio máx. / masa | 32.7 mm / 8.0 mm / 5.77 g | medidos del ensamble CAD y `RODILLO.stl` |
| Semieje del elipsoide | 24.2 mm | calculado, ver nota |
| Brazo | `<position>` por junta, límites del URDF | guía §6 |
| Gripper | `grip_joint` comandable, dedos con `<equality>` | guía §6 |
| Masa total | 4.800 kg | guía §6; el chasis absorbe la diferencia |
| Bits de colisión | piso `1/6`, rodillo `2/1`, chasis `4/0` | guía §4 |

Notas:

- **Semieje del elipsoide.** El rodillo real mide 21.2 mm de semilargo, pero un
  elipsoide con esa medida no reproduce el perfil de barril. Se usa
  `hl² = r² + 2·Rc·r` (24.2 mm), que deja plana la envolvente de la rueda. Con un
  elipsoide más largo dos rodillos tocan el piso a la vez y la rueda se bloquea
  en lateral.
- **Quiralidad.** `roller_sign = +1` es el patrón correcto para este modelo con
  `wheel_polarity = [-1, -1, -1, -1]`. Si se cambia la orientación de las ruedas
  y el robot va al revés en y, se invierte ahí.
- **Chasis.** La conversión del URDF perdió la inercia de `base_link` y de la
  base del brazo. El chasis es una masa concentrada (4.195 kg, caja equivalente
  de 0.24 × 0.17 × 0.10 m, centro de masa en `0.02 0 0` de `base_link`). Es un
  supuesto: hay que sustituirlo por el centro de masa e inercias reales.
- **Colisiones.** Solo chocan rodillos y chasis contra piso y paredes. El brazo
  no tiene geometría de colisión.
- **Fase de los rodillos (corregida el 19 nov).** Los rodillos se colocaban a
  partir de 0° en las cuatro ruedas. En el CAD (`models/reference/omni_mujoco2_cad.xml`) el rodillo 1
  está a 2.187° en FL/BR (cubo DERECHA) y a 17.814° en FR/BL (cubo IZQUIERDA). En
  FR y BL eso dejaba los rodillos visibles hasta 10 mm y 12° fuera de las
  ranuras del cubo: los «rodillos fuera de lugar» del reporte 5. Ahora las fases
  salen de `params.py` (`roller_phase_deg_diag`, `roller_phase_deg_anti`) y
  `test_rodillos_coinciden_con_cad` verifica < 1 mm y < 1° contra el CAD. La
  envolvente de contacto es la misma, así que T1–T9 casi no cambian.
- **Chasis invisible (corregido el 18 nov).** El generador procesaba dos veces
  las geometrías que cuelgan directo de `base_link`, y en la segunda pasada
  borraba la malla verde del chasis (`base_link_omni_dofbot`). Era solo visual:
  la masa y la colisión del chasis no cambian. Si tienen XML generados antes,
  regenérenlos con `build_model.py`.

### Arena (`omni_dofbot_arena.xml`)

| Elemento | Gazebo | MuJoCo |
|---|---|---|
| Gravedad, paso | 9.81 m/s², 0.001 s | igual |
| Piso | caja de 2.42 × 2.42 m, μ = μ2 = 0.68 (isotrópica) | 2.42 × 2.42 m, plano con μ = 0.68 |
| Paredes | 0.30 m de alto, 0.03 m de espesor, μ = 0.5 | cuatro cajas estáticas |
| Pose de reinicio y de aparición | (0, 0, 0.05), yaw = π/2 | igual |
| Marcadores | (0,0), (0,0.8), (0.35,0.8) | solo visuales |

La arena mide **2.42 × 2.42 m**. El AG sintoniza con movimiento en x, y y yaw:
la prueba P4 recorre 0.80 m en lateral desde el origen (hacia −X del mundo, con
el robot en yaw = π/2), y eso no cabe en una arena de 1.22 m de ancho. Con
2.42 m quedan unos 0.28 m entre el robot y la pared al final de P4.

`src/omni_dofbot_bringup/world/arena_pid_tuning.world` también se amplió a
2.42 × 2.42 m (piso y paredes este/oeste en ±1.215 m), para que Gazebo y MuJoCo
usen la misma arena. Las dimensiones de MuJoCo se cambian con
`build_model.py --arena_x ... --arena_y ...`.

## Homologación Gazebo ↔ MuJoCo

El AG de MuJoCo es la versión final (con las correcciones del CHANGELOG #10), así
que los parámetros de Gazebo se igualaron a los de MuJoCo. La fuente única es
`params.py`; Gazebo no puede importarlo, y los valores están copiados a mano en
tres archivos (los tres lo dicen en un comentario):

| Parámetro | MuJoCo (`params.py`) | Gazebo | Archivo de Gazebo |
|---|---|---|---|
| Masa total | `total_mass` = 4.8 kg | 4.8 kg | (suma del URDF) |
| Chasis: masa, CoM, inercia | 4.1955 kg en `(0.02, 0, 0)`, caja 0.24 × 0.17 × 0.10 m | 4.1954 kg, mismos CoM e inercia (0.0136, 0.0236, 0.0302) | `omni_description/urdf/omni_urdf.xacro` |
| Cubo de rueda | 0.0515 kg | 2 platos de 0.02577 kg | `omni_description/urdf/mecanum_wheel.xacro` |
| Rodillos | 9 por rueda, 5.77 g | 9 por rueda, 5.77 g (del CAD) | ídem |
| Damping del rodillo | `roller_damping` = 1e-4 | 1e-4 (antes 0.005) | ídem |
| Fricción seca del rodillo | `roller_frictionloss` = 0 | 0 (antes 0.001) | ídem |
| Damping de la junta de rueda | `wheel_damping` = 0.002 | 0.002 (antes 0) | ídem |
| Par máximo de rueda | `wheel_max_torque` = 1.0 N·m | `effort` = 1.0 (antes 100) | ídem |
| Velocidad máxima de rueda | 25 rad/s | 25 rad/s | ídem |
| Fricción rodillo/piso | μ = 0.68 | μ = μ2 = 0.68 en rodillos y piso (antes piso 0.68/0.4 con `fdir1`) | `mecanum_wheel.xacro`, `arena_pid_tuning.world` |
| Colisiones | solo rodillos y chasis | solo rodillos y chasis (los platos no chocan) | `mecanum_wheel.xacro` |
| Arena, paredes, gravedad, paso | 2.42 m, 0.30 m, 9.81, 1 ms | igual | `arena_pid_tuning.world` |
| Pose inicial | `RESET_POSE` = (0, 0, 0.05), yaw = π/2 | igual al aparecer y en cada teletransporte | `omni_dofbot_controller.launch.py` |
| Nodos de control, odometría y AG | `omni_dofbot_bringup/src` | los mismos archivos | — |

Antes de homologar, el robot de Gazebo pesaba 6.94 kg (chasis de 5 kg con inercia
100 veces mayor que la de una caja de su tamaño, y platos de 0.193 kg).

Diferencias que **no** se pueden igualar (hay que tenerlas en cuenta al comparar):

- **Actuador de rueda.** MuJoCo usa un servo de velocidad `kv = 2` saturado a
  1 N·m; `gz_ros2_control` impone la velocidad como restricción ideal, limitada
  por el mismo `effort`. En los dos casos la dinámica que domina es la planta del
  motor dentro de `mecanum_kinematic_node.py` (τ ≈ 0.58 s), que es la misma.
- **Forma del rodillo.** MuJoCo usa un elipsoide calibrado (envolvente plana);
  Gazebo usa la malla `RODILLO.stl`. URDF no tiene elipsoides.
- **Armadura.** MuJoCo añade armadura a rodillos y ruedas; URDF no tiene ese campo.
- **Motor de física y contacto.** MuJoCo con cono elíptico e `impratio = 10`;
  Gazebo Harmonic con DART. Las fricciones se combinan con el mínimo de las dos
  superficies, por eso rodillo y piso llevan el mismo μ.
- **Ground truth.** `/ground_truth/odom` solo existe en MuJoCo; la comparación
  directa se hace con `/odom`.

### Comparación de la batería P1–P4

Una evaluación del AG con las ganancias del fabricante `[0.8, 0.06, 0.5]` (27 nov):

| | P1 | P2 | P3 | P4 | fitness |
|---|---|---|---|---|---|
| MuJoCo (paso fijo, `run_ag_mujoco.py --battery 0.8 0.06 0.5`) | 17.36 | 0.68 | 4.94 | 17.32 | **10.29** |
| Gazebo homologado (`gazebo_ag.launch.py gui:=false`, RTF 0.67) | 17.60 | 0.66 | 5.20 | 17.73 | **10.52** |

La diferencia es de ~2 %, del orden del ruido de una misma evaluación (±1.5 %,
ver «Revisión de resultados»). En Gazebo la corrida se hizo con el filtro de RTF
desactivado (ver el siguiente apartado), así que es indicativa.

### Gazebo: factor de tiempo real

Con 36 rodillos con colisión de malla, Gazebo corrió a **RTF ≈ 0.67–0.80** sin
ventana en una máquina de 12 núcleos. El AG descarta toda evaluación con RTF medio
< `RTF_MIN = 0.90`, porque el PID incremental no normaliza por dt; con ese RTF
el AG en Gazebo repite cada evaluación sin fin. Opciones:

- Correr Gazebo con `gui:=false rviz:=false` y sin otras cargas.
- Usar MuJoCo en paso fijo para la sintonización (no depende del RTF) y Gazebo para
  validar las ganancias obtenidas.
- Bajar `RTF_MIN` en `ag_motion_tests.py`. Es una decisión de diseño: cambia la
  validez de la medición y debe quedar reportado.

## Correr el AG en Gazebo o en MuJoCo

El mismo `ag_motion_tests.py`, la misma arena y los mismos nodos. Compilar una vez
(ver [Instalación](#instalación)) y, en cada terminal, `source install/setup.bash`.

| | Terminal 1 | Terminal 2 |
|---|---|---|
| **Gazebo** | `ros2 launch omni_dofbot_bringup gazebo_ag.launch.py` | `ros2 run omni_dofbot_bringup ag_motion_tests.py` |
| **MuJoCo con ROS** (tiempo real) | `ros2 launch omni_dofbot_mujoco mujoco_ag.launch.py` | `ros2 run omni_dofbot_bringup ag_motion_tests.py` |
| **MuJoCo sin ROS** (paso fijo, ~5–8×) | `cd src/omni_dofbot_mujoco/scripts && python3 run_ag_mujoco.py --seed 1 --out resultados_ag` | — |

Gazebo:

```bash
ros2 launch omni_dofbot_bringup gazebo_ag.launch.py
```

Sin ventana de Gazebo (recomendado para el AG, por el RTF):

```bash
ros2 launch omni_dofbot_bringup gazebo_ag.launch.py gui:=false
```

MuJoCo con ROS:

```bash
ros2 launch omni_dofbot_mujoco mujoco_ag.launch.py
```

En los dos casos, en otra terminal:

```bash
ros2 run omni_dofbot_bringup ag_motion_tests.py
```

Las salidas (`ag_results.json`, `.png`, `.html`) se escriben en el directorio
desde el que se lanzó el AG, igual en los dos simuladores. Los detalles del AG
(pruebas, costo, rondas) están en `src/omni_dofbot_bringup/README.md`.

### Interfaces ROS 2 del puente

| Interfaz | Tipo | Función |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | tiempo de simulación |
| `/joint_states` | `sensor_msgs/JointState` | ruedas, brazo y gripper (100 Hz) |
| `/wheel_velocity_controller/commands` | `std_msgs/Float64MultiArray` | `[fl, fr, br, bl]` rad/s |
| `/dofbot_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | interpolación lineal hasta `time_from_start` |
| `/dofbot_gripper_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | igual, para `grip_joint` |
| `/world/arena_pid_tuning/set_entity_pose` | `ros_gz_interfaces/SetEntityPose` | teletransporte: fija `qpos`, anula `qvel`, `mj_forward`. En Gazebo, `omni_dofbot_controller.launch.py` ofrece el mismo servicio con `ros_gz_bridge` (→ `set_pose` de gz) |
| `/ground_truth/odom` | `nav_msgs/Odometry` | pose real del chasis (nuevo) |
| `/mujoco_bridge_node/reset` | `std_srvs/Empty` | estado inicial completo |

El puente corre a tiempo real (`real_time_factor = 1.0`) porque el AG mide con
`time.time()`. `/odom` sigue saliendo de `Mecanum_odometry_node.py`, que integra
las ruedas y no ve el deslizamiento: el error de posición real se mide con
`/ground_truth/odom`.

## Instalación

### Solo MuJoCo (sin ROS) — suficiente para T1–T5, T7–T9

```bash
python3 -m pip install mujoco numpy pytest
```

Para el AG sobre MuJoCo sin ROS, además:

```bash
python3 -m pip install deap matplotlib plotly
```

### Con ROS 2 Jazzy — para el puente, T6 y T10

Dos formas probadas. Con el Python del sistema (Ubuntu 24.04 pide
`--break-system-packages`, igual que `deap`):

```bash
python3 -m pip install mujoco deap matplotlib plotly --break-system-packages
```

O con un entorno virtual, como en el reporte semanal 5. Dos detalles:

- **Si el venv queda dentro del workspace, `colcon` intenta compilarlo y falla.**
  Créenlo fuera de la raíz o pongan un `COLCON_IGNORE` dentro del venv. El
  `.gitignore` ya excluye `venv/` y `.venv/`.
- **Dentro del venv no están las dependencias de Python de ROS 2.** Instálenlas
  en el venv, o creen el venv con `--system-site-packages`.

```bash
python3 -m venv venv && touch venv/COLCON_IGNORE
```

```bash
source venv/bin/activate
```

```bash
pip install mujoco deap matplotlib plotly pyyaml catkin_pkg empy lark numpy setuptools jinja2 typeguard
```

En los dos casos:

```bash
sudo apt install ros-jazzy-ros-gz-interfaces
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

Si `ros2 launch` dice `executable '...py' not found on the libexec directory`,
el script no tiene permiso de ejecución. Con `--symlink-install` el ejecutable
instalado es un enlace al archivo fuente y conserva sus permisos. En el
repositorio ya están marcados como ejecutables. Si su copia es anterior:

```bash
chmod +x src/omni_dofbot_mujoco/src/*.py src/omni_dofbot_mujoco/scripts/*.py src/omni_dofbot_bringup/src/*.py
```

`ros2 launch` corre los nodos con el `python3` que encuentre primero en el
PATH. Si `mujoco` y `deap` están instalados solo en el venv, el venv tiene que
estar activado en esa terminal.

## Ejecución rápida

Todos los scripts se ejecutan desde `src/omni_dofbot_mujoco/scripts/` y aceptan
`--help`. Cualquier parámetro de `params.py` es una bandera (`--mu`,
`--roller_damping`, `--wheel_kv`, ...).

```bash
cd src/omni_dofbot_mujoco/scripts
```

Ver el robot moviéndose en lateral (tecla `3` en el visor muestra los rodillos de colisión):

```bash
python3 view.py --vy 0.2
```

Correr las pruebas de aceptación:

```bash
python3 run_acceptance_tests.py
```

Regenerar los XML tras cambiar parámetros:

```bash
python3 build_model.py --mu 0.5 --roller_damping 1e-3
```

Las banderas solo valen para esa regeneración. Si después se corre
`build_model.py` sin banderas, los XML vuelven a los valores por defecto de
`params.py` (μ = 0.68, damping = 1e-4, los de la guía). Para que unos valores
queden fijos, hay que cambiarlos en `params.py`.

## Protocolos de prueba

Las pruebas son las de la sección 8 de la guía. El robot se suelta desde
z = 0.05 m, se deja asentar 1 s y se aplica el comando como escalón. El
desplazamiento se mide con la pose real del chasis, en el marco inicial del robot.

### T1–T5, T7–T9: automáticas, sin ROS

```bash
python3 run_acceptance_tests.py
```

Un subconjunto, o con salida a archivo:

```bash
python3 run_acceptance_tests.py --only T2 T3 --csv resultados.csv
```

Lo mismo con pytest, desde la raíz del repositorio:

```bash
python3 -m pytest src/omni_dofbot_mujoco/test -v
```

| Prueba | Procedimiento | Criterio | Resultado (19 nov, valores de partida) |
|---|---|---|---|
| T1 Reposo | soltar, esperar 2 s, medir 1 s | velocidad < 1e-3, z estable, sin deriva | pasa: 6.2e-5, z varía 0.6 µm, deriva 0.06 mm |
| T2 Avance x | 0.2 m/s, 2 s | error < 5 % | pasa: 0.3993 m (0.18 %) |
| T3 Lateral y | 0.2 m/s, 2 s | error < 5 % | pasa: 0.3872 m (3.20 %) |
| T4 Rotación | wz = 1 rad/s, 2 s | error < 5 % | pasa: 1.9857 rad (0.71 %) |
| T5 Diagonal | vx = vy = 0.2 m/s, 2 s | 45° ± 3° | pasa: 44.26° |
| T7 Con brazo | arena, 0.2 m/s 4 s con HOME → PICK_LEFT → HOME → PICK_RIGHT → HOME | inclinación < 15° | pasa: 0.21° |
| T8 Tiempo real | arena, brazo en movimiento, 3 s | ≥ 1× | pasa: ~16× (solo física, 1 hilo) |
| T9 Determinismo | misma secuencia 3 veces | `qpos` idéntico | pasa: diferencia 0 |

Lecturas que no son aprobado/reprobado:

- **T7.** El brazo apenas perturba la base (+0.2 mm lateral, −0.01° de yaw). Sus
  eslabones suman unos 0.2 kg frente a 4.8 kg totales. La guía espera que la
  perturbación "se note": revisar la masa del brazo y de la carga antes de
  sacar conclusiones.
- **T8.** Mide solo la física. Con el puente, el visor y los nodos ROS será
  menor; el puente imprime cada 10 s el factor de tiempo real medido.

### Barrido de fricción (tabla de la guía §3)

```bash
python3 friction_sweep.py
```

Desplazamiento en y [m] con 0.2 m/s durante 2 s (ideal 0.400):

| μ \ damping | 1e-5 | 1e-4 | 1e-3 | 1e-2 |
|---|---|---|---|---|
| 0.3 | 0.380 | 0.376 | 0.244 | 0.162 |
| 0.68 | 0.389 | 0.387 | 0.352 | 0.231 |
| 1.0 | 0.390 | 0.389 | 0.372 | 0.265 |

Misma tendencia que el esqueleto de la guía: el amortiguamiento del rodillo
pesa más que μ en el movimiento lateral.

### Puente ROS 2: verificación inicial

Primer arranque (ejecutado en el reporte semanal 5; resultados abajo):

```bash
ros2 launch omni_dofbot_mujoco mujoco_ag.launch.py viewer:=true
```

En otra terminal:

```bash
ros2 topic hz /clock
```

```bash
ros2 topic echo /joint_states --once
```

El nodo cinemático trae `kp=1, ki=0` por defecto. Con eso, 0.2 m/s producen
PWM = 200, debajo de la zona muerta de arranque (`pwm_breakaway_x` = 376), y
**las ruedas no giran**. Además el watchdog de `cmd_vel` (0.5 s) detiene el
robot si el comando se publica una sola vez. Para mover el robot, poner las
ganancias del fabricante y publicar de forma continua (Ctrl+C para detener):

```bash
ros2 param set /mecanum_kinematic_node ki 0.06
```

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

```bash
ros2 service call /world/arena_pid_tuning/set_entity_pose ros_gz_interfaces/srv/SetEntityPose "{entity: {name: omni_dofbot, type: 2}, pose: {position: {z: 0.05}, orientation: {z: 0.7071068, w: 0.7071068}}}"
```

Se espera: las cuatro ruedas y las juntas del brazo en `/joint_states`, el robot
avanzando hacia +Y del mundo con `cmd_vel` en x, y el robot de vuelta en el
origen tras el servicio. Con `ki = 0.06`, el modo sin ROS da 38 cm en 2 s; con
`kp=1, ki=0`, 0 cm.

Resultados del reporte 5 y lectura:

| Prueba | Resultado | Lectura |
|---|---|---|
| `/clock` | 860 Hz (RTF ≈ 0.86) con el visor abierto | `ros2 topic hz` en Python suele quedarse corto a 1 kHz; el dato fiable es el «Factor de tiempo real medido» que imprime el puente cada 10 s. Si es < 0.9, el AG **descarta** esas evaluaciones (`_gated`). Opciones: cerrar el visor, `clock_decimation:=2`, o el AG en paso fijo (no depende del RTF). |
| `/joint_states` | 15 juntas con posición y velocidad | Correcto (el orden es el del modelo; el AG busca por nombre). |
| `cmd_vel --once` con 0.2 m/s | Las ruedas casi no giran | Esperado con `kp=1, ki=0`: zona muerta de `mecanum_kinematic_node.py` (ver arriba). Con el mouse en el visor una rueda también cuesta girarla: el servo de velocidad la mantiene en el comando (0 rad/s). |
| Rodillos fuera de lugar en dos ruedas | Visto en FR y BL | Bug del generador, corregido (ver «Fase de los rodillos»). |
| `set_entity_pose` | `success=True` | Correcto. |

### T6. Escalón de rueda contra Gazebo

En cada simulador (MuJoCo con el launch de arriba; Gazebo con
`omni_dofbot_controller.launch.py`), sin el nodo cinemático publicando:

```bash
ros2 bag record -o t6_mujoco /joint_states /clock
```

```bash
ros2 topic pub -r 50 /wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [-5.0, -5.0, -5.0, -5.0]}"
```

Criterio: tiempo de subida y velocidad en régimen de las ruedas similares.
Si MuJoCo responde más lento o con sobrepaso, ajustar `--wheel_kv` y
`--wheel_max_torque` y regenerar el modelo.

### T10. Equivalencia con Gazebo

Con las mismas ganancias Kp, Ki, Kd en ambos simuladores, correr P1, P2, P3 y
P4 (lateral) y grabar:

```bash
ros2 bag record -o t10_mujoco /odom /ground_truth/odom /cmd_vel /joint_states
```

Criterio: curvas vx(t), vy(t), x(t), y(t) y yaw(t) comparables; reportar el RMSE.
`/ground_truth/odom` solo existe en MuJoCo, así que la comparación directa se
hace con `/odom`.

## AG en MuJoCo sin ROS (misma lógica que Gazebo)

No hace falta correr la sintonización al mismo tiempo que Gazebo, pero sí con la
misma lógica. Por eso no se reescribió el AG: `run_ag_mujoco.py` carga **los
archivos reales, sin modificarlos**, y los conecta con MuJoCo en el mismo proceso:

| Pieza | Archivo que se ejecuta |
|---|---|
| AG (DEAP), batería P1–P4, costos, rondas con semilla, gráficas y JSON | `omni_dofbot_bringup/src/ag_motion_tests.py` |
| Cinemática inversa + PID incremental + planta del motor + zona muerta | `omni_dofbot_bringup/src/mecanum_kinematic_node.py` |
| Odometría por cinemática directa | `omni_dofbot_bringup/src/Mecanum_odometry_node.py` |
| Física y teletransporte | `omni_dofbot_mujoco/src/mujoco_bridge_node.py` + modelo MuJoCo |

`fake_ros.py` reemplaza a rclpy y a los mensajes por una versión mínima en proceso
(tópicos y servicios síncronos). `lockstep.py` avanza la física 1 ms por cada
milisegundo que el AG espera, con el reloj de simulación como reloj de todos los
nodos.

Qué cambia frente a correrlo con ROS real:

- **Paso fijo y determinista.** El hilo del brazo del AG corre de forma
  cooperativa entre pasos de física. Con `--seed` la corrida completa es
  reproducible.
- **Más rápido que tiempo real.** Va a unas 5×. La batería P1–P4 son unos 25 s
  simulados y tarda unos 5 s.
- **Sin control de RTF.** La física no depende del reloj de pared, así que
  `_gated()` siempre recibe RTF = 1.0 y nunca descarta evaluaciones. En el JSON,
  `rtf_mean` vale 1.0.
- **Sin latencia de mensajes.** Los tópicos se entregan en el acto. En ROS real
  hay unos pocos ms de retraso.

### Uso

```bash
cd src/omni_dofbot_mujoco/scripts
```

Una batería P1–P4 con unas ganancias (una evaluación del AG):

```bash
python3 run_ag_mujoco.py --battery 0.8 0.06 0.5
```

AG de humo (población 8, 3 generaciones, 1 ronda):

```bash
python3 run_ag_mujoco.py --smoke --seed 1 --out resultados_humo
```

AG completo, con la configuración de `ag_motion_tests.py` (35 individuos, 30 generaciones, hasta 10 rondas):

```bash
python3 run_ag_mujoco.py --seed 1 --out resultados_ag
```

Otras opciones: `--pop`, `--gen` y `--max-rounds` sobrescriben las constantes
del AG. `--viewer --realtime` lo muestra en el visor a velocidad real (necesita
pantalla). `--verbose` muestra también los mensajes de los nodos. Las salidas
son las mismas que en Gazebo (`ag_results.json`, `.png`, `.html`), más
`ag_log.txt`.

Pruebas automáticas (batería del fabricante sin penalización, determinismo,
teletransporte):

```bash
python3 -m pytest src/omni_dofbot_mujoco/test/test_ag_lockstep.py -v
```

### Qué se ve en la terminal

Cada línea lleva el tiempo de simulación entre corchetes. Una evaluación (batería
P1–P4) se ve así:

```
[    0.050] [INFO] [ag_motion_evaluator] === [Progreso AG] Tiempo: 00:00:00 | Ronda: 1 | Gen: 0/3 | Ind: 1/8 ===
[    0.050] [INFO] [ag_motion_evaluator] [AG] Kp=1.000 Ki=0.000 Kd=0.000
[    0.050] [INFO] [ag_motion_evaluator] ── P1: línea recta adelante-atrás ──
[    1.000] [INFO] [ag_motion_evaluator] Odometría reseteada a (0, 0, 90°).
[    1.400] [INFO] [ag_motion_evaluator] [Brazo] hilo iniciado — pick & place lado a lado
[   11.087] [INFO] [ag_motion_evaluator]    ITAE=6.5302 t=9.1s err_f=0.001m cost=79.0470
...
[   63.567] [INFO] [ag_motion_evaluator] [AG] P1=79.0460 P2=68.5563 P3=70.7782 P4=665.4622 fit=221.07179 RTF=1.00 (min 1.00) retries=0
```

Al cerrar cada generación aparece `[RONDA r | GEN g] min=... mean=... best=[Kp,Ki,Kd]`.
Al final aparece `MEJOR INDIVIDUO` con Kp, Ki, Kd y el fitness, luego la corrida
grabada del mejor, `JSON guardado en ...` y `[plot] PNG/HTML guardado en ...`.
La última línea la escribe `run_ag_mujoco.py`:
`Tiempo de simulación ... s en ... s de pared (≈4-5x)`.

Señales normales:

- **Semilla `[1, 0, 0]` con fitness ≈ 221.** Es la primera evaluación de la
  ronda 1 y lleva `PENALTY_TO`, porque con Ki = 0 no se vence la zona muerta.
- **`RTF=1.00` y `Controladores descartados por RTF: 0`.** En paso fijo siempre es así.
- **Con `--smoke`: `Se alcanzó MAX_ROUNDS=1 ...`.** El humo corre una sola ronda a propósito.

Señales de problema: `Traceback`, `ERROR`, `Aborto temprano — deriva lateral` en
casi todos los individuos, o costos ≥ 50 con las ganancias del fabricante.

Duración aproximada (en esta máquina; en WSL puede ser más lento):

| Corrida | Evaluaciones | Tiempo |
|---|---|---|
| `--battery` | 1 | 5–6 s |
| `--smoke` | unas 23, más la corrida del mejor | 2.5 min |
| AG completo | unas 785 por ronda; desde la 2a ronda se corta si la 1a generación mejora < 10 % | ≈ 1.3 h por ronda (estimado) |

`ag_results.*` se escriben al terminar. `ag_log.txt` se va escribiendo
durante la corrida. Si se corta con Ctrl+C, solo queda el log.

Con `--viewer`, en una pantalla virtual (Xvfb) el proceso puede terminar con
`Segmentation fault` **al cerrar**, después de guardar los resultados. Pasa
igual con un visor de MuJoCo mínimo, sin código del proyecto.

### Resultados (18 nov, parámetros de partida del modelo)

| Corrida | Resultado |
|---|---|
| Batería con la semilla del AG `[1, 0, 0]` | fitness 221.1. P1, P2, P3 y P4 se penalizan por timeout: con Ki = 0 no alcanzan la velocidad. |
| Batería con las ganancias del fabricante `[0.8, 0.06, 0.5]` | fitness 10.29 (P1 17.36, P2 0.69, P3 4.94, P4 17.31). Las cuatro llegan a la meta. |
| AG de humo, `--seed 1` | Mejor `[1.275, 0.495, 0.899]`, fitness 8.39. 616 s simulados en 2 min 8 s. |

Una estimación propia del AG completo: cada ronda son unas 785 evaluaciones,
alrededor de 1 h por ronda en esta máquina, contra unas 5 h en Gazebo a tiempo
real.

### Observaciones sobre la lógica del AG (aplican también en Gazebo)

1. **La detección de volcadura nunca se activa.** `_odom_cb` lee roll y pitch de
   `/odom`, pero `Mecanum_odometry_node.py` solo publica yaw, así que roll y
   pitch siempre valen 0. Para detectarla de verdad el AG tendría que leer
   `/ground_truth/odom` (MuJoCo) o la pose de Gazebo. *Sin cambiar.*
2. **El robot rodaba después del teletransporte.** *Corregido (CHANGELOG #10).*
   El filtro del motor seguía moviendo las ruedas 0.8 s y el robot rodaba
   37 mm de media (hasta 59 mm). Ahora `_teleport()` reinicia el controlador
   antes, y el desplazamiento queda en ~1 mm.
3. **`/odom` no ve el deslizamiento.** En P4 la curva de vy es idéntica a la de
   vx en P1, porque la odometría integra las ruedas. El deslizamiento lateral
   real de los rodillos se mide con `/ground_truth/odom`. *Sin cambiar.*

### Revisión de resultados: Kp ≈ 0 y salto del fitness al final (25 nov)

Todo se midió con el AG real en paso fijo. Los scripts de medición no forman
parte del repositorio.

**El salto del fitness en la "última generación" no es un error de evolución.**
La gráfica une las rondas una tras otra. Desde la ronda 2, si la primera
generación no mejora 10 % la corrida se detiene, así que la última generación
graficada es casi siempre la **generación 0 de una ronda nueva**: población
aleatoria, `HallOfFame` vacío y la semilla reevaluada. En una corrida de
prueba el mínimo pasó de 8.394 a 8.527 y la media de 8.617 a 8.639 justo ahí.
El resultado final (`MEJOR INDIVIDUO`) es el mejor global y no se pierde. La
gráfica ahora marca con una línea discontinua el inicio de cada ronda.

**Por qué Kp termina en ≈ 0 y la velocidad oscila:**

- **El costo no castiga la oscilación de velocidad.** Con Ki = 0.495 y Kd = 0.899,
  subir Kp de 0 a 5 reduce la oscilación de vx de 0.083 a 0.002 m/s RMS, pero
  el fitness solo cambia ~4 % (8.43–8.83).
- **Casi todo el costo de P1 y P4 es fijo.** El ITAE mide la distancia a la
  **meta final**. Aun con seguimiento perfecto a 0.4 m/s vale ~1.07, es decir
  unos 12.8 de los ~14.5 puntos de P1. La parte que depende del PID premia
  llegar antes, así que un sobreimpulso de velocidad lo abarata.
- **Con Ki bajo gana Kp = 0.** Con Ki = 0.06 el orden se invierte: Kp = 0 da
  10.11, Kp = 0.8 da 10.32 y Kp = 3 da 10.85. Si el AG cae en esa zona, lleva
  Kp a cero.
- **La medición tiene ruido del mismo tamaño que el efecto de Kp.** Las mismas
  ganancias dan entre 8.40 y 8.67 (±1.5 %) según el desfase entre los relojes
  de 100 Hz y la comprobación de llegada cada `CTRL_DT = 50 ms` (2 cm a
  0.4 m/s). Entre Kp = 1 y Kp = 3.5 la diferencia es ~0.1, menor que ese ruido.
  Como los élites del `HallOfFame` no se reevalúan, el AG conserva individuos
  con evaluaciones "con suerte". En ROS real el ruido es mayor.
- **No influyen:** la banda muerta de error del nodo cinemático (0 o 2 mm/s dan
  lo mismo) ni el individuo evaluado antes.

**Opción nueva: `W_VEL` (CHANGELOG #10, apagada por defecto).** Cada prueba suma
`W_VEL · RMS(v_real − v_ref) / REF`, con REF = 0.04 m/s en P1, P3 y P4 y
0.10 rad/s en P2. Con `W_VEL = 0.0` el costo es idéntico al original. Con
`W_VEL = 0.3`:

| | mejor Kp con Ki = 0.495 | mejor Kp con Ki = 0.06 |
|---|---|---|
| costo original | 2.0 | 0.0 |
| `W_VEL = 0.3` | 3.5 | 0.8 |

Kp = 0 deja de ser competitivo. En MuJoCo:

```bash
python3 run_ag_mujoco.py --w-vel 0.3 --seed 1 --out resultados_ag_wvel
```

En Gazebo o con ROS real se cambia la constante `W_VEL` en `ag_motion_tests.py`.
Activarla cambia la escala del fitness, así que no se pueden comparar fitness
entre corridas con y sin `W_VEL`. Es una decisión de diseño de la tesis.

### Con ROS 2 real (puente probado, AG aún no)

El mismo AG también puede correr contra el puente en tiempo real, como con Gazebo:

```bash
ros2 launch omni_dofbot_mujoco mujoco_ag.launch.py
```

```bash
ros2 run omni_dofbot_bringup ag_motion_tests.py
```

El launch ya incluye el nodo cinemático y la odometría.

## Calibración con el robot real (guía §9)

Parámetros a ajustar: `mu`, `roller_damping`, `wheel_kv` y, si hace falta,
`wheel_frictionloss`.

Plano inclinado virtual, para comparar con la misma prueba en hardware:

```bash
python3 plano_inclinado.py --mu 0.68
```

Con μ = 0.68 da un ángulo crítico de 26.5° en x y 24.4° en y (μ efectivo ≈ 0.50
y 0.45). Es menor que μ porque el rodillo solo transmite fuerza a lo largo de su
eje, a 45°.

Las rampas de PWM se comparan repitiendo `friction_sweep.py` y T2–T4 con los
valores candidatos hasta igualar la velocidad en régimen medida.

## Pendientes y limitaciones

- Correr T6, T10 y el AG con ROS 2 real; revisar que el factor de tiempo real del puente quede ≥ 0.9.
- Correr el AG completo en MuJoCo y comparar las ganancias con las de Gazebo.
- Sustituir la masa concentrada del chasis por inercias reales.
- Confirmar el par máximo del motor (`wheel_max_torque`) y `wheel_kv` con T6.
- Gazebo con rodillos corre a RTF < 0.9 en la máquina de pruebas; ver «Gazebo: factor de tiempo real».
- El piso es un plano infinito, no una caja; dentro de las paredes es equivalente.
- Coordinar con Marino cómo usa el AG `/ground_truth/odom`.
- Plan B de la guía: si T2–T4 dejan de pasar con el modelo calibrado, reportar MuJoCo con ruedas simplificadas como limitación.
