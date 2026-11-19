# X3 Plus + Dofbot — simulación en MuJoCo

Rama `MuJoCo_simulation`. Objetivo: tener en MuJoCo un modelo del X3 Plus + Dofbot
con **rodillos mecanum de junta libre** y un **puente ROS 2** que ofrezca las mismas
interfaces que el Gazebo actual, para correr el AG (`ag_motion_tests.py`) sin
modificarlo.

La especificación de partida es la guía
[docs/Guia_friccion_mecanum_MuJoCo.md](docs/Guia_friccion_mecanum_MuJoCo.md)
(fricción de rodillos, parámetros, arena, puente y pruebas T1–T10). Este README
resume esas especificaciones tal como quedaron implementadas y da los protocolos
para ejecutarlas.

## Estado (19 nov 2026, MuJoCo 4.0.1)

| Parte | Estado |
|---|---|
| Modelo con rodillos libres, actuadores, masa 4.8 kg | Implementado; chasis visible y rodillos alineados con el CAD (ver notas) |
| Arena de 2.42 × 2.42 m, igual en MuJoCo y en `arena_pid_tuning.world` | Implementada |
| T1–T5, T7, T8, T9 (sin ROS) | **Pasan** con los parámetros de partida (resultados abajo) |
| AG de Gazebo (`ag_motion_tests.py` sin modificar) sobre MuJoCo, sin ROS | **Funciona**: batería P1–P4 y AG de humo probados (sección [AG en MuJoCo](#ag-en-mujoco-sin-ros-misma-lógica-que-gazebo)) |
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
    launch/mujoco_ag.launch.py      puente + nodo cinemático + odometría
    test/test_acceptance.py         T1–T9 + regresiones (chasis visible, rodillos vs CAD)
    test/test_ag_lockstep.py        AG sobre MuJoCo: batería, determinismo, teletransporte
  omni_dofbot_bringup/            AG, ZN, nodo cinemático, odometría, mundos Gazebo
                                  (versión final de la sintonización; ver su README.md)
  omni_dofbot_description/ ...    URDF y paquetes Gazebo (sin cambios; ver src/README.md)
  omni_description_mujoco/        referencia: rueda CAD de 9 rodillos (COLCON_IGNORE)
```

Cambios de ubicación respecto a la estructura anterior:

| Antes | Ahora |
|---|---|
| `X3Plus_MuJoCo_portable/assets/` | `src/omni_dofbot_mujoco/models/assets/` |
| `X3Plus_MuJoCo_portable/x3plus_mujoco.xml` | `src/omni_dofbot_mujoco/models/x3plus_rodillos_fijos.xml` |
| `X3_Plus_Simulation/src/omni_description_mujoco/` | `src/omni_description_mujoco/` |
| `Documentación - Conversion URDF a MuJoCo.md` | `docs/Conversion_URDF_a_MuJoCo.md` |

`omni_description_mujoco` declara el mismo nombre de paquete que `omni_description`
(`<name>omni_description</name>`), así que lleva un `COLCON_IGNORE` para que el
workspace compile. Se conserva como referencia de la geometría del rodillo.

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
  partir de 0° en las cuatro ruedas. En el CAD (`omni_mujoco2.xml`) el rodillo 1
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
| Piso | caja con μ = 0.68 (μ2 = 0.4) | 2.42 × 2.42 m, plano con μ = 0.68; sin μ2 |
| Paredes | 0.30 m de alto, 0.03 m de espesor, μ = 0.5 | cuatro cajas estáticas |
| Pose de reinicio | (0, 0, 0.05), yaw = π/2 | igual |
| Marcadores | (0,0), (0,0.8), (0.35,0.8) | solo visuales |

La arena mide **2.42 × 2.42 m**. El AG sintoniza con movimiento en x, y y yaw:
la prueba P4 recorre 0.80 m en lateral desde el origen (hacia −X del mundo, con
el robot en yaw = π/2), y eso no cabe en una arena de 1.22 m de ancho. Con
2.42 m quedan unos 0.28 m entre el robot y la pared al final de P4.

`src/omni_dofbot_bringup/world/arena_pid_tuning.world` también se amplió a
2.42 × 2.42 m (piso y paredes este/oeste en ±1.215 m), para que Gazebo y MuJoCo
usen la misma arena. Las dimensiones de MuJoCo se cambian con
`build_model.py --arena_x ... --arena_y ...`.

### Interfaces ROS 2 del puente

| Interfaz | Tipo | Función |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | tiempo de simulación |
| `/joint_states` | `sensor_msgs/JointState` | ruedas, brazo y gripper (100 Hz) |
| `/wheel_velocity_controller/commands` | `std_msgs/Float64MultiArray` | `[fl, fr, br, bl]` rad/s |
| `/dofbot_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | interpolación lineal hasta `time_from_start` |
| `/dofbot_gripper_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | igual, para `grip_joint` |
| `/world/arena_pid_tuning/set_entity_pose` | `ros_gz_interfaces/SetEntityPose` | teletransporte: fija `qpos`, anula `qvel`, `mj_forward` |
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

No las cambié porque la lógica debía quedar igual, pero conviene revisarlas:

1. **La detección de volcadura nunca se activa.** `_odom_cb` lee roll y pitch de
   `/odom`, pero `Mecanum_odometry_node.py` solo publica yaw, así que roll y
   pitch siempre valen 0. Para detectarla de verdad el AG tendría que leer
   `/ground_truth/odom` (MuJoCo) o la pose de Gazebo.
2. **El robot se mueve después del teletransporte.** El AG reinicia el PID y el
   filtro del motor 0.8 s después de teletransportar. Mientras tanto el filtro
   (τ = 0.58 s) sigue moviendo las ruedas con la velocidad del segmento
   anterior, y el robot rueda unos 5 cm. No afecta al costo porque el origen se
   fija después, pero cada prueba no arranca en (0, 0). Llamar a
   `_reset_controller()` antes del teletransporte lo evitaría.
3. **`/odom` no ve el deslizamiento.** En P4 la curva de vy es idéntica a la de
   vx en P1, porque la odometría integra las ruedas. El deslizamiento lateral
   real de los rodillos se mide con `/ground_truth/odom`.

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
- `omni_dofbot_controller.launch.py` del repositorio no tiene el argumento `world:=` que menciona el README del AG; si su versión local lo tiene, falta subirla.
- El piso es un plano infinito, no una caja; dentro de las paredes es equivalente.
- La dirección con μ2 = 0.4 de Gazebo no tiene equivalente; se calibra con datos reales.
- Coordinar con Marino cómo usa el AG `/ground_truth/odom`.
- `src/robot-x-main3` está registrado como submódulo pero no hay `.gitmodules`; queda vacío al clonar.
- Plan B de la guía: si T2–T4 dejan de pasar con el modelo calibrado, reportar MuJoCo con ruedas simplificadas como limitación.
