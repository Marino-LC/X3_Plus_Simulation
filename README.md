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

## Estado (14 nov 2026, MuJoCo 4.0.1)

| Parte | Estado |
|---|---|
| Modelo con rodillos libres, actuadores, masa 4.8 kg | Implementado |
| Arena de 2.42 × 2.42 m (mismos materiales que `arena_pid_tuning.world`) | Implementada |
| T1–T5, T7, T8, T9 (sin ROS) | **Pasan** con los parámetros de partida (resultados abajo) |
| Puente ROS 2 (`mujoco_bridge_node.py`) y launch | Escritos, **sin probar con ROS 2** (el entorno donde se generó no tiene ROS) |
| T6, T10, AG de humo y AG completo | Pendientes: requieren ROS 2 + Gazebo (protocolo abajo) |
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
    src/mujoco_bridge_node.py       puente ROS 2
    launch/mujoco_ag.launch.py      puente + nodo cinemático + odometría
    test/test_acceptance.py         las mismas pruebas con pytest
  omni_dofbot_bringup/            AG, ZN, nodo cinemático, odometría, mundos Gazebo
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

El `arena_pid_tuning.world` que está en este repositorio todavía mide
1.22 × 2.42 m: para que T10 compare lo mismo, Gazebo debe usar un mundo de
2.42 × 2.42 m. Las dimensiones se cambian con `build_model.py --arena_x ... --arena_y ...`.

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

### Con ROS 2 Jazzy — para el puente, T6, T10 y el AG

En Ubuntu 24.04 `pip` necesita `--break-system-packages` (igual que `deap`):

```bash
python3 -m pip install mujoco deap --break-system-packages
```

```bash
sudo apt install ros-jazzy-ros-gz-interfaces
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

| Prueba | Procedimiento | Criterio | Resultado (14 nov, valores de partida) |
|---|---|---|---|
| T1 Reposo | soltar, esperar 2 s, medir 1 s | velocidad < 1e-3, z estable, sin deriva | pasa: 5.2e-4, z varía 1.2 µm, deriva 0.02 mm |
| T2 Avance x | 0.2 m/s, 2 s | error < 5 % | pasa: 0.3993 m (0.18 %) |
| T3 Lateral y | 0.2 m/s, 2 s | error < 5 % | pasa: 0.3875 m (3.14 %) |
| T4 Rotación | wz = 1 rad/s, 2 s | error < 5 % | pasa: 1.9874 rad (0.63 %) |
| T5 Diagonal | vx = vy = 0.2 m/s, 2 s | 45° ± 3° | pasa: 44.57° |
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
| 0.3 | 0.383 | 0.380 | 0.243 | 0.158 |
| 0.68 | 0.389 | 0.387 | 0.356 | 0.233 |
| 1.0 | 0.391 | 0.389 | 0.373 | 0.262 |

Misma tendencia que el esqueleto de la guía: el amortiguamiento del rodillo
pesa más que μ en el movimiento lateral.

### Puente ROS 2: verificación inicial

Esta parte no se ha ejecutado todavía. Primer arranque:

```bash
ros2 launch omni_dofbot_mujoco mujoco_ag.launch.py viewer:=true
```

Comprobar, en otra terminal:

```bash
ros2 topic hz /clock
```

```bash
ros2 topic echo /joint_states --once
```

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

```bash
ros2 service call /world/arena_pid_tuning/set_entity_pose ros_gz_interfaces/srv/SetEntityPose "{entity: {name: omni_dofbot, type: 2}, pose: {position: {z: 0.05}, orientation: {z: 0.7071068, w: 0.7071068}}}"
```

Se espera: `/clock` cerca de 1000 Hz, las cuatro ruedas y las juntas del brazo
en `/joint_states`, el robot avanzando hacia +Y del mundo con `cmd_vel` en x, y
el robot de vuelta en el origen tras el servicio. Si avanza al revés, el signo
está en los ejes de las juntas de rueda (guía §7, punto 4).

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

### AG de humo y AG completo

`ag_motion_tests.py` fija el tamaño en dos constantes (`POP_SIZE` y `N_GEN`);
no tiene parámetros de línea de comandos. Para el AG de humo hay
que ponerlas temporalmente en 8 y 3.

```bash
ros2 launch omni_dofbot_mujoco mujoco_ag.launch.py
```

```bash
ros2 run omni_dofbot_bringup ag_motion_tests.py
```

El launch ya incluye el nodo cinemático y la odometría. El AG de humo debe
terminar sin timeouts de teletransporte ni volcaduras antes de lanzar el
completo (estimado en la guía: 2 a 4 horas en tiempo real).

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

- Probar el puente con ROS 2 Jazzy y correr T6, T10 y el AG de humo.
- Sustituir la masa concentrada del chasis por inercias reales.
- Confirmar el par máximo del motor (`wheel_max_torque`) y `wheel_kv` con T6.
- Subir al repositorio el mundo de Gazebo de 2.42 × 2.42 m y las versiones vigentes de `ag_motion_tests.py` (con P4) y `mecanum_kinematic_node.py`; las copias de `src/omni_dofbot_bringup/` son anteriores.
- El piso es un plano infinito, no una caja; dentro de las paredes es equivalente.
- La dirección con μ2 = 0.4 de Gazebo no tiene equivalente; se calibra con datos reales.
- Coordinar con Marino cómo usa el AG `/ground_truth/odom`.
- `src/robot-x-main3` está registrado como submódulo pero no hay `.gitmodules`; queda vacío al clonar.
- Plan B de la guía: si T2–T4 dejan de pasar con el modelo calibrado, reportar MuJoCo con ruedas simplificadas como limitación.
