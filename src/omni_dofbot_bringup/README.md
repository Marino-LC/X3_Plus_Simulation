# omni_dofbot_bringup — Sintonización PID de la base mecanum por Algoritmo Genético

Este paquete pone en marcha en Gazebo (gz sim) el manipulador móvil **omni_dofbot**: una base
omnidireccional con ruedas mecanum (tipo Yahboom ROSMASTER X3 Plus) y un brazo Dofbot de 5 GDL con
pinza. Su función principal es **sintonizar las ganancias PID (Kp, Ki, Kd) del lazo de velocidad de
las ruedas con un Algoritmo Genético (AG)**. Para que las ganancias se puedan pasar al robot físico,
el PID se simula en el mismo dominio numérico que el firmware del fabricante.

---

## 1. Idea general del experimento

```
          ┌──────────────────────── ag_motion_tests.py (AG / DEAP) ───────────────────────┐
          │  población de [Kp,Ki,Kd] → para cada individuo: set_parameters → batería P1..P4│
          │  → fitness (ITAE + tiempo + error final) → selección / cruza / mutación         │
          └───────┬───────────────────────────┬──────────────────────────┬────────────────┘
       /cmd_vel   │       set_parameters      │ reset_controller_state   │ teleport (gz set_pose)
                  ▼       (kp, ki, kd)        ▼                          ▼  + reset_pose
   ┌───────────────────────────────┐                          ┌────────────────────────┐
   │   mecanum_kinematic_node.py   │ /wheel_velocity_         │   Gazebo (gz sim)      │
   │  cinemática inversa + PID     │──controller/commands────▶│   omni_dofbot +        │
   │  incremental + planta motor   │                          │   arena_pid_tuning     │
   └───────────────▲───────────────┘                          └──────────┬─────────────┘
                   │ /joint_states (velocidad real de ruedas)            │
                   └─────────────────────────────────┬───────────────────┘
                                                     ▼
                                  ┌──────────────────────────────────┐
                                  │   Mecanum_odometry_node.py       │── /odom ──▶ AG (fitness)
                                  │  cinemática directa + integración│
                                  └──────────────────────────────────┘
```

1. El AG manda las ganancias de un individuo al nodo de cinemática con `set_parameters`.
2. Se ejecuta una batería de 4 pruebas de movimiento. En cada una el robot se teletransporta al
   origen y se reinician la odometría y el estado del PID.
3. Mientras el robot se mueve, el **brazo Dofbot hace pick & place** de un lado a otro
   (coreografía fija). Así la base se sintoniza con la perturbación dinámica que provoca la carga
   del brazo en movimiento.
4. La posición real se mide con `/odom` y con ella se calcula un costo por prueba. El fitness del
   individuo es la suma ponderada de esos costos (**se minimiza**).
5. Al terminar, se repite la batería con el mejor individuo para registrar sus series de tiempo y
   generar gráficas y un JSON con los resultados.

No se usan Nav2, AMCL ni mapa: el robot recibe `/cmd_vel` directamente.

---

## 2. Nodos principales

### 2.1 `src/mecanum_kinematic_node.py`: cinemática inversa + PID + planta del motor

Recibe `/cmd_vel` y publica la velocidad de cada rueda en `/wheel_velocity_controller/commands`.
Gazebo solo acepta rad/s, así que el lazo completo del firmware se modela dentro de este nodo:

```
omega_ref (rad/s) ──×r·1000──▶ v_ref (mm/s)
error = v_ref − v_real (mm/s)
PID INCREMENTAL (igual al firmware STM32, sin normalizar por dt, 100 Hz):
   ΔPWM = Kp·(e − e₁) + Ki·e + Kd·(e − 2e₁ + e₂);   pwm += ΔPWM;   clamp ±2000
──× motor_gain_k──▶ ω (rad/s)          ganancia estática del motor (caracterizada)
──FirstOrderLag(τ≈0.58 s)──▶ ω_filt     dinámica electromecánica
──zona muerta (breakaway)──▶ fricción estática según el tipo de movimiento (X, Y, yaw)
──clamp ±max_wheel_vel──▶ comando a Gazebo
```

- **Cinemática inversa** mecanum estándar con `r=0.040 m`, `lx=0.110 m`, `ly=0.102 m`.
- **Banda muerta de error** (`error_deadband_mm_s`): evita que el PID oscile por el ruido de
  cuantización.
- **Watchdog** (`cmd_vel_timeout = 0.5 s`): si no llega `/cmd_vel`, la referencia baja a cero.
- **Servicio** `~/reset_controller_state` (`std_srvs/Empty`): reinicia el acumulador PWM, el
  filtro y el estado de fricción. El AG lo llama después de cada teleport.
- Las ganancias `kp`, `ki` y `kd` son parámetros que se cambian en tiempo de ejecución.

### 2.2 `src/Mecanum_odometry_node.py`: odometría

Lee `/joint_states`, aplica la cinemática directa mecanum, integra la pose y publica `/odom` y el TF
`odom → base_footprint`. Su servicio `~/reset_pose` (`std_srvs/Empty`) devuelve la pose a
`(0, 0, π/2)`, que es la misma orientación con la que el AG teletransporta al robot.

### 2.3 `src/ag_motion_tests.py`: Algoritmo Genético + batería de pruebas

Es el nodo principal del experimento. Está implementado con [DEAP](https://deap.readthedocs.io).

---

## 3. Batería de pruebas

Todas las pruebas empiezan con un **teleport** a `(0, 0)` con yaw = 90°, una pausa para que el robot
se asiente, una revisión de volcadura y el arranque del hilo del brazo (pick & place). Distancias y
velocidades: `DIST_X = DIST_Y = 0.80 m`, `DIST_RETURN = 0.30 m`, `VX_REF = VY_REF = 0.40 m/s`,
`WZ_REF = 1.0 rad/s`.

| Prueba | Movimiento | Qué evalúa | Costo |
|---|---|---|---|
| **P1: recta** | avanza 0.80 m y regresa 0.80 m en X | seguimiento de velocidad longitudinal | `0.60·ITAE/0.05 + 0.30·t/t_ref + 0.10·err_final/POS_TOL` |
| **P2: rotación pura** | gira +90° y −90° sin desplazarse | modo de giro, deriva de posición | `0.55·err_yaw/(2·YAW_TOL) + 0.25·t/t_ref + 0.20·err_final/POS_TOL` |
| **P3: combinada** | avanza 0.80 m, gira −90°, avanza 0.30 m | trayectoria compuesta; error contra la pose final esperada | `0.45·ITAE/0.08 + 0.20·t/t_ref + 0.15·err_yaw/YAW_TOL + 0.20·err_final/POS_TOL` |
| **P4: lateral** | 0.80 m a la izquierda y regreso en Y | desplazamiento lateral propio de las mecanum | igual que P1, usando Y |

- **ITAE** (∫ t·|e| dt): se integra en el callback de `/odom` con el tiempo de simulación. `e` es la
  distancia euclídea entre la pose actual y la pose objetivo del segmento.
- Cada costo está **normalizado** con una referencia (ITAE de referencia, tiempo ideal
  `distancia/velocidad`, tolerancias `POS_TOL = 0.04 m` y `YAW_TOL = 0.05 rad`). Así las pruebas son
  comparables entre sí.
- **Penalizaciones** (`PENALTY_TO = 50`): se suman si un segmento no llega a su meta antes del
  timeout (8 s), si hay overshoot de más de 12 cm, si la deriva lateral supera 1 m o si el robot
  se vuelca (roll/pitch > 15°).

**Fitness total** (a minimizar):

```
fitness = 0.25·P1 + 0.20·P2 + 0.30·P3 + 0.25·P4
```

### Validez de la medición: Real Time Factor (RTF)

El PID corre sin normalizar por dt, así que si Gazebo va lento el resultado sale distorsionado. Por
eso la batería se ejecuta dentro de `_gated()`:

- Se mide el RTF medio (Δt_sim / Δt_pared). Si es **< 0.90**, la evaluación se descarta y se repite.
- Durante la evolución, después de **3 intentos** fallidos se descarta el individuo y su lugar lo
  ocupa uno aleatorio nuevo. Si hay **10 descartes seguidos**, el experimento se aborta porque
  Gazebo está sobrecargado.

---

## 4. Estructura del Algoritmo Genético

| Elemento | Configuración |
|---|---|
| Individuo | `[Kp, Ki, Kd]` (lista de 3 reales, redondeados a 3 decimales) |
| Rangos | `Kp ∈ [0, 5]`, `Ki ∈ [0, 1]`, `Kd ∈ [0, 2]` (anclados a las ganancias del fabricante: 0.8 / 0.06 / 0.5) |
| Población | `POP_SIZE = 35` |
| Generaciones por ronda | `N_GEN = 30` |
| Selección | Torneo, `tournsize = 3` |
| Cruza | `cxBlend(α = 0.5)`, probabilidad `0.55` |
| Mutación | `mutGaussian(μ=0, σ=[0.40, 0.08, 0.16], indpb=0.3)`, probabilidad `0.25` (σ ≈ 7–8 % del rango de cada gen) |
| Restricción de rango | decorador `_bounded`: recorta cada gen a su rango después de cruzar o mutar |
| Elitismo | `HallOfFame(5)`: los 5 mejores se reinyectan a la población en cada generación |
| Objetivo | `FitnessMin` (minimizar) |

### Bucle por generación

```
offspring = varAnd(pop, CX_PROB, MUT_PROB)       # cruza + mutación
evaluar individuos nuevos (con control de RTF)
pop = torneo(offspring, POP_SIZE − 5) + élites    # elitismo
actualizar HallOfFame y registrar estadísticas (min / media / max / mejor)
```

### Rondas con semilla (reinicio iterativo)

1. **Ronda 1**: la población inicial es aleatoria e incluye la semilla `[Kp=1, Ki=0, Kd=0]` (valores
   por defecto del nodo). Se corren las 30 generaciones.
2. El mejor global pasa a ser la **semilla de la ronda siguiente**, con una población nueva.
3. En las rondas ≥ 2 se revisa la **primera generación**: si el mejor fitness no mejora al menos
   **10 %** respecto a la semilla, se da por convergido y el experimento termina.
4. `MAX_ROUNDS = 10` es solo un tope de seguridad.

Al final se reporta el **mejor individuo global**, se repite la batería con él (`record_best`) y se
generan las salidas.

---

## 5. Salidas

Se escriben en el directorio desde el que se lanzó el nodo:

| Archivo | Contenido |
|---|---|
| `ag_results.json` | configuración, mejor individuo, resumen de rondas, controladores descartados por RTF, estadísticas por generación y series de tiempo completas de la corrida del mejor |
| `ag_results.png` | evolución del fitness, ganancias del mejor por generación, vx/wz/vy ref vs real (P1, P2, P4), error de posición (P3), dispersión Kp–Ki y pose X/Y/Yaw deseada vs obtenida |
| `ag_results.html` | las mismas gráficas en versión interactiva (Plotly); se abre en el navegador |

Durante la corrida también se publican tópicos para verlos en vivo con `rqt_plot` o PlotJuggler:
`ag_live_error`, `ag_live_vel_ref`, `ag_live_vel_real`, `ag_live_{x,y,yaw}_{ref,real}`.

---

## 6. Cómo ejecutar

Dependencias de Python:

```bash
pip install deap matplotlib plotly --break-system-packages
```

Compilar el workspace:

```bash
colcon build --packages-select omni_dofbot_description omni_dofbot_bringup
```

```bash
source install/setup.bash
```

En cuatro terminales, cada una con el workspace cargado (`source install/setup.bash`):

**Terminal 1: simulación.** Usar la arena de sintonización; el AG teletransporta en el mundo
`arena_pid_tuning`.

```bash
ros2 launch omni_dofbot_bringup omni_dofbot_controller.launch.py world:=arena_pid_tuning.world
```

**Terminal 2: cinemática + PID**

```bash
ros2 run omni_dofbot_bringup mecanum_kinematic_node.py
```

**Terminal 3: odometría** (debe estar activa antes de iniciar el AG)

```bash
ros2 run omni_dofbot_bringup Mecanum_odometry_node.py
```

**Terminal 4: algoritmo genético**

```bash
ros2 run omni_dofbot_bringup ag_motion_tests.py
```

> Sin el argumento `world`, el launch carga `arena_world.world`, el mundo que usan Nav2 y
> `waypoints_nav.py`.

Para probar a mano unas ganancias encontradas:

```bash
ros2 param set /mecanum_kinematic_node kp 1.234
```

---

## 7. Estructura del paquete

```
omni_dofbot_bringup/
├── launch/
│   ├── omni_dofbot_controller.launch.py   # Gazebo + robot + controladores (ruedas, brazo, pinza, JSB) + RViz
│   ├── omni_dofbot_diagnostic.launch.py   # variante para diagnóstico
│   └── nav2_arena.launch.py               # Nav2 sobre arena_world (trabajo posterior)
├── src/
│   ├── mecanum_kinematic_node.py          # ★ cinemática inversa + PID incremental + planta del motor
│   ├── Mecanum_odometry_node.py           # ★ odometría por cinemática directa
│   ├── ag_motion_tests.py                 # ★ AG (DEAP) + batería de pruebas P1–P4
│   ├── zn_tuner.py / zn_tuner_openloop.py # sintonización Ziegler–Nichols (método de comparación)
│   ├── ag_tuner.py                        # versión previa del AG (histórica, no se instala)
│   └── waypoints_nav.py                   # seguimiento de waypoints con Nav2
├── world/
│   ├── arena_pid_tuning.world             # ★ arena plana para la sintonización
│   ├── arena_world.world                  # arena con mapa para Nav2
│   └── Mundo_mesa_y_cajas.world
├── config/   (nav2_params.yaml, waypoints.yaml)
├── maps/     (arena_map.pgm/.yaml, generate_map.py)
└── rviz/     (omni_dofbot_trayectory_rviz.rviz)
```

---

## 8. Limitaciones conocidas

- El PID incremental no normaliza por dt, igual que el firmware. Por eso depende de que la
  simulación corra cerca del tiempo real; el filtro de RTF existe para descartar las evaluaciones
  que no cumplan.
- El modelo del motor usa una sola ganancia estática lineal para las 4 ruedas y una zona muerta
  promedio. No se modela la saturación por voltaje de batería ni el lazo de corrección de yaw del
  firmware.
- Las mallas de las ruedas mecanum no modelan cada rodillo, así que la física del desplazamiento
  lateral (P4) en Gazebo es una aproximación.
