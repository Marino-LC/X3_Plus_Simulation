# Implementación: error de estimación de pose de objetos (sin Nav2)

Este documento dice **qué quedó listo en el repositorio**, **cómo usarlo** y **qué falta** para cumplir la [guía](GUIA.md). El código vive en el paquete `src/vla_pose_eval`.

## 1. Estado

| Parte | Estado |
|---|---|
| Núcleo en Python puro (escenas, vistas, métricas, evaluador, reporte) | Implementado y probado: 34 pruebas con `pytest` |
| Flujo completo **sin Gazebo** (`dry-run` con el estimador oráculo) | Implementado y ejecutado |
| Nodos ROS 2, launch y xacro del robot con pose real | Escritos, **no ejecutados**: el entorno donde se prepararon no tiene ROS ni Gazebo. Ver la lista de verificación de la sección 6 |
| Línea base clásica (E1) | Hecha la parte geométrica (caja 2D + profundidad → 3D). **Falta conectar el detector** |
| Modelo aprendido (E2) | Solo la plantilla `VLAEstimator` |
| Objetos de la escena | Primitivas de ejemplo. **Falta sustituirlas por los objetos del RS3** |

## 2. Dónde está cada sección de la guía

| Guía | Archivo | Qué hace |
|---|---|---|
| §2 M1 Percepción | `vla_pose_eval/estimators/base.py` | Interfaz `Estimator`: recibe RGB, profundidad, `K` e instrucción; devuelve clase y pose en el marco óptico |
| §2 Línea base | `estimators/detector_depth.py` | `DetectorDepthEstimator.lift`: caja o máscara + profundidad → posición 3D. `detect` queda por implementar |
| §2 Modelo aprendido | `estimators/vla.py` | Plantilla |
| §2 M2 Fusión | `metrics.fuse`, `evaluate.py` | Agrupa por cercanía en `map`, promedia la posición y vota la clase |
| §3 Escena y verdad | `scene.py`, `config/objects.yaml`, `config/scenes.yaml` | Mundo SDF y `scene_gt.yaml` por tipo (S1, S2, S3) y semilla |
| §3 Verdad del robot | `omni_dofbot_description/urdf/omni_dofbot_vla.xacro` | `PosePublisher` de Gazebo → `/model/omni_dofbot/pose` |
| §4 Pasada | `nodes/recon_pass.py`, `config/views.yaml` | Brazo a observación, teletransporte, espera, captura, pose real |
| §4 Asociación | `metrics.associate` | Uno a uno por distancia, umbral 0.15 m |
| §5 Marcos y error | `geometry.py`, `metrics.error_components`, `evaluate.py` | Cámara → `base_footprint` → `map`; error radial, transversal y z; yaw módulo simetría |
| §6 Interfaces | `nodes/`, `launch/vla_pose_eval.launch.py` | Ver sección 4 |
| §7 Datos | `dataset.py`, `scenes.yaml` (`splits`), `objects.yaml` (`split`) | Formato en disco; separación por escena (semillas) y por objeto (variantes) |
| §8 Métricas y protocolo | `metrics.py`, `report.py`, `config/eval.yaml` (`targets`) | Media, desviación, mediana, P95, bootstrap por escena, Wilson, cortes |
| §9 E0 | `python3 -m vla_pose_eval.cli dry-run` | Produce el CSV con el error por objeto (por ahora sin Gazebo) |

## 3. Uso

### 3.1 Sin ROS (cualquier máquina con `numpy` y `PyYAML`)

```bash
cd src/vla_pose_eval
```

```bash
python3 -m pytest test -q
```

```bash
python3 -m vla_pose_eval.cli dry-run --out ../../vla_runs/dry --seeds 0 20
```

`dry-run` genera las escenas S1 a S3, simula las vistas con la cámara nominal, corre el estimador **oráculo** (la verdad más ruido conocido), evalúa y escribe `summary.md`. Valida el flujo y las métricas, no un modelo. Con `--kwargs '{}'` el oráculo no añade ruido y todos los errores deben salir en cero.

### 3.2 Con Gazebo (un ensayo)

Compilar desde la raíz del workspace:

```bash
colcon build --packages-select omni_description dofbot_description omni_dofbot_description vla_pose_eval
```

```bash
source install/setup.bash
```

Generar las escenas de prueba (semillas fijas 0 a 19 de `scenes.yaml`):

```bash
ros2 run vla_pose_eval vla_pose_eval generate-scenes --out ~/vla_runs/test --split test
```

Terminal 1, simulación de un ensayo:

```bash
ros2 launch vla_pose_eval vla_pose_eval.launch.py trial_dir:=$HOME/vla_runs/test/S1_seed0000
```

Terminal 2, pasada de reconocimiento (guarda `views.yaml` y `views/`):

```bash
ros2 run vla_pose_eval recon_pass --ros-args -p trial_dir:=$HOME/vla_runs/test/S1_seed0000
```

Estimar, evaluar y reportar (sin la simulación; sirve también en la máquina con GPU):

```bash
ros2 run vla_pose_eval vla_pose_eval run-estimator ~/vla_runs/test --estimator oracle
```

```bash
ros2 run vla_pose_eval vla_pose_eval evaluate ~/vla_runs/test --estimators oracle
```

```bash
ros2 run vla_pose_eval vla_pose_eval report ~/vla_runs/test --estimators oracle
```

Para comparar línea base y modelo aprendido se corre `run-estimator` una vez por estimador sobre los mismos ensayos y se pasan ambos nombres a `evaluate` y `report`: los dos leen las mismas imágenes de `views/`.

**No ejecutar `Mecanum_odometry_node.py` ni Nav2 en esta fase.** `ground_truth_publisher` publica `map → base_footprint` con la pose de Gazebo; la odometría publicaría `odom → base_footprint` y el marco tendría dos padres.

### 3.3 Modo en línea (opcional)

`recon_pass` con `-p online:=true` republica cada captura en `/vla/input/*` y espera un `Detection3DArray` en `/vla/estimates` con el mismo sello de tiempo. `estimator_node` envuelve cualquier `Estimator`. El tiempo que se registra en este modo incluye el transporte ROS; el de inferencia se mide dentro del estimador.

## 4. Interfaces ROS 2

| Interfaz | Tipo | Quién |
|---|---|---|
| `/camera/image`, `/camera/depth_image`, `/camera/camera_info` | `sensor_msgs` | Cámara RGB-D del mástil (puente de Gazebo) |
| `/mono_camera/image_raw`, `/mono_camera/camera_info` | `sensor_msgs` | Cámara monocular del brazo (no se usa por defecto) |
| `/model/omni_dofbot/pose` | `geometry_msgs/PoseStamped` | Pose real del robot (Gazebo) |
| TF `map → base_footprint` | `tf2` | `ground_truth_publisher` |
| TF `base_footprint → camera_PC_link` | `tf2` | `robot_state_publisher` |
| `/world/vla_pose_arena/set_pose` | `ros_gz_interfaces/srv/SetEntityPose` | Teletransporte entre vistas |
| `/ground_truth/objects` | `geometry_msgs/PoseArray` | `ground_truth_publisher` (marco `map`, orden de `scene_gt.yaml`) |
| `/ground_truth/markers` | `visualization_msgs/MarkerArray` | `ground_truth_publisher` (RViz) |
| `/vla/input/image`, `/vla/input/depth`, `/vla/input/camera_info` | `sensor_msgs` | `recon_pass` en línea |
| `/vla/estimates` | `vision_msgs/Detection3DArray` | `estimator_node` |
| `/vla/instruction` | `std_msgs/String` | Instrucción opcional |

En `Detection3DArray`: clase y confianza en `results[0].hypothesis`, pose en `bbox.center`, marco óptico de la cámara. Un cuaternión con los cuatro valores en cero significa «sin orientación». El valor por defecto de ROS es la identidad, que se leería como yaw 0: quien publique sin estimar orientación debe poner los ceros.

## 5. Decisiones tomadas

Están en `config/` y se pueden cambiar; conviene confirmarlas con el asesor.

1. **Punto de referencia del objeto: centro geométrico.** En las primitivas coincide con el origen del modelo en Gazebo.
2. **`map` es el marco del mundo de Gazebo.** No hay localización en esta fase.
3. **Objetos estáticos.** La verdad escrita por el generador es exacta porque nada se asienta. Para manipulación (Fase 2) se pone `static_objects: false` y habrá que leer la pose de los objetos de Gazebo.
4. **Un mundo SDF por escena.** Reproducible, pero hay que relanzar Gazebo en cada escena. Si resulta lento para generar datos de entrenamiento, la alternativa es mover un conjunto fijo de modelos con `set_pose`.
5. **Recall y objetos fuera de cuadro.** Por vista, el recall se calcula sobre los objetos que caen en el cuadro (los demás se marcan `OUT` y no cuentan). Para la pasada completa, un objeto que nunca se ve cuenta como omisión y baja la cobertura. Es mi lectura de las secciones 8 y 10 de la guía; con un FOV de 58° y vistas a 360°, contar cada vista sin el objeto como omisión daría un recall bajo por construcción.
6. **«En cuadro» es geométrico:** el centro del objeto se proyecta dentro de la imagen y dentro del rango de profundidad. No considera oclusiones.
7. **Distractores:** una estimación asociada a un distractor cuenta como falso positivo.
8. **Descomposición del error** (en `base_footprint`): radial sobre el rayo horizontal cámara → objeto, transversal perpendicular a ese rayo, y z. Los tres al cuadrado suman el error de posición al cuadrado.
9. **Fusión:** no usa la verdad para agrupar. Un falso positivo visto en una sola vista sobrevive a la fusión, así que la tasa de falsos positivos de la pasada sale más alta que la de las vistas.

## 6. Diferencias entre la guía y el repositorio

Encontradas al leer el URDF y los launch. Cambian cómo se aplica la guía.

1. **La cámara con profundidad no está en el brazo.** La guía dice que la cámara va en `arm_link_04` con un FOV de unos 62°. En el URDF, la cámara de `arm_link_04` es la **monocular** (`/mono_camera/image_raw`, FOV 0.7142 rad ≈ 41°, sin profundidad). La **RGB-D** está fija al chasis en el mástil (`camera_link`, FOV 1.0193 rad ≈ 58.4° horizontal y ≈ 45.5° vertical). El paquete usa la RGB-D del mástil. La pose de observación del brazo se sigue ordenando: fija la cámara monocular y mantiene el brazo en una posición conocida. Falta comprobar que el brazo no tapa la vista de la RGB-D.
2. **Nombres de tópicos.** La guía cita `/camera/image_raw` y `/camera_info`; el puente publica `/camera/image`, `/camera/depth_image` y `/camera/camera_info`.
3. **Distancia mínima útil ≈ 0.9 m.** La RGB-D mira horizontal desde unos 0.53 m de altura y su profundidad empieza en 0.6 m (`<depth_camera><clip><near>`). Un objeto a 0.2 m de altura solo entra en cuadro a partir de unos 0.86 m. Los objetos quedan entre 0.9 y 2.2 m del robot, donde un cubo de 4 cm ocupa unos 15 píxeles (y el de 2.5 cm de `arena_world`, unos 10). Si la manipulación necesita estimar de cerca, hay que inclinar la cámara o usar la monocular del brazo.
4. **Marco de la imagen.** El sensor declara `gz_frame_id = camera_link` (x al frente), que no es un marco óptico. El marco óptico es `camera_PC_link`, y es el que usa el paquete.
5. **Teletransporte.** `ag_motion_tests.py` llama al servicio ROS `/world/<mundo>/set_entity_pose`, pero ningún launch lo puentea, así que en la práctica usa el respaldo `gz service`. El launch nuevo puentea `/world/vla_pose_arena/set_pose` y `recon_pass` conserva el mismo respaldo.
6. **`arena_world.world` no carga los sistemas de Gazebo** (sensores, órdenes de usuario); `arena_pid_tuning.world` sí. Los mundos generados los incluyen.
7. **Plano del LIDAR ≈ 0.12 m.** `laser_link` queda a 0.0815 + 0.0376 m del suelo según el URDF. Los pedestales generados miden entre 0.15 y 0.30 m.
8. **Tamaño de la arena.** Los mundos y el mapa usan 2.7 × 2.7 m; `waypoints.yaml` todavía dice 1.22 × 2.42 m. El generador usa 2.7 × 2.7 m.

## 7. Pendientes por etapa

### E0. Prerrequisitos

Verificar en simulación, en este orden:

- [ ] `colcon build` del paquete `vla_pose_eval`.
- [ ] El launch levanta Gazebo con `scene.world`, el robot y los controladores.
- [ ] `ros2 topic echo /model/omni_dofbot/pose --once` devuelve la pose del robot. Si no llega, revisar los parámetros de `PosePublisher` en `omni_dofbot_vla.xacro`. `recon_pass` avisa y usa la pose ordenada (queda anotado en `views.yaml` como `robot_pose_source: commanded`).
- [ ] El servicio `/world/vla_pose_arena/set_pose` existe en ROS; si no, confirmar que el respaldo `gz service` mueve al robot.
- [ ] `ros2 run tf2_ros tf2_echo base_footprint camera_PC_link` coincide con `camera.xyz_in_base` de `eval.yaml` (−0.035, 0, 0.5315).
- [ ] La profundidad llega como `32FC1` en metros y el brazo no tapa la imagen.
- [ ] Un ensayo completo con el oráculo **sin ruido** da error ≈ 0. Si no, hay un fallo de marcos. Ese es el criterio de salida de E0.
- [ ] Sustituir los objetos de `objects.yaml` por los del RS3 y definir su simetría.

### E1. Línea base clásica

- [ ] Elegir detector (Grounding DINO, OWLv2 o YOLO-World) tras revisar licencia y velocidad en la Jetson.
- [ ] Implementar `detect` en una subclase de `DetectorDepthEstimator`.
- [ ] Fijar `center_offsets` por clase: la profundidad mide la superficie visible y la referencia es el centro, lo que deja un sesgo radial de medio objeto (1 a 3 cm) si no se corrige.
- [ ] Criterio: mediana ≤ 0.05 m en S1.

### E2. Modelo aprendido

- [ ] Generar el conjunto de entrenamiento con `--split train` y la pasada de reconocimiento.
- [ ] Implementar `VLAEstimator` (o una subclase de `DetectorDepthEstimator` si el modelo predice cajas).
- [ ] Aplicar la regla de decisión de la sección 2 de la guía con `report --estimators baseline vla`.

### E3 y E4

- [ ] Correr S2 y S3; revisar recall y falsos positivos.
- [ ] Comparar el error con y sin fusión (tablas «Por vista» y «Pasada completa» del reporte).
- [ ] Ajustar `views.yaml` si la cobertura baja de 100 % (`vla_pose_eval check-views <dir>`).

### Fuera de lo implementado

- Aleatorización de texturas y fondos reales (solo hay luz y colores).
- Estimación de orientación en la línea base.
- Ruido de profundidad de la Astra real.
- E5 (hardware) y todo lo de la sección 11 de la guía (goal pose, Nav2).
