# Implementación: error de estimación de pose de objetos (sin Nav2)

Este documento dice **qué quedó listo en el repositorio**, **cómo usarlo** y **qué falta** para cumplir la [guía](GUIA.md). El código vive en el paquete `src/vla_pose_eval`.

## 1. Estado

| Parte | Estado |
|---|---|
| Núcleo en Python puro (escenas, vistas, métricas, evaluador, reporte, fusión de cámaras, modelo de ruido) | Implementado y probado: 63 pruebas con `pytest` |
| Flujo completo **sin Gazebo** (`dry-run` con los estimadores oráculo) | Implementado y ejecutado |
| Cámara del Dofbot en la identificación (sección 3) | Implementado y probado en el núcleo; captura en Gazebo sin ejecutar |
| Ruido de cámaras (sección 4) | Modelo de profundidad probado; SDF y nodo ROS sin ejecutar |
| Nodos ROS 2, launch y xacro | Escritos y revisados con un analizador estático, **no ejecutados**: el entorno donde se prepararon no tiene ROS ni Gazebo. Ver la lista de verificación de la sección 9 |
| Línea base clásica (E1) | Hecha la geometría (caja 2D + profundidad → 3D) y la fusión con la cámara del Dofbot. **Falta conectar el detector** |
| Modelo aprendido (E2) | Solo la plantilla `VLAEstimator` |
| Objetos de la escena | Primitivas de ejemplo. **Falta sustituirlas por los objetos del RS3** |

## 2. Las dos cámaras

| | Astra (RGB-D) | Cámara del Dofbot |
|---|---|---|
| Montaje | Mástil, fija al chasis (`camera_link`) | `arm_link_04`, se mueve con el brazo |
| Imagen | 1280 × 720, FOV 60° × 36° | 640 × 480, FOV 41° × 31° |
| Profundidad | Sí, desde 0.6 m | No |
| Posición con el brazo en `[0, 0, -1.57, 0, 0]` | (−0.035, 0, 0.53) m en `base_footprint`, mirando al frente | (0.236, −0.010, 0.350) m, mirando al frente (yaw −4°) |
| Distancia mínima para ver un objeto a 0.17 / 0.22 / 0.32 m de altura | 1.11 / 0.96 / 0.65 m | 0.64 / 0.46 / 0.11 m |
| Marco óptico | `camera_PC_link` | `mono_optical_link` (añadido en `omni_dofbot_vla.xacro`) |
| Tópicos | `/camera/image`, `/camera/depth_image`, `/camera/camera_info` | `/mono_camera/image_raw`, `/mono_camera/camera_info` |

Las dos cámaras se **complementan**. La Astra da posición métrica a partir de ~1 m. La del Dofbot cubre el campo cercano que la Astra no ve y tiene más resolución angular.

### 2.1 Cómo participa la cámara del Dofbot

En cada vista, `MultiCameraFusion` (`estimators/multicam.py`) combina:

1. **Objeto visto por ambas:** la detección 3D de la Astra se proyecta en la imagen del Dofbot y se empareja con la caja 2D que la contiene. La **clase** se decide por voto ponderado por confianza; la **posición** sigue siendo la de la Astra. Fuente `rgbd+mono`.
2. **Visto por la Astra y no confirmado por el Dofbot**, estando en su cuadro: la confianza se multiplica por `unconfirmed_factor` (1.0 por defecto, sin efecto). Fuente `rgbd`.
3. **Visto solo por el Dofbot:** posición **monocular por tamaño aparente**, z = f · S / √(ancho · alto de la caja). S es el tamaño aparente esperado de la clase según `objects.yaml`, promediado sobre el giro y corregido por la elevación del rayo, que sale de la calibración. Fuente `mono`. Se descarta si la caja toca el borde de la imagen.

Todas las estimaciones se expresan en el marco óptico de la Astra, así que el evaluador no cambia. Las extrínsecas de ambas cámaras salen de TF: son calibración, no verdad de referencia.

**Límite de la posición monocular:** sin conocer el giro del objeto, su tamaño aparente es ambiguo. Un cubo visto de frente se ve ~20 % más angosto que visto en diagonal, y el error puede llegar a ~20 % de la distancia. Con el oráculo sin ruido, la mediana es 2.8 cm y el P95 9.6 cm. Por eso el reporte separa el error por fuente.

### 2.2 Uso desde un estimador

- `Frame.mono` trae la imagen, `K` y la extrínseca de la cámara del Dofbot; `Frame.T_base_cam` trae la de la Astra.
- Un estimador declara `cameras = ('rgbd', 'mono')` para que el evaluador cuente la visibilidad de ambas.
- `Detection.source` dice qué cámaras produjeron cada estimación.
- `MultiCameraDetectorEstimator` es la línea base con las dos cámaras: se implementa `detect` una vez y se aplica a ambas imágenes. Para usar otro detector en la cámara del Dofbot, se sobrescribe `detect_mono`.

## 3. Ruido de cámaras

Dos capas, documentadas en `config/noise.yaml`:

| Qué | Dónde | Modelo |
|---|---|---|
| Imagen RGB de la Astra | SDF (`omni_description/urdf/camera_sensor.xacro`) | Gaussiano, stddev 0.007 (intensidad 0–1, ~1.8 niveles de 8 bits) |
| Imagen RGB del Dofbot | SDF (`dofbot_description/urdf/monocular_camera_sensor.xacro`) | Gaussiano, stddev 0.012 (cámara USB económica) |
| Profundidad de la Astra | Nodo `depth_noise` (`vla_pose_eval/noise.py`) | Luz estructurada, ver abajo |

El ruido de imagen va en el SDF porque Gazebo lo aplica en el render (GPU) y lo ve cualquier nodo. El de profundidad va en un nodo porque Gazebo solo ofrece ruido gaussiano constante, y una cámara de luz estructurada real tiene además:

- **Desviación que crece con z²:** σ(z) = 0.5 mm + 1.6 mm/m² · z², es decir 2.1 mm a 1 m y 6.9 mm a 2 m. La mitad de la varianza es espacialmente correlacionada, en manchas de ~8 px.
- **Escalones de profundidad** por cuantización de disparidad (f · B / d con d a 1/8 de píxel): ~3 mm a 1 m y ~12 mm a 2 m. Se usa la focal **nativa** del sensor (570 px a 640 × 480), no la del render.
- **Redondeo a milímetros**, porque la Astra publica 16UC1.
- **Huecos** (NaN): 0.2 % aleatorios y 80 % en el lado lejano de los bordes (sombra del proyector), además de los rangos fuera de 0.6–8 m.

`depth_noise` se intercala con el launch: el puente publica la profundidad limpia en `/sim/camera/depth_image` y el nodo publica la ruidosa en `/camera/depth_image`. Con `depth_noise:=false`, la limpia va directa a `/camera/depth_image`. El ruido de cada imagen depende de la semilla y del sello de tiempo, así que es reproducible. `recon_pass` guarda en `views.yaml` la configuración usada, o `depth_noise: null` si el nodo no corría.

**Los valores son propuestas** tomadas de la literatura de cámaras tipo Kinect v1 / Astra. Antes de reportar la brecha sim a real, conviene medir la Astra real: un plano a 0.8, 1.2, 1.6 y 2.0 m, y ajustar `sigma_*` y `subpixel`.

La línea base sigue recuperando el centro de un cubo a 1.2 m con error menor a 1 cm con este ruido (prueba `test_baseline_lift_with_noisy_depth`).

## 4. Dónde está cada sección de la guía

| Guía | Archivo | Qué hace |
|---|---|---|
| §2 M1 Percepción | `estimators/base.py` | Interfaz `Estimator`: recibe las imágenes de ambas cámaras, profundidad, `K`, extrínsecas e instrucción; devuelve clase y pose en el marco óptico de la Astra |
| §2 Línea base | `estimators/detector_depth.py`, `estimators/multicam.py` | Caja 2D + profundidad → 3D; fusión con la cámara del Dofbot. `detect` queda por implementar |
| §2 Modelo aprendido | `estimators/vla.py` | Plantilla |
| §2 M2 Fusión | `metrics.fuse`, `evaluate.py` | Agrupa por cercanía en `map`, promedia la posición y vota la clase |
| §3 Escena y verdad | `scene.py`, `config/objects.yaml`, `config/scenes.yaml` | Mundo SDF y `scene_gt.yaml` por tipo (S1, S2, S3) y semilla |
| §3 Verdad del robot | `omni_dofbot_description/urdf/omni_dofbot_vla.xacro` | `PosePublisher` de Gazebo → `/model/omni_dofbot/pose` |
| §4 Pasada | `nodes/recon_pass.py`, `config/views.yaml` | Brazo a observación, teletransporte, espera, captura de las dos cámaras, pose real |
| §4 Asociación | `metrics.associate` | Uno a uno por distancia, umbral 0.15 m |
| §5 Marcos y error | `geometry.py`, `metrics.error_components`, `evaluate.py` | Cámara → `base_footprint` → `map`; error radial, transversal y z sobre el rayo de la cámara de origen; yaw módulo simetría |
| §6 Interfaces | `nodes/`, `launch/vla_pose_eval.launch.py` | Ver sección 6 |
| §7 Datos | `dataset.py`, `scenes.yaml` (`splits`), `objects.yaml` (`split`) | Formato en disco; separación por escena (semillas) y por objeto (variantes) |
| §8 Métricas y protocolo | `metrics.py`, `report.py`, `config/eval.yaml` (`targets`) | Media, desviación, mediana, P95, bootstrap por escena, Wilson, cortes; identificación y cobertura por cámara |
| §9 E0 | `python3 -m vla_pose_eval.cli dry-run` | Produce el CSV con el error por objeto (por ahora sin Gazebo) |
| §10 Sim a real | `config/noise.yaml`, `noise.py`, `nodes/depth_noise.py` | Ruido de imagen y de profundidad |

## 5. Uso

### 5.1 Sin ROS (cualquier máquina con `numpy` y `PyYAML`)

```bash
cd src/vla_pose_eval
```

```bash
python3 -m pytest test -q
```

```bash
python3 -m vla_pose_eval.cli dry-run --out ../../vla_runs/dry --seeds 0 20
```

`dry-run` hace lo siguiente:

1. Genera las escenas S1 a S3.
2. Simula las vistas con las cámaras nominales.
3. Corre dos oráculos (la verdad más un ruido conocido) con el mismo ruido: `oracle` solo con la Astra y `oracle_multicam` con las dos cámaras.
4. Evalúa y escribe `summary.md`.

Valida el flujo y las métricas, no un modelo. Con `--kwargs '{}'` los oráculos no añaden ruido. En ese caso, todo lo que tiene profundidad debe salir con error cero; lo solo monocular conserva el error del tamaño aparente.

Resultado con las semillas 0 a 19 (60 escenas). Es un ejemplo del flujo con el oráculo, **no un resultado del modelo**:

| | `oracle` (solo Astra) | `oracle_multicam` (Astra + Dofbot) |
|---|---|---|
| Cobertura de la pasada | 97.1 % | 100 % |
| Recall de la pasada | 95.0 % | 97.9 % |
| Exactitud de clase, objetos vistos por ambas cámaras | — | 97.9 % (frente a 90.1 % con una sola) |
| Error de posición por vista, mediana | 1.8 cm | 2.0 cm (1.7 cm `rgbd+mono`, 3.2 cm `mono`) |
| Falsos positivos por vista | 7.1 % | 14.1 % |

El voto de clase solo ayuda si la confianza del detector es informativa. El oráculo, por defecto, da a sus errores de clase confianza 0.5–0.8 y a sus aciertos 0.7–1.0. Con rangos iguales (`wrong_class_conf_range`), el voto entre dos cámaras no mejora nada. Los falsos positivos suben porque la cámara del Dofbot también se equivoca, y porque las estimaciones solo monoculares que se alejan más de 0.15 m del objeto cuentan como falsos positivos.

### 5.2 Con Gazebo (un ensayo)

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

Terminal 1, simulación de un ensayo (con ruido de profundidad; `depth_noise:=false` para desactivarlo):

```bash
ros2 launch vla_pose_eval vla_pose_eval.launch.py trial_dir:=$HOME/vla_runs/test/S1_seed0000
```

Terminal 2, pasada de reconocimiento (guarda `views.yaml` y `views/`, con las imágenes de las dos cámaras):

```bash
ros2 run vla_pose_eval recon_pass --ros-args -p trial_dir:=$HOME/vla_runs/test/S1_seed0000
```

Estimar, evaluar y reportar (sin la simulación; sirve también en la máquina con GPU):

```bash
ros2 run vla_pose_eval vla_pose_eval run-estimator ~/vla_runs/test --estimator oracle_multicam
```

```bash
ros2 run vla_pose_eval vla_pose_eval evaluate ~/vla_runs/test --estimators oracle_multicam
```

```bash
ros2 run vla_pose_eval vla_pose_eval report ~/vla_runs/test --estimators oracle_multicam
```

Para comparar estimadores (por ejemplo, la línea base con una y con dos cámaras, o el modelo aprendido), se corre `run-estimator` una vez por estimador sobre los mismos ensayos y se pasan todos los nombres a `evaluate` y `report`. Todos leen las mismas imágenes de `views/`.

**No ejecutar `Mecanum_odometry_node.py` ni Nav2 en esta fase.** `ground_truth_publisher` publica `map → base_footprint` con la pose de Gazebo; la odometría publicaría `odom → base_footprint` y el marco tendría dos padres.

### 5.3 Modo en línea (opcional)

`recon_pass` con `-p online:=true` republica cada captura en `/vla/input/*`, incluida la imagen del Dofbot en `/vla/input/mono/*` con el mismo sello. Después espera un `Detection3DArray` en `/vla/estimates`. `estimator_node` envuelve cualquier `Estimator`, lee las extrínsecas de TF y anuncia su nombre y sus cámaras en `/vla/estimates/info`. El tiempo que se registra en este modo incluye el transporte ROS; el de inferencia se mide dentro del estimador.

## 6. Interfaces ROS 2

| Interfaz | Tipo | Quién |
|---|---|---|
| `/camera/image`, `/camera/camera_info` | `sensor_msgs` | Astra (puente de Gazebo; imagen con ruido del SDF) |
| `/camera/depth_image` | `sensor_msgs/Image` 32FC1 | `depth_noise` (con ruido) o el puente (`depth_noise:=false`) |
| `/sim/camera/depth_image` | `sensor_msgs/Image` 32FC1 | Puente: profundidad limpia, solo con `depth_noise:=true` |
| `/camera/depth_noise/config` | `std_msgs/String` (JSON, latched) | `depth_noise`: parámetros usados |
| `/mono_camera/image_raw`, `/mono_camera/camera_info` | `sensor_msgs` | Cámara del Dofbot (puente; imagen con ruido del SDF) |
| `/model/omni_dofbot/pose` | `geometry_msgs/PoseStamped` | Pose real del robot (Gazebo) |
| TF `map → base_footprint` | `tf2` | `ground_truth_publisher` |
| TF `base_footprint → camera_PC_link`, `→ mono_optical_link` | `tf2` | `robot_state_publisher` |
| `/world/vla_pose_arena/set_pose` | `ros_gz_interfaces/srv/SetEntityPose` | Teletransporte entre vistas |
| `/ground_truth/objects` | `geometry_msgs/PoseArray` | `ground_truth_publisher` (marco `map`, orden de `scene_gt.yaml`) |
| `/ground_truth/markers` | `visualization_msgs/MarkerArray` | `ground_truth_publisher` (RViz) |
| `/vla/input/image`, `/vla/input/depth`, `/vla/input/camera_info` | `sensor_msgs` | `recon_pass` en línea (Astra) |
| `/vla/input/mono/image`, `/vla/input/mono/camera_info` | `sensor_msgs` | `recon_pass` en línea (Dofbot) |
| `/vla/estimates` | `vision_msgs/Detection3DArray` | `estimator_node` |
| `/vla/estimates/info` | `std_msgs/String` (JSON, latched) | `estimator_node`: nombre y cámaras |
| `/vla/instruction` | `std_msgs/String` | Instrucción opcional |

En `Detection3DArray`:

- La clase y la confianza van en `results[0].hypothesis`.
- La pose va en `bbox.center`, en el marco óptico de la Astra.
- `id` lleva la fuente: `rgbd`, `mono` o `rgbd+mono`.
- Un cuaternión con los cuatro valores en cero significa «sin orientación». El valor por defecto de ROS es la identidad, que se leería como yaw 0: quien publique sin estimar orientación debe poner los ceros.

## 7. Decisiones tomadas

Están en `config/` y se pueden cambiar; conviene confirmarlas con el asesor.

1. **Punto de referencia del objeto: centro geométrico.** En las primitivas coincide con el origen del modelo en Gazebo.
2. **`map` es el marco del mundo de Gazebo.** No hay localización en esta fase.
3. **Objetos estáticos.** La verdad escrita por el generador es exacta porque nada se asienta. Para manipulación (Fase 2) se pone `static_objects: false` y habrá que leer la pose de los objetos de Gazebo.
4. **Un mundo SDF por escena.** Reproducible, pero hay que relanzar Gazebo en cada escena. Si resulta lento para generar datos de entrenamiento, la alternativa es mover un conjunto fijo de modelos con `set_pose`.
5. **Visibilidad y recall.** Un objeto está «en cuadro» si su centro se proyecta dentro de la imagen de **alguna de las cámaras que usa el estimador**. Por vista, el recall se calcula sobre esos objetos (los demás se marcan `OUT`). En la pasada completa, un objeto que nunca se ve cuenta como omisión y baja la cobertura. Así, un estimador solo con la Astra no se penaliza por los objetos que solo ve el Dofbot, y la mejora de cobertura del Dofbot aparece en la cobertura de la pasada.
6. **Escenas visibles por cualquiera de las dos cámaras** (`placement.visible_by: [rgbd, mono]`). Con la Astra sola, todos los objetos quedarían a más de ~1 m del robot.
7. **«En cuadro» es geométrico:** el centro del objeto se proyecta dentro de la imagen y dentro del rango de profundidad. No considera oclusiones.
8. **Distractores:** una estimación asociada a un distractor cuenta como falso positivo.
9. **Descomposición del error** (en `base_footprint`): radial sobre el rayo horizontal entre la cámara que dio la posición y el objeto, transversal perpendicular a ese rayo, y z. Los tres al cuadrado suman el error de posición al cuadrado.
10. **Fusión entre vistas:** no usa la verdad para agrupar. Un falso positivo visto en una sola vista sobrevive a la fusión, así que la tasa de falsos positivos de la pasada sale más alta que la de las vistas.
11. **Ruido de imagen subido de 0.002 a 0.007 en la Astra.** El valor de 0.002 equivalía a medio nivel de gris y casi desaparecía al cuantizar a 8 bits.

## 8. Diferencias entre la guía y el repositorio

1. **Cámaras.** La guía habla de una cámara en `arm_link_04` con FOV de unos 62°. En el URDF hay dos: la del brazo es monocular, de unos 41°, y la RGB-D está en el mástil, con 60° tras el cambio del 18 de noviembre. El paquete usa ambas (sección 2).
2. **Nombres de tópicos.** La guía cita `/camera/image_raw` y `/camera_info`; el puente publica `/camera/image`, `/camera/depth_image` y `/camera/camera_info`.
3. **Distancias mínimas.** Ver la tabla de la sección 2: la Astra no ve objetos bajos a menos de ~1 m, y de ahí el papel de la cámara del Dofbot.
4. **Teletransporte.** `ag_motion_tests.py` llama al servicio ROS `/world/<mundo>/set_entity_pose`, pero ningún launch lo puentea, así que en la práctica usa el respaldo `gz service`. El launch nuevo puentea `/world/vla_pose_arena/set_pose` y `recon_pass` conserva el mismo respaldo.
5. **`arena_world.world` no carga los sistemas de Gazebo** (sensores, órdenes de usuario); `arena_pid_tuning.world` sí. Los mundos generados los incluyen.
6. **Plano del LIDAR ≈ 0.12 m.** `laser_link` queda a 0.0815 + 0.0376 m del suelo según el URDF. Los pedestales generados miden entre 0.15 y 0.30 m.
7. **Tamaño de la arena.** Los mundos y el mapa usan 2.7 × 2.7 m; `waypoints.yaml` todavía dice 1.22 × 2.42 m. El generador usa 2.7 × 2.7 m.

## 9. Pendientes por etapa

### E0. Prerrequisitos

Verificar en simulación, en este orden:

- [ ] `colcon build` del paquete `vla_pose_eval`.
- [ ] El launch levanta Gazebo con `scene.world`, el robot, los controladores y `depth_noise`.
- [ ] `ros2 topic echo /model/omni_dofbot/pose --once` devuelve la pose del robot. Si no llega, revisar los parámetros de `PosePublisher` en `omni_dofbot_vla.xacro`. `recon_pass` avisa y usa la pose ordenada (queda anotado en `views.yaml` como `robot_pose_source: commanded`).
- [ ] El servicio `/world/vla_pose_arena/set_pose` existe en ROS; si no, confirmar que el respaldo `gz service` mueve al robot.
- [ ] TF: `ros2 run tf2_ros tf2_echo base_footprint camera_PC_link` da (−0.035, 0, 0.5315) y `tf2_echo base_footprint mono_optical_link` da ~(0.236, −0.010, 0.350) con el brazo en observación. Si difieren, actualizar `cameras.*.xyz_in_base` en `eval.yaml`.
- [ ] La imagen del Dofbot sale **derecha** (no girada) con el brazo en observación, y el brazo no tapa la imagen de la Astra.
- [ ] Ruido:
  - `ros2 topic list` muestra `/sim/camera/depth_image`, es decir, el puente respeta la reasignación.
  - En RViz, `/camera/depth_image` tiene escalones y huecos en los bordes.
  - La imagen RGB muestra grano. Si Gazebo no aplica `<noise>` a la cámara `rgbd_camera`, anotarlo: entonces la Astra no tendría ruido de imagen.
- [ ] Un ensayo completo con `oracle_multicam` **sin ruido** (`--kwargs '{}'`) da error ≈ 0 en las fuentes `rgbd` y `rgbd+mono`. Si no, hay un fallo de marcos. Ese es el criterio de salida de E0.
- [ ] Sustituir los objetos de `objects.yaml` por los del RS3 y definir su simetría.

### E1. Línea base clásica

- [ ] Elegir detector (Grounding DINO, OWLv2 o YOLO-World) tras revisar licencia y velocidad en la Jetson.
- [ ] Implementar `detect` en una subclase de `MultiCameraDetectorEstimator` (o de `DetectorDepthEstimator` para la variante solo Astra).
- [ ] Fijar `center_offsets` por clase: la profundidad mide la superficie visible y la referencia es el centro, lo que deja un sesgo radial de medio objeto (1 a 3 cm) si no se corrige.
- [ ] Comparar una y dos cámaras con `report --estimators baseline baseline_multicam`.
- [ ] Criterio: mediana ≤ 0.05 m en S1.

### E2. Modelo aprendido

- [ ] Generar el conjunto de entrenamiento con `--split train` y la pasada de reconocimiento. Las imágenes ya incluyen el ruido de ambas cámaras.
- [ ] Implementar `VLAEstimator` (o una subclase de `MultiCameraDetectorEstimator` si el modelo predice cajas).
- [ ] Aplicar la regla de decisión de la sección 2 de la guía con `report --estimators baseline_multicam vla`.

### E3 y E4

- [ ] Correr S2 y S3; revisar recall y falsos positivos.
- [ ] Comparar el error con y sin fusión entre vistas (tablas «Por vista» y «Pasada completa» del reporte).
- [ ] Ajustar `views.yaml` si la cobertura baja de 100 % (`vla_pose_eval check-views <dir>` muestra la cobertura por cámara).

### Ruido (antes de E5)

- [ ] Medir la Astra real (plano a varias distancias) y ajustar `config/noise.yaml`.
- [ ] Medir el ruido de imagen de ambas cámaras reales y ajustar el `stddev` de los xacro.

### Fuera de lo implementado

- Aleatorización de texturas y fondos reales (solo hay luz y colores).
- Estimación de orientación en la línea base.
- Distorsión de lente (Gazebo admite `<distortion>`, pero el estimador tendría que corregirla).
- Triangulación de la cámara del Dofbot entre vistas, que daría mejor posición monocular que el tamaño aparente.
- Filtrar en la pasada las estimaciones vistas en una sola vista, para bajar los falsos positivos.
- E5 (hardware) y todo lo de la sección 11 de la guía (goal pose, Nav2).
