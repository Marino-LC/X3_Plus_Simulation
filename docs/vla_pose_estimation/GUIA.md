> Copia en Markdown de la guía original (`.docx` en esta misma carpeta). El estado de la implementación en el repositorio está en [IMPLEMENTACION.md](IMPLEMENTACION.md).

# Guía: error de estimación de pose de objetos con VLA, sin Nav2

Para Emmanuel y Briana · Corte al 13 de noviembre de 2026 · Proyecto X3 Plus + Dofbot

## 1. Alcance

El alcance de esta guía es **una sola medición: qué tan lejos está la pose estimada de cada objeto de su pose real**, que en simulación se conoce con exactitud. No intervienen Nav2, costmaps, goal poses ni navegación.

Qué incluye:

1.  **Estimación por imagen:** el modelo recibe imágenes (y profundidad) tomadas desde poses conocidas del robot y devuelve la clase y la pose de cada objeto.

2.  **Pasada de reconocimiento:** una lista fija de vistas que cubre todos los objetos de la arena. El robot se coloca en cada vista por teletransporte o girando en sitio, sin navegar.

3.  **Comparación con la verdad de Gazebo:** el error de cada estimación frente a la pose real del objeto.

Quedan fuera, y se retoman después (sección 11): calcular desde dónde manipular, enviar esa pose a Nav2 y medir si el robot llega.

Los pedestales se mantienen en la escena para que más adelante funcionen como obstáculos de LIDAR, pero en esta fase no se usa el LIDAR.

## 2. Decisión previa: qué papel juega el VLA

En el RS3 plantean dudas sobre si OpenVLA-OFT es el modelo adecuado para esta tarea. La duda es razonable:

- OpenVLA y su variante OFT se diseñaron para producir **acciones del efector final** a partir de una imagen y una instrucción. Aquí la salida que se necesita es una **pose del objeto**, no una acción de brazo.

- Trabajan con imágenes RGB; no usan profundidad por defecto.

- Son modelos de 7B parámetros. Correrlos en la Jetson Orin con latencias bajas es dudoso; hay que medirlo.

- Reutilizarlos para estimar poses implica ajustar la salida (nueva cabeza o nuevo formato de acción) y recopilar sus propios datos.

Con este alcance, el trabajo del modelo es de **percepción**: decir qué objeto es y dónde está respecto al robot. No hay control ni planificación.

| Módulo                | Función                                 | Entrada                                       | Salida                                               |
|-----------------------|-----------------------------------------|-----------------------------------------------|------------------------------------------------------|
| M1. Percepción        | Reconocer objetos y estimar su pose     | Imagen RGB, profundidad, instrucción opcional | Clase y pose de cada objeto en el marco de la cámara |
| M2. Fusión (opcional) | Combinar varias vistas del mismo objeto | Estimaciones de M1 y pose real del robot      | Una pose por objeto en el marco map                  |

**Variantes de M1:**

- **Línea base clásica:** detector preentrenado de vocabulario abierto (por ejemplo Grounding DINO, OWLv2 o YOLO-World; verificar licencia y viabilidad en Jetson) más profundidad de la cámara Astra.

- **Modelo aprendido:** el VLM/VLA ajustado con su base de datos.

**Regla de decisión:** el modelo aprendido se considera útil solo si iguala o mejora a la línea base en error de posición, con las mismas escenas y métricas. Si no lo logra, se reporta ese resultado y el VLA se reserva para la manipulación de la Fase 2.

## 3. Escena y verdad de referencia

Cada ensayo se genera con una semilla y guarda su verdad de referencia.

- **Qué se guarda:** por objeto, clase, posición (x, y, z) y orientación (yaw) en el marco map; por vista, la pose real del robot. El script del RS3 ya conoce {objeto, P, O, q}; añadan la pose del robot.

- **Punto de referencia del objeto:** definan una sola vez si la pose es el centro geométrico o el origen del modelo en Gazebo, y úsenla igual en las etiquetas y en la verdad. Un desfase de unos centímetros se confunde con error del modelo.

- **Simetrías:** un cubo se ve igual cada 90° y una taza con asa no. Definan por objeto su ángulo de simetría y reporten el error de orientación módulo ese ángulo.

- **Pedestales:** mismas dimensiones que en la arena de navegación (0.15 × 0.15 m). Su altura debe quedar por encima del plano del LIDAR para el uso futuro (verificar el z de laser_link); ahora solo importa que el objeto caiga dentro del campo de visión.

- **Origen de la verdad:** la pose real viene de Gazebo (el generador de escenas o el tópico de poses puenteado con ros_gz_bridge), nunca de la odometría de las ruedas.

## 4. Pasada de reconocimiento sin navegación

1.  **Brazo en pose de observación** \[0.0, 0.0, -1.57, 0.0, 0.0\] (la del RS3). Así la cámara, montada en arm_link_04, queda en una pose fija respecto al chasis.

2.  **Lista fija de vistas:** poses (x, y, yaw) del robot a lo largo del eje largo de la arena, con giros de 45° o menos en cada punto. La cámara tiene un FOV de unos 62°, así que una vista no basta.

3.  **En cada vista:** teletransportar al robot con set_entity_pose (como hace el AG) o girarlo en sitio con cmd_vel; esperar a que se asiente; capturar imagen, profundidad y camera_info; guardar la pose real del robot.

4.  **Inferencia:** ejecutar el modelo y registrar estimaciones y tiempo de inferencia.

5.  **Asociación:** emparejar cada estimación con el objeto real más cercano (umbral de 0.15 m). Una estimación sin pareja es un falso positivo; un objeto sin estimación es una omisión.

6.  **Fusión (opcional):** promediar la posición y votar la clase entre vistas del mismo objeto; comparar el error con y sin fusión.

Formato sugerido de salida, una entrada por estimación:

```yaml
estimates:
  - view: 12
    robot_pose_gt: {x: 0.00, y: -0.40, yaw: 1.57}
    object_gt: {id: 3, class: taza, x: 0.02, y: 0.80, z: 0.21, yaw: 0.35}
    estimate: {class: taza, x: 0.05, y: 0.78, z: 0.20, yaw: 0.50, conf: 0.91}
    infer_ms: 480
```

Los valores del ejemplo son ilustrativos.

## 5. Marcos de referencia y cálculo del error

El modelo estima en el marco de la cámara. Para compararlo con la verdad:

1.  Transformar la estimación a base_footprint con el TF fijo cámara a base (posible porque el brazo está en pose de observación).

2.  Transformar a map con la pose real del robot de Gazebo, para que el error de localización del robot no entre en la medición.

3.  Calcular el error contra la pose real del objeto.

```latex
e_{pos} = \lVert \hat{p} - p_{gt} \rVert_2 \qquad e_{yaw} = \min_{k \in \mathbb{Z}} \left| \mathrm{wrap}\left(\hat{\psi} - \psi_{gt} - k\,\Delta\right) \right|
```

donde Δ es el ángulo de simetría del objeto.

**Marco principal de reporte:** base_footprint, porque es el que usará la manipulación. Reporten además el error **radial** (a lo largo del rayo de la cámara), el **transversal** y el de z por separado: indican si el fallo viene de la profundidad o de la detección.

## 6. Interfaces ROS 2 sugeridas

| Interfaz                                                             | Tipo                                  | Función                                                           |
|----------------------------------------------------------------------|---------------------------------------|-------------------------------------------------------------------|
| /camera/image_raw, profundidad, /camera_info, /mono_camera/image_raw | sensor_msgs                           | Entradas del modelo                                               |
| TF                                                                   | tf2                                   | Cámara a base_footprint (fijo) y map a base_footprint (de Gazebo) |
| /vla/estimates                                                       | vision_msgs/Detection3DArray          | Clase y pose estimadas                                            |
| /ground_truth/objects                                                | geometry_msgs/PoseArray o MarkerArray | Verdad de referencia publicada por el generador de escenas        |
| set_entity_pose                                                      | Servicio de Gazebo                    | Teletransporte del robot entre vistas                             |
| /vla/instruction                                                     | std_msgs/String                       | Instrucción en lenguaje natural (opcional)                        |

Un nodo evaluador se suscribe a las estimaciones y a la verdad y escribe un CSV por ensayo con los errores de la sección 8. Si el modelo corre fuera de ROS (por ejemplo, con GPU en otra máquina), la API REST del nodo puente de Lair añade latencia; mídanla aparte del tiempo de inferencia.

## 7. Datos de entrenamiento: qué conservar y qué cambiar

**Conservar (del RS3):** el etiquetado automático con la verdad de la simulación y la aleatorización de posición, altura y orientación.

**Cambiar o añadir:**

1.  **Predecir clase y caja (o máscara) y calcular el 3D con profundidad** (cámara Astra RGB-D). Aprender posición métrica desde una sola imagen monocular es ambiguo en escala y altura.

2.  **Escenas con varios objetos, pedestales vacíos y distractores.** En el RS3 hay un solo objeto por imagen, pero en la prueba habrá varios.

3.  **Variar la pose del robot** (distancia y rumbo), no una sola vista. Etiquetar en el marco de la cámara y transformar con TF.

4.  **Aleatorización de dominio:** luz, texturas, fondos y colores. Separar entrenamiento y prueba **por escena y por objeto**, no por imagen, para no inflar los resultados.

5.  **Corregir el script** para que los pedestales se generen dentro del campo de visión (ya lo identificaron).

6.  **Registrar** intrínsecos de la cámara, TF y la semilla de cada escena para poder reproducirla.

Empiecen con un conjunto pequeño, midan y escalen según el análisis de errores. Habrá una brecha sim a real al pasar a hardware; planífiquenla desde ahora (texturas y luz reales).

## 8. Métricas

Las metas son **propuestas** para discutir con el equipo. El resultado principal es la distribución del error de posición, en metros; el resto sirve para interpretarla.

| Métrica                          | Definición                                                  | Meta propuesta                 |
|----------------------------------|-------------------------------------------------------------|--------------------------------|
| Error de posición                | Distancia entre posición estimada y real, en base_footprint | Mediana ≤ 0.05 m; P95 ≤ 0.10 m |
| Error de orientación             | Diferencia de yaw módulo la simetría del objeto             | Mediana ≤ 15°                  |
| Error radial, transversal y en z | Descomposición del error de posición                        | Reportar                       |
| Exactitud de clase               | Clasificaciones correctas ÷ estimaciones asociadas          | ≥ 90 %                         |
| Recall                           | Objetos con estimación asociada ÷ objetos presentes         | ≥ 90 %                         |
| Falsos positivos                 | Estimaciones sin objeto real ÷ estimaciones                 | ≤ 5 %                          |
| Cobertura de la pasada           | Objetos vistos al menos una vez ÷ total                     | 100 %                          |
| Tiempo de inferencia             | Por imagen, p50 y p95                                       | Reportar                       |

Como referencia, la tolerancia de llegada que usa el proyecto para navegación es 0.15 m; el error de estimación debe ser bastante menor para dejar margen cuando se integre con Nav2.

### Cortes del error

Reporten el error de posición por separado según:

- distancia del robot al objeto (rangos de 0.2 m);

- ángulo de vista respecto al objeto;

- altura del pedestal;

- número de objetos en la escena (S1, S2, S3);

- clase del objeto.

### Protocolo de reporte

- **Escenas:** S1 (un pedestal), S2 (tres pedestales) y S3 (cinco pedestales con distractores), con semillas fijas.

- **Separación de datos:** entrenamiento y prueba por escena y por objeto, no por imagen.

- **Misma entrada:** línea base y modelo aprendido se evalúan sobre las mismas imágenes.

- **Errores:** media, desviación estándar, mediana y P95, con intervalo de confianza del 95 % de la mediana por *bootstrap*. Las imágenes de una misma escena están correlacionadas, así que el remuestreo se hace **por escena**, no por imagen.

- **Tasas** (clase, recall): intervalo de Wilson al 95 %. Con 20 ensayos, 17 aciertos (85 %) dan un intervalo de aproximadamente 64 a 95 %.

- **Hardware (Fase 2):** mismo protocolo con N repeticiones y brecha sim a real (Δm).

## 9. Etapas con criterio de salida

| Etapa                        | Qué se hace                                                                                                  | Criterio de salida                                        |
|------------------------------|--------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------|
| E0. Prerrequisitos           | Cámara RGB-D con TF, brazo en pose de observación, generador de escenas con verdad y semilla, nodo evaluador | Un ensayo completo produce el CSV con el error por objeto |
| E1. Línea base clásica       | Detector preentrenado más profundidad                                                                        | Mediana ≤ 0.05 m en S1                                    |
| E2. Modelo aprendido         | VLM/VLA con su base de datos                                                                                 | Igualar o mejorar a E1 en las mismas escenas              |
| E3. Varios objetos           | Escenas S2 y S3 con distractores y pedestales vacíos                                                         | Recall ≥ 90 % y falsos positivos ≤ 5 %                    |
| E4. Pasada completa y fusión | Lista de vistas, comparación con y sin fusión                                                                | Cobertura 100 %                                           |
| E5. Hardware                 | Mismo protocolo en el robot real (Fase 2)                                                                    | N repeticiones y Δm                                       |

Con este alcance, y como estimación mía, para el 20 de diciembre son realistas E0 a E3, con E4 parcial. E5 queda para la Fase 2.

## 10. Riesgos y decisiones abiertas

- **¿OpenVLA-OFT es el modelo adecuado?** Ya lo plantearon; usen la regla de decisión de la sección 2.

- **Punto de referencia y simetrías mal definidos:** generan error espurio (sección 3).

- **Campo de visión de 62°:** un objeto fuera de cuadro cuenta como omisión, no como error de posición.

- **Profundidad simulada frente a real:** el ruido de la Astra real no está en Gazebo, así que el error en simulación no se transfiere tal cual a hardware.

- **Correlación entre imágenes:** sin remuestreo por escena, los intervalos de confianza salen demasiado estrechos.

- **Narrativa del Artículo 3:** si el VLA no supera a la línea base, replantear el artículo como estimación de pose con VLM y reservar el VLA para la manipulación. Conviene hablarlo con el asesor.

## 11. Trabajo posterior (fuera de este alcance)

- **Goal pose:** a partir de la pose estimada, calcular dónde parar el robot a una distancia de trabajo d, que sale de la cinemática directa de Diego.

- **Integración con Nav2:** publicar un PoseStamped en map y medir éxito de misión, error de llegada, validez de la meta y alcanzabilidad. El error medido aquí pasa a ser la parte de percepción del error total.

- **Altura del pedestal frente al plano del LIDAR:** verificar en simulación y en hardware.

- **Inflación frente a alcance del brazo:** con robot_radius de 0.15 m, el robot no se acerca a menos de unos 0.23 m del centro del pedestal; comprobar que el alcance útil del brazo lo permite.

## 12. Fuentes

- Kim et al., *OpenVLA: An Open-Source Vision-Language-Action Model* (2024), y Kim, Finn y Liang, *Fine-Tuning Vision-Language-Action Models: Optimizing Speed and Success* (2025, OpenVLA-OFT). Buscar en arXiv; verificar licencia y requisitos de memoria.

- Documentación de Gazebo Harmonic (gazebosim.org): cámara de profundidad, poses de entidades y puente con ros_gz_bridge.

- Detectores de vocabulario abierto: Grounding DINO (Liu et al., 2023), OWL-ViT y OWLv2 (Minderer et al.), YOLO-World (Cheng et al., 2024). Verificar licencias y velocidad en la Jetson Orin.

- Paquete vision_msgs de ROS 2, para Detection3DArray.

- Efron (1979), *Bootstrap Methods: Another Look at the Jackknife*.

- Wilson (1927), intervalo de confianza para proporciones.
