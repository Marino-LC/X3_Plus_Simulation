# owlv2_detector

Detector de vocabulario abierto OWLv2 para las dos cámaras del robot. Tiene dos usos:

| Uso | Qué es | Cuándo detecta | Dónde deja las imágenes |
|---|---|---|---|
| **Estimador** `owlv2` / `owlv2_rgbd` (`estimator.py`) | Línea base E1 de `vla_pose_eval` | Una vez **por vista** de la pasada (tras cada teletransporte) | `<ensayo>/views/owlv2/view_XXX_{rgbd,mono}.png` y, en línea, `/vla/debug/<cámara>/image` |
| Nodo en vivo `owlv2_node` + `view_saver` | Solo para mirar en RViz | Continuo, desde que arranca la simulación | `/owlv2/<tópico>/image` y `<ensayo>/views/owlv2_live/` |

Para **evaluar**, se usa el estimador, nunca el nodo en vivo.

## Requisitos

`torch`, `transformers` y `pillow` en el mismo Python que ejecuta `vla_pose_eval` (el venv). El modelo (`google/owlv2-base-patch16-ensemble`) se descarga de Hugging Face la primera vez.

## Uso

Sobre las vistas que ya guardó `recon_pass` (sin simulación):

```bash
ros2 run vla_pose_eval vla_pose_eval run-estimator ~/vla_runs/test --estimator owlv2
```

```bash
ros2 run vla_pose_eval vla_pose_eval evaluate ~/vla_runs/test --estimators owlv2
```

En línea, durante la pasada (terminal 2 y 3, con la simulación en la 1):

```bash
ros2 run vla_pose_eval estimator_node --ros-args -p estimator:=owlv2
```

```bash
ros2 run vla_pose_eval recon_pass --ros-args -p trial_dir:=$HOME/vla_runs/test/S1_seed0000 -p online:=true
```

Nodo en vivo (solo RViz):

```bash
ros2 launch vla_pose_eval vla_pose_eval.launch.py trial_dir:=$HOME/vla_runs/test/S1_seed0000 owlv2_live:=true
```

## Consultas

Se definen en `vla_pose_eval/config/objects.yaml`:

- `objects.<clase>.prompts`: consultas de cada clase. La salida usa el nombre de la clase (`cubo`, `caja`, `cilindro`), que es lo que compara el evaluador.
- `detector.negative_prompts`: lo que **no** es un objeto (pedestal, pared, suelo, brazo). Si una caja se parece más a una consulta negativa, se descarta.
- `detector.threshold`, `class_thresholds`, `tiles`: umbral, umbral por clase y mosaicos.

Además, en la Astra se descarta toda caja cuyo tamaño métrico (medido con la profundidad) supere 1.6 × la mayor dimensión de los objetos: un pedestal mide 0.15 m y el objeto más grande 0.07 m.

En la imagen anotada: verde = detección, gris = consulta negativa, rojo = descartada por tamaño. Si una clase no aparece, hay que revisar su puntaje en esas imágenes y bajar su `class_thresholds` o probar otra consulta.
