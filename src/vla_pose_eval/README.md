# vla_pose_eval

Mide qué tan lejos está la pose estimada de cada objeto de su pose real en Gazebo. Sin Nav2, costmaps ni navegación.

- Guía del experimento: [`docs/vla_pose_estimation/GUIA.md`](../../docs/vla_pose_estimation/GUIA.md)
- Estado, uso y pendientes: [`docs/vla_pose_estimation/IMPLEMENTACION.md`](../../docs/vla_pose_estimation/IMPLEMENTACION.md)

## Flujo

```
generate-scenes ──► scene.world + scene_gt.yaml          (por tipo de escena y semilla)
launch + recon_pass ──► views.yaml + views/*.png, *.npy   (Gazebo: teletransporte y captura)
run-estimator ──► estimates_<estimador>.yaml              (oráculo, línea base o VLA)
evaluate ──► errors_<estimador>.csv, errors_fused_<estimador>.csv
report ──► summary.md, summary.json                       (bootstrap por escena, Wilson, cortes)
```

Todo salvo el segundo paso es Python puro (`numpy`, `PyYAML`) y corre sin ROS:

```bash
python3 -m vla_pose_eval.cli --help
```

```bash
python3 -m vla_pose_eval.cli dry-run --out /tmp/vla_dry --seeds 0 20
```

## Estructura

| Ruta | Contenido |
|---|---|
| `config/eval.yaml` | Marcos, cámara, tópicos, umbrales y metas propuestas |
| `config/objects.yaml` | Objetos, simetrías, punto de referencia, pedestales |
| `config/scenes.yaml` | Arena, S1/S2/S3, aleatorización, semillas de entrenamiento y prueba |
| `config/views.yaml` | Lista de vistas de la pasada |
| `vla_pose_eval/scene.py` | Generador de escenas y mundo SDF |
| `vla_pose_eval/views.py` | Vistas, modelo de cámara, visibilidad |
| `vla_pose_eval/metrics.py` | Error, asociación, fusión, bootstrap, Wilson |
| `vla_pose_eval/evaluate.py` | Evaluador de un ensayo → CSV |
| `vla_pose_eval/report.py` | Reporte agregado |
| `vla_pose_eval/estimators/` | Interfaz, oráculo, línea base (geometría) y plantilla del VLA |
| `vla_pose_eval/nodes/` | `recon_pass`, `ground_truth_publisher`, `estimator_node` |
| `launch/vla_pose_eval.launch.py` | Simulación de un ensayo |
| `test/` | Pruebas del núcleo (`python3 -m pytest test -q`) |

## Añadir un estimador

```python
from vla_pose_eval.estimators.detector_depth import Box2D, DetectorDepthEstimator

class MiLineaBase(DetectorDepthEstimator):
    name = 'baseline'

    def detect(self, rgb, instruction):
        # rgb: (H, W, 3) uint8. Devolver cajas en píxeles.
        return [Box2D('cubo', 0.83, (x0, y0, x1, y1))]
```

```bash
python3 -m vla_pose_eval.cli run-estimator ~/vla_runs/test --estimator mi_modulo:MiLineaBase
```

La salida de `estimate` va en el marco **óptico** de la cámara (z al frente, x a la derecha, y abajo) y la posición es el **centro geométrico** del objeto.

## CSV de errores

Una fila por (vista, objeto) o por falso positivo. `status`: `TP` asociada, `FN` omisión, `FP` falso positivo, `OUT` objeto fuera de cuadro en esa vista. Posiciones y errores en `base_footprint` (en `map` en el CSV fusionado). Columnas: ver `COLUMNS` en `evaluate.py`.
