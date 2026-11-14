"""Modelo aprendido (E2): VLM/VLA ajustado con la base de datos del equipo.

Plantilla sin implementar. Dos caminos compatibles con la guía (sección 7):

1. El modelo predice clase y caja/máscara y el 3D se calcula con profundidad:
   hereden de `DetectorDepthEstimator` e implementen `detect`. Es el camino
   recomendado (la posición métrica desde una sola imagen RGB es ambigua).
2. El modelo predice directamente la pose: implementen `estimate` y devuelvan
   `Detection` en el marco óptico de la cámara.

Regla de decisión (sección 2): el modelo se considera útil solo si iguala o
mejora a la línea base en error de posición con las mismas escenas y métricas.
"""

from .base import Estimator


class VLAEstimator(Estimator):
    name = 'vla'

    def __init__(self, checkpoint=None, device='cuda'):
        self.checkpoint = checkpoint
        self.device = device

    def estimate(self, frame):
        raise NotImplementedError(
            'Pendiente (etapa E2): cargar el modelo ajustado y devolver Detection '
            'en el marco óptico. Ver docs/vla_pose_estimation/IMPLEMENTACION.md.')
