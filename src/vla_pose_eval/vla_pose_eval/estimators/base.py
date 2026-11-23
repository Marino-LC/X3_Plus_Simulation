"""Interfaz común de los estimadores de pose (módulo M1 de la guía)."""

import importlib
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np


@dataclass
class CameraImage:
    """Imagen de una cámara auxiliar (la del Dofbot) con su calibración."""

    name: str
    rgb: Optional[np.ndarray]            # (H, W, 3) uint8, RGB
    K: np.ndarray                        # intrínsecos 3x3
    width: int
    height: int
    T_base_cam: np.ndarray               # marco óptico -> base_footprint (TF: calibración)
    depth: Optional[np.ndarray] = None   # la cámara del Dofbot no tiene profundidad


@dataclass
class Frame:
    """Entrada del modelo para una vista.

    Los campos `rgb`, `depth` y `K` son de la RGB-D (Astra), cuyo marco óptico es
    el marco de SALIDA de todas las estimaciones. `T_base_cam` y `mono` son
    calibración (salen de TF) y el estimador SÍ puede usarlos.
    """

    view: int
    rgb: Optional[np.ndarray]            # (H, W, 3) uint8, RGB
    depth: Optional[np.ndarray]          # (H, W) float32 en metros (inf/nan = sin dato)
    K: np.ndarray                        # intrínsecos 3x3
    instruction: str = ''                # instrucción en lenguaje natural (opcional)
    # Metadatos de la vista (T_map_base…). Un estimador real NO debe usarlos:
    # solo existen para el oráculo y para depurar.
    meta: dict = field(default_factory=dict)
    T_base_cam: Optional[np.ndarray] = None    # extrínseca de la RGB-D
    mono: Optional[CameraImage] = None         # cámara del Dofbot (si se capturó)


@dataclass
class Detection:
    """Una estimación: clase y pose del objeto en el marco ÓPTICO de la RGB-D.

    `source` dice qué cámaras la produjeron: 'rgbd', 'mono' o 'rgbd+mono'.
    Aunque venga solo de la cámara del Dofbot, la posición se expresa en el
    marco óptico de la RGB-D (con la extrínseca de TF).
    """

    cls: str
    position: List[float]                        # centro geométrico, metros
    conf: float = 1.0
    orientation_xyzw: Optional[List[float]] = None   # None si no se estima orientación
    source: str = 'rgbd'
    class_scores: Optional[Dict[str, float]] = None  # votos por clase tras fusionar

    def to_dict(self):
        d = {'class': self.cls, 'conf': float(self.conf),
             'position': [float(v) for v in self.position],
             'orientation_xyzw': (None if self.orientation_xyzw is None
                                  else [float(v) for v in self.orientation_xyzw]),
             'source': self.source}
        if self.class_scores:
            d['class_scores'] = {k: float(v) for k, v in self.class_scores.items()}
        return d


class Estimator:
    """Clase base. Implementar `estimate`; la salida va en el marco óptico de la RGB-D."""

    name = 'base'
    cameras = ('rgbd',)      # cámaras que usa: ('rgbd',) o ('rgbd', 'mono')

    def estimate(self, frame: Frame) -> List[Detection]:
        raise NotImplementedError

    def setup(self, gt=None, cfg=None):
        """Se llama una vez por ensayo. `gt` solo lo usa el oráculo."""


def load_estimator(spec, **kwargs):
    """Crea un estimador a partir de un alias o de 'paquete.modulo:Clase'."""
    aliases = {
        'oracle': 'vla_pose_eval.estimators.oracle:OracleEstimator',
        'vla': 'vla_pose_eval.estimators.vla:VLAEstimator',
    }
    if spec == 'oracle_multicam':
        spec = aliases['oracle']
        kwargs = dict({'use_mono': True}, **kwargs)
    spec = aliases.get(spec, spec)
    if ':' not in spec:
        raise ValueError("estimador %r: usar un alias (oracle, oracle_multicam, vla) "
                         "o 'modulo:Clase'" % spec)
    module, cls = spec.split(':', 1)
    return getattr(importlib.import_module(module), cls)(**kwargs)
