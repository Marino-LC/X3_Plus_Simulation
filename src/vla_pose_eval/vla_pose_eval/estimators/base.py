"""Interfaz común de los estimadores de pose (módulo M1 de la guía)."""

import importlib
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


@dataclass
class Frame:
    """Entrada del modelo para una vista."""

    view: int
    rgb: Optional[np.ndarray]            # (H, W, 3) uint8, RGB
    depth: Optional[np.ndarray]          # (H, W) float32 en metros (inf/nan = sin dato)
    K: np.ndarray                        # intrínsecos 3x3
    instruction: str = ''                # instrucción en lenguaje natural (opcional)
    # Metadatos de la vista (T_map_base, T_base_cam…). Un estimador real NO
    # debe usarlos: solo existen para el oráculo y para depurar.
    meta: dict = field(default_factory=dict)


@dataclass
class Detection:
    """Una estimación: clase y pose del objeto en el marco ÓPTICO de la cámara."""

    cls: str
    position: List[float]                        # centro geométrico, metros
    conf: float = 1.0
    orientation_xyzw: Optional[List[float]] = None   # None si no se estima orientación

    def to_dict(self):
        return {'class': self.cls, 'conf': float(self.conf),
                'position': [float(v) for v in self.position],
                'orientation_xyzw': (None if self.orientation_xyzw is None
                                     else [float(v) for v in self.orientation_xyzw])}


class Estimator:
    """Clase base. Implementar `estimate`; la salida va en el marco óptico."""

    name = 'base'

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
    spec = aliases.get(spec, spec)
    if ':' not in spec:
        raise ValueError("estimador %r: usar un alias (%s) o 'modulo:Clase'"
                         % (spec, ', '.join(aliases)))
    module, cls = spec.split(':', 1)
    return getattr(importlib.import_module(module), cls)(**kwargs)
