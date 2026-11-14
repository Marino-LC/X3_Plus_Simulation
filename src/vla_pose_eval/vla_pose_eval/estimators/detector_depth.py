"""Línea base clásica (E1): detector 2D + profundidad de la cámara.

Esta clase ya resuelve la parte geométrica (caja o máscara 2D + profundidad ->
posición 3D en el marco óptico). Falta conectar el detector: hereden de
`DetectorDepthEstimator` e implementen `detect` con Grounding DINO, OWLv2 o
YOLO-World (verificar licencia y viabilidad en la Jetson antes de elegir).

    class MiLineaBase(DetectorDepthEstimator):
        name = 'baseline_owlv2'
        def detect(self, rgb, instruction):
            ...
            return [Box2D('cubo', 0.83, (x0, y0, x1, y1)), ...]

    vla_pose_eval run-estimator <ensayo> --estimator mi_paquete.mi_modulo:MiLineaBase
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from .. import geometry as g
from .base import Detection, Estimator


@dataclass
class Box2D:
    cls: str
    conf: float
    xyxy: Tuple[float, float, float, float]      # píxeles: x0, y0, x1, y1
    mask: Optional[np.ndarray] = None            # (H, W) bool, si el detector segmenta


class DetectorDepthEstimator(Estimator):
    name = 'baseline'

    def __init__(self, shrink=0.5, front_band_m=0.05, min_pixels=5, center_offsets=None):
        """
        shrink: fracción central de la caja usada para leer profundidad (evita
            bordes donde se cuela el fondo).
        front_band_m: se toma la mediana de los píxeles a menos de esta
            distancia del más cercano (la superficie frontal del objeto).
        center_offsets: {clase: metros} que se suman a lo largo del rayo. La
            profundidad mide la SUPERFICIE visible, pero la pose de referencia
            es el CENTRO geométrico; sin esta corrección queda un sesgo radial
            de medio objeto (~1-3 cm). Ver objects.yaml para los tamaños.
        """
        self.shrink = float(shrink)
        self.front_band = float(front_band_m)
        self.min_pixels = int(min_pixels)
        self.center_offsets = dict(center_offsets or {})

    # -- A implementar por el equipo ------------------------------------------
    def detect(self, rgb, instruction) -> List[Box2D]:
        raise NotImplementedError(
            'Conectar aquí el detector de vocabulario abierto (guía, sección 2).')

    # -- Geometría (ya implementada) ------------------------------------------
    def lift(self, box, depth, K):
        """Caja 2D + imagen de profundidad -> punto 3D en el marco óptico, o None.

        Supone que `depth` es profundidad a lo largo del eje óptico (z), que es
        lo que publica la cámara RGB-D de Gazebo y la Astra.
        """
        h, w = depth.shape[:2]
        x0, y0, x1, y1 = box.xyxy
        if box.mask is not None:
            region = depth[box.mask]
            vs, us = np.nonzero(box.mask)
            u_c, v_c = float(us.mean()), float(vs.mean())
        else:
            u_c, v_c = (x0 + x1) / 2.0, (y0 + y1) / 2.0
            hw, hh = (x1 - x0) * self.shrink / 2.0, (y1 - y0) * self.shrink / 2.0
            c0, c1 = int(max(0, np.floor(u_c - hw))), int(min(w, np.ceil(u_c + hw) + 1))
            r0, r1 = int(max(0, np.floor(v_c - hh))), int(min(h, np.ceil(v_c + hh) + 1))
            region = depth[r0:r1, c0:c1]
        vals = region[np.isfinite(region) & (region > 0.0)]
        if vals.size < self.min_pixels:
            return None                  # sin profundidad (p. ej. más cerca que depth_near)
        z = float(np.median(vals[vals <= vals.min() + self.front_band]))
        p = g.backproject(np.asarray(K, float), u_c, v_c, z)
        offset = float(self.center_offsets.get(box.cls, 0.0))
        return p * (1.0 + offset / np.linalg.norm(p))

    def estimate(self, frame):
        if frame.rgb is None or frame.depth is None:
            raise ValueError('la línea base necesita imagen RGB y profundidad')
        dets = []
        for box in self.detect(frame.rgb, frame.instruction):
            p = self.lift(box, frame.depth, frame.K)
            if p is not None:
                # La línea base no estima orientación (queda como mejora: PCA
                # de la nube de puntos dentro de la máscara).
                dets.append(Detection(cls=box.cls, position=list(p), conf=box.conf))
        return dets
