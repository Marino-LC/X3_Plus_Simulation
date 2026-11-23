"""Fusión RGB-D (Astra) + cámara del Dofbot para identificar objetos.

La cámara del Dofbot (arm_link_04) no tiene profundidad, pero:

* tiene más resolución angular (FOV ~41 grados frente a 60) y otro punto de
  vista, así que aporta un segundo voto de CLASE para los objetos que ven ambas;
* con el brazo en pose de observación mira desde 0.35 m de altura y ve el campo
  cercano (~0.1-0.65 m) que la Astra no ve (su profundidad empieza en 0.6 m y su
  FOV vertical es de 36 grados).

`MultiCameraFusion.fuse` combina, por vista:

1. cada detección 3D de la RGB-D se proyecta en la imagen del Dofbot y se
   empareja con la caja 2D que la contiene; la clase se decide por voto
   ponderado por confianza (fuente 'rgbd+mono');
2. una detección 3D que cae en el cuadro del Dofbot sin caja que la confirme
   multiplica su confianza por `unconfirmed_factor` (fuente 'rgbd');
3. una caja del Dofbot sin detección 3D da una estimación SOLO MONOCULAR
   (fuente 'mono'): distancia por tamaño aparente, z = f * S / sqrt(w * h),
   con S el tamaño aparente esperado de la clase (catálogo objects.yaml,
   corregido por la elevación del rayo). Es una estimación aproximada; el
   evaluador la reporta aparte (corte por fuente).
   Se descarta si la caja toca el borde de la imagen (objeto cortado).
"""

import math
from dataclasses import dataclass
from typing import Dict, List

import numpy as np

from .. import geometry as g
from ..scene import shape_dims
from .base import Detection
from .detector_depth import DetectorDepthEstimator


@dataclass
class SizePrior:
    """Extensión aparente esperada de un objeto (m), promediada sobre su giro (yaw).

    width: ancho horizontal de la silueta; para una caja, dx|cos θ| + dy|sin θ|,
        cuya media sobre θ uniforme es 2 (dx + dy) / π.
    depth: extensión a lo largo del rayo (igual que el ancho, por simetría).
    height: altura.
    sphere: la silueta de una esfera no depende del ángulo de vista.
    """

    width: float
    depth: float
    height: float
    sphere: bool = False

    @classmethod
    def from_shape(cls, shape, size):
        dx, dy, dz = shape_dims(shape, size)
        w = dx if shape in ('cylinder', 'sphere') else 2.0 * (dx + dy) / math.pi
        return cls(width=w, depth=w, height=dz, sphere=(shape == 'sphere'))

    def apparent(self, elevation):
        """sqrt(ancho * alto) de la silueta vista con el rayo a `elevation` rad
        bajo la horizontal: desde arriba se ve también la cara superior."""
        if self.sphere:
            return self.width
        e = abs(elevation)
        return math.sqrt(self.width * (self.height * math.cos(e) + self.depth * math.sin(e)))


def size_priors(objects_cfg):
    """{clase: SizePrior} del catálogo objects.yaml (el estimador conoce los modelos)."""
    return {cls: SizePrior.from_shape(spec['shape'], spec['size'])
            for cls, spec in (objects_cfg or {}).get('objects', {}).items()}


class MultiCameraFusion:

    def __init__(self, size_prior: Dict[str, 'SizePrior'] = None, w_rgbd=1.0, w_mono=1.0,
                 match_margin_px=15.0, unconfirmed_factor=1.0, mono_only=True,
                 dedupe_radius_m=0.15, min_conf=0.0, truncation_border_px=2.0):
        self.size_prior = dict(size_prior or {})
        self.w_rgbd, self.w_mono = float(w_rgbd), float(w_mono)
        self.margin = float(match_margin_px)
        self.unconfirmed_factor = float(unconfirmed_factor)
        self.mono_only = bool(mono_only)
        self.dedupe = float(dedupe_radius_m)
        self.min_conf = float(min_conf)
        self.border = float(truncation_border_px)

    def mono_position(self, box, K, width=None, height=None, R_base_cam=None):
        """Caja 2D -> centro 3D en el marco óptico del Dofbot (tamaño aparente).

        Con la extrínseca (`R_base_cam`) se corrige la elevación del rayo. La
        silueta la domina la parte frontal del objeto, así que el tamaño da la
        distancia a esa cara y se suma media profundidad para llegar al centro.

        None si la clase no tiene tamaño conocido o si la caja toca el borde de
        la imagen (objeto cortado: su tamaño aparente no sirve).
        """
        prior = self.size_prior.get(box.cls)
        x0, y0, x1, y1 = box.xyxy
        if width is not None and (x0 <= self.border or y0 <= self.border
                                  or x1 >= width - 1 - self.border
                                  or y1 >= height - 1 - self.border):
            return None
        s_px = math.sqrt(max(x1 - x0, 1e-6) * max(y1 - y0, 1e-6))
        if prior is None or s_px < 1.0:
            return None
        u, v = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        ray = g.backproject(K, u, v, 1.0)
        elevation = 0.0
        if R_base_cam is not None:
            r = np.asarray(R_base_cam, float) @ ray
            elevation = math.atan2(-r[2], math.hypot(r[0], r[1]))
        f = 0.5 * (K[0, 0] + K[1, 1])
        z_front = f * prior.apparent(elevation) / s_px
        rng = z_front * float(np.linalg.norm(ray))          # distancia a lo largo del rayo
        rng += 0.0 if prior.sphere else prior.depth / 2.0
        return ray / np.linalg.norm(ray) * rng

    def fuse(self, dets: List[Detection], boxes, frame) -> List[Detection]:
        mono = frame.mono
        if mono is None or frame.T_base_cam is None:
            return list(dets)
        K = np.asarray(mono.K, float).reshape(3, 3)
        T_mono_rgbd = g.inv_T(np.asarray(mono.T_base_cam, float)) @ np.asarray(frame.T_base_cam,
                                                                                float)
        T_rgbd_mono = g.inv_T(T_mono_rgbd)
        m = self.margin

        # 1. Proyectar cada detección 3D en la imagen del Dofbot y emparejar (voraz).
        cand, inside = [], set()
        for i, d in enumerate(dets):
            u, v, z = g.project(K, g.transform_point(T_mono_rgbd, d.position))
            if not (z > 0 and 0 <= u < mono.width and 0 <= v < mono.height):
                continue
            inside.add(i)
            for j, b in enumerate(boxes):
                x0, y0, x1, y1 = b.xyxy
                if x0 - m <= u <= x1 + m and y0 - m <= v <= y1 + m:
                    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
                    diag = max(1.0, math.hypot(x1 - x0, y1 - y0))
                    cand.append((math.hypot(u - cx, v - cy) / diag, i, j))
        cand.sort()
        pair, used = {}, set()
        for _, i, j in cand:
            if i not in pair and j not in used:
                pair[i] = j
                used.add(j)

        out = []
        for i, d in enumerate(dets):
            if i in pair:
                b = boxes[pair[i]]
                scores = {}
                scores[d.cls] = scores.get(d.cls, 0.0) + self.w_rgbd * d.conf
                scores[b.cls] = scores.get(b.cls, 0.0) + self.w_mono * b.conf
                cls = max(scores, key=scores.get)
                out.append(Detection(
                    cls=cls, position=list(d.position),
                    conf=scores[cls] / (self.w_rgbd + self.w_mono),
                    orientation_xyzw=d.orientation_xyzw if cls == d.cls else None,
                    source='rgbd+mono', class_scores=scores))
            else:
                factor = self.unconfirmed_factor if i in inside else 1.0
                out.append(Detection(cls=d.cls, position=list(d.position), conf=d.conf * factor,
                                     orientation_xyzw=d.orientation_xyzw, source=d.source,
                                     class_scores=d.class_scores))

        # 3. Cajas del Dofbot sin detección 3D: estimación monocular por tamaño aparente.
        if self.mono_only:
            for j, b in enumerate(boxes):
                if j in used:
                    continue
                p_mono = self.mono_position(b, K, mono.width, mono.height,
                                            np.asarray(mono.T_base_cam, float)[:3, :3])
                if p_mono is None:
                    continue
                p = g.transform_point(T_rgbd_mono, p_mono)
                if any(np.linalg.norm(p - np.asarray(o.position)) < self.dedupe for o in out):
                    continue           # probablemente el mismo objeto, ya estimado con profundidad
                out.append(Detection(cls=b.cls, position=list(p), conf=b.conf, source='mono',
                                     class_scores={b.cls: b.conf}))
        return [d for d in out if d.conf >= self.min_conf]


class MultiCameraDetectorEstimator(DetectorDepthEstimator):
    """Línea base con las dos cámaras: el mismo detector 2D en ambas imágenes.

    Hereden e implementen `detect` (y, si quieren otro detector para la cámara
    del Dofbot, `detect_mono`). La geometría y la fusión ya están resueltas.
    """

    name = 'baseline_multicam'
    cameras = ('rgbd', 'mono')

    def __init__(self, fusion=None, **kwargs):
        super().__init__(**kwargs)
        self.fusion_kwargs = dict(fusion or {})
        self.fusion = MultiCameraFusion(**self.fusion_kwargs)

    def setup(self, gt=None, cfg=None):
        if cfg is not None:
            kw = dict({'size_prior': size_priors(cfg['objects'])}, **self.fusion_kwargs)
            self.fusion = MultiCameraFusion(**kw)

    def detect_mono(self, rgb, instruction):
        return self.detect(rgb, instruction)

    def estimate(self, frame):
        dets = super().estimate(frame)
        if frame.mono is None or frame.mono.rgb is None:
            return dets
        return self.fusion.fuse(dets, self.detect_mono(frame.mono.rgb, frame.instruction), frame)
