"""Estimador oráculo: la verdad de referencia más ruido configurable.

NO es un modelo. Sirve para validar el flujo completo (E0: «un ensayo completo
produce el CSV con el error por objeto») y para comprobar que el evaluador
recupera el error que se le inyecta. Con todos los ruidos en cero, el error
reportado debe ser ~0: si no lo es, hay un fallo de marcos o de TF.

Con `use_mono=True` (alias `oracle_multicam`) simula también un detector 2D en
la cámara del Dofbot (cajas = proyección de la caja envolvente del objeto, con
ruido) y aplica la misma fusión que la línea base de dos cámaras.
"""

import itertools

import numpy as np

from .. import geometry as g
from ..scene import shape_dims
from ..views import CameraModel, is_visible
from .base import Detection, Estimator
from .detector_depth import Box2D
from .multicam import MultiCameraFusion, size_priors


def _silhouette_points(obj, n=24):
    """Puntos de la superficie (en el marco del objeto) cuya proyección acota la silueta."""
    dx, dy, dz = (v / 2.0 for v in shape_dims(obj['shape'], obj['size']))
    if obj['shape'] == 'box':
        return [np.array([sx * dx, sy * dy, sz * dz])
                for sx, sy, sz in itertools.product((-1, 1), repeat=3)]
    a = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    if obj['shape'] == 'cylinder':
        return [np.array([dx * np.cos(t), dx * np.sin(t), sz * dz]) for t in a for sz in (-1, 1)]
    # esfera: círculos de latitud
    return [dx * np.array([np.cos(t) * np.cos(p), np.sin(t) * np.cos(p), np.sin(p)])
            for t in a for p in np.linspace(-np.pi / 2, np.pi / 2, 9)]


def project_box(obj, T_cam_map, K, width, height):
    """Caja 2D (x0, y0, x1, y1) de la silueta del objeto, recortada a la imagen.

    Es lo que daría un detector ideal: la caja de la silueta real (no de la caja
    envolvente, que en un cilindro crecería con el giro).
    """
    R = g.rot_z(obj['yaw'])
    c = np.array([obj['x'], obj['y'], obj['z']])
    uv = []
    for q in _silhouette_points(obj):
        p = g.transform_point(T_cam_map, c + R @ q)
        if p[2] <= 0:
            return None
        u, v, _ = g.project(K, p)
        uv.append((u, v))
    uv = np.array(uv)
    x0, y0 = np.clip(uv.min(axis=0), 0, [width - 1, height - 1])
    x1, y1 = np.clip(uv.max(axis=0), 0, [width - 1, height - 1])
    return float(x0), float(y0), float(x1), float(y1)


class OracleEstimator(Estimator):

    def __init__(self, sigma_radial=0.0, sigma_transversal=0.0, sigma_z=0.0,
                 sigma_yaw_deg=0.0, drop_prob=0.0, false_positives_per_view=0.0,
                 class_error_prob=0.0, include_distractors=False, seed=0,
                 use_mono=False, sigma_box_px=0.0, mono_drop_prob=None,
                 mono_class_error_prob=None, mono_false_positives_per_view=None,
                 fusion=None, name=None, conf_range=(0.7, 1.0),
                 wrong_class_conf_range=(0.5, 0.8)):
        self.sigma = (float(sigma_radial), float(sigma_transversal), float(sigma_z))
        self.sigma_yaw = np.radians(float(sigma_yaw_deg))
        self.drop_prob = float(drop_prob)
        self.fp_rate = float(false_positives_per_view)
        self.class_error_prob = float(class_error_prob)
        self.include_distractors = bool(include_distractors)
        self.rng = np.random.default_rng(int(seed))
        self.use_mono = bool(use_mono)
        self.sigma_box = float(sigma_box_px)
        self.mono_drop_prob = self.drop_prob if mono_drop_prob is None else float(mono_drop_prob)
        self.mono_class_error_prob = (self.class_error_prob if mono_class_error_prob is None
                                      else float(mono_class_error_prob))
        self.mono_fp_rate = (self.fp_rate if mono_false_positives_per_view is None
                             else float(mono_false_positives_per_view))
        self.fusion_kwargs = dict(fusion or {})
        # Confianza de un acierto y de un error de clase. Que los errores tengan
        # menos confianza (detector calibrado) es lo que permite que el voto
        # entre cámaras corrija clases; con rangos iguales el voto no ayuda.
        self.conf_range = tuple(float(v) for v in conf_range)
        self.wrong_conf_range = tuple(float(v) for v in wrong_class_conf_range)
        self.name = name or ('oracle_multicam' if self.use_mono else 'oracle')
        self.cameras = ('rgbd', 'mono') if self.use_mono else ('rgbd',)
        self.gt = None
        self.cam_cfg = {}

    def setup(self, gt=None, cfg=None):
        if gt is None:
            raise ValueError('el oráculo necesita la verdad de referencia')
        self.gt = gt
        self.cam_cfg = cfg['eval']['cameras'] if cfg else {}
        self.classes = sorted({o['class'] for o in gt['objects'] if not o['distractor']})
        self.fusion = MultiCameraFusion(
            **dict({'size_prior': size_priors(cfg['objects']) if cfg else {}},
                   **self.fusion_kwargs))

    def _classify(self, cls, prob):
        """(clase, confianza) con probabilidad `prob` de equivocarse de clase."""
        if len(self.classes) > 1 and self.rng.random() < prob:
            return (str(self.rng.choice([c for c in self.classes if c != cls])),
                    float(self.rng.uniform(*self.wrong_conf_range)))
        return cls, float(self.rng.uniform(*self.conf_range))

    def _objects(self):
        return [o for o in self.gt['objects'] if self.include_distractors or not o['distractor']]

    def _rgbd(self, frame, T_map_base):
        cam = CameraModel.from_meta(frame.meta, self.cam_cfg.get('rgbd', {}))
        T_cam_map = g.inv_T(T_map_base @ cam.T_base_cam)
        dets = []
        for o in self._objects():
            p_map = [o['x'], o['y'], o['z']]
            if not is_visible(p_map, T_map_base, cam)[0] or self.rng.random() < self.drop_prob:
                continue
            p_cam = g.transform_point(T_cam_map, p_map)
            # Ruido radial (eje óptico), transversal (x óptico) y vertical (y óptico).
            noise = self.rng.normal(0.0, 1.0, 3) * np.array(
                [self.sigma[1], self.sigma[2], self.sigma[0]])
            yaw = o['yaw'] + self.rng.normal(0.0, 1.0) * self.sigma_yaw
            cls, conf = self._classify(o['class'], self.class_error_prob)
            dets.append(Detection(cls=cls, position=list(p_cam + noise), conf=conf,
                                  orientation_xyzw=g.R_to_quat(T_cam_map[:3, :3] @ g.rot_z(yaw))))
        for _ in range(self.rng.poisson(self.fp_rate)):
            z = self.rng.uniform(1.0, 2.0)
            u, v = self.rng.uniform(0, cam.width), self.rng.uniform(0, cam.height)
            dets.append(Detection(cls=str(self.rng.choice(self.classes)),
                                  position=list(g.backproject(cam.K, u, v, z)),
                                  conf=float(self.rng.uniform(0.3, 0.6))))
        return dets

    def _mono_boxes(self, frame, T_map_base):
        meta = frame.meta['mono']
        cam = CameraModel.from_meta(meta, self.cam_cfg.get('mono', {}))
        T_cam_map = g.inv_T(T_map_base @ cam.T_base_cam)
        boxes = []
        for o in self._objects():
            if (not is_visible([o['x'], o['y'], o['z']], T_map_base, cam)[0]
                    or self.rng.random() < self.mono_drop_prob):
                continue
            box = project_box(o, T_cam_map, cam.K, cam.width, cam.height)
            if box is None:
                continue
            x0, y0, x1, y1 = np.array(box) + self.rng.normal(0.0, self.sigma_box, 4)
            if x1 - x0 < 2 or y1 - y0 < 2:
                continue
            cls, conf = self._classify(o['class'], self.mono_class_error_prob)
            boxes.append(Box2D(cls, conf, (x0, y0, x1, y1)))
        for _ in range(self.rng.poisson(self.mono_fp_rate)):
            w = self.rng.uniform(15, 60)
            x0, y0 = self.rng.uniform(0, cam.width - w), self.rng.uniform(0, cam.height - w)
            boxes.append(Box2D(str(self.rng.choice(self.classes)),
                               float(self.rng.uniform(0.3, 0.6)), (x0, y0, x0 + w, y0 + w)))
        return boxes

    def estimate(self, frame):
        T_map_base = np.asarray(frame.meta['T_map_base'], float).reshape(4, 4)
        dets = self._rgbd(frame, T_map_base)
        if not self.use_mono or frame.mono is None or not frame.meta.get('mono'):
            return dets
        return self.fusion.fuse(dets, self._mono_boxes(frame, T_map_base), frame)
