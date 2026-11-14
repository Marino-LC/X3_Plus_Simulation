"""Estimador oráculo: la verdad de referencia más ruido configurable.

NO es un modelo. Sirve para validar el flujo completo (E0: «un ensayo completo
produce el CSV con el error por objeto») y para comprobar que el evaluador
recupera el error que se le inyecta. Con todos los ruidos en cero, el error
reportado debe ser ~0: si no lo es, hay un fallo de marcos o de TF.
"""

import numpy as np

from .. import geometry as g
from ..views import CameraModel, is_visible
from .base import Detection, Estimator


class OracleEstimator(Estimator):
    name = 'oracle'

    def __init__(self, sigma_radial=0.0, sigma_transversal=0.0, sigma_z=0.0,
                 sigma_yaw_deg=0.0, drop_prob=0.0, false_positives_per_view=0.0,
                 class_error_prob=0.0, include_distractors=False, seed=0):
        self.sigma = (float(sigma_radial), float(sigma_transversal), float(sigma_z))
        self.sigma_yaw = np.radians(float(sigma_yaw_deg))
        self.drop_prob = float(drop_prob)
        self.fp_rate = float(false_positives_per_view)
        self.class_error_prob = float(class_error_prob)
        self.include_distractors = bool(include_distractors)
        self.rng = np.random.default_rng(int(seed))
        self.gt = None
        self.cam_cfg = None

    def setup(self, gt=None, cfg=None):
        if gt is None:
            raise ValueError('el oráculo necesita la verdad de referencia')
        self.gt = gt
        self.cam_cfg = cfg['eval']['camera'] if cfg else {}

    def estimate(self, frame):
        T_map_base = np.asarray(frame.meta['T_map_base'], float).reshape(4, 4)
        T_base_cam = np.asarray(frame.meta['T_base_cam'], float).reshape(4, 4)
        T_cam_map = g.inv_T(T_map_base @ T_base_cam)
        cam = CameraModel(K=np.asarray(frame.K, float), width=frame.meta['width'],
                          height=frame.meta['height'], T_base_cam=T_base_cam,
                          depth_near=float(self.cam_cfg.get('depth_near', 0.0)),
                          depth_far=float(self.cam_cfg.get('depth_far', np.inf)),
                          margin_px=float(self.cam_cfg.get('image_margin_px', 0.0)))
        classes = sorted({o['class'] for o in self.gt['objects'] if not o['distractor']})
        dets = []
        for o in self.gt['objects']:
            if o['distractor'] and not self.include_distractors:
                continue
            p_map = [o['x'], o['y'], o['z']]
            if not is_visible(p_map, T_map_base, cam)[0]:
                continue
            if self.rng.random() < self.drop_prob:
                continue
            p_cam = g.transform_point(T_cam_map, p_map)
            # Ruido radial (eje óptico), transversal (x óptico) y vertical (y óptico).
            noise = self.rng.normal(0.0, 1.0, 3) * np.array(
                [self.sigma[1], self.sigma[2], self.sigma[0]])
            yaw = o['yaw'] + self.rng.normal(0.0, 1.0) * self.sigma_yaw
            R_cam_obj = T_cam_map[:3, :3] @ g.rot_z(yaw)
            cls = o['class']
            if len(classes) > 1 and self.rng.random() < self.class_error_prob:
                cls = self.rng.choice([c for c in classes if c != cls])
            dets.append(Detection(cls=str(cls), position=list(p_cam + noise),
                                  conf=float(self.rng.uniform(0.7, 1.0)),
                                  orientation_xyzw=g.R_to_quat(R_cam_obj)))
        for _ in range(self.rng.poisson(self.fp_rate)):
            z = self.rng.uniform(1.0, 2.0)
            u = self.rng.uniform(0, frame.meta['width'])
            v = self.rng.uniform(0, frame.meta['height'])
            dets.append(Detection(cls=str(self.rng.choice(classes)),
                                  position=list(g.backproject(cam.K, u, v, z)),
                                  conf=float(self.rng.uniform(0.3, 0.6))))
        return dets
