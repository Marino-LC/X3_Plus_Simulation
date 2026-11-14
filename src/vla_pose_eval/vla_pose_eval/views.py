"""Lista de vistas de la pasada de reconocimiento y visibilidad geométrica."""

import math
from dataclasses import dataclass

import numpy as np

from . import geometry as g


@dataclass
class CameraModel:
    """Cámara pinhole fija respecto a base_footprint."""

    K: np.ndarray
    width: int
    height: int
    T_base_cam: np.ndarray          # marco óptico -> base_footprint
    depth_near: float = 0.0
    depth_far: float = float('inf')
    margin_px: float = 0.0

    @classmethod
    def from_config(cls, cam_cfg):
        """Modelo nominal a partir de la sección `camera` de eval.yaml."""
        w, h = int(cam_cfg['width']), int(cam_cfg['height'])
        pitch = math.radians(cam_cfg.get('pitch_down_deg', 0.0))
        cp, sp = math.cos(pitch), math.sin(pitch)
        R_pitch = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        T = g.make_T(R_pitch @ g.R_BODY_OPTICAL, cam_cfg['xyz_in_base'])
        return cls(K=g.intrinsics_from_fov(w, h, float(cam_cfg['hfov'])),
                   width=w, height=h, T_base_cam=T,
                   depth_near=float(cam_cfg.get('depth_near', 0.0)),
                   depth_far=float(cam_cfg.get('depth_far', float('inf'))),
                   margin_px=float(cam_cfg.get('image_margin_px', 0.0)))


def load_views(views_cfg):
    """Devuelve la lista de vistas [{'x','y','yaw'} en m y rad]."""
    if views_cfg.get('views'):
        return [{'x': float(v['x']), 'y': float(v['y']),
                 'yaw': g.wrap(math.radians(float(v['yaw_deg'])))}
                for v in views_cfg['views']]
    grid = views_cfg['grid']
    step = float(grid['yaw_step_deg'])
    if step <= 0 or step > 45.0 + 1e-9:
        raise ValueError('yaw_step_deg debe estar en (0, 45] (guía, sección 4)')
    n = int(round(360.0 / step))
    start = float(grid.get('yaw_start_deg', 0.0))
    return [{'x': float(p['x']), 'y': float(p['y']),
             'yaw': g.wrap(math.radians(start + k * step))}
            for p in grid['positions'] for k in range(n)]


def view_T_map_base(view):
    return g.pose2d_to_T(view['x'], view['y'], view['yaw'])


def is_visible(p_map, T_map_base, cam):
    """¿El punto cae dentro del cuadro y del rango de profundidad?

    Es un criterio puramente geométrico (centro del objeto): NO considera
    oclusiones. Devuelve (visible, u, v, profundidad).
    """
    T_cam_map = g.inv_T(T_map_base @ cam.T_base_cam)
    u, v, z = g.project(cam.K, g.transform_point(T_cam_map, p_map))
    m = cam.margin_px
    ok = (z > 0.0 and cam.depth_near <= z <= cam.depth_far
          and m <= u <= cam.width - m and m <= v <= cam.height - m)
    return bool(ok), u, v, z


def coverage(points_map, views, cam):
    """Para cada punto, lista de índices de vistas donde es visible."""
    out = []
    for p in points_map:
        out.append([i for i, v in enumerate(views)
                    if is_visible(p, view_T_map_base(v), cam)[0]])
    return out
