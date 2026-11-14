"""Transformaciones y geometría de cámara (numpy puro).

Convenciones
------------
* ``map``: marco fijo. En esta fase coincide con el marco del mundo de Gazebo.
* ``base_footprint``: x al frente, y a la izquierda, z arriba.
* Marco **óptico** de cámara: z al frente, x a la derecha, y abajo.
* Las transformaciones son matrices homogéneas 4x4 ``T_a_b`` que llevan
  puntos expresados en ``b`` al marco ``a``:  ``p_a = T_a_b @ p_b``.
"""

import math

import numpy as np

# Rotación del marco óptico expresado en un marco de cuerpo (x al frente):
# x_opt = -y_cuerpo, y_opt = -z_cuerpo, z_opt = +x_cuerpo.
R_BODY_OPTICAL = np.array([[0.0, 0.0, 1.0],
                           [-1.0, 0.0, 0.0],
                           [0.0, -1.0, 0.0]])


def wrap(angle):
    """Lleva un ángulo (rad) a (-pi, pi]."""
    a = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return a + 2.0 * math.pi if a <= -math.pi else a


def yaw_error(yaw_est, yaw_gt, symmetry_deg=360.0):
    """Error de yaw (rad, >= 0) módulo el ángulo de simetría del objeto.

    ``symmetry_deg`` es el giro mínimo que deja al objeto igual: 90 para un
    cubo, 180 para una caja rectangular, 360 si no hay simetría. ``None`` o 0
    indica simetría continua (cilindro, esfera): el yaw no está definido y se
    devuelve NaN.
    """
    if yaw_est is None or yaw_gt is None:
        return float('nan')
    if not symmetry_deg:
        return float('nan')
    delta = math.radians(symmetry_deg)
    d = (yaw_est - yaw_gt) % delta
    return min(d, delta - d)


def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def make_T(R=None, t=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = t
    return T


def pose2d_to_T(x, y, yaw, z=0.0):
    """Pose plana (x, y, yaw) -> T_map_base."""
    return make_T(rot_z(yaw), [x, y, z])


def inv_T(T):
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def transform_point(T, p):
    return T[:3, :3] @ np.asarray(p, dtype=float) + T[:3, 3]


def quat_to_R(q_xyzw):
    x, y, z, w = (float(v) for v in q_xyzw)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    """Matriz de rotación -> cuaternión (x, y, z, w)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return [x, y, z, w]


def yaw_from_R(R):
    """Yaw (giro sobre z) de una rotación expresada en un marco con z arriba."""
    return math.atan2(R[1, 0], R[0, 0])


def yaw_to_quat(yaw):
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


def intrinsics_from_fov(width, height, hfov):
    """Matriz K de una cámara pinhole ideal (píxeles cuadrados)."""
    f = (width / 2.0) / math.tan(hfov / 2.0)
    return np.array([[f, 0.0, width / 2.0],
                     [0.0, f, height / 2.0],
                     [0.0, 0.0, 1.0]])


def project(K, p_cam):
    """Punto en marco óptico -> (u, v, profundidad z)."""
    x, y, z = (float(v) for v in p_cam)
    if z <= 0.0:
        return float('nan'), float('nan'), z
    return K[0, 0] * x / z + K[0, 2], K[1, 1] * y / z + K[1, 2], z


def backproject(K, u, v, z):
    """Píxel (u, v) con profundidad z (a lo largo del eje óptico) -> punto 3D."""
    return np.array([(u - K[0, 2]) * z / K[0, 0],
                     (v - K[1, 2]) * z / K[1, 1],
                     z])
