import math

import numpy as np
import pytest

from vla_pose_eval import geometry as g
from vla_pose_eval.views import CameraModel, is_visible, load_views


def test_wrap():
    assert g.wrap(3 * math.pi) == pytest.approx(math.pi)
    assert g.wrap(-math.pi / 2) == pytest.approx(-math.pi / 2)
    assert -math.pi < g.wrap(-math.pi) <= math.pi


@pytest.mark.parametrize('est_deg, gt_deg, sym, expected', [
    (100, 0, 90, 10),        # cubo: se ve igual cada 90 grados
    (80, 0, 90, 10),
    (100, 0, 180, 80),       # caja rectangular
    (190, 0, 180, 10),
    (350, 0, 360, 10),       # sin simetría (taza con asa)
    (-170, 170, 360, 20),
])
def test_yaw_error_symmetry(est_deg, gt_deg, sym, expected):
    e = g.yaw_error(math.radians(est_deg), math.radians(gt_deg), sym)
    assert math.degrees(e) == pytest.approx(expected, abs=1e-6)


def test_yaw_error_undefined():
    assert math.isnan(g.yaw_error(0.3, 0.1, 0))       # simetría continua
    assert math.isnan(g.yaw_error(None, 0.1, 90))     # el modelo no estimó orientación


def test_quaternion_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = rng.normal(size=4)
        R = g.quat_to_R(q)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
        assert np.allclose(g.quat_to_R(g.R_to_quat(R)), R, atol=1e-9)
    assert g.yaw_from_R(g.quat_to_R(g.yaw_to_quat(0.7))) == pytest.approx(0.7)


def test_transform_inverse():
    T = g.pose2d_to_T(0.3, -0.4, 1.1)
    p = np.array([0.5, 0.2, 0.1])
    assert np.allclose(g.transform_point(g.inv_T(T), g.transform_point(T, p)), p)


def test_project_backproject():
    K = g.intrinsics_from_fov(640, 480, 1.0193)
    p = np.array([0.1, -0.05, 1.3])
    u, v, z = g.project(K, p)
    assert np.allclose(g.backproject(K, u, v, z), p)


def test_optical_convention(cfg):
    """Un punto al frente-izquierda-abajo del robot: z>0, x<0 (izq.), y>0 (abajo)."""
    cam = CameraModel.from_config(cfg['eval']['camera'])
    p_cam = g.transform_point(g.inv_T(cam.T_base_cam), [1.0, 0.2, 0.2])
    assert p_cam[2] > 0 and p_cam[0] < 0 and p_cam[1] > 0
    assert p_cam[2] == pytest.approx(1.0 - cfg['eval']['camera']['xyz_in_base'][0])


def test_visibility(cfg):
    cam = CameraModel.from_config(cfg['eval']['camera'])
    T = g.pose2d_to_T(0.0, 0.0, math.pi / 2)          # mirando a +Y
    assert is_visible([0.0, 1.2, 0.22], T, cam)[0]
    assert not is_visible([0.0, -1.2, 0.22], T, cam)[0]   # detrás
    assert not is_visible([1.2, 0.0, 0.22], T, cam)[0]    # a 90 grados: fuera del FOV
    assert not is_visible([0.0, 0.4, 0.22], T, cam)[0]    # más cerca que depth_near


def test_views_grid(cfg):
    views = load_views(cfg['views'])
    assert len(views) == 3 * 8
    assert views[0]['yaw'] == pytest.approx(math.pi / 2)
    with pytest.raises(ValueError):
        load_views({'grid': {'positions': [{'x': 0, 'y': 0}], 'yaw_step_deg': 60}})
