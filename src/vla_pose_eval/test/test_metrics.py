import math

import numpy as np
import pytest

from vla_pose_eval import metrics as m


def test_error_decomposition():
    cam = np.array([0.0, 0.0, 0.5])
    gt = np.array([1.0, 0.0, 0.2])
    c = m.error_components(gt + [0.03, -0.01, 0.02], gt, cam)
    assert c['e_radial'] == pytest.approx(0.03)        # más lejos sobre el rayo
    assert c['e_transversal'] == pytest.approx(-0.01)  # a la derecha
    assert c['e_z'] == pytest.approx(0.02)
    assert c['e_pos'] ** 2 == pytest.approx(
        c['e_radial'] ** 2 + c['e_transversal'] ** 2 + c['e_z'] ** 2)


def test_associate_one_to_one_and_threshold():
    gt = [[0, 0, 0], [1, 0, 0]]
    est = [[0.05, 0, 0], [0.02, 0, 0], [1.3, 0, 0]]
    pairs, un_est, un_gt = m.associate(est, gt, 0.15)
    assert [(i, j) for i, j, _ in pairs] == [(1, 0)]   # la más cercana gana
    assert un_est == [0, 2]                            # falsos positivos
    assert un_gt == [1]                                # omisión
    assert m.associate([], gt, 0.15) == ([], [], [0, 1])


def test_wilson_matches_guide():
    """Guía, sección 8: 17 aciertos de 20 -> aproximadamente 64 a 95 %."""
    lo, hi = m.wilson_interval(17, 20)
    assert lo == pytest.approx(0.64, abs=0.005)
    assert hi == pytest.approx(0.95, abs=0.005)
    assert m.wilson_interval(0, 0)[0] != m.wilson_interval(0, 0)[0]   # NaN


def test_bootstrap_by_scene_is_wider_than_by_image():
    rng = np.random.default_rng(1)
    scene_bias = rng.normal(0.05, 0.02, 12)            # cada escena tiene su sesgo
    values = np.concatenate([b + rng.normal(0, 0.002, 30) for b in scene_bias])
    scenes = np.repeat(np.arange(12), 30)
    lo_s, hi_s = m.bootstrap_ci(values, scenes, n_resamples=500)
    lo_i, hi_i = m.bootstrap_ci(values, np.arange(len(values)), n_resamples=500)
    assert hi_s - lo_s > 2 * (hi_i - lo_i)
    assert lo_s <= np.median(values) <= hi_s


def test_describe():
    d = m.describe([0.01, 0.02, 0.03, float('nan')], ['a', 'a', 'b', 'b'], n_resamples=100)
    assert d['n'] == 3 and d['median'] == pytest.approx(0.02) and d['n_groups'] == 2
    assert m.describe([])['n'] == 0


def test_fuse_average_and_vote():
    est = [{'position': [1.00, 0, 0], 'class': 'cubo', 'conf': 0.9, 'yaw': 0.1},
           {'position': [1.04, 0, 0], 'class': 'cubo', 'conf': 0.8, 'yaw': 0.3},
           {'position': [1.02, 0, 0], 'class': 'caja', 'conf': 0.6, 'yaw': 2.0},
           {'position': [2.00, 0, 0], 'class': 'caja', 'conf': 0.7, 'yaw': None}]
    fused = sorted(m.fuse(est, 0.15), key=lambda f: f['position'][0])
    assert len(fused) == 2
    assert fused[0]['class'] == 'cubo' and fused[0]['n_views'] == 3
    assert fused[0]['position'][0] == pytest.approx(1.02)
    assert fused[0]['yaw'] == pytest.approx(0.2)       # solo las vistas de la clase ganadora
    assert fused[1]['yaw'] is None
    assert math.isclose(fused[1]['position'][0], 2.0)
