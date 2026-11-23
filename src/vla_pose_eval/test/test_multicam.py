import math

import numpy as np
import pytest

from vla_pose_eval import geometry as g
from vla_pose_eval.estimators import CameraImage, Detection, Frame
from vla_pose_eval.estimators.detector_depth import Box2D
from vla_pose_eval.estimators.multicam import (MultiCameraDetectorEstimator, MultiCameraFusion,
                                               SizePrior, size_priors)
from vla_pose_eval.estimators.oracle import project_box
from vla_pose_eval.views import nominal_cameras


@pytest.fixture
def setup(cfg):
    cams = nominal_cameras(cfg['eval'])
    rg, mo = cams['rgbd'], cams['mono']
    mono = CameraImage(name='mono', rgb=None, K=mo.K, width=mo.width, height=mo.height,
                       T_base_cam=mo.T_base_cam)
    frame = Frame(view=0, rgb=None, depth=None, K=rg.K, T_base_cam=rg.T_base_cam, mono=mono)
    return cfg, cams, frame


def _in_rgbd(cams, p_base):
    return list(g.transform_point(g.inv_T(cams['rgbd'].T_base_cam), p_base))


def _mono_box(cams, obj):
    T_cam_map = g.inv_T(cams['mono'].T_base_cam)      # robot en el origen: map == base
    return project_box(obj, T_cam_map, cams['mono'].K, cams['mono'].width, cams['mono'].height)


def _obj(x, y, z, cls='cubo', yaw=0.3):
    return {'x': x, 'y': y, 'z': z, 'yaw': yaw, 'shape': 'box', 'size': [0.04, 0.04, 0.04],
            'class': cls}


def test_nominal_mono_camera_matches_fk(cfg):
    """La cámara del Dofbot en pose de observación mira al frente desde ~0.35 m."""
    mono = nominal_cameras(cfg['eval'])['mono']
    z_opt = mono.T_base_cam[:3, 2]
    assert z_opt[0] > 0.99 and abs(z_opt[2]) < 0.01
    assert mono.T_base_cam[2, 3] == pytest.approx(0.35)
    assert mono.T_base_cam[:3, 1][2] < -0.99           # y óptico hacia abajo: imagen derecha


def test_vote_fixes_class(setup):
    cfg, cams, frame = setup
    o = _obj(1.2, 0.0, 0.25)
    det = Detection('caja', _in_rgbd(cams, [o['x'], o['y'], o['z']]), conf=0.55)
    box = Box2D('cubo', 0.9, _mono_box(cams, o))
    out = MultiCameraFusion(size_priors(cfg['objects'])).fuse([det], [box], frame)
    assert len(out) == 1 and out[0].cls == 'cubo' and out[0].source == 'rgbd+mono'
    assert out[0].position == det.position             # la posición sigue siendo la 3D
    assert out[0].class_scores == {'caja': 0.55, 'cubo': 0.9}


def test_unconfirmed_detection_is_downweighted(setup):
    cfg, cams, frame = setup
    det = Detection('cubo', _in_rgbd(cams, [1.2, 0.0, 0.25]), conf=0.8)
    far = Detection('cubo', _in_rgbd(cams, [1.2, 1.0, 0.25]), conf=0.8)   # fuera del cuadro mono
    out = MultiCameraFusion({}, unconfirmed_factor=0.5, mono_only=False).fuse([det, far], [], frame)
    assert out[0].conf == pytest.approx(0.4) and out[0].source == 'rgbd'
    assert out[1].conf == pytest.approx(0.8)


@pytest.mark.parametrize('x, z, yaw', [(0.55, 0.32, 0.0), (0.7, 0.26, 0.7), (0.9, 0.22, 1.2)])
def test_mono_only_size_prior(setup, x, z, yaw):
    """Objeto cercano que solo ve el Dofbot: distancia por tamaño aparente."""
    cfg, cams, frame = setup
    o = _obj(x, 0.02, z, yaw=yaw)
    out = MultiCameraFusion(size_priors(cfg['objects'])).fuse(
        [], [Box2D('cubo', 0.9, _mono_box(cams, o))], frame)
    assert len(out) == 1 and out[0].source == 'mono'
    p_base = g.transform_point(cams['rgbd'].T_base_cam, out[0].position)
    err = np.linalg.norm(p_base - [x, 0.02, z])
    dist = np.linalg.norm(np.array([x, 0.02, z]) - cams['mono'].T_base_cam[:3, 3])
    # Sin conocer el giro del objeto, el tamaño aparente es ambiguo (un cubo de frente
    # se ve ~20 % más angosto que en diagonal): el error llega a ~20 % de la distancia.
    assert err < 0.2 * dist


def test_mono_only_skips_truncated_and_unknown(setup):
    cfg, cams, frame = setup
    W, H = cams['mono'].width, cams['mono'].height
    fusion = MultiCameraFusion(size_priors(cfg['objects']))
    assert fusion.fuse([], [Box2D('cubo', 0.9, (0, 100, 40, 140))], frame) == []      # borde
    assert fusion.fuse([], [Box2D('taza', 0.9, (300, 200, 340, 240))], frame) == []   # sin tamaño
    assert len(fusion.fuse([], [Box2D('cubo', 0.9, (300, 200, 340, 240))], frame)) == 1
    assert W > 340 and H > 240


def test_mono_only_dedupes_against_3d(setup):
    cfg, cams, frame = setup
    o = _obj(1.2, 0.0, 0.25)
    det = Detection('cubo', _in_rgbd(cams, [o['x'] + 0.08, 0.0, 0.25]), conf=0.8)  # 3D desplazada
    x0, y0, x1, y1 = _mono_box(cams, o)
    box = Box2D('cubo', 0.9, (x0 + 200, y0, x1 + 200, y1))                          # no coincide
    out = MultiCameraFusion(size_priors(cfg['objects']), match_margin_px=0).fuse([det], [box],
                                                                                 frame)
    assert [d.source for d in out] == ['rgbd', 'mono']
    near = Box2D('cubo', 0.9, (x0, y0, x1, y1))
    fusion = MultiCameraFusion(size_priors(cfg['objects']), match_margin_px=0, dedupe_radius_m=0.5)
    out = fusion.fuse([Detection('cubo', _in_rgbd(cams, [1.2, 0.0, 0.6]), conf=0.8)], [near], frame)
    assert [d.source for d in out] == ['rgbd']          # la caja mono se descarta por cercana


def test_size_prior_elevation():
    p = SizePrior.from_shape('box', [0.04, 0.04, 0.04])
    assert p.width == pytest.approx(2 * 0.08 / math.pi)
    assert p.apparent(0.0) < p.apparent(0.3)            # desde arriba se ve más alto
    s = SizePrior.from_shape('sphere', [0.025])
    assert s.apparent(0.0) == s.apparent(0.8) == pytest.approx(0.05)


def test_multicam_baseline_calls_detector_on_both(setup):
    cfg, cams, frame = setup
    calls = []

    class Fake(MultiCameraDetectorEstimator):
        def detect(self, rgb, instruction):
            calls.append(rgb.shape)
            return []

    est = Fake()
    est.setup(cfg=cfg)
    frame.rgb = np.zeros((720, 1280, 3), np.uint8)
    frame.depth = np.full((720, 1280), 2.0, np.float32)
    frame.mono.rgb = np.zeros((480, 640, 3), np.uint8)
    assert est.estimate(frame) == []
    assert calls == [(720, 1280, 3), (480, 640, 3)]
    assert est.cameras == ('rgbd', 'mono')
