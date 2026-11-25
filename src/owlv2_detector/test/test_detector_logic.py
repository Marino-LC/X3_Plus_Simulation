"""Lógica del detector y del estimador OWLv2 sin el modelo (se simula `_infer`)."""

import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'vla_pose_eval'))

from owlv2_detector.detector import Owlv2Detector, iou, make_tiles, prompts_from_config  # noqa
from owlv2_detector.estimator import Owlv2Estimator  # noqa: E402
from vla_pose_eval import dataset, evaluate as ev, geometry as g, scene  # noqa: E402
from vla_pose_eval.config import load_config  # noqa: E402
from vla_pose_eval.views import view_cameras  # noqa: E402

CFG = load_config(os.path.join(HERE, '..', '..', 'vla_pose_eval', 'config'))


class FakeDetector(Owlv2Detector):
    """`_infer` devuelve cajas fijas, dadas en coordenadas de la imagen completa."""

    def __init__(self, full_boxes, *a, **kw):
        super().__init__(*a, **kw)
        self.full_boxes = full_boxes          # [(xyxy, puntaje, consulta)]
        self.calls = []

    def _infer(self, images, sides):
        tiles = make_tiles(self._w, self._h, self.tiles_mode, self.tile_overlap)
        self.calls.append([im.shape for im in images])
        out = []
        for x0, y0, s in tiles:
            bx, sc, lb = [], [], []
            for (b, score, q) in self.full_boxes:
                tb = [b[0] - x0, b[1] - y0, b[2] - x0, b[3] - y0]
                if tb[2] <= 0 or tb[3] <= 0 or tb[0] >= s or tb[1] >= s:
                    continue
                bx.append([max(tb[0], 0), max(tb[1], 0), min(tb[2], s), min(tb[3], s)])
                sc.append(score)
                lb.append(self.queries.index(q))
            out.append((np.array(bx).reshape(-1, 4), np.array(sc), np.array(lb, int)))
        return out

    def detect_raw(self, rgb):
        self._h, self._w = rgb.shape[:2]
        return super().detect_raw(rgb)


CLASSES = {'cubo': ['a small cube'], 'caja': ['a small rectangular bar']}
NEG = ['a large blue pedestal']


def test_config_prompts():
    classes, negatives, opts = prompts_from_config(CFG['objects'])
    assert set(classes) == {'cubo', 'caja', 'cilindro'}
    assert all(classes.values()) and negatives and 'threshold' in opts
    assert not any('red' in p for ps in classes.values() for p in ps)


def test_tiles_cover_image():
    tiles = make_tiles(1280, 720)
    assert tiles == [(0, 0, 720), (560, 0, 720)]
    assert make_tiles(640, 480) == [(0, 0, 640)]            # cámara del Dofbot: sin mosaicos


def test_negative_query_suppresses_pedestal():
    ped = (600.0, 300.0, 700.0, 600.0)
    cube = (630.0, 260.0, 670.0, 300.0)
    det = FakeDetector([(ped, 0.30, 'a large blue pedestal'), (ped, 0.25, 'a small cube'),
                        (cube, 0.18, 'a small cube')], CLASSES, NEG, threshold=0.1)
    out = det.detect(np.zeros((720, 1280, 3), np.uint8))
    assert [(c, round(s, 2)) for c, s, _ in out] == [('cubo', 0.18)]


def test_class_threshold_and_mapping():
    bar = (100.0, 100.0, 160.0, 130.0)
    det = FakeDetector([(bar, 0.09, 'a small rectangular bar')], CLASSES, NEG, threshold=0.1,
                       class_thresholds={'caja': 0.08})
    out = det.detect(np.zeros((720, 1280, 3), np.uint8))
    assert out and out[0][0] == 'caja'                       # clase del evaluador, no la consulta
    det.class_thresholds = {}
    assert det.detect(np.zeros((720, 1280, 3), np.uint8)) == []


def test_tile_duplicates_and_cut_boxes():
    """Objeto en el solape: una sola caja; el recorte del mosaico izquierdo se descarta."""
    obj = (690.0, 300.0, 740.0, 340.0)                       # cruza x=720 (borde del mosaico 1)
    det = FakeDetector([(obj, 0.4, 'a small cube')], CLASSES, NEG)
    out = det.detect(np.zeros((720, 1280, 3), np.uint8))
    assert len(out) == 1 and out[0][2] == pytest.approx(obj)
    assert det.calls[-1] == [(720, 720, 3), (720, 720, 3)]


def test_iou():
    assert iou((0, 0, 10, 10), (0, 0, 10, 10)) == 1.0
    assert iou((0, 0, 10, 10), (20, 20, 30, 30)) == 0.0


# ── Estimador completo con profundidad trazada por rayos ─────────────────────
def _raycast_depth(cam, T_map_base, boxes):
    """Profundidad (eje óptico) de cajas orientadas [(centro, dims, yaw)]."""
    T = T_map_base @ cam.T_base_cam
    vs, us = np.mgrid[0:cam.height, 0:cam.width]
    rays = np.stack([(us + 0.5 - cam.K[0, 2]) / cam.K[0, 0],
                     (vs + 0.5 - cam.K[1, 2]) / cam.K[1, 1], np.ones_like(us, float)], -1)
    d_map = rays.reshape(-1, 3) @ T[:3, :3].T
    o = T[:3, 3]
    best = np.full(d_map.shape[0], np.inf)
    for c, dims, yaw in boxes:
        R = g.rot_z(yaw)
        ol, dl = (o - c) @ R, d_map @ R
        half = np.asarray(dims) / 2.0
        with np.errstate(divide='ignore', invalid='ignore'):
            t1, t2 = (-half - ol) / dl, (half - ol) / dl
        tmin = np.nanmax(np.minimum(t1, t2), axis=1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=1)
        hit = (tmax >= tmin) & (tmin > 0)
        best = np.where(hit & (tmin < best), tmin, best)
    return best.reshape(cam.height, cam.width).astype(np.float32)


def test_estimator_rejects_pedestal_and_lifts_objects(tmp_path):
    gt = scene.generate_scene('S2', 3, CFG)
    trial = str(tmp_path / gt['scene_id'])
    os.makedirs(os.path.join(trial, 'views'))
    dataset.dump_yaml(gt, dataset.gt_path(trial))
    meta = ev.synthetic_views_meta(gt, CFG)
    meta['views'] = [v for v in meta['views'] if any(
        v['view'] in o['visible_from_views'] for o in gt['objects'])][:3]
    boxes3d = [((p['x'], p['y'], p['z']), p['size'], p['yaw']) for p in gt['pedestals']]
    boxes3d += [((o['x'], o['y'], o['z']), scene.shape_dims(o['shape'], o['size']), o['yaw'])
                for o in gt['objects']]
    fake = {}
    for vm in meta['views']:
        cams = view_cameras(vm, CFG['eval'])
        T_map_base = np.reshape(vm['T_map_base'], (4, 4))
        depth = _raycast_depth(cams['rgbd'], T_map_base, boxes3d)
        np.save(os.path.join(trial, 'views', 'view_%03d_depth.npy' % vm['view']), depth)
        vm['depth'] = 'view_%03d_depth.npy' % vm['view']
        vm['rgb'] = dataset.save_rgb(os.path.join(trial, 'views', 'view_%03d_rgb' % vm['view']),
                                     np.zeros((cams['rgbd'].height, cams['rgbd'].width, 3),
                                              np.uint8))
        vm.pop('mono')
        # "Detector": caja de cada objeto (bien clasificado) y de cada pedestal como "cubo".
        T_cam_map = g.inv_T(T_map_base @ cams['rgbd'].T_base_cam)
        from vla_pose_eval.estimators.oracle import project_box
        items = []
        for o in gt['objects']:
            b = project_box(o, T_cam_map, cams['rgbd'].K, cams['rgbd'].width,
                            cams['rgbd'].height)
            if b and vm['view'] in o['visible_from_views']:
                items.append((b, 0.5, CLASSES_ALL[o['class']]))
        for p in gt['pedestals']:
            b = project_box(dict(p, shape='box'), T_cam_map, cams['rgbd'].K,
                            cams['rgbd'].width, cams['rgbd'].height)
            if b and b[2] - b[0] > 5:
                items.append((b, 0.6, 'a small cube'))       # el error que veían en Gazebo
        fake[vm['view']] = items
    dataset.save_views_meta(trial, meta)

    est = Owlv2Estimator(use_mono=False, debug=False)
    est.setup(gt=gt, cfg=CFG)
    view = {'i': None}
    det = FakeDetector([], *prompts_from_config(CFG['objects'])[:2])
    orig = det.detect_raw

    def detect_raw(rgb):
        det.full_boxes = fake[view['i']]
        return orig(rgb)
    det.detect_raw = detect_raw
    est.detector = det
    errors, size_rejections = [], 0
    for vm in meta['views']:
        view['i'] = vm['view']
        dets = est.estimate(ev.make_frame(trial, vm))
        size_rejections += sum(1 for _, why in est.rejected if why.startswith('tamaño'))
        T = np.reshape(vm['T_map_base'], (4, 4)) @ np.reshape(vm['T_base_cam'], (4, 4))
        for d in dets:
            p = g.transform_point(T, d.position)
            errors.append(min(np.linalg.norm(p - [o['x'], o['y'], o['z']])
                              for o in gt['objects']))
    # Un pedestal levantado a 3D quedaría a > 0.15 m de cualquier objeto.
    assert errors and max(errors) < 0.08 and size_rejections > 0
    assert np.median(errors) < 0.03


CLASSES_ALL = {c: ps[0] for c, ps in prompts_from_config(CFG['objects'])[0].items()}
