import math
import os

import numpy as np
import pytest

from vla_pose_eval import cli, dataset, evaluate as ev, geometry as g, report, scene
from vla_pose_eval.estimators import Estimator, load_estimator
from vla_pose_eval.estimators.detector_depth import Box2D, DetectorDepthEstimator


def _trial(tmp_path, cfg, scene_type='S3', seed=2):
    gt = scene.generate_scene(scene_type, seed, cfg)
    trial = str(tmp_path / gt['scene_id'])
    os.makedirs(trial)
    dataset.dump_yaml(gt, dataset.gt_path(trial))
    dataset.save_views_meta(trial, ev.synthetic_views_meta(gt, cfg))
    return trial, gt


def _run(trial, cfg, **kwargs):
    est = load_estimator('oracle', **kwargs)
    ev.run_estimator(trial, est, cfg)
    return ev.evaluate_trial_dir(trial, 'oracle', cfg)


def test_perfect_estimator_has_zero_error(tmp_path, cfg):
    """Si esto falla hay un error de marcos, no del modelo."""
    trial, gt = _trial(tmp_path, cfg)
    rows, frows = _run(trial, cfg)
    tp = [r for r in rows if r['status'] == 'TP']
    assert tp and all(r['e_pos'] < 1e-9 for r in tp)
    assert all(r['class_ok'] == 1 for r in tp)
    assert all(r['e_yaw_deg'] < 1e-6 for r in tp if not math.isnan(r['e_yaw_deg']))
    assert not [r for r in rows if r['status'] in ('FN', 'FP')]
    assert {r['status'] for r in rows} == {'TP', 'OUT'}
    assert all(r['in_view'] == 0 for r in rows if r['status'] == 'OUT')
    # Pasada completa: todos los objetos vistos y sin error tras fusionar.
    assert [r['status'] for r in frows] == ['TP'] * gt['n_targets']
    assert all(r['e_pos'] < 1e-9 and r['in_view'] == 1 for r in frows)


def test_injected_error_is_recovered(tmp_path, cfg):
    """Un sesgo conocido a lo largo del eje óptico debe aparecer como error radial."""
    trial, gt = _trial(tmp_path, cfg)

    class Biased(Estimator):
        name = 'biased'

        def setup(self, gt=None, cfg=None):
            self.oracle = load_estimator('oracle')
            self.oracle.setup(gt=gt, cfg=cfg)

        def estimate(self, frame):
            dets = self.oracle.estimate(frame)
            for d in dets:
                d.position[2] += 0.04
                d.orientation_xyzw = None
            return dets

    ev.run_estimator(trial, Biased(), cfg)
    rows, _ = ev.evaluate_trial_dir(trial, 'biased', cfg)
    tp = [r for r in rows if r['status'] == 'TP']
    assert tp
    for r in tp:
        assert r['e_pos'] == pytest.approx(0.04, abs=1e-9)
        assert r['e_z'] == pytest.approx(0.0, abs=1e-9)
        assert r['e_radial'] == pytest.approx(0.04 * math.cos(math.radians(r['bearing_deg'])),
                                              abs=1e-9)
        assert math.isnan(r['e_yaw_deg'])


def test_estimate_frames_are_equivalent(tmp_path, cfg):
    trial, gt = _trial(tmp_path, cfg)
    est = load_estimator('oracle', sigma_radial=0.02, sigma_transversal=0.01, seed=3)
    data = ev.run_estimator(trial, est, cfg)
    meta = dataset.load_views_meta(trial)
    ref, _ = ev.evaluate_trial(gt, meta, data, cfg)
    by_view = {v['view']: v for v in meta['views']}
    in_map = {'estimator': 'oracle', 'frame': 'map', 'views': []}
    for v in data['views']:
        vm = by_view[v['view']]
        T = (np.reshape(vm['T_map_base'], (4, 4)) @ np.reshape(vm['T_base_cam'], (4, 4)))
        dets = [dict(d, position=g.transform_point(T, d['position']).tolist(),
                     orientation_xyzw=None) for d in v['detections']]
        in_map['views'].append(dict(v, detections=dets))
    other, _ = ev.evaluate_trial(gt, meta, in_map, cfg)
    assert [r['status'] for r in ref] == [r['status'] for r in other]
    for a, b in zip(ref, other):
        if a['status'] == 'TP':
            assert a['e_pos'] == pytest.approx(b['e_pos'], abs=1e-9)
            assert a['e_radial'] == pytest.approx(b['e_radial'], abs=1e-9)


def test_omissions_and_false_positives(tmp_path, cfg):
    trial, gt = _trial(tmp_path, cfg)
    rows, frows = _run(trial, cfg, drop_prob=1.0)
    assert {r['status'] for r in rows} == {'FN', 'OUT'}        # nada detectado
    assert all(r['status'] == 'FN' for r in frows)
    rows, _ = _run(trial, cfg, false_positives_per_view=2.0, seed=1)
    fp = [r for r in rows if r['status'] == 'FP']
    assert fp and all(r.get('obj_id') is None for r in fp)     # sin objeto real asociado


def test_distractor_detection_is_false_positive(tmp_path, cfg):
    trial, gt = _trial(tmp_path, cfg)
    assert any(o['distractor'] for o in gt['objects'])
    clean, _ = _run(trial, cfg)
    rows, _ = _run(trial, cfg, include_distractors=True)
    fp = [r for r in rows if r['status'] == 'FP']
    assert fp and all(r['distractor'] == 1 for r in fp)
    assert not [r for r in clean if r.get('distractor')]       # ignorarlo no genera filas


def test_baseline_lift_recovers_center():
    """Caja 2D + profundidad -> centro del objeto (con corrección superficie-centro)."""
    K = g.intrinsics_from_fov(640, 480, 1.0193)
    center = np.array([0.15, 0.10, 1.20])              # marco óptico
    half = 0.02
    depth = np.full((480, 640), 2.5, np.float32)       # pared de fondo
    u, v, _ = g.project(K, center)
    r = int(K[0, 0] * half / (center[2] - half))
    depth[int(v) - r:int(v) + r, int(u) - r:int(u) + r] = center[2] - half   # cara frontal
    box = Box2D('cubo', 0.9, (u - r - 6, v - r - 6, u + r + 6, v + r + 6))   # caja holgada
    est = DetectorDepthEstimator(center_offsets={'cubo': half})
    p = est.lift(box, depth, K)
    assert np.linalg.norm(p - center) < 0.005
    biased = DetectorDepthEstimator().lift(box, depth, K)       # sin corrección: sesgo radial
    assert np.linalg.norm(biased - center) == pytest.approx(half, abs=0.005)
    depth[:] = np.inf                                           # sin dato de profundidad
    assert est.lift(box, depth, K) is None
    with pytest.raises(NotImplementedError):
        est.detect(None, '')


def test_dry_run_end_to_end(tmp_path, cfg, capsys):
    out = str(tmp_path / 'runs')
    assert cli.main(['dry-run', '--out', out, '--seeds', '0', '4']) == 0
    trials = dataset.find_trials(out)
    assert len(trials) == 12
    assert all(os.path.isfile(dataset.errors_path(t, 'oracle')) for t in trials)
    summaries, md = report.build_report(out, ['oracle'], cfg)
    s = summaries['oracle']
    assert s['n_scenes'] == 12
    assert 0.0 < s['per_view']['e_pos_m']['median'] < 0.05
    lo, hi = s['per_view']['e_pos_m']['median_ci95']
    assert lo <= s['per_view']['e_pos_m']['median'] <= hi
    assert s['fused']['coverage']['rate'] == 1.0
    assert set(s['per_view']['cuts']) == {'distance_m', 'view_angle_deg', 'pedestal_height_m',
                                          'scene_type', 'class'}
    assert 'Wilson' in md
    assert cli.main(['check-views', out]) == 0
