"""Evaluador de un ensayo: estimaciones + verdad -> CSV de errores (guía, 4-5 y 8)."""

import csv
import math
import time

import numpy as np

from . import dataset, geometry as g, metrics
from .config import dump_yaml, load_yaml
from .estimators.base import Frame
from .views import CameraModel, is_visible, load_views, view_T_map_base

COLUMNS = [
    'scene_id', 'scene_type', 'seed', 'split', 'estimator', 'fused', 'view', 'n_views',
    'robot_x', 'robot_y', 'robot_yaw',
    'status',            # TP, FN (omisión), FP (falso positivo), OUT (fuera de cuadro)
    'obj_id', 'class_gt', 'distractor', 'pedestal_height', 'n_objects', 'in_view',
    'class_est', 'class_ok', 'conf',
    'gt_x', 'gt_y', 'gt_z', 'est_x', 'est_y', 'est_z',      # en base_footprint (map si fused=1)
    'e_pos', 'e_radial', 'e_transversal', 'e_z', 'e_yaw_deg',
    'distance', 'bearing_deg', 'view_angle_deg', 'infer_ms',
]


def _mat(v, n=4):
    return np.asarray(v, float).reshape(n, n)


def _est_in_base(det, frame, T_map_base, T_base_cam):
    """Estimación -> (posición, yaw | None) en base_footprint."""
    if frame == 'camera_optical':
        T = T_base_cam
    elif frame == 'base_footprint':
        T = np.eye(4)
    elif frame == 'map':
        T = g.inv_T(T_map_base)
    else:
        raise ValueError('marco de estimación desconocido: %r' % frame)
    p = g.transform_point(T, det['position'])
    yaw = None
    if det.get('orientation_xyzw') is not None:
        yaw = g.yaw_from_R(T[:3, :3] @ g.quat_to_R(det['orientation_xyzw']))
    return p, yaw


def evaluate_trial(gt, views_meta, estimates, cfg):
    """Devuelve (filas_por_vista, filas_fusionadas); ver COLUMNS."""
    ev = cfg['eval']
    thr = float(ev['evaluation']['association_threshold_m'])
    cam_cfg = ev['camera']
    objects = gt['objects']
    gt_map = np.array([[o['x'], o['y'], o['z']] for o in objects], float).reshape(-1, 3)
    est_by_view = {v['view']: v for v in estimates.get('views', [])}
    est_frame = estimates.get('frame', 'camera_optical')
    common = {'scene_id': gt['scene_id'], 'scene_type': gt['scene_type'], 'seed': gt['seed'],
              'split': gt.get('split'), 'estimator': estimates.get('estimator', '?'),
              'n_objects': len(objects)}
    rows, pool = [], []

    for vm in views_meta['views']:
        T_map_base, T_base_cam = _mat(vm['T_map_base']), _mat(vm['T_base_cam'])
        T_base_map = g.inv_T(T_map_base)
        cam = CameraModel(K=_mat(vm['K'], 3), width=vm['width'], height=vm['height'],
                          T_base_cam=T_base_cam, depth_near=float(cam_cfg['depth_near']),
                          depth_far=float(cam_cfg['depth_far']),
                          margin_px=float(cam_cfg.get('image_margin_px', 0.0)))
        cam_origin = T_base_cam[:3, 3]
        robot_yaw = g.yaw_from_R(T_map_base[:3, :3])
        ev_view = est_by_view.get(vm['view'], {})
        dets = ev_view.get('detections', []) or []
        est_b = [_est_in_base(d, est_frame, T_map_base, T_base_cam) for d in dets]
        est_map = [g.transform_point(T_map_base, p) for p, _ in est_b]
        pairs, un_est, _ = metrics.associate(est_map, gt_map, thr)
        match = {j: i for i, j, _ in pairs}
        base = dict(common, fused=0, view=vm['view'], n_views=1, robot_x=T_map_base[0, 3],
                    robot_y=T_map_base[1, 3], robot_yaw=robot_yaw,
                    infer_ms=ev_view.get('infer_ms'))

        for j, o in enumerate(objects):
            p_gt_b = g.transform_point(T_base_map, gt_map[j])
            in_view = is_visible(gt_map[j], T_map_base, cam)[0]
            ray = p_gt_b - cam_origin
            # Desde dónde se ve el objeto, en su propio marco (0 = de frente a su eje x).
            view_angle = g.wrap(math.atan2(-ray[1], -ray[0]) + robot_yaw - o['yaw'])
            row = dict(base, obj_id=o['id'], class_gt=o['class'], distractor=int(o['distractor']),
                       pedestal_height=o['pedestal_height'], in_view=int(in_view),
                       gt_x=p_gt_b[0], gt_y=p_gt_b[1], gt_z=p_gt_b[2],
                       distance=float(np.linalg.norm(ray[:2])),
                       bearing_deg=math.degrees(math.atan2(ray[1], ray[0])),
                       view_angle_deg=math.degrees(view_angle))
            if j in match:
                i = match[j]
                p_est, yaw_b = est_b[i]
                d = dets[i]
                row.update(metrics.error_components(p_est, p_gt_b, cam_origin))
                yaw_map = None if yaw_b is None else yaw_b + robot_yaw
                row.update(
                    # Una estimación asociada a un distractor es un falso positivo.
                    status='FP' if o['distractor'] else 'TP',
                    class_est=d['class'], class_ok=int(d['class'] == o['class']),
                    conf=d.get('conf'), est_x=p_est[0], est_y=p_est[1], est_z=p_est[2],
                    e_yaw_deg=math.degrees(g.yaw_error(yaw_map, o['yaw'], o['symmetry_deg'])))
                pool.append({'position': est_map[i], 'class': d['class'],
                             'conf': d.get('conf', 1.0), 'yaw': yaw_map})
            elif o['distractor']:
                continue                 # distractor ignorado correctamente: no genera fila
            else:
                # Fuera de cuadro no es error de posición ni omisión de esa vista;
                # si queda fuera en TODA la pasada, cuenta como omisión (fila fusionada).
                row['status'] = 'FN' if in_view else 'OUT'
            rows.append(row)
        for i in un_est:
            p_est, _ = est_b[i]
            rows.append(dict(base, status='FP', class_est=dets[i]['class'],
                             conf=dets[i].get('conf'), est_x=p_est[0], est_y=p_est[1],
                             est_z=p_est[2]))
            pool.append({'position': est_map[i], 'class': dets[i]['class'],
                         'conf': dets[i].get('conf', 1.0), 'yaw': None})

    # ── Pasada completa: fusión entre vistas (M2), en el marco map ──────────
    fused = metrics.fuse(pool, float(ev['evaluation']['fusion_radius_m']))
    pairs, un_est, _ = metrics.associate([f['position'] for f in fused], gt_map, thr)
    match = {j: i for i, j, _ in pairs}
    seen = {o['id']: any(r['in_view'] for r in rows if r.get('obj_id') == o['id'])
            for o in objects}
    fbase = dict(common, fused=1, view=-1)
    frows = []
    for j, o in enumerate(objects):
        row = dict(fbase, obj_id=o['id'], class_gt=o['class'], distractor=int(o['distractor']),
                   pedestal_height=o['pedestal_height'], in_view=int(seen[o['id']]),
                   gt_x=gt_map[j][0], gt_y=gt_map[j][1], gt_z=gt_map[j][2])
        if j in match:
            f = fused[match[j]]
            d = np.asarray(f['position']) - gt_map[j]
            row.update(status='FP' if o['distractor'] else 'TP', class_est=f['class'],
                       class_ok=int(f['class'] == o['class']), conf=f['conf'],
                       n_views=f['n_views'], est_x=f['position'][0], est_y=f['position'][1],
                       est_z=f['position'][2], e_pos=float(np.linalg.norm(d)), e_z=float(d[2]),
                       e_yaw_deg=math.degrees(g.yaw_error(f['yaw'], o['yaw'], o['symmetry_deg'])))
        elif o['distractor']:
            continue
        else:
            row['status'] = 'FN'         # incluye objetos nunca vistos (cobertura < 100 %)
        frows.append(row)
    for i in un_est:
        f = fused[i]
        frows.append(dict(fbase, status='FP', class_est=f['class'], conf=f['conf'],
                          n_views=f['n_views'], est_x=f['position'][0],
                          est_y=f['position'][1], est_z=f['position'][2]))
    return rows, frows


def write_csv(rows, path):
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow({k: ('' if v is None or (isinstance(v, float) and math.isnan(v))
                            else (round(v, 6) if isinstance(v, float) else v))
                        for k, v in ((c, r.get(c)) for c in COLUMNS)})


def evaluate_trial_dir(trial, estimator, cfg):
    """Evalúa `estimates_<estimator>.yaml` del ensayo y escribe los dos CSV."""
    gt = dataset.load_gt(trial)
    rows, frows = evaluate_trial(gt, dataset.load_views_meta(trial),
                                 load_yaml(dataset.estimates_path(trial, estimator)), cfg)
    write_csv(rows, dataset.errors_path(trial, estimator))
    write_csv(frows, dataset.errors_path(trial, estimator, fused=True))
    return rows, frows


def synthetic_views_meta(gt, cfg):
    """views.yaml sintético (sin Gazebo): poses ideales y cámara nominal.

    Para el modo dry-run y las pruebas; no contiene imágenes.
    """
    ev = cfg['eval']
    cam = CameraModel.from_config(ev['camera'])
    views = []
    for i, v in enumerate(load_views(cfg['views'])):
        views.append({'view': i, 'robot_pose_cmd': dict(v), 'robot_pose_source': 'synthetic',
                      'T_map_base': view_T_map_base(v).ravel().tolist(),
                      'T_base_cam': cam.T_base_cam.ravel().tolist(),
                      'K': cam.K.ravel().tolist(), 'width': cam.width, 'height': cam.height,
                      'rgb': None, 'depth': None})
    return {'scene_id': gt['scene_id'], 'source': 'synthetic', 'frames': dict(ev['frames']),
            'views': views}


def run_estimator(trial, estimator, cfg, instruction=''):
    """Ejecuta un estimador sobre las vistas guardadas y escribe sus estimaciones."""
    gt = dataset.load_gt(trial)
    meta = dataset.load_views_meta(trial)
    estimator.setup(gt=gt, cfg=cfg)
    out = []
    for vm in meta['views']:
        rgb, depth = dataset.load_frame_arrays(trial, vm)
        frame = Frame(view=vm['view'], rgb=rgb, depth=depth, K=_mat(vm['K'], 3),
                      instruction=instruction, meta=vm)
        t0 = time.perf_counter()
        dets = estimator.estimate(frame)
        out.append({'view': vm['view'], 'infer_ms': (time.perf_counter() - t0) * 1000.0,
                    'detections': [d.to_dict() for d in dets]})
    data = {'estimator': estimator.name, 'frame': 'camera_optical',
            'instruction': instruction, 'views': out}
    dump_yaml(data, dataset.estimates_path(trial, estimator.name))
    return data
