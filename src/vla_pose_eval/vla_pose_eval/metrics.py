"""Métricas de la guía (secciones 5 y 8): error, asociación, fusión y estadística."""

import math
from collections import Counter

import numpy as np


# ── Error por estimación ─────────────────────────────────────────────────────
def error_components(p_est_base, p_gt_base, cam_origin_base):
    """Descompone el error de posición en `base_footprint`.

    * radial: a lo largo del rayo horizontal cámara -> objeto real
      (positivo = estimado más lejos). Lo domina la profundidad.
    * transversal: perpendicular al rayo en el plano horizontal
      (positivo = a la izquierda visto desde la cámara). Lo domina la detección.
    * z: vertical (positivo = estimado más arriba).

    Cumple  e_pos**2 == radial**2 + transversal**2 + z**2.
    """
    p_est, p_gt = np.asarray(p_est_base, float), np.asarray(p_gt_base, float)
    d = p_est - p_gt
    ray = (p_gt - np.asarray(cam_origin_base, float))[:2]
    n = float(np.linalg.norm(ray))
    r_hat = ray / n if n > 1e-9 else np.array([1.0, 0.0])
    t_hat = np.array([-r_hat[1], r_hat[0]])
    return {'e_pos': float(np.linalg.norm(d)),
            'e_radial': float(d[:2] @ r_hat),
            'e_transversal': float(d[:2] @ t_hat),
            'e_z': float(d[2])}


# ── Asociación estimación <-> objeto real ───────────────────────────────────
def associate(est_xyz, gt_xyz, threshold):
    """Empareja uno a uno por distancia creciente (voraz) bajo un umbral.

    Devuelve (pares [(i_est, i_gt, dist)], est_sin_pareja, gt_sin_pareja).
    """
    est = np.asarray(est_xyz, float).reshape(-1, 3)
    gt = np.asarray(gt_xyz, float).reshape(-1, 3)
    cand = []
    for i in range(len(est)):
        for j in range(len(gt)):
            dist = float(np.linalg.norm(est[i] - gt[j]))
            if dist <= threshold:
                cand.append((dist, i, j))
    cand.sort()
    used_e, used_g, pairs = set(), set(), []
    for dist, i, j in cand:
        if i in used_e or j in used_g:
            continue
        used_e.add(i)
        used_g.add(j)
        pairs.append((i, j, dist))
    return (pairs, [i for i in range(len(est)) if i not in used_e],
            [j for j in range(len(gt)) if j not in used_g])


# ── Fusión entre vistas (M2) ─────────────────────────────────────────────────
def fuse(estimates_map, radius):
    """Agrupa estimaciones (en `map`) de varias vistas y las fusiona.

    `estimates_map`: lista de dicts con 'position' (3), 'class', 'conf' y
    opcionalmente 'yaw'. No usa la verdad de referencia: agrupa por cercanía
    al centroide (voraz, de mayor a menor confianza). Por grupo promedia la
    posición, vota la clase y toma la media circular del yaw de la clase ganadora.
    """
    clusters = []
    for e in sorted(estimates_map, key=lambda e: -float(e.get('conf', 1.0))):
        p = np.asarray(e['position'], float)
        best, best_d = None, radius
        for c in clusters:
            d = float(np.linalg.norm(p - c['centroid']))
            if d <= best_d:
                best, best_d = c, d
        if best is None:
            clusters.append({'members': [e], 'centroid': p})
        else:
            best['members'].append(e)
            best['centroid'] = np.mean([m['position'] for m in best['members']], axis=0)
    fused = []
    for c in clusters:
        votes = Counter()
        for m in c['members']:
            votes[m['class']] += float(m.get('conf', 1.0))
        cls = votes.most_common(1)[0][0]
        yaws = [m['yaw'] for m in c['members']
                if m['class'] == cls and m.get('yaw') is not None]
        yaw = (math.atan2(sum(math.sin(y) for y in yaws), sum(math.cos(y) for y in yaws))
               if yaws else None)
        fused.append({'position': [float(v) for v in c['centroid']], 'class': cls,
                      'conf': float(np.mean([m.get('conf', 1.0) for m in c['members']])),
                      'yaw': yaw, 'n_views': len(c['members'])})
    return fused


# ── Estadística (protocolo de reporte) ───────────────────────────────────────
def wilson_interval(k, n, z=1.959964):
    """Intervalo de Wilson para una proporción k/n (95 % por defecto)."""
    if n <= 0:
        return float('nan'), float('nan')
    p = k / n
    den = 1.0 + z * z / n
    centre = (p + z * z / (2.0 * n)) / den
    half = z * math.sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n)) / den
    return max(0.0, centre - half), min(1.0, centre + half)


def bootstrap_ci(values, groups, stat=np.median, n_resamples=2000, alpha=0.05, seed=0):
    """IC por bootstrap remuestreando POR GRUPO (escena), no por imagen.

    Las imágenes de una misma escena están correlacionadas; remuestrear por
    imagen daría intervalos demasiado estrechos.
    """
    values = np.asarray(values, float)
    groups = np.asarray(groups)
    ok = np.isfinite(values)
    values, groups = values[ok], groups[ok]
    if len(values) == 0:
        return float('nan'), float('nan')
    by_group = [values[groups == gid] for gid in sorted(set(groups.tolist()))]
    rng = np.random.default_rng(seed)
    stats = np.empty(n_resamples)
    for b in range(n_resamples):
        idx = rng.integers(0, len(by_group), len(by_group))
        stats[b] = stat(np.concatenate([by_group[i] for i in idx]))
    return (float(np.quantile(stats, alpha / 2.0)),
            float(np.quantile(stats, 1.0 - alpha / 2.0)))


def describe(values, groups=None, n_resamples=2000, seed=0):
    """Media, desviación estándar, mediana y P95, con IC 95 % de la mediana."""
    v = np.asarray(values, float)
    v_ok = v[np.isfinite(v)]
    if len(v_ok) == 0:
        return {'n': 0}
    out = {'n': int(len(v_ok)), 'mean': float(v_ok.mean()),
           'std': float(v_ok.std(ddof=1)) if len(v_ok) > 1 else 0.0,
           'median': float(np.median(v_ok)), 'p95': float(np.quantile(v_ok, 0.95))}
    if groups is not None:
        lo, hi = bootstrap_ci(v, groups, n_resamples=n_resamples, seed=seed)
        out['median_ci95'] = [lo, hi]
        out['n_groups'] = int(len(set(np.asarray(groups)[np.isfinite(v)].tolist())))
    return out


def rate(k, n):
    lo, hi = wilson_interval(k, n)
    return {'k': int(k), 'n': int(n), 'rate': (k / n if n else float('nan')),
            'wilson_ci95': [lo, hi]}
