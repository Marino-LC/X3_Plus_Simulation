"""Reporte agregado de varios ensayos (guía, sección 8: métricas, cortes y protocolo)."""

import csv
import json
import math
import os

import numpy as np

from . import dataset, metrics

NUMERIC = ('e_pos', 'e_radial', 'e_transversal', 'e_z', 'e_yaw_deg', 'distance',
           'view_angle_deg', 'pedestal_height', 'infer_ms', 'conf')


def read_rows(paths):
    rows = []
    for p in paths:
        with open(p, newline='', encoding='utf-8') as f:
            for r in csv.DictReader(f):
                for k in NUMERIC:
                    r[k] = float(r[k]) if r.get(k) not in (None, '') else float('nan')
                for k in ('view', 'fused', 'in_view', 'distractor'):
                    r[k] = int(r[k]) if r.get(k) not in (None, '') else 0
                r['class_ok'] = int(r['class_ok']) if r.get('class_ok') not in (None, '') else None
                rows.append(r)
    return rows


def _pos_stats(tp, n_boot):
    return metrics.describe([r['e_pos'] for r in tp], [r['scene_id'] for r in tp],
                            n_resamples=n_boot)


def _rates(rows):
    tp = [r for r in rows if r['status'] == 'TP']
    fn = [r for r in rows if r['status'] == 'FN']
    fp = [r for r in rows if r['status'] == 'FP']
    return {'recall': metrics.rate(len(tp), len(tp) + len(fn)),
            'false_positive_rate': metrics.rate(len(fp), len(tp) + len(fp)),
            'class_accuracy': metrics.rate(sum(1 for r in tp if r['class_ok']), len(tp))}


def _cut(tp, key_fn, n_boot):
    groups = {}
    for r in tp:
        k = key_fn(r)
        if k is not None:
            groups.setdefault(k, []).append(r)
    def order(k):
        try:
            return (0, float(str(k).strip('[').split(',')[0]), '')
        except ValueError:
            return (1, 0.0, str(k))
    return {str(k): _pos_stats(groups[k], n_boot) for k in sorted(groups, key=order)}


def _bin(value, width):
    if not math.isfinite(value):
        return None
    lo = math.floor(value / width) * width
    return '[%.1f, %.1f)' % (lo, lo + width)


def summarize(rows, fused_rows, cfg):
    """Resumen de un estimador sobre un conjunto de ensayos."""
    ev = cfg['eval']['evaluation']
    n_boot = int(ev['bootstrap_resamples'])
    tp = [r for r in rows if r['status'] == 'TP']
    scenes = [r['scene_id'] for r in tp]
    s = {'n_scenes': len({r['scene_id'] for r in rows}), 'per_view': {}, 'fused': {}}
    pv = s['per_view']
    pv['e_pos_m'] = _pos_stats(tp, n_boot)
    for k in ('e_radial', 'e_transversal', 'e_z'):
        pv[k + '_m'] = metrics.describe([r[k] for r in tp])
        pv['abs_' + k + '_m'] = metrics.describe([abs(r[k]) for r in tp])
    pv['e_yaw_deg'] = metrics.describe([r['e_yaw_deg'] for r in tp], scenes, n_resamples=n_boot)
    pv.update(_rates(rows))
    infer = {}
    for r in rows:                       # un tiempo por (escena, vista)
        if math.isfinite(r['infer_ms']):
            infer[(r['scene_id'], r['view'])] = r['infer_ms']
    if infer:
        v = np.array(list(infer.values()))
        pv['infer_ms'] = {'n': len(v), 'p50': float(np.quantile(v, 0.5)),
                          'p95': float(np.quantile(v, 0.95))}
    pv['cuts'] = {
        'distance_m': _cut(tp, lambda r: _bin(r['distance'], float(ev['distance_bin_m'])), n_boot),
        'view_angle_deg': _cut(tp, lambda r: _bin(r['view_angle_deg'],
                                                  float(ev['view_angle_bin_deg'])), n_boot),
        'pedestal_height_m': _cut(tp, lambda r: '%.2f' % r['pedestal_height'], n_boot),
        'scene_type': _cut(tp, lambda r: r['scene_type'], n_boot),
        'class': _cut(tp, lambda r: r['class_gt'], n_boot),
    }
    ftp = [r for r in fused_rows if r['status'] == 'TP']
    targets = [r for r in fused_rows if r['status'] in ('TP', 'FN')]
    fu = s['fused']
    fu['e_pos_m'] = _pos_stats(ftp, n_boot)
    fu.update(_rates(fused_rows))
    fu['coverage'] = metrics.rate(sum(r['in_view'] for r in targets), len(targets))
    fu['cuts'] = {'scene_type': _cut(ftp, lambda r: r['scene_type'], n_boot)}

    t = cfg['eval']['targets']

    def check(value, limit, less=True):
        if value is None or not math.isfinite(value):
            return None
        return bool(value <= limit) if less else bool(value >= limit)

    s['targets_met'] = {
        'pos_median': check(pv['e_pos_m'].get('median'), t['pos_median_m']),
        'pos_p95': check(pv['e_pos_m'].get('p95'), t['pos_p95_m']),
        'yaw_median': check(pv['e_yaw_deg'].get('median'), t['yaw_median_deg']),
        'class_accuracy': check(pv['class_accuracy']['rate'], t['class_accuracy'], False),
        'recall': check(pv['recall']['rate'], t['recall'], False),
        'false_positive_rate': check(pv['false_positive_rate']['rate'],
                                     t['false_positive_rate']),
        'coverage': check(fu['coverage']['rate'], t['coverage'], False),
    }
    return s


def _f(x, nd=3):
    return '—' if x is None or (isinstance(x, float) and not math.isfinite(x)) else '%.*f' % (nd, x)


def _stat_line(name, d, unit, nd=3):
    if not d.get('n'):
        return '| %s | 0 | — | — | — | — | — |' % name
    ci = d.get('median_ci95')
    ci_s = '[%s, %s]' % (_f(ci[0], nd), _f(ci[1], nd)) if ci else '—'
    return '| %s (%s) | %d | %s | %s | %s | %s | %s |' % (
        name, unit, d['n'], _f(d['mean'], nd), _f(d['std'], nd), _f(d['median'], nd), ci_s,
        _f(d['p95'], nd))


def _rate_line(name, d):
    return '| %s | %d/%d | %s | [%s, %s] |' % (
        name, d['k'], d['n'], _f(100 * d['rate'], 1), _f(100 * d['wilson_ci95'][0], 1),
        _f(100 * d['wilson_ci95'][1], 1))


def to_markdown(summaries):
    """Reporte legible; `summaries` = {estimador: resumen}."""
    out = ['# Error de estimación de pose de objetos', '',
           'IC 95 % de la mediana por bootstrap remuestreando **por escena**; '
           'tasas con intervalo de Wilson al 95 %. Errores en `base_footprint`.', '']
    hdr = ['| Métrica | n | media | desv. | mediana | IC 95 % mediana | P95 |',
           '|---|---|---|---|---|---|---|']
    for name, s in summaries.items():
        pv, fu = s['per_view'], s['fused']
        out += ['## Estimador: `%s` (%d escenas)' % (name, s['n_scenes']), '',
                '### Por vista', ''] + hdr
        out.append(_stat_line('Error de posición', pv['e_pos_m'], 'm'))
        out.append(_stat_line('Radial (con signo)', pv['e_radial_m'], 'm'))
        out.append(_stat_line('Transversal (con signo)', pv['e_transversal_m'], 'm'))
        out.append(_stat_line('Vertical z (con signo)', pv['e_z_m'], 'm'))
        out.append(_stat_line('Error de orientación', pv['e_yaw_deg'], '°', 1))
        out += ['', '| Tasa | k/n | % | Wilson 95 % |', '|---|---|---|---|',
                _rate_line('Recall (objetos en cuadro)', pv['recall']),
                _rate_line('Falsos positivos', pv['false_positive_rate']),
                _rate_line('Exactitud de clase', pv['class_accuracy'])]
        if 'infer_ms' in pv:
            out += ['', 'Tiempo de inferencia por imagen: p50 = %s ms, p95 = %s ms (n = %d).'
                    % (_f(pv['infer_ms']['p50'], 1), _f(pv['infer_ms']['p95'], 1),
                       pv['infer_ms']['n'])]
        out += ['', '### Pasada completa (fusión entre vistas)', ''] + hdr
        out.append(_stat_line('Error de posición', fu['e_pos_m'], 'm'))
        out += ['', '| Tasa | k/n | % | Wilson 95 % |', '|---|---|---|---|',
                _rate_line('Cobertura (visto en ≥ 1 vista)', fu['coverage']),
                _rate_line('Recall de la pasada', fu['recall']),
                _rate_line('Falsos positivos', fu['false_positive_rate']),
                _rate_line('Exactitud de clase', fu['class_accuracy'])]
        out += ['', '### Cortes del error de posición (por vista)', '']
        for cut, groups in pv['cuts'].items():
            out += ['**%s**' % cut, ''] + hdr
            out += [_stat_line(k, d, 'm') for k, d in groups.items()] + ['']
        out += ['### Metas propuestas', '']
        out += ['- %s: %s' % (k, {True: 'cumple', False: 'NO cumple', None: 'sin datos'}[v])
                for k, v in s['targets_met'].items()]
        out.append('')
    return '\n'.join(out)


def build_report(root, estimators, cfg, out_dir=None, scene_types=None):
    """Lee los CSV de todos los ensayos bajo `root` y escribe summary.{md,json}."""
    trials = dataset.find_trials(root)
    summaries = {}
    for est in estimators:
        per_view = [dataset.errors_path(t, est) for t in trials]
        fused = [dataset.errors_path(t, est, fused=True) for t in trials]
        rows = read_rows([p for p in per_view if os.path.isfile(p)])
        frows = read_rows([p for p in fused if os.path.isfile(p)])
        if scene_types:
            rows = [r for r in rows if r['scene_type'] in scene_types]
            frows = [r for r in frows if r['scene_type'] in scene_types]
        if rows:
            summaries[est] = summarize(rows, frows, cfg)
    out_dir = out_dir or root
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump(summaries, f, indent=2, ensure_ascii=False)
    md = to_markdown(summaries)
    with open(os.path.join(out_dir, 'summary.md'), 'w', encoding='utf-8') as f:
        f.write(md)
    return summaries, md
