"""Línea de órdenes sin ROS:  python3 -m vla_pose_eval.cli <orden> …

    generate-scenes   genera mundos SDF + verdad de referencia (por semilla)
    check-views       comprueba la cobertura geométrica de la lista de vistas
    run-estimator     ejecuta un estimador sobre las vistas guardadas de un ensayo
    evaluate          estimaciones + verdad -> CSV de errores
    report            agrega los CSV de varios ensayos (estadística de la sección 8)
    dry-run           flujo completo sin Gazebo con el estimador oráculo
"""

import argparse
import json
import os
import sys

from . import dataset, evaluate as ev, report as rp, scene, views as vw
from .config import dump_yaml, load_config
from .estimators import load_estimator


def _seeds(args, cfg):
    if args.seeds:
        return list(range(args.seeds[0], args.seeds[1]))
    if args.split:
        lo, hi = cfg['scenes']['splits'][args.split]['seeds']
        return list(range(lo, hi))
    return [0]


def _generate(args, cfg):
    trials = []
    for st in args.types:
        for seed in _seeds(args, cfg):
            gt = scene.generate_scene(st, seed, cfg)
            trial = os.path.join(args.out, gt['scene_id'])
            os.makedirs(trial, exist_ok=True)
            dump_yaml(gt, dataset.gt_path(trial))
            with open(os.path.join(trial, dataset.SCENE_WORLD), 'w', encoding='utf-8') as f:
                f.write(scene.scene_to_sdf(gt, cfg['objects']['pedestal'].get('color',
                                                                              (0.3, 0.3, 0.8))))
            trials.append(trial)
    return trials


def cmd_generate(args, cfg):
    for t in _generate(args, cfg):
        print(t)


def cmd_check_views(args, cfg):
    view_list = vw.load_views(cfg['views'])
    cam = vw.CameraModel.from_config(cfg['eval']['camera'])
    print('%d vistas' % len(view_list))
    bad = 0
    for trial in dataset.find_trials_or_scenes(args.path):
        gt = dataset.load_gt(trial)
        cov = vw.coverage([[o['x'], o['y'], o['z']] for o in gt['objects']], view_list, cam)
        missing = [o['name'] for o, c in zip(gt['objects'], cov) if not c]
        bad += bool(missing)
        print('%s: %d/%d objetos en cuadro%s' % (
            gt['scene_id'], len(cov) - len(missing), len(cov),
            (' — SIN VISTA: ' + ', '.join(missing)) if missing else ''))
    return 1 if bad else 0


def _estimator(args):
    kwargs = json.loads(args.kwargs) if args.kwargs else {}
    return load_estimator(args.estimator, **kwargs)


def cmd_run_estimator(args, cfg):
    est = _estimator(args)
    for trial in dataset.find_trials(args.path):
        ev.run_estimator(trial, est, cfg, instruction=args.instruction)
        print('%s: %s' % (trial, dataset.estimates_path(trial, est.name)))


def cmd_evaluate(args, cfg):
    for trial in dataset.find_trials(args.path):
        for est in args.estimators:
            if not os.path.isfile(dataset.estimates_path(trial, est)):
                print('%s: sin estimaciones de %s' % (trial, est), file=sys.stderr)
                continue
            rows, _ = ev.evaluate_trial_dir(trial, est, cfg)
            tp = [r['e_pos'] for r in rows if r['status'] == 'TP']
            print('%s [%s]: %d TP, %d FN, %d FP%s -> %s' % (
                os.path.basename(trial), est, len(tp),
                sum(r['status'] == 'FN' for r in rows), sum(r['status'] == 'FP' for r in rows),
                (', mediana e_pos = %.3f m' % sorted(tp)[len(tp) // 2]) if tp else '',
                dataset.errors_path(trial, est)))


def cmd_report(args, cfg):
    _, md = rp.build_report(args.path, args.estimators, cfg, args.out, args.types)
    print(md)


def cmd_dry_run(args, cfg):
    """Escenas + vistas sintéticas + oráculo con ruido -> CSV y reporte."""
    kwargs = json.loads(args.kwargs) if args.kwargs else {
        'sigma_radial': 0.02, 'sigma_transversal': 0.008, 'sigma_z': 0.008,
        'sigma_yaw_deg': 5.0, 'drop_prob': 0.05, 'false_positives_per_view': 0.02,
        'class_error_prob': 0.05}
    est = load_estimator('oracle', **kwargs)
    for trial in _generate(args, cfg):
        dataset.save_views_meta(trial, ev.synthetic_views_meta(dataset.load_gt(trial), cfg))
        ev.run_estimator(trial, est, cfg)
        ev.evaluate_trial_dir(trial, est.name, cfg)
    _, md = rp.build_report(args.out, [est.name], cfg)
    print(md)
    print('\n[dry-run] SIN Gazebo: valida el flujo y las métricas, no un modelo.')


def main(argv=None):
    p = argparse.ArgumentParser(prog='vla_pose_eval', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--config-dir', help='directorio con eval/objects/scenes/views.yaml')
    sub = p.add_subparsers(dest='cmd', required=True)

    def scene_args(sp):
        sp.add_argument('--out', required=True, help='directorio raíz de los ensayos')
        sp.add_argument('--types', nargs='+', default=['S1', 'S2', 'S3'])
        sp.add_argument('--seeds', nargs=2, type=int, metavar=('INICIO', 'FIN'),
                        help='rango de semillas [INICIO, FIN)')
        sp.add_argument('--split', choices=['train', 'test'],
                        help='usa el rango de semillas de scenes.yaml')

    sp = sub.add_parser('generate-scenes')
    scene_args(sp)
    sp.set_defaults(fn=cmd_generate)
    sp = sub.add_parser('check-views')
    sp.add_argument('path')
    sp.set_defaults(fn=cmd_check_views)
    sp = sub.add_parser('run-estimator')
    sp.add_argument('path', help='un ensayo o el directorio raíz')
    sp.add_argument('--estimator', required=True, help="'oracle', 'vla' o 'modulo:Clase'")
    sp.add_argument('--kwargs', help='JSON con los argumentos del estimador')
    sp.add_argument('--instruction', default='')
    sp.set_defaults(fn=cmd_run_estimator)
    sp = sub.add_parser('evaluate')
    sp.add_argument('path')
    sp.add_argument('--estimators', nargs='+', required=True)
    sp.set_defaults(fn=cmd_evaluate)
    sp = sub.add_parser('report')
    sp.add_argument('path')
    sp.add_argument('--estimators', nargs='+', required=True)
    sp.add_argument('--types', nargs='+')
    sp.add_argument('--out')
    sp.set_defaults(fn=cmd_report)
    sp = sub.add_parser('dry-run')
    scene_args(sp)
    sp.add_argument('--kwargs', help='JSON con los argumentos del oráculo')
    sp.set_defaults(fn=cmd_dry_run)

    args = p.parse_args(argv)
    return args.fn(args, load_config(args.config_dir)) or 0


if __name__ == '__main__':
    sys.exit(main())
