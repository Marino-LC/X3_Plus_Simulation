"""Formato en disco de un ensayo (un directorio por escena).

    <ensayo>/
      scene.world              mundo SDF generado
      scene_gt.yaml            verdad de referencia de la escena
      views.yaml               por vista: pose real del robot, K, T_base_cam, archivos
      views/view_000_rgb.png   (o .npy)        imagen RGB  (H, W, 3) uint8
      views/view_000_depth.npy                 profundidad (H, W) float32, metros
      estimates_<estimador>.yaml               salida de un estimador
      errors_<estimador>.csv                   una fila por (vista, objeto) o falso positivo
      errors_fused_<estimador>.csv             lo mismo tras fusionar vistas

Todos los estimadores leen las MISMAS imágenes de `views/`, de modo que la
línea base y el modelo aprendido se evalúan sobre la misma entrada.
"""

import os

import numpy as np

from .config import dump_yaml, load_yaml

SCENE_GT = 'scene_gt.yaml'
SCENE_WORLD = 'scene.world'
VIEWS = 'views.yaml'


def gt_path(trial):
    return os.path.join(trial, SCENE_GT)


def views_path(trial):
    return os.path.join(trial, VIEWS)


def estimates_path(trial, estimator):
    return os.path.join(trial, 'estimates_%s.yaml' % estimator)


def errors_path(trial, estimator, fused=False):
    return os.path.join(trial, 'errors_%s%s.csv' % ('fused_' if fused else '', estimator))


def load_gt(trial):
    return load_yaml(gt_path(trial))


def load_views_meta(trial):
    return load_yaml(views_path(trial))


def save_views_meta(trial, meta):
    dump_yaml(meta, views_path(trial))


def is_trial(path):
    return os.path.isfile(gt_path(path)) and os.path.isfile(views_path(path))


def find_trials(root):
    """Directorios de ensayo bajo `root` (o `root` mismo si lo es)."""
    if is_trial(root):
        return [root]
    return sorted(os.path.join(root, d) for d in os.listdir(root)
                  if is_trial(os.path.join(root, d)))


def find_trials_or_scenes(root):
    """Como find_trials, pero basta con que exista scene_gt.yaml."""
    if os.path.isfile(gt_path(root)):
        return [root]
    return sorted(os.path.join(root, d) for d in os.listdir(root)
                  if os.path.isfile(gt_path(os.path.join(root, d))))


def save_rgb(path_no_ext, rgb):
    """Guarda RGB como PNG si hay OpenCV; si no, como .npy. Devuelve el nombre."""
    try:
        import cv2
        cv2.imwrite(path_no_ext + '.png', rgb[:, :, ::-1])
        return os.path.basename(path_no_ext + '.png')
    except ImportError:
        np.save(path_no_ext + '.npy', rgb)
        return os.path.basename(path_no_ext + '.npy')


def load_rgb(path):
    if path.endswith('.npy'):
        return np.load(path)
    try:
        import cv2
        return cv2.imread(path, cv2.IMREAD_COLOR)[:, :, ::-1].copy()
    except ImportError:
        from PIL import Image
        return np.asarray(Image.open(path).convert('RGB'))


def load_frame_arrays(trial, view_meta):
    """(rgb, depth) de una vista; None si el ensayo no tiene imágenes (dry-run)."""
    rgb = depth = None
    if view_meta.get('rgb'):
        rgb = load_rgb(os.path.join(trial, 'views', view_meta['rgb']))
    if view_meta.get('depth'):
        depth = np.load(os.path.join(trial, 'views', view_meta['depth']))
    return rgb, depth
