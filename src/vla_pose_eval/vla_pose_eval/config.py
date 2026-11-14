"""Carga de la configuración (YAML) con rutas por defecto del paquete."""

import os

import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
NAMES = ('eval', 'objects', 'scenes', 'views')


def default_config_dir():
    """Directorio config/ del paquete, instalado o en el árbol de fuentes."""
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory('vla_pose_eval'), 'config')
        if os.path.isdir(d):
            return d
    except Exception:  # sin ROS (máquina de GPU) o paquete sin instalar
        pass
    return os.path.normpath(os.path.join(_HERE, '..', 'config'))


def load_yaml(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def dump_yaml(data, path):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True,
                       default_flow_style=None, width=120)


def load_config(config_dir=None):
    """Devuelve {'eval':…, 'objects':…, 'scenes':…, 'views':…}."""
    config_dir = config_dir or default_config_dir()
    return {n: load_yaml(os.path.join(config_dir, n + '.yaml')) for n in NAMES}
