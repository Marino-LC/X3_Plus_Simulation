"""Generador de escenas con semilla y verdad de referencia (guía, sección 3).

Dado un tipo de escena (S1, S2, S3) y una semilla produce:

* un mundo SDF para Gazebo Harmonic (arena, pedestales, objetos), y
* la verdad de referencia: clase y pose (x, y, z, yaw) de cada objeto en `map`.

Los objetos se generan estáticos por defecto, así que la verdad escrita aquí es
exacta (no hay asentamiento físico). `map` coincide con el marco del mundo.
"""

import math
import random

from . import views as vw

WORLD_HEADER = """<?xml version="1.0" ?>
<!-- Generado por vla_pose_eval.scene: {scene_id}. NO editar a mano. -->
<sdf version="1.9">
  <world name="{world_name}">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <gravity>0 0 -9.81</gravity>
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>{amb} {amb} {amb} 1</ambient>
      <background>0.55 0.65 0.75 1</background>
      <shadows>true</shadows>
    </scene>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>{sun} {sun} {sun} 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>{sdx} {sdy} -0.9</direction>
    </light>
"""

WORLD_FOOTER = """  </world>
</sdf>
"""


def _fmt(v):
    return ('%.6f' % v).rstrip('0').rstrip('.') if isinstance(v, float) else str(v)


def _rgb(c):
    return ' '.join(_fmt(float(x)) for x in c) + ' 1'


def _geometry(shape, size):
    if shape == 'box':
        return '<box><size>%s %s %s</size></box>' % tuple(_fmt(float(s)) for s in size)
    if shape == 'cylinder':
        return '<cylinder><radius>%s</radius><length>%s</length></cylinder>' % (
            _fmt(float(size[0])), _fmt(float(size[1])))
    if shape == 'sphere':
        return '<sphere><radius>%s</radius></sphere>' % _fmt(float(size[0]))
    raise ValueError('forma no soportada: %r' % shape)


def shape_height(shape, size):
    return {'box': lambda: size[2], 'cylinder': lambda: size[1],
            'sphere': lambda: 2.0 * size[0]}[shape]()


def _model(name, shape, size, pose, color, static=True, mass=0.05):
    geo = _geometry(shape, size)
    x, y, z, yaw = pose
    inertial = ''
    if not static:
        i = mass * 0.0004  # orden de magnitud para objetos de ~5 cm
        inertial = ('<inertial><mass>%s</mass><inertia><ixx>%g</ixx><iyy>%g</iyy>'
                    '<izz>%g</izz></inertia></inertial>' % (_fmt(mass), i, i, i))
    return """    <model name="{name}">
      <static>{static}</static>
      <pose>{x} {y} {z} 0 0 {yaw}</pose>
      <link name="link">
        {inertial}
        <collision name="collision"><geometry>{geo}</geometry></collision>
        <visual name="visual">
          <geometry>{geo}</geometry>
          <material><ambient>{rgb}</ambient><diffuse>{rgb}</diffuse><specular>0.1 0.1 0.1 1</specular></material>
        </visual>
      </link>
    </model>
""".format(name=name, static='true' if static else 'false', x=_fmt(x), y=_fmt(y),
           z=_fmt(z), yaw=_fmt(yaw), geo=geo, rgb=_rgb(color), inertial=inertial)


def _variants(catalog, split):
    """[(clase, spec, variante)] permitidas en la partición."""
    out = []
    for cls, spec in (catalog or {}).items():
        for var in spec.get('variants', [{'name': 'default', 'color': [0.5, 0.5, 0.5]}]):
            if split is None or var.get('split', 'both') in ('both', split):
                out.append((cls, spec, var))
    return out


def split_of_seed(seed, scenes_cfg):
    """Partición a la que pertenece una semilla según scenes.yaml (o None)."""
    for name, s in (scenes_cfg.get('splits') or {}).items():
        lo, hi = s['seeds']
        if lo <= seed < hi:
            return name
    return None


def generate_scene(scene_type, seed, cfg, split='auto'):
    """Genera la verdad de referencia de una escena. Devuelve un dict serializable.

    `cfg` es el dict de `config.load_config()`. `split='auto'` deduce la
    partición de la semilla; None desactiva el filtro de variantes por objeto.
    """
    scenes, objs, ev = cfg['scenes'], cfg['objects'], cfg['eval']
    if scene_type not in scenes['scene_types']:
        raise ValueError('tipo de escena desconocido: %r' % scene_type)
    st = scenes['scene_types'][scene_type]
    if st['targets'] + st['distractors'] > st['pedestals']:
        raise ValueError('%s: más objetos que pedestales' % scene_type)
    if split == 'auto':
        split = split_of_seed(seed, scenes)

    # Semilla derivada del tipo de escena para que S1/S2/S3 no compartan sorteo.
    rng = random.Random('%s:%d' % (scene_type, seed))
    pl = scenes['placement']
    region = scenes['arena']['spawn_region']
    ped_cfg = objs['pedestal']
    view_list = vw.load_views(cfg['views'])
    cam = vw.CameraModel.from_config(ev['camera'])
    targets = _variants(objs.get('objects'), split)
    distractors = _variants(objs.get('distractors'), split)
    if not targets or (st['distractors'] and not distractors):
        raise ValueError('no hay variantes de objeto para la partición %r' % split)

    def sample_pedestals():
        peds = []
        for _ in range(st['pedestals']):
            for _ in range(pl['max_attempts']):
                x = rng.uniform(*region['x'])
                y = rng.uniform(*region['y'])
                if any(math.hypot(x - p['x'], y - p['y']) < pl['min_pedestal_separation_m']
                       for p in peds):
                    continue
                if any(math.hypot(x - v['x'], y - v['y']) < pl['robot_keepout_m']
                       for v in view_list):
                    continue
                h = float(rng.choice(ped_cfg['heights']))
                yaw = rng.uniform(-math.pi, math.pi) if ped_cfg.get('randomize_yaw') else 0.0
                peds.append({'x': x, 'y': y, 'height': h, 'yaw': yaw})
                break
            else:
                return None
        return peds

    for _ in range(pl['max_attempts']):
        peds = sample_pedestals()
        if peds is None:
            continue
        roles = (['target'] * st['targets'] + ['distractor'] * st['distractors']
                 + ['empty'] * (st['pedestals'] - st['targets'] - st['distractors']))
        rng.shuffle(roles)
        objects = []
        for pid, (ped, role) in enumerate(zip(peds, roles)):
            if role == 'empty':
                continue
            cls, spec, var = rng.choice(targets if role == 'target' else distractors)
            off = pl.get('max_object_offset_m', 0.0)
            oid = len(objects)
            objects.append({
                'id': oid,
                'name': 'obj_%02d_%s' % (oid, cls),
                'class': cls,
                'variant': var['name'],
                'distractor': role == 'distractor',
                'x': ped['x'] + rng.uniform(-off, off),
                'y': ped['y'] + rng.uniform(-off, off),
                'z': ped['height'] + shape_height(spec['shape'], spec['size']) / 2.0,
                'yaw': rng.uniform(-math.pi, math.pi),
                'symmetry_deg': spec.get('symmetry_deg', 360),
                'shape': spec['shape'],
                'size': [float(s) for s in spec['size']],
                'color': [float(c) for c in var['color']],
                'pedestal_id': pid,
                'pedestal_height': ped['height'],
            })
        cov = vw.coverage([[o['x'], o['y'], o['z']] for o in objects], view_list, cam)
        if pl.get('require_visible', True) and any(not c for c in cov):
            continue
        break
    else:
        raise RuntimeError('no se pudo colocar la escena %s semilla %d: relajar '
                           'placement o revisar las vistas' % (scene_type, seed))

    for o, c in zip(objects, cov):
        o['visible_from_views'] = c
    pedestals = [{
        'id': i, 'name': 'pedestal_%02d' % i,
        'x': p['x'], 'y': p['y'], 'z': p['height'] / 2.0, 'yaw': p['yaw'],
        'height': p['height'],
        'size': [float(ped_cfg['size_xy'][0]), float(ped_cfg['size_xy'][1]), p['height']],
        'empty': role == 'empty',
    } for i, (p, role) in enumerate(zip(peds, roles))]

    ap = scenes.get('appearance', {})
    if ap.get('randomize'):
        ang = rng.uniform(-math.pi, math.pi)
        appearance = {
            'sun_intensity': rng.uniform(*ap['sun_intensity']),
            'ambient': rng.uniform(*ap['ambient']),
            'sun_direction_xy': [0.5 * math.cos(ang), 0.5 * math.sin(ang)],
            'floor_color': list(rng.choice(ap['floor_colors'])),
            'wall_color': list(rng.choice(ap['wall_colors'])),
        }
    else:
        appearance = {'sun_intensity': 0.9, 'ambient': 0.5, 'sun_direction_xy': [-0.5, 0.2],
                      'floor_color': [0.92, 0.87, 0.78], 'wall_color': [0.7, 0.7, 0.7]}

    return {
        'scene_id': '%s_seed%04d' % (scene_type, seed),
        'scene_type': scene_type,
        'seed': seed,
        'split': split,
        'world_name': ev['world_name'],
        'frame': ev['frames']['map'],
        'reference_point': objs.get('reference_point', 'geometric_center'),
        'static_objects': bool(pl.get('static_objects', True)),
        'n_objects': len(objects),                      # incluye distractores
        'n_targets': sum(1 for o in objects if not o['distractor']),
        'arena': {k: scenes['arena'][k] for k in ('size', 'wall_height', 'wall_thickness')},
        'appearance': appearance,
        'pedestals': pedestals,
        'objects': objects,
    }


def scene_to_sdf(gt, pedestal_color=(0.3, 0.3, 0.8)):
    """Mundo SDF (texto) correspondiente a una verdad de referencia."""
    ap, arena = gt['appearance'], gt['arena']
    sx, sy = arena['size']
    wh, wt = arena['wall_height'], arena['wall_thickness']
    parts = [WORLD_HEADER.format(
        scene_id=gt['scene_id'], world_name=gt['world_name'],
        amb=_fmt(float(ap['ambient'])), sun=_fmt(float(ap['sun_intensity'])),
        sdx=_fmt(float(ap['sun_direction_xy'][0])), sdy=_fmt(float(ap['sun_direction_xy'][1])))]
    parts.append(_model('arena_floor', 'box', [sx, sy, 0.02], (0.0, 0.0, -0.01, 0.0),
                        ap['floor_color']))
    walls = {'wall_north': ([sx, wt, wh], (0.0, sy / 2.0)),
             'wall_south': ([sx, wt, wh], (0.0, -sy / 2.0)),
             'wall_east': ([wt, sy, wh], (sx / 2.0, 0.0)),
             'wall_west': ([wt, sy, wh], (-sx / 2.0, 0.0))}
    for name, (size, (x, y)) in walls.items():
        parts.append(_model(name, 'box', size, (x, y, wh / 2.0, 0.0), ap['wall_color']))
    for p in gt['pedestals']:
        parts.append(_model(p['name'], 'box', p['size'], (p['x'], p['y'], p['z'], p['yaw']),
                            pedestal_color))
    for o in gt['objects']:
        parts.append(_model(o['name'], o['shape'], o['size'],
                            (o['x'], o['y'], o['z'], o['yaw']), o['color'],
                            static=gt.get('static_objects', True)))
    parts.append(WORLD_FOOTER)
    return ''.join(parts)
