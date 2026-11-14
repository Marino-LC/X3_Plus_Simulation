import math
import xml.etree.ElementTree as ET

import pytest

from vla_pose_eval import scene
from vla_pose_eval.views import load_views


def test_deterministic_by_seed(cfg):
    assert scene.generate_scene('S2', 7, cfg) == scene.generate_scene('S2', 7, cfg)
    assert scene.generate_scene('S2', 7, cfg) != scene.generate_scene('S2', 8, cfg)


@pytest.mark.parametrize('scene_type, pedestals, targets, distractors',
                         [('S1', 1, 1, 0), ('S2', 3, 3, 0), ('S3', 5, 3, 1)])
def test_scene_types(cfg, scene_type, pedestals, targets, distractors):
    for seed in range(15):
        gt = scene.generate_scene(scene_type, seed, cfg)
        objs = gt['objects']
        assert len(gt['pedestals']) == pedestals
        assert sum(not o['distractor'] for o in objs) == targets == gt['n_targets']
        assert sum(o['distractor'] for o in objs) == distractors
        assert sum(p['empty'] for p in gt['pedestals']) == pedestals - targets - distractors


def test_placement_constraints(cfg):
    pl = cfg['scenes']['placement']
    views = load_views(cfg['views'])
    for seed in range(15):
        gt = scene.generate_scene('S3', seed, cfg)
        peds = gt['pedestals']
        for i, a in enumerate(peds):
            assert a['size'][:2] == [0.15, 0.15]
            assert all(math.hypot(a['x'] - v['x'], a['y'] - v['y']) >= pl['robot_keepout_m']
                       for v in views)
            for b in peds[i + 1:]:
                assert math.hypot(a['x'] - b['x'], a['y'] - b['y']) >= \
                    pl['min_pedestal_separation_m']
        for o in gt['objects']:
            ped = peds[o['pedestal_id']]
            assert o['visible_from_views'], 'todo objeto debe quedar en cuadro en >= 1 vista'
            assert o['z'] > ped['height']               # centro geométrico sobre el pedestal
            assert abs(o['x'] - ped['x']) <= pl['max_object_offset_m'] + 1e-9


def test_split_by_seed_and_object(cfg):
    assert scene.split_of_seed(3, cfg['scenes']) == 'test'
    assert scene.split_of_seed(1005, cfg['scenes']) == 'train'
    assert scene.split_of_seed(500, cfg['scenes']) is None

    def variants(seeds):
        return {(o['class'], o['variant']) for s in seeds
                for o in scene.generate_scene('S3', s, cfg)['objects']}

    allowed = {'train': set(), 'test': set()}
    for group in ('objects', 'distractors'):
        for cls, spec in cfg['objects'][group].items():
            for v in spec['variants']:
                for sp in allowed:
                    if v['split'] in ('both', sp):
                        allowed[sp].add((cls, v['name']))
    assert variants(range(0, 20)) <= allowed['test']
    assert variants(range(1000, 1020)) <= allowed['train']


def test_sdf_world(cfg):
    gt = scene.generate_scene('S3', 1, cfg)
    root = ET.fromstring(scene.scene_to_sdf(gt))
    world = root.find('world')
    assert world.get('name') == cfg['eval']['world_name']
    plugins = {p.get('name') for p in world.findall('plugin')}
    assert 'gz::sim::systems::UserCommands' in plugins     # necesario para set_pose
    assert 'gz::sim::systems::Sensors' in plugins          # necesario para las cámaras
    models = {m.get('name'): m for m in world.findall('model')}
    for o in gt['objects']:
        pose = [float(v) for v in models[o['name']].find('pose').text.split()]
        assert pose[:3] == pytest.approx([o['x'], o['y'], o['z']], abs=1e-5)
        assert pose[5] == pytest.approx(o['yaw'], abs=1e-5)
    assert all(p['name'] in models for p in gt['pedestals'])
