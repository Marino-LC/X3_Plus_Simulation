"""pytest: pruebas de aceptación automatizables (T1-T5, T7-T9).

  python3 -m pytest src/omni_dofbot_mujoco/test -v
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import run_acceptance_tests as acc  # noqa: E402
from omni_dofbot_mujoco.model_builder import build_model  # noqa: E402
from omni_dofbot_mujoco.params import ModelParams  # noqa: E402
from omni_dofbot_mujoco.sim import MujocoSim  # noqa: E402

_cache = {}


def mk(world):
    if world not in _cache:
        _cache[world] = build_model(ModelParams(), world)[0]
    return MujocoSim(model=_cache[world])


@pytest.mark.parametrize("key", list(acc.TESTS))
def test_acceptance(key):
    ok, detail = acc.TESTS[key][1](mk)
    assert ok, f"{key} {acc.TESTS[key][0]}: {detail}"


def test_total_mass():
    import mujoco
    m, _ = build_model(ModelParams(), "arena")
    assert abs(mujoco.mj_getTotalmass(m) - 4.8) < 1e-3


def test_teleport_matches_ag_reset_pose():
    sim = mk("arena")
    sim.set_twist(0.2, 0, 0)
    sim.run(0.5)
    sim.teleport(0.0, 0.0, 0.05, (0.7071068, 0, 0, 0.7071068))
    x, y, yaw = sim.pose2d()
    assert abs(x) < 1e-9 and abs(y) < 1e-9 and abs(yaw - 1.5708) < 1e-4
    assert abs(sim.data.qvel).max() == 0.0


def _cad_rollers():
    """Centro y eje de cada rodillo, en el marco de la rueda, del ensamble CAD."""
    import xml.etree.ElementTree as ET
    import numpy as np
    import mujoco
    path = os.path.join(os.path.dirname(__file__), "..", "models", "reference",
                        "omni_mujoco2_cad.xml")
    if not os.path.isfile(path):
        pytest.skip("no está models/reference/omni_mujoco2_cad.xml")
    out = {}
    for w in ET.parse(path).getroot().iter("body"):
        if not w.get("name", "").endswith("_wheel"):
            continue
        L = []
        for b in w.findall("body"):
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, np.array(b.get("quat").split(), float))
            R = R.reshape(3, 3)
            L.append((np.array(b.get("pos").split(), float) + R @ [0, 0, 0.02119], R[:, 2]))
        out[w.get("name")] = L
    return out


def test_rodillos_coinciden_con_cad():
    """Regresión del reporte 5: los rodillos de FR y BL estaban 17.8° fuera de
    las ranuras del cubo. Cada rodillo debe quedar a < 1 mm y < 1° del CAD."""
    import math
    import mujoco
    import numpy as np
    m, _ = build_model(ModelParams(), "arena")
    for wheel, cad in _cad_rollers().items():
        wid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, wheel)
        gen = [(m.body_pos[b], m.jnt_axis[m.body_jntadr[b]]) for b in range(m.nbody)
               if m.body_parentid[b] == wid]
        for c, a in cad:
            k = int(np.argmin([np.linalg.norm(c - g) for g, _ in gen]))
            assert np.linalg.norm(c - gen[k][0]) < 1e-3, wheel
            assert math.degrees(math.acos(min(1, abs(a @ gen[k][1])))) < 1.0, wheel


def test_chasis_visible():
    """Regresión: la malla visual del chasis debe existir en el modelo."""
    import mujoco
    m, _ = build_model(ModelParams(), "arena")
    mid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_MESH, "base_link_omni_dofbot")
    assert any(m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and m.geom_dataid[g] == mid
               for g in range(m.ngeom))
