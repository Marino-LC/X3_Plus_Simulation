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
