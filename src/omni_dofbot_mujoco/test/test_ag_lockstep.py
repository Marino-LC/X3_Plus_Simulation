"""pytest: AG de Gazebo (sin modificar) corriendo sobre MuJoCo en paso fijo.

  python3 -m pytest src/omni_dofbot_mujoco/test/test_ag_lockstep.py -v

Cada batería P1-P4 son ~25 s simulados (~5 s de pared).
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from omni_dofbot_mujoco.lockstep import LockstepWorld  # noqa: E402

FABRICANTE = (0.8, 0.06, 0.5)     # ganancias del firmware Yahboom


def _battery(gains):
    w = LockstepWorld()
    return w, w.evaluate(*gains)


def test_bateria_fabricante_sin_penalizacion():
    """Con las ganancias del fabricante las 4 pruebas llegan a la meta:
    ningún costo incluye PENALTY_TO (timeout, aborto o volcadura)."""
    w, r = _battery(FABRICANTE)
    for k in ("P1", "P2", "P3", "P4"):
        assert math.isfinite(r[k]) and r[k] < w.ag.PENALTY_TO, (k, r)


def test_determinismo():
    """Dos mundos nuevos con las mismas ganancias dan el mismo fitness."""
    _, a = _battery(FABRICANTE)
    _, b = _battery(FABRICANTE)
    assert a["fitness"] == b["fitness"] and a["sim_s"] == b["sim_s"]


def test_teleport_y_odometria_coinciden_con_el_ag():
    """Tras una batería el robot quedó teletransportado y la odometría
    reiniciada con la convención del AG: yaw = pi/2.

    La posición no queda exactamente en (0, 0): el AG reinicia el PID y el
    filtro del motor 0.8 s DESPUÉS del teleport, y mientras tanto el filtro
    (tau = 0.58 s) sigue moviendo las ruedas. Es la lógica del AG (igual en
    Gazebo); el AG fija su origen después, así que no afecta al costo."""
    w, _ = _battery(FABRICANTE)
    w._evaluator._teleport()
    x, y, yaw = w.sim.pose2d()
    assert math.hypot(x, y) < 0.10 and abs(yaw - math.pi / 2) < 0.05
    assert abs(w.odom._yaw - math.pi / 2) < 0.02
