#!/usr/bin/env python3
"""Plano inclinado virtual (guía, sección 9, método de Williams).

Con las ruedas bloqueadas (amortiguamiento muy alto en su junta) se inclina la gravedad poco a
poco y se reporta el ángulo crítico en que el robot empieza a deslizar:
  * pendiente a lo largo de x del robot (longitudinal)
  * pendiente a lo largo de y del robot (transversal)
tan(ángulo) es el coeficiente de tracción efectivo, comparable con la misma
prueba hecha con el robot real sobre el material de la arena.
"""
import argparse
import math

import _path  # noqa: F401
import mujoco
from build_model import add_param_args, params_from_args
from omni_dofbot_mujoco.params import WHEEL_JOINTS
from omni_dofbot_mujoco.sim import MujocoSim


def critical_angle(params, direction, rate_deg_s=5.0, slip=0.005):
    sim = MujocoSim(params=params, world="vacio")
    for j in WHEEL_JOINTS:       # freno: el servo de velocidad dejaría rodar
        jid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, j)
        sim.model.dof_damping[sim.model.jnt_dofadr[jid]] = 1e3
    sim.reset((0, 0, 0.05, 0))
    sim.run(1.0)
    x0, y0, _ = sim.pose2d()
    while True:
        ang = math.radians(rate_deg_s) * (sim.time - 1.0)
        g = 9.81 * math.sin(ang)
        sim.model.opt.gravity[:] = ((g, 0, 0) if direction == "x" else (0, g, 0))
        sim.model.opt.gravity[2] = -9.81 * math.cos(ang)
        sim.run(0.01)
        x, y, _ = sim.pose2d()
        if math.hypot(x - x0, y - y0) > slip or ang > math.radians(80):
            return math.degrees(ang)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_param_args(ap)
    p = params_from_args(ap.parse_args())
    for d, name in (("x", "longitudinal"), ("y", "transversal")):
        a = critical_angle(p, d)
        print(f"Pendiente en {d} ({name}): ángulo crítico = {a:.1f}°, "
              f"mu efectivo = tan = {math.tan(math.radians(a)):.3f}")
