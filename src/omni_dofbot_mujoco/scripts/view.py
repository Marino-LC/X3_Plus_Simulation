#!/usr/bin/env python3
"""Visor interactivo con un twist constante (sin ROS).

  python3 view.py                      # arena, robot quieto
  python3 view.py --vy 0.2             # desplazamiento lateral
  python3 view.py --world vacio --wz 1
Los rodillos de colisión están en el grupo 3 (tecla 3 en el visor).
"""
import argparse
import time

import _path  # noqa: F401
import mujoco.viewer
from omni_dofbot_mujoco.sim import MujocoSim

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", choices=["arena", "vacio"], default="arena")
    ap.add_argument("--vx", type=float, default=0.0)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--wz", type=float, default=0.0)
    a = ap.parse_args()
    sim = MujocoSim(world=a.world)
    with mujoco.viewer.launch_passive(sim.model, sim.data) as v:
        t0 = time.perf_counter()
        while v.is_running():
            if sim.time > 1.0:
                sim.set_twist(a.vx, a.vy, a.wz)
            while sim.time < time.perf_counter() - t0:
                sim.step()
            v.sync()
            time.sleep(0.005)
