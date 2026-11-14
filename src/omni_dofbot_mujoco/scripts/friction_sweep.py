#!/usr/bin/env python3
"""Barrido mu x damping del rodillo (tabla de la guía, sección 3).

Comando de 0.2 m/s en y durante 2 s; ideal 0.40 m.
  python3 friction_sweep.py
  python3 friction_sweep.py --mus 0.3 0.68 1.0 --dampings 1e-5 1e-4 1e-3 1e-2
"""
import argparse
import dataclasses

import _path  # noqa: F401
from build_model import add_param_args, params_from_args
from omni_dofbot_mujoco.sim import MujocoSim
from run_acceptance_tests import _drive, V

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mus", nargs="+", type=float, default=[0.3, 0.68, 1.0])
    ap.add_argument("--dampings", nargs="+", type=float,
                    default=[1e-5, 1e-4, 1e-3, 1e-2])
    add_param_args(ap)
    args = ap.parse_args()
    base = params_from_args(args)
    print("Desplazamiento en y [m] (ideal 0.400)")
    print("mu \\ damping " + "".join(f"{d:>9.0e}" for d in args.dampings))
    for mu in args.mus:
        row = []
        for damp in args.dampings:
            p = dataclasses.replace(base, mu=mu, roller_damping=damp)
            row.append(_drive(MujocoSim(params=p, world="vacio"), (0, V, 0))[1])
        print(f"{mu:<13}" + "".join(f"{v:>9.3f}" for v in row))
