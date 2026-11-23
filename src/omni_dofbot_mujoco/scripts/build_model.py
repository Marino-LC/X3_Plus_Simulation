#!/usr/bin/env python3
"""Regenera models/omni_dofbot_{arena,vacio}.xml.

Ejemplos:
  python3 build_model.py                       # valores por defecto (params.py)
  python3 build_model.py --mu 0.5 --roller_damping 1e-3
"""
import argparse
import dataclasses

import _path  # noqa: F401
import mujoco
from omni_dofbot_mujoco.model_builder import write_models
from omni_dofbot_mujoco.params import ModelParams


def add_param_args(ap):
    for f in dataclasses.fields(ModelParams):
        if f.type in (float, int, str):
            ap.add_argument(f"--{f.name}", type=f.type, default=f.default)


def params_from_args(args):
    names = {f.name for f in dataclasses.fields(ModelParams)}
    return ModelParams(**{k: v for k, v in vars(args).items() if k in names})


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    add_param_args(ap)
    p = params_from_args(ap.parse_args())
    for path in write_models(p):
        m = mujoco.MjModel.from_xml_path(path)
        print(f"{path}\n  masa total = {mujoco.mj_getTotalmass(m):.3f} kg, "
              f"nv = {m.nv}, actuadores = {m.nu}")
