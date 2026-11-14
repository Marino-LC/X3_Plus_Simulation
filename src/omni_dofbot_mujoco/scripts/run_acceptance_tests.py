#!/usr/bin/env python3
"""Pruebas de aceptación T1-T9 de la guía (sección 8), sin ROS.

  python3 run_acceptance_tests.py                 # todas las automatizables
  python3 run_acceptance_tests.py --only T2 T3    # un subconjunto
  python3 run_acceptance_tests.py --mu 0.5        # con otros parámetros
  python3 run_acceptance_tests.py --csv out.csv   # guarda la tabla

T6 (escalón de rueda) y T10 (equivalencia) comparan contra Gazebo y se hacen
con ROS 2: ver el README, sección de protocolos.
Código de salida 0 si todas las pruebas ejecutadas pasan.
"""
import argparse
import csv
import math
import sys
import time

import _path  # noqa: F401
import numpy as np
from build_model import add_param_args, params_from_args
from omni_dofbot_mujoco.model_builder import build_model
from omni_dofbot_mujoco.sim import MujocoSim

V, WZ, T_CMD, SETTLE = 0.2, 1.0, 2.0, 1.0
ARM_HOME = [0.00, 0.00, 0.00, 0.00, 1.57]
ARM_PICK_LEFT = [-1.20, -1.25, -0.7, -0.3, 1.57]
ARM_PICK_RIGHT = [1.20, -1.25, -0.7, -0.3, 1.57]
ARM_JOINTS = [f"arm_joint_0{i}" for i in range(1, 6)]


def _drive(sim, twist, seconds=T_CMD):
    """Asienta, aplica el escalón de twist y devuelve el desplazamiento en el
    marco inicial del robot: (dx, dy, dyaw)."""
    sim.reset((0, 0, 0.05, 0))
    sim.run(SETTLE)
    x0, y0, yaw0 = sim.pose2d()
    sim.set_twist(*twist)
    yaw_acc, prev = 0.0, yaw0
    for _ in range(int(seconds / 0.01)):
        sim.run(0.01)
        yaw = sim.pose2d()[2]
        yaw_acc += math.atan2(math.sin(yaw - prev), math.cos(yaw - prev))
        prev = yaw
    x, y, _ = sim.pose2d()
    c, s = math.cos(yaw0), math.sin(yaw0)
    return c * (x - x0) + s * (y - y0), -s * (x - x0) + c * (y - y0), yaw_acc


def t1(mk):
    sim = mk("vacio")
    sim.reset((0, 0, 0.05, 0))
    sim.run(2.0)
    p0 = sim.ground_truth()[0].copy()
    vmax, zs = 0.0, []
    for _ in range(100):
        sim.run(0.01)
        p, _, lin, ang = sim.ground_truth()
        vmax = max(vmax, np.abs(lin).max(), np.abs(ang).max())
        zs.append(p[2])
    drift = float(np.linalg.norm(sim.ground_truth()[0][:2] - p0[:2]))
    ok = vmax < 1e-3 and np.ptp(zs) < 1e-4 and drift < 1e-3
    return ok, (f"|v|max={vmax:.1e} (<1e-3), z={np.mean(zs) * 1e3:.2f} mm "
                f"(ptp {np.ptp(zs) * 1e6:.1f} um), deriva={drift * 1e3:.3f} mm")


def _dist_test(twist, ideal, pick, unit):
    def run(mk):
        d = _drive(mk("vacio"), twist)
        val = pick(d)
        err = abs(val - ideal) / abs(ideal) * 100
        return err < 5.0, (f"medido={val:.4f} {unit}, ideal={ideal:.4f} {unit}, "
                           f"error={err:.2f} % (<5 %)")
    return run


t2 = _dist_test((V, 0, 0), V * T_CMD, lambda d: d[0], "m")
t3 = _dist_test((0, V, 0), V * T_CMD, lambda d: d[1], "m")
t4 = _dist_test((0, 0, WZ), WZ * T_CMD, lambda d: d[2], "rad")


def t5(mk):
    dx, dy, dyaw = _drive(mk("vacio"), (V, V, 0))
    ang = math.degrees(math.atan2(dy, dx))
    return abs(ang - 45) < 3, (f"dirección={ang:.2f}° (45±3°), "
                               f"|d|={math.hypot(dx, dy):.4f} m, yaw={math.degrees(dyaw):.2f}°")


def _arm_run(sim, with_arm):
    """Avance de 0.2 m/s durante 4 s; opcionalmente con la coreografía del AG."""
    sim.reset()                      # pose del AG: (0, 0, 0.05), yaw = pi/2
    sim.set_trajectory(ARM_JOINTS, [(0.5, ARM_HOME)])
    sim.run(SETTLE)
    x0, y0, yaw0 = sim.pose2d()
    sim.set_twist(V, 0, 0)
    if with_arm:
        sim.set_trajectory(ARM_JOINTS, [(1.0, ARM_PICK_LEFT), (2.0, ARM_HOME),
                                        (3.0, ARM_PICK_RIGHT), (4.0, ARM_HOME)])
        sim.set_trajectory(["grip_joint"], [(1.0, [-0.75]), (3.0, [0.0])])
    tilt = 0.0
    for _ in range(400):
        sim.run(0.01)
        tilt = max(tilt, sim.tilt())
    x, y, yaw = sim.pose2d()
    # yaw0 = pi/2: el avance es +Y del mundo y el deslizamiento lateral es X
    return y - y0, x - x0, yaw - yaw0, tilt


def t7(mk):
    sim = mk("arena")
    a0, l0, y0, _ = _arm_run(sim, False)
    a1, l1, y1, tilt = _arm_run(sim, True)
    ok = tilt < math.radians(15)
    return ok, (f"inclinación máx={math.degrees(tilt):.2f}° (<15°). Perturbación del "
                f"brazo: avance {1e3 * (a1 - a0):+.1f} mm, lateral {1e3 * (l1 - l0):+.1f} mm, "
                f"yaw {math.degrees(y1 - y0):+.2f}°")


def t8(mk):
    sim = mk("arena")
    sim.reset()
    sim.set_twist(V, 0, 0)
    sim.set_trajectory(ARM_JOINTS, [(1.0, ARM_PICK_LEFT), (2.0, ARM_HOME)])
    t0 = time.perf_counter()
    sim.run(3.0)
    rtf = 3.0 / (time.perf_counter() - t0)
    return rtf >= 1.0, (f"factor de tiempo real={rtf:.1f}x (>=1x) solo física, 1 hilo; "
                        f"con ROS 2 y visor medir de nuevo (README)")


def t9(mk):
    def traj(sim):
        sim.reset()
        out = []
        for tw in ((V, 0, 0), (0, V, 0), (0, 0, WZ), (0, 0, 0)):
            sim.set_twist(*tw)
            sim.run(0.5)
            out.append(sim.data.qpos.copy())
        return np.concatenate(out)
    sim = mk("arena")
    a, b = traj(sim), traj(sim)          # misma instancia tras reset()
    c = traj(mk("arena"))                # instancia nueva
    diff = max(np.abs(a - b).max(), np.abs(a - c).max())
    return diff == 0.0, f"diferencia máx. de qpos entre repeticiones={diff:.1e} (=0)"


TESTS = {"T1": ("Reposo", t1), "T2": ("Avance en x", t2),
         "T3": ("Avance lateral en y", t3), "T4": ("Rotación", t4),
         "T5": ("Diagonal", t5), "T7": ("Con brazo", t7),
         "T8": ("Tiempo real", t8), "T9": ("Determinismo", t9)}


def run_tests(params, only=None):
    cache = {}

    def mk(world):
        if world not in cache:
            cache[world] = build_model(params, world)[0]
        return MujocoSim(model=cache[world])

    rows = []
    for key, (name, fn) in TESTS.items():
        if only and key not in only:
            continue
        ok, detail = fn(mk)
        rows.append((key, name, "PASA" if ok else "FALLA", detail))
    return rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", nargs="*", choices=list(TESTS))
    ap.add_argument("--csv")
    add_param_args(ap)
    args = ap.parse_args()
    rows = run_tests(params_from_args(args), args.only)
    for key, name, res, detail in rows:
        print(f"{key:<3} {name:<20} {res:<5} {detail}")
    print("T6 / T10: requieren Gazebo + ROS 2 (protocolo en el README)")
    if args.csv:
        with open(args.csv, "w", newline="") as f:
            csv.writer(f).writerows([("prueba", "nombre", "resultado", "detalle")] + rows)
    sys.exit(0 if all(r[2] == "PASA" for r in rows) else 1)
