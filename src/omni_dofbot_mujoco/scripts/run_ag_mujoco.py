#!/usr/bin/env python3
"""AG de sintonización PID (ag_motion_tests.py, sin modificar) sobre MuJoCo, sin ROS.

Usa los archivos reales de omni_dofbot_bringup/src (AG, nodo cinemático con PID
+ planta del motor, odometría) y el puente de MuJoCo, conectados en proceso y
en paso fijo (ver omni_dofbot_mujoco/lockstep.py). Corre más rápido que tiempo
real y es determinista con --seed.

Ejemplos:
  # una batería P1-P4 con unas ganancias (una evaluación del AG)
  python3 run_ag_mujoco.py --battery 0.8 0.06 0.5

  # AG de humo (población 8, 3 generaciones, 1 ronda)
  python3 run_ag_mujoco.py --smoke --seed 1 --out resultados_humo

  # AG completo con la configuración de ag_motion_tests.py
  python3 run_ag_mujoco.py --seed 1 --out resultados_ag

  # verlo en el visor a velocidad real (requiere pantalla)
  python3 run_ag_mujoco.py --battery 0.8 0.06 0.5 --viewer --realtime

Salidas del AG (en --out): ag_results.json, ag_results.png, ag_results.html,
igual que en Gazebo, más ag_log.txt con todo el registro.
"""
import argparse
import json
import os
import time

import _path  # noqa: F401
from omni_dofbot_mujoco.lockstep import LockstepWorld

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--battery", nargs=3, type=float, metavar=("KP", "KI", "KD"),
                    help="solo una batería P1-P4 con estas ganancias")
    ap.add_argument("--smoke", action="store_true",
                    help="AG de humo: POP_SIZE=8, N_GEN=3, MAX_ROUNDS=1")
    ap.add_argument("--pop", type=int, help="sobrescribe POP_SIZE")
    ap.add_argument("--gen", type=int, help="sobrescribe N_GEN")
    ap.add_argument("--max-rounds", type=int, help="sobrescribe MAX_ROUNDS")
    ap.add_argument("--seed", type=int, help="semilla de random (corrida reproducible)")
    ap.add_argument("--w-vel", type=float,
                    help="sobrescribe W_VEL del AG: peso del seguimiento de velocidad "
                         "(0 = costo original; ver CHANGELOG #10)")
    ap.add_argument("--out", default="resultados_ag_mujoco", help="carpeta de salida")
    ap.add_argument("--model", help="MJCF (por defecto models/omni_dofbot_arena.xml)")
    ap.add_argument("--bringup-dir", help="carpeta con ag_motion_tests.py y los nodos")
    ap.add_argument("--viewer", action="store_true", help="abrir el visor de MuJoCo")
    ap.add_argument("--realtime", action="store_true", help="con --viewer: a velocidad real")
    ap.add_argument("--verbose", action="store_true",
                    help="mostrar también el INFO de los nodos cinemático y de odometría")
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, "ag_log.txt"), "w") as log:
        w = LockstepWorld(model_path=a.model, bringup_dir=a.bringup_dir,
                          verbose=a.verbose, viewer=a.viewer, realtime=a.realtime,
                          log_file=log)
        w.configure(W_VEL=a.w_vel)
        t0 = time.perf_counter()
        try:
            if a.battery:
                r = w.evaluate(*a.battery)
                wall = time.perf_counter() - t0
                r["wall_s"] = wall
                print(json.dumps(r, indent=2))
                with open(os.path.join(a.out, "bateria.json"), "w") as f:
                    json.dump(r, f, indent=2)
            else:
                if a.smoke:
                    w.configure(POP_SIZE=8, N_GEN=3, MAX_ROUNDS=1)
                w.configure(POP_SIZE=a.pop, N_GEN=a.gen, MAX_ROUNDS=a.max_rounds)
                w.run_ag(out_dir=a.out, seed=a.seed)
                wall = time.perf_counter() - t0
            print(f"Tiempo de simulación {w.sim.time:.1f} s en {wall:.1f} s de pared "
                  f"({w.sim.time / wall:.1f}x). Salidas en {os.path.abspath(a.out)}")
        finally:
            w.close()
