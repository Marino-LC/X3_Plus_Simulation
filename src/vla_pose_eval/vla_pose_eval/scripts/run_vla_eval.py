#!/usr/bin/env python3
"""
Ejecuta el paquete vla_pose_eval con ros2, EN ORDEN (cada paso usa la salida
del anterior):

  1. run-estimator  por estimador: escribe estimates_<est>.yaml en cada ensayo
  2. evaluate       por estimador: lee esas estimaciones, escribe errors_<est>.csv
  3. report         todos juntos: lee los CSV, escribe summary.md / summary.json

`evaluate` y `report` no pueden ir en paralelo: `report` leería CSV viejos o a
medio escribir.

Uso:
  python3 run_vla_eval.py --run-dir ~/vla_runs/test --estimators oracle owlv2
  python3 run_vla_eval.py --ws ~/ROS2Dev/VLA_Model --estimators owlv2 owlv2_rgbd

Logs en <RUN_DIR>/logs/
"""

import argparse
import glob
import os
import shlex
import subprocess
import sys
from pathlib import Path

PKG = "vla_pose_eval"


def find_ros_setup() -> Path:
    """Busca el setup.bash de ROS 2 en /opt/ros (usa ROS_DISTRO si está definido)."""
    candidates = sorted(glob.glob("/opt/ros/*/setup.bash"))
    if not candidates:
        sys.exit("No encontré ROS 2 en /opt/ros. Revisa tu instalación.")
    distro = os.environ.get("ROS_DISTRO")
    if distro:
        for path in candidates:
            if f"/opt/ros/{distro}/" in path:
                return Path(path)
    return Path(candidates[-1])


def ros2_cmd(ws_dir: Path, *args) -> list:
    """Comando que carga ROS 2 y el workspace antes de ejecutar ros2."""
    ws_setup = ws_dir / "install" / "setup.bash"
    if not ws_setup.exists():
        sys.exit(f"No encontré {ws_setup}. Compila el workspace con 'colcon build'.")
    shell = (
        f"source {shlex.quote(str(find_ros_setup()))} && "
        f"source {shlex.quote(str(ws_setup))} && "
        "exec ros2 " + " ".join(shlex.quote(str(a)) for a in args)
    )
    return ["bash", "-c", shell]


def run(name: str, ws_dir: Path, log_dir: Path, args: list) -> bool:
    """Ejecuta un paso, espera a que termine y deja su salida en log_dir/<name>.log."""
    log_path = log_dir / f"{name}.log"
    with open(log_path, "w") as log:
        code = subprocess.call(ros2_cmd(ws_dir, *args), stdout=log, stderr=subprocess.STDOUT)
    ok = code == 0
    print(f"[{'OK' if ok else 'ERROR'}]".ljust(8) + name + ("" if ok else f" (revisa {log_path})"))
    return ok


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ws", default="~/ROS2Dev/VLA_Model",
                   help="workspace compilado (contiene install/)")
    p.add_argument("--run-dir", default="~/vla_runs/test", help="directorio de los ensayos")
    p.add_argument("--estimators", nargs="+", default=["oracle"],
                   help="p. ej. oracle oracle_multicam owlv2 owlv2_rgbd")
    p.add_argument("--kwargs", default="", help="JSON para run-estimator (todos los estimadores)")
    p.add_argument("--skip-estimate", action="store_true",
                   help="solo evaluate + report (reutiliza estimates_*.yaml)")
    a = p.parse_args()

    ws_dir, run_dir = Path(a.ws).expanduser(), Path(a.run_dir).expanduser()
    if not run_dir.exists():
        print(f"[ERROR] No existe la carpeta de ejecución: {run_dir}")
        return 1
    log_dir = run_dir / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    for est in a.estimators:
        if not a.skip_estimate:
            extra = ["--kwargs", a.kwargs] if a.kwargs else []
            if not run(f"run-estimator_{est}", ws_dir, log_dir,
                       ["run", PKG, PKG, "run-estimator", run_dir, "--estimator", est, *extra]):
                return 1
        if not run(f"evaluate_{est}", ws_dir, log_dir,
                   ["run", PKG, PKG, "evaluate", run_dir, "--estimators", est]):
            return 1
    if not run("report", ws_dir, log_dir,
               ["run", PKG, PKG, "report", run_dir, "--estimators", *a.estimators]):
        return 1
    print(f"Reporte: {run_dir / 'summary.md'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
