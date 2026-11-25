#!/usr/bin/env python3
"""
Ejecuta el paquete vla_pose_eval con ros2:
  1. run-estimator (secuencial)
  2. evaluate y report (en paralelo, cuando termina el paso 1)

Logs en <RUN_DIR>/logs/
"""

import glob
import os
import shlex
import subprocess
import sys
from pathlib import Path

PKG = "vla_pose_eval"
WS_DIR = Path("~/ROS2Dev/VLA_Model").expanduser()   # workspace donde compilaste (contiene install/)
RUN_DIR = Path("~/vla_runs/test").expanduser()
ESTIMATOR = "oracle"
LOG_DIR = RUN_DIR / "logs"


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


def ros2_cmd(*args) -> list[str]:
    """Construye un comando que carga ROS 2 y el workspace antes de ejecutar ros2."""
    ros_setup = find_ros_setup()
    ws_setup = WS_DIR / "install" / "setup.bash"
    if not ws_setup.exists():
        sys.exit(f"No encontré {ws_setup}. Compila el workspace con 'colcon build'.")

    shell = (
        f"source {shlex.quote(str(ros_setup))} && "
        f"source {shlex.quote(str(ws_setup))} && "
        "exec ros2 " + " ".join(shlex.quote(str(a)) for a in args)
    )
    return ["bash", "-c", shell]


def start(name: str, args: list) -> tuple[subprocess.Popen, object]:
    """Lanza un proceso con su salida redirigida a LOG_DIR/<name>.log."""
    log = open(LOG_DIR / f"{name}.log", "w")
    proc = subprocess.Popen(ros2_cmd(*args), stdout=log, stderr=subprocess.STDOUT)
    return proc, log


def main() -> int:
    LOG_DIR.mkdir(parents=True, exist_ok=True)

    if not RUN_DIR.exists():
        print(f"[ERROR] No existe la carpeta de ejecución: {RUN_DIR}")
        return 1

    # Paso 1: run-estimator (debe terminar antes de los demás)
    print("Paso 1: run-estimator")
    proc, log = start(
        "run-estimator",
        ["run", PKG, PKG, "run-estimator", RUN_DIR, "--estimator", ESTIMATOR],
    )
    code = proc.wait()
    log.close()
    if code != 0:
        print(f"[ERROR] run-estimator falló (revisa {LOG_DIR / 'run-estimator.log'})")
        return 1
    print("[OK]    run-estimator")

    # Paso 2: evaluate y report en paralelo
    print("Paso 2: evaluate y report en paralelo")
    jobs = {
        "evaluate": ["run", PKG, PKG, "evaluate", RUN_DIR, "--estimators", ESTIMATOR],
        "report": ["run", PKG, PKG, "report", RUN_DIR, "--estimators", ESTIMATOR],
    }
    running = {name: start(name, args) for name, args in jobs.items()}

    status = 0
    for name, (proc, log) in running.items():
        code = proc.wait()
        log.close()
        if code == 0:
            print(f"[OK]    {name}")
        else:
            print(f"[ERROR] {name} (revisa {LOG_DIR / f'{name}.log'})")
            status = 1

    return status


if __name__ == "__main__":
    sys.exit(main())
