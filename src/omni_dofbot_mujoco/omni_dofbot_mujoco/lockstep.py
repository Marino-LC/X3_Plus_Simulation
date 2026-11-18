"""AG de Gazebo sobre MuJoCo, SIN ROS y en paso fijo.

Carga los archivos reales, sin modificarlos:
  omni_dofbot_bringup/src/ag_motion_tests.py
  omni_dofbot_bringup/src/mecanum_kinematic_node.py
  omni_dofbot_bringup/src/Mecanum_odometry_node.py
  omni_dofbot_mujoco/src/mujoco_bridge_node.py
y los conecta con fake_ros (tópicos y servicios síncronos). La lógica del AG,
del PID + planta del motor y de la odometría es por lo tanto la MISMA que en
Gazebo; solo cambia el motor de física.

Cómo se sincroniza (diferencia con ROS real):
  * El reloj de todos los nodos es data.time de MuJoCo.
  * Cada time.sleep() del hilo principal del AG avanza la física 1 paso (1 ms)
    por cada milisegundo pedido; _sleep() del AG ya espera en tiempo de
    simulación, así que los tiempos de la batería quedan idénticos.
  * El hilo del brazo del AG corre de forma cooperativa: solo avanza entre
    pasos de física, nunca en paralelo. Resultado: corridas deterministas.
  * Como la física no depende del reloj de pared, el control de RTF del AG
    (_gated) no aplica: se reporta RTF = 1.0 y ninguna evaluación se descarta.
  * El nodo cinemático corre a 100 Hz con temporizador en tiempo de simulación;
    /joint_states se publica a 100 Hz con header.stamp de simulación.
"""
import importlib.util
import math
import os
import random
import sys
import threading
import time as _real_time
import types

from . import fake_ros
from .model_builder import models_dir
from .sim import MujocoSim

QUIET_NODES = ("mecanum_kinematic_node", "mecanum_odometry_node", "mujoco_bridge_node")


def default_bringup_dir():
    """omni_dofbot_bringup/src en el árbol fuente, o lib/ del paquete instalado."""
    here = os.path.dirname(os.path.realpath(__file__))
    src = os.path.normpath(os.path.join(here, "..", "..", "omni_dofbot_bringup", "src"))
    if os.path.isfile(os.path.join(src, "ag_motion_tests.py")):
        return src
    from ament_index_python.packages import get_package_prefix
    return os.path.join(get_package_prefix("omni_dofbot_bringup"), "lib",
                        "omni_dofbot_bringup")


def _bridge_path():
    here = os.path.dirname(os.path.realpath(__file__))
    src = os.path.normpath(os.path.join(here, "..", "src", "mujoco_bridge_node.py"))
    if os.path.isfile(src):
        return src
    from ament_index_python.packages import get_package_prefix
    return os.path.join(get_package_prefix("omni_dofbot_mujoco"), "lib",
                        "omni_dofbot_mujoco", "mujoco_bridge_node.py")


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


# ═════════════════════════════════════════════════════════════════════════════
# Planificador cooperativo
# ═════════════════════════════════════════════════════════════════════════════
class _CoopThread:
    """Sustituto de threading.Thread para los hilos que crea el AG: corre en un
    hilo real, pero solo cuando el planificador le pasa el turno."""

    def __init__(self, sched, group=None, target=None, name=None, args=(),
                 kwargs=None, daemon=None):
        self._s, self._target = sched, target
        self._args, self._kwargs = args, kwargs or {}
        self._go, self._back = threading.Semaphore(0), threading.Semaphore(0)
        self.alive, self.wake_at, self._exc = False, 0.0, None
        self.daemon = daemon

    def _body(self):
        self._go.acquire()
        try:
            if self._target:
                self._target(*self._args, **self._kwargs)
        except BaseException as e:          # se re-lanza en el hilo principal
            self._exc = e
        finally:
            self.alive = False
            self._s.workers.pop(threading.get_ident(), None)
            self._back.release()

    def start(self):
        self.alive = True
        t = threading.Thread(target=self._body, daemon=True)
        t.start()
        self._s.workers[t.ident] = self
        self.resume()

    def resume(self):
        """Hilo principal: deja correr al trabajador hasta su próximo sleep."""
        self._go.release()
        self._back.acquire()
        if self._exc is not None:
            exc, self._exc = self._exc, None
            raise exc

    def park(self, dt):
        """Hilo trabajador: duerme `dt` s de simulación y devuelve el turno."""
        self.wake_at = self._s.now() + max(dt, 0.0)
        self._back.release()
        self._go.acquire()

    def join(self, timeout=None):
        while self.alive:
            self._s.tick()

    def is_alive(self):
        return self.alive


class Scheduler:
    def __init__(self, world):
        self.w = world
        self.workers = {}
        self.main_ident = threading.get_ident()

    def now(self):
        return self.w.sim.time

    def tick(self):
        """Un paso de física + temporizadores + hilos cooperativos que despiertan."""
        self.w.step()
        t = self.now()
        for wk in list(self.workers.values()):
            if wk.alive and t >= wk.wake_at - 1e-9:
                wk.resume()

    def sleep(self, dt):
        ident = threading.get_ident()
        wk = self.workers.get(ident)
        if wk is not None:
            wk.park(dt)
        elif ident == self.main_ident:
            for _ in range(max(1, int(round(dt / self.w.dt)))):
                self.tick()
        else:
            _real_time.sleep(dt)

    def time_module(self):
        m = types.ModuleType("time")
        m.__dict__.update({k: getattr(_real_time, k) for k in dir(_real_time)
                           if not k.startswith("__")})
        m.sleep = self.sleep
        return m

    def threading_module(self):
        m = types.ModuleType("threading")
        m.__dict__.update({k: getattr(threading, k) for k in dir(threading)
                           if not k.startswith("__")})
        sched = self
        m.Thread = lambda *a, **k: _CoopThread(sched, *a, **k)
        return m


class _LockstepRtf:
    """Sustituye a RtfMonitor: en paso fijo la física no depende del reloj de pared."""

    def __init__(self, node, window=3.0): pass
    def begin(self): pass
    def end(self): return 1.0, 1.0


# ═════════════════════════════════════════════════════════════════════════════
# Mundo: MuJoCo + puente + nodos reales + AG
# ═════════════════════════════════════════════════════════════════════════════
class LockstepWorld:

    def __init__(self, model_path=None, bringup_dir=None, verbose=False,
                 viewer=False, realtime=False, log_file=None):
        self.verbose = verbose
        self._log_file = log_file
        self.sim = MujocoSim(model=model_path or os.path.join(models_dir(),
                                                              "omni_dofbot_arena.xml"))
        self.dt = self.sim.model.opt.timestep
        self.graph = fake_ros.install(fake_ros.Graph(lambda: self.sim.time, self._log))
        self.graph.param_overrides["mujoco_bridge_node"] = {"autostart": False}
        bdir = bringup_dir or default_bringup_dir()

        bridge_mod = _load("mujoco_bridge_node", _bridge_path())
        kin_mod = _load("mecanum_kinematic_node",
                        os.path.join(bdir, "mecanum_kinematic_node.py"))
        odom_mod = _load("Mecanum_odometry_node",
                         os.path.join(bdir, "Mecanum_odometry_node.py"))
        self.ag = _load("ag_motion_tests", os.path.join(bdir, "ag_motion_tests.py"))

        self.bridge = bridge_mod.MujocoBridge(sim=self.sim)
        self.kin = kin_mod.MecanumKinematicNode()
        self.odom = odom_mod.MecanumOdometryNode()

        self.sched = Scheduler(self)
        self.ag.time = self.sched.time_module()
        self.ag.threading = self.sched.threading_module()
        self.ag.RtfMonitor = _LockstepRtf
        self.ag.webbrowser = types.SimpleNamespace(open=lambda *a, **k: False)

        self._viewer, self._realtime = None, realtime
        if viewer:
            import mujoco.viewer
            self._viewer = mujoco.viewer.launch_passive(self.sim.model, self.sim.data)
            self._view_next = 0.0
        self._wall0, self._sim0 = _real_time.perf_counter(), self.sim.time

    # ── registro ─────────────────────────────────────────────────────────────
    def _log(self, node, level, msg):
        line = f"[{self.sim.time:9.3f}] [{level}] [{node}] {msg}"
        if self._log_file:
            self._log_file.write(line + "\n")
        if self.verbose or level != "INFO" or node not in QUIET_NODES:
            print(line, flush=True)

    # ── paso ─────────────────────────────────────────────────────────────────
    def step(self):
        self.bridge.step_once()
        self.graph.fire_timers()
        if self._viewer is not None:
            now = _real_time.perf_counter()
            if self._realtime:
                lag = (self.sim.time - self._sim0) - (now - self._wall0)
                if lag > 0:
                    _real_time.sleep(lag)
            if now >= self._view_next:
                self._viewer.sync()
                self._view_next = now + 1 / 30

    # ── API ──────────────────────────────────────────────────────────────────
    def configure(self, **constants):
        """Sobrescribe constantes del AG (POP_SIZE, N_GEN, MAX_ROUNDS, ...)."""
        for k, v in constants.items():
            if v is None:
                continue
            if not hasattr(self.ag, k):
                raise AttributeError(f"ag_motion_tests no define {k}")
            setattr(self.ag, k, v)

    def run_ag(self, out_dir=".", seed=None):
        """Ejecuta ag_motion_tests.main() completo. Salidas en out_dir."""
        if seed is not None:
            random.seed(seed)
        os.makedirs(out_dir, exist_ok=True)
        cwd = os.getcwd()
        os.chdir(out_dir)
        try:
            self.ag.main()
        finally:
            os.chdir(cwd)

    def evaluate(self, kp, ki, kd):
        """Una batería P1-P4 con estas ganancias (como una evaluación del AG).
        Devuelve dict con costos, fitness y tiempo de simulación usado."""
        if not hasattr(self, "_evaluator"):
            self._evaluator = self.ag.AGMotionEvaluator()
        t0 = self.sim.time
        (fit,) = self._evaluator.evaluate([kp, ki, kd])
        il = self._evaluator._all_individuals[-1]
        return {"kp": kp, "ki": ki, "kd": kd, "P1": il.cost_p1, "P2": il.cost_p2,
                "P3": il.cost_p3, "P4": il.cost_p4, "fitness": fit,
                "sim_s": self.sim.time - t0}

    def record(self, kp, ki, kd):
        """Corrida grabada (record_best): segmentos con series de tiempo."""
        if not hasattr(self, "_evaluator"):
            self._evaluator = self.ag.AGMotionEvaluator()
        return self._evaluator.record_best([kp, ki, kd])

    def close(self):
        if self._viewer is not None:
            self._viewer.close()


def isfinite_costs(res):
    return all(math.isfinite(res[k]) for k in ("P1", "P2", "P3", "P4", "fitness"))
