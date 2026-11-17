#!/usr/bin/env python3
"""
waypoints_nav.py
================
Ejecutor del experimento de navegación del Artículo 1 (solo Gazebo).

Por cada corrida recorre las 5 goal poses de config/goals.yaml. Cada goal es un
EPISODIO INDEPENDIENTE:
  1. Teleport del robot al spawn; reset de odometría, PID, localización
     (AMCL: /initialpose; SLAM: /slam_toolbox/reset) y costmaps.
  2. NavigateToPose directo al goal.
  3. Durante el episodio registra a 20 Hz la pose real (/ground_truth/odom), la
     odometría (/odom) y la pose estimada (TF map -> base_footprint), además de
     cada plan global (/plan), colisiones (/contacts/*) y recuperaciones.
  4. Al terminar calcula las métricas (omni_dofbot_bringup/nav_metrics.py):
       éxito (Nav2 OK + pose real dentro de pos_tol/yaw_tol + sin colisión),
       SPL, error final de posición y orientación, ATE de la localización y de
       la odometría, error transversal y de rumbo respecto al plan activo,
       tiempo de misión, recuperaciones, colisiones, replanificaciones, RTF,
       CPU de Gazebo y de la máquina por segundo simulado y, en modo SLAM,
       IoU del mapa y error medio de pared.

Salidas en output_dir (por defecto ./results/<fecha>_<localización>/, relativo
al directorio desde el que se lanza):
  results.json            configuración, resumen (global y por goal, con
                          intervalo de Wilson 95 %) y todos los episodios
  episodes.csv            una fila por episodio
  traj/runXX_goalY.csv        trayectoria (real, odometría, estimada)
  traj/runXX_goalY_plans.csv  planes globales recibidos
  slam_maps/runXX_goalY.*     mapa de SLAM al final del episodio (modo slam)
  plots/goal_Y.png, plots/overview.png
  reference_map/          copia del mapa de referencia usado

EJECUCIÓN (desde la raíz del workspace, con el entorno cargado):
  # T1 — simulación + control + Nav2 (AMCL)
  ros2 launch omni_dofbot_bringup arena_nav.launch.py
  # T2 — experimento
  ros2 run omni_dofbot_bringup waypoints_nav.py --ros-args -p runs:=20
  # Con SLAM: lanzar con localization:=slam y pasar el mismo parámetro aquí.

Regenerar resumen y gráficas de una carpeta:
  python3 -m omni_dofbot_bringup.nav_metrics results/<carpeta>
"""

import os
import json
import math
import time
import shutil
import threading
import subprocess

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter as RclpyParameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from geometry_msgs.msg import (
    PoseStamped, PoseWithCovarianceStamped, Twist, Pose, Point, Quaternion)
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from action_msgs.msg import GoalStatus
from std_srvs.srv import Empty

from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity, Contacts

import tf2_ros

from omni_dofbot_bringup import nav_metrics as nm

CONTACT_GAP = 0.5        # s simulados sin contacto para contar una colisión nueva
TRAJ_PERIOD = 0.05       # s simulados entre muestras de trayectoria (20 Hz)


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _quat(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class NavExperiment(Node):

    def __init__(self):
        super().__init__(
            "waypoint_runner",
            parameter_overrides=[
                RclpyParameter("use_sim_time", RclpyParameter.Type.BOOL, True)])
        self._cbg = ReentrantCallbackGroup()

        # ── Parámetros ────────────────────────────────────────────────────────
        self.declare_parameter("runs", 20)             # N >= 20 episodios por goal
        self.declare_parameter("localization", "amcl")  # amcl | slam (igual que el launch)
        self.declare_parameter("pos_tol", 0.08)        # m, contra verdad de terreno
        self.declare_parameter("yaw_tol", 0.15)        # rad, contra verdad de terreno
        self.declare_parameter("goal_timeout", 90.0)   # s simulados por episodio
        self.declare_parameter("goals_file", "")       # vacío = config/goals.yaml del paquete
        self.declare_parameter("output_dir", "")       # vacío = ./results/<fecha>_<loc>
        self.declare_parameter("make_plots", True)
        self.declare_parameter("set_gains", True)      # fija las ganancias del AG
        self.declare_parameter("kp", 1.949)            # Ganancia Proporcional definitiva
        self.declare_parameter("ki", 0.256)            # Ganancia Integral definitiva
        self.declare_parameter("kd", 0.938)            # Ganancia Derivativa definitiva

        self._loc = str(self.get_parameter("localization").value).lower()
        if self._loc not in ("amcl", "slam"):
            raise ValueError("localization debe ser 'amcl' o 'slam'")

        share = get_package_share_directory("omni_dofbot_bringup")
        goals_file = self.get_parameter("goals_file").value or os.path.join(
            share, "config", "goals.yaml")
        with open(goals_file) as f:
            self._cfg = yaml.safe_load(f)
        self._map_yaml = os.path.join(share, "maps", "arena_map.yaml")
        self._ref = nm.load_map_yaml(self._map_yaml)
        arena = self._cfg["arena"]
        self._world = arena["world_name"]
        self._robot = arena["robot_name"]
        sp = self._cfg["spawn"]
        self._spawn = (float(sp["x"]), float(sp["y"]), float(sp["yaw"]))
        self._goals = self._validate_goals()

        # ── Clientes / publicadores ───────────────────────────────────────────
        self._pid_client = self.create_client(
            SetParameters, "/mecanum_kinematic_node/set_parameters",
            callback_group=self._cbg)
        self._nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self._cbg)
        self._tp_client = self.create_client(
            SetEntityPose, f"/world/{self._world}/set_entity_pose",
            callback_group=self._cbg)
        self._reset_odom_cli = self.create_client(
            Empty, "/mecanum_odometry_node/reset_pose", callback_group=self._cbg)
        self._reset_ctrl_cli = self.create_client(
            Empty, "/mecanum_kinematic_node/reset_controller_state",
            callback_group=self._cbg)
        self._clear_costmap_clis = [
            self.create_client(ClearEntireCostmap, name, callback_group=self._cbg)
            for name in ("/global_costmap/clear_entirely_global_costmap",
                         "/local_costmap/clear_entirely_local_costmap")]
        self._slam_reset_cli = None
        if self._loc == "slam":
            from slam_toolbox.srv import Reset   # solo hace falta en modo SLAM
            self._SlamReset = Reset
            self._slam_reset_cli = self.create_client(
                Reset, "/slam_toolbox/reset", callback_group=self._cbg)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)

        # ── Estado medido ─────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._gt = None            # (x, y, yaw) real, marco del mundo
        self._odom = None          # (x, y, yaw) odometría, marco odom
        self._recording = False
        self._traj = []
        self._plans = []
        self._last_sample_t = -1.0
        self._collisions = []
        self._last_contact_t = {}
        self._map_msg = None
        self._map_rx_wall = 0.0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(Odometry, "/ground_truth/odom", self._gt_cb, 50,
                                 callback_group=self._cbg)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 50,
                                 callback_group=self._cbg)
        self.create_subscription(Path, "/plan", self._plan_cb, 10,
                                 callback_group=self._cbg)
        self.create_subscription(
            OccupancyGrid, "/map", self._map_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self._cbg)
        for model in arena["contact_models"]:
            self.create_subscription(
                Contacts, f"/contacts/{model}",
                lambda msg, m=model: self._contact_cb(m, msg), 10,
                callback_group=self._cbg)

        self.get_logger().info(
            f"Experimento listo — {len(self._goals)} goals, localización={self._loc}")

    # ── Configuración ─────────────────────────────────────────────────────────
    def _validate_goals(self):
        """Comprueba que cada goal es alcanzable y calcula la ruta más corta (SPL)."""
        fp = self._cfg["footprint"]
        goals, errors = [], []
        for g in self._cfg["goals"]:
            pose = (float(g["x"]), float(g["y"]), float(g["yaw"]))
            clr = nm.footprint_clearance(self._ref, pose, fp["x_min"], fp["x_max"], fp["y_half"])
            if clr < fp["goal_clearance"]:
                errors.append(f"goal {g['id']}: holgura de la huella {clr:.3f} m "
                              f"< {fp['goal_clearance']} m (dentro o junto a un obstáculo)")
                continue
            try:
                length, path = nm.shortest_path(self._ref, self._spawn[:2], pose[:2],
                                                fp["inscribed_radius"])
            except ValueError as e:
                errors.append(f"goal {g['id']}: {e}")
                continue
            if not math.isfinite(length):
                errors.append(f"goal {g['id']}: no hay ruta desde el spawn")
                continue
            goals.append({"id": int(g["id"]), "label": g.get("label", ""),
                          "x": pose[0], "y": pose[1], "yaw": pose[2],
                          "footprint_clearance": clr,
                          "shortest_path_length": length,
                          "shortest_path": path.tolist()})
        if errors:
            for e in errors:
                self.get_logger().error(e)
            raise RuntimeError("Hay goals inválidos en goals.yaml; corrígelos antes de medir")
        for g in goals:
            self.get_logger().info(
                f"Goal {g['id']} ({g['x']:.2f}, {g['y']:.2f}, {g['yaw']:.2f}) "
                f"holgura={g['footprint_clearance']:.3f} m "
                f"ruta más corta={g['shortest_path_length']:.3f} m")
        return goals

    # ── Utilidades ────────────────────────────────────────────────────────────
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _sleep_sim(self, seconds: float):
        """Espera en tiempo SIMULADO (independiente del RTF)."""
        t_end = self._now() + seconds
        while self._now() < t_end and rclpy.ok():
            time.sleep(0.01)

    def _call(self, client, req, timeout=3.0):
        """Llama a un servicio sin bloquear el executor."""
        fut = client.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        return fut.result() if fut.done() else None

    @staticmethod
    def _wait_future(fut, timeout):
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        return fut.done()

    def _lookup_est(self):
        """Pose estimada por la localización: TF map -> base_footprint."""
        try:
            tf = self._tf_buffer.lookup_transform("map", "base_footprint", Time())
        except Exception:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y,
                _yaw_from_quat(tf.transform.rotation))

    # ── Callbacks de medición ─────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose
        with self._lock:
            self._odom = (p.position.x, p.position.y, _yaw_from_quat(p.orientation))

    def _gt_cb(self, msg: Odometry):
        p = msg.pose.pose
        x, y, yaw = p.position.x, p.position.y, _yaw_from_quat(p.orientation)
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        with self._lock:
            self._gt = (x, y, yaw)
            if not self._recording or t - self._last_sample_t < TRAJ_PERIOD:
                return
            self._last_sample_t = t
            odom = self._odom or (math.nan,) * 3
        est = self._lookup_est() or (math.nan,) * 3
        with self._lock:
            if self._recording:
                self._traj.append((t, x, y, yaw, *odom, *est))

    def _plan_cb(self, msg: Path):
        if not self._recording or not msg.poses:
            return
        xy = np.array([(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses])
        with self._lock:
            if self._recording:
                self._plans.append((self._now(), xy))

    def _map_cb(self, msg: OccupancyGrid):
        with self._lock:
            self._map_msg = msg
            self._map_rx_wall = time.time()

    def _contact_cb(self, model: str, msg: Contacts):
        if not self._recording:
            return
        if not any(self._robot in c.collision1.name or self._robot in c.collision2.name
                   for c in msg.contacts):
            return
        now = self._now()
        with self._lock:
            last = self._last_contact_t.get(model)
            self._last_contact_t[model] = now
            if last is None or now - last > CONTACT_GAP:
                self._collisions.append({"object": model, "t": now})
                self.get_logger().warn(f"COLISIÓN con {model}")

    def _gt_pose(self):
        with self._lock:
            return self._gt

    def _odom_pose(self):
        with self._lock:
            return self._odom

    # ── Reset del robot ───────────────────────────────────────────────────────
    def _teleport(self, x, y, yaw):
        z = 0.05
        q = _quat(yaw)
        if self._tp_client.wait_for_service(timeout_sec=2.0):
            req = SetEntityPose.Request()
            req.entity.name = self._robot
            req.entity.type = Entity.MODEL
            req.pose = Pose(position=Point(x=x, y=y, z=z), orientation=q)
            if self._call(self._tp_client, req) is not None:
                return
        rs = (f'name: "{self._robot}", position: {{x: {x}, y: {y}, z: {z}}}, '
              f'orientation: {{x: 0, y: 0, z: {q.z}, w: {q.w}}}')
        try:
            subprocess.run(
                ["gz", "service", "-s", f"/world/{self._world}/set_pose",
                 "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                 "--timeout", "2000", "--req", rs],
                check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except (subprocess.CalledProcessError, FileNotFoundError) as e:
            self.get_logger().error(f"gz service falló: {e}")

    def reset_robot(self):
        """Lleva el robot al spawn y reinicia odometría, PID, localización y
        costmaps. Devuelve (T_world_map, T_world_odom) o None si falló."""
        self._cmd_vel_pub.publish(Twist())
        self._sleep_sim(0.3)
        x, y, yaw = self._spawn
        self._teleport(x, y, yaw)
        self._sleep_sim(1.0)  # el robot cae y se asienta

        for cli, name in ((self._reset_odom_cli, "reset_pose de odometría"),
                          (self._reset_ctrl_cli, "reset_controller_state")):
            if not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(f"{name} no disponible")
                return None
            self._call(cli, Empty.Request(), 2.0)
        self._sleep_sim(0.3)

        if self._loc == "amcl":
            ip = PoseWithCovarianceStamped()
            ip.header.frame_id = "map"
            ip.header.stamp = self.get_clock().now().to_msg()
            ip.pose.pose = Pose(position=Point(x=x, y=y, z=0.0), orientation=_quat(yaw))
            ip.pose.covariance[0] = 0.01     # x
            ip.pose.covariance[7] = 0.01     # y
            ip.pose.covariance[35] = 0.01    # yaw
            self._initpose_pub.publish(ip)
            self._sleep_sim(1.0)
        else:
            if not self._slam_reset_cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("/slam_toolbox/reset no disponible")
                return None
            t_reset = time.time()
            self._call(self._slam_reset_cli, self._SlamReset.Request(), 5.0)
            self._sleep_sim(2.0)             # primeros scans del mapa nuevo
            t0 = time.time()
            while self._map_rx_wall < t_reset and time.time() - t0 < 10.0:
                time.sleep(0.1)
            if self._map_rx_wall < t_reset:
                self.get_logger().error("SLAM no publicó un mapa nuevo tras el reset")
                return None

        for cli in self._clear_costmap_clis:
            if cli.wait_for_service(timeout_sec=2.0):
                self._call(cli, ClearEntireCostmap.Request(), 2.0)
        self._sleep_sim(1.0)   # los costmaps se vuelven a llenar con /scan

        gt, odom, est = self._gt_pose(), self._odom_pose(), self._lookup_est()
        if gt is None or odom is None or est is None:
            self.get_logger().error("Reset fallido: falta verdad de terreno, odometría o TF map")
            return None
        if math.hypot(gt[0] - x, gt[1] - y) > 0.05 or abs(nm.wrap(gt[2] - yaw)) > 0.1:
            self.get_logger().error(
                f"Reset fallido: el robot está en ({gt[0]:.3f}, {gt[1]:.3f}, "
                f"{gt[2]:.2f} rad), no en el spawn")
            return None

        T_world_odom = nm.align_from_pair(gt, odom)
        if self._loc == "amcl":
            T_world_map = nm.IDENTITY       # el mapa estático está en el marco del mundo
            err = math.hypot(est[0] - gt[0], est[1] - gt[1])
            if err > 0.10:
                self.get_logger().warn(f"AMCL arranca a {err:.3f} m de la pose real")
        else:
            T_world_map = nm.align_from_pair(gt, est)   # el mapa de SLAM nace en el spawn
        return T_world_map, T_world_odom

    # ── PID ───────────────────────────────────────────────────────────────────
    def set_pid(self, kp, ki, kd):
        if not self._pid_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("mecanum_kinematic_node no disponible")
            return

        def _p(n, v):
            return Parameter(name=n, value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))

        req = SetParameters.Request()
        req.parameters = [_p("kp", kp), _p("ki", ki), _p("kd", kd)]
        res = self._call(self._pid_client, req)
        if res is None or not all(r.successful for r in res.results):
            self.get_logger().error("No se pudieron fijar las ganancias PID")
        else:
            self.get_logger().info(f"PID fijado: Kp={kp} Ki={ki} Kd={kd}")

    # ── Episodio ──────────────────────────────────────────────────────────────
    def run_episode(self, run, goal, T_world_map, T_world_odom, out_dir, gz_pids):
        gx, gy, gyaw = goal["x"], goal["y"], goal["yaw"]
        gm = nm.se2_compose(nm.se2_inverse(T_world_map), (gx, gy, gyaw))   # goal en map

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = PoseStamped()
        nav_goal.pose.header.frame_id = "map"
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose = Pose(position=Point(x=gm[0], y=gm[1], z=0.0),
                                  orientation=_quat(gm[2]))

        recoveries = [0]

        def _feedback(msg):
            recoveries[0] = max(recoveries[0], int(msg.feedback.number_of_recoveries))

        with self._lock:
            self._traj, self._plans = [], []
            self._last_sample_t = -1.0
            self._collisions, self._last_contact_t = [], {}
            self._recording = True
        cpu0 = nm.cpu_snapshot(gz_pids)
        wall0, t0 = time.time(), self._now()

        nav_status = None
        send_future = self._nav_client.send_goal_async(nav_goal, feedback_callback=_feedback)
        if self._wait_future(send_future, 10.0) and send_future.result() is not None \
                and send_future.result().accepted:
            handle = send_future.result()
            result_future = handle.get_result_async()
            timeout = float(self.get_parameter("goal_timeout").value)
            while not result_future.done():
                if self._now() - t0 > timeout:
                    self.get_logger().warn(f"Goal {goal['id']}: tiempo agotado, cancelando")
                    self._wait_future(handle.cancel_goal_async(), 5.0)
                    self._wait_future(result_future, 5.0)
                    break
                time.sleep(0.05)
            if result_future.done() and result_future.result() is not None:
                nav_status = int(result_future.result().status)
        else:
            self.get_logger().error(f"Nav2 no aceptó el goal {goal['id']}")

        mission_time = self._now() - t0
        wall = time.time() - wall0
        cpu1 = nm.cpu_snapshot(gz_pids)
        self._cmd_vel_pub.publish(Twist())
        self._sleep_sim(0.5)                 # que el robot termine de detenerse
        with self._lock:
            self._recording = False
            traj = np.array(self._traj, dtype=float).reshape(-1, 10)
            plans = list(self._plans)
            collisions = list(self._collisions)
            map_msg = self._map_msg

        gt = self._gt_pose()
        pos_err = math.hypot(gx - gt[0], gy - gt[1])
        yaw_err = abs(float(nm.wrap(gyaw - gt[2])))
        within = (pos_err <= float(self.get_parameter("pos_tol").value)
                  and yaw_err <= float(self.get_parameter("yaw_tol").value))
        if nav_status != GoalStatus.STATUS_SUCCEEDED:
            fail = "nav2_fallo" if nav_status is not None else "nav2_timeout_o_rechazo"
        elif collisions:
            fail = "colision"
        elif not within:
            fail = "fuera_de_tolerancia"
        else:
            fail = None
        success = fail is None

        sim_dt = max(mission_time, 1e-6)
        ep = {
            "run": run, "goal_id": goal["id"], "valid": True,
            "success": success, "fail_reason": fail, "nav_status": nav_status,
            "mission_time": mission_time, "wall_time": wall,
            "rtf": mission_time / wall if wall > 0 else None,
            "cpu_gazebo_per_sim_s": (cpu1[0] - cpu0[0]) / sim_dt if gz_pids else None,
            "cpu_total_per_sim_s": (cpu1[1] - cpu0[1]) / sim_dt,
            "shortest_path_length": goal["shortest_path_length"],
            "final_pos_error": pos_err, "final_yaw_error": yaw_err,
            "within_tolerance": within,
            "recoveries": recoveries[0], "collisions": len(collisions),
            "collision_events": collisions, "replans": max(0, len(plans) - 1),
            "T_world_map": list(T_world_map), "T_world_odom": list(T_world_odom),
        }
        ep.update(nm.episode_metrics(traj, plans, (gx, gy, gyaw), T_world_map,
                                     T_world_odom, goal["shortest_path_length"], success))

        name = f"run{run:02d}_goal{goal['id']}"
        ep["files"] = {"trajectory": f"traj/{name}.csv", "plans": f"traj/{name}_plans.csv"}
        nm.write_traj_csv(os.path.join(out_dir, ep["files"]["trajectory"]), traj)
        nm.write_plans_csv(os.path.join(out_dir, ep["files"]["plans"]), plans)

        ep.update({"map_iou": None, "map_iou_tol1": None, "map_wall_error_cm": None})
        if self._loc == "slam" and map_msg is not None:
            info = map_msg.info
            origin = (info.origin.position.x, info.origin.position.y,
                      _yaw_from_quat(info.origin.orientation))
            ep.update(nm.map_quality(self._ref, map_msg.data, info.width, info.height,
                                     info.resolution, origin, T_world_map))
            ep["files"]["slam_map"] = f"slam_maps/{name}.yaml"
            self._save_occupancy(map_msg, os.path.join(out_dir, "slam_maps"), name)

        self.get_logger().info(
            f"Run {run} goal {goal['id']}: {'ÉXITO' if success else 'FALLO (' + fail + ')'} "
            f"t={mission_time:.1f}s L={ep['path_length'] or 0:.2f}m SPL={ep['spl']:.2f} "
            f"err={pos_err:.3f}m/{yaw_err:.3f}rad CTE={_fmt(ep['cte_rmse'])} "
            f"rec={recoveries[0]} col={len(collisions)}")
        return ep

    @staticmethod
    def _save_occupancy(msg, folder, name):
        """Guarda un OccupancyGrid como pgm + yaml (formato map_server, trinario)."""
        os.makedirs(folder, exist_ok=True)
        info = msg.info
        data = np.array(msg.data, dtype=np.int16).reshape(info.height, info.width)
        img = np.full(data.shape, 205, dtype=np.uint8)
        img[(data >= 0) & (data <= 25)] = 254
        img[data >= 65] = 0
        with open(os.path.join(folder, f"{name}.pgm"), "wb") as f:
            f.write(f"P5\n{info.width} {info.height}\n255\n".encode())
            f.write(np.flipud(img).tobytes())
        with open(os.path.join(folder, f"{name}.yaml"), "w") as f:
            f.write(f"image: {name}.pgm\nmode: trinary\nresolution: {info.resolution}\n"
                    f"origin: [{info.origin.position.x}, {info.origin.position.y}, "
                    f"{_yaw_from_quat(info.origin.orientation)}]\n"
                    "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")

    # ── Rutina principal ──────────────────────────────────────────────────────
    def run(self):
        runs = int(self.get_parameter("runs").value)
        out_dir = self.get_parameter("output_dir").value or os.path.join(
            os.getcwd(), "results", time.strftime("%Y%m%d_%H%M%S") + f"_{self._loc}")
        for sub in ("traj", "reference_map"):
            os.makedirs(os.path.join(out_dir, sub), exist_ok=True)
        for fn in ("arena_map.yaml", "arena_map.pgm"):
            shutil.copy(os.path.join(os.path.dirname(self._map_yaml), fn),
                        os.path.join(out_dir, "reference_map", fn))

        t0 = time.time()
        while self._gt_pose() is None and time.time() - t0 < 15.0:
            time.sleep(0.1)
        if self._gt_pose() is None:
            self.get_logger().error(
                "Sin verdad de terreno en /ground_truth/odom: no se pueden medir "
                "las corridas. Revisa el plugin OdometryPublisher y el bridge.")
            return
        while not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Esperando Nav2...")

        gains = None
        if self.get_parameter("set_gains").value:
            gains = {k: float(self.get_parameter(k).value) for k in ("kp", "ki", "kd")}
            self.set_pid(gains["kp"], gains["ki"], gains["kd"])

        gz_pids = nm.gazebo_pids()
        if not gz_pids:
            self.get_logger().warn("No se encontraron procesos de Gazebo: no se medirá su CPU")

        config = {
            "simulator": "gazebo", "world": self._world, "localization": self._loc,
            "runs": runs, "pos_tol": float(self.get_parameter("pos_tol").value),
            "yaw_tol": float(self.get_parameter("yaw_tol").value),
            "goal_timeout": float(self.get_parameter("goal_timeout").value),
            "pid_gains": gains, "spawn": dict(zip(("x", "y", "yaw"), self._spawn)),
            "goals": self._goals, "reference_map": "reference_map/arena_map.yaml",
            "gazebo_pids": gz_pids, "started_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        episodes = []
        for run in range(1, runs + 1):
            for goal in self._goals:
                self.get_logger().info(f"── Corrida {run}/{runs} — goal {goal['id']} ──")
                align = self.reset_robot()
                if align is None:
                    episodes.append({"run": run, "goal_id": goal["id"], "valid": False,
                                     "success": False, "fail_reason": "reset_fallido"})
                else:
                    episodes.append(self.run_episode(run, goal, *align, out_dir, gz_pids))
                self._save(out_dir, config, episodes)

        self._save(out_dir, config, episodes)
        if self.get_parameter("make_plots").value:
            try:
                for p in nm.make_plots(out_dir, self._ref):
                    self.get_logger().info(f"Gráfica: {p}")
            except ImportError:
                self.get_logger().warn("matplotlib no disponible: se omiten las gráficas")
        o = nm.summarize(episodes)["overall"]
        self.get_logger().info(
            f"Terminado: {o['n_success']}/{o['n_episodes']} episodios exitosos "
            f"(Wilson 95 % {o['wilson_95']}). Resultados en {out_dir}")

    @staticmethod
    def _save(out_dir, config, episodes):
        with open(os.path.join(out_dir, "results.json"), "w") as f:
            json.dump({"config": config, "summary": nm.summarize(episodes),
                       "episodes": episodes}, f, indent=2)
        nm.write_episodes_csv(os.path.join(out_dir, "episodes.csv"),
                              [e for e in episodes if e.get("valid", True)])


def _fmt(v):
    return "—" if v is None else f"{v:.3f}m"


def main(args=None):
    rclpy.init(args=args)
    node = NavExperiment()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    while node._now() == 0.0:      # espera al reloj de simulación
        time.sleep(0.05)

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
