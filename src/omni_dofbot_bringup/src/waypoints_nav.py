#!/usr/bin/env python3
"""
waypoints_nav.py
================
Ejecutor de corridas de navegación para las métricas del Artículo 1.

Por cada corrida:
  1. Teletransporta el robot al WP1 (spawn) en Gazebo.
  2. Reinicia odometría, estado del PID, pose de AMCL y costmaps.
  3. Envía WP2..WPn a Nav2 UNO POR UNO (NavigateToPose), de modo que el robot
     se detiene en cada waypoint y la tolerancia se puede verificar.
  4. En cada waypoint compara la pose REAL del simulador (/ground_truth/odom,
     marco del mundo) con el waypoint: tolerancia pos_tol / yaw_tol.
  5. Cuenta colisiones (sensores de contacto de paredes, cajas y pedestal) y
     recuperaciones de Nav2, mide tiempo de misión (simulado), longitud de
     trayectoria real y factor de tiempo real.

Una corrida es EXITOSA si Nav2 completa todos los waypoints, la pose real
queda dentro de tolerancia en cada uno y no hubo ninguna colisión.

Salidas (en output_dir, por defecto ~/x3_plus_results/<fecha>/):
  results.json        resumen + una entrada por corrida (tasa de éxito con
                      intervalo de Wilson 95 %)
  run_XX_traj.csv     trayectoria a 20 Hz: verdad de terreno (mundo),
                      odometría (marco odom) y pose estimada map->base_footprint

Requiere que el marco `map` coincida con el mundo de Gazebo (mapa generado por
maps/generate_map.py).

EJECUCIÓN:
  # T1 — simulación
  ros2 launch omni_dofbot_bringup omni_dofbot_controller.launch.py
  # T2 — cinemática + PID
  ros2 run omni_dofbot_bringup mecanum_kinematic_node.py
  # T3 — odometría
  ros2 run omni_dofbot_bringup Mecanum_odometry_node.py
  # T4 — Nav2
  ros2 launch omni_dofbot_bringup nav2_arena.launch.py
  # T5 — este nodo
  ros2 run omni_dofbot_bringup waypoints_nav.py --ros-args -p runs:=20
"""

import os
import csv
import json
import math
import time
import threading
import subprocess

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter as RclpyParameter
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from geometry_msgs.msg import (
    PoseStamped, PoseWithCovarianceStamped, Twist, Pose, Point, Quaternion)
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
from std_srvs.srv import Empty

from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity, Contacts

import tf2_ros

WORLD_NAME = "arena_world"
ROBOT_NAME = "omni_dofbot"
# Modelos con sensor de contacto en arena_world.world (tópicos /contacts/<modelo>)
CONTACT_MODELS = ("wall_north", "wall_south", "wall_east", "wall_west",
                  "obstacle_1", "obstacle_2", "pedestal")
# mecanum_odometry_node se reinicia a (0, 0, pi/2); el teleport y la pose
# inicial de AMCL usan el mismo yaw para que odom, AMCL y Gazebo coincidan.
RESET_YAW = math.pi / 2.0
CONTACT_GAP = 0.5        # s simulados sin contacto para contar una colisión nueva
TRAJ_PERIOD = 0.05       # s simulados entre muestras de trayectoria (20 Hz)


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _ang_diff(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def wilson_interval(successes: int, n: int, z: float = 1.96):
    """Intervalo de Wilson para una proporción (95 % con z = 1.96)."""
    if n == 0:
        return float("nan"), float("nan")
    p = successes / n
    den = 1.0 + z * z / n
    centre = (p + z * z / (2.0 * n)) / den
    half = z * math.sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n)) / den
    return max(0.0, centre - half), min(1.0, centre + half)


class WaypointRunner(Node):

    def __init__(self):
        super().__init__(
            "waypoint_runner",
            parameter_overrides=[
                RclpyParameter("use_sim_time", RclpyParameter.Type.BOOL, True)])
        self._cbg = ReentrantCallbackGroup()

        # ── Parámetros ────────────────────────────────────────────────────────
        self.declare_parameter("runs", 20)            # N >= 20 por simulador
        self.declare_parameter("pos_tol", 0.08)       # m, contra verdad de terreno
        self.declare_parameter("yaw_tol", 0.15)       # rad, contra verdad de terreno
        self.declare_parameter("goal_timeout", 60.0)  # s simulados por waypoint
        self.declare_parameter("output_dir", "")
        self.declare_parameter("set_gains", True)    # si False, no toca el PID
        self.declare_parameter("kp", 1.949)          # Ganancia Proporcional definitiva
        self.declare_parameter("ki", 0.256)          # Ganancia Integral definitiva
        self.declare_parameter("kd", 0.938)          # Ganancia Derivativa definitiva

        # ── Clientes / publicadores ───────────────────────────────────────────
        self._pid_client = self.create_client(
            SetParameters, "/mecanum_kinematic_node/set_parameters",
            callback_group=self._cbg)
        self._nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self._cbg)
        self._tp_client = self.create_client(
            SetEntityPose, f"/world/{WORLD_NAME}/set_entity_pose",
            callback_group=self._cbg)
        # OJO: el nodo se llama 'mecanum_odometry_node' (minúsculas)
        self._reset_odom_cli = self.create_client(
            Empty, "/mecanum_odometry_node/reset_pose", callback_group=self._cbg)
        self._reset_ctrl_cli = self.create_client(
            Empty, "/mecanum_kinematic_node/reset_controller_state",
            callback_group=self._cbg)
        self._clear_costmap_clis = [
            self.create_client(ClearEntireCostmap, name, callback_group=self._cbg)
            for name in ("/global_costmap/clear_entirely_global_costmap",
                         "/local_costmap/clear_entirely_local_costmap")]
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)

        # ── Estado medido ─────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._gt = None            # (x, y, yaw) real, marco del mundo
        self._odom = None          # (x, y, yaw) odometría, marco odom
        self._recording = False
        self._traj = []
        self._path_len = 0.0
        self._last_gt_xy = None
        self._last_sample_t = -1.0
        self._collisions = []      # [{"object":..., "t":...}]
        self._last_contact_t = {}

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(
            Odometry, "/ground_truth/odom", self._gt_cb, 50,
            callback_group=self._cbg)
        self.create_subscription(
            Odometry, "/odom", self._odom_cb, 50, callback_group=self._cbg)
        for model in CONTACT_MODELS:
            self.create_subscription(
                Contacts, f"/contacts/{model}",
                lambda msg, m=model: self._contact_cb(m, msg), 10,
                callback_group=self._cbg)

        self._waypoints = self._load_waypoints()
        self.get_logger().info(
            f"WaypointRunner listo — {len(self._waypoints)} waypoints cargados")

    # ── Utilidades ────────────────────────────────────────────────────────────
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _sleep_sim(self, seconds: float):
        """Espera en tiempo SIMULADO (independiente del RTF)."""
        t_end = self._now() + seconds
        while self._now() < t_end and rclpy.ok():
            time.sleep(0.01)

    def _load_waypoints(self) -> list:
        share = get_package_share_directory("omni_dofbot_bringup")
        path = os.path.join(share, "config", "waypoints.yaml")
        self.get_logger().info(f"Buscando waypoints en: {path}")
        if not os.path.exists(path):
            self.get_logger().error(f"Archivo no encontrado: {path}")
            return []
        with open(path, "r") as f:
            return yaml.safe_load(f)["waypoints"]

    @staticmethod
    def _yaw_to_quat(yaw: float):
        return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def _call(self, client, req, timeout=3.0):
        """Llama a un servicio sin bloquear el executor."""
        fut = client.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        return fut.result() if fut.done() else None

    def _wait_future(self, fut, timeout):
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        return fut.done()

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
            if not self._recording:
                return
            if self._last_gt_xy is not None:
                self._path_len += math.hypot(x - self._last_gt_xy[0],
                                             y - self._last_gt_xy[1])
            self._last_gt_xy = (x, y)
            if t - self._last_sample_t < TRAJ_PERIOD:
                return
            self._last_sample_t = t
            odom = self._odom or (math.nan,) * 3
        est = (math.nan,) * 3
        try:   # pose estimada por la localización (AMCL o SLAM) en el marco map
            tf = self._tf_buffer.lookup_transform("map", "base_footprint", Time())
            est = (tf.transform.translation.x, tf.transform.translation.y,
                   _yaw_from_quat(tf.transform.rotation))
        except Exception:
            pass
        with self._lock:
            if self._recording:
                self._traj.append((t, x, y, yaw, *odom, *est))

    def _contact_cb(self, model: str, msg: Contacts):
        if not self._recording:
            return
        # El cubo descansa sobre el pedestal: solo cuentan contactos con el robot.
        if not any(ROBOT_NAME in c.collision1.name or ROBOT_NAME in c.collision2.name
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

    def _start_recording(self):
        with self._lock:
            self._traj = []
            self._path_len = 0.0
            self._last_gt_xy = None
            self._last_sample_t = -1.0
            self._collisions = []
            self._last_contact_t = {}
            self._recording = True

    def _stop_recording(self):
        with self._lock:
            self._recording = False
            return list(self._traj), self._path_len, list(self._collisions)

    # ── Reset del robot ───────────────────────────────────────────────────────
    def reset_robot(self) -> bool:
        """Teletransporta al WP1 y reinicia odometría, PID, AMCL y costmaps."""
        self._cmd_vel_pub.publish(Twist())
        self._sleep_sim(0.3)

        spawn = self._waypoints[0]
        x, y = float(spawn["x"]), float(spawn["y"])
        qx, qy, qz, qw = self._yaw_to_quat(RESET_YAW)
        z = 0.05

        done = False
        if self._tp_client.wait_for_service(timeout_sec=2.0):
            req = SetEntityPose.Request()
            req.entity.name = ROBOT_NAME
            req.entity.type = Entity.MODEL
            req.pose = Pose(position=Point(x=x, y=y, z=z),
                            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))
            done = self._call(self._tp_client, req) is not None

        if not done:  # fallback: gz service
            rs = (f'name: "{ROBOT_NAME}", position: {{x: {x}, y: {y}, z: {z}}}, '
                  f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}')
            try:
                subprocess.run(
                    ["gz", "service", "-s", f"/world/{WORLD_NAME}/set_pose",
                     "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                     "--timeout", "2000", "--req", rs],
                    check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            except (subprocess.CalledProcessError, FileNotFoundError) as e:
                self.get_logger().error(f"gz service falló: {e}")

        self._sleep_sim(1.0)  # el robot cae y se asienta

        for cli, name in ((self._reset_odom_cli, "reset_pose de odometría"),
                          (self._reset_ctrl_cli, "reset_controller_state")):
            if cli.wait_for_service(timeout_sec=2.0):
                self._call(cli, Empty.Request(), 2.0)
            else:
                self.get_logger().error(f"{name} no disponible")
                return False
        self._sleep_sim(0.2)

        # AMCL: misma pose que el teleport (map == mundo)
        ip = PoseWithCovarianceStamped()
        ip.header.frame_id = "map"
        ip.header.stamp = self.get_clock().now().to_msg()
        ip.pose.pose = Pose(position=Point(x=x, y=y, z=0.0),
                            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))
        ip.pose.covariance[0] = 0.01     # x
        ip.pose.covariance[7] = 0.01     # y
        ip.pose.covariance[35] = 0.01    # yaw
        self._initpose_pub.publish(ip)
        self._sleep_sim(1.0)

        for cli in self._clear_costmap_clis:
            if cli.wait_for_service(timeout_sec=2.0):
                self._call(cli, ClearEntireCostmap.Request(), 2.0)
        self._sleep_sim(1.0)   # los costmaps se vuelven a llenar con /scan

        gt = self._gt_pose()
        err = math.hypot(gt[0] - x, gt[1] - y)
        if err > 0.05 or abs(_ang_diff(gt[2], RESET_YAW)) > 0.1:
            self.get_logger().error(
                f"Reset fallido: el robot está en ({gt[0]:.3f}, {gt[1]:.3f}, "
                f"{gt[2]:.2f} rad), no en el spawn")
            return False
        self.get_logger().info("Robot reseteado en WP1")
        return True

    # ── PID (opcional) ────────────────────────────────────────────────────────
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

    # ── Navegación ────────────────────────────────────────────────────────────
    def _go_to(self, wp: dict) -> dict:
        """Navega a un waypoint y evalúa la llegada contra la verdad de terreno."""
        gx, gy, gyaw = float(wp["x"]), float(wp["y"]), float(wp["yaw"])
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        qx, qy, qz, qw = self._yaw_to_quat(gyaw)
        goal.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        recoveries = [0]

        def _feedback(msg):
            recoveries[0] = max(recoveries[0], int(msg.feedback.number_of_recoveries))

        out = {"id": wp["id"], "nav_status": None, "recoveries": 0,
               "time": 0.0, "pos_error": None, "yaw_error": None,
               "within_tolerance": False}
        t0 = self._now()
        send_future = self._nav_client.send_goal_async(goal, feedback_callback=_feedback)
        if not self._wait_future(send_future, 10.0) or send_future.result() is None \
                or not send_future.result().accepted:
            self.get_logger().error(f"Nav2 no aceptó el WP{wp['id']}")
            return out

        handle = send_future.result()
        result_future = handle.get_result_async()
        timeout = float(self.get_parameter("goal_timeout").value)
        while not result_future.done():
            if self._now() - t0 > timeout:
                self.get_logger().warn(f"WP{wp['id']}: tiempo agotado, cancelando")
                self._wait_future(handle.cancel_goal_async(), 5.0)
                self._wait_future(result_future, 5.0)
                break
            time.sleep(0.05)

        out["time"] = self._now() - t0
        out["recoveries"] = recoveries[0]
        if result_future.done() and result_future.result() is not None:
            out["nav_status"] = int(result_future.result().status)

        self._sleep_sim(0.5)   # que el robot termine de detenerse
        x, y, yaw = self._gt_pose()
        out["pos_error"] = math.hypot(gx - x, gy - y)
        out["yaw_error"] = abs(_ang_diff(gyaw, yaw))
        out["within_tolerance"] = (
            out["pos_error"] <= float(self.get_parameter("pos_tol").value)
            and out["yaw_error"] <= float(self.get_parameter("yaw_tol").value))
        self.get_logger().info(
            f"WP{wp['id']}: status={out['nav_status']} "
            f"err={out['pos_error']:.3f} m / {out['yaw_error']:.3f} rad "
            f"{'OK' if out['within_tolerance'] else 'FUERA DE TOLERANCIA'} "
            f"t={out['time']:.1f}s rec={out['recoveries']}")
        return out

    def run_once(self, index: int, out_dir: str) -> dict:
        """Una corrida completa WP2..WPn. Devuelve el registro de la corrida."""
        wall0 = time.time()
        self._start_recording()
        t_start = self._now()

        wps = []
        fail_reason = None
        for wp in self._waypoints[1:]:      # WP1 es solo el spawn
            r = self._go_to(wp)
            wps.append(r)
            if r["nav_status"] != GoalStatus.STATUS_SUCCEEDED:
                fail_reason = f"nav2_fallo_wp{wp['id']}"
                break                        # Nav2 abortó: la corrida ya falló

        mission_time = self._now() - t_start
        wall = time.time() - wall0
        traj, path_len, collisions = self._stop_recording()
        self._cmd_vel_pub.publish(Twist())

        if fail_reason is None and not all(w["within_tolerance"] for w in wps):
            fail_reason = "fuera_de_tolerancia"
        if collisions and fail_reason is None:
            fail_reason = "colision"

        traj_file = os.path.join(out_dir, f"run_{index:02d}_traj.csv")
        with open(traj_file, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "gt_x", "gt_y", "gt_yaw", "odom_x", "odom_y", "odom_yaw",
                        "est_x", "est_y", "est_yaw"])
            w.writerows(traj)

        return {
            "run": index,
            "success": fail_reason is None,
            "fail_reason": fail_reason,
            "mission_time": mission_time,          # s simulados
            "wall_time": wall,                     # s reales
            "rtf": mission_time / wall if wall > 0 else None,
            "path_length": path_len,               # m, verdad de terreno
            "recoveries": sum(w["recoveries"] for w in wps),
            "collisions": len(collisions),
            "collision_events": collisions,
            "waypoints": wps,
            "trajectory_file": os.path.basename(traj_file),
        }

    # ── Rutina principal ──────────────────────────────────────────────────────
    def run(self):
        runs = int(self.get_parameter("runs").value)
        out_dir = self.get_parameter("output_dir").value or os.path.join(
            os.path.expanduser("~"), "x3_plus_results",
            time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(out_dir, exist_ok=True)

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

        if self.get_parameter("set_gains").value:
            self.set_pid(self.get_parameter("kp").value,
                         self.get_parameter("ki").value,
                         self.get_parameter("kd").value)

        results = []
        for i in range(1, runs + 1):
            self.get_logger().info(f"── Corrida {i}/{runs} ──")
            if not self.reset_robot():
                results.append({"run": i, "success": False,
                                "fail_reason": "reset_fallido", "valid": False})
                continue
            r = self.run_once(i, out_dir)
            results.append(r)
            self.get_logger().info(
                f"Corrida {i}: {'ÉXITO' if r['success'] else 'FALLO (' + r['fail_reason'] + ')'} "
                f"t={r['mission_time']:.1f}s L={r['path_length']:.2f}m "
                f"colisiones={r['collisions']} recuperaciones={r['recoveries']}")
            self._save(out_dir, results)

        self._save(out_dir, results)
        self.get_logger().info(f"Resultados guardados en {out_dir}")

    def _save(self, out_dir: str, results: list):
        valid = [r for r in results if r.get("valid", True)]
        n = len(valid)
        k = sum(1 for r in valid if r["success"])
        lo, hi = wilson_interval(k, n)
        summary = {
            "simulator": "gazebo",
            "world": WORLD_NAME,
            "pos_tol": self.get_parameter("pos_tol").value,
            "yaw_tol": self.get_parameter("yaw_tol").value,
            # Ganancias aplicadas por este nodo; None = no se tocaron (las del nodo PID)
            "pid_gains": ({k: self.get_parameter(k).value for k in ("kp", "ki", "kd")}
                          if self.get_parameter("set_gains").value else None),
            "n_runs": n,
            "n_success": k,
            "success_rate": k / n if n else None,
            "wilson_95": [lo, hi] if n else None,
            "waypoints": [{"id": w["id"], "x": w["x"], "y": w["y"], "yaw": w["yaw"]}
                          for w in self._waypoints],
        }
        with open(os.path.join(out_dir, "results.json"), "w") as f:
            json.dump({"summary": summary, "runs": results}, f, indent=2)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRunner()

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
