#!/usr/bin/env python3
"""
waypoint_nav.py
========================
Nodo puente: lee waypoints de waypoints.yaml y los envía a Nav2 (NavigateThroughPoses) para que el robot los siga.

Secuencia de acciones:  
  1. Teletransportar el robot al WP1 (spawn) en Gazebo.
  2. Reiniciar odometría y estado del controlador.
  3. (Opcional) Fijar unas ganancias PID dadas por parámetros ROS 2.
  4. Enviar los waypoints de waypoints.yaml a Nav2 (NavigateThroughPoses)
     y esperar a que termine.
  5. Reportar tiempo, error final e ITAE de cada corrida y guardarlos en JSON.

Las ganancias ocupadas previamente se obtuieron a partir de un AG (ag_motion_test.py) que optimizó el desempeño del robot en simulación.

EJECUCIÓN:
  # T1 — simulación
  ros2 launch omni_dofbot_bringup omni_dofbot_controller.launch.py
  # T2 — cinemática + PID
  ros2 run omni_dofbot_bringup mecanum_kinematic_node.py
  # T3 — odometría
  ros2 run omni_dofbot_bringup Mecanum_odometry_node.py
  # T4 — Nav2
  ros2 launch omni_dofbot_bringup nav2_arena.launch.py
  # T5 — este nodo (parámetros opcionales)
  ros2 run omni_dofbot_bringup ag_tuner.py --ros-args \
      -p runs:=3 -p set_gains:=true -p kp:=2.734 -p ki:=0.335 -p kd:=0.756
"""

import os
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
 
from geometry_msgs.msg import PoseStamped, Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
from std_srvs.srv import Empty
 
from nav2_msgs.action import NavigateThroughPoses
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
 
WORLD_NAME = "arena_world"
ROBOT_NAME = "omni_dofbot"
NAV_TIMEOUT = 120.0      # s (pared) máximo por corrida
# mecanum_odometry_node se reinicia a yaw=pi/2 (RESET_YAW); el teleport debe
# usar el mismo yaw para que odom y Gazebo coincidan.
RESET_YAW = math.pi / 2.0
OUT_JSON = "/tmp/waypoint_run_results.json"
 
 
class WaypointRunner(Node):
 
    def __init__(self):
        super().__init__(
            "waypoint_runner",
            parameter_overrides=[
                RclpyParameter("use_sim_time", RclpyParameter.Type.BOOL, True)])
        self._cbg = ReentrantCallbackGroup()
 
        # ── Parámetros ────────────────────────────────────────────────────────
        self.declare_parameter("runs", 3)   # cuántas veces repetir la ruta
        self.declare_parameter("set_gains", False)   # si False, no toca el PID
        self.declare_parameter("kp", 1.949)
        self.declare_parameter("ki", 0.256)
        self.declare_parameter("kd", 0.938)
        self.declare_parameter("reset_between_runs", True)
 
        # ── Clientes ──────────────────────────────────────────────────────────
        self._pid_client = self.create_client(
            SetParameters, "/mecanum_kinematic_node/set_parameters",
            callback_group=self._cbg)
        self._nav_client = ActionClient(
            self, NavigateThroughPoses, "navigate_through_poses",
            callback_group=self._cbg)
        self._tp_client = self.create_client(
            SetEntityPose, f"/world/{WORLD_NAME}/set_entity_pose",
            callback_group=self._cbg)
        self._reset_odom_cli = self.create_client(
            Empty, "/Mecanum_odometry_node/reset_pose",
            callback_group=self._cbg)
        self._reset_ctrl_cli = self.create_client(
            Empty, "/mecanum_kinematic_node/reset_controller_state",
            callback_group=self._cbg)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
 
        # ── Odometría para ITAE / error final ─────────────────────────────────
        self._odom_lock = threading.Lock()
        self._x = 0.0
        self._y = 0.0
        self._measuring = False
        self._ref_x = 0.0
        self._ref_y = 0.0
        self._t0 = 0.0
        self._last_t = 0.0
        self._itae = 0.0
        self.create_subscription(
            Odometry, "/odom", self._odom_cb, 10, callback_group=self._cbg)
 
        self._waypoints = self._load_waypoints()
        self.get_logger().info(
            f"WaypointRunner listo — {len(self._waypoints)} waypoints cargados")
 
    # ── Utilidades ────────────────────────────────────────────────────────────
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9
 
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
 
    # ── Odometría / ITAE ──────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        with self._odom_lock:
            self._x = msg.pose.pose.position.x
            self._y = msg.pose.pose.position.y
            if not self._measuring:
                return
            now = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
            dt = now - self._last_t
            if dt <= 0.0:
                dt = 1e-3
            self._last_t = now
            err = math.hypot(self._ref_x - self._x, self._ref_y - self._y)
            self._itae += (now - self._t0) * err * dt
 
    def _start_measuring(self, ref_x, ref_y):
        with self._odom_lock:
            self._ref_x, self._ref_y = ref_x, ref_y
            self._itae = 0.0
            self._t0 = self._last_t = self._now()
            self._measuring = True
 
    def _stop_measuring(self):
        with self._odom_lock:
            self._measuring = False
            err_f = math.hypot(self._ref_x - self._x, self._ref_y - self._y)
            return self._itae, err_f
 
    # ── Reset del robot ───────────────────────────────────────────────────────
    def reset_robot(self):
        """Teletransporta al WP1 y reinicia odometría y controlador."""
        self._cmd_vel_pub.publish(Twist())
        time.sleep(0.2)
 
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
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"gz service falló: {e.stderr.decode()}")
 
        time.sleep(1.0)  # el robot cae y se asienta
 
        if self._reset_odom_cli.wait_for_service(timeout_sec=2.0):
            self._call(self._reset_odom_cli, Empty.Request(), 2.0)
        else:
            self.get_logger().warn("reset_pose de odometría no disponible")
        if self._reset_ctrl_cli.wait_for_service(timeout_sec=2.0):
            self._call(self._reset_ctrl_cli, Empty.Request(), 2.0)
        else:
            self.get_logger().warn("reset_controller_state no disponible")
        time.sleep(0.3)
        self.get_logger().info("Robot reseteado en WP1")
 
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
    def navigate_waypoints(self):
        """Envía WP2..WPn a Nav2. Devuelve dict con el resultado."""
        while not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Esperando Nav2...")
 
        goal = NavigateThroughPoses.Goal()
        for wp in self._waypoints[1:]:      # WP1 es solo el spawn
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(wp["x"])
            pose.pose.position.y = float(wp["y"])
            qx, qy, qz, qw = self._yaw_to_quat(float(wp["yaw"]))
            pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
            pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
            goal.poses.append(pose)
 
        last = self._waypoints[-1]
        self._start_measuring(float(last["x"]), float(last["y"]))
        t_start = self._now()
 
        send_future = self._nav_client.send_goal_async(goal)
        deadline = time.time() + 5.0
        while not send_future.done() and time.time() < deadline:
            time.sleep(0.05)
 
        if not send_future.done() or send_future.result() is None \
                or not send_future.result().accepted:
            self.get_logger().error("Nav2 no aceptó la ruta")
            self._stop_measuring()
            return {"success": False, "time": 0.0, "itae": 0.0,
                    "final_error": float("nan"), "status": None}
 
        result_future = send_future.result().get_result_async()
        deadline = time.time() + NAV_TIMEOUT
        while not result_future.done() and time.time() < deadline:
            time.sleep(0.1)
 
        elapsed = self._now() - t_start
        itae, err_f = self._stop_measuring()
 
        if not result_future.done():
            self.get_logger().warn("Nav2 timeout")
            return {"success": False, "time": elapsed, "itae": itae,
                    "final_error": err_f, "status": None}
 
        status = result_future.result().status
        ok = status == GoalStatus.STATUS_SUCCEEDED
        (self.get_logger().info if ok else self.get_logger().warn)(
            f"Ruta {'completada' if ok else 'fallida'} — status={status} "
            f"t={elapsed:.1f}s ITAE={itae:.3f} err_final={err_f:.3f}m")
        return {"success": ok, "time": elapsed, "itae": itae,
                "final_error": err_f, "status": int(status)}
 
    # ── Rutina principal ──────────────────────────────────────────────────────
    def run(self):
        runs = int(self.get_parameter("runs").value)
        results = []
 
        if self.get_parameter("set_gains").value:
            self.set_pid(self.get_parameter("kp").value,
                         self.get_parameter("ki").value,
                         self.get_parameter("kd").value)
 
        for i in range(runs):
            self.get_logger().info(f"── Corrida {i + 1}/{runs} ──")
            if self.get_parameter("reset_between_runs").value or i == 0:
                self.reset_robot()
            results.append(self.navigate_waypoints())
 
        with open(OUT_JSON, "w") as f:
            json.dump({"runs": results}, f, indent=2)
        self.get_logger().info(f"Resultados guardados en {OUT_JSON}")
 
 
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
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()