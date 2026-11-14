#!/usr/bin/env python3
"""
mujoco_bridge_node.py

Puente ROS 2 <-> MuJoCo. Ofrece las mismas interfaces que el Gazebo actual
(gz_ros2_control + ros_gz_bridge) para que ag_motion_tests.py, zn_tuner.py,
mecanum_kinematic_node.py y Mecanum_odometry_node.py corran SIN cambios.

PUBLICA:
  /clock                 (rosgraph_msgs/Clock)    tiempo de simulación
  /joint_states          (sensor_msgs/JointState) ruedas, brazo y gripper
  /ground_truth/odom     (nav_msgs/Odometry)      pose real del chasis (nuevo)

SUSCRIBE:
  /wheel_velocity_controller/commands           (std_msgs/Float64MultiArray)
        [fl, fr, br, bl] en rad/s -> data.ctrl de las ruedas
  /dofbot_trajectory_controller/joint_trajectory (trajectory_msgs/JointTrajectory)
  /dofbot_gripper_controller/joint_trajectory    (trajectory_msgs/JointTrajectory)

SERVICIOS:
  /world/<world_name>/set_entity_pose (ros_gz_interfaces/SetEntityPose)
        Teletransporta el chasis: fija qpos, anula qvel y llama mj_forward.
  ~/reset (std_srvs/Empty)  Estado inicial completo (pose del AG, brazo en cero).

PARÁMETROS:
  model_path        : MJCF a cargar ('' = models/omni_dofbot_arena.xml)
  world_name        : nombre del mundo en el servicio   (default: arena_pid_tuning)
  robot_name        : entidad aceptada en el servicio   (default: omni_dofbot)
  real_time_factor  : 1.0 = tiempo real (obligatorio mientras el AG use
                      time.time()); <= 0 = tan rápido como se pueda
  state_rate        : Hz de /joint_states y /ground_truth/odom (default: 100)
  clock_decimation  : publicar /clock cada N pasos de física  (default: 1)
  viewer            : abrir el visor de MuJoCo                (default: false)
"""
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory

from omni_dofbot_mujoco.model_builder import models_dir
from omni_dofbot_mujoco.sim import MujocoSim

try:
    from ros_gz_interfaces.srv import SetEntityPose
except ImportError:          # el AG lo necesita; el puente arranca igual sin él
    SetEntityPose = None


class MujocoBridge(Node):

    def __init__(self):
        super().__init__('mujoco_bridge_node')
        self.declare_parameter('model_path', '')
        self.declare_parameter('world_name', 'arena_pid_tuning')
        self.declare_parameter('robot_name', 'omni_dofbot')
        self.declare_parameter('real_time_factor', 1.0)
        self.declare_parameter('state_rate', 100.0)
        self.declare_parameter('clock_decimation', 1)
        self.declare_parameter('viewer', False)

        path = self.get_parameter('model_path').value or os.path.join(
            models_dir(), 'omni_dofbot_arena.xml')
        self.sim = MujocoSim(model=path)
        self._lock = threading.Lock()
        self._rtf = float(self.get_parameter('real_time_factor').value)
        self._state_dt = 1.0 / float(self.get_parameter('state_rate').value)
        self._clock_dec = max(1, int(self.get_parameter('clock_decimation').value))
        self._robot = self.get_parameter('robot_name').value

        self._clock_pub = self.create_publisher(Clock, '/clock', 10)
        self._js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self._gt_pub = self.create_publisher(Odometry, '/ground_truth/odom', 10)

        self.create_subscription(Float64MultiArray,
                                 '/wheel_velocity_controller/commands',
                                 self._wheel_cb, 10)
        self.create_subscription(JointTrajectory,
                                 '/dofbot_trajectory_controller/joint_trajectory',
                                 self._traj_cb, 10)
        self.create_subscription(JointTrajectory,
                                 '/dofbot_gripper_controller/joint_trajectory',
                                 self._traj_cb, 10)

        world = self.get_parameter('world_name').value
        if SetEntityPose is not None:
            self.create_service(SetEntityPose, f'/world/{world}/set_entity_pose',
                                self._set_pose_cb)
        else:
            self.get_logger().warn(
                'ros_gz_interfaces no está instalado: no se ofrece set_entity_pose '
                '(sudo apt install ros-$ROS_DISTRO-ros-gz-interfaces)')
        self.create_service(Empty, '~/reset', self._reset_cb)

        self._running = True
        self._thread = threading.Thread(target=self._sim_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f'MuJoCo listo: {path} | dt={self.sim.model.opt.timestep} s | '
            f'RTF objetivo={self._rtf}')

    # ── Callbacks ROS ─────────────────────────────────────────────────────────
    def _wheel_cb(self, msg):
        if len(msg.data) != 4:
            self.get_logger().warn('Se esperaban 4 velocidades [fl, fr, br, bl]',
                                   throttle_duration_sec=2.0)
            return
        with self._lock:
            self.sim.set_wheel_commands(list(msg.data))

    def _traj_cb(self, msg):
        pts = [(p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
                list(p.positions)) for p in msg.points]
        if pts:
            with self._lock:
                self.sim.set_trajectory(list(msg.joint_names), pts)

    def _set_pose_cb(self, req, res):
        if req.entity.name and req.entity.name != self._robot:
            res.success = False
            return res
        p, q = req.pose.position, req.pose.orientation
        with self._lock:
            self.sim.teleport(p.x, p.y, p.z, (q.w, q.x, q.y, q.z))
            self._resync()
        res.success = True
        return res

    def _reset_cb(self, _req, res):
        with self._lock:
            t = self.sim.time
            self.sim.reset()
            self.sim.data.time = t       # el reloj de simulación nunca retrocede
            self._resync()
        return res

    # ── Lazo de simulación ────────────────────────────────────────────────────
    def _resync(self):
        self._wall0, self._sim0 = time.perf_counter(), self.sim.time

    def _stamp(self):
        return Time(nanoseconds=int(self.sim.time * 1e9)).to_msg()

    def _publish_state(self):
        stamp = self._stamp()
        names, q, v = self.sim.joint_state()
        js = JointState()
        js.header.stamp = stamp
        js.name, js.position, js.velocity = names, q.tolist(), v.tolist()
        pos, quat, lin, ang = self.sim.ground_truth()
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = 'world'
        od.child_frame_id = 'base_footprint'
        pp, oo = od.pose.pose.position, od.pose.pose.orientation
        pp.x, pp.y, pp.z = pos.tolist()
        oo.w, oo.x, oo.y, oo.z = quat.tolist()
        tl, ta = od.twist.twist.linear, od.twist.twist.angular
        tl.x, tl.y, tl.z = lin.tolist()
        ta.x, ta.y, ta.z = ang.tolist()
        return js, od

    def _sim_loop(self):
        viewer = None
        if self.get_parameter('viewer').value:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(self.sim.model, self.sim.data)
        self._resync()
        next_state, next_view, steps, rtf_t = 0.0, 0.0, 0, time.perf_counter()
        rtf_sim = self.sim.time
        while self._running and rclpy.ok():
            out = []
            with self._lock:
                if self._rtf > 0:
                    target = self._sim0 + (time.perf_counter() - self._wall0) * self._rtf
                    if target - self.sim.time > 0.1:      # atraso: no acumular
                        self._resync()
                        target = self.sim.time + self.sim.model.opt.timestep
                else:
                    target = self.sim.time + 0.01
                n = 0
                while self.sim.time < target and n < 100:
                    self.sim.step()
                    n += 1
                    steps += 1
                    if steps % self._clock_dec == 0:
                        out.append(Clock(clock=self._stamp()))
                    if self.sim.time >= next_state:
                        next_state = self.sim.time + self._state_dt - 1e-9
                        out.extend(self._publish_state())
                if viewer is not None and time.perf_counter() >= next_view:
                    viewer.sync()
                    next_view = time.perf_counter() + 1 / 30
            for msg in out:
                if isinstance(msg, Clock):
                    self._clock_pub.publish(msg)
                elif isinstance(msg, JointState):
                    self._js_pub.publish(msg)
                else:
                    self._gt_pub.publish(msg)
            now = time.perf_counter()
            if now - rtf_t > 10.0:
                self.get_logger().info(
                    f'Factor de tiempo real medido: '
                    f'{(self.sim.time - rtf_sim) / (now - rtf_t):.2f}x')
                rtf_t, rtf_sim = now, self.sim.time
            if self._rtf > 0:
                time.sleep(0.0005)
        if viewer is not None:
            viewer.close()

    def destroy_node(self):
        self._running = False
        self._thread.join(timeout=1.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = MujocoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
