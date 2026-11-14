#!/usr/bin/env python3
"""Pasada de reconocimiento sin navegación (guía, sección 4).

Para cada vista de views.yaml:
  1. teletransporta al robot (servicio set_pose de Gazebo; `gz service` de respaldo),
  2. espera a que se asiente (tiempo de simulación),
  3. lee la pose REAL del robot de Gazebo (nunca de la odometría),
  4. captura imagen, profundidad y camera_info posteriores al asentamiento,
  5. guarda T_base_cam de TF, y
  6. (modo en línea, opcional) republica la captura en /vla/input/* y espera un
     vision_msgs/Detection3DArray en /vla/estimates con el mismo sello de tiempo.

Escribe en el directorio del ensayo `views.yaml` y `views/`. En modo en línea
escribe además `estimates_<nombre>.yaml` y los CSV de errores.

Uso (con la simulación lanzada sobre el mundo del MISMO ensayo):
  ros2 run vla_pose_eval recon_pass --ros-args -p trial_dir:=/ruta/S1_seed0000
"""

import math
import os
import subprocess
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration as RclDuration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_ros

from .. import dataset, evaluate, geometry as g
from ..config import dump_yaml, load_config
from ..views import load_views
from . import ros_utils as ru


class ReconPass(Node):

    def __init__(self):
        super().__init__('recon_pass', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.declare_parameter('trial_dir', '')
        self.declare_parameter('config_dir', '')
        self.declare_parameter('online', False)
        self.declare_parameter('estimator_name', 'online')
        self.trial = self.get_parameter('trial_dir').value
        if not self.trial or not os.path.isfile(dataset.gt_path(self.trial)):
            raise RuntimeError('trial_dir debe apuntar a un ensayo con scene_gt.yaml')
        self.cfg = load_config(self.get_parameter('config_dir').value or None)
        self.ev = self.cfg['eval']
        self.online = bool(self.get_parameter('online').value)
        t = self.ev['topics']

        self._lock = threading.Lock()
        self._rgb = self._depth = self._info = self._robot_pose = None
        self._estimates = {}
        self.create_subscription(Image, t['rgb'], self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, t['depth'], self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, t['camera_info'], self._on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, t['robot_pose_gt'], self._on_pose, 10)
        self._arm_pub = self.create_publisher(JointTrajectory, t['arm_trajectory'], 10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        from ros_gz_interfaces.srv import SetEntityPose
        self._SetEntityPose = SetEntityPose
        self._tp_cli = self.create_client(
            SetEntityPose, '/world/%s/set_pose' % self.ev['world_name'])

        if self.online:
            from vision_msgs.msg import Detection3DArray
            self._in_rgb = self.create_publisher(Image, t['input_rgb'], 1)
            self._in_depth = self.create_publisher(Image, t['input_depth'], 1)
            self._in_info = self.create_publisher(CameraInfo, t['input_camera_info'], 1)
            self.create_subscription(Detection3DArray, t['estimates'], self._on_estimates, 10)

    # ── Callbacks ────────────────────────────────────────────────────────────
    def _on_rgb(self, m):
        with self._lock:
            self._rgb = m

    def _on_depth(self, m):
        with self._lock:
            self._depth = m

    def _on_info(self, m):
        with self._lock:
            self._info = m

    def _on_pose(self, m):
        with self._lock:
            self._robot_pose = m

    def _on_estimates(self, m):
        with self._lock:
            self._estimates[(m.header.stamp.sec, m.header.stamp.nanosec)] = (m, time.time())

    # ── Utilidades ───────────────────────────────────────────────────────────
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _sleep_sim(self, seconds, wall_timeout=60.0):
        t0, w0 = self._now(), time.time()
        while self._now() - t0 < seconds and time.time() - w0 < wall_timeout:
            time.sleep(0.02)

    def _arm_to_observation(self):
        arm = self.ev['arm']
        msg = JointTrajectory()
        msg.joint_names = list(arm['joints'])
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in arm['observation_pose']]
        sec = float(arm['move_time_s'])
        pt.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
        msg.points.append(pt)
        for _ in range(3):               # el controlador puede tardar en suscribirse
            self._arm_pub.publish(msg)
            time.sleep(0.2)
        self._sleep_sim(sec + float(arm['settle_time_s']))

    def _teleport(self, view):
        x, y, yaw = view['x'], view['y'], view['yaw']
        z = float(self.ev['recon']['teleport_z'])
        q = g.yaw_to_quat(yaw)
        done = False
        if self._tp_cli.wait_for_service(timeout_sec=2.0):
            req = self._SetEntityPose.Request()
            req.entity.name = self.ev['robot_name']
            req.entity.type = req.entity.MODEL
            req.pose.position.x, req.pose.position.y, req.pose.position.z = x, y, z
            req.pose.orientation.z, req.pose.orientation.w = q[2], q[3]
            fut = self._tp_cli.call_async(req)
            t0 = time.time()
            while not fut.done() and time.time() - t0 < 3.0:
                time.sleep(0.02)
            done = fut.done() and fut.result() is not None and fut.result().success
        if not done:                     # respaldo: igual que ag_motion_tests.py
            req = ('name: "%s", position: {x: %f, y: %f, z: %f}, '
                   'orientation: {x: 0, y: 0, z: %f, w: %f}'
                   % (self.ev['robot_name'], x, y, z, q[2], q[3]))
            subprocess.run(
                ['gz', 'service', '-s', '/world/%s/set_pose' % self.ev['world_name'],
                 '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '2000', '--req', req],
                check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def _capture(self, after):
        """Espera RGB, profundidad y camera_info con sello posterior a `after`."""
        w0 = time.time()
        while time.time() - w0 < float(self.ev['recon']['capture_timeout_s']):
            with self._lock:
                rgb, depth, info = self._rgb, self._depth, self._info
            if (rgb is not None and depth is not None and info is not None
                    and ru.stamp_to_sec(rgb.header.stamp) > after
                    and ru.stamp_to_sec(depth.header.stamp) > after):
                return rgb, depth, info
            time.sleep(0.02)
        raise RuntimeError('no llegaron imágenes nuevas: revisar el puente de la cámara')

    def _robot_T(self, view, after):
        """T_map_base real (Gazebo). Si no hay tópico, usa la pose ordenada y avisa."""
        w0 = time.time()
        while time.time() - w0 < 2.0:
            with self._lock:
                m = self._robot_pose
            if m is not None and ru.stamp_to_sec(m.header.stamp) > after:
                return ru.pose_to_T(m.pose), 'gazebo'
            time.sleep(0.02)
        self.get_logger().warn(
            'Sin pose real del robot en %s: se usa la pose ORDENADA. El error de '
            'asentamiento entrará en la medición.' % self.ev['topics']['robot_pose_gt'])
        return g.pose2d_to_T(view['x'], view['y'], view['yaw']), 'commanded'

    def _T_base_cam(self):
        f = self.ev['frames']
        tf = self._tf_buffer.lookup_transform(
            f['base'], f['camera_optical'], Time(), timeout=RclDuration(seconds=5.0))
        return ru.transform_to_T(tf.transform)

    def _ask_estimator(self, rgb, depth, info):
        key = (rgb.header.stamp.sec, rgb.header.stamp.nanosec)
        depth.header.stamp = info.header.stamp = rgb.header.stamp
        t0 = time.time()
        self._in_info.publish(info)
        self._in_depth.publish(depth)
        self._in_rgb.publish(rgb)
        while time.time() - t0 < float(self.ev['recon']['estimate_timeout_s']):
            with self._lock:
                hit = self._estimates.pop(key, None)
            if hit is not None:
                msg, t1 = hit
                dets = []
                for d in msg.detections:
                    if not d.results:
                        continue
                    best = max(d.results, key=lambda r: r.hypothesis.score)
                    p, q = d.bbox.center.position, d.bbox.center.orientation
                    quat = [q.x, q.y, q.z, q.w]
                    dets.append({'class': best.hypothesis.class_id,
                                 'conf': float(best.hypothesis.score),
                                 'position': [p.x, p.y, p.z],
                                 # cuaternión nulo = el estimador no da orientación
                                 'orientation_xyzw': quat if any(quat) else None})
                # Incluye transporte ROS; el estimador debe medir aparte su inferencia.
                return {'infer_ms': (t1 - t0) * 1000.0, 'detections': dets}
            time.sleep(0.01)
        self.get_logger().warn('El estimador no respondió a tiempo: vista sin estimaciones')
        return {'infer_ms': None, 'detections': []}

    # ── Pasada ───────────────────────────────────────────────────────────────
    def run(self):
        log = self.get_logger()
        gt = dataset.load_gt(self.trial)
        view_list = load_views(self.cfg['views'])
        os.makedirs(os.path.join(self.trial, 'views'), exist_ok=True)
        log.info('Brazo a pose de observación…')
        self._arm_to_observation()
        T_base_cam = None
        metas, est_views = [], []
        for i, view in enumerate(view_list):
            self._teleport(view)
            self._sleep_sim(float(self.ev['recon']['settle_time_s']))
            after = self._now()
            rgb, depth, info = self._capture(after)
            T_map_base, source = self._robot_T(view, after - 0.5)
            if T_base_cam is None:       # fijo: el brazo no se mueve durante la pasada
                T_base_cam = self._T_base_cam()
            stem = os.path.join(self.trial, 'views', 'view_%03d' % i)
            rgb_file = dataset.save_rgb(stem + '_rgb', ru.image_to_rgb(rgb))
            np.save(stem + '_depth.npy', ru.image_to_depth(depth))
            metas.append({
                'view': i, 'stamp': ru.stamp_to_sec(rgb.header.stamp),
                'robot_pose_cmd': {k: float(view[k]) for k in ('x', 'y', 'yaw')},
                'robot_pose_gt': {'x': float(T_map_base[0, 3]), 'y': float(T_map_base[1, 3]),
                                  'z': float(T_map_base[2, 3]),
                                  'yaw': float(g.yaw_from_R(T_map_base[:3, :3]))},
                'robot_pose_source': source,
                'T_map_base': T_map_base.ravel().tolist(),
                'T_base_cam': T_base_cam.ravel().tolist(),
                'K': [float(v) for v in info.k], 'width': int(info.width),
                'height': int(info.height),
                'rgb': rgb_file, 'depth': os.path.basename(stem + '_depth.npy')})
            if self.online:
                est_views.append(dict(self._ask_estimator(rgb, depth, info), view=i))
            log.info('Vista %d/%d (%.2f, %.2f, %.0f°) [%s]' % (
                i + 1, len(view_list), view['x'], view['y'], math.degrees(view['yaw']), source))
        dataset.save_views_meta(self.trial, {
            'scene_id': gt['scene_id'], 'source': 'gazebo', 'frames': dict(self.ev['frames']),
            'views': metas})
        log.info('Vistas guardadas en %s' % self.trial)
        if self.online:
            name = self.get_parameter('estimator_name').value
            dump_yaml({'estimator': name, 'frame': 'camera_optical', 'views': est_views},
                      dataset.estimates_path(self.trial, name))
            evaluate.evaluate_trial_dir(self.trial, name, self.cfg)
            log.info('CSV de errores: %s' % dataset.errors_path(self.trial, name))


def main():
    rclpy.init()
    node = ReconPass()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        node.run()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
