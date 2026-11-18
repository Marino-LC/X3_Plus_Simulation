#!/usr/bin/env python3
"""
scan_self_filter.py
===================
Quita del LaserScan los rayos que chocan con el PROPIO robot.

El haz del LiDAR (~0.19 m del piso) corta el mástil, el brazo y la parte alta
del chasis. Esos puntos, que se mueven con el robot:
  - dejan al collision_monitor permanentemente en "stop" (cmd_vel vacío);
  - en modo SLAM se mapean como obstáculo en la posición del robot, y el
    planificador no puede salir del spawn.

El nodo transforma cada rayo al marco base_footprint y, si el punto cae dentro
de la huella del robot ampliada por `margin`, lo reemplaza por +inf (sin
retorno). Publica el resultado en /scan_filtered, que es lo que consumen AMCL,
los costmaps, el collision_monitor y slam_toolbox.

Parámetros (huella por defecto = la de nav2_params.yaml y goals.yaml):
  input_topic  (/scan)          output_topic (/scan_filtered)
  base_frame   (base_footprint)
  x_min (-0.18)  x_max (0.15)  y_half (0.12)  margin (0.02)  [m]
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros


class ScanSelfFilter(Node):

    def __init__(self):
        super().__init__(
            'scan_self_filter',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_filtered')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('x_min', -0.18)
        self.declare_parameter('x_max', 0.15)
        self.declare_parameter('y_half', 0.12)
        self.declare_parameter('margin', 0.02)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._T = None            # (x, y, yaw) del láser en base_footprint (estático)
        self._cache = None        # (n, angle_min, angle_inc) -> cos/sin precalculados
        self._last_report = -1e9

        self._pub = self.create_publisher(
            LaserScan, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('input_topic').value, self._cb,
            qos_profile_sensor_data)
        self.get_logger().info(
            f"Filtrando {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}")

    def _laser_pose(self, frame):
        if self._T is None:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self.get_parameter('base_frame').value, frame, Time())
            except Exception:
                return None
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self._T = (tf.transform.translation.x, tf.transform.translation.y, yaw)
            self.get_logger().info(
                f"Láser '{frame}' en {self.get_parameter('base_frame').value}: "
                f"x={self._T[0]:.3f} y={self._T[1]:.3f} yaw={self._T[2]:.3f}")
        return self._T

    def _cb(self, msg: LaserScan):
        T = self._laser_pose(msg.header.frame_id)
        if T is None:
            return              # sin TF todavía: no se publica un scan sin filtrar
        n = len(msg.ranges)
        key = (n, msg.angle_min, msg.angle_increment)
        if self._cache is None or self._cache[0] != key:
            ang = msg.angle_min + np.arange(n) * msg.angle_increment + T[2]
            self._cache = (key, np.cos(ang), np.sin(ang))
        _, c, s = self._cache

        r = np.asarray(msg.ranges, dtype=np.float32)
        x = T[0] + r * c
        y = T[1] + r * s
        m = float(self.get_parameter('margin').value)
        inside = ((x >= self.get_parameter('x_min').value - m)
                  & (x <= self.get_parameter('x_max').value + m)
                  & (np.abs(y) <= self.get_parameter('y_half').value + m)
                  & np.isfinite(r))
        r[inside] = np.inf

        out = LaserScan()
        out.header = msg.header
        out.angle_min, out.angle_max = msg.angle_min, msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment, out.scan_time = msg.time_increment, msg.scan_time
        out.range_min, out.range_max = msg.range_min, msg.range_max
        out.ranges = r.tolist()
        out.intensities = msg.intensities
        self._pub.publish(out)

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_report > 10.0:
            self._last_report = now
            self.get_logger().info(
                f"{int(inside.sum())}/{n} rayos dentro de la huella del robot "
                f"(eliminados del scan)")


def main(args=None):
    rclpy.init(args=args)
    node = ScanSelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
