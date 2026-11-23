#!/usr/bin/env python3
"""Ruido de profundidad realista para la Astra simulada.

Se intercala entre el puente de Gazebo y los consumidores:

    gz /camera/depth_image -> (ros_gz_bridge) /sim/camera/depth_image
        -> depth_noise -> /camera/depth_image  (32FC1, metros, NaN = sin dato)

El modelo (vla_pose_eval/noise.py, parámetros en config/noise.yaml) añade lo que
Gazebo no modela: desviación que crece con z^2 y en parte correlacionada,
escalones por cuantización de disparidad, milímetros, huecos aleatorios y en
bordes. El ruido de cada imagen depende de (seed, sello de tiempo): dos
ejecuciones con los mismos sellos producen el mismo ruido.

Publica su configuración en /camera/depth_noise/config (std_msgs/String, JSON,
latched) para que recon_pass la guarde con cada ensayo.

  ros2 run vla_pose_eval depth_noise --ros-args -p use_sim_time:=true
"""

import json

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ..config import load_config
from ..noise import DepthNoiseModel
from . import ros_utils as ru


class DepthNoiseNode(Node):

    def __init__(self):
        super().__init__('depth_noise')
        self.declare_parameter('config_dir', '')
        self.declare_parameter('input', '/sim/camera/depth_image')
        self.declare_parameter('enabled', True)
        cfg = load_config(self.get_parameter('config_dir').value or None)
        d = dict(cfg['noise'].get('depth', {}))
        d['enabled'] = bool(d.get('enabled', True)) and bool(self.get_parameter('enabled').value)
        self.model = DepthNoiseModel.from_config(d)
        t = cfg['eval']['topics']
        self._pub = self.create_publisher(Image, t['depth'], qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('input').value, self._on_depth,
                                 qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cfg_pub = self.create_publisher(String, t['depth_noise_config'], latched)
        cfg_pub.publish(String(data=json.dumps(self.model.to_dict())))
        self._min_dt = 1.0 / self.model.max_rate_hz if self.model.max_rate_hz > 0 else 0.0
        self._last = None
        self.get_logger().info('Ruido de profundidad %s: sigma(1 m) = %.1f mm, sigma(2 m) = %.1f mm'
                               % ('ACTIVO' if self.model.enabled else 'desactivado',
                                  1000 * self.model.sigma(1.0), 1000 * self.model.sigma(2.0)))

    def _on_depth(self, msg):
        t = ru.stamp_to_sec(msg.header.stamp)
        if self._min_dt and self._last is not None and 0.0 <= t - self._last < self._min_dt:
            return
        self._last = t
        depth = ru.image_to_depth(msg)
        rng = self.model.rng_for(msg.header.stamp.sec, msg.header.stamp.nanosec)
        out = np.ascontiguousarray(self.model.apply(depth, rng), dtype=np.float32)
        o = Image()
        o.header = msg.header
        o.height, o.width = out.shape
        o.encoding = '32FC1'
        o.is_bigendian = 0
        o.step = o.width * 4
        o.data = out.tobytes()
        self._pub.publish(o)


def main():
    rclpy.init()
    node = DepthNoiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
