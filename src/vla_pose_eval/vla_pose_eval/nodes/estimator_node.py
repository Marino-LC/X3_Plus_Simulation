#!/usr/bin/env python3
"""Envoltura ROS 2 de un estimador (modo en línea).

Recibe en /vla/input/{image,depth,camera_info} (y, si el estimador usa la cámara
del Dofbot, /vla/input/mono/{image,camera_info}) un juego de mensajes con el
mismo sello de tiempo (lo publica recon_pass con online:=true), ejecuta el
estimador y publica vision_msgs/Detection3DArray en /vla/estimates con ese
mismo sello y el marco óptico de la Astra.

* Las extrínsecas de las cámaras (calibración) se leen de TF.
* `Detection3D.id` lleva las cámaras de origen ('rgbd', 'mono', 'rgbd+mono').
* Si el estimador no da orientación, el cuaternión va en cero (recon_pass lo
  interpreta como «sin orientación»).
* En /vla/estimates/info (latched, JSON) anuncia su nombre y sus cámaras.

  ros2 run vla_pose_eval estimator_node --ros-args \
      -p estimator:=mi_paquete.mi_modulo:MiLineaBase -p estimator_kwargs:='{}'

El oráculo no funciona en línea (necesita la verdad): úsese con
`python3 -m vla_pose_eval.cli run-estimator`.
"""

import json

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import tf2_ros
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from ..config import load_config
from ..estimators import CameraImage, Frame, load_estimator
from . import ros_utils as ru


class EstimatorNode(Node):

    def __init__(self):
        super().__init__('vla_estimator')
        self.declare_parameter('estimator', 'vla')
        self.declare_parameter('estimator_kwargs', '{}')
        self.declare_parameter('config_dir', '')
        cfg = load_config(self.get_parameter('config_dir').value or None)
        t = cfg['eval']['topics']
        self.frames = cfg['eval']['frames']
        self.optical_frame = self.frames['camera_optical']
        self.estimator = load_estimator(
            self.get_parameter('estimator').value,
            **json.loads(self.get_parameter('estimator_kwargs').value))
        self.estimator.setup(gt=None, cfg=cfg)
        self.use_mono = 'mono' in getattr(self.estimator, 'cameras', ('rgbd',))
        self.needed = {'rgb', 'depth', 'info'} | ({'mono', 'mono_info'} if self.use_mono else set())
        self.instruction = ''
        self._pending = {}
        self._count = 0
        self._T = {}
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        def sub(kind, msg_type, topic):
            self.create_subscription(msg_type, topic, lambda m: self._on(kind, m), 1)

        sub('rgb', Image, t['input_rgb'])
        sub('depth', Image, t['input_depth'])
        sub('info', CameraInfo, t['input_camera_info'])
        if self.use_mono:
            sub('mono', Image, t['input_mono_rgb'])
            sub('mono_info', CameraInfo, t['input_mono_camera_info'])
        self.create_subscription(String, t['instruction'], self._on_instruction, 1)
        self._pub = self.create_publisher(Detection3DArray, t['estimates'], 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        info_pub = self.create_publisher(String, t['estimates'] + '/info', latched)
        info_pub.publish(String(data=json.dumps(
            {'name': self.estimator.name, 'cameras': list(self.estimator.cameras)})))

    def _on_instruction(self, msg):
        self.instruction = msg.data

    def _extrinsic(self, frame_key):
        """T_base_cam de TF (fijo durante la pasada; se guarda la primera lectura)."""
        if frame_key not in self._T:
            tf = self._tf_buffer.lookup_transform(self.frames['base'], self.frames[frame_key],
                                                  Time(), timeout=Duration(seconds=2.0))
            self._T[frame_key] = ru.transform_to_T(tf.transform)
        return self._T[frame_key]

    def _on(self, kind, msg):
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        slot = self._pending.setdefault(key, {})
        slot[kind] = msg
        if not self.needed <= set(slot):
            return
        self._pending = {}
        mono = None
        if self.use_mono:
            mi = slot['mono_info']
            mono = CameraImage(name='mono', rgb=ru.image_to_rgb(slot['mono']),
                               K=np.asarray(mi.k, float).reshape(3, 3), width=int(mi.width),
                               height=int(mi.height), T_base_cam=self._extrinsic('mono_optical'))
        frame = Frame(view=self._count, rgb=ru.image_to_rgb(slot['rgb']),
                      depth=ru.image_to_depth(slot['depth']),
                      K=np.asarray(slot['info'].k, float).reshape(3, 3),
                      instruction=self.instruction,
                      T_base_cam=self._extrinsic('camera_optical'), mono=mono)
        self._count += 1
        out = Detection3DArray()
        out.header.stamp = slot['rgb'].header.stamp
        out.header.frame_id = self.optical_frame
        for d in self.estimator.estimate(frame):
            det = Detection3D()
            det.header = out.header
            det.id = d.source
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id, hyp.hypothesis.score = d.cls, float(d.conf)
            c = det.bbox.center
            c.position.x, c.position.y, c.position.z = (float(v) for v in d.position)
            q = d.orientation_xyzw or [0.0, 0.0, 0.0, 0.0]
            c.orientation.x, c.orientation.y, c.orientation.z, c.orientation.w = (
                float(v) for v in q)
            hyp.pose.pose = c
            det.results.append(hyp)
            out.detections.append(det)
        self._pub.publish(out)


def main():
    rclpy.init()
    node = EstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
