#!/usr/bin/env python3
"""Envoltura ROS 2 de un estimador (modo en línea).

Recibe en /vla/input/{image,depth,camera_info} una terna con el mismo sello de
tiempo (la publica recon_pass con online:=true), ejecuta el estimador y publica
vision_msgs/Detection3DArray en /vla/estimates con ese mismo sello y el marco
óptico de la cámara. Si el estimador no da orientación, deja el cuaternión en
cero (recon_pass lo interpreta como «sin orientación»).

  ros2 run vla_pose_eval estimator_node --ros-args \
      -p estimator:=mi_paquete.mi_modulo:MiLineaBase -p estimator_kwargs:='{}'

El oráculo no funciona en línea (necesita la verdad): úsese con
`python3 -m vla_pose_eval.cli run-estimator`.
"""

import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from ..config import load_config
from ..estimators import Frame, load_estimator
from . import ros_utils as ru


class EstimatorNode(Node):

    def __init__(self):
        super().__init__('vla_estimator')
        self.declare_parameter('estimator', 'vla')
        self.declare_parameter('estimator_kwargs', '{}')
        self.declare_parameter('config_dir', '')
        cfg = load_config(self.get_parameter('config_dir').value or None)
        t = cfg['eval']['topics']
        self.optical_frame = cfg['eval']['frames']['camera_optical']
        self.estimator = load_estimator(
            self.get_parameter('estimator').value,
            **json.loads(self.get_parameter('estimator_kwargs').value))
        self.estimator.setup(gt=None, cfg=cfg)
        self.instruction = ''
        self._pending = {}
        self._count = 0
        self.create_subscription(Image, t['input_rgb'], lambda m: self._on('rgb', m), 1)
        self.create_subscription(Image, t['input_depth'], lambda m: self._on('depth', m), 1)
        self.create_subscription(CameraInfo, t['input_camera_info'],
                                 lambda m: self._on('info', m), 1)
        self.create_subscription(String, t['instruction'], self._on_instruction, 1)
        self._pub = self.create_publisher(Detection3DArray, t['estimates'], 10)

    def _on_instruction(self, msg):
        self.instruction = msg.data

    def _on(self, kind, msg):
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        slot = self._pending.setdefault(key, {})
        slot[kind] = msg
        if len(slot) < 3:
            return
        self._pending = {}
        frame = Frame(view=self._count, rgb=ru.image_to_rgb(slot['rgb']),
                      depth=ru.image_to_depth(slot['depth']),
                      K=np.asarray(slot['info'].k, float).reshape(3, 3),
                      instruction=self.instruction)
        self._count += 1
        out = Detection3DArray()
        out.header.stamp = slot['rgb'].header.stamp
        out.header.frame_id = self.optical_frame
        for d in self.estimator.estimate(frame):
            det = Detection3D()
            det.header = out.header
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
