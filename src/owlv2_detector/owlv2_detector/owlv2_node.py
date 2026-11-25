#!/usr/bin/env python3
"""OWLv2 en vivo sobre las cámaras (solo para VER en RViz; no es la evaluación).

Procesa una de cada `process_every_n` imágenes de cada tópico, de forma
continua. Para detectar en cada vista de la pasada de reconocimiento se usa el
estimador (owlv2_detector.estimator), no este nodo.

Usa el mismo detector y las mismas consultas que el estimador (objects.yaml de
vla_pose_eval), salvo que se pasen `queries` explícitas.

Publica por tópico:
  /owlv2/<tópico>/image        imagen anotada (verde: objeto, gris: consulta negativa)
  /owlv2/<tópico>/detections   JSON [{source, label, score, box}]
"""

import json
import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .detector import Owlv2Detector, draw, prompts_from_config


class Owlv2Node(Node):

    def __init__(self):
        super().__init__('owlv2_detector')
        self.declare_parameter('image_topics', ['/camera/image', '/mono_camera/image_raw'])
        self.declare_parameter('queries', [''])        # vacío: consultas de objects.yaml
        self.declare_parameter('threshold', -1.0)      # < 0: el de objects.yaml
        self.declare_parameter('process_every_n', 3)
        self.declare_parameter('model_name', '')
        self.declare_parameter('config_dir', '')

        from vla_pose_eval.config import load_config
        objects = load_config(self.get_parameter('config_dir').value or None)['objects']
        classes, negatives, det_cfg = prompts_from_config(objects)
        queries = [q for q in self.get_parameter('queries').value if q]
        if queries:                                    # consultas sueltas: la clase es la consulta
            classes, negatives = {q: [q] for q in queries}, []
        if self.get_parameter('threshold').value >= 0:
            det_cfg['threshold'] = float(self.get_parameter('threshold').value)
        if self.get_parameter('model_name').value:
            det_cfg['model_name'] = self.get_parameter('model_name').value
        self.detector = Owlv2Detector(classes, negatives, **det_cfg)
        self.get_logger().info('Cargando %s…' % self.detector.model_name)
        self.detector.load()
        self.get_logger().info('OWLv2 en %s; clases %s; negativas %s'
                               % (self.detector.device, list(classes), negatives))

        self.topics = list(self.get_parameter('image_topics').value)
        self.every_n = int(self.get_parameter('process_every_n').value)
        self.bridge = CvBridge()
        self.frame_idx = {t: 0 for t in self.topics}
        self.pubs = {}
        for topic in self.topics:
            short = topic.strip('/').replace('/', '_')
            self.pubs[topic] = (self.create_publisher(Image, '/owlv2/%s/image' % short, 10),
                                self.create_publisher(String, '/owlv2/%s/detections' % short, 10))
            self.create_subscription(Image, topic, lambda msg, t=topic: self.image_cb(msg, t),
                                     qos_profile_sensor_data)
            self.get_logger().info('Escuchando %s -> /owlv2/%s/' % (topic, short))

    def image_cb(self, msg, topic):
        self.frame_idx[topic] += 1
        if self.frame_idx[topic] % self.every_n != 0:
            return
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn('%s: no pude convertir la imagen (%s)' % (topic, e))
            return
        t0 = time.time()
        raw = self.detector.detect_raw(rgb)
        dets = [{'source': topic, 'label': b.cls, 'score': round(b.score, 3),
                 'box': [int(round(v)) for v in b.xyxy]} for b in raw if not b.negative]
        img_pub, det_pub = self.pubs[topic]
        out = draw(rgb, raw)
        if out is not None:
            img_msg = self.bridge.cv2_to_imgmsg(out, encoding='rgb8')
            img_msg.header = msg.header
            img_pub.publish(img_msg)
        det_pub.publish(String(data=json.dumps(dets)))
        self.get_logger().info('[%s] %d detecciones en %.2f s'
                               % (topic, len(dets), time.time() - t0))


def main(args=None):
    rclpy.init(args=args)
    node = Owlv2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
