#!/usr/bin/env python3
import json
import time

import cv2
import rclpy
import torch
from cv_bridge import CvBridge
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from transformers import Owlv2ForObjectDetection, Owlv2Processor


class Owlv2Node(Node):
    def __init__(self):
        super().__init__("owlv2_detector")

        self.declare_parameter("image_topics", ["/camera/image", "/mono_camera/image_raw"])
        self.declare_parameter("queries", ["a red cube", "a cylinder", "a box"])
        self.declare_parameter("threshold", 0.2)
        self.declare_parameter("process_every_n", 3)
        self.declare_parameter("model_name", "google/owlv2-base-patch16-ensemble")

        self.topics = list(self.get_parameter("image_topics").value)
        self.queries = list(self.get_parameter("queries").value)
        self.threshold = float(self.get_parameter("threshold").value)
        self.every_n = int(self.get_parameter("process_every_n").value)
        model_name = self.get_parameter("model_name").value

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Cargando {model_name} en {self.device}...")
        self.processor = Owlv2Processor.from_pretrained(model_name)
        self.model = Owlv2ForObjectDetection.from_pretrained(model_name).to(self.device).eval()

        self.bridge = CvBridge()
        self.frame_idx = {t: 0 for t in self.topics}
        self.pubs = {}

        for topic in self.topics:
            short = topic.strip("/").replace("/", "_")
            img_pub = self.create_publisher(Image, f"/owlv2/{short}/image", 10)
            det_pub = self.create_publisher(String, f"/owlv2/{short}/detections", 10)
            self.pubs[topic] = (img_pub, det_pub)
            self.create_subscription(
                Image, topic,
                lambda msg, t=topic: self.image_cb(msg, t),
                qos_profile_sensor_data,
            )
            self.get_logger().info(f"Escuchando {topic} -> /owlv2/{short}/")

        self.get_logger().info(f"Queries: {self.queries}")

    def image_cb(self, msg, topic):
        self.frame_idx[topic] += 1
        if self.frame_idx[topic] % self.every_n != 0:
            return

        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            self.get_logger().warn(f"{topic}: no pude convertir la imagen ({e})")
            return

        pil = PILImage.fromarray(rgb)
        inputs = self.processor(text=[self.queries], images=pil, return_tensors="pt")
        inputs = {k: v.to(self.device) for k, v in inputs.items()}

        t0 = time.time()
        with torch.no_grad():
            outputs = self.model(**inputs)

        # OWLv2 rellena la imagen a cuadrado, así que las cajas usan el lado mayor
        side = max(pil.size)
        target = torch.tensor([[side, side]], device=self.device)
        res = self.processor.post_process_object_detection(
            outputs=outputs, threshold=self.threshold, target_sizes=target
        )[0]

        boxes = res["boxes"].cpu().numpy()
        scores = res["scores"].cpu().numpy()
        labels = res["labels"].cpu().numpy()

        out = rgb.copy()
        dets = []
        for box, score, label in zip(boxes, scores, labels):
            x0, y0, x1, y1 = [int(v) for v in box]
            name = self.queries[int(label)]
            dets.append({
                "source": topic,
                "label": name,
                "score": round(float(score), 3),
                "box": [x0, y0, x1, y1],
            })
            cv2.rectangle(out, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2.putText(out, f"{name} {score:.2f}", (x0, max(y0 - 5, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        img_pub, det_pub = self.pubs[topic]
        img_msg = self.bridge.cv2_to_imgmsg(out, encoding="rgb8")
        img_msg.header = msg.header
        img_pub.publish(img_msg)
        det_pub.publish(String(data=json.dumps(dets)))

        self.get_logger().info(f"[{topic}] {len(dets)} detecciones en {time.time() - t0:.2f} s")


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


if __name__ == "__main__":
    main()
