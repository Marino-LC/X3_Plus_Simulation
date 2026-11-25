#!/usr/bin/env python3
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ViewSaver(Node):
    def __init__(self):
        super().__init__("owlv2_view_saver")

        self.declare_parameter("save_dir", "")
        self.declare_parameter("topics", [
            "/owlv2/camera_image/image",
            "/owlv2/mono_camera_image_raw/image",
        ])
        self.declare_parameter("every_n", 3)

        self.save_dir = self.get_parameter("save_dir").value
        self.topics = list(self.get_parameter("topics").value)
        self.every_n = int(self.get_parameter("every_n").value)

        if not self.save_dir:
            self.get_logger().error("Falta el parámetro save_dir; el nodo no guardará nada.")
            return

        os.makedirs(self.save_dir, exist_ok=True)
        self.bridge = CvBridge()
        self.counts = {t: 0 for t in self.topics}

        for topic in self.topics:
            self.create_subscription(
                Image, topic,
                lambda msg, t=topic: self.cb(msg, t),
                qos_profile_sensor_data,
            )
            self.get_logger().info(f"Guardando {topic} en {self.save_dir}")

    def cb(self, msg, topic):
        self.counts[topic] += 1
        if self.counts[topic] % self.every_n != 0:
            return

        short = topic.strip("/").replace("/", "_")
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Captura numerada y una copia "última" para revisar rápido
        cv2.imwrite(os.path.join(self.save_dir, f"{short}_{self.counts[topic]:05d}.png"), frame)
        cv2.imwrite(os.path.join(self.save_dir, f"last_{short}.png"), frame)


def main(args=None):
    rclpy.init(args=args)
    node = ViewSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
