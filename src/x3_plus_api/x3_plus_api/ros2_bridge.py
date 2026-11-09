'''
Puente ROS2 para robot X3 Plus:
Este módulo se encarga de comunicar el robot X3 Plus con ROS2 accediendo a los tópicos para mantener la
información del robot actualizada y controlar la activación de la teleoperación.

'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import threading
import time

# Clase representantiva del puente ROS2
class ROS2Bridge(Node):
    def __init__(self):
        super().__init__('x3_plus_api_bridge')
        self._lock = threading.Lock()

        # Estado actual del robot
        self.state = {
            "mobile": { # Estado del robot móvil
                "position": {'x': 0.0, 'y': 0.0, 'theta': 0.0},
                "velocity": {'linear': 0.0, 'angular': 0.0},
            },
            "arm": { # Estado del brazo
                "joint_names": [],
                "positions": [],
                "velocities": []
            }
        }

        # Buffer de imágenes de la cámara
        self.recording = False
        self.teleop_active = False
        self.image_sequence = []
        self.state_sequence = []

        # Suscripción a tópicos de ROS2
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Publisher para activar x3_plus_teleop
        self.teleop_publisher = self.create_publisher(Bool, '/x3_plus_teleop/activate', 10)

    # Definimos callbacks para actualizar el estado del robot según los mensajes recibidos de ROS2
    def odom_callback(self, msg: Odometry):
        with self._lock:
            self.state["mobile"]["position"]['x'] = msg.pose.pose.position.x
            self.state["mobile"]["position"]['y'] = msg.pose.pose.position.y
            self.state["mobile"]["velocity"]['linear'] = msg.twist.twist.linear.x
            self.state["mobile"]["velocity"]['angular'] = msg.twist.twist.angular.z

    def joint_state_callback(self, msg: JointState):
        with self._lock:
            self.state["arm"]["joint_names"] = list(msg.name)
            self.state["arm"]["positions"] = list(msg.position)
            self.state["arm"]["velocities"] = list(msg.velocity)

    def image_callback(self, msg: Image):
        if not self.recording:
            return
        with self._lock:
            timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
            # Guardamos metadatos o frame codificado para no saturar RAM
            frame_info = {
                "id": f"I_{len(self.image_sequence) + 1}",
                "timestamp": timestamp,
                "width": msg.width,
                "height": msg.height,
                "encoding": msg.encoding
            }
            self.image_sequence.append(frame_info)
            # Sincronizamos con una copia del estado actual
            self.state_sequence.append({
                "timestamp": timestamp,
                "mobile": self.state["mobile"].copy(),
                "arm": self.state["arm"].copy()
            })

    # Función para obtener un snapshot del estado actual del robot y activar/desactivar la teleoperación
    def get_snapshot(self):
        with self._lock:
            return {
                "teleop_active": self.teleop_active,
                "recording": self.recording,
                "status": self.state.copy(),
                "total_images_captured": len(self.image_sequence)
            }

    def set_teleop(self, enable: bool):
        with self._lock:
            self.teleop_active = enable
            msg = Bool()
            msg.data = enable
            self.teleop_pub.publish(msg)
        return self.teleop_active

    # Funciones para manipular la captura de imágenes y estados
    def start_capture(self):
        with self._lock:
            self.recording = True
            self.image_sequence.clear()
            self.state_sequence.clear()

    def stop_capture(self):
        with self._lock:
            self.recording = False
            payload = {
                "Image": list(self.image_sequence),
                "Sync_States": list(self.state_sequence),
                "total_frames": len(self.image_sequence)
            }
        return payload 
    