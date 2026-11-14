"""Utilidades compartidas por los nodos ROS 2 (conversión de mensajes)."""

import numpy as np

from .. import geometry as g


def image_to_rgb(msg):
    """sensor_msgs/Image (rgb8 o bgr8) -> (H, W, 3) uint8 RGB, sin cv_bridge."""
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError('codificación RGB no soportada: %s' % msg.encoding)
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    a = a[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return (a[:, :, ::-1] if msg.encoding == 'bgr8' else a).copy()


def image_to_depth(msg):
    """sensor_msgs/Image (32FC1 en m, o 16UC1 en mm) -> (H, W) float32 en metros."""
    if msg.encoding == '32FC1':
        a = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.step // 4)
        return a[:, :msg.width].copy()
    if msg.encoding == '16UC1':
        a = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.step // 2)
        d = a[:, :msg.width].astype(np.float32) / 1000.0
        d[d == 0.0] = np.nan
        return d
    raise ValueError('codificación de profundidad no soportada: %s' % msg.encoding)


def pose_to_T(pose):
    """geometry_msgs/Pose -> matriz 4x4."""
    q, p = pose.orientation, pose.position
    return g.make_T(g.quat_to_R([q.x, q.y, q.z, q.w]), [p.x, p.y, p.z])


def transform_to_T(tf):
    """geometry_msgs/Transform -> matriz 4x4."""
    q, t = tf.rotation, tf.translation
    return g.make_T(g.quat_to_R([q.x, q.y, q.z, q.w]), [t.x, t.y, t.z])


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9
