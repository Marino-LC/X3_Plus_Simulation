#!/usr/bin/env python3
"""Publica la verdad de referencia de la escena y el TF map -> base_footprint.

* /ground_truth/objects  (geometry_msgs/PoseArray, marco map): un Pose por
  objeto, en el orden de `objects` de scene_gt.yaml.
* /ground_truth/markers  (visualization_msgs/MarkerArray): para RViz.
* TF map -> base_footprint a partir de la pose real del robot en Gazebo.

OJO: no ejecutar a la vez Mecanum_odometry_node.py (publica odom ->
base_footprint y base_footprint tendría dos padres). En esta fase la pose del
robot viene de Gazebo, nunca de la odometría de las ruedas.
"""

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

from .. import dataset, geometry as g
from ..config import load_config


class GroundTruthPublisher(Node):

    def __init__(self):
        super().__init__('ground_truth_publisher', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.declare_parameter('trial_dir', '')
        self.declare_parameter('config_dir', '')
        self.declare_parameter('publish_tf', True)
        ev = load_config(self.get_parameter('config_dir').value or None)['eval']
        self.frames = ev['frames']
        gt = dataset.load_gt(self.get_parameter('trial_dir').value)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._poses_pub = self.create_publisher(PoseArray, ev['topics']['ground_truth'], latched)
        self._markers_pub = self.create_publisher(MarkerArray, '/ground_truth/markers', latched)
        self._poses, self._markers = self._build(gt)
        self._tf = tf2_ros.TransformBroadcaster(self)
        if self.get_parameter('publish_tf').value:
            self.create_subscription(PoseStamped, ev['topics']['robot_pose_gt'], self._on_pose, 10)
        self.create_timer(1.0, self._publish)
        self._publish()

    def _build(self, gt):
        poses, markers = PoseArray(), MarkerArray()
        poses.header.frame_id = self.frames['map']
        shape = {'box': Marker.CUBE, 'cylinder': Marker.CYLINDER, 'sphere': Marker.SPHERE}
        for o in gt['objects']:
            p = Pose()
            p.position.x, p.position.y, p.position.z = float(o['x']), float(o['y']), float(o['z'])
            q = g.yaw_to_quat(o['yaw'])
            p.orientation.z, p.orientation.w = q[2], q[3]
            poses.poses.append(p)
            m = Marker()
            m.header.frame_id = self.frames['map']
            m.ns, m.id, m.type, m.pose = o['class'], int(o['id']), shape[o['shape']], p
            s = o['size']
            if o['shape'] == 'box':
                m.scale.x, m.scale.y, m.scale.z = s[0], s[1], s[2]
            elif o['shape'] == 'cylinder':
                m.scale.x, m.scale.y, m.scale.z = 2 * s[0], 2 * s[0], s[1]
            else:
                m.scale.x = m.scale.y = m.scale.z = 2 * s[0]
            m.color.r, m.color.g, m.color.b = (float(c) for c in o['color'])
            m.color.a = 0.8
            markers.markers.append(m)
        return poses, markers

    def _publish(self):
        now = self.get_clock().now().to_msg()
        self._poses.header.stamp = now
        for m in self._markers.markers:
            m.header.stamp = now
        self._poses_pub.publish(self._poses)
        self._markers_pub.publish(self._markers)

    def _on_pose(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.frames['map']
        t.child_frame_id = self.frames['base']
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self._tf.sendTransform(t)


def main():
    rclpy.init()
    node = GroundTruthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
