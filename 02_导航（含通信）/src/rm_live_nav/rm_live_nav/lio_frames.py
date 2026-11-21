"""Convert Point-LIO's IMU pose into the fixed chassis pose and remove self points."""
from collections import deque
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster
import yaml
from .geometry import (base_from_imu, matrix_quaternion, outside_footprint,
                       quaternion_matrix, rpy_matrix, validate_mounting)


class LioFrames(Node):
    def __init__(self):
        super().__init__('lio_frames')
        self.mapping_only = self.declare_parameter('mapping_only', True).value
        self.frame = 'mapping_base' if self.mapping_only else 'base_footprint'
        self.footprint = None
        if not self.mapping_only:
            with open(self.declare_parameter('mounting_config', '').value) as stream:
                xyz, rpy, self.footprint = validate_mounting(yaml.safe_load(stream))
            self.base_lidar_t, self.base_lidar_r = xyz, rpy_matrix(rpy)
        self.imu_lidar_t = np.array(self.declare_parameter('imu_lidar_t', [-0.011, -0.02329, 0.04412]).value)
        self.imu_lidar_r = np.array(self.declare_parameter('imu_lidar_r', [1., 0., 0., 0., 1., 0., 0., 0., 1.]).value).reshape(3, 3)
        self.poses = deque(maxlen=100)
        self.pending = deque(maxlen=5)
        self.last_stamp = 0
        self.tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'live/odometry', 5)
        self.cloud_pub = self.create_publisher(PointCloud2, 'live/cloud', 5)
        self.create_subscription(Odometry, 'aft_mapped_to_init', self.odom, 5)
        self.create_subscription(PointCloud2, 'cloud_registered', self.cloud, qos_profile_sensor_data)

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec

    def odom(self, msg):
        stamp = self.stamp(msg)
        if msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'body' or stamp <= self.last_stamp:
            return
        pose = msg.pose.pose
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        q = pose.orientation
        try:
            rotation = quaternion_matrix([q.x, q.y, q.z, q.w])
        except ValueError:
            return
        if not np.isfinite(position).all():
            return
        if self.mapping_only:
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
            rotation = rpy_matrix((0., 0., yaw))
        else:
            rotation, position = base_from_imu(rotation, position, self.base_lidar_t,
                self.base_lidar_r, self.imu_lidar_t, self.imu_lidar_r)
        quat = matrix_quaternion(rotation)
        out = Odometry()
        out.header, out.child_frame_id = msg.header, self.frame
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, position)
        out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = map(float, quat)
        if self.poses:
            previous_stamp, previous_rotation, previous_position = self.poses[-1]
            dt = (stamp-previous_stamp)*1e-9
            if 0.01 <= dt <= 0.2:
                velocity = rotation.T @ ((position-previous_position)/dt)
                delta = previous_rotation.T @ rotation
                out.twist.twist.linear.x, out.twist.twist.linear.y = float(velocity[0]), float(velocity[1])
                out.twist.twist.angular.z = math.atan2(delta[1, 0], delta[0, 0])/dt
        transform = TransformStamped()
        transform.header, transform.child_frame_id = out.header, self.frame
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, position)
        transform.transform.rotation = out.pose.pose.orientation
        self.tf.sendTransform(transform)
        self.odom_pub.publish(out)
        self.poses.append((stamp, rotation, position))
        self.last_stamp = stamp
        pending = list(self.pending)
        self.pending.clear()
        for cloud in pending:
            self.cloud(cloud)

    def cloud(self, msg):
        if msg.header.frame_id != 'camera_init' or not msg.width*msg.height:
            return
        stamp = self.stamp(msg)
        # Clouds can be published before the matching odometry callback.
        match = min(self.poses, key=lambda pose: abs(pose[0]-stamp), default=None)
        if match is None or abs(match[0]-stamp) > 20_000_000:
            if match is None or stamp > self.poses[-1][0]:
                self.pending.append(msg)
            return
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)
        points = points[np.isfinite(points).all(axis=1)]
        if not len(points):
            return
        if self.footprint is not None:
            base_points = (points[:, :3]-match[2]) @ match[1]
            points = points[outside_footprint(base_points[:, :2], self.footprint)]
        if len(points):
            fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
            self.cloud_pub.publish(point_cloud2.create_cloud(msg.header, fields, points.astype(np.float32)))


def main():
    rclpy.init()
    node = LioFrames()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
