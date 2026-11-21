"""Latch off on stale sensors, pose, faults or near obstacles. Never auto-enable USB."""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from .geometry import quaternion_matrix


class MotionGate(Node):
    def __init__(self):
        super().__init__('motion_gate')
        self.armed = False
        self.activity = (False, 0.)
        self.received = {}
        self.source_stamps = {}
        self.command = None
        self.diagnostic = None
        self.obstacle = True
        self.level_pose = False
        self.position = None
        self.start_position = None
        self.radius = self.declare_parameter('stop_radius', 0.7).value
        self.output = self.create_publisher(TwistStamped, 'live/cmd_vel', 1)
        self.ready_pub = self.create_publisher(Bool, 'live/navigation_ready', 1)
        self.frame_pub = self.create_publisher(Bool, 'frame_ready', 1)
        self.state_pub = self.create_publisher(String, 'live/motion_state', 1)
        self.create_subscription(PointCloud2, 'livox/lidar/pointcloud', lambda msg: self.mark('raw_cloud', msg, bool(msg.width*msg.height)), qos_profile_sensor_data)
        self.create_subscription(Odometry, 'live/odometry', self.odom, 5)
        self.create_subscription(LaserScan, 'live/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, 'map', lambda msg: self.mark('map', msg, bool(msg.data) and msg.header.frame_id == 'map'),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(DiagnosticArray, 'diagnostics', self.diagnose, 5)
        self.create_subscription(Twist, 'live/cmd_vel_raw', self.cmd, 1)
        self.create_subscription(Bool, 'live/exploration_active', self.active, 1)
        self.create_service(SetBool, 'motion_gate/set_enabled', self.enable)
        self.create_timer(0.02, self.tick)
        self.last_reason = None

    def mark(self, name, msg, valid=True):
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)*1e-9
        if valid and -0.05 <= age <= (3. if name == 'map' else 0.3) and stamp > self.source_stamps.get(name, 0):
            self.received[name] = time.monotonic()
            self.source_stamps[name] = stamp
            return True
        return False

    def odom(self, msg):
        q = msg.pose.pose.orientation
        try:
            rot = quaternion_matrix([q.x, q.y, q.z, q.w])
            values = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                      msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z]
            valid = np.isfinite(values).all() and msg.header.frame_id == 'camera_init' and msg.child_frame_id == 'base_footprint'
            if self.mark('odom', msg, valid):
                self.level_pose = rot[2, 2] > math.cos(0.15)
                self.position = np.array(values[:2])
        except ValueError:
            self.level_pose = False

    def scan(self, msg):
        valid = (msg.header.frame_id == 'base_footprint' and len(msg.ranges) > 10
                 and math.isfinite(msg.range_min) and msg.range_min >= 0)
        if self.mark('scan', msg, valid):
            ranges = np.asarray(msg.ranges)
            finite = ranges[np.isfinite(ranges) & (ranges >= msg.range_min)]
            self.obstacle = not len(finite) or (finite < self.radius).any()

    def diagnose(self, msg):
        for status in msg.status:
            if status.name == '/chassis_comm':
                self.diagnostic = status
                self.received['diagnostics'] = time.monotonic()

    def cmd(self, msg):
        values = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
        if all(math.isfinite(value) for value in values) and not any(values[i] for i in (2, 3, 4)):
            self.command = (msg, time.monotonic(), self.get_clock().now().to_msg())

    def active(self, msg):
        if not msg.data or not self.activity[0]:
            self.command = None
        self.activity = (msg.data, time.monotonic())

    def health(self):
        now = time.monotonic()
        for name, ttl in [('raw_cloud', .3), ('odom', .3), ('scan', .3), ('map', 3.), ('diagnostics', 1.5)]:
            if now-self.received.get(name, 0.) > ttl:
                return 'stale '+name
        if not self.level_pose:
            return 'chassis pose is tilted; verify mounting'
        if self.obstacle:
            return 'near obstacle or no usable scan returns'
        if self.start_position is not None and self.position is not None and np.linalg.norm(self.position-self.start_position) > 3.:
            return '3 m commissioning boundary reached'
        values = {item.key: item.value for item in self.diagnostic.values}
        if self.diagnostic.level not in (0, b'\x00') or values.get('fault_mask') != '0':
            return 'chassis not READY or fault != 0'
        return ''

    def enable(self, request, response):
        if request.data:
            self.start_position = None
        reason = self.health() if request.data else ''
        self.armed = request.data and not reason
        if self.armed:
            self.start_position = self.position.copy()
        self.command = None
        response.success = not reason
        response.message = reason or ('Gate enabled; enable chassis and start exploration separately' if self.armed else 'Gate disabled')
        return response

    def tick(self):
        reason = self.health()
        if self.armed and self.activity[0] and time.monotonic()-self.activity[1] >= .3:
            reason = 'stale exploration heartbeat'
        if reason:
            self.armed = False
            self.command = None
        # Frame health is independent of arming, so a disabled chassis can handshake.
        # Chassis diagnostics depend on frame_ready, so exclude that circular gate.
        frame_ready = (self.level_pose and not self.obstacle and all(
            time.monotonic()-self.received.get(k, 0.) < ttl
            for k, ttl in [('raw_cloud', .3), ('odom', .3), ('scan', .3), ('map', 3.)]))
        self.frame_pub.publish(Bool(data=bool(frame_ready)))
        values = {} if self.diagnostic is None else {v.key: v.value for v in self.diagnostic.values}
        ready = self.armed and not reason and values.get('enabled') == 'true'
        self.ready_pub.publish(Bool(data=ready))
        output = TwistStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = 'base_footprint'
        active, received = self.activity
        if ready and active and time.monotonic()-received < .3 and self.command and time.monotonic()-self.command[1] < .15:
            cmd, _, stamp = self.command
            scale = min(1., .1/max(math.hypot(cmd.linear.x, cmd.linear.y), 1e-12))
            output.twist.linear.x, output.twist.linear.y = cmd.linear.x*scale, cmd.linear.y*scale
            output.twist.angular.z = max(-.25, min(.25, cmd.angular.z))
            output.header.stamp = stamp  # Never refresh an old controller command's timestamp.
        self.output.publish(output)
        state = reason or ('enabled' if self.armed else 'disabled; manual enable required')
        if state != self.last_reason:
            self.get_logger().info(state)
            self.last_reason = state
        self.state_pub.publish(String(data=state))


def main():
    rclpy.init()
    node = MotionGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
