"""One NavigateToPose goal at a time, selected from reachable map frontiers."""
import math
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener, TransformException
from .geometry import frontier_goal, quaternion_matrix


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.clearance = self.declare_parameter('clearance', 0.7).value
        self.max_radius = self.declare_parameter('max_radius', 3.).value
        self.enabled = False
        self.ready = (False, 0.)
        self.grid = None
        self.grid_received = 0.
        self.goal = None
        self.pending = False
        self.generation = 0
        self.blacklist = []
        self.last_attempt = 0.
        self.no_frontier_since = None
        self.goal_point = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.active_pub = self.create_publisher(Bool, 'live/exploration_active', 1)
        self.status = self.create_publisher(String, 'live/exploration_state', 1)
        self.create_subscription(Bool, 'live/navigation_ready', self.on_ready, 1)
        self.create_subscription(OccupancyGrid, 'map', self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(SetBool, 'frontier_explorer/set_enabled', self.enable)
        self.create_timer(0.1, self.tick)

    def on_ready(self, msg):
        self.ready = (msg.data, time.monotonic())

    def on_map(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        age = self.get_clock().now().nanoseconds*1e-9-stamp
        if (msg.header.frame_id == 'map' and 0 < msg.info.width*msg.info.height <= 4_000_000
                and len(msg.data) == msg.info.width*msg.info.height and -0.05 <= age < 3.):
            self.grid, self.grid_received = msg, time.monotonic()

    def robot_xy(self):
        transform = self.tf.lookup_transform('map', 'base_footprint', Time())
        age = (self.get_clock().now().nanoseconds-(transform.header.stamp.sec*1_000_000_000+transform.header.stamp.nanosec))*1e-9
        if not -0.05 <= age < 0.3:
            raise ValueError('Stale chassis transform')
        return np.array([transform.transform.translation.x, transform.transform.translation.y])

    def healthy(self):
        return (self.ready[0] and time.monotonic()-self.ready[1] < .3 and
                self.grid is not None and time.monotonic()-self.grid_received < 3.)

    def stop(self, reason):
        self.enabled = False
        self.generation += 1
        if self.goal is not None:
            self.goal.cancel_goal_async()
        self.goal = None
        self.active_pub.publish(Bool(data=False))
        self.status.publish(String(data=reason))
        self.get_logger().info(reason)

    def enable(self, request, response):
        if not request.data:
            self.stop('Exploration stopped')
            response.success, response.message = True, 'Stopped; current Nav2 goal cancelled'
            return response
        try:
            if self.enabled or self.pending:
                raise ValueError('Already exploring or waiting for previous action reply')
            if not self.healthy() or not self.client.server_is_ready():
                raise ValueError('Need fresh map/TF, healthy sensors, enabled motion gate and enabled chassis')
            self.start_xy = self.robot_xy()
            self.blacklist.clear()
            self.no_frontier_since = None
            self.enabled = True
            self.generation += 1
            response.success, response.message = True, 'Exploration started; radius 3 m, speed <= 0.1 m/s'
        except (ValueError, TransformException) as error:
            response.success, response.message = False, str(error)
        return response

    def tick(self):
        if self.enabled and not self.healthy():
            self.stop('Input/communication gate lost; manual restart required')
        self.active_pub.publish(Bool(data=self.enabled))
        if not self.enabled:
            return
        if self.goal is not None:
            if time.monotonic()-self.sent_at > 60.:
                self.stop('Nav2 goal timed out; inspect obstacles/localization before restarting')
            return
        if self.pending or time.monotonic()-self.last_attempt < 1.:
            return
        self.last_attempt = time.monotonic()
        try:
            robot = self.robot_xy()
            info = self.grid.info
            q = info.origin.orientation
            rot = quaternion_matrix([q.x, q.y, q.z, q.w])
            result = frontier_goal(np.asarray(self.grid.data).reshape(info.height, info.width),
                info.resolution, np.array([info.origin.position.x, info.origin.position.y]),
                math.atan2(rot[1, 0], rot[0, 0]), robot, self.clearance,
                self.start_xy, self.max_radius, self.blacklist)
        except (ValueError, TransformException) as error:
            self.stop(str(error))
            return
        if result is None:
            if self.no_frontier_since is None:
                self.no_frontier_since = time.monotonic()
            elif time.monotonic()-self.no_frontier_since > 10.:
                self.stop('No safe reachable frontier within test range; stopped (this does not prove the whole map is explored)')
            return
        self.no_frontier_since = None
        point, yaw = result
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = map(float, point)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.pending = True
        generation = self.generation
        future = self.client.send_goal_async(goal)
        self.goal_point, self.sent_at = point, time.monotonic()
        self.status.publish(String(data=f'Frontier goal x={point[0]:.2f}, y={point[1]:.2f}'))

        def accepted(reply):
            self.pending = False
            try:
                handle = reply.result()
                if generation != self.generation or not self.enabled:
                    if handle.accepted:
                        handle.cancel_goal_async()
                    return
                if not handle.accepted:
                    self.blacklist.append(point)
                    return
                self.goal = handle

                def done(result_future):
                    if generation == self.generation:
                        self.goal = None
                        self.blacklist.append(point)
                        try:
                            status = result_future.result().status
                            self.status.publish(String(data=f'Goal result={status}; selecting next frontier'))
                        except Exception as error:
                            self.stop('Nav2 result error: '+str(error))
                handle.get_result_async().add_done_callback(done)
            except Exception as error:
                self.stop('Nav2 action error: '+str(error))
        future.add_done_callback(accepted)


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
