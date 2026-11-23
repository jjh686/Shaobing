#!/usr/bin/env python3
"""Report fresh localization and plans against a confirmed navigation grid."""

from dataclasses import dataclass
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from intent_nav_bridge import load_points, stamp_ns
from rm2027_decision.msg import NavigationEvidence, TacticalObjective


POSE_AGE_NS = 750_000_000
PLAN_AGE_NS = 1_500_000_000
PLAN_TIMEOUT_S = 1.0
CANCEL_GRACE_S = 1.0
ENDPOINT_TOLERANCE_M = 0.25
REPLAN_DISTANCE_M = 0.30


def xy(pose):
    return (pose.pose.position.x, pose.pose.position.y)


@dataclass(frozen=True)
class RouteGrid:
    frame: str
    resolution: float
    width: int
    height: int
    origin: tuple
    rotation: tuple
    cells: bytes

    @classmethod
    def from_msg(cls, msg):
        info = msg.info
        x, y = info.origin.position.x, info.origin.position.y
        q = info.origin.orientation
        if (not all(math.isfinite(v) for v in (info.resolution, x, y, q.x, q.y, q.z, q.w))
                or info.resolution <= 0 or not info.width or not info.height
                or len(msg.data) != info.width * info.height
                or abs(q.x) > 1e-6 or abs(q.y) > 1e-6
                or not math.isclose(q.z * q.z + q.w * q.w, 1.0, abs_tol=1e-3)):
            raise ValueError('Invalid or nonplanar route grid')
        yaw = 2.0 * math.atan2(q.z, q.w)
        return cls(msg.header.frame_id, info.resolution, info.width, info.height, (x, y),
                   (math.cos(yaw), math.sin(yaw)), bytes(cell == 0 for cell in msg.data))

    def free(self, point, padding):
        x, y = point[0] - self.origin[0], point[1] - self.origin[1]
        cos_yaw, sin_yaw = self.rotation
        col, row = cos_yaw * x + sin_yaw * y, -sin_yaw * x + cos_yaw * y
        if (not padding <= col < self.width * self.resolution - padding
                or not padding <= row < self.height * self.resolution - padding):
            return False
        low_col = math.floor((col - padding) / self.resolution)
        high_col = math.floor((col + padding) / self.resolution)
        low_row = math.floor((row - padding) / self.resolution)
        high_row = math.floor((row + padding) / self.resolution)
        return (0 <= low_col <= high_col < self.width and 0 <= low_row <= high_row < self.height
                and all(self.cells[self.width * row + col]
                        for row in range(low_row, high_row + 1)
                        for col in range(low_col, high_col + 1)))

    def path_distance(self, path, start, target, clearance):
        if path.header.frame_id != self.frame or not path.poses:
            return None
        if any(p.header.frame_id not in ('', self.frame) for p in path.poses):
            return None
        waypoints = [start, *(xy(p) for p in path.poses), target]
        step = self.resolution / 2.0
        # Extra padding covers the continuous span between sampled points.
        padding = clearance + step / 2.0
        if any(not all(map(math.isfinite, p)) or not self.free(p, padding) for p in waypoints):
            return None
        if (math.dist(start, waypoints[1]) > ENDPOINT_TOLERANCE_M
                or math.dist(target, waypoints[-2]) > ENDPOINT_TOLERANCE_M):
            return None
        distance = 0.0
        for left, right in zip(waypoints, waypoints[1:]):
            length = math.dist(left, right)
            distance += length
            count = math.ceil(length / step)
            for i in range(1, count):
                fraction = i / count
                point = (left[0] + (right[0] - left[0]) * fraction,
                         left[1] + (right[1] - left[1]) * fraction)
                if not self.free(point, padding):
                    return None
        return distance


@dataclass
class Probe:
    target: str
    pose: PoseStamped
    grid: RouteGrid
    started: float
    ack: object
    handle: object = None
    result: object = None
    cancel_reply: object = None
    canceled_at: object = None


class NavigationEvidenceNode(Node):
    def __init__(self):
        super().__init__('navigation_evidence')
        task_file = self.fixed('tasks_file', '')
        frame, self.points = load_points(task_file)
        self.frame = self.fixed('map_frame', frame)
        self.base_frame = self.fixed('base_frame', 'base_footprint')
        if self.frame != frame or not self.base_frame or self.base_frame == self.frame:
            raise ValueError('Evidence frames must match the task map and a distinct robot frame')
        self.confirmed = self.fixed('protected_map_confirmed', False)
        self.clearance = self.fixed('clearance_radius_m', 0.0)
        self.speed = self.fixed('nominal_speed_mps', 0.15)
        if (not math.isfinite(self.speed) or self.speed <= 0
                or not math.isfinite(self.clearance) or self.clearance < 0):
            raise ValueError('Invalid nominal speed or clearance')
        self.planner_id = self.fixed('planner_id', '')
        self.tasks = self.approved_tasks(task_file)
        self.grid = None
        self.plans = {}
        self.probe = None
        self.next_task = 0
        self.last_now = 0
        self.pose_stamp = 0
        self.pose_received = 0.0
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.client = ActionClient(self, ComputePathToPose, self.fixed('planner_action', '/compute_path_to_pose'))
        self.pub = self.create_publisher(NavigationEvidence, 'navigation_evidence', qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, self.fixed('route_map_topic', '/map'), self.on_grid, map_qos)
        self.create_timer(0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def fixed(self, name, fallback):
        return self.declare_parameter(name, fallback, ParameterDescriptor(read_only=True)).value

    def approved_tasks(self, filename):
        if not self.points:
            return {}
        raw = yaml.safe_load(Path(filename).read_text(encoding='utf-8'))
        tasks = {}
        for item in raw['objectives']:
            target, risk = item['id'], item.get('risk')
            if risk is None:
                self.get_logger().warning(f'Skipping {target} without reviewed route risk')
                continue
            radius = item.get('radius_m', ENDPOINT_TOLERANCE_M)
            if (type(risk) not in (int, float) or not 0.0 <= risk <= 1.0
                    or type(radius) not in (int, float) or not math.isfinite(radius) or radius <= 0):
                raise ValueError(f'Invalid risk or radius_m for {target}')
            tasks[target] = (float(risk), float(radius))
        return tasks

    def on_grid(self, msg):
        if not self.confirmed:
            return
        try:
            grid = RouteGrid.from_msg(msg) if msg.header.frame_id == self.frame else None
        except ValueError:
            grid = None
        if grid != self.grid:
            self.grid = grid
            self.plans.clear()
            self.cancel_probe()

    def localization(self, now):
        try:
            tf = self.buffer.lookup_transform(self.frame, self.base_frame, Time())
        except TransformException:
            return None
        stamp = stamp_ns(tf.header.stamp)
        if not 0 < stamp <= now or now - stamp >= POSE_AGE_NS or stamp < self.pose_stamp:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        if (not all(math.isfinite(v) for v in (t.x, t.y, t.z, q.x, q.y, q.z, q.w))
                or not math.isclose(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1.0, abs_tol=1e-3)):
            return None
        wall_now = time.monotonic()
        if stamp > self.pose_stamp:
            self.pose_stamp, self.pose_received = stamp, wall_now
        elif wall_now - self.pose_received >= POSE_AGE_NS / 1e9:
            return None
        pose = PoseStamped()
        pose.header = tf.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = t.x, t.y, t.z
        pose.pose.orientation = q
        return pose

    def cancel_probe(self):
        probe = self.probe
        if probe is not None and probe.canceled_at is None:
            probe.canceled_at = time.monotonic()
            self.plans.pop(probe.target, None)
            if probe.handle is not None:
                self.send_cancel(probe)

    def send_cancel(self, probe):
        try:
            probe.cancel_reply = probe.handle.cancel_goal_async()
        except Exception as error:
            self.get_logger().warning(f'Could not cancel planner request: {error}', throttle_duration_sec=5.0)

    def retire_probe(self):
        probe, self.probe = self.probe, None
        for future in (probe.ack, probe.result, probe.cancel_reply):
            if future is not None and not future.done():
                future.cancel()

    def poll_probe(self, pose, now):
        probe = self.probe
        if probe is None:
            return
        if time.monotonic() - probe.started >= PLAN_TIMEOUT_S:
            self.cancel_probe()
        try:
            if probe.handle is None and probe.ack.done():
                probe.handle = probe.ack.result()
                if not probe.handle.accepted:
                    self.plans.pop(probe.target, None)
                    self.retire_probe()
                    return
                if probe.canceled_at is not None:
                    self.send_cancel(probe)
                probe.result = probe.handle.get_result_async()
            if probe.result is None or not probe.result.done():
                if probe.canceled_at is not None and time.monotonic() - probe.canceled_at >= CANCEL_GRACE_S:
                    self.retire_probe()
                return
            result = probe.result.result()
        except Exception as error:
            self.get_logger().warning(f'Planner request failed: {error}', throttle_duration_sec=5.0)
            self.plans.pop(probe.target, None)
            self.cancel_probe()
            self.retire_probe()
            return
        self.retire_probe()
        self.plans.pop(probe.target, None)
        source_ns = stamp_ns(probe.pose.header.stamp)
        if (probe.canceled_at is not None or pose is None or self.grid is not probe.grid
                or result.status != GoalStatus.STATUS_SUCCEEDED or not 0 < source_ns <= now
                or now - source_ns >= PLAN_AGE_NS or math.dist(xy(pose), xy(probe.pose)) > REPLAN_DISTANCE_M):
            return
        point = self.points[probe.target]
        distance = probe.grid.path_distance(result.result.path, xy(probe.pose), (point.x, point.y), self.clearance)
        if distance is not None:
            self.plans[probe.target] = (source_ns, xy(probe.pose), distance)

    def request_plan(self, pose, now):
        tasks = tuple(self.tasks)
        target = tasks[self.next_task]
        self.next_task = (self.next_task + 1) % len(tasks)
        point = self.points[target]
        goal = ComputePathToPose.Goal()
        goal.start = pose
        goal.use_start = True
        goal.planner_id = self.planner_id
        goal.goal.header.frame_id, goal.goal.header.stamp = self.frame, now.to_msg()
        goal.goal.pose.position.x, goal.goal.pose.position.y = point.x, point.y
        goal.goal.pose.orientation.z, goal.goal.pose.orientation.w = math.sin(point.yaw / 2), math.cos(point.yaw / 2)
        try:
            ack = self.client.send_goal_async(goal)
        except Exception as error:
            self.plans.pop(target, None)
            self.get_logger().warning(f'Could not queue planner request: {error}', throttle_duration_sec=5.0)
            return
        self.probe = Probe(target, pose, self.grid, time.monotonic(), ack)

    def tick(self):
        clock_now = self.get_clock().now()
        now = clock_now.nanoseconds
        reset = now < self.last_now
        self.last_now = now
        if reset:
            self.pose_stamp = 0
        pose = None if reset else self.localization(now)
        ready = pose is not None and self.grid is not None and bool(self.tasks) and self.client.server_is_ready()
        if not ready:
            self.plans.clear()
            self.cancel_probe()
        self.poll_probe(pose, now)
        if ready and self.probe is None:
            self.request_plan(pose, clock_now)
        if now <= 0:
            return
        msg = NavigationEvidence()
        msg.header.frame_id, msg.header.stamp = self.frame, clock_now.to_msg()
        if pose is not None:
            msg.localization_valid = True
            msg.localization_stamp = pose.header.stamp
        if ready:
            oldest = now
            for target, (stamp, start, distance) in self.plans.items():
                if not 0 <= now - stamp < PLAN_AGE_NS or math.dist(xy(pose), start) > REPLAN_DISTANCE_M:
                    continue
                point, (risk, radius) = self.points[target], self.tasks[target]
                msg.objectives.append(TacticalObjective(
                    id=target, kind=point.kind, terrain=TacticalObjective.GROUND, reachable=True,
                    route_verified=True, risk_known=True, risk=risk, travel_time_s=distance / self.speed,
                    x=point.x, y=point.y, radius=radius,
                    self_present=math.dist(xy(pose), (point.x, point.y)) <= radius))
                oldest = min(oldest, stamp)
            if msg.objectives:
                msg.objectives_valid = True
                msg.objectives_stamp = Time(nanoseconds=oldest).to_msg()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = None
    try:
        node = NavigationEvidenceNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
