#!/usr/bin/env python3
"""Convert validated onboard decision intents into local Nav2 goals."""

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rm2027_decision.msg import DecisionIntent, DecisionState, TacticalObjective
from std_msgs.msg import String


KIND_BY_NAME = {
    'recovery': TacticalObjective.RECOVERY,
    'zone': TacticalObjective.ZONE,
    'defence': TacticalObjective.DEFENCE,
    'patrol': TacticalObjective.PATROL,
}
MODE_KINDS = {
    'Resupply': {TacticalObjective.RECOVERY},
    'Cover': {TacticalObjective.ZONE, TacticalObjective.DEFENCE},
    'Capture': {TacticalObjective.ZONE},
    'Patrol': {TacticalObjective.PATROL},
}


@dataclass(frozen=True)
class Point:
    kind: int
    x: float
    y: float
    yaw: float


@dataclass
class Observation:
    message: object
    source_ns: int
    received: float


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def load_points(filename):
    if not filename:
        return 'map', {}
    data = yaml.safe_load(Path(filename).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('Task file must be a mapping')
    frame = data.get('map_frame', 'map')
    if not isinstance(frame, str) or not frame.strip() or len(frame) > 64:
        raise ValueError('Invalid map_frame in task file')
    objectives = data.get('objectives', [])
    if not isinstance(objectives, list) or len(objectives) > 16:
        raise ValueError('Task file must contain at most 16 objectives')
    points = {}
    for item in objectives:
        if not isinstance(item, dict):
            raise ValueError('Each objective must be a mapping')
        target = item.get('id')
        if not isinstance(target, str) or not target.strip() or len(target) > 64 or target in points:
            raise ValueError('Objective ids must be nonempty, unique, and at most 64 characters')
        kind = KIND_BY_NAME.get(item.get('kind'))
        if kind is None:
            raise ValueError(f'Invalid objective kind for {target}')
        coords = [item.get('x'), item.get('y'), item.get('yaw', 0.0)]
        if not all(type(value) in (int, float) and math.isfinite(value) for value in coords):
            raise ValueError(f'Invalid pose for {target}')
        points[target] = Point(kind, *map(float, coords))
    return frame, points


class IntentNavBridge(Node):
    def __init__(self):
        super().__init__('intent_nav_bridge')
        self.execute = self.fixed('execute_navigation', False)
        self.frame, self.points = load_points(self.fixed('tasks_file', ''))
        self.intent_timeout = self.fixed('intent_timeout', 0.5)
        self.state_timeout = self.fixed('source_timeout', 2.5)
        self.max_task_risk = self.fixed('max_task_risk', 0.50)
        self.max_recovery_risk = self.fixed('max_recovery_risk', 0.50)
        self.cautious_capture_risk = self.fixed('cautious_capture_risk', 0.25)
        for timeout in (self.intent_timeout, self.state_timeout):
            if not math.isfinite(timeout) or timeout <= 0:
                raise ValueError('Timeouts must be positive and finite')
        for risk in (self.max_task_risk, self.max_recovery_risk, self.cautious_capture_risk):
            if not math.isfinite(risk) or not 0.0 <= risk <= 1.0:
                raise ValueError('Risk limits must be in [0, 1]')
        if self.execute and not self.points:
            raise ValueError('Live navigation requires a task file with approved points')

        action = self.fixed('navigation_action', '/red_standard_robot1/navigate_to_pose')
        self.client = ActionClient(self, NavigateToPose, action) if self.execute else None
        self.status_pub = self.create_publisher(String, 'navigation_status', 10)
        self.create_subscription(DecisionIntent, 'intent', self.receive_intent, 1)
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(DecisionState, 'state', self.receive_state, state_qos)

        self.intent: Optional[Observation] = None
        self.state: Optional[Observation] = None
        self.source_times = {}
        self.last_clock_ns = 0
        self.last_status = ''
        self.last_attempt = None
        self.active_target = None
        self.goal_handle = None
        self._stopping = False
        self._cancel_pending = False
        self._cancel_accepted = False
        self._cancel_retry_at = 0.0
        # Expire commands even if simulation time pauses.
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.1, self.tick, clock=self.watchdog_clock)

    def fixed(self, name, value):
        return self.declare_parameter(
            name, value, ParameterDescriptor(read_only=True)
        ).value

    def observe_clock(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self.last_clock_ns:
            self.intent = None
            self.state = None
            self.source_times.clear()
            self.stop_active()
        self.last_clock_ns = now_ns
        return now_ns

    def receive_intent(self, message):
        now_ns = self.observe_clock()
        sample_ns = stamp_ns(message.header.stamp)
        if self.accept_stamp(sample_ns, now_ns, self.intent_timeout, self.intent):
            self.intent = Observation(message, sample_ns, time.monotonic())

    def receive_state(self, message):
        now_ns = self.observe_clock()
        sample_ns = stamp_ns(message.header.stamp)
        if not self.accept_stamp(sample_ns, now_ns, self.state_timeout, self.state):
            return
        wall = time.monotonic()
        # Republishing an unchanged source stamp does not renew its wall-clock lifetime.
        for name in ('game', 'robot', 'localization', 'objectives'):
            source_ns = stamp_ns(getattr(message, name + '_stamp'))
            prior = self.source_times.get(name)
            if getattr(message, name + '_valid') and 0 < source_ns <= sample_ns:
                if prior is None or source_ns > prior[0]:
                    self.source_times[name] = (source_ns, wall)
        self.state = Observation(message, sample_ns, wall)

    @staticmethod
    def accept_stamp(sample_ns, now_ns, timeout, previous):
        return (
            sample_ns > 0 and 0 <= now_ns - sample_ns < timeout * 1e9
            and (previous is None or sample_ns > previous.source_ns)
        )

    @staticmethod
    def recent(observation, now_ns, wall, timeout):
        return (
            observation is not None
            and 0 <= now_ns - observation.source_ns < timeout * 1e9
            and wall - observation.received < timeout
        )

    def source_recent(self, message, name, now_ns, wall):
        source_ns = stamp_ns(getattr(message, name + '_stamp'))
        sample = self.source_times.get(name)
        return (
            getattr(message, name + '_valid') and sample is not None and source_ns == sample[0]
            and 0 <= now_ns - source_ns < self.state_timeout * 1e9
            and wall - sample[1] < self.state_timeout
        )

    def select(self, now_ns, wall):
        if not self.recent(self.intent, now_ns, wall, self.intent_timeout):
            return None, 'stale_intent'
        intent = self.intent.message
        if intent.mode not in MODE_KINDS:
            return None, f'intent:{intent.mode}'
        if not self.recent(self.state, now_ns, wall, self.state_timeout):
            return None, 'stale_state'
        state = self.state.message
        if (
            state.header.frame_id != self.frame or state.manual_override
            or (state.game_type, state.game_progress) != (4, 4)
            or not state.life_state_valid or not 0 < state.hp <= state.max_hp
            or (state.weakened and intent.mode != 'Resupply')
        ):
            return None, 'robot_or_game_ineligible'
        if not all(self.source_recent(state, part, now_ns, wall)
                   for part in ('game', 'robot', 'localization', 'objectives')):
            return None, 'stale_source'
        point = self.points.get(intent.target)
        matching = [obj for obj in state.objectives if obj.id == intent.target]
        if point is None or len(matching) != 1 or point.kind not in MODE_KINDS[intent.mode]:
            return None, 'unknown_or_mismatched_target'
        goal = matching[0]
        budget = self.max_recovery_risk if intent.mode == 'Resupply' else self.max_task_risk
        if intent.mode == 'Capture':
            if goal.owner not in (DecisionState.OWNER_NEUTRAL, DecisionState.OWNER_ENEMY):
                return None, 'capture_requires_neutral_or_enemy_zone'
            # The policy applies the stricter cap if either ally is unknown or both
            # are unavailable. The original selector determines this status.
            category = intent.team_situation.split('|', 1)[0]
            if category in ('unknown', 'allies_recovering', 'all_low'):
                budget = min(budget, self.cautious_capture_risk)
                if goal.owner != DecisionState.OWNER_NEUTRAL:
                    return None, 'capture_requires_neutral_zone'
        if (
            goal.kind != point.kind or goal.terrain != TacticalObjective.GROUND
            or not goal.reachable or not goal.route_verified
            or goal.route_uses_highland or goal.route_uses_tunnel or not goal.risk_known
            or not 0 <= goal.risk <= budget
            or not math.isclose(goal.x, point.x, rel_tol=0.0, abs_tol=1e-6)
            or not math.isclose(goal.y, point.y, rel_tol=0.0, abs_tol=1e-6)
        ):
            return None, 'route_or_coordinates_unverified'
        return intent.target, 'approved'

    def report(self, text):
        if text != self.last_status:
            self.last_status = text
            self.status_pub.publish(String(data=text))
            self.get_logger().info(text)

    def tick(self):
        target, reason = self.select(self.observe_clock(), time.monotonic())
        if not self.execute:
            self.report(f'preview:{target}' if target else f'waiting:{reason}')
            return
        if self.active_target is not None:
            if target != self.active_target or self._stopping:
                self.stop_active()
                self.report(f'stopping:{self.active_target}')
            return
        if target is None:
            self.report(f'waiting:{reason}')
            return
        if target == self.last_attempt:
            return
        if not self.client.server_is_ready():
            self.report('waiting:navigate_to_pose_unavailable')
            return
        self.send(target)

    def send(self, target):
        point = self.points[target]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = point.x
        goal.pose.pose.position.y = point.y
        goal.pose.pose.orientation.z = math.sin(point.yaw / 2)
        goal.pose.pose.orientation.w = math.cos(point.yaw / 2)
        self.active_target = self.last_attempt = target
        self.report(f'sending:{target}')
        try:
            self.client.send_goal_async(goal).add_done_callback(self.accepted)
        except Exception as exc:
            self.report(f'goal_transport_error:{exc}')

    def accepted(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            # Unknown outcome: keep the slot occupied to avoid overlapping goals.
            self.report(f'goal_transport_error:{exc}')
            return
        if not handle.accepted:
            self.finish('rejected')
            return
        self.goal_handle = handle
        try:
            handle.get_result_async().add_done_callback(self.result)
        except Exception as exc:
            self.report(f'result_transport_error:{exc}')
            self.stop_active()
            return
        target, _ = self.select(self.observe_clock(), time.monotonic())
        if self._stopping or target != self.active_target:
            self.stop_active()
        else:
            self.report(f'active:{self.active_target}')

    def stop_active(self):
        if self.active_target is None:
            return
        self._stopping = True
        if self.goal_handle is None or self._cancel_pending or self._cancel_accepted:
            return
        now = time.monotonic()
        if now < self._cancel_retry_at:
            return
        self._cancel_pending = True
        self._cancel_retry_at = now + 0.5
        handle = self.goal_handle
        try:
            handle.cancel_goal_async().add_done_callback(lambda future: self.canceled(handle, future))
        except Exception as exc:
            self._cancel_pending = False
            self.report(f'cancel_transport_error:{exc}')

    def canceled(self, handle, future):
        if handle is not self.goal_handle:
            return
        self._cancel_pending = False
        try:
            self._cancel_accepted = bool(future.result().goals_canceling)
        except Exception as exc:
            self.report(f'cancel_transport_error:{exc}')
        # Hold the slot until Nav2 reports a terminal result, even if cancellation
        # was acknowledged or the server rejected it.

    def result(self, future):
        try:
            code = future.result().status
        except Exception as exc:
            self.report(f'result_transport_error:{exc}')
            return
        outcome = {
            GoalStatus.STATUS_SUCCEEDED: 'succeeded',
            GoalStatus.STATUS_CANCELED: 'canceled',
            GoalStatus.STATUS_ABORTED: 'aborted',
        }.get(code)
        if outcome is None:
            self.report(f'unknown_goal_status:{code}')
            return
        self.finish(outcome)

    def finish(self, outcome):
        self.report(f'{outcome}:{self.active_target}')
        if outcome == 'canceled':
            # An interrupted goal can be resumed once it becomes valid again.
            self.last_attempt = None
        self.active_target = None
        self.goal_handle = None
        self._stopping = False
        self._cancel_pending = False
        self._cancel_accepted = False
        self._cancel_retry_at = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = IntentNavBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop_active()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
