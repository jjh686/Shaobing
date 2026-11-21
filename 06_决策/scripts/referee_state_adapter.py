#!/usr/bin/env python3
"""Normalize official referee data and local navigation evidence into DecisionState."""

from copy import deepcopy
import math
import os
import re
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import UInt8MultiArray

from referee_rx import OwnLife, Receiver, decode
from rm2027_decision.msg import DecisionState, NavigationEvidence, TacticalObjective


UART_PATH = re.compile(r'/dev/(?:serial/by-id/[^/]+|tty(?:USB|ACM|S|AMA)[0-9]+|pts/[0-9]+)')


def read_only_uart(path):
    import termios
    import tty

    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY)
    try:
        tty.setraw(fd, termios.TCSANOW)
        settings = termios.tcgetattr(fd)
        settings[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE | termios.HUPCL
                         | getattr(termios, 'CRTSCTS', 0))
        settings[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
        settings[4] = settings[5] = termios.B115200
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        termios.tcflush(fd, termios.TCIFLUSH)
        return fd
    except termios.error as exc:
        os.close(fd)
        raise OSError(*exc.args) from exc


class Adapter(Node):
    def __init__(self):
        super().__init__('referee_state_adapter')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.uart = self.declare_parameter('referee_uart', '').value
        self.robot_id = self.declare_parameter('robot_id', 0).value
        if self.robot_id not in (0, 7, 107) or not self.map_frame:
            raise ValueError('Invalid robot ID or map frame')
        if self.uart and self.robot_id == 0:
            raise ValueError('Dedicated UART requires confirmed robot_id 7 or 107')
        if self.uart and not UART_PATH.fullmatch(self.uart):
            raise ValueError('Expected an explicit local referee UART device')
        self.receiver = Receiver()
        self.sources = {}
        self.evidence = None
        self.last_ros_time = None
        self.life = OwnLife()
        self.fd = None
        self.uart_retry_at = 0.0
        self.uart_retry_delay = 1.0
        if not self.uart:
            self.create_subscription(UInt8MultiArray, 'referee_bytes',
                                     lambda msg: self.ingest(msg.data), qos_profile_sensor_data)
        self.create_subscription(NavigationEvidence, 'navigation_evidence',
                                 self.on_evidence, qos_profile_sensor_data)
        self.pub = self.create_publisher(DecisionState, 'state', 1)
        self.timer = self.create_timer(0.05, self.tick)

    def reset_referee(self):
        self.receiver.buffer.clear()
        self.sources.clear()
        self.life = OwnLife()

    def now(self):
        stamp = self.get_clock().now()
        if self.last_ros_time is not None and stamp.nanoseconds < self.last_ros_time:
            self.reset_referee()
            self.evidence = None
        self.last_ros_time = stamp.nanoseconds
        return stamp

    @staticmethod
    def stamp_ns(stamp):
        if not 0 <= stamp.nanosec < 1_000_000_000:
            return 0
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def on_evidence(self, evidence):
        now = self.now().nanoseconds
        stamp = self.stamp_ns(evidence.header.stamp)
        if evidence.header.frame_id != self.map_frame or not 0 < stamp <= now:
            return
        if self.evidence and stamp <= self.stamp_ns(self.evidence[0].header.stamp):
            return
        self.evidence = evidence, time.monotonic()

    def ingest(self, chunk):
        if len(chunk) > 4096:
            self.receiver.buffer.clear()
            return
        stamp = self.now()
        now = time.monotonic()
        for command, payload in self.receiver.feed(bytes(chunk)):
            data = decode(command, payload)
            game = self.recent(0x0001, stamp.nanoseconds, 2.0)
            if command == 0x0201:
                robot_id, hp, max_hp, _ = data
                previous_robot = self.sources.get(0x0201)
                if robot_id not in (7, 107) or (self.robot_id and self.robot_id != robot_id):
                    self.reset_referee()
                    return
                if previous_robot and previous_robot[0][0] != robot_id:
                    self.reset_referee()
                    return
                if not 0 < max_hp or hp > max_hp:
                    self.sources.pop(command, None)
                    self.life = OwnLife()
                    continue
                if game:
                    self.life.game(*game[0])
                    self.life.robot(hp, now)
                else:
                    self.life = OwnLife()
            elif command == 0x0001:
                if not game or game[0] != data:
                    for expired in (0x0003, 0x0101, 0x0209):
                        self.sources.pop(expired, None)
                if not game or (game[0] != data and (game[0], data) != ((4, 3), (4, 4))):
                    self.sources.pop(0x0201, None)
                    self.life = OwnLife()
                self.life.game(*data)
            elif command == 0x0209 and game:
                self.life.supply_tag(data, now)
            self.sources[command] = data, stamp.to_msg(), now

    def recent(self, command, now, seconds):
        sample = self.sources.get(command)
        if sample is None:
            return None
        stamp = self.stamp_ns(sample[1])
        if (stamp > 0 and 0 <= time.monotonic() - sample[2] < seconds
                and 0 <= now - stamp < seconds * 1e9):
            return sample
        return None

    def fill_teammates(self, out, now):
        health = self.recent(0x0003, now, 1.2)
        observations = self.active_evidence()
        for index, (name, ally) in enumerate((('heavy', out.heavy), ('infantry', out.infantry))):
            if health:
                ally.hp = health[0][index]
                ally.health_stamp = health[1]
            if observations is None:
                continue
            source = getattr(observations, name)
            if (source.position_valid and self.evidence_is_recent(source.position_stamp, observations, now, 1)
                    and math.isfinite(source.x) and math.isfinite(source.y)):
                ally.x, ally.y = source.x, source.y
                ally.position_stamp, ally.position_valid = source.position_stamp, True
            # Official HP does not identify absence, penalty, max HP or weakness.
            life_matches = (ally.hp == 0) == (source.hp == 0) and (ally.hp > 0 or not source.weakened)
            if (health and source.health_valid and source.life_state_valid and source.max_hp > 0
                    and self.evidence_is_recent(source.health_stamp, observations, now, 1)
                    and max(ally.hp, source.hp) <= source.max_hp and life_matches):
                ally.max_hp = source.max_hp
                ally.life_state_valid, ally.health_valid = True, True
                ally.weakened = source.weakened
                if self.stamp_ns(source.health_stamp) < self.stamp_ns(ally.health_stamp):
                    ally.health_stamp = source.health_stamp

    def evidence_is_recent(self, stamp, evidence, now, seconds):
        source_ns = self.stamp_ns(stamp)
        return (0 < source_ns <= self.stamp_ns(evidence.header.stamp)
                and 0 <= now - source_ns < seconds * 1e9)

    def active_evidence(self):
        if self.evidence and 0 <= time.monotonic() - self.evidence[1] < 1.0:
            return self.evidence[0]
        return None

    def fill_evidence(self, out, now):
        evidence = self.active_evidence()
        if evidence is None:
            return
        out.localization_stamp = evidence.localization_stamp
        out.localization_valid = (evidence.localization_valid and
                                  self.evidence_is_recent(out.localization_stamp, evidence, now, 1))
        if (not evidence.objectives_valid or
                not self.evidence_is_recent(evidence.objectives_stamp, evidence, now, 2)):
            return
        event = self.recent(0x0101, now, 2.0)
        out.objectives_stamp = evidence.objectives_stamp
        for objective in evidence.objectives:
            if objective.kind == TacticalObjective.ZONE:
                # Only the central ground zone has an official ownership mapping here.
                if objective.id != 'central_zone' or not event:
                    continue
                objective = deepcopy(objective)
                objective.owner = (1, 2, 3, 0)[event[0]]
                if self.stamp_ns(event[1]) < self.stamp_ns(out.objectives_stamp):
                    out.objectives_stamp = event[1]
            out.objectives.append(objective)
        out.objectives_valid = bool(out.objectives)

    def close_uart(self):
        if self.fd is not None:
            fd, self.fd = self.fd, None
            try:
                os.close(fd)
            except OSError as exc:
                self.get_logger().warning(f'Could not close referee UART: {exc}')

    def poll_uart(self):
        now = time.monotonic()
        if self.fd is None:
            if now < self.uart_retry_at:
                return
            try:
                self.fd = read_only_uart(self.uart)
            except OSError as exc:
                self.schedule_uart_retry(now)
                self.get_logger().warning(f'Waiting for referee UART: {exc}')
                return
            self.get_logger().info('Referee UART opened read-only')
        try:
            chunk = os.read(self.fd, 4096)
            if not chunk:
                raise OSError('End of referee UART input')
        except BlockingIOError:
            return
        except OSError as exc:
            self.get_logger().warning(f'Referee UART disconnected: {exc}')
            self.close_uart()
            self.reset_referee()
            self.schedule_uart_retry(now)
            return
        self.uart_retry_delay = 1.0
        self.ingest(chunk)

    def schedule_uart_retry(self, now):
        self.uart_retry_at = now + self.uart_retry_delay
        self.uart_retry_delay = min(5.0, 2 * self.uart_retry_delay)

    def tick(self):
        if self.uart:
            self.poll_uart()
        stamp = self.now()
        now = stamp.nanoseconds
        out = DecisionState()
        out.header.stamp, out.header.frame_id = stamp.to_msg(), self.map_frame
        game = self.recent(0x0001, now, 2.0)
        if game:
            (out.game_type, out.game_progress), out.game_stamp, _ = game
            out.game_valid = True
        robot = self.recent(0x0201, now, 0.75)
        if robot:
            (_, out.hp, out.max_hp, chassis_power), out.robot_stamp, _ = robot
            out.robot_valid = True
            out.life_state_valid = self.life.known
            out.weakened = self.life.weakened
            out.manual_override = not chassis_power and out.hp > 0
        self.fill_teammates(out, now)
        self.fill_evidence(out, now)
        self.pub.publish(out)

    def destroy_node(self):
        self.close_uart()
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = Adapter()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
