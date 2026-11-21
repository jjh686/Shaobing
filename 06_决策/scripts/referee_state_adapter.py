#!/usr/bin/env python3
"""Normalize official referee data and local navigation evidence into DecisionState."""

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


def read_only_uart(path):
    import termios
    import tty

    if not re.fullmatch(r'/dev/(?:serial/by-id/[^/]+|tty(?:USB|ACM|S|AMA)[0-9]+|pts/[0-9]+)', path):
        raise ValueError('Expected an explicit local referee UART device')
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY)
    try:
        tty.setraw(fd, termios.TCSANOW)
        settings = termios.tcgetattr(fd)
        settings[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE | termios.HUPCL)
        settings[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
        settings[4] = settings[5] = termios.B115200
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        termios.tcflush(fd, termios.TCIFLUSH)
        return fd
    except Exception:
        os.close(fd)
        raise


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
        self.receiver = Receiver()
        self.sources = {}
        self.evidence = None
        self.last_ros_time = None
        self.life = OwnLife()
        self.fd = read_only_uart(self.uart) if self.uart else None
        if not self.uart:
            self.create_subscription(UInt8MultiArray, 'referee_bytes',
                                     lambda msg: self.ingest(msg.data), qos_profile_sensor_data)
        self.create_subscription(NavigationEvidence, 'navigation_evidence',
                                 self.on_evidence, qos_profile_sensor_data)
        self.pub = self.create_publisher(DecisionState, 'state', 1)
        self.timer = self.create_timer(0.05, self.tick)

    def now(self):
        stamp = self.get_clock().now()
        if self.last_ros_time is not None and stamp.nanoseconds < self.last_ros_time:
            self.receiver = Receiver()
            self.sources.clear()
            self.evidence = None
            self.life = OwnLife()
        self.last_ros_time = stamp.nanoseconds
        return stamp

    @staticmethod
    def stamp_ns(stamp):
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
            if command == 0x0201:
                robot_id, hp, max_hp, _ = data
                if robot_id not in (7, 107) or (self.robot_id and self.robot_id != robot_id):
                    self.sources.pop(command, None)
                    self.life = OwnLife()
                    continue
                if not 0 < max_hp or hp > max_hp:
                    self.sources.pop(command, None)
                    continue
                self.life.robot(hp, now)
            elif command == 0x0001:
                if not self.recent(command, stamp.nanoseconds, 2.0):
                    self.life = OwnLife()
                self.life.game(*data)
            elif command == 0x0209:
                self.life.supply_tag(data, now)
            self.sources[command] = data, stamp.to_msg(), now

    def recent(self, command, now, seconds):
        sample = self.sources.get(command)
        if sample is None:
            return None
        if (0 <= time.monotonic() - sample[2] < seconds
                and 0 <= now - self.stamp_ns(sample[1]) < seconds * 1e9):
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
            if (source.position_valid and self.is_recent(source.position_stamp, now, 1)
                    and math.isfinite(source.x) and math.isfinite(source.y)):
                ally.x, ally.y = source.x, source.y
                ally.position_stamp, ally.position_valid = source.position_stamp, True
            # Official HP does not identify absence, penalty, max HP or weakness.
            if (health and source.health_valid and source.life_state_valid
                    and self.is_recent(source.health_stamp, now, 1)
                    and 0 < ally.hp <= source.max_hp):
                ally.max_hp = source.max_hp
                ally.life_state_valid, ally.health_valid = True, True
                ally.weakened = source.weakened
                if self.stamp_ns(source.health_stamp) < self.stamp_ns(ally.health_stamp):
                    ally.health_stamp = source.health_stamp

    def is_recent(self, stamp, now, seconds):
        source_ns = self.stamp_ns(stamp)
        return source_ns > 0 and 0 <= now - source_ns < seconds * 1e9

    def active_evidence(self):
        if self.evidence and time.monotonic() - self.evidence[1] < 1.0:
            return self.evidence[0]
        return None

    def fill_evidence(self, out, now):
        evidence = self.active_evidence()
        if evidence is None:
            return
        out.localization_stamp = evidence.localization_stamp
        out.localization_valid = (evidence.localization_valid and
                                  self.is_recent(out.localization_stamp, now, 1))
        if (not evidence.objectives_valid or
                not self.is_recent(evidence.objectives_stamp, now, 2)):
            return
        event = self.recent(0x0101, now, 2.0)
        out.objectives_stamp = evidence.objectives_stamp
        for objective in evidence.objectives:
            if objective.kind == TacticalObjective.ZONE:
                # Only the central ground zone has an official ownership mapping here.
                if objective.id != 'central_zone' or not event:
                    continue
                objective.owner = (1, 2, 3, 0)[event[0]]
                if self.stamp_ns(event[1]) < self.stamp_ns(out.objectives_stamp):
                    out.objectives_stamp = event[1]
            out.objectives.append(objective)
        out.objectives_valid = bool(out.objectives)

    def tick(self):
        if self.fd is not None:
            try:
                chunk = os.read(self.fd, 4096)
                if chunk:
                    self.ingest(chunk)
            except BlockingIOError:
                pass
            except OSError as exc:
                self.get_logger().error(f'Referee UART disconnected: {exc}')
                os.close(self.fd)
                self.fd = None
                self.sources.clear()
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
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
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
