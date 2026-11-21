#!/usr/bin/env python3
"""MID-360点云触发单次底盘前进；仅验证链路，不进行障碍判断或导航。"""
import argparse
import fcntl
import ipaddress
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import termios
import time
import tty

from check_port import check
from motor_test import Link, interrupted

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TOPIC = '/lidar_link_test/livox/lidar'


def cloud_points(msg):
    """Validate the layout and sample bounded XYZ data, including padded rows."""
    if msg.width <= 0 or msg.height <= 0 or msg.point_step <= 0 or \
            msg.row_step < msg.width * msg.point_step or \
            len(msg.data) != msg.row_step * msg.height:
        raise ValueError('点云为空或数据布局无效')
    fields = {}
    for name in ('x', 'y', 'z'):
        matches = [field for field in msg.fields if field.name == name]
        if len(matches) != 1:
            raise ValueError('点云缺少唯一的 XYZ 字段')
        field = matches[0]
        code = {7: 'f', 8: 'd'}.get(field.datatype)
        if code is None or field.count != 1 or field.offset < 0 or \
                field.offset + struct.calcsize(code) > msg.point_step:
            raise ValueError('点云 XYZ 字段格式无效')
        fields[name] = (field.offset, ('>' if msg.is_bigendian else '<') + code)
    total, valid = msg.width * msg.height, 0
    samples = min(total, 256)
    for i in range(samples):
        row, col = divmod(i * total // samples, msg.width)
        offset = row * msg.row_step + col * msg.point_step
        xyz = [struct.unpack_from(fmt, msg.data, offset + shift)[0]
               for shift, fmt in fields.values()]
        if all(math.isfinite(value) for value in xyz) and sum(v*v for v in xyz) > .0001:
            valid += 1
    return total, valid


class LidarWatchdog:
    """Once armed, any interruption latches a failure until the process exits."""
    def __init__(self, timeout=.15, min_points=20, min_frames=5):
        self.timeout = timeout
        self.min_points, self.min_frames = min_points, min_frames
        self.last_received = None
        self.last_stamp = 0
        self.consecutive = self.accepted = self.rejected = 0
        self.points = self.sample_valid_points = 0
        self.reason = '尚未收到有效点云'
        self.armed = False
        self.latched = None
        self.previous_clock = None

    def reject(self, reason):
        self.rejected += 1
        self.consecutive = 0
        self.reason = reason
        self.last_received = None
        if self.armed and self.latched is None:
            self.latched = reason
        return False

    def clock_ok(self, ros_ns, mono):
        previous = self.previous_clock
        self.previous_clock = ros_ns, mono
        if previous and abs((ros_ns - previous[0]) * 1e-9 - (mono - previous[1])) > .1:
            return self.reject('系统时钟跳变，点云时间无法确认')
        return True

    def receive(self, msg, ros_ns, mono):
        if not self.clock_ok(ros_ns, mono):
            return False
        stamp = msg.header.stamp
        source_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age = (ros_ns - source_ns) * 1e-9
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000 or source_ns <= 0 or \
                age < -.005 or age >= self.timeout:
            return self.reject('点云原始时间戳过期、超前或无效')
        if source_ns <= self.last_stamp:
            return self.reject('点云时间戳重复或倒退')
        if msg.header.frame_id != 'front_mid360':
            return self.reject('点云 frame_id 不是测试驱动的 front_mid360')
        try:
            total, valid = cloud_points(msg)
        except (ValueError, struct.error) as exc:
            return self.reject(str(exc))
        if valid < self.min_points:
            return self.reject('点云中有效非零 XYZ 点不足')
        if self.last_received is not None and \
                (mono - self.last_received >= self.timeout or
                 (source_ns - self.last_stamp) * 1e-9 >= self.timeout):
            self.reject('点云流中断后恢复；本次运动不会自动恢复')
        self.last_stamp, self.last_received = source_ns, mono
        self.points, self.sample_valid_points = total, valid
        self.accepted += 1
        self.consecutive += 1
        self.reason = ''
        return True

    def require_fresh(self, ros_ns, mono):
        self.clock_ok(ros_ns, mono)
        if self.latched:
            raise RuntimeError(self.latched)
        if self.last_received is None:
            raise RuntimeError(self.reason)
        age = (ros_ns - self.last_stamp) * 1e-9
        if mono - self.last_received >= self.timeout or age >= self.timeout or age < -.005:
            self.reject('点云中断或原始数据超时，停止本次测试')
            raise RuntimeError(self.reason)
        if self.consecutive < self.min_frames:
            raise RuntimeError(f'等待连续 {self.min_frames} 帧有效点云')

    def arm(self, ros_ns, mono):
        self.require_fresh(ros_ns, mono)
        self.armed = True

    def summary(self):
        return dict(accepted=self.accepted, rejected=self.rejected,
                    points=self.points, sampled_valid_points=self.sample_valid_points,
                    last_source_stamp_ns=self.last_stamp, failure=self.latched or self.reason)


class LidarInput:
    def __init__(self, topic, timeout):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import PointCloud2
        self.ros = rclpy
        self.node = rclpy.create_node('lidar_link_test_input')
        self.watchdog = LidarWatchdog(timeout)
        self.subscription = self.node.create_subscription(
            PointCloud2, topic,
            lambda msg: self.watchdog.receive(msg, time.time_ns(), time.monotonic()),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE))

    def pump(self):
        if not self.ros.ok():
            raise RuntimeError('ROS上下文已停止')
        # Bounded work; a fast publisher must not starve USB STOP/ACK processing.
        for _ in range(4):
            self.ros.spin_once(self.node, timeout_sec=0.)

    def guard(self):
        self.pump()
        self.watchdog.require_fresh(time.time_ns(), time.monotonic())

    def wait(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.ros.spin_once(self.node, timeout_sec=.02)
            try:
                self.watchdog.require_fresh(time.time_ns(), time.monotonic())
            except RuntimeError:
                continue
            return
        raise RuntimeError('等待实时雷达点云超时：' + self.watchdog.reason)


def move_once(args, sensor, report):
    port = check(args.port)
    report['port'] = port
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    exclusive, link = False, None
    peak_vx, last_active = 0., None

    def active_ack(feedback):
        nonlocal peak_vx, last_active
        peak_vx = max(peak_vx, feedback['vx'])
        last_active = dict(feedback)

    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        exclusive = True
        tty.setraw(fd)
        attrs = termios.tcgetattr(fd)
        attrs[4] = attrs[5] = termios.B115200
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        link = Link(fd)
        sensor.watchdog.arm(time.time_ns(), time.monotonic())
        print('点云有效；发送1秒STOP，检查遥控许可、四电机和本轮ACK。', flush=True)
        report['ready_ACK'] = link.phase(1., ready=True, poll=sensor.guard)
        print(f'仅本次前进：vx={args.speed}m/s，持续{args.duration}s。Ctrl+C停止。', flush=True)
        report['active_ACK'] = link.phase(
            args.duration, (args.speed, 0., 0.), enabled=True,
            poll=sensor.guard, on_ack=active_ack)
        sensor.guard()
        report['command_accepted'] = True
        report['motion_observed'] = peak_vx >= min(.005, args.speed * .2)
        if not report['motion_observed']:
            raise RuntimeError('底盘接受了命令，但未检测到前向轮速；请检查电机反馈及供电')
        report['passed'] = True
    finally:
        report['peak_measured_vx'] = peak_vx
        report['last_active_feedback'] = last_active
        if link is not None:
            try:
                # STOP must work even when the ROS context or lidar has failed.
                report['stop_ACK'] = link.phase(.6)
                report['stop_confirmed'] = True
                print('已收到本轮STOP确认。', flush=True)
            except BaseException as exc:
                report['stop_error'] = str(exc) or type(exc).__name__
                report['passed'] = False
                print('STOP未确认，请用遥控急停或切断电机电源。', flush=True)
            report['last_feedback'] = link.last
            report['stream_diagnostics'] = link.parser.diagnostics
            report['ack_diagnostics'] = link.ack_stats.report()
        if exclusive:
            try:
                fcntl.ioctl(fd, termios.TIOCNXCL)
            except OSError:
                pass
        os.close(fd)
    if not report['stop_confirmed']:
        raise RuntimeError('测试结束但未确认STOP')


def run_driver(args):
    host, lidar = ipaddress.IPv4Address(args.host_ip), ipaddress.IPv4Address(args.lidar_ip)
    if any(ip.is_multicast or ip.is_unspecified or ip.is_loopback for ip in (host, lidar)) or host == lidar:
        raise ValueError('请填写互不相同的实际网卡IPv4地址和雷达IPv4地址')
    networks = json.loads(subprocess.check_output(['ip', '-j', '-4', 'address'], text=True))
    assigned = {address['local'] for interface in networks for address in interface.get('addr_info', [])}
    if str(host) not in assigned:
        raise ValueError(f'本机网卡尚未配置 {host}；请连接雷达网线并配置网卡，或用 --host-ip 指定实际地址')
    runtime = Path(args.runtime)
    executable = runtime/'install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node'
    if not executable.is_file():
        raise ValueError('雷达驱动尚未构建，请先运行 bash tools/lidar_link_test.sh build')
    template = ROOT.parent/'02_导航（含通信）/外部依赖源码/livox_ros_driver2/config/MID360_config.json'
    config = json.loads(template.read_text())
    for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
        config['Mid360s']['host_net_info'][key] = str(host)
    config['lidar_configs'][0]['ip'] = str(lidar)
    output = runtime/'mid360_link_test.json'
    output.write_text(json.dumps(config, indent=2)+'\n')
    print(f'MID-360 {lidar} → 本机 {host} → PointCloud2 {args.topic}', flush=True)
    os.execv(str(executable), [str(executable), '--ros-args',
             '-r', '__node:=lidar_link_test_driver', '-r', 'livox/lidar:=' + args.topic,
             '-r', 'livox/imu:=/lidar_link_test/livox/imu',
             '-p', 'user_config_path:=' + str(output), '-p', 'xfer_format:=0',
             '-p', 'multi_topic:=0', '-p', 'publish_freq:=20.0', '-p', 'data_src:=0',
             '-p', 'output_data_type:=0', '-p', 'frame_id:=front_mid360'])


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest='mode', required=True)
    for name in ('driver', 'observe', 'move'):
        mode = modes.add_parser(name)
        mode.add_argument('--topic', default=DEFAULT_TOPIC)
        mode.add_argument('--runtime', default=str(Path.home()/'.cache/rm_lidar_link_test'))
        if name == 'driver':
            mode.add_argument('--host-ip', default='192.168.1.50')
            mode.add_argument('--lidar-ip', default='192.168.1.106')
        else:
            mode.add_argument('--report', default=str(ROOT/'logs'/f'lidar-link-{name}-last.json'))
            mode.add_argument('--wait-seconds', type=float, default=10.)
            mode.add_argument('--lidar-timeout', type=float, default=.15)
            if name == 'observe':
                mode.add_argument('--observe-seconds', type=float, default=5.)
            else:
                mode.add_argument('--port', required=True)
                mode.add_argument('--speed', type=float, default=.05)
                mode.add_argument('--duration', type=float, default=1.)
    args = parser.parse_args()
    if not args.topic.startswith('/'):
        parser.error('--topic 请填写完整的ROS话题路径')
    if args.mode != 'driver':
        for value, low, high, label in (
            (args.lidar_timeout, .10, .30, '点云超时'),
            (args.wait_seconds, .2, 60., '点云等待时长'),
            (getattr(args, 'observe_seconds', 5.), .1, 60., '观察时长'),
            (getattr(args, 'speed', .05), .01, .05, '前进速度'),
            (getattr(args, 'duration', 1.), .2, 2., '运动时长')):
            if not math.isfinite(value) or not low <= value <= high:
                parser.error(f'{label}须在[{low}, {high}]范围内')
    return args


def main():
    args = arguments()
    signal.signal(signal.SIGTERM, interrupted)
    if args.mode == 'driver':
        try:
            run_driver(args)
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            print('FAIL: ' + str(exc), flush=True)
            return 1
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    report = dict(mode=args.mode, topic=args.topic, passed=False,
                  stop_confirmed=False, command_accepted=False, motion_observed=False)
    sensor = None
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        sensor = LidarInput(args.topic, args.lidar_timeout)
        print(f'等待连续5帧实时有效PointCloud2：{args.topic}', flush=True)
        sensor.wait(args.wait_seconds)
        if args.mode == 'move':
            report.update(speed=args.speed, duration=args.duration)
            move_once(args, sensor, report)
        else:
            sensor.watchdog.arm(time.time_ns(), time.monotonic())
            deadline = time.monotonic() + args.observe_seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(sensor.node, timeout_sec=.005)
                sensor.guard()
            report['passed'] = True
            report['stop_confirmed'] = None  # Observe never opens the MCU USB.
    except (OSError, ValueError, RuntimeError, KeyboardInterrupt) as exc:
        report['passed'] = False
        report['error'] = str(exc) or '用户中止'
        print('FAIL: ' + report['error'], flush=True)
    finally:
        if sensor is not None:
            report['lidar'] = sensor.watchdog.summary()
            sensor.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        output = Path(args.report)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print('报告：' + str(output), flush=True)
    if report['passed']:
        print('PASS：' + ('点云触发短时前进、轮速反馈和STOP均已验证。' if args.mode == 'move'
                         else '实时点云持续有效；此模式没有打开底盘USB。'), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
