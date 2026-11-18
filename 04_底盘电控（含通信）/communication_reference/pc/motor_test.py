#!/usr/bin/env python3
"""架空轮子短时测试：STOP握手→低速指令→STOP；不需要ROS或雷达。"""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import secrets
import select
import signal
import termios
import time
import tty

from check_port import check
from usb_protocol import Stream, command

DIRECTIONS = {
    'forward': (1, 0, 0), 'back': (-1, 0, 0),
    'left': (0, 1, 0), 'right': (0, -1, 0),
    'ccw': (0, 0, 1), 'cw': (0, 0, -1),
}
FAULTS = {1: '运动锁', 2: '遥控器/导航档未许可', 4: '四电机未全部在线',
          8: '坐标开关未解锁', 16: 'CAN接收故障'}


class Link:
    def __init__(self, fd):
        self.fd = fd
        self.parser = Stream()
        self.session = secrets.randbelow(0xffffffff) + 1
        self.sequence = 0
        self.started = time.monotonic()
        self.last_uptime = None
        self.last = None

    def phase(self, duration, velocity=(0., 0., 0.), enabled=False, ready=False):
        start = time.monotonic()
        next_send = start
        last_ack = None
        count = 0
        sent = {}
        while time.monotonic() - start < duration:
            now = time.monotonic()
            if enabled and now - (last_ack if last_ack is not None else start) > .15:
                raise RuntimeError('运动阶段150ms内没有新鲜底盘ACK，已停止发送ENABLE')
            if now >= next_send:
                seq = self.sequence
                frame = command(self.session, seq, int((now-self.started)*1000),
                                enabled=enabled, ttl=100 if enabled else 0,
                                vx=velocity[0], vy=velocity[1], wz=velocity[2])
                # A missed deadline must not enqueue another enabled packet.
                offset = 0
                deadline = time.monotonic() + .02
                while offset < len(frame):
                    remaining = deadline - time.monotonic()
                    if remaining <= 0 or not select.select([], [self.fd], [], remaining)[1]:
                        raise RuntimeError('USB写入超时')
                    try:
                        offset += os.write(self.fd, frame[offset:])
                    except BlockingIOError:
                        continue
                sent[seq] = time.monotonic()
                self.sequence = (seq + 1) & 65535
                sent = {s: t for s, t in sent.items() if now-t < .15}
                next_send = time.monotonic() + .02
            if not select.select([self.fd], [], [], .005)[0]:
                continue
            try:
                data = os.read(self.fd, 4096)
            except BlockingIOError:
                continue
            if not data:
                raise RuntimeError('USB已断开；无法确认STOP到达')
            for f in self.parser.push(data):
                self.last = f
                now = time.monotonic()
                if f['session'] != self.session or not f['status'] & 64 or \
                        f['sequence'] not in sent or now-sent[f['sequence']] > .15:
                    continue
                if self.last_uptime is not None and not \
                        0 < ((f['uptime']-self.last_uptime)&0xffffffff) < 0x80000000:
                    continue
                self.last_uptime = f['uptime']
                if bool(f['status'] & 2) != enabled:
                    raise RuntimeError('底盘ACTIVE与本阶段命令不一致')
                permitted = f['fault'] == 0 and f['status'] & 9 == 9 and not f['status'] & 52
                if enabled and not permitted:
                    raise RuntimeError('底盘撤销许可：' + json.dumps(f, ensure_ascii=False))
                if ready and not permitted:
                    count = 0
                    continue
                last_ack = now
                count += 1
        if count < (10 if ready else 3) or last_ack is None or time.monotonic()-last_ack > .15:
            detail = ''
            if self.last:
                detail = '；' + ', '.join(v for k, v in FAULTS.items() if self.last['fault'] & k)
                detail += '；最后反馈=' + json.dumps(self.last, ensure_ascii=False)
            raise RuntimeError('没有持续的新鲜底盘许可/ACK' + detail)
        return count


def run(port, direction='forward', duration=1., speed=None, wheels_raised=False, report_path=None):
    if not wheels_raised:
        raise ValueError('请先架空四轮，确认后加 --wheels-raised')
    if direction not in DIRECTIONS or not math.isfinite(duration) or not .2 <= duration <= 10:
        raise ValueError('方向无效或时长超限（0.2～10秒）')
    angular = direction in ('ccw', 'cw')
    speed = (.1 if angular else .05) if speed is None else speed
    if not math.isfinite(speed) or not 0 < speed <= (.2 if angular else .1):
        raise ValueError('平移速度须为(0,0.1]m/s，旋转速度须为(0,0.2]rad/s')
    port = check(port)
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    exclusive = False
    link = None
    report = {'port': port, 'direction': direction, 'duration': duration,
              'speed': speed, 'passed': False, 'stop_confirmed': False}
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
        print('先发送1秒STOP，核对遥控许可、四电机在线和本轮底盘ACK。', flush=True)
        report['ready_ACK'] = link.phase(1., ready=True)
        velocity = tuple(speed * x for x in DIRECTIONS[direction])
        print(f'低速测试 {direction}：{velocity}，持续{duration}秒。Ctrl+C停止。', flush=True)
        report['active_ACK'] = link.phase(duration, velocity, enabled=True)
        report['passed'] = True
    except BaseException as exc:
        report['error'] = str(exc) or type(exc).__name__
        raise
    finally:
        if link is not None:
            try:
                report['stop_ACK'] = link.phase(.6)
                report['stop_confirmed'] = True
                print('STOP已收到新鲜底盘确认；轮子可能因惯性继续转动。', flush=True)
            except BaseException as exc:
                report['stop_error'] = str(exc) or type(exc).__name__
                report['passed'] = False
                print('STOP未确认，请断开电机主电源。' + report['stop_error'], flush=True)
            report['last_feedback'] = link.last
            report['frames'] = link.parser.frames
        if exclusive:
            try:
                fcntl.ioctl(fd, termios.TIOCNXCL)
            except OSError:
                pass
        os.close(fd)
        if report_path:
            path = Path(report_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    if not report['stop_confirmed']:
        raise RuntimeError('测试结束但未确认STOP，需断开电机主电源')
    print('PASS：底盘接受了短时速度命令并返回STOP；请现场核对每个轮子的转向。', flush=True)
    return report


def interrupted(signum, frame):
    raise KeyboardInterrupt('收到终止信号')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port')
    parser.add_argument('--direction', choices=DIRECTIONS, default='forward')
    parser.add_argument('--duration', type=float, default=1.,
                        help='运动阶段时长0.2～10秒；另有1秒STOP预检和0.6秒STOP收尾')
    parser.add_argument('--speed', type=float, help='默认平移0.05m/s；旋转0.1rad/s')
    parser.add_argument('--wheels-raised', action='store_true', help='确认四轮已架空')
    parser.add_argument('--report')
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, interrupted)
    try:
        run(args.port, args.direction, args.duration, args.speed, args.wheels_raised, args.report)
    except (OSError, ValueError, RuntimeError, KeyboardInterrupt) as exc:
        raise SystemExit('FAIL: ' + (str(exc) or '用户中止'))
