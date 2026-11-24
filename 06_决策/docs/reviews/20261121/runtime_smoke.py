#!/usr/bin/env python3
"""Isolated DDS startup and synthetic input checks. execute_navigation stays false."""
import json
import signal
import struct
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rm2027_decision.msg import DecisionIntent, DecisionState, NavigationEvidence, TacticalObjective
from std_msgs.msg import UInt8MultiArray, String

OUT = Path(__file__).parent
sys.path.insert(0, str(Path('/home/wangjiaming/文档/GitHub/RM_11_13/06_决策/scripts')))
from referee_rx import crc


def frame(command, data):
    h = struct.pack('<BHB', 0xa5, len(data), 0)
    h += bytes([crc(h, 0xff, 0x8c)])
    h += struct.pack('<H', command) + data
    return h + struct.pack('<H', crc(h, 0xffff, 0x8408))


rclpy.init()
observer = Node('rm06_review_observer')
intents, states, statuses = [], [], []
observer.create_subscription(DecisionIntent, '/decision2027/intent', intents.append, 10)
observer.create_subscription(DecisionState, '/decision2027/state', states.append, 10)
observer.create_subscription(String, '/decision2027/navigation_status', lambda m: statuses.append(m.data), 10)
raw = observer.create_publisher(UInt8MultiArray, '/decision2027/referee_bytes', 10)
evidence_pub = observer.create_publisher(NavigationEvidence, '/decision2027/navigation_evidence', 10)


def pump(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(observer, timeout_sec=0.04)


def start(name, arguments):
    log = open(OUT / (name + '.log'), 'w')
    proc = subprocess.Popen(['ros2', 'launch', 'rm2027_decision'] + arguments,
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    return proc, log


def stop(proc, log):
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            __import__('os').killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=3)
    log.close()
    pump(0.2)


results = {}
try:
    for name, args, mode in [
        ('intents-default', ['intents.launch.py'], 'Disabled'),
        ('intents-enabled-no-input', ['intents.launch.py', 'enabled:=true'], 'WaitState'),
        ('three-nodes-default', ['rm_11_13.launch.py'], 'Disabled'),
    ]:
        intents.clear(); statuses.clear(); states.clear()
        proc, log = start(name, args)
        try:
            pump(4.0)
            assert proc.poll() is None, (name, 'launch exited early')
            assert any(m.mode == mode for m in intents), (name, [m.mode for m in intents])
            results[name] = {'observed_modes': sorted(set(m.mode for m in intents)),
                             'intent_messages': len(intents), 'state_messages': len(states),
                             'navigation_statuses': statuses[:10], 'passed': True}
        finally:
            stop(proc, log)

    tasks = OUT / 'synthetic_tasks.yaml'
    tasks.write_text('map_frame: map\nobjectives:\n  - {id: review_patrol, kind: patrol, x: 0.0, y: 0.0, yaw: 0.0}\n')
    intents.clear(); statuses.clear(); states.clear()
    proc, log = start('three-nodes-synthetic-input', [
        'rm_11_13.launch.py', 'enabled:=true', 'execute_navigation:=false',
        'tasks_file:=' + str(tasks)])
    try:
        pump(2.0)
        def publish(phase):
            robot = bytearray(17)
            robot[0] = 7
            struct.pack_into('<HHHHHfB', robot, 2, 400, 400, 30, 260, 100, 25.0, 7)
            game = bytes([(phase << 4) | 4]) + struct.pack('<HQ', 280, 0)
            raw.publish(UInt8MultiArray(data=list(frame(0x0001, game) + frame(0x0201, robot))))
            evidence = NavigationEvidence()
            stamp = observer.get_clock().now().to_msg()
            evidence.header.stamp = stamp
            evidence.header.frame_id = 'map'
            evidence.localization_valid = evidence.objectives_valid = True
            evidence.localization_stamp = evidence.objectives_stamp = stamp
            objective = TacticalObjective()
            objective.id = 'review_patrol'
            objective.kind = TacticalObjective.PATROL
            objective.reachable = objective.route_verified = objective.risk_known = True
            objective.risk = 0.1
            objective.radius = 0.5
            evidence.objectives.append(objective)
            evidence_pub.publish(evidence)
        for phase, duration in [(3, 0.8), (4, 2.0)]:
            end = time.monotonic() + duration
            while time.monotonic() < end:
                publish(phase)
                pump(0.08)
        assert any(m.mode == 'Patrol' and m.target == 'review_patrol' for m in intents)
        assert 'preview:review_patrol' in statuses, statuses
        assert not any(m.fire_allowed for m in intents)
        pump(3.0)
        assert intents[-1].mode == 'WaitState', intents[-1].mode
        results['three-nodes-synthetic-input'] = {
            'observed_modes': sorted(set(m.mode for m in intents)),
            'preview_target': 'preview:review_patrol' in statuses,
            'after_input_stops': intents[-1].mode,
            'fire_permission_frames': sum(bool(m.fire_allowed) for m in intents),
            'physical_uart_opened': False, 'execute_navigation': False, 'passed': True}
    finally:
        stop(proc, log)
finally:
    observer.destroy_node()
    rclpy.shutdown()

(OUT / 'runtime-results.json').write_text(json.dumps(results, ensure_ascii=False, indent=2))
print(json.dumps(results, ensure_ascii=False, indent=2))
