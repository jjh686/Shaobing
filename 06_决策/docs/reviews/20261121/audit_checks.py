#!/usr/bin/env python3
"""Review probes only: no physical UART and no real Nav2 server."""
import importlib.util
import json
import struct
import sys
import time
from pathlib import Path

import rclpy
from rm2027_decision.msg import DecisionState, NavigationEvidence

ROOT = Path('/home/wangjiaming/文档/GitHub/RM_11_13/06_决策')
OUT = Path(__file__).parent
sys.path.insert(0, str(ROOT / 'scripts'))
from referee_rx import Receiver, OwnLife, crc, decode
from referee_state_adapter import Adapter
from intent_nav_bridge import IntentNavBridge, Point


def packet(command, payload, seq=0):
    header = struct.pack('<BHB', 0xa5, len(payload), seq)
    header += bytes([crc(header, 0xff, 0x8c)])
    frame = header + struct.pack('<H', command) + payload
    return frame + struct.pack('<H', crc(frame, 0xffff, 0x8408))


results = {}
payload = bytes([0x44]) + bytes(10)
frame = packet(0x0001, payload)
receiver = Receiver()
decoded = []
for byte in b'noise' + frame:
    decoded.extend(receiver.feed(bytes([byte])))
assert decoded == [(0x0001, payload)]
corrupt = bytearray(frame)
corrupt[-1] ^= 1
assert list(Receiver().feed(corrupt)) == []
assert len(list(Receiver().feed(bytes(corrupt) + frame))) == 1
assert decode(0x0101, bytes(4) + struct.pack('<I', 2 << 26)) == 2
assert list(Receiver().feed(packet(0x0003, bytes(20)))) == []
results['receiver'] = {'fragmented_frame': 'pass', 'bad_crc_rejected': 'pass',
                       'resynchronization': 'pass', 'central_owner_bit': 'pass',
                       '2026_hp_length_rejected': 'pass'}

normal = OwnLife()
normal.game(4, 3)
normal.robot(400, 0.0)
normal.game(4, 4)
normal.robot(400, 0.1)
assert normal.known and not normal.weakened
normal.robot(0, 0.2)
normal.robot(80, 0.3)
assert normal.known and normal.weakened
normal.supply_tag(True, 0.4)
assert normal.known and not normal.weakened
results['observed_round_and_respawn'] = 'pass'

late = OwnLife()
late.game(4, 4)
late.robot(400, 1.0)
late.supply_tag(True, 1.1)
late.robot(400, 1.2)
assert not late.known
normal.robot(400, 2.0)
normal.supply_tag(True, 2.1)
normal.robot(400, 2.2)
assert not normal.known
results['unknown_life_does_not_recover_on_supply'] = {
    'late_start_known': late.known, 'after_gap_known': normal.known,
    'finding_reproduced': True}

rclpy.init()
adapter = Adapter()
bridge = IntentNavBridge()
try:
    stamp = adapter.get_clock().now().to_msg()
    now = adapter.get_clock().now().nanoseconds
    adapter.sources[0x0003] = ((0, 350, 400), stamp, time.monotonic())
    evidence = NavigationEvidence()
    evidence.header.stamp = stamp
    evidence.header.frame_id = 'map'
    for name, hp, maximum in [('heavy', 0, 450), ('infantry', 350, 350)]:
        ally = getattr(evidence, name)
        ally.health_valid = ally.life_state_valid = True
        ally.health_stamp = stamp
        ally.hp, ally.max_hp = hp, maximum
    adapter.evidence = evidence, time.monotonic()
    normalized = DecisionState()
    adapter.fill_teammates(normalized, now)
    assert normalized.heavy.hp == 0 and not normalized.heavy.health_valid
    assert normalized.infantry.health_valid
    results['confirmed_dead_teammate_dropped'] = {
        'heavy_hp': normalized.heavy.hp,
        'heavy_health_valid': normalized.heavy.health_valid,
        'infantry_health_valid': normalized.infantry.health_valid,
        'finding_reproduced': True}

    class Capture:
        def __init__(self): self.message = None
        def publish(self, message): self.message = message
    capture = Capture()
    adapter.pub = capture
    adapter.tick()
    assert not capture.message.ammunition_valid
    assert not capture.message.shooter_enabled
    assert 0x0202 not in __import__('referee_rx').LENGTHS
    assert 0x0208 not in __import__('referee_rx').LENGTHS
    results['shooting_evidence_unimplemented'] = {
        'ammunition_valid': capture.message.ammunition_valid,
        'shooter_enabled': capture.message.shooter_enabled}

    class DummyClient:
        def server_is_ready(self): return True
    bridge.execute = True
    bridge.client = DummyClient()
    sent = []
    bridge.send = sent.append
    bridge.select = lambda now_ns, wall: ('central_zone', 'approved')
    for outcome in ['aborted', 'rejected', 'succeeded']:
        bridge.active_target = bridge.last_attempt = 'central_zone'
        bridge.finish(outcome)
        bridge.tick()
        assert not sent
    bridge.active_target = bridge.last_attempt = 'central_zone'
    bridge.finish('canceled')
    bridge.tick()
    assert sent == ['central_zone']
    results['same_target_not_reissued'] = {
        'after_aborted': False, 'after_rejected': False,
        'after_succeeded_then_new_need': False,
        'after_canceled': True, 'finding_reproduced': True}
finally:
    adapter.destroy_node()
    bridge.destroy_node()
    rclpy.shutdown()

(OUT / 'audit-checks-results.json').write_text(json.dumps(results, ensure_ascii=False, indent=2))
print(json.dumps(results, ensure_ascii=False, indent=2))
