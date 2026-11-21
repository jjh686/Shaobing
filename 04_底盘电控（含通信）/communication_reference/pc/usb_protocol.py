"""New navigation USB codec. SI units; route 2; little endian IEEE754 binary32."""
import math
import struct


def crc(data, initial=65535, polynomial=0x8408, mask=65535):
    value = initial
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (polynomial if value & 1 else 0)
    return value & mask


def command(session, sequence, uptime, *, enabled=False, ttl=0, vx=0., vy=0., wz=0.):
    if not 0 < session <= 0xffffffff or not 0 <= sequence <= 65535:
        raise ValueError('Invalid session/sequence')
    if not all(math.isfinite(x) for x in (vx, vy, wz)):
        raise ValueError('Non-finite velocity')
    if enabled:
        if not 40 <= ttl <= 150 or math.hypot(vx, vy) > .3 or abs(wz) > .5:
            raise ValueError('Commissioning speed/TTL limit')
    elif ttl or vx or vy or wz:
        raise ValueError('STOP requires zero TTL and velocities')
    head = bytes((0x5a, 28, 0x20))
    head += bytes((crc(head, 255, 0x8c, 255),))
    data = head + struct.pack('<IBBHHHIfff', uptime & 0xffffffff, 1, 2,
                              sequence, ttl, int(enabled), session, vx, vy, wz)
    return data + struct.pack('<H', crc(data))


def feedback(frame):
    if len(frame) != 34 or frame[:3] != b'\x5a\x1c\x21' or \
       crc(frame[:3], 255, 0x8c, 255) != frame[3] or crc(frame[:-2]) != int.from_bytes(frame[-2:], 'little'):
        raise ValueError('Invalid feedback frame')
    values = struct.unpack('<IBBHHHIfff', frame[4:32])
    uptime, version, route, sequence, status, fault, session, vx, vy, wz = values
    if version != 1 or route != 2 or status & ~127 or (status & 64 and not session) or \
       not all(math.isfinite(x) for x in (vx, vy, wz)) or abs(vx)>20 or abs(vy)>20 or abs(wz)>100 or \
       (not status & 8 and any((vx, vy, wz))):
        raise ValueError('Invalid feedback fields')
    return dict(uptime=uptime, sequence=sequence, status=status, fault=fault,
                session=session, vx=vx, vy=vy, wz=wz)


class Stream:
    """Consume complete USB feedback, joint and SP frames before resyncing."""
    def __init__(self):
        self.data = bytearray()
        self.bytes_received = 0
        self.frames = {'SP': 0, 'gimbal': 0, 'feedback': 0}
        self.last_gimbal = None
        self.diagnostics = dict(resync_bytes=0, crc_errors=0, header_crc_errors=0,
                                invalid_feedback=0, invalid_gimbal=0)

    def push(self, data):
        self.bytes_received += len(data)
        self.data.extend(data)
        while self.data:
            size = 0
            if self.data[0] == ord('S'):
                if len(self.data) < 2:
                    break
                if self.data[:2] == b'SP':
                    size = 43
            elif self.data[0] == 0x5a:
                if len(self.data) < 3:
                    break
                size = {(28, 0x21): 34, (40, 0x22): 46}.get(tuple(self.data[1:3]), 0)
            if not size:
                self.diagnostics['resync_bytes'] += 1
                del self.data[0]
                continue
            if len(self.data) < size:
                break
            frame = bytes(self.data[:size])
            bad_crc = crc(frame[:-2]) != int.from_bytes(frame[-2:], 'little')
            bad_header = frame[0] == 0x5a and crc(frame[:3],255,0x8c,255) != frame[3]
            if bad_crc or bad_header:
                self.diagnostics['crc_errors'] += int(bad_crc)
                self.diagnostics['header_crc_errors'] += int(bad_header)
                self.diagnostics['resync_bytes'] += 1
                del self.data[0]
                continue
            del self.data[:size]
            if size == 43:
                self.frames['SP'] += 1
            elif size == 46 and frame[8:10] == b'\x01\x02':
                self.frames['gimbal'] += 1
                self.last_gimbal = dict(
                    uptime=int.from_bytes(frame[4:8], 'little'),
                    sequence=int.from_bytes(frame[10:12], 'little'),
                    session=int.from_bytes(frame[16:20], 'little'),
                    command_received_ms=int.from_bytes(frame[24:28], 'little'))
            elif size == 46:
                self.diagnostics['invalid_gimbal'] += 1
            if size == 34:
                try:
                    decoded = feedback(frame)
                except ValueError:
                    self.diagnostics['invalid_feedback'] += 1
                else:
                    self.frames['feedback'] += 1
                    yield decoded


class FeedbackStats:
    """Count filtering separately from stream corruption and transport gaps."""
    def __init__(self):
        self.counts = dict(received=0, accepted=0, wrong_session=0, no_ack_flag=0,
                           unknown_sequence=0, stale_ack=0, nonincreasing_uptime=0,
                           active_mismatch=0, not_permitted=0)
        self.last_received = self.last_accepted = None
        self.last_uptime = None
        self.max_feedback_gap_ms = self.max_accepted_gap_ms = self.max_uptime_gap_ms = 0.

    def received(self, frame, now):
        self.counts['received'] += 1
        if self.last_received is not None:
            self.max_feedback_gap_ms = max(self.max_feedback_gap_ms, (now-self.last_received)*1000)
        if self.last_uptime is not None:
            delta = (frame['uptime']-self.last_uptime) & 0xffffffff
            if 0 < delta < 0x80000000:
                self.max_uptime_gap_ms = max(self.max_uptime_gap_ms, delta)
        self.last_received, self.last_uptime = now, frame['uptime']

    def reject(self, reason):
        self.counts[reason] += 1

    def accept(self, now):
        self.counts['accepted'] += 1
        if self.last_accepted is not None:
            self.max_accepted_gap_ms = max(self.max_accepted_gap_ms, (now-self.last_accepted)*1000)
        self.last_accepted = now

    def report(self):
        return dict(self.counts, max_feedback_gap_ms=self.max_feedback_gap_ms,
                    max_accepted_gap_ms=self.max_accepted_gap_ms,
                    max_uptime_gap_ms=self.max_uptime_gap_ms)
