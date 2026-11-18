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
                del self.data[0]
                continue
            if len(self.data) < size:
                break
            frame = bytes(self.data[:size])
            if crc(frame[:-2]) != int.from_bytes(frame[-2:], 'little') or \
               (frame[0] == 0x5a and crc(frame[:3],255,0x8c,255) != frame[3]):
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
            if size == 34:
                try:
                    decoded = feedback(frame)
                except ValueError:
                    pass
                else:
                    self.frames['feedback'] += 1
                    yield decoded
