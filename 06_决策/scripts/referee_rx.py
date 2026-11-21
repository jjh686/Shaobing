"""Read-only RM2027 V1.0.0 referee decoder for navigation and health data."""

import struct


def crc(data, initial, polynomial):
    for byte in data:
        initial ^= byte
        for _ in range(8):
            initial = (initial >> 1) ^ (polynomial if initial & 1 else 0)
    return initial


def u16(data, offset):
    return struct.unpack_from('<H', data, offset)[0]


LENGTHS = {0x0001: 11, 0x0003: 24, 0x0101: 8, 0x0201: 17, 0x0209: 6}


class Receiver:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, chunk):
        # Bound each callback and retain at most one partial frame.
        if len(chunk) > 4096:
            self.buffer.clear()
            return
        self.buffer.extend(chunk)
        while self.buffer:
            start = self.buffer.find(b'\xa5')
            if start < 0:
                self.buffer.clear()
                break
            del self.buffer[:start]
            if len(self.buffer) < 5:
                break
            length = u16(self.buffer, 1)
            if length > 512 or crc(self.buffer[:4], 0xff, 0x8c) != self.buffer[4]:
                del self.buffer[0]
                continue
            end = 9 + length
            if len(self.buffer) < end:
                break
            if crc(self.buffer[:end - 2], 0xffff, 0x8408) != u16(self.buffer, end - 2):
                del self.buffer[0]
                continue
            packet = bytes(self.buffer[:end])
            del self.buffer[:end]
            command = u16(packet, 5)
            if LENGTHS.get(command) == length:
                yield command, packet[7:-2]


def decode(command, data):
    if command == 0x0001:
        return data[0] & 15, data[0] >> 4
    if command == 0x0003:
        return u16(data, 0), u16(data, 4), u16(data, 10)
    if command == 0x0101:
        field = struct.unpack_from('<I', data, 4)[0]
        return (field >> 26) & 3
    if command == 0x0201:
        return data[0], u16(data, 2), u16(data, 4), bool(data[16] & 2)
    if command == 0x0209:
        return bool(data[0] & 1)
    raise ValueError('Unsupported referee command')


class OwnLife:
    """Track weakness across continuous HP observations within a match."""

    def __init__(self):
        self.phase = None
        self.hp = None
        self.hp_stamp = None
        self.known = False
        self.weakened = False

    def game(self, kind, phase):
        same_round = kind == 4 and (
            (phase == 3 and self.phase == 3) or (phase == 4 and self.phase in (3, 4))
        )
        if not same_round:
            self.known = self.weakened = False
            self.hp = self.hp_stamp = None
        self.phase = phase if kind == 4 else None

    def robot(self, hp, now):
        if self.phase == 3:
            self.known = hp > 0
            self.hp, self.hp_stamp = hp, now
            return
        if self.phase != 4:
            return
        contiguous = self.hp_stamp is not None and 0 <= now - self.hp_stamp < 1.0
        if hp == 0:
            self.known, self.weakened = True, False
        elif contiguous and self.hp == 0:
            self.known, self.weakened = True, True
        elif not contiguous:
            self.known, self.weakened = False, False
        self.hp, self.hp_stamp = hp, now

    def supply_tag(self, detected, now):
        if (detected and self.phase == 4 and self.hp and self.hp_stamp is not None
                and 0 <= now - self.hp_stamp < 0.75):
            # Own supply RFID clears weakness under the RMUL respawn rule.
            self.known, self.weakened = True, False
