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
        # 限制单次输入大小，保留未收完的帧供下次拼接。
        if len(chunk) > 4096:
            self.buffer.clear()
            return
        self.buffer.extend(chunk)
        while self.buffer:
            # 先找帧头，再按长度和 CRC 验证整帧。
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
        # 首字节的低四位为赛项，高四位为阶段。
        return data[0] & 15, data[0] >> 4, u16(data, 1)
    if command == 0x0003:
        # 依次读取己方重装、步兵、哨兵血量及双方控制度。
        return tuple(u16(data, offset) for offset in (0, 4, 10, 20, 22))
    if command == 0x0101:
        # 中央区的控制归属和双方激活状态各占两位。
        field = struct.unpack_from('<I', data, 4)[0]
        return (field >> 26) & 3, (field >> 24) & 3
    if command == 0x0201:
        # 本车编号、当前与最大血量、底盘供电状态。
        return data[0], u16(data, 2), u16(data, 4), bool(data[16] & 2)
    if command == 0x0209:
        # 位 0 表示检测到己方补给区 RFID。
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
        # 连续观测到零血恢复为正血，记为复活后虚弱。
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
            # 本车近期血量为正且检测到己方补给区，解除虚弱。
            self.known, self.weakened = True, False
