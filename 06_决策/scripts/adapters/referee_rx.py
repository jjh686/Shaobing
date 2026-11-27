"""Read-only RM2027 V1.0.0 referee decoder for navigation and health data."""

import math
import struct


def crc(data, initial, polynomial):
    for byte in data:
        initial ^= byte
        for _ in range(8):
            initial = (initial >> 1) ^ (polynomial if initial & 1 else 0)
    return initial


def u16(data, offset):
    return struct.unpack_from('<H', data, offset)[0]


LENGTHS = {0x0001: 11, 0x0003: 24, 0x0101: 8, 0x0201: 17, 0x0202: 14, 0x0206: 7, 0x0208: 8, 0x0209: 6,
           0x020B: 40}

# Official 0x020B / handbook 3.2.1 battlefield size. Inner playing area only.
FIELD_LENGTH = 12.0
FIELD_WIDTH = 8.0


def official_to_map(x, y):
    """Convert 0x020B official field coordinates into the team map-centre frame.

    Official frame (protocol table 1-21, 0x020B): origin is the fence intersection
    near the red supply station; +X along the long side toward blue; +Y along the
    short side toward the red parking apron; units are metres.

    Team frame (maps/offline, figure 3-5 orientation): origin at the field centre,
    +X right, +Y up, metres. Red and blue features are 180-degree symmetric about
    that centre in this single shared frame.

    Handbook 3.2.1 states the battlefield is 12 m by 8 m. Figure 3-5 and figure 3-6
    place the red start/supply zone at the top-left 2.5 m by 2.0 m rectangle, whose
    centre is (-4.75, 3.0) in the team frame, matching maps/offline/red_tasks.yaml.
    That fixes official (0, 0) at the inner top-left corner, team (-6, 4): official
    +X equals team +X, official +Y equals team -Y.

        map_x = official_x - FIELD_LENGTH / 2
        map_y = FIELD_WIDTH / 2 - official_y

    (0, 0) is the fence corner itself and is treated as "not reported". 0x0203 is
    this same official frame for the *own* robot only and is never used as a
    teammate pose. 0x0301 and radar links are not decoded: sentry inter-robot
    comms are forbidden, and RMUL 3V3 has no radar.
    """
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    if (x == 0.0 and y == 0.0) or not (-0.5 <= x <= FIELD_LENGTH + 0.5 and -0.5 <= y <= FIELD_WIDTH + 0.5):
        return None
    return x - FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0 - y


def teammate_map_xy(positions, name, use_infantry_four=False):
    heavy_x, heavy_y, infantry3_x, infantry3_y, infantry4_x, infantry4_y = positions
    if name == 'heavy':
        return official_to_map(heavy_x, heavy_y)
    if name == 'infantry':
        if use_infantry_four:
            return official_to_map(infantry4_x, infantry4_y)
        return official_to_map(infantry3_x, infantry3_y)
    return None


def prefer_observed_position(observation, referee):
    """Prefer a fresh onboard observation; fall back to 0x020B. Neither implies vacancy."""
    if observation is not None:
        return observation
    return referee


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
        # 重装、3号步兵、哨兵、己方/对方控制度，以及 4 号步兵（3 号未上场时备用）。
        return tuple(u16(data, offset) for offset in (0, 4, 10, 20, 22, 6))
    if command == 0x0101:
        # 中央区的控制归属和双方激活状态各占两位。
        field = struct.unpack_from('<I', data, 4)[0]
        return (field >> 26) & 3, (field >> 24) & 3
    if command == 0x0201:
        # 本车编号、当前与最大血量、热量上限、底盘/发射机构供电。
        return (data[0], u16(data, 2), u16(data, 4), bool(data[16] & 2),
                u16(data, 8), bool(data[16] & 4))
    if command == 0x0202:
        # 缓冲能量、17mm 与 42mm 枪口热量。哨兵使用 17mm 热量。
        return u16(data, 8), u16(data, 10), u16(data, 12)
    if command == 0x0206:
        # 受击：装甲/原因、序号和命中位置（mm）。
        return data[0] & 15, data[0] >> 4, u16(data, 1), u16(data, 3), u16(data, 5)
    if command == 0x0208:
        # 17mm/42mm 允许发弹量、剩余金币、堡垒储备弹量。协议发给重装、步兵和哨兵。
        return u16(data, 0), u16(data, 2), u16(data, 4), u16(data, 6)
    if command == 0x0209:
        # 位 0 表示检测到己方补给区 RFID。
        return bool(data[0] & 1)
    if command == 0x020B:
        # 表 1-21：己方重装、3 号步兵、备用 4 号步兵的官方场地坐标（m）。不含本车，也不含 0x0203。
        values = struct.unpack_from('<10f', data, 0)
        return values[0], values[1], values[4], values[5], values[6], values[7]
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
