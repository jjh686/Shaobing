"""Receive timestamped tracker status from the local vision process."""

import re
import socket
import time


VISION_PORT = 9871
FRESHNESS_NS = 500_000_000
_PACKET = re.compile(rb'RMVISION1 ([0-9]{1,20}) (lost|detecting|tracking|temp_lost|switching|unavailable)\n')


def decode_status(datagram):
    match = _PACKET.fullmatch(datagram)
    if match is None:
        return None
    return int(match[1]), match[2].decode('ascii')


class VisionReceiver:
    def __init__(self, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.setblocking(False)
            self.socket.bind(('127.0.0.1', port))
        except OSError:
            self.close()
            raise
        self.newest_ns = 0

    def poll(self):
        latest = None
        for _ in range(32):
            try:
                datagram = self.socket.recv(64)
            except BlockingIOError:
                break
            sample = decode_status(datagram)
            if sample is None:
                continue
            stamp, _ = sample
            if stamp > self.newest_ns and 0 <= time.monotonic_ns() - stamp < FRESHNESS_NS:
                self.newest_ns = stamp
                latest = sample
        return latest

    def close(self):
        self.socket.close()
