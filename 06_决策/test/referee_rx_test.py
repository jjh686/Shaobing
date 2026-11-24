import struct
import sys
import unittest
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts' / 'adapters'))

from referee_rx import decode


class MatchFieldsTest(unittest.TestCase):
    def test_phase_remaining_time_is_unsigned_little_endian(self):
        self.assertEqual(decode(0x0001, bytes.fromhex('44 02 01') + bytes(8)), (4, 4, 258))

    def test_control_scores_follow_the_team_hp_fields(self):
        payload = bytearray(24)
        for offset, value in ((0, 450), (4, 350), (10, 400), (20, 512), (22, 386)):
            struct.pack_into('<H', payload, offset, value)
        self.assertEqual(decode(0x0003, payload), (450, 350, 400, 512, 386))

    def test_central_activation_flags_are_independent_of_ownership(self):
        for active in range(4):
            with self.subTest(active=active):
                payload = struct.pack('<II', 0xffffffff, (2 << 26) | (active << 24))
                self.assertEqual(decode(0x0101, payload), (2, active))


if __name__ == '__main__':
    unittest.main()
