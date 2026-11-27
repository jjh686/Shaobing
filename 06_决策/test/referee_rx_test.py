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
        for offset, value in ((0, 450), (4, 350), (6, 0), (10, 400), (20, 512), (22, 386)):
            struct.pack_into('<H', payload, offset, value)
        self.assertEqual(decode(0x0003, payload), (450, 350, 400, 512, 386, 0))

    def test_infantry_four_is_appended_for_empty_slot_three(self):
        payload = bytearray(24)
        for offset, value in ((0, 450), (4, 0), (6, 280), (10, 400), (20, 1), (22, 2)):
            struct.pack_into('<H', payload, offset, value)
        self.assertEqual(decode(0x0003, payload), (450, 0, 400, 1, 2, 280))

    def test_heat_ammo_and_hit_fields_match_the_2027_tables(self):
        robot = bytes([7, 1]) + struct.pack('<HHHHH', 300, 400, 20, 80, 0) + struct.pack('<f', 0) + bytes([0b00000110])
        self.assertEqual(decode(0x0201, robot), (7, 300, 400, True, 80, True))
        heat = bytes(8) + struct.pack('<HHH', 40, 12, 0)
        self.assertEqual(decode(0x0202, heat), (40, 12, 0))
        ammo = struct.pack('<HHHH', 50, 0, 9, 0)
        self.assertEqual(decode(0x0208, ammo), (50, 0, 9, 0))
        hit = bytes([0x05]) + struct.pack('<HHH', 3, 10, 20)
        self.assertEqual(decode(0x0206, hit), (5, 0, 3, 10, 20))

    def test_central_activation_flags_are_independent_of_ownership(self):
        for active in range(4):
            with self.subTest(active=active):
                payload = struct.pack('<II', 0xffffffff, (2 << 26) | (active << 24))
                self.assertEqual(decode(0x0101, payload), (2, active))


    def test_ground_robot_positions_match_the_2027_0x020B_table(self):
        payload = bytearray(40)
        for offset, value in ((0, 1.25), (4, 1.0), (16, 6.0), (20, 4.0), (24, 10.75), (28, 7.0)):
            struct.pack_into('<f', payload, offset, value)
        self.assertEqual(decode(0x020B, payload), (1.25, 1.0, 6.0, 4.0, 10.75, 7.0))

    def test_official_positions_convert_to_the_map_centre_frame(self):
        from referee_rx import official_to_map, teammate_map_xy, prefer_observed_position
        self.assertEqual(official_to_map(1.25, 1.0), (-4.75, 3.0))
        self.assertEqual(official_to_map(10.75, 7.0), (4.75, -3.0))
        self.assertEqual(official_to_map(6.0, 4.0), (0.0, 0.0))
        self.assertIsNone(official_to_map(0.0, 0.0))
        self.assertIsNone(official_to_map(float('nan'), 1.0))
        self.assertIsNone(official_to_map(20.0, 1.0))
        positions = (1.25, 1.0, 6.0, 4.0, 0.0, 0.0)
        self.assertEqual(teammate_map_xy(positions, 'heavy'), (-4.75, 3.0))
        self.assertEqual(teammate_map_xy(positions, 'infantry'), (0.0, 0.0))
        self.assertIsNone(teammate_map_xy(positions, 'infantry', use_infantry_four=True))
        observed = (-1.0, 2.0)
        self.assertEqual(prefer_observed_position(observed, (0.0, 0.0)), observed)
        self.assertEqual(prefer_observed_position(None, (4.75, -3.0)), (4.75, -3.0))
        self.assertIsNone(prefer_observed_position(None, None))


if __name__ == '__main__':
    unittest.main()
