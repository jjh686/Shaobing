import select
import socket
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

from vision_feedback import FRESHNESS_NS, VisionReceiver, decode_status


class VisionFeedbackTest(unittest.TestCase):
    def test_status_format_rejects_unknown_version_and_extra_fields(self):
        self.assertEqual(decode_status(b'RMVISION1 123 detecting\n'), (123, 'detecting'))
        for data in (b'RMVISION2 123 tracking\n', b'RMVISION1 123 invalid\n',
                     b'RMVISION1 123 tracking extra\n', b'RMVISION1 123 tracking'):
            self.assertIsNone(decode_status(data))

    @patch('vision_feedback.time.monotonic_ns', return_value=2_000_000_000)
    def test_only_fresh_and_newer_datagrams_replace_current_status(self, now):
        receiver = VisionReceiver(0)
        self.addCleanup(receiver.close)
        address = receiver.socket.getsockname()
        self.assertEqual(address[0], '127.0.0.1')

        def wait_for_sample():
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                select.select([receiver.socket], [], [], 0.05)
                sample = receiver.poll()
                if sample is not None:
                    return sample
            return None

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            def send(stamp, state):
                sender.sendto(f'RMVISION1 {stamp} {state}\n'.encode('ascii'), address)

            stamp = now.return_value
            send(stamp - FRESHNESS_NS, 'tracking')
            send(stamp + 1, 'tracking')
            send(stamp - 1, 'tracking')
            self.assertEqual(wait_for_sample(), (stamp - 1, 'tracking'))
            send(stamp - 2, 'tracking')
            send(stamp, 'unavailable')
            self.assertEqual(wait_for_sample(), (stamp, 'unavailable'))
            send(stamp, 'tracking')
            self.assertTrue(select.select([receiver.socket], [], [], 1)[0])
            self.assertIsNone(receiver.poll())


if __name__ == '__main__':
    unittest.main()
