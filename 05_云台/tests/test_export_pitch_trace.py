"""Host-only tests: no bridge, probe, or MCU access."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    "export_pitch_trace", Path(__file__).resolve().parents[1]/"tools/export_pitch_trace.py")
trace = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trace)


class FakeRead:
    def __init__(self, total=6, count=4, next_slot=2):
        self.header = dict(zip(trace.HEADER, (total, count, next_slot, 1000)))
        self.records = {}
        self.headers_read = 0
        self.change_total = False
        self.reset_tick = False
        for offset in range(count):
            slot = (next_slot-count+offset) % 4
            row = dict.fromkeys(trace.FIELDS, 0)
            row.update(sequence=(total-count+offset+1)&trace.U32, tick=500+offset*50,
                       driver_target_rad=0.1, measured_rad=0.05, mode=3, enabled=1)
            self.records[slot] = row

    def __call__(self, names):
        if names == trace.HEADER:
            self.headers_read += 1
            result = self.header.copy()
            if self.headers_read > 1:
                if self.change_total:
                    result[trace.HEADER[0]] += 1
                if self.reset_tick:
                    result["uwTick"] = 1
            return result
        slot = int(names[0].split("[")[1].split("]")[0])
        return {name: self.records[slot][name.rsplit(".", 1)[1]] for name in names}


class TraceExportTests(unittest.TestCase):
    def test_ring_order_and_units(self):
        rows, _ = trace.capture(FakeRead(), capacity=4)
        self.assertEqual([r["slot"] for r in rows], [2, 3, 0, 1])
        self.assertEqual([r["sequence"] for r in rows], [3, 4, 5, 6])
        self.assertAlmostEqual(rows[-1]["time_s"], 0.15)
        self.assertAlmostEqual(rows[0]["error_deg"], 2.86478897565)

    def test_sequence_rollover(self):
        rows, _ = trace.capture(FakeRead(total=1), capacity=4)
        self.assertEqual([r["sequence"] for r in rows], [0xfffffffe, 0xffffffff, 0, 1])

    def test_active_capture_is_rejected(self):
        read = FakeRead()
        read.change_total = True
        with self.assertRaisesRegex(RuntimeError, "still recording"):
            trace.capture(read, capacity=4)

    def test_rewritten_slot_is_rejected(self):
        read = FakeRead()
        read.records[2]["sequence"] = 7
        with self.assertRaisesRegex(RuntimeError, "overwritten"):
            trace.capture(read, capacity=4)

    def test_reset_is_rejected(self):
        read = FakeRead()
        read.reset_tick = True
        with self.assertRaisesRegex(RuntimeError, "MCU time"):
            trace.capture(read, capacity=4)

    def test_unavailable_is_not_a_measurement(self):
        for result in ({"partial": True}, {"values": {"x": {"error": {"code": "BUDGET"}}}},
                       {"values": {"x": {"value": "NaN"}}}):
            with self.assertRaises(RuntimeError):
                trace.scalar_values(result, ["x"])


if __name__ == "__main__":
    unittest.main()
