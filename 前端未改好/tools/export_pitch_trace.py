#!/usr/bin/env python3
"""Export a retained Pitch RAM trace through EmberProbe's read-only skill."""
import argparse
import csv
import datetime as dt
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import time

FIELDS = (
    "sequence", "tick", "motor_tick", "gate_flags", "fault_reason_flags",
    "source_target_rad", "driver_target_rad", "measured_rad", "speed_rad_s",
    "roll_rad", "mechanical_deg", "raw_speed_ref_rad_s", "speed_ref_rad_s",
    "p_nm", "i_nm", "gravity_nm", "friction_nm",
    "requested_nm", "applied_nm", "reported_nm",
    "mode", "enabled", "motor_state", "fault",
)
HEADER = ("gimbal_pitch_trace_total", "gimbal_pitch_trace_count",
          "gimbal_pitch_trace_next", "uwTick")
U32 = 0xffffffff


def scalar_values(result, names):
    if result.get("partial") or result.get("variableErrors"):
        raise RuntimeError("Partial read; unavailable values cannot be used as measurements.")
    values = {}
    for name in names:
        item = result.get("values", {}).get(name, {})
        value = item.get("value")
        if item.get("error") or isinstance(value, bool) or not isinstance(value, (int, float)):
            raise RuntimeError(f"No numeric value for {name}: {item}")
        if not math.isfinite(value):
            raise RuntimeError(f"Nonfinite value for {name}")
        values[name] = value
    return values


class Reader:
    def __init__(self, workspace, node, output):
        self.workspace, self.node, self.output = workspace, node, output
        self.script = workspace / ".agents/skills/mcu-variables/scripts/read.js"
        self.binding = None
        self.firmware_statuses = set()
        self.sources = set()

    def read(self, names):
        # One record has 24 scalar leaves: keep each call under the 32-read budget.
        run = subprocess.run(
            [self.node, str(self.script), "--workspace", str(self.workspace),
             "--variables", ",".join(names)],
            text=True, capture_output=True, timeout=45,
        )
        with (self.output / "raw_reads.jsonl").open("a", encoding="utf-8") as f:
            f.write(json.dumps({"names": names, "exit_code": run.returncode,
                                "stdout": run.stdout, "stderr": run.stderr},
                               ensure_ascii=False) + "\n")
        if run.returncode:
            # Preserve the Bridge diagnostic; never retry by starting another server.
            raise RuntimeError(run.stderr.strip() or run.stdout.strip())
        result = json.loads(run.stdout)
        status = result.get("firmwareConsistency", {}).get("status", "unknown")
        if status == "mismatch":
            raise RuntimeError("ELF_TARGET_MISMATCH: disk symbols do not match the programming record.")
        self.firmware_statuses.add(status)
        elf = result.get("elf", {})
        binding = (elf.get("path"), elf.get("sha256"))
        if not all(binding):
            raise RuntimeError("The read did not return an ELF path and SHA256.")
        if self.binding is not None and binding != self.binding:
            raise RuntimeError("ELF changed during export; discard this capture.")
        self.binding = binding
        self.sources.add(result.get("source", "unknown"))
        return scalar_values(result, names)


def capture(read, capacity=128):
    started = time.monotonic()
    before = read(HEADER)
    total, count, next_slot = (before[name] for name in HEADER[:3])
    if any(not isinstance(x, int) for x in (total, count, next_slot)):
        raise RuntimeError("Invalid trace counters.")
    if not 0 < count <= capacity or not 0 <= next_slot < capacity:
        raise RuntimeError("No trace, or counters do not match the configured capacity.")
    records = []
    for offset in range(count):
        slot = (next_slot-count+offset) % capacity
        names = tuple(f"gimbal_pitch_trace[{slot}].{field}" for field in FIELDS)
        values = read(names)
        row = {field: values[name] for field, name in zip(FIELDS, names)}
        expected = (total-count+offset+1) & U32
        if row["sequence"] != expected:
            raise RuntimeError("Trace was overwritten or reset during export; no valid CSV produced.")
        row["slot"] = slot
        records.append(row)
        if (offset+1) % 16 == 0:
            print(f"Read {offset+1}/{count} records", file=sys.stderr, flush=True)
    after = read(HEADER)
    if any(before[name] != after[name] for name in HEADER[:3]):
        raise RuntimeError("Trace is still recording. Remove the target, wait for the 2-second tail, then export again.")
    elapsed_ms = (time.monotonic()-started)*1000
    if ((after["uwTick"]-before["uwTick"]) & U32) > elapsed_ms+5000:
        raise RuntimeError("MCU time changed unexpectedly; possible reset during export.")
    first_tick = records[0]["tick"]
    for row in records:
        row["time_s"] = ((row["tick"]-first_tick) & U32)/1000
        row["error_deg"] = math.degrees(row["driver_target_rad"]-row["measured_rad"])
        for field in ("source_target_rad", "driver_target_rad", "measured_rad", "roll_rad"):
            row[field.removesuffix("_rad")+"_deg"] = math.degrees(row[field])
    return records, {"before": before, "after": after}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path, help="New output directory; existing directories are refused")
    parser.add_argument("--node", help="Node executable used to run the installed EmberProbe skill")
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    node = args.node or shutil.which("node")
    if not node:
        bundled = Path.home()/".cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node"
        if bundled.is_file():
            node = str(bundled)
    if not node:
        parser.error("Node was not found; specify --node.")
    now = dt.datetime.now(dt.timezone(dt.timedelta(hours=8)))
    output = args.output or workspace.parent.parent/("Pitch历史导出_"+now.strftime("%Y%m%d_%H%M%S_%f"))
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    reader = Reader(workspace, node, output)
    try:
        records, counters = capture(reader.read)
        with (output/"pitch_trace.csv").open("w", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=list(records[0]))
            writer.writeheader()
            writer.writerows(records)
        metadata = {
            "created_at": now.isoformat(), "workspace": str(workspace),
            "elf": {"path": reader.binding[0], "sha256": reader.binding[1]},
            "firmware_consistency_statuses": sorted(reader.firmware_statuses),
            "sources": sorted(reader.sources), "counters": counters,
            "record_count": len(records),
            "tracking_records": sum(r["mode"] == 3 and r["enabled"] == 1 for r in records),
            "duration_s": records[-1]["time_s"],
            "max_abs_driver_error_deg": max(abs(r["error_deg"]) for r in records),
            "max_abs_applied_nm": max(abs(r["applied_nm"]) for r in records),
            "scope": "Read-only diagnostic; programming-record match is not independent flash verification. "
                     "Unknown firmware consistency requires checking the flashed ELF. "
                     "Fields may differ by a control task period; 20 Hz history cannot capture every transient. "
                     "No automatic root-cause diagnosis or physical calibration.",
        }
        (output/"metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+"\n")
        print(output/"pitch_trace.csv")
        return 0
    except (RuntimeError, ValueError, OSError, subprocess.TimeoutExpired) as error:
        diagnostic = {"error": str(error), "valid_capture": False}
        (output/"error.json").write_text(json.dumps(diagnostic, ensure_ascii=False, indent=2)+"\n")
        print(str(error), file=sys.stderr)
        print(f"Diagnostic saved in {output}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
