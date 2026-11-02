#!/usr/bin/env python3
"""Summarize source/receive timing around a saved ground-bridge fault."""
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
raw = []
with path.open(encoding="utf-8") as handle:
    for line in handle:
        row = json.loads(line)
        if row.get("kind") == "raw":
            data = row["data"]
            raw.append((data["server_stamp_us"], data["receive_monotonic_us"]))
if len(raw) < 2:
    raise SystemExit("insufficient raw samples")
base_source, base_receive = raw[0]
worst = []
for index, (source, receive) in enumerate(raw):
    drift = (source - base_source) - (receive - base_receive)
    if index:
        source_gap = source - raw[index - 1][0]
        receive_gap = receive - raw[index - 1][1]
        worst.append((abs(drift), index, drift, source_gap, receive_gap))
worst.sort(reverse=True)
print(json.dumps({
    "raw_count": len(raw),
    "span_source_s": (raw[-1][0] - base_source) / 1e6,
    "span_receive_s": (raw[-1][1] - base_receive) / 1e6,
    "final_relative_drift_ms": ((raw[-1][0] - base_source) - (raw[-1][1] - base_receive)) / 1e3,
    "worst": [
        {"index": i, "drift_ms": d / 1e3, "source_gap_ms": sg / 1e3, "receive_gap_ms": rg / 1e3}
        for _, i, d, sg, rg in worst[:8]
    ],
}, ensure_ascii=False, indent=2))
