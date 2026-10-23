"""Inspect actual native/ROS gaps; do not change acceptance limits."""
import json
from pathlib import Path
import statistics
ROOT = Path(__file__).resolve().parent
runs = sorted(p for p in ROOT.glob("diagnostic_*") if p.is_dir())
out = []
for run in runs:
    path = run / "samples.jsonl"
    if not path.exists():
        continue
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    raw = [r["data"] for r in rows if r["kind"] == "raw"]
    gaps = []
    for a,b in zip(raw, raw[1:]):
        gaps.append(dict(receive_gap_us=b["receive_monotonic_us"]-a["receive_monotonic_us"],
                         source_gap_us=b["server_stamp_us"]-a["server_stamp_us"],
                         after_source_us=a["server_stamp_us"],
                         after_receive_wall_us=a["receive_wall_us"]))
    ordered = sorted(gaps, key=lambda d:d["receive_gap_us"], reverse=True)
    validation = json.loads((run / "validation.json").read_text(encoding="utf-8"))
    cleanup = json.loads((run / "process_cleanup.json").read_text(encoding="utf-8"))
    native_path = run / "source/native.stderr.log"
    result = dict(directory=run.name, raw_frames=len(raw),
                  largest_receive_gaps=ordered[:8],
                  receive_gaps_over_150ms=sum(r["receive_gap_us"] > 150000 for r in gaps),
                  median_receive_gap_us=statistics.median(r["receive_gap_us"] for r in gaps),
                  source_nonincreasing=sum(r["source_gap_us"] <= 0 for r in gaps),
                  native_summary=native_path.read_text(encoding="utf-8") if native_path.exists() else None,
                  validation=validation, cleanup=cleanup)
    out.append(result)
(ROOT / "diagnostic_analysis.json").write_text(json.dumps(out, indent=2), encoding="utf-8")
print(json.dumps(out, indent=2))
