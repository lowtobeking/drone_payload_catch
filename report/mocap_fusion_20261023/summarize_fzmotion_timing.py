"""Summarize measured evidence without authorizing a clock mapping or flight."""
import json
from pathlib import Path
import statistics

WORK=Path(__file__).resolve().parent
wire=json.loads((WORK/"wire_time_20261023T150023/result.json").read_text(encoding="utf-8"))
messages=json.loads((WORK/"message_time_20261023T150644/result.json").read_text(encoding="utf-8"))
roundtrip=json.loads((WORK/"roundtrip_20261023T151229/result.json").read_text(encoding="utf-8"))
diagnostic=json.loads((WORK/"diagnostic_20261023T070724Z/validation.json").read_text(encoding="utf-8"))
analysis=json.loads((WORK/"diagnostic_analysis.json").read_text(encoding="utf-8"))
current=next(row for row in analysis if row["directory"]=="diagnostic_20261023T070724Z")
pongs=[r for r in roundtrip if r["kind"]=="pong"]
summary=next(r for r in roundtrip if r["kind"]=="summary")
assert len(pongs)==20 and not summary["failed"] and not summary["pending"] and summary["unsolicited"]==0
lower=max(r["local_minus_source_lower_us"] for r in pongs)
upper=min(r["local_minus_source_upper_us"] for r in pongs)
candidate={"lower_us":lower,"upper_us":upper,"intersection_nonempty":lower<=upper}
if lower<=upper:
    candidate.update(midpoint_us=(lower+upper)/2,half_width_us=(upper-lower)/2)
result={
    "software_reported_by_operator":"FZMotion",
    "software_version_unknown":True,
    "server_wire_matches_callback":wire["conclusion"]=="wire_contains_same_timestamp_as_callback",
    "wire_timestamp_source_minus_receive_us_range":wire["source_minus_receive_us_range"],
    "pong_and_pose_share_large_time_difference":True,
    "roundtrip_samples":len(pongs),
    "rtt_us":{"minimum":min(r["rtt_us"] for r in pongs),
              "median":statistics.median(r["rtt_us"] for r in pongs),
              "maximum":max(r["rtt_us"] for r in pongs)},
    "CONDITIONAL_local_minus_server_offset_interval":candidate,
    "interval_assumptions":["Pong timestamp generated during this request processing",
                          "Pong and pose timestamps use the same source clock",
                          "Negligible unbounded drift during measurement",
                          "No unrecognized unsolicited response or echo semantics"],
    "server_timestamp_semantics_verified":False,
    "sampling_exposure_delay_verified":False,
    "mapping_written_to_bridge_config":False,
    "physical_origin_offset_known":False,
    "diagnostic_18s_passed":diagnostic["passed"],
    "diagnostic_matched_positions":diagnostic["matched_positions"],
    "diagnostic_max_receive_gap_us":current["largest_receive_gaps"][0]["receive_gap_us"],
    "forwarded_to_px4":False,
    "flight_approval":False,
    "remaining":["FZMotion version and time-output semantics",
                 "Rigid-body origin/installation offset",
                 "Tracking-loss and frozen-source checks",
                 "Longer bounded continuity validation",
                 "Actual PX4 input and EKF fusion acceptance"]
}
(WORK/"fzmotion_timing_progress.json").write_text(
    json.dumps(result,indent=2,ensure_ascii=False),encoding="utf-8")
print(json.dumps(result,indent=2,ensure_ascii=False))
