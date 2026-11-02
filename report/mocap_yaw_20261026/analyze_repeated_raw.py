"""Offline analysis only: exact raw repeats and attitude near the failed yaw run."""
import json
import math
from pathlib import Path
import statistics
WORK=Path(__file__).resolve().parent
RUN=WORK/"lift_reference_20261026T032518Z"
with (RUN/"samples.jsonl").open(encoding="utf-8") as handle:
    rows=[json.loads(line) for line in handle]
raw=[r for r in rows if r["kind"]=="raw"]
groups=[]
start=0
for i in range(1,len(raw)+1):
    equal=(i<len(raw) and raw[i]["data"]["position"]==raw[start]["data"]["position"]
           and raw[i]["data"]["quaternion_xyzw"]==raw[start]["data"]["quaternion_xyzw"])
    if equal:continue
    part=raw[start:i]
    elapsed=(part[-1]["data"]["receive_monotonic_us"]-part[0]["data"]["receive_monotonic_us"])/1e6
    if elapsed>.1:
        groups.append(dict(frames=len(part),span_s=elapsed,start_monotonic=part[0]["monotonic"],
            end_monotonic=part[-1]["monotonic"],
            source_span_s=(part[-1]["data"]["server_stamp_us"]-part[0]["data"]["server_stamp_us"])/1e6,
            position=part[0]["data"]["position"],quaternion_xyzw=part[0]["data"]["quaternion_xyzw"]))
    start=i
fault=[r for r in rows if r["kind"]=="bridge" and r["data"].get("latched_fault")]
inputs=[r for r in rows if r["kind"]=="input"]
local=[r for r in rows if r["kind"]=="local"]
def attitude(q):
    w,x,y,z=q
    return dict(roll_deg=math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))),
        pitch_deg=math.degrees(math.asin(max(-1.,min(1.,2*(w*y-z*x))))),
        yaw_deg=math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))),
        tilt_deg=math.degrees(math.acos(max(-1.,min(1.,1-2*(x*x+y*y))))))
last=inputs[-1]
q=attitude(last["data"]["q"])
result=dict(offline_only=True,raw_frames=len(raw),repeats_over_100ms=groups,
    first_bridge_fault_monotonic=fault[0]["monotonic"] if fault else None,
    last_forwarded_attitude=q,last_forwarded_monotonic=last["monotonic"],
    longest_repeat=max(groups,key=lambda x:x["span_s"]) if groups else None,
    source_receive_gap_ms_max=max(b["data"]["receive_monotonic_us"]-a["data"]["receive_monotonic_us"] for a,b in zip(raw,raw[1:]))/1000,
    conclusion="Exact repetition occurs in raw VRPN samples with advancing timestamps; root cause unproven. No thresholds changed.",
    operator_observation="Markers visible, no observed tracking loss.")
(WORK/"offline_repeat_analysis.json").write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
print(json.dumps(result,ensure_ascii=False,indent=2))
