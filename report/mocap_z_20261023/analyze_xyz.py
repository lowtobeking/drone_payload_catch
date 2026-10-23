"""Offline XYZ/yaw agreement in settled successful ground window; preserve all results."""
from bisect import bisect_right
import json
import math
from pathlib import Path
import statistics
ROOT=Path(__file__).resolve().parent
test=sorted(ROOT.glob("xyz_reference_20*"))[-1]
v=json.loads((test/"validation.json").read_text(encoding="utf-8"))
rows=[json.loads(line) for line in (test/"samples.jsonl").read_text(encoding="utf-8").splitlines()]
inputs=[r for r in rows if r["kind"]=="input"]
times=[r["monotonic"] for r in inputs]
start=v.get("stable_start_monotonic")
result=dict(run=test.name,passed=v["passed"],scope="static_ground_only",physical_height_accuracy_verified=False,
            all_disarmed=all(r["data"]["arming_state"]==1 for r in rows if r["kind"]=="status"),
            all_landed=all(r["data"]["landed"] for r in rows if r["kind"]=="land"))
def stats(vals):
    vals=sorted(vals)
    return dict(mean=statistics.mean(vals),maximum=max(vals),p95=vals[math.ceil(len(vals)*.95)-1])
def yaw(q):
    w,x,y,z=q
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
pairs=[]
if start:
    for r in rows:
        if r["kind"]!="local" or not start+5<=r["monotonic"]<=start+25:continue
        i=bisect_right(times,r["monotonic"])-1
        if i<0 or r["monotonic"]-times[i]>.06:continue
        a=r["data"];b=inputs[i]["data"]
        xyz=[a[k]-b["position"][j] for j,k in enumerate(("x","y","z"))]
        angle=a["heading"]-yaw(b["q"])
        pairs.append(dict(xyz=xyz,yaw_deg=math.degrees(math.atan2(math.sin(angle),math.cos(angle))),
                          input_xyz=b["position"],px4_xyz=[a[k] for k in ("x","y","z")]))
if pairs:
    result.update(pairs=len(pairs),z_abs_error_m=stats([abs(p["xyz"][2]) for p in pairs]),
        xy_error_m=stats([math.hypot(*p["xyz"][:2]) for p in pairs]),
        yaw_abs_error_deg=stats([abs(p["yaw_deg"]) for p in pairs]),
        input_xyz_mean_m=[statistics.mean(p["input_xyz"][j] for p in pairs) for j in range(3)],
        px4_xyz_mean_m=[statistics.mean(p["px4_xyz"][j] for p in pairs) for j in range(3)])
ev=[r for r in rows if r["kind"]=="flags" and all(r["data"][k] for k in ("cs_ev_pos","cs_ev_yaw","cs_ev_hgt"))]
result["ev_xyz_yaw_flags_samples"]=len(ev)
result["ev_xyz_yaw_sample_span_s"]=ev[-1]["monotonic"]-ev[0]["monotonic"] if ev else 0
result["magnetic_heading_or_3d_during_ev"]=any(r["data"]["cs_mag_hdg"] or r["data"]["cs_mag_3d"] for r in ev)
result["position_pairing"]="nearest preceding input by observer receive time <=60ms; not acquisition-time alignment"
(test/"xyz_comparison.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
print(json.dumps(result,indent=2))
