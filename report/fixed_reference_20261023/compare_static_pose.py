"""Offline comparison of captured PX4 input and estimate in the settled static window."""
from bisect import bisect_right
import json
import math
from pathlib import Path
import statistics
ROOT=Path(__file__).resolve().parent
RUN=ROOT/"fixed_reference_20261023T094649Z"
v=json.loads((RUN/"validation.json").read_text(encoding="utf-8"))
rows=[json.loads(line) for line in (RUN/"samples.jsonl").read_text(encoding="utf-8").splitlines()]
begin=v["stable_start_monotonic"]+5
end=v["stable_start_monotonic"]+25
inputs=[r for r in rows if r["kind"]=="input"]
times=[r["monotonic"] for r in inputs]
pairs=[]
def yaw(q):
    w,x,y,z=q
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
def wrap(a):return math.atan2(math.sin(a),math.cos(a))
for row in rows:
    if row["kind"]!="local" or not begin<=row["monotonic"]<=end:continue
    idx=bisect_right(times,row["monotonic"])-1
    if idx<0:continue
    source=inputs[idx]
    gap=row["monotonic"]-source["monotonic"]
    if gap>.06:continue
    a=row["data"];b=source["data"]
    dx=a["x"]-b["position"][0];dy=a["y"]-b["position"][1]
    pairs.append(dict(gap_s=gap,dx_m=dx,dy_m=dy,xy_error_m=math.hypot(dx,dy),
        yaw_error_deg=math.degrees(wrap(a["heading"]-yaw(b["q"]))),
        input_xy_m=b["position"][:2],px4_xy_m=[a["x"],a["y"]],
        input_yaw_deg=math.degrees(yaw(b["q"])),px4_yaw_deg=math.degrees(a["heading"]),
        input_z_m=b["position"][2],px4_z_m=a["z"]))
def stats(vals):
    vals=sorted(vals)
    return dict(mean=statistics.mean(vals),median=statistics.median(vals),
                p95=vals[math.ceil(.95*len(vals))-1],minimum=vals[0],maximum=vals[-1])
result=dict(scope="settled_static_window_only",window_s=[begin,end],pairs=len(pairs),
    pairing="nearest preceding captured VehicleOdometry input by observer monotonic receive time, <=60ms",
    xy_error_m=stats([r["xy_error_m"] for r in pairs]),
    yaw_absolute_difference_deg=stats([abs(r["yaw_error_deg"]) for r in pairs]),
    signed_yaw_difference_deg=stats([r["yaw_error_deg"] for r in pairs]),
    receive_pair_gap_s=stats([r["gap_s"] for r in pairs]),
    input_xy_mean_m=[statistics.mean(r["input_xy_m"][i] for r in pairs) for i in (0,1)],
    px4_xy_mean_m=[statistics.mean(r["px4_xy_m"][i] for r in pairs) for i in (0,1)],
    input_yaw_mean_deg=statistics.mean(r["input_yaw_deg"] for r in pairs),
    px4_yaw_mean_deg=statistics.mean(r["px4_yaw_deg"] for r in pairs),
    input_z_mean_m=statistics.mean(r["input_z_m"] for r in pairs),
    px4_z_mean_m=statistics.mean(r["px4_z_m"] for r in pairs),
    height_source="PX4 barometric height, EV height not enabled; vertical origins not aligned",
    limitations=["Comparison is against transformed motion-capture input, not raw world axes",
                 "Static proximity does not establish physical accuracy, source time alignment or dynamic tracking",
                 "Roll/pitch estimation comparison not captured in this log"])
(RUN/"static_pose_comparison.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
print(json.dumps(result,indent=2))
