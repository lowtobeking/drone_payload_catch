"""Read-only source/input/PX4 motion ranges; retain failure evidence."""
import json
import math
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def yaw(q,xyzw=False):
    if xyzw:
        x,y,z,w=q
    else:
        w,x,y,z=q
    return math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
def wrap(x):
    return (x+180)%360-180
def summary(rows,xyzw=False):
    values=[yaw(d["quaternion_xyzw"] if xyzw else d["q"],xyzw) for d in rows]
    base=values[0]
    deltas=[wrap(v-base) for v in values]
    return {"frames":len(rows),"first_yaw_deg":values[0],"last_yaw_deg":values[-1],"relative_yaw_range_deg":[min(deltas),max(deltas)],"position_ranges_m":[max(d["position"][i] for d in rows)-min(d["position"][i] for d in rows) for i in range(3)]}
out={}
for folder in sorted(ROOT.glob("raw_turn_20*")):
    rows=[json.loads(x) for x in (folder/"raw.jsonl").read_text(encoding="utf-8").splitlines()]
    out[folder.name]=summary(rows,True)
for folder in sorted(ROOT.glob("lift_reference_20*")):
    if not (folder/"samples.jsonl").exists():
        continue
    rows=[json.loads(x) for x in (folder/"samples.jsonl").read_text(encoding="utf-8").splitlines()]
    baseline=json.loads((folder/"baseline.json").read_text(encoding="utf-8"))
    result={}
    for kind in ("raw","input"):
        stream=[r["data"] for r in rows if r["kind"]==kind]
        result[kind+"_full"]=summary(stream,kind=="raw")
        stream=[r["data"] for r in rows if r["kind"]==kind and r["monotonic"]>=baseline["monotonic"]]
        result[kind+"_after_baseline"]=summary(stream,kind=="raw")
    flags=[r["data"] for r in rows if r["kind"]=="flags" and r["monotonic"]>=baseline["monotonic"]]
    result["fusion_flags_after_baseline"]={"samples":len(flags),"all_ev_xyz_yaw_no_mag":all(all(f[k] for k in ("cs_ev_pos","cs_ev_hgt","cs_ev_yaw")) and not f["cs_mag_hdg"] and not f["cs_mag_3d"] for f in flags)}
    result["validation"]=json.loads((folder/"validation.json").read_text(encoding="utf-8"))
    out[folder.name]=result
(ROOT/"motion_analysis.json").write_text(json.dumps(out,indent=2),encoding="utf-8")
print(json.dumps(out,indent=2))
