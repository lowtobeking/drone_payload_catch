"""Analyze actual EKF flags and ground gate timing, preserving failures."""
import json
from pathlib import Path
import statistics
ROOT=Path(__file__).resolve().parent
test=sorted(ROOT.glob("fixed_reference_20*"))[-1]
rows=[json.loads(s) for s in (test/"samples.jsonl").read_text(encoding="utf-8").splitlines()]
groups={k:[r for r in rows if r["kind"]==k] for k in {r["kind"] for r in rows}}
out={"run":test.name,"counts":{k:len(v) for k,v in groups.items()}}
for k in ("status","land","flags","local"):
    rs=groups.get(k,[])
    out[k+"_max_gap_s"]=max((b["monotonic"]-a["monotonic"] for a,b in zip(rs,rs[1:])),default=None)
flags=groups.get("flags",[])
ev=[r for r in flags if r["data"]["cs_ev_pos"] and r["data"]["cs_ev_yaw"]]
out["ev_both_active_samples"]=len(ev)
out["ev_flags"]=[{k:r["data"][k] for k in ("cs_ev_pos","cs_ev_yaw","cs_mag_hdg","cs_mag_3d","cs_yaw_align","cs_tilt_align")} | {"monotonic":r["monotonic"]} for r in ev]
if ev:
    first,last=ev[0]["monotonic"],ev[-1]["monotonic"]
    lr=[r for r in groups["local"] if first<=r["monotonic"]<=last]
    out["ev_active_span_s"]=last-first
    out["local_during_ev"]={k:sum(bool(r["data"][k]) for r in lr) for k in ("xy_valid","v_xy_valid","heading_good_for_control","dead_reckoning")}
    out["local_during_ev"]["total"]=len(lr)
    out["yaw_deg_first_last"]=[lr[0]["data"]["heading"]*180/3.141592653589793,lr[-1]["data"]["heading"]*180/3.141592653589793] if lr else []
faults=[r for r in groups.get("bridge",[]) if r["data"].get("latched_fault")]
if faults:
    f=faults[0];stamp=f["monotonic"]
    out["first_fault"]=f
    out["nearby_land"]=[r for r in groups["land"] if abs(r["monotonic"]-stamp)<4]
    out["nearby_status"]=[{k:r["data"][k] for k in ("arming_state","system_id","component_id")}|{"monotonic":r["monotonic"]} for r in groups["status"] if abs(r["monotonic"]-stamp)<4]
out["all_land_true"]=all(r["data"]["landed"] for r in groups.get("land",[]))
out["all_disarmed"]=all(r["data"]["arming_state"]==1 for r in groups.get("status",[]))
(test/"analysis.json").write_text(json.dumps(out,indent=2),encoding="utf-8")
print(json.dumps(out,indent=2))
