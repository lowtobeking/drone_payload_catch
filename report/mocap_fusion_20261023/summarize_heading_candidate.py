"""Preserve qualified ground-only heading result and before/after full parameter comparison."""
from pathlib import Path
import json
import math
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[2]
WORK=ROOT/"report/mocap_fusion_20261023"
runs=sorted(p for p in WORK.glob("heading_candidate_20*") if (p/"validation.json").exists())
run=runs[-1]
v=json.loads((run/"validation.json").read_text(encoding="utf-8"))
assert v["passed"]
before=json.loads((run/"parameters_before.json").read_text(encoding="utf-8"))
post=sorted(p for p in WORK.glob("parameters_20*/parameters_backup.json"))[-1]
after=json.loads(post.read_text(encoding="utf-8"))
assert before["complete"] and after["complete"]
diff={k:{"before":before["parameters"].get(k),"after":after["parameters"].get(k)}
      for k in set(before["parameters"])|set(after["parameters"])
      if before["parameters"].get(k)!=after["parameters"].get(k)}
rows=[json.loads(line) for line in (run/"samples.jsonl").read_text(encoding="utf-8").splitlines()]
end=v["enabled_at_monotonic"]+27
tail=[r for r in rows if end-10<r["monotonic"]<end]
lp=[r["data"] for r in tail if r["kind"]=="local"]
flags=[r["data"] for r in tail if r["kind"]=="flags"]
raw=[r["data"] for r in tail if r["kind"]=="raw"]
ev=[r["data"] for r in tail if r["kind"]=="ev_input"]
ref=v["final"]["bridge"]["heading_reference"]
a=ref["yaw_offset_rad"]
rz=np.array([[math.cos(a),-math.sin(a),0.],[math.sin(a),math.cos(a),0.],[0.,0.,1.]])
mat=rz@np.array(ref["base_world_to_local"])
baseline=dict(
    frame="px4_heading_aligned_ned",north_verified=False,
    reference=ref,transform_world_to_local=mat.tolist(),
    raw_position_mean_m=np.mean([r["position"] for r in raw],axis=0).tolist(),
    raw_quaternion_mean_xyzw=np.mean([r["quaternion_xyzw"] for r in raw],axis=0).tolist(),
    ev_position_mean_m=np.mean([r["position"] for r in ev],axis=0).tolist(),
    px4_position_mean_m=[float(np.mean([r[k] for r in lp])) for k in ("x","y","z")],
    px4_heading_mean_rad=float(np.mean([r["heading"] for r in lp])),
    height_source="barometer, not external vision",movement_not_yet_tested=True,
)
audit=json.loads((WORK/"receive_final_audit.json").read_text(encoding="utf-8"))
status=[r["data"] for r in rows if r["kind"]=="status"]
land=[r["data"] for r in rows if r["kind"]=="land"]
summary=dict(ground_static_candidate_passed=True,flight_approval=False,north_verified=False,
    reference_method="px4_magnetic_snapshot",yaw_rotation_deg=math.degrees(a),
    source_run=run.name,source_probe=ref["evidence_directory"],
    ev_both_active_samples=v["ev_both_active_samples"],ev_both_active_span_s=v["ev_active_span_s"],
    local_all_valid_samples=v["local_all_valid_samples"],
    tail_position_and_velocity_and_heading_all_valid=all(all(d[k] for k in (
        "xy_valid","v_xy_valid","heading_good_for_control")) for d in lp),
    tail_not_dead_reckoning=all(not d["dead_reckoning"] for d in lp),
    tail_ev_pos_yaw=all(d["cs_ev_pos"] and d["cs_ev_yaw"] for d in flags),
    all_observed_disarmed=bool(status) and all(d["arming_state"]==1 for d in status),
    all_observed_landed=bool(land) and all(d["landed"] for d in land),
    parameter_count=len(after["parameters"]),parameter_differences=diff,
    restored_EKF2_EV_CTRL=after["parameters"]["EKF2_EV_CTRL"]["value"],
    UXRCE_DDS_SYNCT=after["parameters"]["UXRCE_DDS_SYNCT"]["value"],
    final_bridge=v["final"]["bridge"],parameter_lease=v["lease"],
    post_parameter_snapshot=str(post.relative_to(ROOT)),audit=audit,baseline=baseline,
    earlier_failures=[dict(directory=p.name,result=json.loads((p/"validation.json").read_text(encoding="utf-8")))
                      for p in runs[:-1]],
    limitations=["unknown absolute north accuracy / indoor magnetic error",
                 "static single-pose test only","unknown marker-to-IMU offset",
                 "receive timestamp includes acquisition/processing/network delay",
                 "barometric height not validated for indoor takeoff",
                 "startup dropout occurred; warmup does not prove root cause or long-term stability"])
assert not diff and summary["restored_EKF2_EV_CTRL"]==0 and summary["UXRCE_DDS_SYNCT"]==1
assert summary["all_observed_disarmed"] and summary["all_observed_landed"]
assert not audit["remaining_task_processes"]
assert summary["tail_position_and_velocity_and_heading_all_valid"] and summary["tail_ev_pos_yaw"]
(WORK/"heading_candidate_summary.json").write_text(json.dumps(summary,indent=2),encoding="utf-8")
(WORK/"heading_movement_baseline.json").write_text(json.dumps(baseline,indent=2),encoding="utf-8")
cfg=yaml.safe_load((ROOT/"config/mocap_a.receive_ground.yaml").read_text(encoding="utf-8"))
cfg["transform"]["world_to_local"]=mat.tolist()
cfg["output_pose_frame"]="px4_heading_aligned_ned"
cfg["heading_reference"]=ref
(run/"candidate_config.yaml").write_text(yaml.safe_dump(cfg,sort_keys=False),encoding="utf-8")
print(json.dumps({k:v for k,v in summary.items() if k not in ("earlier_failures","final_bridge","baseline")},indent=2))
