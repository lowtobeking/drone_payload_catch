"""Offline evidence summary and exact full-parameter comparison."""
from pathlib import Path
import ast
import json
import math
ROOT=Path(__file__).resolve().parents[2]
WORK=ROOT/"report/mocap_fusion_20261023"
runs=sorted(p for p in WORK.glob("receive_ground_20*") if (p/"validation.json").exists())
run=runs[-1]
v=json.loads((run/"validation.json").read_text(encoding="utf-8"))
before=json.loads((run/"parameters_before.json").read_text(encoding="utf-8"))
afterpath=sorted(p for p in WORK.glob("parameters_20*/parameters_backup.json")
                 if p.parent.name>"parameters_20261023T074000Z")[-1]
after=json.loads(afterpath.read_text(encoding="utf-8"))
assert before["complete"] and after["complete"]
diff={k:{"before":before["parameters"].get(k),"after":after["parameters"].get(k)}
      for k in set(before["parameters"])|set(after["parameters"])
      if before["parameters"].get(k)!=after["parameters"].get(k)}
rows=[json.loads(line) for line in (run/"samples.jsonl").read_text(encoding="utf-8").splitlines()]
end=v["enabled_at_monotonic"]+27
tail=[r for r in rows if end-10<r["monotonic"]<end]
lp=[r["data"] for r in tail if r["kind"]=="local"]
statuses=[r["data"] for r in rows if r["kind"]=="status"]
land=[r["data"] for r in rows if r["kind"]=="land"]
flags=[r["data"] for r in tail if r["kind"]=="flags"]
inputs=[r["data"] for r in tail if r["kind"]=="ev_input"]
localmean=[sum(d[k] for d in lp)/len(lp) for k in ("x","y","z")]
evmean=[sum(d["position"][i] for d in inputs)/len(inputs) for i in range(3)]
audit=json.loads((WORK/"receive_final_audit.json").read_text(encoding="utf-8"))
summary=dict(
    real_fcu_fusion_proven=v["ev_both_active_samples"]>5,
    full_ground_acceptance_passed=v["passed"],flight_approval=False,
    run_directory=str(run.relative_to(ROOT)),server_software="FZMotion 3.0.0.9",
    temporary_EKF2_EV_CTRL=9,restored_EKF2_EV_CTRL=after["parameters"]["EKF2_EV_CTRL"]["value"],
    UXRCE_DDS_SYNCT=after["parameters"]["UXRCE_DDS_SYNCT"]["value"],
    parameter_count=len(after["parameters"]),parameter_differences=diff,
    all_observed_status_disarmed=bool(statuses) and all(d["arming_state"]==1 for d in statuses),
    all_observed_land_true=bool(land) and all(d["landed"] for d in land),
    ev_both_active_samples=v["ev_both_active_samples"],ev_both_active_span_s=v["ev_active_span_s"],
    tail_local_position_samples=len(lp),
    tail_all_xy_vxy_valid=all(d["xy_valid"] and d["v_xy_valid"] for d in lp),
    tail_all_dead_reckoning_false=all(not d["dead_reckoning"] for d in lp),
    tail_heading_good_for_control=all(d["heading_good_for_control"] for d in lp),
    tail_ev_pos_and_yaw=all(d["cs_ev_pos"] and d["cs_ev_yaw"] for d in flags),
    mean_ev_position_m=evmean,mean_fused_position_m=localmean,
    horizontal_static_mean_difference_m=math.hypot(localmean[0]-evmean[0],localmean[1]-evmean[1]),
    height_fusion_enabled=False,
    final_bridge=v["final"]["bridge"],parameter_lease=v["lease"],
    no_remaining_task_processes=not audit["remaining_task_processes"],
    audit=audit,source_sampling_time_verified=False,lever_arm_measured=False,
    after_snapshot=str(afterpath.relative_to(ROOT)),
)
assert not diff and summary["all_observed_status_disarmed"] and summary["all_observed_land_true"]
assert summary["restored_EKF2_EV_CTRL"]==0 and summary["UXRCE_DDS_SYNCT"]==1
assert summary["no_remaining_task_processes"]
(WORK/"receive_ground_summary.json").write_text(json.dumps(summary,indent=2),encoding="utf-8")
for p in [ROOT/"payload_catch/mocap_odometry.py",ROOT/"payload_catch/mocap_odometry_node.py",
          ROOT/"tools/test_mocap_receive_ground.py",ROOT/"tools/check_mocap_odometry_ros.py",
          WORK/"receive_ground_runtime_a.py",WORK/"receive_parameter_guard_a.py",WORK/"run_receive_ground_on_a.py"]:
    ast.parse(p.read_text(encoding="utf-8"),str(p))
print(json.dumps(summary,indent=2))
