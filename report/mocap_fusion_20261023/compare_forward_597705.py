"""Compare user-declared forward 0.597705m using common raw/world references."""
from pathlib import Path
import json
import math
import sys
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from payload_catch.mocap_odometry import load_config
from payload_catch.mocap_transform import Transform
WORK=ROOT/"report/mocap_fusion_20261023"
base=json.loads((WORK/"heading_movement_baseline.json").read_text(encoding="utf-8"))
probe=sorted(p for p in WORK.glob("heading_probe_20*") if (p/"validation.json").exists())[-1]
pv=json.loads((probe/"validation.json").read_text(encoding="utf-8"))
assert pv["passed"]
rows=[json.loads(s) for s in (probe/"samples.jsonl").read_text(encoding="utf-8").splitlines()]
raw=[r["data"] for r in rows if r["kind"]=="raw"]
raw=raw[len(raw)//2:]
endp=np.mean([d["position"] for d in raw],axis=0)
endq=Rotation.from_quat([d["quaternion_xyzw"] for d in raw]).mean().as_quat()
cfg,tr=load_config(ROOT/"config/mocap_a.bridge.yaml")
p0,q0=tr.convert(base["raw_position_mean_m"],base["raw_quaternion_mean_xyzw"])
p1,q1=tr.convert(endp,endq)
dp=p1-p0
r0=Rotation.from_quat(q0)
body=r0.inv().apply(dp)
rel=r0.inv()*Rotation.from_quat(q1)
yaw0=r0.as_euler("xyz")[2]
heading_forward=math.cos(yaw0)*dp[0]+math.sin(yaw0)*dp[1]
heading_right=-math.sin(yaw0)*dp[0]+math.cos(yaw0)*dp[1]
length=float(np.linalg.norm(dp))
result=dict(
    operator_distance_m=.597705,baseline_file="heading_movement_baseline.json",
    probe_directory=probe.name,source_world_delta_m=(endp-np.array(base["raw_position_mean_m"])).tolist(),
    laboratory_frd_delta_m=dp.tolist(),
    baseline_body_frd_delta_m=body.tolist(),
    horizontal_heading_forward_m=float(heading_forward),horizontal_heading_right_m=float(heading_right),
    up_m=float(-dp[2]),distance_3d_m=length,horizontal_distance_m=float(np.linalg.norm(dp[:2])),
    distance_difference_m=length-.597705,
    relative_rotation_rpy_deg=rel.as_euler("xyz",degrees=True).tolist(),
    distance_not_independent_calibration=True,
    source_mean_position_m=endp.tolist(),source_mean_quaternion_xyzw=endq.tolist(),
    static_endpoint_comparison_only=True,flight_approval=False,
    north_verified=False,
    forward_direction_matches=bool(heading_forward>0 and abs(heading_right)<.15 and abs(dp[2])<.15),
    raw_samples=len(raw))
old_probe=WORK/base["reference"]["evidence_directory"]
old_validation=json.loads((old_probe/"validation.json").read_text(encoding="utf-8"))
def difference_deg(a,b):
    return math.degrees(math.atan2(math.sin(math.radians(a-b)),math.cos(math.radians(a-b))))
result["heading_reference_check"]=dict(
    old_px4_yaw_deg=old_validation["px4_yaw"]["mean_deg"],
    new_px4_yaw_deg=pv["px4_yaw"]["mean_deg"],
    old_mocap_yaw_deg=old_validation["mocap_yaw"]["mean_deg"],
    new_mocap_yaw_deg=pv["mocap_yaw"]["mean_deg"],
    px4_heading_change_deg=difference_deg(pv["px4_yaw"]["mean_deg"],old_validation["px4_yaw"]["mean_deg"]),
    mocap_heading_change_deg=difference_deg(pv["mocap_yaw"]["mean_deg"],old_validation["mocap_yaw"]["mean_deg"]),
    old_reference_offset_deg=math.degrees(base["reference"]["yaw_offset_rad"]),
    current_reference_offset_deg=pv["candidate_local_to_px4_yaw"]["mean_deg"],
    reference_offset_change_deg=difference_deg(pv["candidate_local_to_px4_yaw"]["mean_deg"],
                                              math.degrees(base["reference"]["yaw_offset_rad"])),
    current_magnetic_heading_consistent=pv["final"]["flags"]["cs_mag_heading_consistent"],
    current_heading_good_for_control=pv["final"]["local"]["heading_good_for_control"],
    comparable_reference_proven=False,
    conclusion="Do not resnapshot/redefine axes at each location; static stability does not prove a reliable shared reference.",
)
if len(sys.argv)>1 and sys.argv[1]=="--with-fusion":
    if abs(result["heading_reference_check"]["reference_offset_change_deg"]) > 5.:
        raise RuntimeError("reference drift: no valid common-frame fusion comparison; do not recapture magnetic axes per position")
    run=sorted(p for p in WORK.glob("heading_candidate_20*") if (p/"validation.json").exists())[-1]
    v=json.loads((run/"validation.json").read_text(encoding="utf-8"))
    assert v["passed"]
    ref=v["final"]["bridge"]["heading_reference"]
    assert ref["evidence_directory"]==probe.name
    a=ref["yaw_offset_rad"]
    rz=Rotation.from_euler("z",a).as_matrix()
    mat=rz@np.array(ref["base_world_to_local"])
    rows=[json.loads(s) for s in (run/"samples.jsonl").read_text(encoding="utf-8").splitlines()]
    end=v["enabled_at_monotonic"]+27
    tail=[r for r in rows if end-10<r["monotonic"]<end]
    lp=[r["data"] for r in tail if r["kind"]=="local"]
    ev=[r["data"] for r in tail if r["kind"]=="ev_input"]
    raw=[r["data"] for r in tail if r["kind"]=="raw"]
    pnow=np.mean([d["position"] for d in raw],axis=0)
    evnow=np.mean([d["position"] for d in ev],axis=0)
    fused=np.array([np.mean([d[k] for d in lp]) for k in ("x","y","z")])
    oldmat=np.array(base["transform_world_to_local"])
    # Re-express both old input and old fused XY into THIS snapshot reference before subtracting.
    pbase=mat@np.array(base["raw_position_mean_m"])
    oldfused=np.array(base["px4_position_mean_m"])
    rebased_old_fused=mat@oldmat.T@oldfused
    evdelta=evnow-pbase
    fused_delta=fused-rebased_old_fused
    new_base_yaw=Rotation.from_matrix(mat@oldmat.T).as_euler("xyz")[2]+base["px4_heading_mean_rad"]
    fused_forward=math.cos(new_base_yaw)*fused_delta[0]+math.sin(new_base_yaw)*fused_delta[1]
    fused_right=-math.sin(new_base_yaw)*fused_delta[0]+math.cos(new_base_yaw)*fused_delta[1]
    flags=[r["data"] for r in tail if r["kind"]=="flags"]
    result["fusion"]=dict(run_directory=run.name,ground_static_passed=v["passed"],
        yaw_offset_deg=math.degrees(a),
        baseline_reexpressed_in_new_reference=True,
        ev_delta_new_reference_m=evdelta.tolist(),fused_delta_new_reference_m=fused_delta.tolist(),
        fused_forward_m=float(fused_forward),fused_right_m=float(fused_right),
        fused_horizontal_distance_m=float(np.linalg.norm(fused_delta[:2])),
        current_input_mean_position_m=evnow.tolist(),current_fused_mean_position_m=fused.tolist(),
        current_horizontal_residual_m=float(np.linalg.norm(fused[:2]-evnow[:2])),
        all_tail_xy_vxy_heading_valid=all(all(d[k] for k in ("xy_valid","v_xy_valid","heading_good_for_control")) for d in lp),
        all_tail_ev_position_yaw=all(d["cs_ev_pos"] and d["cs_ev_yaw"] for d in flags),
        all_tail_not_dead_reckoning=all(not d["dead_reckoning"] for d in lp),
        ev_active_span_s=v["ev_active_span_s"],local_all_valid_samples=v["local_all_valid_samples"],
        bridge_final=v["final"]["bridge"],lease=v["lease"],
        height_comparison_valid=False,
        explanation="Z remains barometric and has an independent reference; do not interpret fused Z delta as mocap motion.",
        continuous_motion_observed=False)
    # Preserve the measured endpoint as the starting point for right-move verification.
    checkpoint=dict(source_run=run.name,raw_position_mean_m=pnow.tolist(),
        raw_quaternion_mean_xyzw=Rotation.from_quat([d["quaternion_xyzw"] for d in raw]).mean().as_quat().tolist(),
        transform_world_to_local=mat.tolist(),px4_position_mean_m=fused.tolist(),
        ev_position_mean_m=evnow.tolist(),px4_heading_mean_rad=float(np.mean([d["heading"] for d in lp])),
        reference=ref,frame="px4_heading_aligned_ned",north_verified=False,
        height_source="barometer, not external vision")
    (WORK/"forward_endpoint_597705.json").write_text(json.dumps(checkpoint,indent=2),encoding="utf-8")
(WORK/"forward_597705_summary.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
print(json.dumps(result,indent=2))
