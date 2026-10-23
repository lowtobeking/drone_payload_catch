"""Summarize stationary tail of actual ROS preview; no ROS publishers."""
from pathlib import Path
import json
import sys
import numpy as np
from scipy.spatial.transform import Rotation
logs=Path(sys.argv[1])
label=sys.argv[2]
reference=Path(sys.argv[3]) if len(sys.argv)>3 else None
capture=json.loads((logs/"capture.json").read_text(encoding="utf-8"))
assert capture["passed"] and not capture["forwarded_to_px4"]
rows=[json.loads(line) for line in (logs/"poses.jsonl").read_text(encoding="utf-8").splitlines()]
preview=[r for r in rows if r["kind"]=="preview"]
assert preview
end=preview[-1]["elapsed_s"]
tail=[r for r in preview if r["elapsed_s"]>=end-3.]
assert len(tail)>150
p=np.asarray([r["position"] for r in tail])
rot=Rotation.from_quat([r["quaternion_xyzw"] for r in tail])
mean=rot.mean()
center=p.mean(axis=0)
position_spread=float(np.max(np.linalg.norm(p-center,axis=1)))
orientation_spread=float(np.max((mean.inv()*rot).magnitude())*180/np.pi)
result=dict(label=label,preview_frame="payload_lab_frd",tail_seconds=3.,samples=len(tail),
            position_local_m=center.tolist(),quaternion_body_to_local_xyzw=mean.as_quat().tolist(),
            roll_pitch_yaw_deg=mean.as_euler("xyz",degrees=True).tolist(),
            max_position_deviation_m=position_spread,max_orientation_deviation_deg=orientation_spread,
            stationary=position_spread<.02 and orientation_spread<2.,
            forwarded_to_px4=False,flight_approval=False)
if reference is not None:
    previous=json.loads(reference.read_text(encoding="utf-8"))
    prev=Rotation.from_quat(previous["quaternion_body_to_local_xyzw"])
    delta=center-np.asarray(previous["position_local_m"])
    relative=prev.inv()*mean
    result["comparison"]=dict(reference_label=previous["label"],
        delta_local_m=delta.tolist(),delta_previous_body_frd_m=prev.inv().apply(delta).tolist(),
        relative_rotation_vector_previous_body_deg=np.rad2deg(relative.as_rotvec()).tolist(),
        relative_euler_xyz_deg=relative.as_euler("xyz",degrees=True).tolist())
(logs/"axis_summary.json").write_text(json.dumps(result,indent=2,allow_nan=False),encoding="utf-8")
print(json.dumps(result,indent=2,allow_nan=False))
if not result["stationary"]:raise SystemExit(2)
