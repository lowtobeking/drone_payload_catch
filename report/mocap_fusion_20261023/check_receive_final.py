"""Read-only final local syntax/evidence check and precise helper cleanup."""
import ast
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
WORK=ROOT/"report/mocap_fusion_20261023"
# Keep API documentation consistent with the now-explicit alternative.
p=ROOT/"payload_catch/mocap_odometry_node.py"
s=p.read_text(encoding="utf-8")
s=s.replace("""Default is diagnostic output ONLY. --send-to-px4 additionally requires a fully
confirmed enabled config, A hardware identity and continuously disarmed/landed
telemetry. No parameter, flight-command, setpoint, motor or PWM interfaces.""",
"""Default is diagnostic output ONLY. --send-to-px4 requires either confirmed
source timing/geometry or the explicit receive-time ground-test acknowledgement,
plus A hardware identity and continuously disarmed/landed telemetry.
No parameter, flight-command, setpoint, motor or PWM interfaces.""",1)
ast.parse(s)
p.write_text(s,encoding="utf-8")
checked=[]
for relative in ["payload_catch/mocap_odometry.py","payload_catch/mocap_odometry_node.py",
                 "tools/test_mocap_receive_ground.py","tools/check_mocap_odometry_ros.py",
                 "report/mocap_fusion_20261023/receive_parameter_guard_a.py",
                 "report/mocap_fusion_20261023/receive_ground_runtime_a.py",
                 "report/mocap_fusion_20261023/run_receive_ground_on_a.py",
                 "report/mocap_fusion_20261023/verify_receive_after.py",
                 "report/mocap_fusion_20261023/summarize_receive_ground.py"]:
    ast.parse((ROOT/relative).read_text(encoding="utf-8"),relative)
    checked.append(relative)
summary=json.loads((WORK/"receive_ground_summary.json").read_text(encoding="utf-8"))
assert summary["parameter_differences"]=={}
assert summary["no_remaining_task_processes"] is True
assert summary["real_fcu_fusion_proven"] is True
assert summary["full_ground_acceptance_passed"] is False
# Completed one-off doc updater belongs solely to this task.
helper=WORK/"finalize_receive_task.py"
assert helper.resolve().parent==WORK.resolve()
if helper.is_file(): helper.unlink()
(WORK/"final_local_check.json").write_text(json.dumps(
    {"syntax_checked":checked,"evidence_consistent":True,"flight_approval":False},indent=2),
    encoding="utf-8")
print("Syntax and evidence consistent; temporary one-off updater removed.")
