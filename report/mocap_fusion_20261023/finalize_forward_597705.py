"""Final offline parameter, cleanup and documentation verification for the forward endpoint."""
from pathlib import Path
import ast
import json
ROOT=Path(__file__).resolve().parents[2]
WORK=ROOT/"report/mocap_fusion_20261023"
before=json.loads((WORK/"parameters_20261023T081620Z/parameters_backup.json").read_text(encoding="utf-8"))
after=json.loads((WORK/"parameters_20261023T082943Z/parameters_backup.json").read_text(encoding="utf-8"))
diff={k:{"before":before["parameters"].get(k),"after":after["parameters"].get(k)}
      for k in set(before["parameters"])|set(after["parameters"])
      if before["parameters"].get(k)!=after["parameters"].get(k)}
audit=json.loads((WORK/"forward_final_audit.json").read_text(encoding="utf-8"))
summary=json.loads((WORK/"forward_597705_summary.json").read_text(encoding="utf-8"))
assert before["complete"] and after["complete"] and len(after["parameters"])==1127
assert not diff and not audit["remaining_task_processes"]
assert after["parameters"]["EKF2_EV_CTRL"]["value"]==0
assert after["parameters"]["UXRCE_DDS_SYNCT"]["value"]==1
checked=[]
for name in ("compare_forward_597705.py","run_heading_candidate_on_a.py","verify_forward_after.py"):
    p=WORK/name
    ast.parse(p.read_text(encoding="utf-8"),str(p))
    checked.append(name)
report=dict(parameter_count=1127,parameter_differences=diff,
    EKF2_EV_CTRL=0,UXRCE_DDS_SYNCT=1,task_processes_remaining=0,
    core_bridge_modified_this_turn=False,flight_or_actuator_commands=0,
    fusion_test_not_restarted=True,
    heading_reference_change_deg=summary["heading_reference_check"]["reference_offset_change_deg"],
    heading_guard_observed_refusal_before_ssh=True,syntax_checked=checked,audit=audit)
(WORK/"forward_final_verification.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
p=ROOT/"MEMORY.md"
s=p.read_text(encoding="utf-8")
note="""

## 2026-10-23 前移59.7705厘米端点检查：磁航向参考跨位置/时间不稳定（最新）

用户回复前移约59.7705cm。A同boot/serial、SSH.123→.101正常。本轮**只读**heading_probe_20261023T082553Z，1601配对，末半801raw位姿与heading_movement_baseline对比：总位移60.0873cm（比用户值+0.3168cm），按原机头水平分解前+59.8868cm/左4.9057cm，up约0.0072cm，相对yaw+1.7587°。方向基本符合；不是独立精度或连续运动验收。

发现关键新问题：前后动捕yaw -1.274543°→+.483709°，PX4 -47.109135°→-86.288412°（变化-39.1793°），候选参考旋转 -45.834592°→-86.772121°，相对参考变化-40.9375°。当前mag_heading_consistent=false、heading_good=false、EV_CTRL0且不在融合。中间有上次临时融合/恢复，不能断言全部是移动本身/磁干扰所致；但**磁航向快照不能作为已稳定标定，静态通过不等于跨位置/时间可用**。

没有重采新磁偏移来启用融合、没有更新旧时间冒充新参考。run_heading_candidate_on_a.py增加已有基准与新旋转>5°本地拒绝，实测-40.938°在任何SSH暂存/参数写前拒绝。离线--with-fusion也拒绝漂移；未修改核心桥接或门禁。只读完整1127参数与上轮恢复后差异0，EV_CTRL0/SYNCT1、任务进程0，current b4453dd/Agent1011/relay1013正常，无硬件/网络/时钟改变。源子进程正常退出。

下一步先保持当前位置，不继续右移，把航向参考跨位置/时间变化排查清楚并建立固定独立参考，不能每个位置重设世界轴。此前“静态三标志通过”为单次事实，不是可靠北向/飞行标定完成；本次尚未通过PX4融合位移验证。高度、偏移、时延、失效保护仍待。文档docs/A机前移验证与航向参考漂移_20261023.md；证据forward_597705_summary.json、forward_final_verification.json、forward_final_audit.json、parameters_20261023T082943Z。保留原heading_movement_baseline未覆盖，不把新点假称已融合基准。
"""
assert "前移59.7705厘米端点检查：磁航向参考跨位置/时间不稳定（最新）" not in s
p.write_text(s+note,encoding="utf-8")
p=ROOT/"docs/A机航向参考与静态融合验证_20261023.md"
s=p.read_text(encoding="utf-8")
idx=s.index("\n")
s=s[:idx+1]+"""
> 后续前移验证发现：动捕机头仅变化约1.76°，PX4航向却变化约39°，参考偏移变化约40.94°。当前快照法不能视为稳定标定，未重新启用融合。见 [前移验证与参考漂移](A机前移验证与航向参考漂移_20261023.md)。以下静态通过只描述当次单位置结果。
"""+s[idx+1:]
p.write_text(s,encoding="utf-8")
helper=WORK/"record_forward_heading_issue.py"
assert helper.resolve().parent==WORK.resolve()
if helper.is_file(): helper.unlink()
print(json.dumps(report,indent=2))
