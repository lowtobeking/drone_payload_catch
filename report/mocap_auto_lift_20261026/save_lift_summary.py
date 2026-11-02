"""Preserve partial lift evidence and restoration; never mark a missing return passed."""
import json
from pathlib import Path
WORK=Path(__file__).resolve().parent
ROOT=WORK.parents[1]
test=sorted(WORK.glob("lift_reference_20*"))[-1]
post=sorted(WORK.glob("after_20*"))[-1]
v=json.loads((test/"validation.json").read_text())
a=json.loads((post/"comparison.json").read_text())
b=v["phases"]["baseline"];e=v["phases"]["elevated"]
mi=-(e["input_xyz"][2]-b["input_xyz"][2])
mp=-(e["px4_xyz"][2]-b["px4_xyz"][2])
result=dict(run=test.name,post=post.name,up_input_m=mi,up_px4_m=mp,
    displacement_difference_m=abs(mi-mp),plateau_sample_counts=dict(local=e["local_samples"],input=e["input_samples"]),
    same_direction=mi>0 and mp>0,return_recorded="returned" in v["phases"],full_test_passed=v["passed"],
    error=v.get("error"),restoration=v["lease"],post_audit_passed=a["passed"],
    parameter_differences=a["parameter_differences"],flight_approval=False)
(WORK/"final_summary.json").write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
doc=f"""# A机自动抬高记录与未完成放回（2026-10-26）

## 本轮事实

用户确认抬高并保持、随后确认已放回。自动检测保存baseline与elevated，动捕抬高{mi:.6f}米，PX4抬高{mp:.6f}米，位移差{abs(mi-mp):.6f}米。该稳定段PX4局部样本{e['local_samples']}、动捕输入{e['input_samples']}；方向一致。实际物理距离未独立测量，不能把数据接近当绝对精度。

returned未保存：96秒人工窗口在用户接到放回提示前后已结束；原程序passed=false，错误{v.get('error')}。因此只有抬高片段证据，整轮往返未通过。没有跨运行补点或伪造returned。瞬态残差与时延尚待分析，本报告不把稳定段位移接近扩展为全程动态验收。

## 恢复

EV_CTRL0/HGT_REF1/MAG_TYPE0/SYNCT1已恢复并重启核对，独立完整1127参数审计passed={a['passed']}。唯一差异{json.dumps(a['parameter_differences'])}，为气压计自动相对校准候选，不强行覆写。Jetson boot/current/原Agent1155与relay1156不变，测试进程已退出，当前无常驻融合。没有解锁/飞行/PWM/网络修改。

## 证据与下一步

report/mocap_auto_lift_20261026/{test.name}保存完整日志、samples.jsonl、基准/抬高段与恢复记录；{post.name}/comparison.json独立恢复审计通过。分离机载任务避免SSH断线终止采集，源120秒/桥接115秒与独立lease保留。接下来每轮只采一个人工动作，优先航向左转；往返及物理遮挡/精确延迟/安装偏移仍待。
"""
(ROOT/"docs/A机自动抬高片段验证_20261026.md").write_text(doc,encoding="utf-8")
p=ROOT/"MEMORY.md"
with p.open("a",encoding="utf-8") as f:
    f.write(f"\n\n## 2026-10-26 自动抬高片段记录（最新）\n\n自动有界独立机载任务{test.name}保存baseline/elevated；动捕上升{mi:.6f}m、PX4{mp:.6f}m，差{abs(mi-mp):.6f}m，物理距离未独立测量。用户已抬高/随后放回，但returned未捕获，96秒窗口超时，原passed=false，不得记全程通过。参数EV0/HGT1/MAG0/SYNCT1恢复重启核验；{post.name}1127参数独立审计通过，仅CAL_BARO0_OFF自动校准候选差异，进程无残留，current/Jetsonboot/原服务不变。证据report/mocap_auto_lift_20261026/final_summary.json，说明docs/A机自动抬高片段验证_20261026.md。下一步每轮只采单个动作，避免提示延迟使往返超时。\n")
print(json.dumps(result,ensure_ascii=False,indent=2))
