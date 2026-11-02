"""Save right-turn and forward-translation evidence without overwriting earlier runs."""
from datetime import datetime
import bisect
import json
import math
from pathlib import Path
import statistics
PROJECT=Path(__file__).resolve().parents[2]
REPORT=PROJECT/"report"
def read(p):
    return json.loads(p.read_text(encoding="utf-8"))
summaries={}
for name,phase in (("mocap_right_20261026","right_turn"),("mocap_forward_20261026","forward_move")):
    root=REPORT/name
    run=sorted(root.glob("lift_reference_20*"))[-1]
    val=read(run/"validation.json")
    audit_paths=[p for p in root.glob("after_*/comparison.json") if read(p)["test"]==str(run)]
    assert len(audit_paths)==1,(name,audit_paths)
    audit=read(audit_paths[0])
    assert audit["passed"] and audit["complete"] and audit["parameter_count"]==1127
    assert not audit["remaining_task_processes"]
    rows=[json.loads(line) for line in (run/"samples.jsonl").read_text(encoding="utf-8").splitlines()]
    base=val.get("phases",{}).get("baseline")
    start=base["monotonic"] if base else float("-inf")
    flags=[r["data"] for r in rows if r["kind"]=="flags" and r["monotonic"]>=start]
    valid_flags=bool(flags) and all(all(d[k] for k in ("cs_ev_pos","cs_ev_hgt","cs_ev_yaw")) and not d["cs_mag_hdg"] and not d["cs_mag_3d"] for d in flags)
    bridge=[r["data"] for r in rows if r["kind"]=="bridge"]
    fault=any(r.get("latched_fault") for r in bridge)
    local=[r for r in rows if r["kind"]=="local"]
    times=[r["monotonic"] for r in local]
    comparisons=[]
    for r in rows:
        if r["kind"]!="input" or r["monotonic"]<start:
            continue
        j=bisect.bisect_left(times,r["monotonic"])
        choices=[i for i in (j-1,j) if 0<=i<len(local)]
        if not choices:
            continue
        point=min((local[i] for i in choices),key=lambda q:abs(q["monotonic"]-r["monotonic"]))
        age=abs(point["monotonic"]-r["monotonic"])
        if age>.03:
            continue
        err=math.hypot(point["data"]["x"]-r["data"]["position"][0],point["data"]["y"]-r["data"]["position"][1])
        comparisons.append({"monotonic":r["monotonic"],"nearest_local_time_difference_s":age,"horizontal_disagreement_m":err})
    (run/"nearest_receive_time_comparison.json").write_text(json.dumps(comparisons,indent=2),encoding="utf-8")
    errors=sorted(p["horizontal_disagreement_m"] for p in comparisons)
    compare={"method":"nearest local ROS receipt time within 30ms; diagnostic only, not capture time/latency calibrated","pairs":len(comparisons),"mean_horizontal_disagreement_m":statistics.mean(errors) if errors else None,"p95_horizontal_disagreement_m":errors[int(.95*(len(errors)-1))] if errors else None,"max_horizontal_disagreement_m":max(errors) if errors else None}
    raw=[r["data"] for r in rows if r["kind"]=="raw"]
    groups=[]
    first=0
    for i in range(1,len(raw)+1):
        same=i<len(raw) and raw[i]["position"]==raw[first]["position"] and raw[i]["quaternion_xyzw"]==raw[first]["quaternion_xyzw"]
        if same:
            continue
        duration=(raw[i-1]["receive_monotonic_us"]-raw[first]["receive_monotonic_us"])/1e6
        if duration>.1:
            groups.append({"frames":i-first,"duration_s":duration})
        first=i
    stage=val.get("phases",{}).get(phase)
    endpoint=math.hypot(stage["px4_xyz"][0]-stage["input_xyz"][0],stage["px4_xyz"][1]-stage["input_xyz"][1]) if stage else None
    out={"updated":datetime.now().astimezone().isoformat(),"run":run.name,"phase":phase,"ground_test_passed":bool(val["passed"] and valid_flags and not fault),"flight_approval":False,"validation":val,"audit":audit,"fusion_flags_after_baseline":len(flags),"all_observed_flags_ev_xyz_yaw_no_mag":valid_flags,"latched_bridge_fault":fault,"exact_raw_pose_repeats_over_100ms":groups,"endpoint_horizontal_mean_disagreement_m":endpoint,"receipt_time_pair_diagnostic":compare,"persistent_fusion_running":False}
    (root/"final_summary.json").write_text(json.dumps(out,indent=2),encoding="utf-8")
    summaries[name]=out
right=summaries["mocap_right_20261026"]
forward=summaries["mocap_forward_20261026"]
a=right["validation"]["yaw"]
b=forward["validation"].get("translation",{})
append=f"""
## 2026-10-26 后续右转与前移验收（最新）

本段为前述午后左转之后的新增进度，先前失败与限制仍保留。

| 项目 | 本轮结果 | 数据 |
| --- | --- | --- |
| 右转真实融合 | {"通过" if right["ground_test_passed"] else "未通过"} | 动捕{a["input_change_deg"]:.6f}°，PX4 {a["px4_change_deg"]:.6f}°，变化差{a["displacement_difference_deg"]:.6f}°；基准与right_turn稳定段均保存 |
| 前移真实融合停稳段 | {"通过" if forward["ground_test_passed"] else "未通过"} | 动捕向前{b.get("input_forward_m",float("nan")):.6f}m，PX4向前{b.get("px4_forward_m",float("nan")):.6f}m；水平位移向量差{b.get("horizontal_displacement_difference_m",float("nan"))*1000:.3f}mm，停稳端点差{b.get("horizontal_endpoint_difference_m",float("nan"))*1000:.3f}mm |
| 真实融合向右平移 | 尚待 | 右转不等于右移 |
| 抬高后放回全程 | 尚待 | 先前仅抬高稳定片段，返回稳定段未验收 |
| 飞行动态精度/时延 | 尚待 | 本轮均值与停稳一致性不代替运动全过程误差或时延标定 |

右转稳定段水平位置均值差约{right["endpoint_horizontal_mean_disagreement_m"]*100:.2f}cm，原因未确定。前移专门等待输入位置/航向稳定、额外保持8秒，再取4秒均值，停稳后位置已对齐。本轮前移约{b.get("input_forward_m",float("nan"))*100:.2f}cm，伴随横向偏移{b.get("input_cross_m",float("nan"))*100:.2f}cm；未独立测量实际距离，不宣称动捕具有亚毫米绝对精度，也不由停稳相符认定转向时水平误差问题已解决。最近ROS接收时间配对的误差诊断单独保存，不冒充同步采集时刻比较。

两轮均保持XYZ/yaw融合启用、磁融合关闭；本轮原始数据没有持续100ms以上完全重复位姿，没有锁存桥接故障。午前83帧重复根因仍未查清，原保护阈值未放宽。

两轮均已恢复EV0/HGT1/MAG0/SYNCT1，飞控恢复重启核验，完整1127参数独立审计通过；额外差异仅CAL_BARO0_OFF自动校准候选，完整原值在各comparison.json，未强行覆写。Jetson boot、正式部署链接、Agent1019/relay1024/网络辅助2770保持，任务进程无残留，不常驻融合，没有解锁/电机/PWM/网络更改。

证据：
- report/mocap_right_20261026/final_summary.json；成功run={right["run"]}，审计={Path(right["audit"]["post"]).name}。
- report/mocap_forward_20261026/final_summary.json；run={forward["run"]}，审计={Path(forward["audit"]["post"]).name}。
- 各run保留samples.jsonl、baseline、动作稳定段、参数前后快照、运行清理和nearest_receive_time_comparison.json。
"""
doc=PROJECT/"docs/A机动捕PX4验收进度与继续入口_20261026.md"
text=doc.read_text(encoding="utf-8")
text=text.replace("右转真实融合、水平平移、抬高放回全程及飞行条件仍未验收完毕。","右转及前移停稳段本轮也已通过；向右平移、抬高放回全程及飞行条件仍未验收完毕。")
text=text.replace("| 右转真实融合 | 尚待 | 本次右转只采原始VRPN，没有写入PX4 |","| 右转真实融合 | 后续本轮通过 | 动捕58.269°、PX4 57.832°，变化差0.437°，见末尾新增结果 |")
text=text.replace("| 前/右平移真实融合 | 尚待 | 本次手持左转带来的位置变化不作为专门平移验收 |","| 前/右平移真实融合 | 前移停稳段通过；向右平移尚待 | 前移停稳数据见末尾，运动时延/动态精度仍待 |")
text=text.replace("1. 做单动作右转的真实PX4融合，核对符号、稳定段、跳变和完整恢复。","1. 右转单动作本轮通过；继续做单动作向右平移，核对符号、稳定段和完整恢复。")
text=text.replace("2. 分别做前/右平移和抬高—放回的真实融合；每轮新基准，保持独立参数回退，避免把原始预览当融合验收。","2. 前移停稳段本轮通过；继续向右平移和抬高—放回的真实融合，每轮新基准和独立参数回退。")
doc.write_text(text+append,encoding="utf-8",newline="\n")
(PROJECT/"2026-10-26_A机验收进度.md").write_text(f"""# 2026-10-26 A机验收进度

静态XYZ与航向、水平左转、水平右转、前移停稳段的实机地面融合验收已通过。

- 左转：动捕61.93°，PX4 61.72°，变化差0.21°。
- 右转：动捕58.27°，PX4 57.83°，变化差0.44°。
- 前移停稳：动捕向前76.45cm，PX4 76.52cm，水平位移向量差约0.83mm，仅一致性检查，不是绝对精度标定。

本轮参数恢复和任务清理均已独立复查通过，当前没有持续融合。向右平移、抬高放回全程、动态误差/源时延、午前冻结根因和飞行准备仍待。

[详细验收表、恢复状态与继续入口](docs/A机动捕PX4验收进度与继续入口_20261026.md)
""",encoding="utf-8",newline="\n")
with (PROJECT/"MEMORY.md").open("a",encoding="utf-8",newline="\n") as f:
    f.write("\n\n"+append.replace("## 2026-10-26 后续右转与前移验收（最新）","## 2026-10-26 右转和前移停稳段真实融合通过（最新）"))
print(json.dumps({name:{"ground_test_passed":s["ground_test_passed"],"flags":s["fusion_flags_after_baseline"],"endpoint_horizontal_mean_disagreement_m":s["endpoint_horizontal_mean_disagreement_m"],"receipt_diagnostic":s["receipt_time_pair_diagnostic"],"audit_passed":s["audit"]["passed"]} for name,s in summaries.items()},ensure_ascii=False))
