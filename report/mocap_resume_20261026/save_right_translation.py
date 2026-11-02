"""Persist right-translation ground evidence and update acceptance progress."""
from datetime import datetime
import bisect
import json
import math
from pathlib import Path
import statistics

PROJECT=Path(__file__).resolve().parents[2]
ROOT=PROJECT/"report/mocap_right_translation_20261026"
def read(path):
    return json.loads(path.read_text(encoding="utf-8"))
run=sorted(ROOT.glob("lift_reference_20*"))[-1]
validation=read(run/"validation.json")
audits=[p for p in ROOT.glob("after_*/comparison.json") if read(p)["test"]==str(run)]
assert len(audits)==1
audit=read(audits[0])
assert audit["passed"] and audit["complete"] and audit["parameter_count"]==1127
assert not audit["remaining_task_processes"] and audit["services_unchanged"] and audit["current_unchanged"]
rows=[json.loads(line) for line in (run/"samples.jsonl").read_text(encoding="utf-8").splitlines()]
base=validation.get("phases",{}).get("baseline")
start=base["monotonic"] if base else float("-inf")
flags=[r["data"] for r in rows if r["kind"]=="flags" and r["monotonic"]>=start]
valid_flags=bool(flags) and all(all(f[k] for k in ("cs_ev_pos","cs_ev_hgt","cs_ev_yaw")) and not f["cs_mag_hdg"] and not f["cs_mag_3d"] for f in flags)
faults=[r["data"] for r in rows if r["kind"]=="bridge" and r["data"].get("latched_fault")]
raw=[r["data"] for r in rows if r["kind"]=="raw"]
repeats=[]
first=0
for index in range(1,len(raw)+1):
    same=index<len(raw) and raw[index]["position"]==raw[first]["position"] and raw[index]["quaternion_xyzw"]==raw[first]["quaternion_xyzw"]
    if same:
        continue
    span=(raw[index-1]["receive_monotonic_us"]-raw[first]["receive_monotonic_us"])/1e6
    if span>=.1:
        repeats.append({"frames":index-first,"span_s":span})
    first=index
local=[r for r in rows if r["kind"]=="local"]
times=[r["monotonic"] for r in local]
paired=[]
for row in rows:
    if row["kind"]!="input" or row["monotonic"]<start:
        continue
    index=bisect.bisect_left(times,row["monotonic"])
    candidates=[local[i] for i in (index-1,index) if 0<=i<len(local)]
    if not candidates:
        continue
    point=min(candidates,key=lambda r:abs(r["monotonic"]-row["monotonic"]))
    gap=abs(point["monotonic"]-row["monotonic"])
    if gap>.03:
        continue
    paired.append({"monotonic":row["monotonic"],"nearest_receive_time_gap_s":gap,
        "horizontal_disagreement_m":math.hypot(point["data"]["x"]-row["data"]["position"][0],point["data"]["y"]-row["data"]["position"][1])})
(run/"nearest_receive_time_comparison.json").write_text(json.dumps(paired,indent=2),encoding="utf-8")
errors=sorted(p["horizontal_disagreement_m"] for p in paired)
diagnostic={"method":"nearest ROS receipt within 30ms; not calibrated acquisition-time or latency comparison",
    "pairs":len(errors),"mean_m":statistics.mean(errors) if errors else None,
    "p95_m":errors[int(.95*(len(errors)-1))] if errors else None,"max_m":max(errors) if errors else None}
attempt_history=[]
for attempt in sorted(ROOT.glob("lift_reference_20*")):
    if not (attempt/"validation.json").exists():
        continue
    matches=[p for p in ROOT.glob("after_*/comparison.json") if read(p)["test"]==str(attempt)]
    attempt_history.append({"run":attempt.name,"validation":read(attempt/"validation.json"),
        "audits":[read(p) for p in matches]})
passed=bool(validation["passed"] and valid_flags and not faults)
result={"updated":datetime.now().astimezone().isoformat(),"run":run.name,"ground_test_passed":passed,
    "flight_approval":False,"validation":validation,"audit":audit,"fusion_flags_samples":len(flags),
    "all_observed_flags_ev_xyz_yaw_no_mag":valid_flags,"latched_bridge_faults":faults,
    "retry_correction":read(ROOT/"retry_correction.json"),
    "attempt_history":attempt_history,"proof_clock_fix":read(ROOT/"proof_clock_fix.json"),
    "exact_raw_pose_repeats_ge_100ms":repeats,"receipt_time_pair_diagnostic":diagnostic,
    "persistent_fusion_running":False,"next_pending":["return-after-lift settled segment; lift itself already verified",
    "mounting/lever arm and landing plane measurement","source acquisition time and flight delay calibration",
    "previous repeated pose root cause","physical tracking-loss and recovery","flight bridge, operator takeover and complete preflight"]}
(ROOT/"final_summary.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
translation=validation.get("translation")
state="通过" if passed else "未通过"
if translation:
    numeric=f'动捕向右{translation["input_right_m"]*100:.3f}cm，PX4向右{translation["px4_right_m"]*100:.3f}cm；水平位移向量差{translation["horizontal_displacement_difference_m"]*1000:.3f}mm，停稳端点差{translation["horizontal_endpoint_difference_m"]*1000:.3f}mm，航向差{translation["yaw_difference_deg"]:.3f}°。伴随沿基准机头方向偏移{translation["input_cross_m"]*100:.3f}cm。'
else:
    numeric=f'未保存合格右移稳定段。错误：{validation.get("error")}'
addition=f"""
## 2026-10-26 向右平移真实融合（最新）

用户指定验证向右移动。本轮{run.name}，地面动作验收**{state}**。

首轮062808Z在基准前因验收脚本误将转发计数字段forwarded写为righted而中止，未发出移动提示。该误改已修正，五组方向投影检查通过；原失败记录与独立恢复审计保留在retry_correction.json，不改写为成功。该首轮只修验收工具。第二轮063332Z完成baseline后，因fixed reference parameter readback stale/future锁存停止，未保存合格right_move；完整日志与独立1127参数恢复审计保留。本次另修项目桥接mocap_odometry_node.py的参数证明校验顺序：读完文件后再获取单调时钟，避免并发刷新时将新证明误判为未来。旧顺序竞态已离线复现，7项检查通过，实际未来/超时/身份/磁融合冲突及锁存保护保留。现场第二轮缺少故障瞬间证明时间，不能断言唯一根因已证实；本轮成功不抹去此前失败。2秒证明时限、源时间与新鲜度门禁、冻结保护均未取消。修复在本地源码及本轮独立测试目录，正式current未切换。

{numeric}

按基准时机头方向的右侧计算正向位移，保持原先的位置/姿态新鲜度、未解锁、XYZ/yaw融合和无磁融合门禁，保留冻结保护及完整参数回退。输入位置/航向稳定后额外保持8秒再保存4秒均值。物理距离未经独立测量；毫米差值只表示输入与估计在停稳段的一致性，不是绝对定位精度或运动全过程/时延验收。基准后观测{len(flags)}个融合状态样本，共同融合是否全部有效：{valid_flags}；桥接锁存故障数：{len(faults)}；原始完全相同位姿持续100ms以上的片段数：{len(repeats)}。

本轮参数EV0/HGT1/MAG0/SYNCT1已恢复且重启核验；{Path(audit["post"]).name}/comparison.json独立1127参数审计通过。额外差异仅气压计自动校准候选CAL_BARO0_OFF，详情{json.dumps(audit["parameter_differences"],ensure_ascii=False)}。原正式部署、Jetson boot、Agent/relay/网络辅助服务不变，任务无残留，无持续融合，无执行器动作或网络修改。

用户已明确：**抬高本身已有成功验证**（约70.08cm vs69.97cm），无需重新记作未完成；只剩放回后的稳定段/全程尚未验收。已有静态、左转、右转、前移的成功记录保留。向右移动本轮状态见本段，不再混同右转。

下一步仍需处理源时间/时延、安装偏移、午前冻结根因、物理跟踪失效与恢复、飞行桥接及接管/起飞条件；地面动作通过不放行飞行。

证据：report/mocap_right_translation_20261026/final_summary.json、{run.name}/validation.json和samples.jsonl、基准/动作稳定段与参数前后快照。没有提交、推送或新部署。
"""
doc=PROJECT/"docs/A机动捕PX4验收进度与继续入口_20261026.md"
content=doc.read_text(encoding="utf-8")
if passed:
    content=content.replace("向右平移、抬高放回全程及飞行条件仍未验收完毕。","向右平移本轮也已通过；放回后的高度稳定段及飞行条件仍未验收完毕（抬高本身已验证）。")
    content=content.replace("| 前/右平移真实融合 | 前移停稳段通过；向右平移尚待 | 前移停稳数据见末尾，运动时延/动态精度仍待 |","| 前/右平移真实融合 | 前移与向右平移停稳段均通过 | 本日新增结果见末尾，运动时延/动态精度仍待 |")
    content=content.replace("| 真实融合向右平移 | 尚待 | 右转不等于右移 |","| 真实融合向右平移 | 后续本轮通过 | 详细数据见向右平移最新记录 |")
    content=content.replace("1. 右转单动作本轮通过；继续做单动作向右平移，核对符号、稳定段和完整恢复。","1. 左转、右转、前移及右移均已有单动作成功记录，继续核对动态偏差与时延。")
    content=content.replace("2. 前移停稳段本轮通过；继续向右平移和抬高—放回的真实融合，每轮新基准和独立参数回退。","2. 抬高本身已验证；需补放回稳定段/全程验收，保留新基准和独立参数回退。")
doc.write_text(content+addition,encoding="utf-8",newline="\n")
summary=PROJECT/"2026-10-26_A机验收进度.md"
with summary.open("a",encoding="utf-8",newline="\n") as stream:
    stream.write(f"\n## 最新：向右平移{state}\n\n{numeric}\n\n抬高本身已验证，尚需补放回稳定段；其余飞行准备与时延/失效验证仍待。参数已恢复、独立审计通过，当前无持续融合。\n")
with (PROJECT/"MEMORY.md").open("a",encoding="utf-8",newline="\n") as stream:
    stream.write("\n\n"+addition)
print(json.dumps({"ground_test_passed":passed,"translation":translation,"fusion_flags_samples":len(flags),
    "audit_passed":audit["passed"],"exact_raw_pose_repeats_ge_100ms":repeats,"receipt_time_pair_diagnostic":diagnostic},ensure_ascii=False))
