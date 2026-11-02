"""Save grounded acceptance conclusions and recovery evidence to the project."""
from datetime import datetime
import importlib.util
import json
import math
from pathlib import Path
import statistics

WORK=Path(__file__).resolve().parent
PROJECT=WORK.parents[1]
def read(path):
    return json.loads(path.read_text(encoding="utf-8"))
runs=[]
for folder in sorted(WORK.glob("lift_reference_20*")):
    audits=[read(p) for p in WORK.glob("after_*/comparison.json") if read(p)["test"]==str(folder)]
    assert len(audits)==1,(folder,audits)
    runs.append({"run":folder.name,"validation":read(folder/"validation.json"),"audit":audits[0]})
latest=runs[-1]
assert latest["validation"]["passed"] and latest["audit"]["passed"]
assert latest["audit"]["parameter_count"]==1127 and not latest["audit"]["remaining_task_processes"]
spec=importlib.util.spec_from_file_location("raw_analysis",WORK/"analyze_raw.py")
raw_module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(raw_module)
raw_runs=[]
for folder in sorted(WORK.glob("raw_turn_20*")):
    raw_module.analyze(folder)
    result=read(folder/"raw_analysis.json")
    raw_runs.append({"run":folder.name,"frames":result["frames"],"duration_s":result["duration_s"],"max_receive_gap_s":result["max_receive_gap_s"],"identical_groups_ge_100ms":len(result["identical_groups_ge_100ms"]),"identical_groups_gt_500ms":result["identical_groups_gt_500ms"]})
samples=[json.loads(line) for line in (WORK/latest["run"]/"samples.jsonl").read_text(encoding="utf-8").splitlines()]
baseline=latest["validation"]["phases"]["baseline"]["monotonic"]
flags=[r["data"] for r in samples if r["kind"]=="flags" and r["monotonic"]>=baseline]
bridges=[r["data"] for r in samples if r["kind"]=="bridge" and r["monotonic"]>=baseline]
assert flags and all(all(r[k] for k in ("cs_ev_pos","cs_ev_hgt","cs_ev_yaw")) and not r["cs_mag_hdg"] and not r["cs_mag_3d"] for r in flags)
assert bridges and not any(r.get("latched_fault") for r in bridges)
snapshot=latest["validation"]["phases"]["left_turn"]
xy_error=math.hypot(*(snapshot["px4_xyz"][i]-snapshot["input_xyz"][i] for i in (0,1)))
result={"updated":datetime.now().astimezone().isoformat(),"scope":"single left-turn disarmed ground acceptance","yaw_ground_test_passed":True,"flight_approval":False,"raw_runs":raw_runs,"fusion_runs":runs,"latest_run":latest["run"],"all_observed_flags_after_baseline_ev_xyz_yaw_no_mag":True,"flags_samples_after_baseline":len(flags),"no_latched_bridge_fault":True,"turned_plateau_horizontal_disagreement_m":xy_error,"yaw":latest["validation"]["yaw"],"current_parameters":{"EKF2_EV_CTRL":0,"EKF2_HGT_REF":1,"EKF2_MAG_TYPE":0,"UXRCE_DDS_SYNCT":1},"persistent_fusion_running":False,"remaining_task_processes":[],"pending":["right-turn true fusion","forward/right translation true fusion","lift-return complete acceptance","root cause of previous 83 exactly repeated raw frames","rigid-body-to-IMU mounting and lever-arm measurement","server acquisition timestamps and flight delay calibration","physical tracking-loss/recovery","flight bridge/failsafe/operator takeover and complete preflight"]}
(WORK/"final_summary.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
yaw=result["yaw"]
audit=latest["audit"]
doc=f"""# A机动捕与PX4验收进度（2026-10-26）

更新时间：{result["updated"]}。用户午后明确恢复验证，本次续验已结束并完成恢复审计。

## 结论

**静态XYZ与航向融合已通过；本次单动作水平左转地面验收通过。右转真实融合、水平平移、抬高放回全程及飞行条件仍未验收完毕。当前没有持续融合运行。**

| 验收项 | 当前状态 | 证据与限制 |
| --- | --- | --- |
| Wi-Fi SSH | 已恢复，换电重启后也可用 | 电脑自动地址.4，A自动地址.5；网络辅助服务保留，地址将来可能变化 |
| 原始VRPN实时转向 | 左/右方向均已收到 | 左转约89°、右转约55°；三轮各约120秒、约120Hz，无100ms以上完全重复姿态 |
| 固定坐标转换 | 已实现并做过方向验证 | [Y,X,-Z]和完整姿态转换；室内参考，不宣称地理北向 |
| XYZ+航向静态融合 | 已通过 | 午前93秒记录及午后基准；融合启用XYZ和yaw、磁融合关闭 |
| 水平左转真实融合 | **本次单动作通过** | 动捕变化{yaw["input_change_deg"]:.6f}°、PX4变化{yaw["px4_change_deg"]:.6f}°，差{yaw["displacement_difference_deg"]:.6f}°；已保存基准和4秒转向稳定段 |
| 右转真实融合 | 尚待 | 本次右转只采原始VRPN，没有写入PX4 |
| 前/右平移真实融合 | 尚待 | 本次手持左转带来的位置变化不作为专门平移验收 |
| 抬高 | 先前有成功片段 | 动捕上升0.700788m、PX4上升0.699714m，不是独立物理距离精度标定 |
| 抬高后放回 | 尚未完整验收 | 先前returned稳定段未采到，原整轮失败结论保留 |
| 午前重复原始位姿 | 原因未定，本次未复现 | 83帧/0.678秒重复记录保留，没有放宽保护 |
| 参数恢复及任务清理 | 本轮独立审计通过 | 完整1127项读回；服务和部署链接不变，本轮任务无残留 |
| 飞行准备 | 尚未完成 | 安装偏移、时间/延迟、跟踪失效、接管和飞行桥接等仍待 |

## 午后具体记录

- 原始记录：report/mocap_resume_20261026/raw_turn_20261026T054438Z、raw_turn_20261026T054825Z、raw_turn_20261026T055732Z。
- 第一次融合重试lift_reference_20261026T055101Z：原始姿态在动作窗口内几乎不变，没有采到左转；passed=false、manual left-turn window expired。用户反馈FZMotion模型跟随转动，原因未确定，不归咎用户、不补造动作或结论。静态XYZ/yaw融合标志正常，无重复保护触发，恢复审计通过。
- 第二次融合重试{latest["run"]}：baseline和left_turn均保存，validation.json passed=true；基准之后{len(flags)}个观测到的EKF状态样本均显示EV位置、高度、航向融合，磁力计融合关闭。未出现锁存桥接故障，原冻结门禁保留。
- 左转稳定段输入航向179.626003°、PX4航向179.841152°；使用环绕差计算跨±180°的变化。
- 左转稳定段输入与PX4的水平位置均值仍相差约{xy_error*100:.2f}厘米，手持有平移及滤波动态影响，原因尚未专门验证；不能由本次航向成功推导水平动态精度合格。
- 本次角度是动捕输入与PX4相对变化的一致性，操作者实际角度没有独立标定；不是绝对方向精度或飞行时延验收。

## 恢复后的设备状态

独立审计：report/mocap_resume_20261026/{Path(audit["post"]).name}/comparison.json，passed=true，完整1127项。

| 参数 | 已恢复值 |
| --- | ---: |
| EKF2_EV_CTRL | 0 |
| EKF2_HGT_REF | 1 |
| EKF2_MAG_TYPE | 0 |
| UXRCE_DDS_SYNCT | 1 |

测试临时值为EV11/HGT3/MAG5/SYNCT1；每轮配置与恢复各重启一次飞控，Jetson未由本任务重启。午后用户自行重启后的Jetson boot为dc981513-5789-4a7c-ba91-57367486bfa9。Agent1019、relay1024和网络辅助2770始终保持；正式current仍b4453dd…，未切换部署。

最新参数额外差异仅CAL_BARO0_OFF（自动气压计校准候选），原始差异：{json.dumps(audit["parameter_differences"],ensure_ascii=False)}。没有强行重写此项，不宣称参数全字节相同。

source、bridge、guard正常退出，独立审计本任务进程无残留；原始限时采样已结束。没有解锁、飞行设定点、电机或PWM动作，没有修改网络或主机时钟。SSH命令逐条退出，没有常驻保活会话。网络辅助服务为用户要求的保留功能，不是待清理测试进程。

## 下一步

1. 做单动作右转的真实PX4融合，核对符号、稳定段、跳变和完整恢复。
2. 分别做前/右平移和抬高—放回的真实融合；每轮新基准，保持独立参数回退，避免把原始预览当融合验收。
3. 排查午前重复位姿；本次未复现不能记为已修复，仍保留冻结保护。
4. 测量刚体原点到IMU的偏移、安装姿态及起降平面；当前z是实验室刚体原点坐标，不是离地高度。
5. 动捕源时间仍比Jetson约快12小时；receive_time_ground仅是显式地面模式，采集时刻和真实链路时延未获验证。
6. 完成物理遮挡/断流/恢复及接管测试，设计常驻飞行桥接和失效策略，再完成工程起飞自检及飞行场地/操控条件验收。有界地面工具不能直接用于飞行。

## 继续入口和证据

先读MEMORY.md和本文件；确认当次无桨/未解锁、planeA跟踪状态，再核对实时IP、设备身份、boot及参数。不复用旧的基准或时序证明。

- 本次全部结果：report/mocap_resume_20261026/final_summary.json。
- 成功左转：report/mocap_resume_20261026/{latest["run"]}/validation.json、samples.jsonl、baseline.json、left_turn.json。
- 原始/输入对照：report/mocap_resume_20261026/motion_analysis.json。
- 午前静态：report/mocap_continue_20261026/final_summary.json。
- 午前抬高片段：report/mocap_auto_lift_20261026/final_summary.json。
- 午前冻结失败：report/mocap_yaw_20261026/final_summary.json及原始samples.jsonl；83帧重复的离线细查在offline_repeat_analysis.json。
- 没有提交、推送或新部署；大日志保存在工程本地，不以Git忽略代替归档。
"""
target=PROJECT/"docs/A机动捕PX4验收进度与继续入口_20261026.md"
target.write_text(doc,encoding="utf-8",newline="\n")
(PROJECT/"2026-10-26_A机验收进度.md").write_text("""# 2026-10-26 A机验收进度

午后续验：静态XYZ与航向融合已通过，**水平左转单动作地面融合验收本次通过**：动捕61.93°、PX4为61.72°，变化差约0.21°。参数和任务清理已独立复查通过，当前没有持续融合。

右转真实融合、水平平移、抬高放回全程、午前原始冻结根因及飞行准备仍待；本次不代表可飞。

[详细验收表、恢复状态与继续入口](docs/A机动捕PX4验收进度与继续入口_20261026.md)
""",encoding="utf-8",newline="\n")
with (PROJECT/"MEMORY.md").open("a",encoding="utf-8",newline="\n") as f:
    f.write(f"""

## 2026-10-26 午后水平左转单动作融合通过（最新）

用户午后明确继续，确认无桨/未解锁/水平放稳并可配合；A用户重启后boot=dc981513-5789-4a7c-ba91-57367486bfa9，Wi-Fi .4→.5恢复，Agent1019/relay1024/网络辅助2770。仅只读VRPN三轮各约120秒，静态、左转约89°和右转约55°均保存，无100ms以上完全重复位姿；午前83帧重复仍无根因，不放宽门禁。

融合重试055101Z在窗口内原始姿态变化不足0.11°，没有合格转向，原passed=false保留；用户称FZMotion模型跟随，原因未确定。再次实时原始右转后复验{latest["run"]}，baseline/left_turn已保存，validation passed=true，动捕变化{yaw["input_change_deg"]:.6f}°、PX4变化{yaw["px4_change_deg"]:.6f}°、差{yaw["displacement_difference_deg"]:.6f}°。基准后{len(flags)}个flags样本均EV XYZ/yaw真、mag融合假，无锁存故障。只通过单次左转地面动作，物理角度未独立测量；转向稳定段水平位置均值差约{xy_error*100:.2f}cm仍待专门平移/动态核对，不宣称全部动态精度通过。

EV0/HGT1/MAG0/SYNCT1恢复重启核验，{Path(audit["post"]).name}独立1127项审计passed=true；仅CAL_BARO0_OFF自动校准候选差异保留。Jetson boot/current/三服务PID保持，任务进程无残留，不常驻融合或SSH保活；没有解锁/执行器/飞行/网络或时钟修改。证据report/mocap_resume_20261026/final_summary.json，更新根目录和docs验收总表。右转真实融合、前/右平移、抬高放回全程、时间/安装偏移/失效保护及飞行桥接仍待；本次未提交/推送/新部署。
""")
print(json.dumps({"summary":str(WORK/"final_summary.json"),"yaw":yaw,"audit_passed":audit["passed"],"flags_samples":len(flags),"turned_xy_disagreement_m":xy_error},ensure_ascii=False))
