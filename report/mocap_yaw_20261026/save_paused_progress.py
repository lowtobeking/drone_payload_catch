"""Save progress and a checked local snapshot; no SSH or drone actions."""
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import zipfile

WORK=Path(__file__).resolve().parent
ROOT=WORK.parents[1]
RUN=sorted(WORK.glob("lift_reference_20*"))[-1]
POST=sorted(WORK.glob("after_20*"))[-1]
CLOSE=ROOT/"report/session_pause_20261026"
CLOSE.mkdir(exist_ok=True)
validation=json.loads((RUN/"validation.json").read_text(encoding="utf-8"))
audit=json.loads((POST/"comparison.json").read_text(encoding="utf-8"))
lift=json.loads((ROOT/"report/mocap_auto_lift_20261026/final_summary.json").read_text(encoding="utf-8"))
static=json.loads((ROOT/"report/mocap_continue_20261026/final_summary.json").read_text(encoding="utf-8"))
with (RUN/"samples.jsonl").open(encoding="utf-8") as handle:
    rows=[json.loads(line) for line in handle]
raw=[r for r in rows if r["kind"]=="raw"]
inputs=[r for r in rows if r["kind"]=="input"]
local=[r for r in rows if r["kind"]=="local"]
def yaw(q):
    w,x,y,z=q
    return math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
def wrap(deg):
    return math.degrees(math.atan2(math.sin(math.radians(deg)),math.cos(math.radians(deg))))
b=validation["phases"]["baseline"]
last=inputs[-1]
nearest=min(local,key=lambda r:abs(r["monotonic"]-last["monotonic"]))
endpoint=dict(input_yaw_deg=yaw(last["data"]["q"]),px4_yaw_deg=math.degrees(nearest["data"]["heading"]),
    match_age_s=abs(nearest["monotonic"]-last["monotonic"]))
endpoint["input_change_deg"]=wrap(endpoint["input_yaw_deg"]-b["input_yaw_deg"])
endpoint["px4_change_deg"]=wrap(endpoint["px4_yaw_deg"]-b["px4_yaw_deg"])
endpoint["note"]="Last accepted input matched to nearest PX4 sample; diagnostic endpoint only, no accepted stable left_turn snapshot."
best=None
start=0
for i in range(1,len(raw)+1):
    same=(i<len(raw) and raw[i]["data"]["position"]==raw[start]["data"]["position"]
          and raw[i]["data"]["quaternion_xyzw"]==raw[start]["data"]["quaternion_xyzw"])
    if same:continue
    group=raw[start:i]
    span=(group[-1]["data"]["receive_monotonic_us"]-group[0]["data"]["receive_monotonic_us"])/1e6
    item=dict(count=len(group),receive_span_s=span,
        source_stamp_advanced=group[-1]["data"]["server_stamp_us"]>group[0]["data"]["server_stamp_us"])
    if best is None or span>best["receive_span_s"]:best=item
    start=i
faults=[r["data"]["latched_fault"] for r in rows if r["kind"]=="bridge" and r["data"].get("latched_fault")]
summary=dict(manual_yaw_passed=False,raw_verdict=validation["passed"],raw_error=validation.get("error"),
    detected_faults=sorted(set(faults)),endpoint_diagnostic=endpoint,longest_exact_pose_repeat=best,
    operator_report="Reflective markers visible; operator did not observe tracking loss.",
    cause="Undetermined: do not attribute to occlusion or relax repeated-pose guard without evidence.",
    post_audit_passed=audit["passed"],restored_parameters={"EKF2_EV_CTRL":0,"EKF2_HGT_REF":1,"EKF2_MAG_TYPE":0,"UXRCE_DDS_SYNCT":1},
    parameter_differences=audit["parameter_differences"],remaining_task_processes=audit["remaining_task_processes"],
    current_fusion_running=False,flight_approval=False,run=RUN.name,post=POST.name)
(WORK/"final_summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
stamp=datetime.now().astimezone().isoformat()
doc=f"""# A机动捕与PX4验收进度及暂停入口（2026-10-26）

更新时间：{stamp}。用户要求先验证到这里、保存进度后暂停去吃饭。当前没有采集或动捕融合测试运行；用户明确继续前不再连接或操作无人机。

## 验收结论

**静态XYZ与航向融合已成功；动态验收未完成，尚不能据此放行飞行。**

| 验收项 | 当前状态 | 已有证据与限制 |
| --- | --- | --- |
| Wi-Fi SSH | 本机已验证可用 | 电脑DHCP、自动DNS；当前A=.5，电脑=.4；按电脑网卡MAC维护回程路由，详见网络指南 |
| planeA原始数据 | 已验证 | FZMotion→VRPN→Jetson，约120Hz；先前8秒960帧、无无效帧 |
| 固定坐标变换 | 已实现、方向已有人工预览验证 | [Y,X,-Z]及完整姿态变换；室内固定参考，不宣称地理北向 |
| PX4融合X/Y/Z与航向 | 静态通过 | 本日连续约{static['observed_static_span_s']:.2f}秒，共同融合及位置/速度/可控航向状态有效，磁融合关闭 |
| 实际抬高 | 片段已记录 | 动捕上升{lift['up_input_m']:.6f}m、PX4上升{lift['up_px4_m']:.6f}m，位移差{lift['displacement_difference_m']*1000:.3f}mm；仅稳定片段接近，不是绝对精度或全程动态通过 |
| 抬高后放回 | 未完成验收 | 用户已放回，但返回稳定段未在96秒窗口内保存，整轮passed=false |
| 水平左转 | 有转向响应证据，未通过验收 | 动捕/PX4端点诊断见下；没有保存合格left_turn稳定段，期间重复位姿检测触发停发 |
| 本轮真实融合前/右平移 | 尚未完成 | 早期预览方向验证不能代替当前真实PX4融合动态验收 |
| 重复数据保护 | 已观察到触发 | 并不证明动捕一定丢失跟踪，需排查是否冻结/保持值或保护误判 |
| 参数备份与恢复 | 本轮核验通过 | 完整1127参数备份与独立读回；测试参数已恢复，原有服务、Jetson启动及部署版本保持 |
| 飞行所需标定与保护 | 尚待完成 | 详见后续列表，不允许直接拿有界地面工具飞行 |

## 本次转向具体结果

用户确认保持水平左转停稳。最后一个被接受的里程计输入与最近PX4样本相差{endpoint['match_age_s']:.6f}秒，诊断值：

- 动捕输入航向变化：{endpoint['input_change_deg']:.3f}°。
- PX4航向变化：{endpoint['px4_change_deg']:.3f}°。
- 左转在本室内NED导航约定中是负航向变化。该端点只能说明当时响应，不代替完整稳定段、连续跟随或时延验收。

桥接故障：{", ".join(sorted(set(faults)))}。原始程序error={validation.get('error')}、passed=false。
原始数据中最长完全相同的位置与四元数组：{best['count']}帧、接收时间跨度{best['receive_span_s']:.6f}秒；这段源时间戳是否继续增长：{best['source_stamp_advanced']}。

用户现场反馈：**反光点可见，没看到跟踪丢失。**因此不能将原因定为遮挡、动捕丢失或软件误报。保持现有冻结保护，不为通过验收放宽阈值；后续应把FZMotion跟踪状态、原始VRPN位姿/时间戳和桥接判定对齐分析。

没有保存left_turn.json/movement_result.json，不能补造记录，也不能把手动转过来或某个端点相符写为整轮成功。

## 当前设备状态与恢复

独立恢复审计：{POST.name}/comparison.json，passed={audit['passed']}，完整读回1127项。融合相关恢复值：

| 参数 | 当前恢复值 |
| --- | ---: |
| EKF2_EV_CTRL | 0 |
| EKF2_HGT_REF | 1 |
| EKF2_MAG_TYPE | 0 |
| UXRCE_DDS_SYNCT | 1 |

唯一额外参数差异：
{json.dumps(audit['parameter_differences'],ensure_ascii=False)}

CAL_BARO0_OFF属于此前已分析的PX4启动自动相对气压计校准候选，保留原始差异，未强行覆写；不是全部参数字节相同。测试配置和恢复按已核验流程各重启一次飞控，Jetson boot保持{audit['jetson_boot']}，未重启Jetson。

- 本轮source、bridge、guard均退出；独立审计remaining_task_processes为空。
- 原Agent1155、relay1156仍active；正式current仍b4453dd…，未切换部署。
- 无常驻动捕融合。网络修复的payload-catch-wifi-peer.service保留运行，属于此前用户要求的Wi-Fi连接功能。
- 没有解锁、模式切换、飞行设定点、电机或PWM动作，没有修改网络配置和原通信服务。
- 没有保留常驻SSH会话、后台监视或自动继续任务。

## 还缺哪些

1. 排清转向时完全重复位姿的原因，区分真实跟踪失效、服务器保持输出与静止量化/滤波导致的误判，保留原保护。
2. 重新做水平左转/右转，保存同一连续运行中的基准、稳定转向段、融合状态和残差，核对符号、幅度、跳变与恢复。
3. 完成前/右移动和抬高—放回的真实融合验收；每轮只做清晰动作，避免交互等待耗尽采集窗口。
4. 测量刚体原点到IMU的偏移、安装姿态及起降平面。现有z为实验室刚体原点，不是离地高度。
5. 排查源时间仍快约12小时的问题，标定真实采集到PX4的延迟；当前receive_time_ground仅用于显式地面试验，不能写为已解决源时间。
6. 物理遮挡、断流、恢复、重启及人工接管验收；然后再做独立飞行常驻桥接和失效策略。
7. 完成工程原有起飞自检、电池/电源、RC/QGC与场地条件验收，再评估飞行。不得因本次静态融合通过而绕过。

## 下次从哪里继续

先阅读MEMORY.md和本文件。确认用户已回来并要求继续，无桨/未解锁/水平放稳、planeA跟踪正常，再核对当次身份与IP；不要直接复用旧基准、旧启动标识或参数证明。

优先离线分析本次重复位姿：report/mocap_yaw_20261026/{RUN.name}/samples.jsonl、bridge.log、source/native.stderr.log。需要现场配合时再启动有界地面工具，保留完整参数备份、独立恢复守护和原未解锁/时序门禁。

## 证据入口

- 静态93秒：report/mocap_continue_20261026/final_summary.json。
- 自动抬高片段：report/mocap_auto_lift_20261026/final_summary.json，及docs/A机自动抬高片段验证_20261026.md。
- 本次转向：report/mocap_yaw_20261026/final_summary.json、{RUN.name}/。
- 最后恢复审计：report/mocap_yaw_20261026/{POST.name}/comparison.json。
- 原始大日志与采样已在工程本地保存；不以Git忽略或小型快照代替原始证据。
- 本轮保存不等于Git提交或远端推送；没有执行提交、推送或新部署。
"""
guide=ROOT/"docs/A机动捕PX4验收进度与继续入口_20261026.md"
guide.write_text(doc,encoding="utf-8")
(ROOT/"2026-10-26_A机验收进度.md").write_text(
    "# 2026-10-26 A机验收进度\n\n用户要求暂停，成果已保存。静态XYZ与航向融合通过；抬高有片段，放回未完成；左转有响应但重复位姿保护触发，动态验收未通过。融合参数已恢复，当前无常驻融合和测试任务。\n\n"
    "[详细验收表、恢复状态与继续入口](docs/A机动捕PX4验收进度与继续入口_20261026.md)\n",encoding="utf-8")
memory=ROOT/"MEMORY.md"
with memory.open("a",encoding="utf-8") as handle:
    handle.write(f"""

## 2026-10-26 用户暂停去吃饭，保存验收进度（最新）

用户明确先验收到这里并保存进度，去吃饭；后续明确要求继续前不再连接/操作A，也不自动启动测试。静态XYZ/yaw93.04秒通过；自动抬高片段约70.08cm vs69.97cm，返回未采到，不是整轮通过。左转run={RUN.name}自动检测exactly repeated pose stream，桥接停发、原passed=false，没有合格left_turn稳定段。用户确认反光点可见且未观察到跟踪丢失，原因未定，不写成已证遮挡，不放宽阈值。端点诊断input变化{endpoint['input_change_deg']:.3f}°/PX4{endpoint['px4_change_deg']:.3f}°，仅点态响应证据。
{POST.name}独立1127参数恢复审计通过，EV0/HGT1/MAG0/SYNCT1；仅CAL_BARO0_OFF自动校准候选差异，原始详情保留。Jetsonboot/current b4453dd/原Agent1155和relay1156不变；采集/源/桥接/守护无残留，当前没有持续融合。保留此前用户要求的Wi-Fi回程辅助服务，不留监视或SSH保活。
集中进度docs/A机动捕PX4验收进度与继续入口_20261026.md，根目录2026-10-26_A机验收进度.md；转向汇总report/mocap_yaw_20261026/final_summary.json。下一步优先离线排查重复位姿及源时间语义，再经用户继续后做单动作左/右转与真实融合平移/抬高放回；精确安装偏移、源延迟、物理遮挡恢复与飞行桥接仍待。未提交/推送/新部署。
""")
# A compact checkpoint: full raw logs remain in their original local run directories.
selected=[guide,ROOT/"2026-10-26_A机验收进度.md",memory,
          WORK/"final_summary.json",RUN/"manifest.json",RUN/"validation.json",
          RUN/"baseline.json",RUN/"parameter_lease_result.json",RUN/"process_cleanup.json",
          POST/"comparison.json",ROOT/"report/mocap_auto_lift_20261026/final_summary.json",
          ROOT/"report/mocap_continue_20261026/final_summary.json"]
for folder in ("payload_catch","config"):
    selected.extend(p for p in (ROOT/folder).iterdir() if p.is_file() and p.suffix in (".py",".yaml"))
snapshot=CLOSE/("progress_checkpoint_"+datetime.now().strftime("%Y%m%dT%H%M%S")+".zip")
checks={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in selected}
with zipfile.ZipFile(snapshot,"x",compression=zipfile.ZIP_DEFLATED) as archive:
    for p in selected:archive.write(p,str(p.relative_to(ROOT)))
with zipfile.ZipFile(snapshot) as archive:
    if archive.testzip():raise RuntimeError("Archive CRC failed")
    for name,expected in checks.items():
        if hashlib.sha256(archive.read(name)).hexdigest()!=expected:raise RuntimeError("Archive mismatch "+name)
manifest=dict(path=str(snapshot.relative_to(ROOT)),sha256=hashlib.sha256(snapshot.read_bytes()).hexdigest(),
    entries=checks,raw_logs_remain_in_original_directories=True,verified=True)
(CLOSE/"checkpoint_manifest.json").write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
(CLOSE/"pause_state.json").write_text(json.dumps(dict(paused_by_user=True,progress_document=str(guide.relative_to(ROOT)),
    no_automatic_continuation=True,last_independent_audit=str((POST/"comparison.json").relative_to(ROOT)),
    current_fusion_running=False,remaining_task_processes=[],network_helper_retained=True),ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
print(json.dumps(dict(progress_document=str(guide),snapshot=str(snapshot),snapshot_verified=True,
    endpoint_diagnostic=endpoint,repeated_pose_evidence=best,restoration_verified=audit["passed"],
    next_action="Wait for explicit user continuation; no more drone operations"),ensure_ascii=False,indent=2))
