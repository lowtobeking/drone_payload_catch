"""Write verified outcome and preserve failed attempts without claiming flight approval."""
from pathlib import Path
import json
import yaml
ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parents[1]
test=sorted(ROOT.glob("fixed_reference_20*"))[-1]
post=sorted(ROOT.glob("after_20*"))[-1]
v=json.loads((test/"validation.json").read_text(encoding="utf-8"))
a=json.loads((test/"analysis.json").read_text(encoding="utf-8"))
r=json.loads((post/"comparison.json").read_text(encoding="utf-8"))
assert v["passed"] and v["static_passed"] and r["passed"]
lease=v["lease"]
assert lease["restored"] and lease["restore_restart_verified"]
summary=dict(passed=True,scope="disarmed_static_ground_only",flight_approval=False,
    run=test.name,post_audit=post.name,static_command_window_s=25,
    ev_active_sample_span_s=a["ev_active_span_s"],local_valid_samples=a["local_during_ev"],
    all_disarmed=a["all_disarmed"],all_landed=a["all_land_true"],
    magnetic_heading_and_3d_fusion_active=False,
    software_source_loss={k:v["software_source_loss"][k] for k in
        ("no_input_after_400ms","bridge_latched","physical_tracking_loss_tested")},
    forwarded=v["counts"]["input"],restored_parameters={"EKF2_EV_CTRL":0,"EKF2_MAG_TYPE":0,"UXRCE_DDS_SYNCT":1},
    restore_restart_verified=True,jetson_boot_unchanged=r["jetson_boot_unchanged"],
    remaining_task_processes=r["remaining_task_processes"],parameter_differences=r["parameter_differences"],
    continuous_manual_motion_validated=False,permanent_flight_bridge=False)
(ROOT/"final_summary.json").write_text(json.dumps(summary,indent=2),encoding="utf-8")
text=f"""# A机固定动捕航向实机地面验证（2026-10-23）

本轮已完成：在真实A机上临时关闭PX4磁力计融合，使用固定实验室参考的动捕水平位置和航向。**静态与软件断流测试通过，尚未放行飞行。** 测试后已恢复原参数并核验恢复重启，当前没有常驻动捕融合。

## 实际结果

| 项目 | 结果 |
| --- | --- |
| 飞控 | PX4 1.16.0；SYSID1/component1；UID3544984969635313204 |
| 设备 | Jetson序列号1612425621719；本轮Jetson未重启 |
| 动捕 | FZMotion3.0.0.9；192.168.1.111:3883；planeA/sensor0 |
| 临时配置 | EKF2_MAG_TYPE=5、EKF2_EV_CTRL=9；UXRCE_DDS_SYNCT保持1 |
| 静态记录 | 主动采集25秒；EV位置/航向同时有效样本跨度{a["ev_active_span_s"]:.3f}秒 |
| EKF状态 | cs_ev_pos/cs_ev_yaw/cs_yaw_align/cs_tilt_align=true；cs_mag_hdg/cs_mag_3d=false |
| 定位状态 | 融合窗口内{a["local_during_ev"]["total"]}个位置样本的xy_valid/v_xy_valid/heading_good_for_control全部true，dead_reckoning全部false |
| 真实输入 | {v["counts"]["input"]}帧VehicleOdometry送入PX4 |
| 软件断流 | 停止本轮VRPN源后，400ms后无新输入；桥接锁存停发；不是物理遮挡测试 |
| 恢复 | EV_CTRL=0、MAG_TYPE=0、SYNCT=1；恢复后再次重启飞控并读回验证 |
| 清理 | 本轮机载测试进程0残留；原Agent/relay仍active，正式current未切换 |

参数全量1127项均重新读取。唯一额外变化是CAL_BARO0_OFF：-48.375→-38.1484375，类型/索引不变。PX4 VehicleAirData::UpdateRelativeCalibrations在启动后会自动调整并保存次级气压计相对偏移，源码与现象相符；本工具没有写该参数，也没有强行覆写它。**不是1127项完全一致**，完整差异保留在恢复核对文件。

## 固定参考如何实现

位置和完整姿态同时按既有人工验证矩阵变换，local=[worldY,worldX,-worldZ]，共享原点world=[0,0,0]；不随飞机当前磁航向重新对零。固定室内导航约定为X参考、Y向右、Z向下，PX4输入使用POSE_FRAME_NED的导航编码，**没有宣称实验室X为地理北向**。新配置显式声明该语义，并要求磁航向/三轴及GNSS位置/速度/航向均未融合，否则停发。

原FRD路径在PX4 1.16启动EVyaw时会清yaw_align，造成heading_good_for_control=false。本地直接链接同版PX4 EKF库验证7个场景后，才实机验证固定导航编码；不是仅为让检查变绿修改frame数字。

代码：payload_catch/mocap_odometry.py、mocap_odometry_node.py；独立地面配置config/mocap_a.fixed_ground.yaml。默认严格配置不放宽；mounting_confirmed、lever_arm_measured、server_timing_verified、tracking_loss_verified仍false。实际使用原生回调接收时间，保留DDS时间同步，仍只限地面。

## 调试过程中发现并修复

前四轮失败证据均保留，没有用最终通过覆盖：

1. 实机飞控重启后约82秒才恢复遥测。最初50/65秒时限不足；后来增加有界恢复时间、只读请求及新启动时间证明。曾有自动回退未完成，已通过独立连接恢复并全参数核对。
2. 刚启动后的再次重启，新的可读启动时间约81秒，可能高于重启前值；改为同时核对旧连续运行时间与重启实际经过时间。早期恢复重试导致额外飞控重启，记录保留。
3. 跨重启的本轮旧ROS订阅未恢复；只重建本任务节点/订阅，未重启共享Agent或relay。最终重新读到新鲜未解锁/着陆状态。
4. 第四轮已实证9.10秒动捕航向有效且无磁融合，但串行参数证明刷新过期而停发；此前临时将次生landed过期称作首因，日志分析后已更正。改为三项批量读回，1.2秒内完成、证明仍2秒过期，没有放宽安全阈值。最终本轮通过。
5. 清理顺序改为先停本轮源/桥接，再回退参数及重启，避免恢复等待期间无用进程持续运行。

全过程没有解锁、模式切换、飞行设定点、电机或PWM指令，也没有修改网络、时钟、开机自启或正式发布current。飞控单独重启会中断QGC/DDS约82秒；Jetson/SSH不随之重启。掉电或飞控不可达时，任何软件都不能保证立即写回参数，不能称为无条件自动恢复。

## 本地验证

固定参考5项、原23项里程计/9项接收时间/6项旧航向候选/10项坐标回归及隔离ROS通过；实际PX4 EKF库7场景通过。参数事务4项失败回退、实际重启证明1项含两种原启动时间、批量证明2项通过。最终机载验收passed=true仅涵盖上表静态和软件断流范围。

## 下一步

1. 同一固定参考、一次连续融合中，人工缓慢前移/右移/抬高/转向，核对PX4位置与动捕方向、距离、航向和响应时延。
2. 实测刚体原点到IMU偏移与安装姿态；标定实际采样/网络延迟及物理遮挡恢复。此前59.7705cm移动不作为本轮动态融合验收。
3. 单独确定并验证室内高度来源；本轮保留气压高度，没有启用动捕高度或外部速度输入。
4. 实现飞行常驻桥接与动捕失效策略，验证人工接管/急停/失联，复核供电、电池≥40%等起飞门禁。现有工具解锁即停发，不能直接拿来飞行。

## 证据与复核

通过轮次：report/fixed_reference_20261023/{test.name}/validation.json、samples.jsonl、analysis.json、manifest.json、configure_reboot_verified.json、restore_reboot_verified.json、parameter_lease_result.json。

独立恢复审计：report/fixed_reference_20261023/{post.name}/comparison.json及同目录完整参数快照。总入口report/fixed_reference_20261023/final_summary.json。失败轮次与原始stderr均在同任务目录。机载暂存位置由通过轮次manifest精确记录，不是正式current部署。
"""
doc=PROJECT/"docs/A机固定动捕航向实机验证_20261023.md"
doc.write_text(text,encoding="utf-8")
plan=PROJECT/"config/mocap_a.fixed_reference_ground.plan.yaml"
d=yaml.safe_load(plan.read_text(encoding="utf-8"))
d["status"]="static_ground_passed_parameters_restored"
d["reference"]["navigation_message_convention_validated"]=True
d["reference"]["px4_input_frame_decision"]="explicit_fixed_indoor_navigation_NED_encoding_ground_only"
d["validation_report"]="docs/A机固定动捕航向实机验证_20261023.md"
d["limitations"]["continuous_motion_validated"]=False
plan.write_text("# Planning/status data; not a bridge config or executable parameter transaction.\n"+yaml.safe_dump(d,allow_unicode=True,sort_keys=False),encoding="utf-8")
old=PROJECT/"docs/A机固定动捕航向地面验证步骤_20261023.md"
s=old.read_text(encoding="utf-8")
s=s.replace("本文件是下一轮实施方案，未写入实机参数、未重启设备，也不是飞行许可。", "本文件保留最初方案。**2026-10-23更新：本地实现、真实静态融合和软件断流已通过，测试参数已恢复并重启核验；最新结果见《A机固定动捕航向实机验证_20261023.md》。以下“尚未实现/未执行”描述为实施前历史，不再代表当前状态。**")
old.write_text(s,encoding="utf-8")
memory=PROJECT/"MEMORY.md"
entry=f"""

## 2026-10-23 固定动捕航向/MAG_NONE实机静态验证通过（最新）

用户换电后继续授权无桨未解锁地面融合。A序列号1612425621719/Jetson boot39417b19保持，SSH .123→.4。实现独立fixed_lab_navigation配置，固定[Y,X,-Z]、world原点0和完整姿态同转；明确非地理北向的室内导航NED编码，fresh参数证明MAG5/EV9/SYNCT1、同boot与UID、磁/地理观测冲突即停发，不复用磁航向快照。PX4同版EKF库7场景、本地5固定+既有23/9/6/10及ROS、4事务/1重启证据/2批量证明验证通过。

最终{test.name}机载passed=true：MAG5/EV9重启读回，静态25秒，EV位置+航向标志28样本跨度26.270秒，2627个位置样本xy/vxy/heading_good_for_control均true、dead_reckoning均false；磁航向/3D均false，真实输入1035帧。软件停止源后400ms无新输入并锁存，物理遮挡未测。先前4轮失败保留：约82秒飞控网络恢复超过早期时限、连续重启uptime判断、跨重启旧订阅、串行参数证明过期；最终修复且未放宽2秒证明过期/未解锁着陆门禁。

结束EV0/MAG0/SYNCT1已恢复并再次重启飞控核验。独立{post.name}1127参数读全，仅CAL_BARO0_OFF -48.375→-38.1484375变化，符合PX4启动自动相对气压计校准源码；不是全参数完全一致，不强行覆写。Agent1006/relay1008持续active、current仍b4453dd、本任务机载进程0，未改网络/时钟/自启，未解锁/飞行/电机/PWM。核心目标已在静态通过；当前不常驻融合。人工连续移动/转向、精确安装偏移/源延迟/物理遮挡/高度/飞行桥接及失效策略仍待，不能用地面工具直接飞行。报告docs/A机固定动捕航向实机验证_20261023.md；证据report/fixed_reference_20261023/final_summary.json。所有本轮文件留在Ubuntu22.04工程，旧工程及C盘无新辅助文件。
"""
if entry.strip() not in memory.read_text(encoding="utf-8"):
    with memory.open("a",encoding="utf-8") as f:f.write(entry)
print(json.dumps(summary,indent=2))
print("Document:",doc)
