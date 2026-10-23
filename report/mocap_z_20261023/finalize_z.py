"""Publish verified Z result and rollback evidence, never infer success from parameters alone."""
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parents[1]
run=sorted(ROOT.glob("xyz_reference_20*"))[-1]
post=sorted(ROOT.glob("after_20*"))[-1]
v=json.loads((run/"validation.json").read_text(encoding="utf-8"))
a=json.loads((run/"xyz_comparison.json").read_text(encoding="utf-8"))
p=json.loads((post/"comparison.json").read_text(encoding="utf-8"))
assert v["passed"] and a["passed"] and p["passed"],"do not publish unverified success"
s=v["software_source_loss"]
summary=dict(passed=True,flight_approval=False,scope="XYZ_yaw_static_ground_only",
    run=run.name,post_audit=post.name,ev_xyz_yaw_sample_span_s=a["ev_xyz_yaw_sample_span_s"],
    comparison=a,source_loss={"no_input_after_400ms":s["no_input_after_400ms"],
        "bridge_latched":s["bridge_latched"],"physical_tracking_loss_tested":False,
        "ev_height_active_after_3s":s["final_flags"].get("cs_ev_hgt")},
    restored={"EKF2_EV_CTRL":0,"EKF2_MAG_TYPE":0,"EKF2_HGT_REF":1,"UXRCE_DDS_SYNCT":1},
    restore_restart_verified=v["lease"]["restore_restart_verified"],
    parameter_differences=p["parameter_differences"],jetson_boot_unchanged=p["jetson_boot_unchanged"],
    remaining_task_processes=p["remaining_task_processes"],permanent_fusion_enabled=False)
(ROOT/"final_summary.json").write_text(json.dumps(summary,indent=2),encoding="utf-8")
z=a["z_abs_error_m"]
doc=f"""# A机动捕XYZ与航向融合地面验证（2026-10-23）

**Z融合实机静止验证已通过。** A机临时配置为动捕提供XYZ位置和航向，以动捕为高度参考，保留气压计辅助；磁力计融合关闭。测试后已恢复原参数并核验飞控重启，当前没有常驻融合。此次没有解锁、模式/设定点、电机或PWM指令。

## 参数与坐标

| 参数 | 本轮备份/恢复值 | 临时测试值 | 含义 |
| --- | --- | --- | --- |
| EKF2_EV_CTRL | 0 | 11 | bit0水平位置、bit1垂直位置、bit3航向 |
| EKF2_HGT_REF | 1 | 3 | Vision作为高度参考；元数据要求重启 |
| EKF2_MAG_TYPE | 0 | 5 | 不融合磁力计；元数据要求重启 |
| UXRCE_DDS_SYNCT | 1 | 保持1 | 保留DDS时间转换 |

配置config/mocap_a.xyz_ground.yaml明确height_fusion=external_vision_reference、height_origin=fixed_laboratory_marker_origin_not_agl。转换仍local=[worldY,worldX,-worldZ]，固定world原点0，抬高对应Z减小，不随当前位置重置高度原点。当前是刚体标记原点的实验室高度，**不是起落架离地高度，也未标定IMU安装偏移**。

核心mocap_odometry新增显式高度合同；XYZ路径要求新鲜、同boot/UID、重启已核验的MAG5/EV11/HGT3/SYNCT1读回证明。原XY配置仍精确要求旧MAG5/EV9/SYNCT1，不会隐式改成XYZ。错误高度参考、磁/地理观测冲突、证明/动捕/状态过期均停发锁存。默认严格源配置及所有未标定确认项仍保持原限制。

## 实机结果

A序列号1612425621719；PX4 1.16.0 UID3544984969635313204。FZMotion3.0.0.9，planeA，192.168.1.111:3883。

- 静止采集25秒；cs_ev_pos/cs_ev_yaw/cs_ev_hgt共同有效样本跨度{a["ev_xyz_yaw_sample_span_s"]:.3f}秒。
- 静态验收xy/vxy/z/vz/heading_good_for_control均有效；磁航向/3D融合关闭。
- 稳定20秒日志{a["pairs"]}组接收时间相近的对照：Z绝对差平均{z["mean"]:.6f}m、最大{z["maximum"]:.6f}m。动捕XYZ均值{a["input_xyz_mean_m"]}，PX4均值{a["px4_xyz_mean_m"]}。
- 此差值只说明输入和估计在静止时接近，不代表真实测量精度，也未证明动态延迟。
- 停止软件源后400ms后无新PX4输入，桥接锁存；3秒后cs_ev_hgt={s["final_flags"].get("cs_ev_hgt")}。这里只验证软件断流，不是物理遮挡或飞行中断流处置。

## 本地与恢复核对

同版PX4 EKF库8场景通过，覆盖不同Z、气压计高度变化、外部高度断流。5项高度合同、5项参数失败回退（含高度写入失败）、旧23/9/6/10回归及隔离ROS通过；XYZ ROS案例含Z符号和错误高度参考停发。代码只解析/编译通过不作为实机验收替代。

全过程由A机独立参数守护执行，完整1127参数先备份；测试结束先停本轮源/桥接，再恢复EV0/HGT1/MAG0，单独重启飞控并读回。配置重启/恢复重启实测通常各约82秒恢复遥测；只重建本任务订阅，不重启Jetson/共享服务。掉电或飞控不可达时无法无条件保证立即回退，失败证据必须保留。

独立审计完整参数差异：{json.dumps(p["parameter_differences"],ensure_ascii=False)}。气压计自动校准候选差异单独保留；不宣称全量参数完全一致，不强行覆盖设备自行更新的校准。Jetson boot保持，既有Agent/relay仍active，正式current未切换，本任务进程无残留；网络、时钟、自启未改。

## 下一步仍待

人工保持无桨未解锁，连续抬高已知距离并落回，检查Z变化符号、幅度、响应和复位；再核对平移/转向。地面工具要求已着陆，人工移动可能触发着陆状态变化而停发，不能为完成动作测试跳过现有保护；需先设计专用手持验证边界。

确定起降平面与IMU偏移，避免把实验室Z直接当离地高度；核实源时间/网络延迟、物理遮挡与恢复、气压计辅助表现。实现并验收飞行常驻桥接、定位失效策略和人工接管/急停后，才进入受控起飞测试。当前地面桥接解锁即停发，不能直接用于飞行。

## 证据

report/mocap_z_20261023/{run.name}/validation.json、samples.jsonl、xyz_comparison.json、manifest.json、参数守护与两次重启核验文件。
独立审计report/mocap_z_20261023/{post.name}/comparison.json。
总入口report/mocap_z_20261023/final_summary.json。所有本轮脚本、配置、日志位于Ubuntu22.04工程，无C盘辅助文件。
"""
(PROJECT/"docs/A机动捕XYZ与航向融合_20261023.md").write_text(doc,encoding="utf-8")
entry=f"""

## 2026-10-23 动捕Z参考实机静止融合通过（最新）

用户明确要求“你来融合z”。新增config/mocap_a.xyz_ground.yaml及显式height_fusion=external_vision_reference/height_origin=固定实验室marker原点非AGL；核心XYZ路径严格要求MAG5/EV11/HGT3/SYNCT1同boot新鲜重启证明，旧XY路径不放宽。位置仍[Y,X,-Z]、固定world原点0，未伪造安装/时序/遮挡标定。PX4同版EKF8场景、高度5/三参数回退5/既有23+9+6+10及隔离ROS通过。

A机{run.name}passed=true，静态25秒；EV位置/航向/高度标志共同有效跨度{a["ev_xyz_yaw_sample_span_s"]:.3f}秒，XYZ及速度/可控航向有效、磁航向/3D关闭。{a["pairs"]}组稳定日志Z输入/估计差均值{z["mean"]:.6f}m/最大{z["maximum"]:.6f}m，仅静态数据接近不是测量精度。软件断流400ms后无输入并锁存，物理遮挡与人工抬高/动态未测。

测试后EV0/HGT1/MAG0/SYNCT1已恢复、飞控重启核验通过；完整1127参数独立审计{post.name}passed=true，差异{json.dumps(p["parameter_differences"],ensure_ascii=False)}，自动校准候选保留，不宣称参数全部相同。Jetson未重启、Agent/relay与current不变、本轮进程0、未解锁/飞行/执行器/网络时钟自启修改。当前没有常驻XYZ融合。后续人工抬高/移动、起降平面与IMU偏移/源延迟/遮挡/飞行桥接及失效保护待。报告docs/A机动捕XYZ与航向融合_20261023.md，证据report/mocap_z_20261023/final_summary.json。
"""
with (PROJECT/"MEMORY.md").open("a",encoding="utf-8") as f:f.write(entry)
print(json.dumps(summary,indent=2))
