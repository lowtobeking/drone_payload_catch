"""Collect this manual direction check's evidence; no device or flight IO."""
from pathlib import Path
import json
work=Path(__file__).resolve().parent
project=work.parents[1]
state=json.loads((work/"axis_check_state_20261021.json").read_text(encoding="utf-8"))
assert state["label"]=="yaw_left"
records={}
while state is not None:
    local=Path(state["local"])
    assert local.parent==work
    sample=json.loads((local/"axis_summary.json").read_text(encoding="utf-8"))
    proc=json.loads((local/"processes.json").read_text(encoding="utf-8"))
    assert sample["stationary"] and not sample["forwarded_to_px4"]
    assert all(p["exited"] and p["exit"]==0 for p in proc["cleanup"])
    records[sample["label"]]=dict(directory=str(local),remote_logs=state["remote_logs"],sample=sample,processes=proc)
    state=state.get("reference")
assert set(records)=={"baseline","forward","right","up","yaw_baseline","yaw_left"}
forward=records["forward"]["sample"]["comparison"]["delta_previous_body_frd_m"]
right=records["right"]["sample"]["comparison"]["delta_previous_body_frd_m"]
up=records["up"]["sample"]["comparison"]["delta_local_m"]
turn=records["yaw_left"]["sample"]["comparison"]["relative_euler_xyz_deg"]
checks=dict(forward_positive=forward[0]>.5 and abs(forward[1])<.05,
            right_positive=right[1]>.5 and abs(right[0])<.15,
            up_negative_z=up[2]<-.15,
            left_negative_yaw=-105<turn[2]<-75,
            level_after_turn=all(abs(x)<2 for x in records["yaw_left"]["sample"]["roll_pitch_yaw_deg"][:2]))
assert all(checks.values())
result=dict(manual_direction_checks=checks,
            scope="A Jetson live preview; translation direction and level yaw direction only",
            records=records,
            operator_clarification={"forward_distance":"operator says close to 1m, not ruler measurement"},
            precise_scale_calibrated=False,full_mounting_calibrated=False,lever_arm_measured=False,
            server_timing_verified=False,px4_fusion_verified=False,forwarded_to_px4=False,flight_approval=False)
(work/"axis_check_results_20261021.json").write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
doc="""# planeA 实时方向验证（2026-10-21）

本轮通过SSH在A机Jetson执行真实planeA接收与坐标转换预览。每步读取未解锁/已着陆状态，
核对已部署预览文件哈希，在ROS191 localhost域限时采集18秒，使用最后3秒静止样本统计；
每步结束自动关闭本次源/转换/原生接收进程。未运行飞行或PWM，未向PX4输入定位数据或更改参数。

## 实测结果

| 操作 | 实测结果 | 判断 |
|---|---|---|
| 沿机头前移 | 前向+0.9367m、侧向-0.0079m | 前向方向正确；用户确认实际接近1m |
| 沿机身右移 | 右向+0.8782m、前向+0.0770m | 主要为正右方向，伴有手动前向偏移 |
| 整机抬高 | 实验室Z -0.2501m，水平同时偏移约0.125m | 向上对应Z减小，幅度处于所要求20–30cm范围 |
| 水平左转 | 相对水平基准航向-86.31度 | 左转对应负航向，接近手动约90度 |

前/右投影使用各步前一静止姿态的机体FRD轴；高度使用固定实验室坐标Z。
机体坐标投影是分析读数，不是把共享实验室坐标系随飞机重置。
抬高时姿态显示滚转11.51/俯仰-4.52度，因此放回水平后另取yaw_baseline。
新基准roll/pitch/yaw=[0.430,-0.557,-2.825]度；
左转后=[0.292,0.179,-89.130]度，相对转角约-86.31度，不能仅用最终-89.13度作为相对转角。
水平左转伴随约11.6cm水平位置变化；目前无已测杠杆臂/旋转中心，不能据此判断位置转换误差。

## 结论与限制

本轮确认候选转换在A机实时预览中的前、右、上、水平左转方向与实际手动动作相符。
这不是精确尺寸/角度标定：未使用独立尺具或角度仪；手动偏移不能视作跟踪误差或比例误差。
尚未独立校准完整roll/pitch安装矩阵、动捕刚体原点到机体/IMU原点的偏移、跟踪丢失与服务器时序。
源时间戳与A机墙钟语义/同步仍待核实；本机接收时间不能替代已验证的采样时间。
mounting_confirmed/lever_arm_measured/server_timing_verified仍false，不提升为飞行授权，不宣称PX4已经融合。

下一阶段先补测原点偏移和时序/跟踪失效，再实现PX4里程计发送、备份并核对实机EKF参数、
无桨地面检验水平位置/速度/可控航向与定位失效处理。

## 证据与入口

全部六步详细统计、原始JSONL、接收器日志与PID退出记录：
axis_check_results_20261021.json引用的axis_check_20261021T*目录。
采集入口sample_axis_preview_a_20261021.py，统计axis_preview_stats_20261021.py。
前移的30cm建议与94cm读数差异已向用户核实：用户实际移动接近1m，因此未据此判定尺度错误。
"""
(work/"planeA实时方向验证_20261021.md").write_text(doc,encoding="utf-8")
memory=project/"MEMORY.md"
text=memory.read_text(encoding="utf-8")
marker="2026-10-21 planeA实时人工移动方向核对完成"
if marker not in text:
    text+="\n"+marker+"：在A机已部署隔离预览逐步采集baseline/forward/right/up/yaw_baseline/yaw_left。机体前+0.9367m（用户确认实际接近1m）、右+0.8782m伴前偏0.077m、实验室Z抬高-0.2501m；抬高有倾斜故重取水平转向基准，左转相对yaw-86.31度，最终roll0.292/pitch0.179/yaw-89.130度。四项方向与手动动作相符，六次流程均正常清理。完整姿态安装、精确尺度、杠杆臂、源时间/跟踪失效仍未验收，不改mounting_confirmed等标志。未发送PX4/改参数/运行飞行或PWM。证据report/mocap_connection_20261019/axis_check_results_20261021.json及planeA实时方向验证_20261021.md；采集状态当前yaw_left，未来新一轮baseline应显式重取。\n"
    memory.write_text(text,encoding="utf-8",newline="\n")
print(json.dumps(dict(checks=checks,report=str(work/"planeA实时方向验证_20261021.md")),ensure_ascii=False))
