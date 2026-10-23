#!/usr/bin/env python3
"""Record verified A-only results and archive this task's temporary editors."""
from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess

root=Path(__file__).resolve().parents[2]
report=root/"report/a_solo_gazebo_20261019"
runs=[]
for name in ("run_103809","run_104115"):
    path=report/name
    data=json.loads((path/"result.json").read_text(encoding="utf-8"))
    assert data["passed"] and data["workers_stopped"]
    assert data["final"]["landed"] and not data["final"]["armed"]
    assert data["max_height"] >= .8 and data["gz_max_z"] >= .8
    assert "HOVER" in data["task_phases"] and "LAND_REQUESTED" in data["task_phases"]
    runs.append(dict(run=name,estimated_max_height_m=data["max_height"],
                     gazebo_max_z_m=data["gz_max_z"],landed=True,armed=False))
ros=json.loads((root/"report/a_solo_validation_20261019/ros_summary.json").read_text(encoding="utf-8"))
print("ROS_SUMMARY",json.dumps(ros,ensure_ascii=False),flush=True)

# Remove one-off intrusive diagnostics from the reusable demonstration.
runner=root/"tools/run_a_solo_gazebo.py"
source=runner.read_text(encoding="utf-8")
source=source.replace('px4process=launch("px4",','launch("px4",')
source=source.replace(';diagnosed=False','')
start=source.index('            if latest.get("phase")=="LAND" and latest.get("reason")!="LAND_REQUESTED" and not diagnosed:')
end=source.index('            if latest.get("phase")=="DONE":',start)
source=source[:start]+source[end:]
source=source.replace('simulation_rc=True,passed=False','simulation_rc=True,simulation_external_local=True,passed=False')
import ast
ast.parse(source,str(runner))
runner.write_text(source,encoding="utf-8")

# Archive exact files created during this Gazebo task, without changing prior work.
names=[
"fix_a_gazebo_params.py","fix_a_gazebo_land_watch.py","fix_a_gazebo_heartbeat.py",
"fix_a_gazebo_protocol.py","fix_a_gazebo_clock.py","fix_a_gazebo_delivery.py",
"fix_a_gazebo_clock_snapshot.py","fix_a_gazebo_clock_domains.py",
"fix_a_gazebo_diagnostics.py","fix_a_gazebo_buzzer_trial.py",
"fix_a_gazebo_queue_diagnostic.py","fix_a_gazebo_diagnostic_scope.py",
"cleanup_a_gazebo_retry.py","inspect_a_solo_gazebo.py","inspect_a_function_log.py"]
archive=report/"diagnostic_helpers"
archive.mkdir(exist_ok=True)
moved=[]
for name in names:
    path=root/"tools"/name
    assert path.resolve().parent == (root/"tools").resolve()
    if not path.exists():
        continue
    dest=archive/name
    assert not dest.exists(),str(dest)
    moved.append(dict(source=str(path.relative_to(root)),destination=str(dest.relative_to(root)),
                      sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    shutil.move(str(path),str(dest))
(report/"archived_helpers.json").write_text(json.dumps(moved,indent=2)+"\n",encoding="utf-8")

# netns hides transient PIDs from the host manifest; inspect actual host argv instead.
ps=subprocess.run(["/bin/ps","-eo","pid,ppid,args"],cwd=root,timeout=10,
                  capture_output=True,text=True,encoding="utf-8",errors="replace",check=True)
(report/"process_audit.txt").write_text(ps.stdout,encoding="utf-8")
active=[]
markers=("tools/run_a_solo_gazebo.py","tools/check_a_solo_ros.py","tools/run_a_solo_isolated.py",
         "MicroXRCEAgent udp4 -p 8888",str(report/"run_"))
for line in ps.stdout.splitlines()[1:]:
    fields=line.split(None,2)
    if len(fields)==3 and int(fields[0])!=os.getpid() and any(m in fields[2] for m in markers):
        active.append(line)
assert not active,"Task processes still present: "+repr(active)
validation=dict(date="2026-10-19",scope="A-only functional validation",real_hardware=False,
    gazebo_runs=runs,ros_cases=ros,unit_tests=26,takeoff_gate_tests=10,
    mavlink_protocol_tests=7,fcu_parameter_tests="passed",diff_check="passed",
    task_processes_remaining=active,
    limitation="Earlier battery/magnetometer telemetry interruptions remain unexplained; two later consecutive flights passed. No real flight, PWM release, or B coordination tested.")
(report/"functional_validation.json").write_text(json.dumps(validation,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")

doc="""# A 机单机 Gazebo 功能检查（2026-10-19）

本次在 Ubuntu-22.04 电脑内运行真实 PX4 1.16 SITL 和 Gazebo GUI，使用工程的 A-only 监督及任务节点。没有连接或部署 Jetson，没有真实飞控解锁、PWM 动作或 B 机。

## 已通过

- 完整流程连续通过两轮：自检 → 等待显式 start → Offboard 预发送 → 模式确认 → 解锁 → 垂直起飞 1 m → 稳定悬停 10 s → 请求降落 → 着陆并上锁。
- 通过条件同时检查 PX4 估计高度、Gazebo 真实模型高度、HOVER 阶段和正常 LAND_REQUESTED；保护结束不会被计作正常任务成功。
- 隔离 ROS 8 场景通过：关键数据缺失拒飞、39% 电量拒飞、控制发布者冲突拒飞、仅检查模式无控制输出、正常起降、操作员停止后降落、任务进程失联后降落、人工接管后不抢回。
- A-only 逻辑 26 项、起飞门禁 10 项、MAVLink 参数读取 7 项和飞控参数校验全部通过；git diff --check 通过。

| 运行 | PX4 估计最大升高 | Gazebo 模型最高 z | 最终状态 |
| --- | --- | --- | --- |
| run_103809 | 1.012 m | 0.997 m | landed=true，armed=false |
| run_104115 | 1.017 m | 1.009 m | landed=true，armed=false |

仿真提供模拟遥控器输入，以及从 Gazebo 模型姿态生成的外部定位；两者均进入 PX4 处理，不伪造 PX4 输出健康遥测。无 GPS 的本轮通过，不能替代真实定位和遥控器验收。X500 仿真参数不能直接复制到实机。

## 修正范围

1. 参数读取跳过飞控转发的 GCS 心跳，但仍验证 A 的 SYSID1/component1/PX4 身份。
2. 识别 PX4 参数列表中的 _HASH_CHECK 元数据，继续检查完整参数计数和索引。
3. 兼容 PX4 1.16 的 BAT_LOW_THR/BAT_CRIT_THR/BAT_EMERGEN_THR，仍拒绝冲突阈值。
4. 单机适配器按实际 1 Hz 着陆状态设置接收窗口。
5. 仅隔离 Gazebo 测试注入仿真时钟，处理跨传输通道到达顺序；真实节点默认时钟和门禁保持原逻辑。

早期运行出现过解锁后电池与磁力计遥测中断，任务执行了保护降落，没有误报成功。声音模块对照未解决该现象，相关参数已撤回；根因尚未最终定位。之后两个连续运行完整通过，因此本次证明正常功能及上述故障分支可执行，尚不能保证长期运行稳定性。

舵机 1700 µs/约 1.5 s 的投放、双机通信与接取不属于这个 A-only 起降入口；本次没有验收这些功能，也没有放宽正式双机门禁。

## 证据及复现

- functional_validation.json：本次汇总。
- run_103809/result.json、run_104115/result.json：完整任务结果。
- 各运行目录的 px4.log、mission.log、supervisor.log 和 log/*.ulg：原始证据。
- ../a_solo_validation_20261019/ros_summary.json：8 个隔离 ROS 场景。
- diagnostic_helpers/：排障临时脚本归档，不是日常启动入口。

复现（仅本机隔离仿真）：

```bash
cd /home/caolihao/drone_payload_catch
sudo -n /usr/bin/python3 tools/run_a_solo_gazebo.py
```

启动器建立仅 loopback 的网络命名空间，普通用户运行仿真，结束后清理本次进程。两次成功运行以及本轮 ROS 回归的工作进程均已退出，完整主机进程清单保存在 process_audit.txt。
"""
(report/"A机单机功能检查.md").write_text(doc,encoding="utf-8")

entry="""

## A 机单机 Gazebo 功能验证（2026-10-19）

本机 Ubuntu22.04 / PX4 1.16 SITL / Gazebo GUI 连续两次完成自检、Offboard、解锁、1m起飞、稳定悬停10s、正常降落并上锁；Gazebo最高z分别0.997m和1.009m。8个隔离ROS场景、26逻辑、10起飞门禁、7参数协议及飞控参数测试通过。修正转发GCS心跳、_HASH_CHECK元数据和BAT_*共享阈值兼容；仿真时钟仅隔离测试注入，真实默认不变。早期电池/磁力计遥测中断曾触发保护，根因尚未最终定位，不能据两次通过宣称长期稳定。模拟RC+外部定位，不验收真实A/B/PWM；本次未SSH/部署/真实动作。证据report/a_solo_gazebo_20261019/A机单机功能检查.md。临时编辑脚本已归档diagnostic_helpers，任务进程已退出。
"""
for filename in ("MEMORY.md","AGENTS.md"):
    path=root/filename
    with path.open("a",encoding="utf-8") as handle:
        handle.write(entry)
print("RECORDED",report/"A机单机功能检查.md",flush=True)
