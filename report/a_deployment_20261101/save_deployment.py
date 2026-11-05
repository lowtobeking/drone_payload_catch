"""Preserve deployment, validation and exact rollback instructions."""
from datetime import datetime
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
WORK=Path(__file__).resolve().parent
m=json.loads((WORK/"manifest.json").read_text(encoding="utf-8"))
post=json.loads((WORK/"post_verification.json").read_text(encoding="utf-8"))
validation=json.loads((WORK/"activation_result.json").read_text(encoding="utf-8"))
assert m["activated"] and post["passed"]
assert all(t["exit"]==0 for t in validation["tests"])
doc=f"""# A机单机起降程序部署（2026-11-01）

本次用户授权SSH部署“自检→起飞→悬停→降落”程序。已部署并选为当前版本，未运行任务。

- A机：nvidia@192.168.1.5，nvidia-desktop，序列号1612425621719。
- 本地源码提交：307056e。
- 当前版本：{m["release_id"]}。
- 当前链接：/home/nvidia/payload_catch_deploy/current。
- 回退版本：{Path(m["before_current"]).name}，保留在previous。
- 机载构建：ARM64，colcon退出0；构建临时服务已退出。
- 机载纯逻辑验证：83项通过；3个部署Bash入口语法通过；版本清单完整性通过。
- 激活前只读确认：飞控arming_state=1、landed=true，新鲜度检查通过。
- 激活后：无任务/动捕桥接进程；Jetson启动标识、既有配置、Agent/relay/Wi-Fi辅助服务及PID、重启计数保持。
- 未发送飞行、模式、解锁或执行器命令，未修改飞控参数、网络、引脚、自启或旧工程。

## 已部署任务

入口：/home/nvidia/payload_catch_deploy/current/src/deploy/run_a_solo.sh。

单机空载程序按当次有效自检凭据及现场确认进入就绪，健康持续3秒，等操作员明确开始，再预发送设定点、切Offboard、解锁并垂直起飞。现有配置height_m=1.0、hover_s=10.0；达到目标并稳定至少2秒后持续悬停10秒，再请求PX4降落，确认着陆上锁后结束。没有物块投放或B机任务。上电不会自动执行。

## 当前配置限制

/home/nvidia/payload_catch_deploy/a_solo.yaml仍为positioning=outdoor_gps、site_checked=false、payload_removed=false，字节哈希与部署前相同。用户说明动捕目前不可用，本轮未配置室内external_local，也未恢复EV融合；本部署不表示已具备室内起飞条件。动捕飞行桥接和组合启动候选随包保存，但未启动。现场条件应在实际运行前据实填写，不伪造自检通过。

## 回退

确认A机着陆、未解锁、没有任务运行后，在A机SSH终端执行：

    /usr/bin/python3 /home/nvidia/payload_catch_deploy/current/src/tools/fleet_deploy.py rollback --base /home/nvidia/payload_catch_deploy --ground-confirmed

本次previous指向{Path(m["before_current"]).name}，命令只切换程序版本，不启动任务。新旧版本及本次部署包均保留。

## 记录

本地完整记录：report/a_deployment_20261101，包含before/after、manifest、build_status、activation_result、post_verification及日志。
远端记录：{m["remote"]}。
首次机载离线测试因部署包未附带历史report父目录失败，未切换版本；补齐测试父目录后83项全部通过，未改业务代码。

当前会话没有持续SSH保活、任务或测试进程；保留原有通信与网络辅助服务。本记录不代表真实起飞验收，不涉及GitHub推送。
"""
(WORK/"A机部署记录.md").write_text(doc,encoding="utf-8",newline="\n")
(ROOT/"docs/A机单机起降部署_20261101.md").write_text(doc,encoding="utf-8",newline="\n")
with (ROOT/"MEMORY.md").open("a",encoding="utf-8",newline="\n") as f:
 f.write(f"""

## 2026-11-01 用户授权A机真实SSH部署（最新）

用户要求把单机自检→起飞1m→稳定悬停10秒→降落上锁程序部署到真实A。已SSH核对serial1612425621719/boot2e0322d9-53cb-40ae-a82b-abacfc391a3d，上传源码307056e的版本化包，ARM64构建和机载83项纯逻辑测试通过，未解锁/已落地新鲜状态通过后切换current到{m["release_id"]}；previous保留{Path(m["before_current"]).name}。独立复查post_verification通过，Agent1060/relay1063/Wi-Fi辅助2988以及重启计数、原配置与Jetson boot不变，本轮任务进程无残留，未运行融合/任务/电机/PWM、未改飞控参数和开机自启。原a_solo.yaml仍outdoor_gps/site_checked=false/payload_removed=false，动捕暂不可用，未伪造现场确认，不代表能室内飞。首次单测缺report父目录导致失败，补齐测试目录后全部通过，无业务代码改动。本地部署和回退记录docs/A机单机起降部署_20261101.md、report/a_deployment_20261101。后续实际运行仍需有效自检、现场条件和明确开始；不自动继续试飞。
""")
print(json.dumps({"saved":True,"current":post["current"],"rollback":post["previous"],"checks_passed":83,"mission_started":False,"post_verification_passed":post["passed"]},ensure_ascii=False))
