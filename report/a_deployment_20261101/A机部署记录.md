# A机单机起降程序部署（2026-11-01）

本次用户授权SSH部署“自检→起飞→悬停→降落”程序。已部署并选为当前版本，未运行任务。

- A机：nvidia@192.168.1.5，nvidia-desktop，序列号1612425621719。
- 本地源码提交：307056e。
- 当前版本：da8bcec5fd2a796727a771077a9fcbaf4a064056938035e54bb42bbb8be1cff4。
- 当前链接：/home/nvidia/payload_catch_deploy/current。
- 回退版本：b4453dd12c8482d808613769fb6c781bd7debff566633bf0fc4827a6d2654d37，保留在previous。
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

本次previous指向b4453dd12c8482d808613769fb6c781bd7debff566633bf0fc4827a6d2654d37，命令只切换程序版本，不启动任务。新旧版本及本次部署包均保留。

## 记录

本地完整记录：report/a_deployment_20261101，包含before/after、manifest、build_status、activation_result、post_verification及日志。
远端记录：/home/nvidia/payload_catch_a_deploy_20261101。
首次机载离线测试因部署包未附带历史report父目录失败，未切换版本；补齐测试父目录后83项全部通过，未改业务代码。

当前会话没有持续SSH保活、任务或测试进程；保留原有通信与网络辅助服务。本记录不代表真实起飞验收，不涉及GitHub推送。

## 2026-11-01 室内动捕配置更正

- 按用户要求，A机现场 a_solo.yaml 与本地单机示例选择 external_local，不要求 GPS；任务仍为相对起飞1米、稳定后悬停10秒、降落。
- 现场配置只改变 positioning，保留 site_checked=false、payload_removed=false；备份 /home/nvidia/payload_catch_a_deploy_20261101/a_solo.before_indoor.yaml。
- 独立SSH读回通过，host/runtime.yaml 未变，26项单机逻辑测试通过。
- 未启动任务/桥接/解锁/执行器，未修改 PX4 参数。动捕缺失或 PX4 融合本地定位无效时仍禁止起飞。本次不是实飞验收。
