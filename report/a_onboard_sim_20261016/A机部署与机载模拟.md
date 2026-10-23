# A机部署与机载隔离模拟（2026-10-16）

用户授权将最新工程部署至A机，并在机载Jetson运行完整流程隔离模拟。本次已完成部署、ARM原机构建、机载模拟以及退出/实机状态核对。

## 当前部署

- 设备：nvidia@192.168.155.186，nvidia-desktop，Orin NX 8GB、Ubuntu22.04.5、ROS2 Humble、aarch64。
- 当前版本：`fa46032caae12d28676da9014545e6b888a944572744ad205e50fcb0812c026a`。
- 入口：`/home/nvidia/payload_catch_deploy/current`，ROLE=a。
- previous：`e27cb2caa69a0ff8fc9cf4d43adbcc3a40419dc07c803536187a1a3608a9d9cb`，原版本及构建完整保留。
- 新工程只在版本工作区内构建；没有安装系统软件、改飞控参数、启动文件或系统服务，没有配置开机自动运行任务。

## 本次机载运行

完整流程在Jetson CPU上运行，耗时约27.89秒。不是电脑WSL代跑。真实ANode、安全监督、释放门禁和舵机逻辑参与；模拟飞控、B机反馈和PWM在独立网络空间内运行，不连接真实飞控。

| 事件 | 从模拟开始计时 |
| --- | --- |
| begin_mock_preflight | 0.006秒 |
| mock_selfcheck_passed | 4.567秒 |
| simulated_arm_accepted | 7.79秒 |
| arrived_at_release_point | 12.292秒 |
| mock_pwm_1700_applied | 12.736秒 |
| mock_sensor_detachment | 14.231秒 |
| cleared_release_area | 16.909秒 |
| mock_B_caught_report | 16.91秒 |
| simulated_land_command_1 | 21.168秒 |
| simulated_touchdown_disarmed_1 | 23.667秒 |
| simulated_land_command_0 | 23.717秒 |
| simulated_touchdown_disarmed_0 | 27.888秒 |

缺关键数据与39%电量阶段均拒绝起飞；模拟自检通过后连续观察健康3秒。模拟1700µs只执行一次，约1.5秒的理想机构模型生成脱离边沿，再离开投放区并完成先B后A降落。

**1.5秒是模拟机构的输入参数，不是这次重新实测的机械延迟。** 模拟边沿按接受开锁命令的时刻加1.5秒生成，反馈送达延迟单独记录；原有20毫秒脱离容差、授权新鲜度和30毫秒命令窗口没有放宽。B接取报告在A清场后合成，本次不验收B的实际轨迹或捕获动力学。

原生测试：35项释放、24项安全运行、10项起飞门禁全部通过；机载完整模拟通过；部署源码指纹验证通过。测试日志中的部分“拒飞”是负例测试的预期输出。

## 适配与问题记录

最初将所有节点和高频合成数据放在单线程模拟器中，在Jetson出现授权晚到；同进程多线程也出现模拟心跳超时。最终让guard、servo、supervisor分进程运行，A与模拟飞控运行在测试主进程，贴近部署结构，合成遥测降至20Hz。全过程未放宽真实安全阈值、未通过结束用户系统程序来提高性能。

模拟机构最初误用回调接收时刻作为脱离边沿，已改为区分理想物理边沿与反馈送达。还修正了新部署报告目录权限、目标ROS环境加载，以及ROS信号处理导致的测试子进程重复关闭问题。各轮失败证据保留initial_*目录。

已部署的源码也包括此前修复：完成且反馈确认的释放不会被正常降落停止通知误报FAULT；动作中止、原有故障锁存与单次动作保护保留。

## 收尾实机状态

前后只读采样均确认真实飞控未解锁；当前arming_state=1。PWM保持period=20000000、duty=0、enable=0，33脚PWM模式按用户要求保持。无真实电机或舵机动作。

原mav-relay和microxrce-agent服务仍active；已核对的extlinux.conf哈希及boot_id未变。没有真实任务进程或模拟测试进程残留。真实飞行舵机配置仍enabled=false、calibrated=false，不能因为模拟通过而自动开启真实释放。

## 再次机载模拟

地面停机后，在Jetson执行：

```bash
sudo -n /usr/bin/python3 /home/nvidia/payload_catch_deploy/current/src/tools/run_a_full_mission_isolated.py
```

该入口创建独立netns、普通用户ROS、MockPWM和有限寿命的测试子进程；退出回收。报告在current/src/report/a_full_mission_20261016，复跑会覆盖同名结果，需保留时先归档。它不是实际飞行启动命令。

## 回退

确认飞机已落地、飞控未解锁且没有任务进程后，在Jetson执行：

```bash
python3 /home/nvidia/payload_catch_deploy/current/src/tools/fleet_deploy.py rollback --base /home/nvidia/payload_catch_deploy --ground-confirmed
```

该命令持部署/任务锁后将current切回previous，原源码、构建和日志不删除，不自动启动任务。此回退入口的目标已核对为原e27cb2…版本，本轮没有实际执行回退。

## 证据与限制

build.log、onboard_validation.log、jetson_mission.log、jetson_result.json、inspect.json、after.json、final_audit.json、activation.log和deployment_result.json记录完整结果。Jetson上传目录/home/nvidia/payload_catch_onboard_sim_20261016作为部署排错证据保留，没有常驻程序。中间未选中的版本也作为失败诊断记录保留。

这验证了ARM环境上的代码兼容、任务状态转换、模拟时序与软件门禁，不等于实机可起飞。真实定位、独立RC接管、A/B通信和接取、真实锁扣/脱离反馈、驱动写入与机械延迟抖动、电池遥测来源等仍需实机验收。
