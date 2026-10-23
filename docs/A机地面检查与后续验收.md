> **2026-10-16 最新：当前实物没有锁扣/脱离传感器。已增加显式定时开锁模式，旧的强制锁扣反馈要求仅适用于sensor模式；定时动作完成不代表物理脱离已确认。代码已验证，真实驱动仍禁用/未标定，尚未部署本轮。详见[无反馈定时释放模式](无反馈定时释放模式.md)。**

# A机地面检查与后续验收

## 第一步：固定地面入口（已完成）

在A机Jetson终端或通过SSH执行：

```bash
bash /home/nvidia/payload_catch_deploy/ground/runtime_20261016/run_a_ground_live.sh
```

默认运行20秒，允许用`--seconds 10`设置5至60秒。脚本在Jetson上加载current指向的已部署工程，运行真实ANode、SafetySupervisor、ReleaseGuard、PayloadServo，订阅真实A/B飞控数据。当前工程源码为tools/run_a_ground_live.py和tools/run_a_ground_live.sh；A机入口是此次校验指纹后安装的独立工具副本，正式发布current和原接管回退记录未改变。后续修改工具须重新同步并核对指纹。

入口先核对本工程通信环境、A角色、ARM架构和已部署文件完整性，持有任务锁。只允许从已落地/未解锁、PWM未使能状态开始。运行期间要求源时间递增、真实落地和未解锁数据持续新鲜；数据缺失或异常即退出。重复检查不能与正式任务同时运行。

所有飞控输入输出重映射到诊断话题，自动解锁false、起飞批准读取/dev/null、双机/RC/GPS门禁保留、PWM enabled/calibrated必须为false。不填假B机/位置/反馈，不生成起飞批准。监视真实与诊断输出，任何控制/释放消息均视为检查失败。本入口不测试舵机、电机或实际飞行。

每次在Jetson的`/home/nvidia/payload_catch_deploy/ground/reports/run-时间-随机值/`创建run.log和result.json；保留历史，不覆盖。正常结束、异常或超时后清理节点；最长80秒外层限制。JSON的passed只表示地面节点启动检查通过，flight_approval与flight_gate_allowed始终false；health列出的失败项仍须完成。结果不包含正式fleet配置、参数读回、传感器校准或电气波形验收。

## 本次机载验证

Jetson=nvidia-desktop、aarch64，PID=7308，运行20.05秒。真实状态46帧，飞控/诊断输出和授权释放计数全0，PWM保持禁用。真实A数据持续到达，B/RC缺项继续阻止任务。进程已退出，结束只读检查无任务进程或飞控输入发布者，真实飞控未解锁；通信服务保留运行。

机载日志：/home/nvidia/payload_catch_deploy/ground/reports/run-20261016T091209Z-zalNpI。本地证据：report/a_ground_entry_20261016/jetson_result.json、jetson_run.log、cleanup_check.json、helper_install.json。遥测缺失/超时/错机/解锁/空中/重复时间戳4项回归及工程quick38/38通过。

## 后续逐步推进

| 步骤 | 当前状态 | 完成条件 |
| --- | --- | --- |
| A机固定地面入口 | 已完成并机载验证 | 真实节点、真实数据、无飞控/PWM输出、记录缺项、退出清理 |
| 电池遥测核实 | 待做 | 解释16.44V但cell_count=1的来源，对照实际电池及参数，核实remaining可信度 |
| 真实释放链 | 待验收 | 本boot PWM基线、真实锁扣/脱离反馈、多次直出1700µs→脱离约1.5s及波动验收 |
| 操作员停止和故障 | 模拟已有，真实链待验收 | 动作前/中停止、进程退出、失联及反馈丢失，验证停脉冲和不重复释放 |
| B机与飞行条件 | 待补齐 | 独立RC、有效定位、双机网络/时钟/几何/配置与飞控参数验收 |
| 完整飞行 | 仅隔离模拟通过 | 自检批准后分阶段实飞，投放接取与顺序降落现场验收 |

每步按照代码/配置→机载运行→检查日志→记录验收推进。物理动作需按当次实际接线与现场条件确定；不为继续下一步而伪造健康数据或放宽正式飞行门禁。机载通信自启动、飞行任务不自启动的现有行为保持。
