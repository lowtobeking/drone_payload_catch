# A机现有服务复用验收（2026-10-16）

用户要求将Jetson旧工程中适用的部分复用到当前工程。已完成源码接入、电脑端检查、ARM构建、真实通信只读验证、机载隔离任务模拟与部署切换。

## 已复用

| 项目 | 复用方式 |
| --- | --- |
| Humble与Fast DDS | 使用系统现有Humble，显式rmw_fastrtps_cpp，不重复安装 |
| PX4消息 | 使用/home/nvidia/ros2_ws/install/setup.bash；提交392e831c1f659429ca83902e66820d7094591410、源码干净 |
| Agent | 现有microxrce-agent.service、UDP8888、域0，进程1003始终未重启 |
| QGC中继 | 现有mav-relay.service、UDP14550，进程1002始终未重启；核对原脚本SHA256、FC10.41.10.2和默认地面站192.168.1.123 |
| 网络配置 | eno1=10.41.10.100、mesh=192.168.1.101、FC SYSID=1 |

没有复制旧飞行控制逻辑、覆盖旧工程、重装软件、改飞控参数、改.bashrc、重启服务或增加任务自启。旧中继脚本仍是运行依赖，不能删除其目录。地面站默认目标是旧服务现值，本次未验证当前QGC电脑IP或双机接取通信。

## 工程接入

新增payload_catch/host_runtime.py、tools/jetson_runtime.py、config/jetson_a.runtime.yaml、deploy/check_jetson.sh。deploy/runtime_env.sh自动读取A机绑定，显式选择域0，清理继承的模拟DDS配置并加载已有消息overlay。

deploy/run_fleet.sh沿用原双机配置、角色/几何/自检门禁，并在启动节点前增加真实基础设施检查：域/角色/地址、服务状态、端口归属、重复Agent、消息版本/源码漂移、中继目标/指纹、FC身份/未解锁、关键遥测新鲜且源时间递增，以及是否已有ROS控制发布者。

fleet支持域0须显式domain_zero_confirmed=true。仍要实际核对双方FC/Agent，示例保留未确认、未填B机与未标定几何时拒绝启动。基础设施检查passed不是起飞批准，不检查成功就自动起飞，也不替代40%电量、定位有效性、RC、双机健康及释放反馈。

## 已部署

- current：8accbc1eca8dcba99b747f953e33ffddee2ac55f16d09db4d48a6df77a4781dd
- previous：fa46032caae12d28676da9014545e6b888a944572744ad205e50fcb0812c026a
- 绑定：/home/nvidia/payload_catch_deploy/host/runtime.yaml
- 配置指纹：24a42070264f64720e7419d6e335852d0b4b8366613a029bd4dc3853f8d8647c
- 真实舵机配置仍disabled/uncalibrated。

实际从ROS_DOMAIN_ID=42启动检查，自动采用绑定域0。真实只读检查确认状态、姿态、位置、电池时间戳递增；SYSID1、未解锁、没有发现ROS控制发布者。已有Agent和中继PID前后保持1003/1002。

本地quick36/36；机载9运行环境、16部署组网、35释放、24安全、10起飞测试通过；机载完整隔离模拟约27.89秒通过。B、飞控动力学、1.5秒锁扣模型和PWM均为模拟，真实运行只订阅传感器和服务信息。没有真实解锁或执行器输出，没有残留测试进程。

首轮验证有一次编排报告目录错误，证据validation_initial_report_path.log保留。修正的是编排路径，工程源码与运行服务未因此变更。原先诊断上传目录的文件已从上一轮归档恢复；新证据在独立/home/nvidia/payload_catch_jetson_reuse_20261016。

## 使用

Jetson上仅检查现有通信环境：

```bash
bash /home/nvidia/payload_catch_deploy/current/src/deploy/check_jetson.sh
```

无需手动启动第二个Agent或中继，也无需改终端默认域。命令不会运行任务/输出PWM/发飞控命令。加载当前终端环境可source current/src/deploy/runtime_env.sh。详细说明在deploy/README_jetson_reuse.md。

## 回退

在地面停机且确认飞控未解锁、没有任务进程后运行：

```bash
python3 /home/nvidia/payload_catch_jetson_reuse_20261016/rollback_reuse.py --ground-confirmed
```

脚本核对当前/previous版本与绑定文件指纹，并持部署/任务锁；将current切回此前fa46032…版本，把新增绑定改名归档。它不删除工程、构建或日志，不改已有服务，不启动任务。本轮核对了回退目标和代码语法，未实际执行回退。

## 实据

reuse_check.json与bound_check.json：绑定前后真实通信；onboard_validation.log：ARM回归和模拟；jetson_result.json/jetson_mission.log：机载模拟；inspect.json/after.json：真实飞控/PWM/服务/已检查boot文件；final_audit.json：指纹与退出；validation.json：汇总；before/：修改前文件。
