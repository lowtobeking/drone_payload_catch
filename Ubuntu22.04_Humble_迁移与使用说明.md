# Ubuntu 22.04 / ROS 2 Humble 开发环境迁移记录

日期：2026-10-14。本文件对应电脑的新 WSL 发行版 `Ubuntu-22.04`。

## 环境与路径

- Ubuntu 22.04.5 LTS（Jammy），Python 3.10.12，ROS 2 Humble ros-base。
- 电脑为 x86_64 / WSL 2；Jetson 为 ARM64 / L4T 36.4.7。Ubuntu、ROS 和 Python 对齐，内核、架构及 Jetson 驱动不同。构建产物不能直接复制到 Jetson。
- Windows 安装目录：`C:\Users\caoli\WSL\Ubuntu-22.04`。
- 开发用户：`caolihao`。新 WSL 用户配置了免密码 sudo；没有复制旧系统的密码或 SSH 私钥。
- 新工程：`/home/caolihao/drone_payload_catch`。
- 新工程的 Windows 访问路径：`\\wsl.localhost\Ubuntu-22.04\home\caolihao\drone_payload_catch`。
- 工程工作区：`~/payload_catch_ws`；消息工作区：`~/drone_package_20260908/ros2_ws`。
- Python 虚拟环境：`~/drone_package_20260908/venv`，使用 `--system-site-packages` 保留系统 ROS 依赖可见性。
- 旧 Ubuntu-24.04 工程保留；新旧工程是独立副本，后续修改不会自动同步。
- Windows 的 WSL 默认发行版没有切换，日常开发要明确进入 Ubuntu-22.04。

## 进入开发环境

在 Windows PowerShell 运行：

```powershell
wsl -d Ubuntu-22.04 -u caolihao
```

新用户 `.bashrc` 会加载工程 `env.sh`。也可以手动加载：

```bash
source ~/drone_payload_catch/env.sh
cd ~/drone_payload_catch
python3 --version
printenv ROS_DISTRO
```

预期 Python 为 3.10.12、ROS_DISTRO 为 humble。`env.sh` 根据 Ubuntu 版本选择 ROS，已加载其他 ROS 时拒绝混用；不要复制旧 Jazzy 的 install/setup.bash 到新环境。

## 迁移方法及版本

完整备份保存源代码、未提交修改、未跟踪文件和 Git 历史。迁移源码包排除了旧 `build/install/log`、Python 缓存和虚拟环境。仅迁移实际使用的依赖源码，然后在新环境编译。

| 组件 | 迁移基线 |
| --- | --- |
| PX4 | 已验证快照 `PX4-Autopilot-1.16`，提交 `8714f2442ae63e47c9f23fd924a7106f09a9f634`，保留原 SITL 修改 |
| px4_msgs | tag `v1.16.2`，提交 `392e831c1f659429ca83902e66820d7094591410` |
| acados | 提交 `21376cb1af6b7dd45f675367272d3ba8100b26c0`，从源码重建，启用 qpOASES |
| Micro XRCE-DDS Agent | `v2.4.2`，提交 `57d086216d01ec43121845d385894a25987f8a2c`，原配置固定 Fast DDS v2.12.2 |
| Gazebo | OSRF 官方 Jammy 软件源的 Harmonic；Python 接口 `gz.transport13`、`gz.msgs10` |
| NumPy / SciPy | 虚拟环境固定 `1.26.4` / `1.11.4` |
| CasADi | `3.8.0`，沿用原环境版本；acados 对该版本有“尚未测试”提示，须结合实际求解验证 |
| 模板渲染工具 | `t_renderer 0.2.1`；官方下载超时后复用旧环境已存在的同架构独立工具，记录 SHA256 |

PX4 快照包含只适用于仿真的电池、传感器和失效保护参数修改。它们保留在 SITL 源码中，没有写入任何真机，不应作为真机参数模板。

迁移中的工程源码改动只有 `env.sh`（ROS 选择、虚拟环境）和 `package.xml`（补齐 NumPy/SciPy/YAML/launch 运行依赖）。备份源码逐文件比较结果见 `report/humble_migration_20261014/source_comparison.json`；原安全和算法代码保持一致。

## 构建与验证命令

```bash
source /opt/ros/humble/setup.bash
source ~/drone_package_20260908/venv/bin/activate
cd ~/drone_package_20260908/ros2_ws
python /usr/bin/colcon build --symlink-install --executor sequential --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
cd ~/payload_catch_ws
python /usr/bin/colcon build --symlink-install --executor sequential
source ~/drone_payload_catch/env.sh
cd ~/drone_payload_catch
bash tools/run_checks.sh
```

上述检查离线运行，不启动真机。完整套件中的 `offline_run --all` 会明确跳过部分垂直堆叠工况，不代表那些工况都已在新系统重新验收。

## 本机隔离验证

迁移验证使用 `ROS_DOMAIN_ID=97`、`ROS_LOCALHOST_ONLY=1`；Gazebo 限定 `GZ_IP=127.0.0.1`。测试不会自动生成起飞批准记录，也不运行真机启动脚本。

缺失数据测试在 Humble 中实际初始化 A/B、相对导航、载荷、安全监督与释放门禁节点，检查没有健康输入时不发出解锁、Offboard 或授权释放命令。测试结果和日志存放于同目录。

## 备份与回退

Windows 备份和安装脚本：

```text
C:\Users\caoli\humble_migration_20261014
```

`drone_payload_catch_full_backup.tar.gz` 是迁移前完整工程备份；`source_uncommitted.patch`、`source_git_status.txt` 记录未提交内容；`archive_sha256.txt` 记录迁移包校验值。完整安装、构建和验证日志也在该目录。

回到原环境只需：

```powershell
wsl -d Ubuntu-24.04 -u caolihao
```

不要删除原发行版或在旧工程上执行 reset/clean。新发行版是可独立保留的开发环境，不需要通过卸载旧环境完成迁移。

## 边界

本轮只安装和迁移电脑环境，没有 SSH 部署 Jetson，没有修改 Jetson 启动配置、PWM 或飞控参数。Jetson 实物波形、ARM64 构建、真实释放驱动、HIL 和真机飞行需要另行验证。`px4_msgs v1.16.2` 对应当前仿真基线；真机连接前仍须核对实际飞控固件，不能因 ROS 版本一致就认为消息兼容。

## 最终验收结果

- 完整离线检查：34/34 通过，包含 MPC 代码生成、编译、求解与离线场景。
- pytest：15/15 通过；colcon test：15 项、0 错误、0 失败、0 跳过。
- Humble 消息与工程从源码编译通过；6 个可执行节点可发现；3 个 launch 文件参数解析通过。
- 实际初始化 A/B、安全监督、释放门禁、相对导航及载荷节点，在 6 秒缺失健康输入测试中：0 飞控命令、0 解锁、0 Offboard、0 授权释放。
- Gazebo 8.15.0、gz.transport13 / gz.msgs10 Python 接口通过；无界面世界限时启动通过。
- `pip check` 无依赖冲突；`dpkg --audit` 无未完成软件包。
- 230 个既有文件比较确认：安全与算法源码未改；迁移修改为 env.sh/package.xml，README/MEMORY 增加新环境说明。

检查记录保存在 `report/humble_migration_20261014/`。`provisioning/` 保存本轮实际脚本供追溯，它们含本机路径和已有备份假设，不是任意电脑可直接运行的通用安装器。

本次没有执行完整双机空投捕获飞行，历史捕获率不能作为迁移后实测成绩。

构建过程说明：最初使用 `gz_x500` make 目标时，其在编译后自动进入仿真启动阶段，尽管设置了 DONT_RUN；已停止该次 PX4/Gazebo 进程，之后改用只编译的 CMake build 目标。该次没有运行飞行任务或人工发送解锁命令。后续正式遥测验收在仅有 loopback 的独立网络命名空间完成。重复 shell 提示的完整日志已压缩保存在 Windows 备份目录，精简日志保留构建和启动信息。

双机 SITL 遥测验证通过：在仅有 loopback 的临时网络命名空间运行 Gazebo、Agent 与两台 PX4，读取状态、定位和电池；两机均未解锁，位置有效、电量满足门槛。未启动 A/B 飞行任务或发布飞控命令。

```json
{
  "network_isolated": true,
  "command_publishers": 0,
  "flight_task_started": false,
  "aircraft": {
    "0": {
      "arming_state": 1,
      "pre_flight_checks_pass": true,
      "xy_valid": true,
      "z_valid": true,
      "battery_remaining": 0.9999465942382812
    },
    "1": {
      "arming_state": 1,
      "pre_flight_checks_pass": true,
      "xy_valid": true,
      "z_valid": true,
      "battery_remaining": 0.9998378753662109
    }
  }
}
```
