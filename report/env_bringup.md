# 环境打通记录（B 阶段）

日期：2026-09-16 · 机器：WSL Ubuntu 24.04 · ROS 2 Jazzy · PX4 main (`cdecd90`) · Gazebo Sim 8.11

## 结论摘要

| 项 | 状态 |
|---|---|
| `px4_msgs` 版本匹配 + 编译 | ✅ 已修复（根因见下） |
| SITL 启动（gz + PX4 + MicroXRCEAgent） | ✅ 能启动，模型 `x500_0` 成功生成 |
| DDS 链路（ROS 2 ↔ PX4） | ✅ 通：`/fmu/out/vehicle_status_v4` @ 1.98 Hz，话题命名带版本后缀 |
| PX4 传感器桥（gz IMU → PX4） | ❌ **未通**：`ERROR [sensors] Gyro #0 fail: STALE!`，`vehicle_local_position_v1` / `vehicle_attitude` / `sensor_combined` 无数据 |

**一句话**：链路层（DDS）已通，卡在 **Gazebo 传感器数据进入 PX4 的桥接**，所以 EKF 起不来、位置/姿态话题无数据，无法进入 offboard。

---

## 1. 已修复：`px4_msgs` 版本不匹配

**现象**：`~/ros2_ws/src/px4_msgs` 在 `release/1.14`（2023），而 `~/PX4-Autopilot` 是 **main**（二进制内嵌 git 哈希 `cdecd90…`，`VehicleStatus.MESSAGE_VERSION=4`）。1.16/1.17 都是 `MESSAGE_VERSION=1`，只有 main 匹配。

**定位方法（可复用）**：拿 PX4 `HEAD` 的 `.msg` 与 px4_msgs 各候选提交的 `.msg` **逐字段比对**（注意：uORB 内部 struct 会按对齐重排字段，**不能**拿它比；DDS 序列化用 `.msg` 声明顺序）。

**结果**（263 个消息）：
- `ee2e90c`（"Update to PX4 6bfe6050", 8-25）→ **字段差异 0、缺失 0，完全匹配** ✅
- `14435c7`（8-28）起：0 差异但多 1 个 PX4 没有的消息
- `ed9ddee`（8-29）起：`SensorAccel/SensorBaro/SensorGyro/SensorMag` 等出现字段差异 ❌

**处理**：`px4_msgs` 切到本地分支 `main-cdecd90` = `ee2e90c`，`colcon build --packages-select px4_msgs`（4.5 分钟）。验证：
```
ros2 pkg prefix px4_msgs            # OK
OffboardControlMode fields: timestamp, position, velocity, acceleration,
                            attitude, body_rate, thrust_and_torque, direct_actuator
```
（与 PX4 编译产物 `uORB/topics/offboard_control_mode.h` 一致。）

> ⚠️ 切分支前 `px4_msgs` 是 `release/1.14`，与已安装 PX4 **根本无法通信**。这是本次 B 的关键修复。

---

## 2. DDS 链路验证（已通）

启动流程（PX4 main：先起 gz，再起 px4 挂模型）：
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
source ~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh
export PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=default
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp          # 必须：MicroXRCEAgent 是 Fast-DDS

python3 ~/PX4-Autopilot/Tools/simulation/gz/simulation-gazebo --headless \
    --model_store ~/PX4-Autopilot/Tools/simulation/gz --world default &
cd ~/PX4-Autopilot/build/px4_sitl_default && ./bin/px4 -d -i 0 &
MicroXRCEAgent udp4 -p 8888 &
```
验证结果：
- `gz model --list` → `ground_plane, x500_0` ✅
- `ros2 topic list | grep fmu` 有 `/fmu/in/*`、`/fmu/out/*` ✅
- `/fmu/out/vehicle_status_v4` ≈ 1.98 Hz ✅
- `/fmu/out/vehicle_local_position_v1` 存在但**无数据** ❌

---

## 3. 未解决：Gyro STALE（gz 传感器 → PX4）

**证据**
- gz 侧 IMU 正常：`/world/default/model/x500_0/link/base_link/sensor/imu_sensor/imu` 有发布者、`gz topic -e` 能取到 `seq=15695` 的消息，模型里 `imu_sensor` 为 `<always_on>1, update_rate=250`。
- `gz topic -i` 显示该话题**有 1 个订阅者**（应为 PX4 的 gz_bridge）。
- 但 PX4 日志 `ERROR [sensors] Gyro #0 fail: STALE!`，且 `sensor_combined`、`vehicle_attitude`、`vehicle_local_position_v1` 全程无数据。
- PX4 侧订阅字符串与 gz 话题**逐字一致**（`GZBridge.cpp:148`）：
  `/world/$WORLD/model/$MODEL/link/base_link/sensor/imu_sensor/imu`，日志 `[gz_bridge] world: default, model: x500_0`。

**已排除**
- 不是模型未生成（`x500_0` 在 gz 里）。
- 不是订阅失败（否则 PX4 会打印 `failed to subscribe to ...`；且 `gz topic -i` 有订阅者）。
- 不是话题名拼错（逐字比对一致）。
- 不是同一套 gz 版本不一致（server/插件/PX4 都链接 `/opt/ros/jazzy/opt/gz_*_vendor` 的 8.11）。

**主要嫌疑**
1. **lockstep 调度冲突**：本 PX4 编译含 `-DENABLE_LOCKSTEP_SCHEDULER`（`build.ninja` 确认）。日志出现 `[lockstep_scheduler] setting initial absolute time`，说明 clock 回调生效；但 IMU 回调依赖同一条 gz-transport 通道——若 lockstep 的步进服务在 vendor 版 gz 上不生效，会出现"clock 能到、传感器不更新"。变体实验（gz 不加 `-r`）反而卡在启动（缺 `Startup script returned successfully`），说明 lockstep 需要 gz 在跑，进一步指向此处。
2. **两套 Gazebo 并存**：系统装了 `libgz-sim8 8.13.0`（`/usr/lib`），ROS Jazzy 自带 `8.11.0`（`/opt/ros/jazzy/opt`）。本次运行都走 vendor，但两套共存本身就是隐患（`LD_LIBRARY_PATH`/`GZ_CONFIG_PATH` 任何一处漂移就会混用）。

**建议的下一步（择一）**
- **A（推荐先试，最可能直接解决）**：用 `nolockstep` 变体重建 PX4（仓库已有 `boards/px4/sitl/nolockstep.px4board`），再跑一次 smoke test。
  ```bash
  cd ~/PX4-Autopilot && make px4_sitl_nolockstep -j$(nproc)
  # 用 build/px4_sitl_nolockstep/bin/px4 重复第 2 节流程
  ```
- **B**：统一 Gazebo——决定用系统 8.13 还是 ROS vendor 8.11（建议按 PX4 官方 apt 安装的 Harmonic，走系统库），清掉另一套，重建 PX4 使其 `find_package` 命中同一套。
- **C**：换到你已经跑通过 Multi-UAV SITL 的那台 Ubuntu 仿真机做 M1（这台 WSL 的 PX4/脚本/ Gazebo 组合与参考项目不一致）。

---

## 4. 环境里发现的其他隐患（与 M1 相关，先记下）

1. `~/ros2_ws/src/mpc_control` 是**断链** → `/home/idt/Multi-UAV-simulation-full`（不存在）。参考项目的 `mpc_control` 源码需重新软链或复制进来才能 `colcon build`。
2. `config/scenarios.yaml` 默认 `px4_version: '1.16'`，但本机 PX4 是 **main**（`vehicle_status_v4`）。参考项目代码里有分支处理，但场景默认值需要改。
3. `~/PX4-Autopilot/msg/OffboardControlMode.msg` 工作区被改过（`bool actuator`），与 **已编译产物**（`thrust_and_torque`+`direct_actuator`）不一致。当前二进制用的是后者，所以能跑；**一旦重编 PX4 就会变**，需先决定要哪个并同步 px4_msgs。
4. 参考项目的 `start_N_px4.sh` / `spawn_px4.sh` 用的是 PX4 1.14 时代的 `PX4_GZ_STANDALONE` / `PX4_GZ_MODEL_POSE`，**本机 PX4 main 不支持**，不能直接复用，需改成"先起 gz、再 `px4 -i N`"。
5. `~/.simulation-gazebo` 模型库不存在；本次用 `--model_store ~/PX4-Autopilot/Tools/simulation/gz` 绕开，未触发下载。
6. `ACADOS_SOURCE_DIR` 未设（acados 在 `~/drone_package_20260908/acados`，`libacados.so`/`t_renderer` 齐全）。项目内 `env.sh` 已给出。

---

## 5. 复现命令（清理）

```bash
for p in 'bin/px4' 'gz sim' 'simulation-gazebo' 'MicroXRCEAgent'; do pkill -9 -f "$p"; done
```
> ⚠️ 不要把这些 `pkill` 与包含同样字符串的命令写在**同一条** bash 里——`pkill -f` 会把调用者自己杀掉（本次已踩两次）。

---

## 6. B-继续 的结果：lockstep / 系统 gz 均未解决，根因是 PX4 检出被本地大改

### 6.1 尝试一：nolockstep 变体重建（成功，但不含 Gazebo）

`make px4_sitl_nolockstep` 最终 **872/872 编译成功**（`build/px4_sitl_nolockstep/bin/px4`，`BUILD_EXIT=0`），
构建日志确认 `PX4 lockstep: disabled`。但 configure 报 `Could NOT find gz-transport`，
所以**该 build 不含 gz_bridge**（无 `rootfs/gz_env.sh`、参数里无 `SIM_GZ_*`），无法用于 gz SITL。

失败链（逐个修复，均为浅克隆/本地回退造成）：
| 障碍 | 处理 |
|---|---|
| `ZENOH_KCONFIG_TOPICS` 未设 → kconfig 解析 EISDIR | 指向默认 build 里已生成的 `Kconfig.topics` 当桩 |
| 浅克隆无 tag → `git describe --always` 回退成 hash，版本解析越界 | 在 HEAD 建本地 tag `v1.99.0`（**非真实版本**） |
| NuttX 子模块 `.git` 是坏桩 → `git tag` exit 128 | 改名 `.git` → `.git.disabled-broken`（SITL 不需要 NuttX） |
| 本地改动删了 `pxh.cpp` 的 `#include <cstdint>` | 恢复该 include（与 HEAD 一致） |
| GCC 13 `-Werror=array-bounds` 误报 | `cmake/px4_add_common_flags.cmake` 加 `-Wno-error` |
| `gz_bridge/CMakeLists.txt` 被回退成 2022 版，找 `gz-transport12`（系统只有 13） | **未修** → 就是它导致 gz 找不到 |

### 6.2 尝试二：用【系统 Gazebo 8.13】跑（编译期/运行时/服务器三者一致）

发现默认 `px4_sitl_default` 是**用系统 gz 8.13 编译**的（cache: `/usr/lib/.../cmake/gz-*`），
但运行时因 ROS 的 `LD_LIBRARY_PATH`/`GZ_CONFIG_PATH` 加载了 **vendor 8.11**。
用干净环境（`LD_LIBRARY_PATH` 只留系统路径、`PATH` 指向 `/usr/bin/gz` = 8.13）重跑：

```
[gz] /usr/bin/gz  Gazebo Sim 8.13.0
模型: ground_plane, x500_0 ✅
ERROR [sensors] Gyro #0 fail: STALE!   ← 仍然存在
/fmu/out/sensor_combined / vehicle_attitude / vehicle_local_position_v1 → 全部无数据
```

**结论：Gazebo 8.11/8.13 混用不是（唯一）根因**；lockstep 仍未被真正排除（nolockstep build 缺 gz，测不了）。

### 6.3 真正的拦路虎：这个 PX4 检出不是干净的 PX4

`git diff --stat src/modules/simulation` → **37 个文件、+1525/−2870**：整个仿真栈被本地回退/改造过
（`sensor_*_sim`、`simulator_mavlink`、`simulator_sih`、`gz_bridge/CMakeLists.txt`、`OffboardControlMode.msg`、`pxh.cpp` …）。
默认 build 能跑是因为它是在**更早的一致状态**下编好的，且依赖 ccache/已编译产物；
**任何干净重配都会撞上这些回退**。这解释了为什么 B 的每一步修复后面都还有下一个。

---

## 7. 本次对 PX4 仓库做的本地改动（如需可回滚）

| 文件/对象 | 改动 | 回滚 |
|---|---|---|
| tag `v1.99.0` | 在 HEAD 新建本地 tag | `git tag -d v1.99.0` |
| `platforms/nuttx/NuttX/nuttx/.git` | 改名 `.git.disabled-broken` | `mv .git.disabled-broken .git` |
| `platforms/.../pxh.cpp` | 加 `#include <cstdint>`（恢复 HEAD） | 删该行 |
| `cmake/px4_add_common_flags.cmake` | 加 `-Wno-error`（带注释，可整行删） | 删该行 |
| `build/px4_sitl_nolockstep/` | 新增构建目录（不含 gz） | `rm -rf` |
| `px4_msgs` 仓库 | 新建分支 `main-cdecd90` = `ee2e90c` | `git checkout release/1.14` |
| git tags | `git fetch --tags` 拉入 167 个 tag | 无需回滚 |

> 另注：`git fetch --tags` 触发了递归子模块抓取，报错 `Errors during submodule fetch: src/drivers/gps/devices`，并可能加重了子模块残缺。

---

## 8. 最终建议

1. **不要在这个检出上继续重编 PX4**。它的仿真栈被本地大改，干净重建需要先把那 37 个文件与
   HEAD 对齐（或换一份干净 PX4），这本身是个独立任务，且风险（改到你依赖的改动）不小。
2. **要验证 lockstep 假设**，最小路径是把 `gz_bridge/CMakeLists.txt` 恢复成 HEAD（找 gz-transport13）、
   重建 nolockstep（约 6 分钟），再跑 smoke test。但即便成功，也只是绕过一个被改坏的检出。
3. **首选：换到你已经跑通过 Multi-UAV SITL 的那台仿真机**做 M1；这台 WSL 更适合做
   离线算法（C）与代码开发。
4. 若必须在这台 WSL 上跑 SITL，建议单独排期做「环境修复」：确定用系统 gz 还是 ROS vendor gz、
   对齐 PX4 源码、修子模块、去掉两份 Gazebo 之一。

