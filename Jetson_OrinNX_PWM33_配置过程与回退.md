# Jetson Orin NX：物理 33 脚 PWM 配置过程与回退记录

记录日期：2026-10-13。本文记录本次实际操作、验证结果和恢复方法。

## 1. 结果和适用范围

已在这台 Jetson 上，通过 40 针排针的**物理 33 脚**输出硬件 PWM，驱动外部 5V 供电的 **GDW DS041MG** 舵机。用户确认舵机能够转动，最后一次慢速测试正常。

本次使用的是**运行期间的临时配置**。各次测试结束后均关闭 PWM、释放通道并恢复引脚原状态；最后一次测试结束时输出已关闭。没有配置开机自动启用 PWM，没有修改启动配置或刷机，也没有通过重启应用配置。

本次没有把无人机工程部署到 Jetson，没有执行无人机起飞、解锁或电机操作。舵机测试脚本与备份保存在 Jetson 上，本文件仅记录过程。

## 2. 设备与接线

| 项目 | 本机核实结果 |
| --- | --- |
| 模组 | Jetson Orin NX 8GB |
| 设备树兼容信息 | `nvidia,p3768-0000+p3767-0001`、`nvidia,p3767-0001`、`nvidia,tegra234` |
| 操作系统 | Ubuntu 22.04.5，ARM64 |
| Jetson Linux / L4T | 36.4.7 |
| 内核 | 5.15.148-tegra |
| ROS | ROS 2 Humble，ros-base |
| 舵机 | GDW DS041MG |
| 舵机供电 | 用户提供的外部 5V 电源 |
| 信号线 | Jetson 排针物理 33 脚 |
| 地线 | 外部电源、舵机与 Jetson 共地，用户已确认 |

“物理 33 脚”指排针位置编号，不是 GPIO 编号。Jetson 在本次接线中仅提供 PWM 信号，舵机不从 Jetson GPIO 取电，外部 5V 不接入信号引脚。已有可靠共地连接时，不要求再单独连接物理 34 脚。

连接舵机进行测试前，用户确认舵机已脱开机构或可以自由转动，不会被机械结构卡住。Jetson 信号为 3.3V 电平；本次舵机实际响应正常，但未找到该舵机输入高电平阈值的明确官方参数。

## 3. 为什么采用临时配置

先阅读了本机桌面的 `Jetson-PWM-开启指南.md`，随后按实际硬件重新核对。Nano 的寄存器配置不能直接用于 Orin；Jetson.GPIO 的 PWM 依赖硬件 PWM 控制器，不能假定任意 GPIO 都可输出硬件 PWM。

本机 Jetson-IO 存在预先存在的 DTB 匹配异常：

- 运行设备树识别为普通 Orin NX 开发套件。
- `/boot/dtb` 中可用文件为 `kernel_tegra234-p3768-0000+p3767-0001-nv-super.dtb`，其型号、兼容信息对应 Super 配置。
- Jetson-IO 报错：`No DTB found for NVIDIA Jetson Orin NX Engineering Reference Developer Kit!`。
- `/boot/extlinux/extlinux.conf` 默认仍为 `primary`；其中还有非默认的 JetsonIO 相机配置项引用 Super DTB。

这个异常影响通过 Jetson-IO 生成、应用配置的常规路径。本次没有修改 DTB、启动项或尝试修复这处异常，而是核对现有 PWM 控制器后，只临时修改目标引脚的复用状态，并安排退出恢复。

## 4. 引脚和控制器核对

结合设备内 Jetson.GPIO 引脚表、运行设备树、内核 pinctrl 调试信息和 PWM 控制器路径，核实如下：

| 排针物理编号 | GPIO 名称 | PWM 控制器地址 | 本次 sysfs 映射 |
| --- | --- | --- | --- |
| 15 | PN.01 | `3280000.pwm` | `pwmchip0/pwm0` |
| 32 | PG.06 | `32e0000.pwm` | `pwmchip3/pwm0` |
| **33** | **PH.00** | **`32c0000.pwm`** | **`pwmchip2/pwm0`** |

最终选择物理 33 脚。脚本按 `32c0000.pwm` 的实际设备路径查找控制器，不把 `pwmchip2` 编号写死。

散热风扇使用 `32a0000.pwm`，本次没有操作它。`39c0000.tachometer` 也不是本次排针 PWM 输出通道。

目标引脚起初未被其他 GPIO 用户占用，PWM 通道未导出。脚本在申请前重新检查占用情况，发现已有使用者时拒绝接管。

## 5. 修改前的备份

Jetson 上的备份和脚本目录：

```text
/home/nvidia/pwm_pin33_safety
```

目录由 root 管理，权限为 700。主要文件包括：

| 文件 | 用途 |
| --- | --- |
| `extlinux.conf`、DTB 备份、`running.dtb`、`running.dts` | 保存启动配置及运行设备树 |
| `manifest.json` | 备份来源及文件哈希，用于检查启动文件是否变化 |
| `pinconf-groups.txt`、`pinmux-pins.txt`、`pwm_before.txt` | 保存初始引脚和 PWM 状态 |
| `pin33_register_original.json` | 保存寄存器地址、原值及本次启动的 boot ID |
| `pin33_pwm_safe.py` | 状态检查、通道申请、临时配置及恢复工具 |
| `test_pwm33_isolated.sh`、`jetson_pwm_pin33_bounded_test.py` | 信号线断开时的限时输出测试 |
| `ds041mg_short_test.py` | 首次连接舵机的小范围测试 |
| `ds041mg_larger_test.py` | 18 秒较大范围测试 |
| `ds041mg_slow_test.py` | 最后一次 30 秒慢速测试 |
| 对应的 `*_result.json` | 保存各次测试的软件验证结果 |

Windows 侧还保存了初期备份：

```text
C:\Users\caoli\jetson_pwm_backups\pwm_pin33_safety.tar.gz
C:\Users\caoli\jetson_pwm_backups\pwm_pin33_safety_verified.tar.gz
```

这些 Windows 压缩备份早于连接舵机的后续测试，不能当作包含全部最新测试记录的备份。

本次核对的 `/boot/extlinux/extlinux.conf` SHA256：

```text
6d55aea184afb2f389d624e6a9c83fb280837164eda406c3e81035d84f19bc32
```

## 6. 临时切换 PWM 的具体过程

### 6.1 核实 PH0 寄存器

本机 PH0 引脚的 pad 寄存器地址为 **`0x02434040`**，即 pinmux 基址 `0x02430000` 加偏移 `0x4040`。

本次读取的原值为 **`0x54`**，对应原 GPIO 复用、输入使能、三态、下拉状态。临时 PWM 目标值为 **`0x405`**。

仅修改以下位，其他位保留原值：

| 位 | 配置含义 | PWM 目标 |
| --- | --- | --- |
| `[1:0]` | 功能复用 | GP 功能，索引 1 |
| `[4]` | tristate | 0，关闭三态 |
| `[6]` | enable-input | 0，关闭输入 |
| `[10]` | GPIO / SFIO 选择 | 1，选择外设功能 |

工具中的计算为：

```python
MASK = 3 | (1 << 4) | (1 << 6) | (1 << 10)
PWM_BITS = 1 | (1 << 10)
target = (original_value & ~MASK) | PWM_BITS
# 本次 original_value=0x54，target=0x405
```

写入后再次读取寄存器，并检查内核 pinconf 中 `soc_gpio21_ph0` 的状态：

```text
function=gp
tristate=0
enable-input=0
gpio-mode=1
```

以上地址和数值只适用于本次核实的设备与基线，不能作为其他 Jetson 板卡的通用写寄存器命令。

### 6.2 先准备关闭状态的控制器

`prepare` 执行以下步骤：

1. 核对主板、boot ID、启动配置哈希、寄存器原值和 PH.00 占用情况。
2. 确认目标 PWM 通道没有被其他程序导出。
3. 记录本工具的通道所有权，导出通道 0。
4. 设置周期 `20,000,000 ns`、占空时间 `0`，保持 `enable=0`。
5. 此阶段引脚仍保持原状态，不输出舵机控制脉冲。

连接舵机的测试再将关闭状态的控制器改为所需周期和中心脉宽，设置引脚复用，检查读回值后才启用输出。

### 6.3 回退保护

- 使用 `/run/pwm-pin33-config.lock`，避免本工具的并发操作。
- 测试脚本在 `finally` 中恢复；接收 SIGTERM、SIGHUP 后也进入恢复流程。
- 连接舵机的测试通过 systemd 临时服务运行，设置运行时限和独立 `ExecStopPost` 恢复命令。
- 最后一次慢速测试设置 `RuntimeMaxSec=45`、`TimeoutStopSec=3`。
- 信号线断开时的 `activate --servo-isolated` 路径另外安排 120 秒恢复计时器；该路径不能用于声称舵机已断开但实际仍连接的情况。
- 恢复顺序为：关闭输出、占空时间归零、恢复寄存器原值、释放本工具导出的 PWM 通道、删除临时所有权记录。

SSH 断开不等于立即停止舵机：临时服务可能继续到测试结束或超时，随后由 systemd 执行恢复。需要提前停止时，应停止对应服务，见第 9 节。

## 7. 舵机参数选择

GDW 官方规格图标注 DS041MG 工作电压 **4.8–8.4V**，控制参数为 **1520µs / 333Hz**。因此，在用户确认外部供电为 5V 后，连接舵机的测试采用：

```text
频率：约 333Hz
周期：3,003,003ns
中心高电平时间：1,520,000ns，即 1520µs
极性：normal
```

官网存在不同角度、连续旋转等版本选项，本次没有确定用户手中舵机的完整行程版本。因此没有直接测试完整脉宽范围，也没有把脉宽换算成确定角度。

最终慢速运动命令为：

```python
pulse_us = round(1520 + 120 * math.sin(2 * math.pi * elapsed / 15))
```

每约 50ms 更新一次，范围 1400–1640µs，每个完整往返周期约 15 秒，运行两个周期。范围比上一轮小，速度也更慢。软件范围本身不构成机械限位检测，实际动作由用户现场确认。

## 8. 已完成的测试与结果

| 阶段 | 条件与参数 | 结果 |
| --- | --- | --- |
| 控制器准备及手动恢复 | 输出关闭，引脚不切换 | 成功申请、配置、释放通道 |
| 独立恢复验证 | 3 秒 systemd 计时器 | 恢复程序成功运行 |
| 信号线断开输出测试 | 50Hz、1500µs，5 秒 | 软件状态确认启用，结束后恢复 |
| 首次接舵机测试 | 333Hz，1520→1440→1600→1520µs，约 2.7 秒保持时间 | 用户确认舵机转动，结束后恢复 |
| 较大范围测试 | 333Hz，1300–1740µs，6 秒一周期，3 周期，设定运动时间 18 秒 | 软件记录完成并恢复；用户反馈感受到的时间较短 |
| 最后慢速测试 | 333Hz，1400–1640µs，15 秒一周期，2 周期 | 程序实测运动时间 **30.003 秒**，用户确认“没问题” |

最后慢速测试日志：

```text
18:24:45 START: 333Hz, 1400..1640us, 15s per cycle, 30s sweep
18:25:16 SWEEP finished: actual=30.003s
18:25:17 RESTORED: original PH0 register 0x54, PWM channel free; boot unchanged
18:25:17 OUTPUT enabled-to-restored elapsed=31.714s
18:25:17 TEST COMPLETED AND RESTORED
```

输出开启到完成恢复约 31.714 秒，包含运动前后中心位置保持及恢复过程；systemd 报告服务总运行约 31.928 秒、退出码 0。运动时间用单调时钟测量，不靠观察日志时间戳估算。

恢复后核对：

- PH0 寄存器恢复为 `0x54`。
- PWM 通道未导出、输出关闭。
- 完整 `pinconf-groups` 与该次测试前的内容一致。
- 启动文件哈希未变化。
- 信号线断开测试还核对了其他 PWM 控制器未变化。

本次没有用示波器或逻辑分析仪测量排针波形。sysfs 读回、内核状态和舵机实际响应提供了软件及功能验证，但不代表频率、脉宽误差、电平及抖动已经仪器实测。

## 9. 查看状态、重复测试与提前停止

以下命令在 **Jetson 的终端或 SSH 会话内**执行。它们使用设备上已有的脚本；本文未把这些脚本复制进无人机工程。

### 查看当前状态

```bash
sudo python3 /home/nvidia/pwm_pin33_safety/pin33_pwm_safe.py status
```

本次测试结束后的预期输出包括：

```text
physical_pin=33, controller=32c0000.pwm, chip=/sys/class/pwm/pwmchip2
register=0x54, original=0x54
channel unexported
boot configuration unchanged
```

其中 chip 编号可能改变，应以脚本按控制器路径查到的值为准。

### 重复最后的慢速测试

先确认接线与供电仍正确、舵机可自由运动、通道空闲。下面命令会实际驱动舵机：

```bash
sudo systemd-run --wait --collect \
  --unit=ds041mg-slow-test \
  --property=RuntimeMaxSec=45 \
  --property=TimeoutStopSec=3 \
  --property='ExecStopPost=/usr/bin/python3 /home/nvidia/pwm_pin33_safety/pin33_pwm_safe.py restore' \
  /usr/bin/python3 /home/nvidia/pwm_pin33_safety/ds041mg_slow_test.py
```

运行结束后查看日志和状态：

```bash
sudo journalctl -u ds041mg-slow-test --no-pager -n 30
sudo python3 /home/nvidia/pwm_pin33_safety/pin33_pwm_safe.py status
sudo cat /home/nvidia/pwm_pin33_safety/ds041mg_slow_test_result.json
```

### 提前停止正在运行的慢速测试

在另一个 SSH 会话执行：

```bash
sudo systemctl stop ds041mg-slow-test.service
sudo python3 /home/nvidia/pwm_pin33_safety/pin33_pwm_safe.py status
```

停止服务会触发脚本退出恢复和 `ExecStopPost` 恢复。其他测试对应的服务名为 `ds041mg-short-test.service`、`ds041mg-larger-test.service`。

单独执行 `restore` 会等待配置锁；测试仍运行并持锁时，应先停止测试服务。

### 手动恢复本工具持有的临时配置

```bash
sudo python3 /home/nvidia/pwm_pin33_safety/pin33_pwm_safe.py restore
```

`restore` 只恢复本工具记录拥有的通道，遇到异常原值、其他所有者或启动配置变化时拒绝盲目覆盖。不要为了让命令通过而删除所有权记录或绕过检查。

关闭 PWM 意味着停止控制脉冲，**不等于切断舵机电源或保证舵机立即失去保持力**；无信号行为取决于舵机。如果出现卡住、异常响声等情况，需要现场人员断开舵机外部电源，不能只依赖网络停止。

## 10. 重启与后续使用

本次引脚复用没有写进启动配置，重新上电按原配置初始化。工具保存了本次 boot ID，设备重启后会拒绝沿用旧基线；届时必须重新检查硬件、占用情况、寄存器原值及启动文件，并更新基线。不能通过强行改 boot ID 来跳过核实。

这套流程适合本次有限时长的验证。后续如果需要长期使用或纳入工程，应另行完成启动配置方案、单一控制进程管理、停止策略及实物波形测量；本文没有实施这些持久化改动。

## 11. 核对来源

- 用户提供的指南：`C:\Users\caoli\Desktop\Jetson-PWM-开启指南.md`。
- 设备内引脚表：`/usr/lib/python3/dist-packages/Jetson/GPIO/gpio_pin_data.py`。
- 设备内运行设备树、`/sys/class/pwm`、`/sys/kernel/debug/pinctrl/2430000.pinmux/pinconf-groups`、GPIO 占用及 PWM 调试状态。
- [Linux 6.8 Tegra234 pinctrl 驱动](https://raw.githubusercontent.com/torvalds/linux/v6.8/drivers/pinctrl/tegra/pinctrl-tegra234.c)。
- [OE4T L4T 36.2 Tegra234 pinctrl 驱动](https://raw.githubusercontent.com/OE4T/linux-jammy-nvidia-tegra/oe4t-patches-l4t-r36.2-1018.18/drivers/pinctrl/tegra/pinctrl-tegra234.c)。
- [GDW DS041MG 官方产品页面](https://gdwservo.com/product/gdw-ds041mg-5kg-torque-metal-gear-micro-mini-digital-servo-for-450-helicopter-fix-wing-rc-auto-robot-arm/)。
- [GDW DS041MG 官方规格图](https://gdwservo.com/wp-content/uploads/2022/10/DS041MGaplus2.jpg)。

公开驱动代码用于交叉核对，没有安装到 Jetson。本次没有核实完全对应 L4T 36.4.7 的源码；36.4.4 分支下载超时，未计入核对依据。访问 GitHub 是为读取驱动参考资料，没有登录账号。
