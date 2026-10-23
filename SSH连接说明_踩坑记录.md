# SSH 连接 Jetson 机载电脑 —— 实测说明与踩坑记录

> 记录时间：2026-10 本次实测
> 目标机：NVIDIA Jetson（Tegra，Linux 5.15.148-tegra），主机名 `nvidia-desktop`
> 结论：**已连通并配好免密**，可长期使用。
>
> 相关脚本（本仓库 `tools/` 下）：
> - `tools/ssh_drone.sh` —— SSH 连接助手（连接/执行/对时/传文件/装公钥）
> - `tools/find_drone_eth.ps1` —— 有线直连时自动找 Jetson（需管理员）
> - `tools/deploy_to_jetson.sh` —— 把飞行相关路径部署到机载电脑（来源 drone_package）

---

## 一、最终可用的连接方式

### 免密 SSH（当前可用）
```bash
ssh nvidia@192.168.155.186            # WiFi
# 或
bash tools/ssh_drone.sh shell jetson
```

### 凭据
| 项 | 值 |
|----|----|
| 用户名 | `nvidia` |
| **默认密码** | **`nvidia`**（用户名=密码；实测可登录） |
| 私钥 | `~/.ssh/id_ed25519`（Windows：`C:\Users\caoli\.ssh\id_ed25519`） |
| sudo | **免密**（Jetson 上 `nvidia` 已配 sudo 免密、在 `dialout` 组） |

### 连接样式（文档里给的规范写法）
```bash
ssh nvidia@<IP> -i ~/.ssh/id_ed25519
```

---

## 二、这台 Jetson（A10）的网络地图（从机器里实测读出）

```
wlP1p1s0 (WiFi)         192.168.155.186/22     ← 本次用的链路，已配免密
enx00e04c680065 (USB网卡) 192.168.1.101/24       ← 文档里的 A10 mesh 地址（接 LQ 电台）
eno1                     10.41.10.100/24        ← 飞控点对点链路（接飞控）
```

- WiFi MAC：`dc:4a:9e:38:20:7f`（**用它认机**，IP 会变）
- `eno1` MAC：`3c:6d:66:d9:3a:59`；USB 网卡 MAC：`00:e0:4c:68:00:65`
- 路由默认网关：`192.168.152.1`（走 WiFi）
- 这台配置与文档里的 **A10（drone1）** 一致。

> 文档旧值 `192.168.155.176` 已过时 —— Jetson 走 DHCP，实测变成了 `.186`。
> 认机请用 MAC，不要认 IP。

---

## 三、踩坑清单（按踩到的顺序）

### 1. Windows 的 `ping -c` 是无效参数 → 误判"不通"
- **坑**：在 Git Bash 里用 `ping -c1 -W1 <ip>`，实际调的是 `C:\Windows\system32\ping.exe`，它**不认 `-c`**，直接报错 → 脚本/我以为是对端不通，得出错误结论。
- **正解**：Windows 语法用 **`-n`（次数）/ `-w`（超时毫秒）**：
  ```bash
  ping -n 1 -w 1500 <ip>
  ```
- **教训**：判断"能不能通"之前，先确认本机 `ping` 是哪一份、参数对不对。

### 2. 网卡被"锁死 10 Mbps" → 误以为是网线坏
- **坑**：有线一直只协商到 **10 Mbps**，第一反应是网线坏了一对线。实际是网卡高级属性里 **"速度和双工模式"被设成了 `10 Mbps 全双工`**（Realtek 驱动，`RegistryKeyword = *SpeedDuplex`，值 `2`）。
- **正解**（**管理员** PowerShell）：
  ```powershell
  Set-NetAdapterAdvancedProperty -InterfaceDescription "Realtek Gaming GbE Family Controller" `
      -RegistryKeyword "*SpeedDuplex" -RegistryValue 0     # 0 = 自动协商
  ```
  改完立刻协商到 **1 Gbps**。
- **教训**：链路速率异常先查网卡"速度和双工"，别急着换线。

### 3. Jetson 用静态 IP，对端没有 DHCP → 笔记本网口拿到 APIPA
- **坑**：直连后笔记本网口一直是 `169.254.x.x`（APIPA），**拿不到 DHCP**，于是找不到对端。
- **正解**：必须给笔记本网口**手动设同网段静态 IP**（见"有线连接"一节），
  **并且不设默认网关**（设了会抢走 WiFi 的默认路由、断网）。
- 需要**管理员权限**：
  - `New-NetIPAddress` / `Remove-NetIPAddress` / `Set-NetIPInterface` / `Set-NetAdapterAdvancedProperty` 都必须管理员。

### 4. 有线插错口：插到了飞控口 `eno1`，不是 mesh
- **坑**：把网线插在 Jetson 的 `eno1` 上，却把笔记本设成 mesh 网段 `192.168.1.123`，结果怎么都不通。
  因为 `eno1` 是**飞控点对点链路**（`10.41.10.100`），`192.168.1.101` 在另一块 **USB 网卡**上。
- **各口对应关系**（务必按机区分）：
  | 口 | 网段 | 用途 |
  |----|------|------|
  | `eno1` | `10.41.10.0/24`（A10）/ `192.168.77.0/24`（A08） | 飞控 |
  | USB 网卡 | `192.168.1.0/24` | mesh（接电台） |
  | WiFi | DHCP（本次 `.186`） | 日常 SSH |
- **教训**：插线前先确认插的是哪个口；`eno1` 接笔记本就等于把飞控断开了。

### 5. mesh 网段地址规则：别用 `.20x`
- 机载电脑一律 **`.10x`**（`.101/.102/.103`）；`.200/.202/.203` 是 **LQ-MESH 电台自己占的**，严禁使用。
- 冲突表现极具迷惑性：ping 两边都通，只有 TCP 被 RST，ARP 归属来回翻。
- **判据**：同一 IP 在不同机器上 `arp -a` / `ip neigh` 比 MAC 是否一致。
- 笔记本（地面站）mesh 地址惯例 `192.168.1.123`。

### 6. 该用 MAC 认机，不要认 IP
- WiFi 走 DHCP，Jetson 的 IP 从文档的 `.176` 变成了实测的 `.186`，以后还可能变。
- **认机 MAC**：`dc:4a:9e:38:20:7f`（WiFi）。
- 根治：路由器里按 MAC 绑定固定 IP。

### 7. Windows 自带 OpenSSH **没有 `ssh-copy-id`**
- Windows 的 `C:\Windows\System32\OpenSSH\` 里只有 `ssh/scp/ssh-keygen/...`，**没有 `ssh-copy-id`**。
- **正解**：用 **Git Bash**（`/usr/bin/ssh-copy-id` 存在），即 `bash tools/ssh_drone.sh init <IP>`。

### 8. 私钥权限过松会被拒绝
- 私钥 `~/.ssh/id_ed25519` 权限是 `-rw-r--r--`（644），OpenSSH 会警告 `UNPROTECTED PRIVATE KEY FILE`。
- **正解**：`chmod 600 ~/.ssh/id_ed25519`。

### 9. 无 `sshpass`/`expect` 时如何非交互喂密码
- 本机没有 `sshpass`、没有 `expect`、Python 也没有 `paramiko`。
- **正解**：用 OpenSSH（10.3+）自带的 `SSH_ASKPASS` + `SSH_ASKPASS_REQUIRE=force`：
  ```bash
  printf '#!/bin/sh\necho "nvidia"\n' > /tmp/askpass.sh && chmod +x /tmp/askpass.sh
  SSH_ASKPASS=/tmp/askpass.sh SSH_ASKPASS_REQUIRE=force DISPLAY=:0 \
    ssh -o NumberOfPasswordPrompts=1 -o PreferredAuthentications=password,keyboard-interactive \
        nvidia@192.168.155.186 "hostname"
  rm -f /tmp/askpass.sh        # 用完删掉，别留明文密码
  ```
- 注意：这是拿来**装公钥**的一次性手段；装好后就免密，不用再喂密码。

### 10. 非交互 SSH 不读 `~/.bashrc`（来自 drone_package 文档）
- `ssh host '命令'`、systemd、脚本都属于**非交互 shell**，不会 source `.bashrc`。
- 后果：ROS2 / acados 的环境变量（`RMW_IMPLEMENTATION`、`ROS_DOMAIN_ID`、`LD_LIBRARY_PATH`）不生效，
  `mpc_node` 会因找不到 `libhpipm.so` **静默崩溃**（进程还在但不干活）。
- **正解**：在远程命令里**显式 export**，或 `. ~/mesh_dds.env`。

### 11. `pkill -f` 经 ssh 内联会"自杀"（来自 drone_package 文档）
- 整段远程脚本是外层 shell 的 cmdline，`pkill -f` 的 pattern 若在脚本里再次出现，会把自己杀掉，
  表现为 `exit 255`、后半段静默不执行。
- **正解**：用方括号技巧 `pkill -f '[p]x4'`，且**"杀进程"和"起新任务/验证"拆成两条独立 ssh**。
- 另外：**SSH `exit 255` 不代表命令没执行**，要用幂等检查（`ps`/`ls`）确认真实状态。

### 12. 环境相关的小坑
- PowerShell 里 `$_` 在 Git Bash 双引号中会被 bash 展开 → 用**单引号**包 PowerShell 命令。
- 中文适配器名"以太网"在脚本里易受编码影响 → 用 `InterfaceDescription`（ASCII，如
  `"Realtek Gaming GbE Family Controller"`）或按 `MediaType -eq '802.3'` 匹配物理网口。
- 找物理网口时排除 `Hyper-V/Virtual/Wi-Fi/Loopback/Bluetooth`，否则会选到 `vEthernet (WSL)`。

### 13. Jetson 的 WiFi 偏弱、会抖
- 实测 ping 延迟在 **131ms → 6ms** 之间跳（功率管理/信号弱），文档也提过 WiFi 会断。
- **建议**：长期使用走有线（1 Gbps、稳定）；WiFi 只作应急。

### 14. 密码历史泄露，建议轮换
- 文档明确记载：这套设备的明文 SSH 密码曾随仓库 public 泄露过，`nvidia` 又是弱密码。
- **建议改密码**（Jetson sudo 免密）：
  ```bash
  ssh nvidia@192.168.155.186 "sudo passwd nvidia"
  ```
  改完免密依然有效（公钥没变）。

---

## 四、有线连接（如果想走网线，更稳）

**步骤（管理员 PowerShell）：**
```powershell
# 1) 先把网卡从"10Mbps全双工"改回自动协商
Set-NetAdapterAdvancedProperty -InterfaceDescription "Realtek Gaming GbE Family Controller" `
    -RegistryKeyword "*SpeedDuplex" -RegistryValue 0

# 2) 给网口设同网段静态 IP —— 取决于线插在 Jetson 哪个口：
#    插 eno1（飞控口）→ 用 10.41.10.x；插 USB 网卡（mesh）→ 用 192.168.1.x
New-NetIPAddress -InterfaceAlias "以太网" -IPAddress 10.41.10.50 -PrefixLength 24   # eno1 情形
# 或  New-NetIPAddress -InterfaceAlias "以太网" -IPAddress 192.168.1.123 -PrefixLength 24

# 3) 连接对应地址
#    eno1:  ssh nvidia@10.41.10.100
#    mesh:  ssh nvidia@192.168.1.101
```
> ⚠️ **不要设默认网关**，否则会抢走 WiFi 默认路由、断掉上网。
> ⚠️ 拿 `eno1` 接笔记本 = 飞控断开；要同时用需加交换机。

**一键自动找（推荐）：**
```powershell
powershell -ExecutionPolicy Bypass -File "\\wsl.localhost\Ubuntu-24.04\home\caolihao\drone_payload_catch\tools\find_drone_eth.ps1"
```
它会：先把网卡改回自动协商 → 依次试 `192.168.1.0/24`、`192.168.77.0/24`、`10.41.10.0/24`
三个网段（含已知地址、必要时整段扫描）→ 命中就保留 IP 并测 SSH。

**用完还原网口：**
```powershell
Set-NetIPInterface -InterfaceIndex <索引> -Dhcp Enabled
Remove-NetIPAddress -InterfaceIndex <索引> -AddressFamily IPv4 -Confirm:$false
```

---

## 五、命令速查

| 目的 | 命令 |
|------|------|
| 连接 | `ssh nvidia@192.168.155.186` |
| 免密测试 | `ssh -o BatchMode=yes nvidia@192.168.155.186 true` |
| 装公钥（一次性，要密码） | `bash tools/ssh_drone.sh init 192.168.155.186` |
| 远程执行 | `bash tools/ssh_drone.sh sh jetson "ip -br addr"` |
| 现场对时（Jetson 无 RTC，开机是 1970） | `bash tools/ssh_drone.sh sync-time jetson` |
| 传文件 | `bash tools/ssh_drone.sh copy 文件 jetson:/tmp/` |
| 看各网口 IP | `ssh nvidia@192.168.155.186 "ip -br addr"` |
| 看链路速率 | `ssh nvidia@192.168.155.186 "cat /sys/class/net/eno1/speed"` |

---

## 六、一句话总结

**用户名 `nvidia`、默认密码 `nvidia`；WiFi 实测 `192.168.155.186`（认 MAC `dc:4a:9e:38:20:7f`），
已配好免密。有线那次不通的真凶是"网卡被锁 10Mbps"+"插错到飞控口 eno1"，都不是网线问题。**
