# A 机网络分配与 SSH 连接指南

> 更新：2026-10-26。依据本次实机网络、身份与飞控通信记录整理。
> Wi-Fi 地址是本次分配结果，不是永久固定地址；下次使用先确认地址。

## 1. 设备身份

| 项目 | 已核对信息 |
| --- | --- |
| 机载电脑 | Jetson，主机名 nvidia-desktop |
| 登录用户 | nvidia |
| Jetson 序列号 | 1612425621719 |
| Wi-Fi 网卡 MAC | dc:4a:9e:38:20:7f |
| 自带网口 MAC | 3c:6d:66:d9:3a:59 |
| USB 网卡 MAC | 00:e0:4c:68:00:65 |
| 电脑 SSH 私钥 | C:\Users\caoli\.ssh\id_ed25519 |
| 已保存的 SSH 主机密钥别名 | 192.168.155.186 |

主机密钥别名用于核对 A 机身份，仍连接命令末尾指定的当前 IP；它不是要求电脑或 A 机改回旧地址。不要因地址变化而删除旧主机密钥或关闭校验。

## 2. 当前网络分配

| 设备/接口 | 地址 | 获取方式/用途 |
| --- | --- | --- |
| 电脑 Wi-Fi | 192.168.1.4/22 | DHCP；SSID Aero-LAB_5G |
| A 机 Wi-Fi：wlP1p1s0 | 192.168.1.5/22 | DHCP；日常 SSH、实验室 Wi-Fi 通信 |
| 实验室 Wi-Fi 网关 | 192.168.1.1 | 本次默认网关 |
| 电脑有线网卡 | 192.168.1.123/24 | 既有静态设置；无默认网关、无 DNS |
| A 机自带网口：eno1 | 10.41.10.100/24 | 固定；平时连接飞控 |
| 飞控 | 10.41.10.2 | 本次实测可达并持续发送数据 |
| A 机 USB 网卡：enx00e04c680065 | 192.168.1.101/24 | 固定；连接 Mesh 数传电台 |
| 用户说明的数传电台 | 192.168.1.203 | 电台地址，不是 Jetson SSH 地址 |
| 动捕服务器 | 192.168.1.111:3883 | 既有 FZMotion VRPN 地址；刚体 planeA，本次未复测动捕 |
| A 机 docker0 | 172.17.0.1/16 | 本次 linkdown；不作为日常连接入口 |

/22 对应子网掩码 255.255.252.0；/24 对应 255.255.255.0。

电脑 Wi-Fi **保持自动获取 IP 和 DNS**。本次已核对 DHCP 开启、自动 DNS 正常，未保留先前试连用的电脑静态 .2。

## 3. 日常通过 Wi-Fi 连接（推荐）

1. A 机上电，等待 Jetson 启动完成。
2. 电脑和 A 机连接 Aero-LAB_5G。
3. 确认 A 当前仍为 .5 后，在电脑 PowerShell 中执行：

```powershell
ssh -o HostKeyAlias=192.168.155.186 -i C:\Users\caoli\.ssh\id_ed25519 nvidia@192.168.1.5
```

不需要把电脑 IP 改成固定值，也不需要电脑网线。退出使用：

```bash
exit
```

退出 SSH 只关闭当前终端连接，不会关闭 Jetson、飞控通信服务或下面的网络辅助服务。

### 为什么现在可以用 Wi-Fi

A 的 Wi-Fi /22 与 USB Mesh /24 网段重叠，原来回复电脑 .4 的流量会走错 Mesh 网口。现已新增：

- 服务：payload-catch-wifi-peer.service，已启用开机启动。
- 程序：/opt/payload_catch/wifi_ssh_20261026_v3/wifi_peer_route.py。
- 识别对象：本电脑 Wi-Fi MAC **98:5f:41:64:80:ed**。
- 工作方式：识别该网卡发来的 ARP，维护到它当前地址的一条 Wi-Fi /32 回程路由，持续刷新；仅清理自己维护的路由。
- 适用范围：A 的指定 Aero-LAB_5G 配置，以及这台电脑的上述网卡身份。MAC 仅用于识别地址，SSH 身份验证仍依靠主机密钥。

**同一 Wi-Fi 下，这台电脑已经实测可 SSH。其他电脑、更换网卡、启用随机 MAC、切换其他 Wi-Fi 时不能直接保证。** 两个网络的网段重叠仍存在，未全局重规划。

A 自己的 .5 也可能随 DHCP 改变。此服务解决电脑回程，不负责固定 A 的地址。

## 4. 地址变化时如何找 A

优先在 A 本机终端查看：

```bash
ip -br -4 address
hostname -I
```

看 wlP1p1s0 的 IPv4 地址；不要把 eno1 的飞控链路地址或 USB Mesh 地址当 Wi-Fi 地址。

电脑也可以查看已发现的邻居：

```powershell
Get-NetNeighbor -AddressFamily IPv4 | Where-Object LinkLayerAddress -eq 'DC-4A-9E-38-20-7F'
```

邻居表没有记录并不证明 A 未上电，记录也可能过期。可以通过路由器 DHCP 列表按该 MAC 查地址，或用下面的有线方式登录后查询。

不要照搬历史 .155.186、.1.4；它们曾属于 A，目前 .1.4 是电脑自身。

## 5. 经数传电台 / USB 网卡连接

电脑电台与 A 机电台正确组网、路由互通时，可尝试固定的 Jetson Mesh 地址：

```powershell
ssh -o HostKeyAlias=192.168.155.186 -i C:\Users\caoli\.ssh\id_ed25519 nvidia@192.168.1.101
```

此前成功的源地址是电脑有线 192.168.1.123/24 → Jetson .101。当前电台端到端 SSH **本轮没有重新验收**；QGC 连上不等于 SSH 端口一定可达。

.200/.202/.203 等电台地址不能分配给电脑或 Jetson，也不能用它们代替 Jetson .101 登录。电脑同时连有线与 Wi-Fi 时，两个网络重叠可能影响选路，需检查实际接口，不能只根据 ping 下结论。

## 6. 经 Jetson 自带网口救援

Jetson 自带网口 eno1 平时接飞控。只有地面未解锁、无任务运行时，才可临时改接电脑；拔原飞控网线会暂时中断 Jetson 与飞控之间的网络通信。

本次成功配置：

| 项目 | 临时救援设置 |
| --- | --- |
| 电脑有线口 | 额外增加 10.41.10.50/24 |
| 电脑有线默认网关 / DNS | 不设置，保留 Wi-Fi 上网 |
| A 自带网口 | 原有 10.41.10.100/24，不改 |
| 完成后的恢复 | 删除电脑临时 .50 地址，原飞控网线接回 eno1 |

连接命令：

```powershell
ssh -b 10.41.10.50 -o HostKeyAlias=192.168.155.186 -i C:\Users\caoli\.ssh\id_ed25519 nvidia@10.41.10.100
```

该命令要求电脑有线口已具有 .50 地址；本次结束后该临时地址已删除，不能直接运行就期待成功。再次配置需备份有线原设置，不能覆盖 Wi-Fi DHCP/DNS。

## 7. 飞控与既有工程状态

本次保持：

- Jetson eno1=10.41.10.100/24、飞控=10.41.10.2。
- MicroXRCEAgent：/opt/payload_catch/managed_20261016/bin/MicroXRCEAgent，UDP 8888。
- MAVLink 转发：/opt/payload_catch/managed_20261016/mavlink_relay.py，UDP 14550；本次读取的实际参数为 --fc-ip 10.41.10.2 --gcs-ip 192.168.1.123。
- .203 是电台地址；不要把它与转发程序中实际的地面站目标 .123 混为一项。本次没有修改 QGC 转发目标。
- 正式工程入口：/home/nvidia/payload_catch_deploy/current，仍指向既有 b4453dd… 发布；未部署或启动新飞行任务。
- 旧 microxrce-agent.service / mav-relay.service 是 masked 状态；实际运行的是既有 managed 路径程序，不要为了“恢复通信”擅自启动旧服务。

飞控网线接回后 ping 2/2，12 秒被动采集到 5844 个飞控 UDP 数据包，包含发送到 Agent 8888 端口的数据。原 Agent/relay PID 与启动时间不变；7 个原配置文件校验值一致，原主路由一致。

以上验证通信路径，不代表已完成定位融合、起飞自检或飞行许可。本次没有改飞控参数、发送解锁/飞行/PWM 指令、重启飞控/Jetson或原有通信服务。

## 8. 检查辅助服务与回退

在 A 机查看：

```bash
systemctl status payload-catch-wifi-peer.service --no-pager
journalctl -u payload-catch-wifi-peer.service -n 30 --no-pager
ip route get 192.168.1.4
```

最后一条需替换为电脑当前 Wi-Fi IP，正常应看到 wlP1p1s0。本次服务 active、enabled、NRestarts=0；已设置开机启动，但**未做整机重启复测**。真实电脑 DHCP 地址更换未刻意制造，离线测试已覆盖地址切换逻辑。

备份与精确回退入口：

```bash
sudo /usr/bin/python3 /home/nvidia/payload_catch_network_backup_20261026_v3/transaction.py rollback
```

回退会停止/禁用本次新增的辅助服务，删除经过哈希核对的新增文件和本服务持有的路由；可能立即断开依赖该路由的 Wi-Fi SSH。应通过 A 本机终端或救援有线连接执行。它不恢复/覆盖原 NetworkManager 配置，因为本次未修改那些配置。备份目录保留。

## 9. 工程记录位置

- 开发工程：WSL Ubuntu-22.04 的 /home/caolihao/drone_payload_catch。
- Windows 工程路径：\\wsl.localhost\Ubuntu-22.04\home\caolihao\drone_payload_catch。
- 详细修复记录：[A机WiFi_SSH修复与回退_20261026.md](docs/A机WiFi_SSH修复与回退_20261026.md)。
- 本轮源码、9项离线测试、事务脚本及原始验证日志：report/ssh_connection_20261026/。
- 后续工作先看 MEMORY.md。Ubuntu-24.04 副本不作为当前开发工程。
