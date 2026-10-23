# A机由payload_catch工程独立管理

2026-10-16起，A机运行环境由当前工程管理。此前直接引用旧工程中继、共享消息工作区的方案已被替换。旧工程文件保留用于回退，不作为当前运行依赖。

| 组件 | 当前管理位置 |
| --- | --- |
| DDS Agent | payload-catch-agent.service；/opt/payload_catch/managed_20261016/bin/MicroXRCEAgent |
| Agent依赖 | 同一受保护目录下lib；从已验证版本独立复制并记录指纹 |
| QGC中继 | 本工程tools/mavlink_relay.py；安装至受保护runtime目录；payload-catch-mav-relay.service |
| PX4消息 | 同一runtime目录下px4_msgs_ws，固定392e831c1f659429ca83902e66820d7094591410，自行构建，无旧工作区环境链 |
| ROS与网络 | 系统Humble、Fast DDS、域0、eno1=10.41.10.100、FC=10.41.10.2、Mesh=192.168.1.101 |

旧microxrce-agent.service和mav-relay.service停用并mask，原unit文件已备份。新通信服务开机自启；飞行任务和PWM输出没有自启。消息、Agent和中继文件由root持有且不可由普通用户修改。

中继只允许配置的FC和GCS地址转发，拒绝其他来源；取消原中继任意学习地面站IP的行为。默认GCS仍为192.168.1.123，实际QGC连接尚需核对。中继只检查帧起始字节，MAVLink内容校验与命令接受仍由飞控承担，不能视为完整网络身份认证。

runtime_env.sh先清理旧Python/ROS/虚拟环境路径，再加载系统Humble及当前独立消息工作区。交互shell原域42设置已停用，显式加载当前工程环境；不自动运行任务。

## 检查

```bash
bash /home/nvidia/payload_catch_deploy/current/src/deploy/check_jetson.sh
```

该只读命令检查当前服务和端口归属、消息版本、受保护运行文件和Agent库指纹、旧服务必须inactive/masked、真实飞控身份/未解锁/新鲜数据，以及ROS控制发布者冲突。通过不等于获得起飞批准。原双机自检、每台至少40%电量、定位、RC、几何标定及释放反馈要求保留。

## 迁移和回退

deploy/manage_ownership.py记录并保存旧服务文件、启用状态、当前/previous发布链接、原host/runtime.yaml和.bashrc。切换使用180秒独立systemd回退定时器；通信验证并提交前，失败或中断会自动恢复原状态。提交后取消定时器，保留备份。

需要人工回退时，确认地面、未解锁且无任务运行，再执行：

```bash
sudo -n python3 /opt/payload_catch/managed_20261016/manage_ownership.py restore --ground-confirmed
```

脚本还会订阅真实状态确认落地和未解锁，并持部署/任务锁；只恢复记录的服务、发布链接及配置，不删除工程或系统。回退会重新启用旧通信服务，等于撤销本轮独立接管，应明确用于故障恢复。

这隔离了旧工程的目录依赖、默认环境和已知启动入口。若以后人为以root权限重新启用旧工程或在同一飞控网络运行其他控制程序，仍需按冲突检查处理。

中继采用非阻塞UDP；地面站离线或发送队列拥塞时丢弃当前包并记录send_dropped，不重试或延后重放控制包。网络恢复后继续处理新包。
