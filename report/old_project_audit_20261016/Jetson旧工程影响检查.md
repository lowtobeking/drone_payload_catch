# Jetson旧工程影响检查（2026-10-16）

只读检查Jetson nvidia-desktop；未修改、停用或删除旧工程及服务。当前部署仍为fa46032caae12d28676da9014545e6b888a944572744ad205e50fcb0812c026a。

工程源码/构建在payload_catch_deploy独立版本目录内，本轮没有覆盖旧工程或更换系统ROS/Python包。此前机载隔离模拟通过，仅证明该隔离测试运行成功，不能证明真实运行与所有旧配置完全没有冲突。

## 当前发现

- mav-relay.service：enabled/active；启动/home/nvidia/Multi-UAV-simulation/tools/mav_relay.py 192.168.1.123，占UDP14550。
- microxrce-agent.service：enabled/active；MicroXRCEAgent udp4 -p 8888，为当前飞控ROS通信提供连接。不可重复启动相同端口Agent。
- 旧MAVLink中继为双向通信，能把地面站指令转给飞控；不是只读遥测。近期6条服务日志gcs->FC均为0，只能说明记录时段内未观察到由中继转入的地面站包，不是未来控制来源隔离证明。
- /home/nvidia/.bashrc两次设置ROS_DOMAIN_ID=42。显式加载ROS后的默认域0能发现飞控输入话题，输入发布者均为空。检查域42仅发现本次诊断节点，没有发现飞控话题或旧控制节点。因此后续正式启动应显式设定并验证与Agent一致的ROS域，不依赖交互终端默认值。
- 没有发现旧A/B任务、Offboard或PWM相关常驻进程；域0和42采样期间未发现飞控输入控制发布者。
- 没有运行中的Docker容器（普通用户无权限，sudo -n docker ps只读复核为空）。nvidia用户无crontab；扫描到的相关用户/系统服务、定时器、桌面启动项没有额外任务入口。
- PWM仍period20000000、duty0、enable0。

## 对当前工程的影响

静态存放旧工程不会直接改写新工程。实际注意共享通信服务和端口、双向中继允许的控制来源，以及ROS域与消息overlay。正式启动前应复用并核对已有Agent，明确MAVLink端口/地面站目标，确保旧控制程序不同时运行。新工程deploy/run_fleet.sh显式加载Humble、px4_msgs overlay和当前发布包，并加载network.env；仍需在正式组网时填写正确的网络配置。

本次不执行服务停用、重启、环境更改或真机任务。检查是当前状态快照，未穷举所有可能以其他名字运行的程序/其他用户启动项，不据此宣称永久无冲突。

证据：audit_domain0.json、audit_domain42.json、diagnostic_sources/jetson_old_project_audit.py；检查脚本已退出，远端临时文件已清理。
