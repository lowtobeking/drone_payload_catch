# A机室内无桨部署记录

2026-10-15，用户明确授权在A机部署以便室内无桨验收。主机nvidia@192.168.155.186（nvidia-desktop），Orin NX 8GB、Ubuntu22.04.5、ROS2 Humble、aarch64。

- 当前版本：`e27cb2caa69a0ff8fc9cf4d43adbcc3a40419dc07c803536187a1a3608a9d9cb`
- 入口目录：`/home/nvidia/payload_catch_deploy/current`
- 角色文件：`/home/nvidia/payload_catch_deploy/ROLE`，内容a。
- 独立构建：当前版本下ws/install，现有/home/nvidia/ros2_ws未覆盖。
- 源码哈希校验及目标ARM构建通过，消息源码提交392e831c1f659429ca83902e66820d7094591410与电脑基线一致。
- 原生32项释放测试、24项安全运行、10项起飞门禁，以及启动参数、ROS模拟释放/动作前中急停通过。
- 独立网络namespace内实例化A节点/释放guard/禁用舵机，5秒250个状态样本；监听器QoS匹配且发布者确认有订阅者，vehicle_command/offboard/setpoint/authorized_release全部0。网络空间只影响测试子进程，系统路由/现有Agent不变。
- 33脚PWM保持20ms，duty=0/enable=0；启动DTB和extlinux.conf哈希前后相同。无新服务、无开机项、无真实释放、无飞控参数修改。测试已结束，无任务常驻。

真实PWM仍enabled=false/calibrated=false。直出1700µs、1.5秒已写入配置，用户新测量记录operator_direct；尚无B机、最终定位网络、真实锁扣/脱离反馈和完整时序验收，不能起飞。此次为安装及隔离软件效果验证，没有舵机动作或实机双机协调。

## 查看与重复验收

普通用户加载环境：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
source /home/nvidia/payload_catch_deploy/current/ws/install/setup.bash
ros2 pkg executables payload_catch
```

再次运行同样的无输出隔离验收（先确认地面停机）：

```bash
sudo -n unshare --net -- bash /home/nvidia/payload_catch_upload_e27cb2caa69a/namespace_validate.sh
```

此脚本只在新网络空间启用lo并以nvidia普通用户测试，不使用真实PWM，不启动MicroXRCEAgent或发送真实飞控流量。

## 回退

本次是首次正式部署，没有previous正式版本。已保留版本化目录及中间未激活的构建记录。停任务后撤销当前选择：

```bash
python3 /home/nvidia/payload_catch_deploy/unselect_a.py
```

脚本持任务/操作锁，只删除指向本次版本的current符号链接，不删任何源码/构建/日志，不修改引脚和系统。后续有多个正式版本时用fleet_deploy.py rollback选择previous。恢复系统不需要重装，因为部署未改变系统软件或启动设置。

证据：build.log、native_validation.log、activation.log、deployment_result.json。远端日志在/home/nvidia/payload_catch_upload_e27cb2caa69a；电脑副本C:/Users/caoli/a_deployment_20261015。
