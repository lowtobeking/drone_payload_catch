# A机单机版部署记录

2026-10-19，按用户要求仅部署，不运行。

- 设备：nvidia@192.168.155.186，nvidia-desktop，ARM64 / Ubuntu22.04 / ROS2 Humble。
- 当前目录：/home/nvidia/payload_catch_deploy/current。
- 当前版本：b4453dd12c8482d808613769fb6c781bd7debff566633bf0fc4827a6d2654d37。
- 回退版本：13c1e49da415339aa236e19547c63a809de15e55235584917a700e964dd1702a。
- 配置：/home/nvidia/payload_catch_deploy/a_solo.yaml，现场确认字段仍为false。

ARM编译通过。文件完整性、版本链接和部署前后只读检查通过。无任务或测试进程残留；启动配置、通信服务、主机运行配置及PWM状态未改变。未运行任务或执行器，未添加开机自启。

单机入口用于A机自检、起飞1m、稳定悬停10秒、降落上锁验收，不含B机协调或舵机投放。完整工程其他入口保留，未启动。

此前26项单机逻辑、7项参数协议、10项起飞门禁及飞控参数测试通过。机载隔离ROS测试因新建日志目录权限失败，尚未通过；本次未重新运行，未生成验收通过标记。本部署记录不能替代实机飞行验收。

## 回退

在A机SSH终端执行下列命令，仅恢复版本链接，不运行任务：

```bash
/usr/bin/python3 /home/nvidia/payload_catch_a_solo_20261019/select_files_only.py b4453dd12c8482d808613769fb6c781bd7debff566633bf0fc4827a6d2654d37 --rollback
```

部署前指针保存在A机 /home/nvidia/payload_catch_a_solo_20261019/deploy_only_before.json。旧版代码及构建产物均保留。

## 证据

- deployment_status.json：最终部署状态。
- deploy_only_before_probe.json / deploy_only_after_probe.json：前后只读检查。
- select_only.stdout.log / select_only.stderr.log：远端切换结果。
- build.stdout.log / build.stderr.log：机载编译日志。
- onboard_validation.stdout.log / onboard_validation.stderr.log：此前验收失败记录。
