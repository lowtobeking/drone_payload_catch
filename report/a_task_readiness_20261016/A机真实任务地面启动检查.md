# A机机载真实任务节点地面启动检查

## 实际运行地点和范围

通过SSH在无人机A的Jetson（nvidia-desktop，aarch64）运行已部署工程里的ANode、SafetySupervisor、ReleaseGuard、PayloadServo，共20.02秒，机载进程PID=6807，发布版本13c1e49da415339aa236e19547c63a809de15e55235584917a700e964dd1702a。电脑只负责SSH、收集日志和生成报告。四个类的实际源文件位置已记录在ground_task.json。

本轮不是完整的catch_real_a_launch启动；正式入口仍因现场fleet配置未填写而不能通过。使用工程内诊断脚本导入真实已部署类，读取真实飞控遥测，没有合成B机、GPS、RC或反馈数据，没有修改任务源码或健康门限。

## 地面输出边界

显式auto_arm=false、require_fleet_link=true，使用不存在的起飞批准路径，GPS/RC检查保留。SafetySupervisor和ANode全部A/B飞控输入输出重映射到/payload_ground_check下的诊断话题，已核实实际publisher.topic_name。PWM采用真实配置的disabled/uncalibrated状态，不创建硬件worker。

监测真实飞控输入话题和重映射后的诊断话题：vehicle_command、offboard_control_mode、trajectory_setpoint、authorized_release消息全部为0。任务的_takeoff_ready和mission_allowed始终false。收到约46帧A机状态；最后3秒A的位置、姿态、状态、电池等缓存持续更新，RC为0帧，B各数据源为0帧。舵机状态1000次报告DISABLED，安全监督400次报告，无双机许可或释放许可。

全部节点已在finally中销毁并关闭执行器。独立结束检查finished.json确认没有剩余任务进程、没有飞控输入发布者，真实飞控未解锁。Jetson的通信基础服务继续运行。本轮未运动电机/舵机、未开启PWM、未创建任务自启服务。

## 正式任务缺项

- 双机现场配置未填写，正式launch门禁不通过。
- B机未连接，无双机通信租约与任务就绪。
- A机水平位置/速度无效，dead_reckoning=true，无有效GPS/RTK。
- 独立RC无数据。
- PX4自身pre_flight_checks_pass=false。
- 真实锁扣反馈未接入，PWM disabled/uncalibrated，重启后33脚GPIO状态。
- 完整双机参数验收与自检批准记录未生成。

## 其他发现

- 电池报告remaining约91%，voltage约16.44V但cell_count=1，来源/芯数需核实。
- 通用preflight_check要求models目录，fleet部署包未包含Gazebo模型，此为工具与部署范围差异，非真机传感器故障。

电池电量是飞控上报值，并非独立测量；芯数异常未通过改飞控参数掩盖。室内无卫星属于现场条件，不能通过改GPS阈值或伪造定位来补齐。B暂不在场不妨碍本次A机启动检查，但正式双机任务仍拒绝。

## 日志与复现

本机：ground_task.log/json、inputs.log/json、preflight.log/json、infrastructure.log/json、finished.log/json、validation.json。全部在Ubuntu22.04工程report/a_task_readiness_20261016中。

机载诊断目录：/home/nvidia/payload_catch_ownership_20261016/readiness。ground_task_result.json由Jetson写入；SSH捕获的ground_task.log、结束检查及此报告随后保存回该目录。脚本不作为飞行启动入口，也不生成飞行批准。

本项目辅助文件今后全部留在Ubuntu22.04工程内。此前归档的562个Windows工作资料已移入report/workfiles/windows_home_20261016，不再使用工程外workfiles目录。
