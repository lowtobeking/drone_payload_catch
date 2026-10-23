# A 机动捕转换预览

> 2026-10-23 更新：已新增独立的 PX4 里程计地面桥接和诊断验证，默认禁用真实输出；当前仍未完成真实 EKF 融合。本页介绍的 mocap_preview 仍仅为 PoseStamped 预览。最新代码、实机间隙失败证据与待办见 [A机动捕PX4桥接与诊断](A机动捕PX4桥接与诊断_20261023.md)。

截至 2026-10-21，Ubuntu 22.04 工程已实现 mocap_transform、mocap_preview_node 和 config/mocap_a.preview.yaml，并已独立部署到 A 机完成真实数据预览及前、右、上、水平左转方向核对。当前仅为诊断预览，不创建 /fmu/in 发布者，没有解锁、飞行或 PWM 接口，未更改 PX4 参数。

完整完成项、待办与证据入口见 [2026-10-21 A 机动捕接入与实机验证进度](../2026-10-21_A机动捕接入与实机验证进度.md)。

## 当前变换候选

原始动捕 +Z 朝上。选定共享场地局部 FRD：local=[world_y, world_x, -world_z]。该局部方向不宣称地理北方，不应未经核实标成 NED。

机体 FRD 到动捕刚体候选关系：forward=-marker_y、right=-marker_x、down=-marker_z。完整姿态转换为 R_local_body=R_local_world*R_world_marker*R_marker_body；两侧都要转换，不能只改位置或给 yaw 减 90 度。

候选来源是手动测量。安装偏移、完整 roll/pitch 安装关系、标记原点到飞控/机体原点杠杆臂、精确距离尺度、服务器采样时序均未完成验收。配置保留 mounting_confirmed=false、lever_arm_measured=false、server_timing_verified=false；零杠杆臂是占位，不能据此投入飞行。

固定共享场地原点，不在每次启动或两机分别启动时自动归零。实际 PX4 两机估计坐标与共同场地参考的对应还需要分别核对。

## 预览使用

加载 Humble 环境后，从工程源码运行：

```bash
python3 -m payload_catch.mocap_preview_node --config config/mocap_a.preview.yaml
```

需要另行启动并验证 VRPN 客户端，输入 /vrpn_mocap/planeA/pose、frame_id=world。本节点不自动启动客户端、DDS Agent 或任务节点。构建后也可使用 mocap_preview ROS 入口。

现场 vrpn_mocap 1.1.0 自动发现旧 450_1 会因数字命名空间崩溃，其实际参数不支持 trackers；不要将指定 trackers 当作已验证解决办法。A 机预览使用仅显式订阅 planeA 的原生接收器与 ROS 源；工具与日志保存在 report/mocap_connection_20261019。该诊断链为限时运行，不是开机常驻桥接服务。

输出：

- /payload/mocap/a/preview_pose：geometry_msgs/PoseStamped，frame_id=payload_lab_frd。
- /payload/mocap/a/preview_status：预览计数、拒绝原因和失效状态。

输入及 PoseStamped 输出四元数顺序为 xyzw；未来 PX4 VehicleOdometry 的 wxyz 顺序必须单独处理。

消息逐帧处理，不定时重播旧位置。本地 ROS 时间戳缺失、重复、倒退、过期、未来异常或坐标/四元数无效时拒绝；ROS 时钟倒退锁存直到重启。输入 frame_id 必须匹配配置。

VRPN 客户端可能用本机接收时间替代服务器采样时间。本地时间戳检查不能证明服务器采样新鲜，也不能防止源冻结后客户端不断重发。实测源时间与 A 墙钟数值约差 12 小时，语义/同步仍待核实；接入飞控前必须核对源时间、重复数据、跟踪质量和失联行为。预览话题不是飞行健康批准。

## 已验证和未验证

10 项纯转换/时间门禁测试通过。隔离 ROS 域 191、localhost-only 检查通过：6 帧有效输出，重复、NaN、错误 frame 和过期 4 帧被拒绝，停止输入后状态过期，飞控输入发布者 0。

日志：report/mocap_connection_20261019/preview_unit_tests.log、preview_ros_check.log、preview_measured_conversion.json。验证进程均正常退出。

A 机独立预览部署路径：/home/nvidia/payload_catch_mocap_preview_20261021/9695526c87f980d0。18 秒采集原始 2139 / 转换 2125 条，未向 PX4 发送；同戳位置变换计算一致。详见 [A 机动捕预览运行记录](../report/mocap_connection_20261019/preview_on_a_20261021T091329Z/A机动捕预览运行记录.md)。

用户实际移动验证：前向 +0.9367 m（用户确认实际接近 1 m）、右向 +0.8782 m、抬高对应实验室 Z -0.2501 m、水平左转相对 yaw -86.31°。方向与实物动作相符，不代替精确安装/尺度/杠杆臂标定。详见 [planeA 实时方向验证](../report/mocap_connection_20261019/planeA实时方向验证_20261021.md)。

原预览节点仍不支持真实转发。2026-10-23 已另行实现默认禁用的 VehicleOdometry 地面桥接，并完成参数备份和诊断；后续先解决安装/原点、源时间/跟踪失效及实测接收间隙，再验收真实 PX4 输入与 EKF 融合。
