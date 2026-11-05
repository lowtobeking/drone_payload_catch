# A 机动作链只读门禁核对（2026-11-05）

- SSH：数控电台地址 `192.168.1.5` 可连接，使用既有 HostKeyAlias。
- A 机当前 boot ID：`f0caa4d5-3b51-43f1-9e01-d677132ee2cb`。
- `payload-catch-mocap-a.service`：`active`（仅服务状态，未证明连续跟踪）。
- 受保护 PWM pin33 基线 boot ID：`64244966-0f68-4732-b814-1da1248c28e9`。
- 两个 boot ID 不一致。现有 `JetsonPWM.verify()` 会拒绝真实输出；需在地面按寄存器、GPIO、PWM export、boot 文件和任务占用逐项重验，不能只改记录的 boot ID。
- 本轮没有下发飞控命令、没有输出 PWM、没有在本机构建。
- 本地 A-only 任务几何和纯逻辑状态机仍未接入 ROS/PX4/PWM，不能当作可执行动作链。
