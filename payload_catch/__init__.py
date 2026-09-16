#!/usr/bin/env python3
"""payload_catch —— 空投—空中捕获（A 抛投、B 接住）的多无人机任务。

分层（刻意让核心算法层不依赖 ROS，便于离线验证与移植）：
  · payload_model.py  载荷抛体动力学（纯 Python）
  · rendezvous.py     时空会合协调求解（纯 Python）
  · sim_core.py       离线闭环仿真（纯 Python）
  · *_node.py         ROS 2 节点（后续接入 PX4 SITL）

坐标：世界系 NED（x=北, y=东, z=下），高度(离地) = -z。
"""

__version__ = '0.1.0'
