#!/usr/bin/env python3
"""A（投送方）节点：起飞 → 悬停在释放点。

M6 垂直堆叠模式下，A 额外把自身【世界系 NED】位姿广播到 `/drone_a/state`，
供 B 做**相对定位**（mesh/UWB 的仿真替身：B 订阅后加噪声/延迟）。

话题格式 Float64MultiArray:
    [t, N, E, D, vN, vE, vD]   （世界系 NED）
"""
from __future__ import annotations

import numpy as np
import rclpy
from std_msgs.msg import Bool, Float64MultiArray

from .px4_iface import Px4Drone


class ANode(Px4Drone):
    def __init__(self):
        super().__init__('a_node', default_id=0)
        self.declare_parameter('hover_world', [0.0, 0.0, -2.5])   # 世界系 NED
        self.declare_parameter('publish_state', True)
        self.declare_parameter('auto_land', False)          # 捕获后自动降落
        self.declare_parameter('land_after_catch_s', 6.0)   # 捕获后再悬停多久降落
        self.hover = np.asarray(self.get_parameter('hover_world').value, float).reshape(3)
        self.publish_state = bool(self.get_parameter('publish_state').value)
        self.auto_land = bool(self.get_parameter('auto_land').value)
        self.land_after_catch_s = float(self.get_parameter('land_after_catch_s').value)
        self._caught = False
        self._caught_t = None
        self.pub_state = self.create_publisher(Float64MultiArray, '/drone_a/state', 10)
        self.create_subscription(Bool, '/payload/caught', self._on_caught, 10)

    def _on_caught(self, msg):
        if msg.data and not self._caught:
            self._caught = True
            self._caught_t = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().warn('A: 收到 /payload/caught')

    def control(self):
        if self.auto_land and self._caught and not self._landing:
            if (self.get_clock().now().nanoseconds * 1e-9 - self._caught_t
                    >= self.land_after_catch_s):
                self.land()
        # 世界系目标（A 原点在世界 (0,0,0)，world_offset 默认 0）
        v = self.hover_velocity(self.hover[:2], -self.hover[2],
                                kp_xy=1.0, max_speed=1.0, kp_z=1.0, max_climb=1.0)
        self.publish_velocity(v, yaw=self.yaw)
        if self.publish_state:
            pw = self.pos_world
            m = Float64MultiArray()
            m.data = [float(self.get_clock().now().nanoseconds * 1e-9),
                      float(pw[0]), float(pw[1]), float(pw[2]),
                      float(self.vel[0]), float(self.vel[1]), float(self.vel[2])]
            self.pub_state.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
