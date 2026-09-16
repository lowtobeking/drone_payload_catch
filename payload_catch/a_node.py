#!/usr/bin/env python3
"""A（投送方）节点：起飞 → 悬停在释放点。载荷由 payload_node 解析模拟（M1 无真实挂载）。"""
from __future__ import annotations

import numpy as np
import rclpy

from .px4_iface import Px4Drone


class ANode(Px4Drone):
    def __init__(self):
        super().__init__('a_node', default_id=0)
        self.declare_parameter('hover_world', [0.0, 0.0, -2.5])   # 世界系 NED
        self.hover = np.asarray(self.get_parameter('hover_world').value, float).reshape(3)

    def control(self):
        # 世界系目标（A 原点在世界 (0,0,0)，world_offset 默认 0）
        v = self.hover_velocity(self.hover[:2], -self.hover[2],
                                kp_xy=1.0, max_speed=1.0, kp_z=1.0, max_climb=1.0)
        self.publish_velocity(v, yaw=self.yaw)


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
