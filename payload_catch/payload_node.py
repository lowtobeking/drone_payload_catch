#!/usr/bin/env python3
"""载荷状态源（M1：解析抛体 + 软件捕获，不放进 Gazebo）。

发布：
  /payload/state   Float64MultiArray  [t, px,py,pz, vx,vy,vz]   世界系 NED
  /payload/released Bool                                          是否已释放
  /payload/caught   Bool                                          是否已被捕获（由 capture 节点置位/回填）

M1 简化：释放点/初速/释放时刻由参数给定（A 悬停释放）。载荷按无阻力抛体积分。
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Float64MultiArray

from .payload_model import PayloadModel, PayloadParams


class PayloadNode(Node):
    def __init__(self):
        super().__init__('payload_node')
        self.declare_parameter('release_pos', [0.0, 0.0, -2.5])   # 世界系 NED
        self.declare_parameter('release_vel', [0.0, 0.0, 0.0])
        self.declare_parameter('release_delay', 15.0)             # 相对节点启动的时刻(s)
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('hz', 100.0)

        self.p_r = np.asarray(self.get_parameter('release_pos').value, float).reshape(3)
        self.v_r = np.asarray(self.get_parameter('release_vel').value, float).reshape(3)
        self.t_r = float(self.get_parameter('release_delay').value)
        self.hz = float(self.get_parameter('hz').value)
        self.dt = 1.0 / self.hz

        self.model = PayloadModel(PayloadParams(gravity=float(self.get_parameter('gravity').value)))
        self.t = 0.0
        self.caught = False

        self.pub_state = self.create_publisher(Float64MultiArray, '/payload/state', 10)
        self.pub_released = self.create_publisher(Bool, '/payload/released', 10)
        self.create_subscription(Bool, '/payload/caught', self._on_caught, 10)
        # 规划器给出的"绝对释放时刻"(ROS 秒)；收到后覆盖 release_delay
        self.create_subscription(Float64, '/payload/release_at', self._on_release_at, 10)
        self.abs_release_at = None
        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'payload_node: release at t={self.t_r:.1f}s from {self.p_r} v={self.v_r}')

    def _on_caught(self, msg):
        self.caught = bool(msg.data)

    def _on_release_at(self, msg):
        if self.abs_release_at is None:
            self.abs_release_at = float(msg.data)
            self.get_logger().warn(f'payload_node: 收到释放时刻 {self.abs_release_at:.3f}')

    def _tick(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        due = (now_s >= self.abs_release_at) if self.abs_release_at is not None else (self.t >= self.t_r)
        if (not self.model.released) and due:
            self.model.release(self.p_r, self.v_r)
            self.pub_released.publish(Bool(data=True))
            self.get_logger().warn(f'PAYLOAD RELEASED at t={self.t:.3f}s pos={self.p_r}')

        if self.model.released:
            p, v = self.model.pos.copy(), self.model.vel.copy()
            self.model.step(self.dt)
        else:
            p, v = self.p_r.copy(), self.v_r.copy()

        m = Float64MultiArray()
        m.data = [float(self.t), float(p[0]), float(p[1]), float(p[2]),
                  float(v[0]), float(v[1]), float(v[2])]
        self.pub_state.publish(m)
        self.t += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = PayloadNode()
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
