#!/usr/bin/env python3
"""B（接收方）节点：起飞 → 待命悬停 → 载荷释放后闭环会合 → 捕获。

流程：
  HOLD  : 悬停在待命点，等 /payload/released
  RENDEZ : 每 replan_dt 用 solve_inflight(载荷状态, B 状态) 重解会合参考；
           速度 setpoint = 参考速度前馈 + 位置 P 纠偏（限幅）
  DONE  : 捕获后悬停
捕获判据：|p_B−p_p| < r_c 且 |v_B−v_p| < v_c（世界系 NED）
"""
from __future__ import annotations

import numpy as np
import rclpy
from std_msgs.msg import Bool, Float64, Float64MultiArray

from .px4_iface import Px4Drone
from .rendezvous import RendezvousPlanner


class BNode(Px4Drone):
    def __init__(self):
        super().__init__('b_node', default_id=1)
        self.declare_parameter('standby_world', [1.2, 0.0, -3.5])
        self.declare_parameter('capture_radius', 0.30)
        self.declare_parameter('capture_rel_speed', 1.50)
        self.declare_parameter('replan_dt', 0.10)
        self.declare_parameter('inflight_tau_max', 2.0)
        self.declare_parameter('catch_alt_min', 0.8)
        self.declare_parameter('catch_alt_max', 2.8)
        self.declare_parameter('b_max_speed', 5.0)
        self.declare_parameter('b_max_accel', 6.0)
        self.declare_parameter('kp_pos', 1.2)
        self.declare_parameter('a_release_world', [0.0, 0.0, -3.0])   # A 的悬停/释放点(世界系)
        self.declare_parameter('start_delay', 18.0)                   # 起飞稳定后开始规划(s)
        self.declare_parameter('plan_tr_max', 3.0)

        self.standby = np.asarray(self.get_parameter('standby_world').value, float).reshape(3)
        self.r_c = float(self.get_parameter('capture_radius').value)
        self.v_c = float(self.get_parameter('capture_rel_speed').value)
        self.replan_dt = float(self.get_parameter('replan_dt').value)
        self.tau_max = float(self.get_parameter('inflight_tau_max').value)
        self.calt = (float(self.get_parameter('catch_alt_min').value),
                     float(self.get_parameter('catch_alt_max').value))
        self.v_max = float(self.get_parameter('b_max_speed').value)
        self.kp = float(self.get_parameter('kp_pos').value)

        self.a_release = np.asarray(self.get_parameter('a_release_world').value,
                                    float).reshape(3)
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.plan_tr_max = float(self.get_parameter('plan_tr_max').value)
        self._t_node0 = None
        self.planned = False
        self.plan = None
        self.pub_release_at = self.create_publisher(Float64, '/payload/release_at', 10)
        self.planner = RendezvousPlanner(
            g=9.81, b_max_speed=self.v_max,
            b_max_accel=float(self.get_parameter('b_max_accel').value),
            capture_radius=self.r_c, capture_rel_speed=self.v_c)

        self.p_pay = None
        self.v_pay = None
        self.released = False
        self.caught = False
        self.phase = 'HOLD'
        self.ref_t = None
        self.ref_p = None
        self.ref_v = None
        self.ref_t0 = 0.0
        self.last_replan = -1e9
        self.t_sim = 0.0

        self.create_subscription(Float64MultiArray, '/payload/state', self._on_payload, 10)
        self.create_subscription(Bool, '/payload/released', self._on_released, 10)
        self.pub_caught = self.create_publisher(Bool, '/payload/caught', 10)
        self.get_logger().info(f'b_node: standby={self.standby} offset={self.world_offset}')

    def _on_payload(self, msg):
        if len(msg.data) >= 7:
            self.p_pay = np.array([msg.data[1], msg.data[2], msg.data[3]])
            self.v_pay = np.array([msg.data[4], msg.data[5], msg.data[6]])
            self.t_sim = float(msg.data[0])

    def _on_released(self, msg):
        if msg.data and not self.released:
            self.released = True
            self.phase = 'RENDEZ'
            self.get_logger().warn('B: payload released → rendezvous')

    def _capture_check(self):
        if self.p_pay is None or self.caught:
            return
        d = float(np.linalg.norm(self.pos_world - self.p_pay))
        rv = float(np.linalg.norm(self.vel - self.v_pay))
        if d < self.r_c and rv < self.v_c:
            self.caught = True
            self.phase = 'DONE'
            self.pub_caught.publish(Bool(data=True))
            self.get_logger().warn(
                f'*** CAPTURED *** d={d:.3f}m rel_v={rv:.3f}m/s '
                f'p_B={self.pos_world.round(2)} p_p={self.p_pay.round(2)}')

    def control(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._tick_count % int(self.hz) == 0:
            pp = None if self.p_pay is None else self.p_pay.round(2)
            self.get_logger().info(
                f'B phase={self.phase} pos_w={self.pos_world.round(2)} vel={self.vel.round(2)} '
                f'pay={pp} caught={self.caught}')
        # ⚠️ 早退只能看 DONE；不能因 p_pay is None 早退——否则 HOLD 阶段的规划永远执行不到
        if self.phase == 'DONE':
            v = self.hover_velocity(self.standby[:2], -self.standby[2],
                                    kp_xy=1.0, max_speed=self.v_max, kp_z=1.0, max_climb=1.5)
            self.publish_velocity(v, yaw=self.yaw)
            return

        if self.phase == 'HOLD':
            # 起飞稳定 start_delay 后做一次规划，并立即开始执行 B 的会合参考（预位）
            if self._t_node0 is None:
                self._t_node0 = now
            if (not self.planned) and (now - self._t_node0) >= self.start_delay:
                plan = self.planner.solve(
                    self.a_release, self.pos_world, a_vel=(0.0, 0.0, 0.0),
                    b_v0=self.vel,
                    t_r_range=(1.0, self.plan_tr_max), tau_range=(0.05, 1.5),
                    t_r_step=0.05, tau_step=0.01, catch_alt_range=self.calt)
                self.planned = True
                if plan.feasible:
                    self.plan = plan
                    self.ref_t, self.ref_p, self.ref_v, _ = RendezvousPlanner.resample(plan, n=401)
                    self.ref_t0 = now
                    self.phase = 'RENDEZ'
                    self.pub_release_at.publish(Float64(data=now + plan.t_r))
                    self.get_logger().warn(
                        f'B: PLAN ok t_r={plan.t_r:.2f}s p_c={plan.p_c.round(2)} '
                        f'v_p={plan.v_p.round(2)} h_c={plan.h_c:.2f} → 广播释放时刻，开始预位')
                else:
                    self.get_logger().error(f'B: PLAN infeasible: {plan.reason}')
            v = self.hover_velocity(self.standby[:2], -self.standby[2],
                                    kp_xy=1.0, max_speed=self.v_max, kp_z=1.0, max_climb=1.5)
            self.publish_velocity(v, yaw=self.yaw)
            return

        # RENDEZ：闭环重规划 + 速度前馈 + 位置 P
        if self.p_pay is None:                       # 载荷状态还没来：先悬停
            v = self.hover_velocity(self.standby[:2], -self.standby[2],
                                    kp_xy=1.0, max_speed=self.v_max, kp_z=1.0, max_climb=1.5)
            self.publish_velocity(v, yaw=self.yaw)
            return
        if (now - self.last_replan) >= self.replan_dt:
            self.last_replan = now
            rp = self.planner.solve_inflight(
                self.p_pay, self.v_pay, self.pos_world, self.vel,
                tau_range=(0.05, self.tau_max), tau_step=0.01,
                catch_alt_range=self.calt)
            if rp.feasible:
                self.ref_t, self.ref_p, self.ref_v, _ = RendezvousPlanner.resample(rp, n=201)
                self.ref_t0 = now

        if (self.ref_t is not None) and (now - self.ref_t0) < float(self.ref_t[-1]):
            tl = max(now - self.ref_t0, 0.0)
            j = int(round(tl / float(self.ref_t[-1]) * (len(self.ref_t) - 1)))
            p_ref = self.ref_p[j]
            v_ref = self.ref_v[j].copy()
            v_sp = v_ref + self.kp * (p_ref - self.pos_world)
        else:
            # 参考执行完（或已过会合时刻）→ 在会合点悬停。
            # ⚠️ 不能继续用末点 v_ref(=载荷速度)：那会让 B 一直俯冲砸地。
            tgt = self.ref_p[-1] if self.ref_t is not None else self.standby
            v_sp = self.hover_velocity(tgt[:2], -tgt[2],
                                       kp_xy=1.5, max_speed=self.v_max, kp_z=1.5, max_climb=2.0)
        n = float(np.linalg.norm(v_sp[:2]))
        if n > self.v_max:
            v_sp[:2] *= self.v_max / n
        v_sp[2] = float(np.clip(v_sp[2], -self.v_max, self.v_max))
        self.publish_velocity(v_sp, yaw=self.yaw)
        self._capture_check()


def main(args=None):
    rclpy.init(args=args)
    node = BNode()
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
