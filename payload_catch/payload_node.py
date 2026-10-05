#!/usr/bin/env python3
"""载荷状态源（M5-1：Gazebo 真实物理下落）。

流程：
  1) 到释放时刻，用 gz `EntityFactory` 的 `/world/<world>/create` 服务在 A 的位置生成
     `payload` 模型（真实物理：重力/碰撞/接触）。
  2) 订阅该模型的 odometry（`/payload/odom`，含位置+速度，ENU），转成世界系 NED，
     发布 `/payload/state`。
  3) 捕获判据仍由 b_node 做（M5 第二步再做真实接触/吸附）。

坐标：Gazebo ENU → 世界系 NED：NED=[ENU_y, ENU_x, -ENU_z]（速度同理）。
use_gazebo=false 时退回解析抛体（旧行为，便于离线/对照）。
"""
from __future__ import annotations

import os

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Float64MultiArray

from .payload_model import PayloadModel, PayloadParams

try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.boolean_pb2 import Boolean
    _HAS_GZ = True
except Exception:                                    # 无 gz 绑定时退回解析
    _HAS_GZ = False


def enu_to_ned_pos(p):
    return np.array([p.y, p.x, -p.z])


def enu_to_ned_vec(v):
    return np.array([v.y, v.x, -v.z])


def ned_to_enu_pos(p):
    return (float(p[1]), float(p[0]), float(-p[2]))     # (x_east, y_north, z_up)


class PayloadNode(Node):
    def __init__(self):
        super().__init__('payload_node')
        self.declare_parameter('release_pos', [0.0, 0.0, -3.0])   # 世界系 NED
        self.declare_parameter('release_vel', [0.0, 0.0, 0.0])
        self.declare_parameter('release_delay', 30.0)
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('hz', 100.0)
        self.declare_parameter('use_gazebo', True)
        self.declare_parameter('world', 'default')
        self.declare_parameter('model_path',
                               os.path.expanduser('~/drone_payload_catch/models/payload/model.sdf'))
        self.declare_parameter('odom_topic', '/payload/odom')
        self.declare_parameter('release_offset', [0.0, 0.0, 0.0])   # 相对 A 的释放点偏移(NED)
        self.declare_parameter('release_xy_sigma', 0.0)             # 释放点水平误差 (m)
        self.declare_parameter('release_seed', 0)
        self.declare_parameter('use_a_state', False)                # 用 A 实际位姿作释放点
        self.declare_parameter('a_state_topic', '/drone_a/state')
        # ── M6-moving：编队同速投放（载荷挂载到 A，分离时继承 A 速度）──
        self.declare_parameter('attach_to_a', False)                # 是否挂载到 A（DetachableJoint）
        self.declare_parameter('formation_topic', '/formation/start')  # 收到后挂载到 A
        self.declare_parameter('attach_topic', '/payload/attach')
        self.declare_parameter('detach_topic', '/payload/detach')
        # ── 主动保持（B 侧锁扣）：捕获后把载荷锁到 B 的漏斗 link ──
        self.declare_parameter('lock_to_b', False)
        self.declare_parameter('lock_request_topic', '/payload/lock_request')
        self.declare_parameter('lock_model_path', '')
        self.declare_parameter('lock_topic', '/payload/lock')
        self.declare_parameter('unlock_topic', '/payload/unlock')

        self.p_r = np.asarray(self.get_parameter('release_pos').value, float).reshape(3)
        self.v_r = np.asarray(self.get_parameter('release_vel').value, float).reshape(3)
        self.t_r = float(self.get_parameter('release_delay').value)
        self.hz = float(self.get_parameter('hz').value)
        self.dt = 1.0 / self.hz
        self.world = str(self.get_parameter('world').value)
        self.model_path = str(self.get_parameter('model_path').value)
        self.use_gz = bool(self.get_parameter('use_gazebo').value) and _HAS_GZ
        self.release_offset = np.asarray(
            self.get_parameter('release_offset').value, float).reshape(3)
        self.release_xy_sigma = float(self.get_parameter('release_xy_sigma').value)
        self._rng = np.random.default_rng(int(self.get_parameter('release_seed').value))
        self.use_a_state = bool(self.get_parameter('use_a_state').value)
        self._a_world = None
        self.attach_to_a = bool(self.get_parameter('attach_to_a').value)
        self._attached = False
        self._attach_pub = None
        self._detach_pub = None
        self._empty_cls = None
        self.lock_to_b = bool(self.get_parameter('lock_to_b').value)
        self.lock_model_path = str(self.get_parameter('lock_model_path').value)
        self._locked = False
        self._lock_pub = None
        self._unlock_pub = None

        # 解析兜底
        self.model = PayloadModel(PayloadParams(gravity=float(self.get_parameter('gravity').value)))
        self.t = 0.0
        self.abs_release_at = None
        self.caught = False
        self.spawned = False
        # Gazebo 观测
        self._gz_p = None
        self._gz_v = None
        self._gz_t = None
        self._gz = None

        if self.use_gz:
            self._gz = GzNode()
            # 用 SceneBroadcaster 的全实体位姿（世界 ENU，最可靠）；速度用有限差分
            self._gz.subscribe(Odometry, str(self.get_parameter('odom_topic').value), self._on_odom)
            self.get_logger().info(
                f'payload: Gazebo 模式，pose=/world/{self.world}/pose/info model={self.model_path}')
            import os as _os
            self.get_logger().info(f'payload: model 存在? {_os.path.exists(self.model_path)}')
        else:
            self.get_logger().warn('payload: 解析模式（Gazebo 关闭/不可用）')

        self.pub_state = self.create_publisher(Float64MultiArray, '/payload/state', 10)
        self.pub_released = self.create_publisher(Bool, '/payload/released', 10)
        self.create_subscription(Bool, '/payload/caught', self._on_caught, 10)
        self.create_subscription(Float64, '/payload/release_at', self._on_release_at, 10)
        self.create_subscription(Bool, '/payload/release_abort', self._on_release_abort, 10)
        if self.use_a_state:
            self.create_subscription(Float64MultiArray,
                                     str(self.get_parameter('a_state_topic').value),
                                     self._on_a_state, 10)
        if self.attach_to_a:
            from gz.msgs10.empty_pb2 import Empty
            self._empty_cls = Empty
            if self._gz is not None:
                self._attach_pub = self._gz.advertise(
                    str(self.get_parameter('attach_topic').value), Empty)
                self._detach_pub = self._gz.advertise(
                    str(self.get_parameter('detach_topic').value), Empty)
            self.create_subscription(Bool, str(self.get_parameter('formation_topic').value),
                                     self._on_formation_start, 10)
            self.get_logger().warn('payload_node: attach_to_a=True（编队投放：挂载→分离）')
        if self.lock_to_b:
            if self._gz is not None:
                from gz.msgs10.empty_pb2 import Empty
                self._empty_cls = Empty
                self._lock_pub = self._gz.advertise(
                    str(self.get_parameter('lock_topic').value), Empty)
                self._unlock_pub = self._gz.advertise(
                    str(self.get_parameter('unlock_topic').value), Empty)
            self.create_subscription(
                Float64MultiArray, str(self.get_parameter('lock_request_topic').value),
                self._on_lock_request, 10)
            self.get_logger().warn(
                f'payload_node: lock_to_b=True（捕获时在 B 漏斗处重生成并锁定）'
                f' lock_model={self.lock_model_path}')
        self.active = not self.use_gz          # use_gz 时：释放并瞬移后才发布状态
        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info(f'payload_node: release at t={self.t_r:.1f}s from {self.p_r}')
        if self.use_gz:
            import threading
            target = self._spawn_attached if self.attach_to_a else self._spawn_parked
            threading.Thread(target=target, daemon=True).start()

    # ------------------------------------------------------------- callbacks
    def _on_caught(self, msg):
        self.caught = bool(msg.data)

    def _on_release_at(self, msg):
        if self.abs_release_at is None:
            self.abs_release_at = float(msg.data)
            self.get_logger().warn(f'payload_node: 收到释放时刻 {self.abs_release_at:.3f}')

    def _on_release_abort(self, msg):
        """释放取消（A 在 lead 窗口复核失败）：撤销已排定的释放时刻（载荷未释放时）。"""
        if msg.data and not self.model.released:
            if self.abs_release_at is not None:
                self.get_logger().warn('payload_node: 收到释放取消，撤销已排定的释放时刻')
            self.abs_release_at = None

    def _on_a_state(self, msg):
        if len(msg.data) >= 4:
            self._a_world = np.array([msg.data[1], msg.data[2], msg.data[3]])

    def _on_formation_start(self, msg):
        """B 进入编队巡航：载荷已在生成时由 DetachableJoint attach（configure 即挂载）。"""
        if msg.data and self.attach_to_a:
            self.get_logger().warn(
                f'payload: 编队开始，载荷已挂载（spawned={self.spawned}, attached={self._attached}）')

    def _on_lock_request(self, msg):
        """B 报告捕获（msg.data=[x,y,z] NED 漏斗口）→ 把自由载荷换成带 B 侧关节的载荷。

    因为 `DetachableJoint` 在模型 configure 即建关节：直接在 **B 漏斗处**重生成
    `lock_model`（偏移≈0）就能稳定锁住；而“远处停车位 spawn”会因巨幅偏移把 B 拽下去。
    """
        if not self.lock_to_b or self._locked or self._gz is None:
            return
        if not self.lock_model_path or not os.path.exists(self.lock_model_path):
            self.get_logger().error(f'payload: lock_model_path 无效 {self.lock_model_path}')
            return
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        from gz.msgs10.entity_pb2 import Entity
        from gz.msgs10.boolean_pb2 import Boolean
        pose_ned = np.array(list(msg.data[:3]), float)
        # 1) 删除当前自由载荷
        ereq = Entity()
        ereq.name = 'payload'
        try:
            self._gz.request(f'/world/{self.world}/remove', ereq, Entity, Boolean, 3000)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'payload: remove 异常 {e}')
        # 2) 在 B 漏斗处 spawn 带关节的载荷（configure 即 attach，偏移≈0）
        x_e, y_n, z_u = ned_to_enu_pos(pose_ned)
        req = EntityFactory()
        req.sdf_filename = self.lock_model_path
        req.name = 'payload_locked'
        req.allow_renaming = True
        req.pose.position.x, req.pose.position.y, req.pose.position.z = x_e, y_n, z_u
        req.pose.orientation.w = 1.0
        ok, _resp = (False, None)
        for _ in range(3):
            try:
                ok, _resp = self._gz.request(f'/world/{self.world}/create', req,
                                             EntityFactory, Boolean, 5000)
                if ok:
                    break
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'payload: lock create 异常 {e}')
        self._locked = True
        self.get_logger().warn(
            f'payload: 主动保持 —— 已在 B 漏斗处重生成并锁定 @NED={pose_ned.round(2)}')

    def _spawn_attached(self):
        """attach 模式：等 A 状态 → 在 A 正下方生成载荷 → 挂载（避免长偏移关节拖垮 A）。"""
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        import time as _t
        for _ in range(100):                      # 等 A 状态（≈最多 20s）
            if self._a_world is not None:
                break
            _t.sleep(0.2)
        if self._a_world is None:
            self.get_logger().error('payload: 等不到 A 状态，无法挂载')
            return
        p_rel = self._a_world + self.release_offset
        self.p_r = p_rel.copy()
        x_e, y_n, z_u = ned_to_enu_pos(p_rel)
        req = EntityFactory()
        req.sdf_filename = self.model_path
        req.name = 'payload'
        req.allow_renaming = False
        req.pose.position.x, req.pose.position.y, req.pose.position.z = x_e, y_n, z_u
        req.pose.orientation.w = 1.0
        for attempt in range(20):
            if self.spawned:
                break
            try:
                ok, resp = self._gz.request(f'/world/{self.world}/create', req,
                                            EntityFactory, Boolean, 5000)
                if ok and getattr(resp, 'data', False):
                    self.spawned = True
                    self.get_logger().warn(
                        f'payload: 在 A 下方生成 @NED={p_rel.round(2)} (attempt {attempt+1})')
                    break
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'payload: 挂载生成异常 {e}')
            _t.sleep(0.5)
        _t.sleep(0.3)
        if self.spawned:
            # DetachableJoint 在模型 configure 时即已与 A 建关节；
            # 不可再发 attach，否则会叠加第二个固定关节 → 过约束 → 速度爆振。
            self._attached = True
            self.get_logger().warn('payload: 载荷已挂载到 A（随 A 飞行）')

    def _on_odom(self, msg):
        p = msg.pose.position
        v = msg.twist.linear
        self._gz_p = enu_to_ned_pos(p)
        self._gz_v = enu_to_ned_vec(v)
        self._gz_t = self.get_clock().now().nanoseconds * 1e-9

    # ----------------------------------------------------------------- spawn
    def _spawn_parked(self):
        """启动时把载荷生成在远处停机位（避免释放时刻 create 服务延迟）。"""
        from gz.msgs10.pose_pb2 import Pose as GzPose
        import time as _t
        park = (100.0, 100.0, 0.06)          # ENU，远离作业区、贴地
        req = EntityFactory()
        req.sdf_filename = self.model_path
        req.name = 'payload'
        req.allow_renaming = False
        req.pose.position.x, req.pose.position.y, req.pose.position.z = park
        req.pose.orientation.w = 1.0
        for attempt in range(20):
            if self.spawned:
                return
            try:
                ok, resp = self._gz.request(f'/world/{self.world}/create', req,
                                            EntityFactory, Boolean, 5000)
                if ok and getattr(resp, 'data', False):
                    self.spawned = True
                    self.get_logger().warn(f'payload: 停车位生成 OK @ENU={park} (attempt {attempt+1})')
                    return
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'payload: 停车位生成异常 {e}')
            _t.sleep(1.0)
        self.get_logger().error('payload: 停车位生成失败')

    def _teleport_to_release(self):
        """释放时把载荷瞬移到释放点（随后自然下落）。"""
        from gz.msgs10.pose_pb2 import Pose as GzPose
        x_e, y_n, z_u = ned_to_enu_pos(self.p_r)
        req = GzPose()
        req.name = 'payload'
        req.position.x, req.position.y, req.position.z = x_e, y_n, z_u
        req.orientation.w = 1.0
        try:
            ok, resp = self._gz.request(f'/world/{self.world}/set_pose', req,
                                        GzPose, Boolean, 3000)
            self.get_logger().warn(f'payload: 瞬移到释放点 @ENU=({x_e:.2f},{y_n:.2f},{z_u:.2f}) ok={ok}')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'payload: set_pose 异常 {e}')

    # ------------------------------------------------------------------ tick
    def _tick(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        due = (now_s >= self.abs_release_at) if self.abs_release_at is not None else (self.t >= self.t_r)

        if (not self.model.released) and due:
            p_rel = self.p_r.copy()
            if self.use_a_state and self._a_world is not None:
                p_rel = self._a_world + self.release_offset
            if self.release_xy_sigma > 0:      # 释放误差（水平）
                p_rel = p_rel.copy()
                p_rel[:2] = p_rel[:2] + self._rng.normal(0.0, self.release_xy_sigma, 2)
            self.p_r = p_rel.copy()
            self.model.release(p_rel, self.v_r)         # 标记已释放
            self.pub_released.publish(Bool(data=True))
            self.get_logger().warn(f'PAYLOAD RELEASED at t={self.t:.3f}s pos={p_rel}')
            if self.use_gz:
                if self._attached and self._detach_pub is not None:
                    # 编队模式：从 A 分离 → 继承 A 的速度自由下落（不再瞬移）
                    self._detach_pub.publish(self._empty_cls())
                    self._attached = False
                    self.get_logger().warn('payload: 已与 A 分离（继承 A 速度）')
                else:
                    self._teleport_to_release()
                self.active = True
            else:
                self.active = True

        if self.use_gz and self.spawned and self._gz_p is not None:
            p, v = self._gz_p, self._gz_v
        elif self.model.released:
            p, v = self.model.pos.copy(), self.model.vel.copy()
            self.model.step(self.dt)
        else:
            p, v = self.p_r.copy(), self.v_r.copy()

        if self.active:
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
