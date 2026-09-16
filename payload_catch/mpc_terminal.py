#!/usr/bin/env python3
"""B 的终端（会合）MPC —— acados 实现，纯离线可用。

模型：双积分器  ṗ=v, v̇=u（u = 加速度指令），状态 x=[p(3),v(3)]，控制 u(3)。
代价：LINEAR_LS，逐级跟踪解析会合参考 [p_ref, v_ref, 0]；终端重罚会合状态 [p_c, v_c]。
约束：|u| ≤ a_max（逐轴硬）；|v| ≤ v_max（逐轴软，避免不可行）。

为什么用"参考 + 终端代价"而不是固定终点时刻的终端约束：
  acados 的 tf = N·dt 在生成期固定，而会合的剩余时间 T_rem 每拍在变。
  解析会合参考定义在 [0, T_rem]，T_rem 之后保持在会合点，
  所以把参考按 min(k·dt, T_rem) 采样，时间对齐天然成立，
  终端代价再压向 (p_c, v_c)。MPC 每拍重优化 → 在约束/扰动下优于纯 PD 跟踪。
"""
from __future__ import annotations

import os
from typing import Optional, Tuple

import numpy as np

from .rendezvous import PlanResult


class TerminalMPC:
    def __init__(self, N: int = 20, dt: float = 0.05,
                 a_max: float = 6.0, v_max: float = 5.0,
                 q_pos: float = 40.0, q_vel: float = 8.0, r_a: float = 0.5,
                 q_pos_e: float = 400.0, q_vel_e: float = 80.0,
                 build_dir: str = '~/.cache/payload_catch/acados_terminal_mpc',
                 print_level: int = 0):
        self.N = int(N); self.dt = float(dt)
        self.a_max = float(a_max); self.v_max = float(v_max)
        self.q_pos = q_pos; self.q_vel = q_vel; self.r_a = r_a
        self.q_pos_e = q_pos_e; self.q_vel_e = q_vel_e
        self._build_dir = os.path.expanduser(build_dir)
        os.makedirs(self._build_dir, exist_ok=True)
        self._print_level = print_level
        self._solver = None
        self._setup()

    # ------------------------------------------------------------------ ocp
    def _setup(self):
        import casadi as ca
        from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

        nx, nu = 6, 3
        x = ca.SX.sym('x', nx)
        u = ca.SX.sym('u', nu)
        model = AcadosModel()
        model.name = 'payload_catch_terminal_mpc'
        model.x = x; model.u = u
        xdot = ca.SX.sym('xdot', nx)
        model.xdot = xdot
        f_expl = ca.vertcat(x[3], x[4], x[5], u[0], u[1], u[2])
        model.f_expl_expr = f_expl
        model.f_impl_expr = xdot - f_expl

        ocp = AcadosOcp()
        ocp.model = model
        ocp.solver_options.N_horizon = self.N

        ny = nx + nu                       # [p, v, u]
        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'
        ocp.cost.W = np.diag([self.q_pos]*3 + [self.q_vel]*3 + [self.r_a]*3)
        Vx = np.zeros((ny, nx)); Vx[0:3, 0:3] = np.eye(3); Vx[3:6, 3:6] = np.eye(3)
        Vu = np.zeros((ny, nu)); Vu[6:9, 0:3] = np.eye(3)
        ocp.cost.Vx = Vx; ocp.cost.Vu = Vu
        ocp.cost.W_e = np.diag([self.q_pos_e]*3 + [self.q_vel_e]*3)
        ocp.cost.Vx_e = np.eye(nx)
        ocp.cost.yref = np.zeros(ny)
        ocp.cost.yref_e = np.zeros(nx)

        # 约束：|u|≤a_max 硬；|v|≤v_max 软
        ocp.constraints.lbu = np.full(nu, -self.a_max)
        ocp.constraints.ubu = np.full(nu, +self.a_max)
        ocp.constraints.idxbu = np.arange(nu)
        ocp.constraints.lbx = np.full(3, -self.v_max)
        ocp.constraints.ubx = np.full(3, +self.v_max)
        ocp.constraints.idxbx = np.array([3, 4, 5])
        ocp.constraints.idxsbx = np.arange(3)
        ocp.cost.zl = np.full(3, 1e4); ocp.cost.zu = np.full(3, 1e4)
        ocp.cost.Zl = np.full(3, 1e2); ocp.cost.Zu = np.full(3, 1e2)
        ocp.constraints.x0 = np.zeros(nx)

        ocp.solver_options.tf = self.N * self.dt
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 1
        ocp.solver_options.print_level = self._print_level
        ocp.solver_options.qp_solver_iter_max = 50
        ocp.code_export_directory = os.path.join(self._build_dir, 'c_generated_code')
        ocp.code_gen_options.json_file = os.path.join(self._build_dir, 'acados_ocp.json')
        self._solver = AcadosOcpSolver(ocp)
        self._nx, self._nu = nx, nu

    # ---------------------------------------------------------------- solve
    def solve(self, x0: np.ndarray, ref_t: np.ndarray, ref_p: np.ndarray,
              ref_v: np.ndarray, p_c: np.ndarray, v_c: np.ndarray,
              t_start: float = 0.0) -> Tuple[np.ndarray, int, np.ndarray]:
        """给定期望参考（定义在 [0, ref_t[-1]]，其后保持），返回 (u0, status)。

        参考按 t_start + min(k·dt, T_rem) 采样（t_start = 当前时刻在参考里的位置）；
        终端 yref_e = (p_c, v_c)。
        """
        N = self.N
        T_rem = max(1e-9, float(ref_t[-1]) - float(t_start))
        x0 = np.asarray(x0, float).reshape(self._nx)
        self._solver.set(0, 'lbx', x0)
        self._solver.set(0, 'ubx', x0)
        nref = len(ref_t)
        T_end = float(ref_t[-1])

        def _idx(tr: float) -> int:
            j = int(round(tr / max(T_end, 1e-9) * (nref - 1)))
            return max(0, min(j, nref - 1))

        for k in range(N):
            tr = float(t_start) + min(k * self.dt, T_rem)
            j = _idx(tr)
            yref = np.concatenate([ref_p[j], ref_v[j], np.zeros(self._nu)])
            self._solver.set(k, 'yref', yref)
        # 终端参考 = 参考在 t_start + N·dt 处的状态（当视界覆盖到会合时刻，
        # 它自然等于会合状态；否则只是“参考前进一步”的中间点）。
        # ⚠️ 不能直接填 (p_c, v_c)：那会让 MPC 无论剩余多久都想在 N·dt 内到达会合点。
        j_end = _idx(float(t_start) + N * self.dt)
        self._solver.set(N, 'yref', np.concatenate([ref_p[j_end], ref_v[j_end]]))
        status = self._solver.solve()
        u0 = np.asarray(self._solver.get(0, 'u'), float).reshape(self._nu)
        # 预测的下一拍速度（PX4 速度接口用它当前馈设定点）
        x1 = np.asarray(self._solver.get(1, 'x'), float).reshape(self._nx)
        return u0, int(status), x1[3:6].copy()


if __name__ == '__main__':
    # 自测：让 B 从静止走到 (p_c,v_c)，看 u0 是否合理
    mpc = TerminalMPC(N=20, dt=0.05, a_max=6.0, v_max=5.0)
    ref_t = np.linspace(0.0, 0.5, 26)
    ref_p = np.zeros((26, 3)); ref_v = np.zeros((26, 3))
    for k in range(26):
        s = ref_t[k] / 0.5
        ref_p[k] = [s * 1.0, 0, -0.5 * s]
        ref_v[k] = [2.0, 0, -1.0]
    p_c = np.array([1.0, 0.0, -0.5]); v_c = np.array([2.0, 0.0, -1.0])
    x0 = np.zeros(6)
    u0, st = mpc.solve(x0, ref_t, ref_p, ref_v, p_c, v_c)
    print(f'status={st} u0={np.round(u0,3)}  (期望大致朝 +x/-z 加速)')
