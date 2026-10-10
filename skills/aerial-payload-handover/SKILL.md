---
name: aerial-payload-handover
description: 空中载荷交接（无人机 A 抛投、无人机 B 在空中接住）的仿真、末端机构选型与可行性分析。当需要评估空投捕获成功率、选择漏斗/托盘/主动锁扣末端、计算泡棉缓冲与围边、评估释放时刻/相对定位精度对捕获概率的影响，或生成仿真报告时使用。
---

# 空中载荷交接仿真技能（Aerial Payload Handover）

本技能让 Agent 能：**给定载荷与机型规格，跑仿真、选末端、查释放证书、出报告**。
核心算法层是纯 Python，**离线秒级可跑**，不需要 ROS/SITL。

## 何时使用

- 用户问"无人机空投/空中接住/会合捕获"能否做、参数怎么定、成功率多少；
- 需要**末端选型**（实心漏斗 / 空心杯 / 圆形托盘 / 主动锁扣）或算围边/泡棉；
- 需要评估**释放时刻、相对定位精度 σ、通信延迟**对捕获的影响（C1 证书）；
- 需要评估**动力学/接触**（倾角推力限幅、冲击可恢复性、带载余量）；
- 需要生成 Word 仿真报告。

## 前置与约定

- 仓库根：`$DRONE_PAYLOAD_CATCH`，默认 `~/drone_payload_catch`。
- 坐标：世界系 **NED**（x=北, y=东, z=下），高度 = `-z`。
- 单一真值源：`config/catch_scenarios.yaml`（几何/参数/工况都在这里，勿硬编码）。
- 离线脚本：直接 `python3 ...` 即可。
- SITL 脚本：先 `source env.sh`（PX4-1.16 + px4_msgs v1.16.2 + Gazebo）。

## 标准工作流

1. **明确输入**：载荷（质量/尺寸/形状）、投放方式（悬停 / 编队同速）、接收末端、
   约束（下落高度 gap、风、相对定位 σ、通信延迟）。
2. **选任务型**：
   - 定点垂直投放 → M6（`stack_drop`，解析规划器）；
   - 带水平速度会合 → M1–M4（`rendezvous` 通用规划器）。
3. **跑仿真**（见下"脚本"）——先单次看规划是否 feasible，再蒙特卡洛看成功率。
4. **末端选型**：用 `size_tray.sh` / `run_m6_mc.sh` 扫盘径、围边、泡棉 e。
5. **可行性/安全**：`check_release_cert.sh`（C1 证书 + 相对 σ）、`analyze_dynamics_contact.sh`。
6. **出报告**：`make_report.sh` → `.docx`。

## 脚本（都在 `scripts/`，薄封装）

| 脚本 | 作用 | 关键输出 |
|---|---|---|
| `run_offline.sh [--all\|scenario]` | 会合类（M1–M4）离线体检 | 最近距离/相对速度/PASS |
| `run_m6_mc.sh [scenario] [n]` | M6 垂直投放蒙特卡洛 | 成功率、水平偏差、余量 |
| `size_tray.sh <args>` | 圆形托盘选型 | eff_r / v_retain / 是否弹出 / 最大 gap |
| `measure_foam_e.sh <args>` | 泡棉恢复系数 e 换算 | 推荐 TRAY_E / v_retain |
| `check_release_cert.sh` | 相对不确定度 + C1 证书 | 释放阈值 T、证书可行性 |
| `analyze_dynamics_contact.sh` | 动力学限幅 + 接触冲击 | 倾角/权限、可恢复性、推力余量 |
| `run_checks.sh [--quick]` | **一键离线自检**（改代码后必跑） | 各模块自测 / C1 证书 / C5 CBF / 安全层 / acados 是否全绿 |
| `preflight.sh [--sitl] [--live]` | **起飞前自检**（跑 SITL/真机前） | 配置·依赖·PX4·RMW·ROS·EKF 是否就绪 |
| `make_report.sh` | 生成 Word 仿真报告 | `report/*.docx` |
| `run_sitl_tray.sh [vel]` | SITL：M6-moving + 托盘（可选） | `STACK CAPTURED` |

调用示例：

```bash
bash scripts/size_tray.sh --diameter 0.30 --rim 0.05 --e 0.15 --obj 0.06 --gap 1.0
bash scripts/run_m6_mc.sh M6_stack_tray 300
bash scripts/check_release_cert.sh
bash scripts/run_checks.sh --quick     # 改代码后必跑：全绿才算没回归
```

## 关键物理（速查）

- 抛体：`p_p(τ) = p_r + v_r·τ + ½g·τ²`（释放继承 A 速度）。
- M6 垂直接触速度下界：`v_rel = √(2(g − a_dive)·gap)`（B 只能往下压，`a_dive<g`）。
- 刚性末端保持判据：`位置：水平偏差 < eff_r`；`速度：v_rel ≤ v_retain = √(2g·depth)/e`。
- **圆形托盘**（无杯深）：保持条件 `e²·gap ≤ h`（h=围边+凹垫），`eff_r = 盘内半径 − 物半宽`。
- **释放证书（C1）**：`‖δ̂‖ ≤ T(ε)`，`T` 由精确 Rice/Marcum-Q 求；**`σ` 必须是相对定位 σ**，
  不是绝对 σ（`σ_rel²=σ_A²+σ_B²−2ρσ_Aσ_B+…`）。
- **动力学**：`a_x ≤ (g−a_z)·tanθ_max`（水平权限随下潜衰减）；`a_max` 是姿态约束的投影。
- **接触**：冲量 `J=m_p·v_rel`；偏心角速度 `ω=J·d_off/I_B`；带载悬停需 `T_max ≥ (m_B+m_p)g`。

## 注意事项（本工程血泪）

- **不要重跑排查**：先读 `references/` 与 `report/`，很多问题是已知坑。
- **相对定位是成败单点**：`eff_r≈0.12–0.17m`，相对 σ 必须远小于它。
- **SITL 平台边界**：`v_p≈4.9 m/s` 触发 B 姿态 failsafe（平台层，非算法）。
- **仿真偏乐观**：模型泡棉 e≈0，真机须实测（`measure_foam_e.sh`）。
- **清理进程**：`pkill -f` 的模式别写进调用方命令行（会误杀自己），用独立脚本。

## 参考（渐进式披露，按需加载）

- `references/physics.md`：物理公式、判据、证书推导。
- `references/end_effectors.md`：漏斗 / 空心杯 / 托盘 / 锁扣 对比与选择。
- `references/architecture.md`：代码与数据流地图（模块、话题、脚本）。
- `references/negative_results.md`：已证负结果（避免重走弯路）。
- 深入：仓库 `report/*.md`（38 份）、`README.md`、`MEMORY.md`、`SIM_COMMANDS.md`。
