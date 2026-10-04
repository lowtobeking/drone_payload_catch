# 末端捕获能力：大漏斗（口半径 0.20→0.30m）离线 + SITL 验证

> 承接 `report/m6_robustness_opt.md` 的结论：**末端能力（`eff_r` / `v_retain`）是压倒性的第一杠杆**。
> 本文把这一杠杆**真正落地**：新增一个大漏斗模型 `x500_funnel_big`、把 SITL 启动参数化，
> 并在**离线**与 **SITL** 两层验证。
>
> 复现：
> ```bash
> # 离线：大漏斗 vs 原漏斗
> python3 tools/stack_run.py --scenario M6_stack_bigfunnel --mc 100
> python3 tools/stack_run.py --sweep-funnel
> # SITL（无窗口）：FUNNEL_MOUTH=0.30 自动选用大漏斗模型
> source ~/drone_payload_catch/env.sh
> FUNNEL_MOUTH=0.30 bash ~/drone_payload_catch/run_m6_sitl.sh 70
> # 加相对定位噪声：
> FUNNEL_MOUTH=0.30 LAUNCH_EXTRA="rel_pos_sigma:=0.10 rel_latency:=0.10 rel_bias:=0.05" \
>   bash ~/drone_payload_catch/run_m6_sitl.sh 70
> ```

---

## 1. 为什么做这个

`--mc 200` 量出 M6 标称捕获余量 `eff_r − hmiss` 的 **min 仅 0.0036m**——捕获"贴边"。
而实验（`m6_robustness_opt.md` §3）表明：口半径 0.16→0.36 使侧风 w=2 从 **3/20→20/20**，
机械臂 reach +0.10m 使 w=2 从 **11→19/20**。**软件估计/控制都造不出物理余量**，所以先做**末端几何**。

本次采取"**加大漏斗口半径**"这一最直接、零新机构的路线（下一步才是真空心漏斗/机械臂）。

---

## 2. 实现（非破坏式）

| 改动 | 说明 |
|---|---|
| `models/x500_funnel_big/model.sdf`（新增） | 与 `x500_funnel` 同构（标准 x500 + 顶部圆锥、roll=π、口朝上、低恢复系数面），**仅口半径 0.20→0.30m、length 0.14 不变**（口平面仍 z≈0.45，即 base_link 上方 0.21m）。 |
| `run_m6_sitl.sh` / `run_m6_gui.sh` | 新增 `FUNNEL_MOUTH` 环境变量：=0.20 用原模型；≠0.20 自动切大模型，并把 `funnel_mouth_radius` / `funnel_eff_radius(=mouth−0.05)` 传给节点。原默认行为不变。 |
| `config/catch_scenarios.yaml` | 新增 `M6_stack_bigfunnel`（`mouth_radius: 0.30`）——离线单一真值源。 |

> 设计原则：**不动原模型与默认配置**，用变体 + 环境变量做 A/B；把大漏斗设为推荐配置由调用方决定。

---

## 3. 离线验证（MC n=100，相对定位噪声 0.05m/0.05s）

| 场景 | eff_r | 成功率 | 捕获水平偏差 max | **捕获余量 min** |
|---|---|---|---|---|
| `M6_stack_drop`（原漏斗 0.20） | 0.15m | 100/100 | 0.142m | **0.0076m** |
| `M6_stack_bigfunnel`（大漏斗 0.30） | 0.25m | 100/100 | 0.142m | **0.1076m** |

**捕获余量 min 提升约 14×**（0.0076→0.1076m）。成功率在无风标称下都是 100%，但**余量**才是鲁棒性的度量——
上一轮的侧风/噪声扫描证明，原漏斗在 w=2 就会掉到 7–11/30，而大漏斗 24–30/30。

---

## 4. SITL 验证（已跑通 ✅）

### 4.1 标称

```
create B 模型 x500_funnel_1 @ENU (0,5.0,0)  sdf=.../x500_funnel_big/model.sdf
funnel: mouth=0.30 eff=0.25
B: ALIGNED rel_xy=0.063m spd_xy=0.141 → release
PAYLOAD RELEASED at t=18.210s
B: DIVE plan t_c=0.435s v_rel=2.963 v_retain=4.044 feasible=True
*** STACK CAPTURED *** horiz=0.028m rel_v=2.193m/s  z_mouth=-3.93
B phase=LAND ... caught=True
px4_0 / px4_1: Ready for takeoff → Armed → Takeoff detected
```

### 4.2 加相对定位噪声（`rel_pos_sigma=0.10, rel_latency=0.10, rel_bias=0.05`）

```
funnel: mouth=0.30 eff=0.25
B: ALIGNED rel_xy=0.055m spd_xy=0.133 → release
B: DIVE plan t_c=0.441s v_rel=3.003 v_retain=4.044 feasible=True
*** STACK CAPTURED *** horiz=0.030m rel_v=2.174m/s
B phase=LAND ... caught=True    （两机均无 Failsafe）
```

**两次 SITL 均 `STACK CAPTURED` 并落地**；捕获水平偏差 0.028–0.030m，相对 `eff_r=0.25m`
余量 ~0.22m（原模型只有 ~0.12m 余量）。

---

## 5. 结论与局限

**结论**
1. **大漏斗在离线与 SITL 两层都验证通过**：捕获余量 min **0.0076→0.1076m（14×）**；
   注入定位噪声后 SITL 仍稳捕获、无 failsafe。
2. 这**直接印证**了"末端能力 > 估计 > 控制"的杠杆排序——**加 0.10m 半径**比调 KF/控制有效得多。
3. 改动**非破坏**：原模型/默认配置不动，用 `FUNNEL_MOUTH` 与 `M6_stack_bigfunnel` 切换。

**局限 / 下一步**
- 该漏斗仍是**实心圆锥宽口朝上 = 平顶盘**（靠低恢复系数"砸住"），只是**变大**了；
  不是空心导向漏斗，也无夹持。
- 0.30m 半径（0.60m 直径）是**大平板**：有气动/结构不现实之嫌；SITL 未见失稳，但真机需评估。
- **下一步**：
  1. **真空心导向漏斗 + 保持机构**（内壁网格 + 夹持/磁吸），把"变大"升级为"导向—夹持—带走"；
  2. **机械臂 reach/absorb**（判据层已实现 `arm_reach/arm_absorb`，见 `m6_robustness_opt.md` §3-D）；
  3. 把大漏斗升为**默认配置**（`config` + 默认 SDF），需同步更新 README/MEMORY 的历史数字。
