# 悬停优先控制：把鲁棒几何规则落到 b_node（离线 + SITL）

> 承接 `report/robust_geometry_and_retention.md` 的结论：**横风下"下潜"是反效果的**
> （它拉长下落时间、增大漂移），免下潜条件为 `gap ≤ v_retain²/(2g)`。
> 本文把它**落到控制**：`b_node` 的 DIVE 阶段从"无条件 `a_dive=3`"改为
> **"先悬停、只有 `v_rel` 会超 `v_retain` 时才取最小必要下潜"**。
>
> 复现：
> ```bash
> # 离线
> python3 tools/stack_run.py --scenario M6_stack_nodive
> # SITL（默认 auto_min_dive=true）
> source ~/drone_payload_catch/env.sh
> FUNNEL_MOUTH=0.30 bash ~/drone_payload_catch/run_m6_sitl.sh 70
> ```

---

## 1. 规则与实现

**规则**：`a_dive = max(0, g − v_retain²/(2·gap))`，并封顶 `b_max_accel`。
- `gap ≤ v_retain²/(2g)`（当前 0.833m）→ `a_dive=0`，**B 悬停等待，载荷自己落进漏斗**；
- 否则取**刚好**使 `v_rel ≤ v_retain` 的最小下潜。

**改动**（均非破坏、可关）：
| 文件 | 改动 |
|---|---|
| `payload_catch/stack_drop.py` | 新增 `minimal_dive(gap, v_retain, g, a_dive_max)`；`plan_stack_drop(a_dive=None)` 自动取最小；`simulate_stack(vert_mode='minimal')` |
| `payload_catch/b_node.py` | 参数 `auto_min_dive`（默认 true）；DIVE 规划用 `minimal_dive`，日志打印 `a_dive=…(auto=True)` |
| `launch/catch_stack_launch.py` | 新增 launch 参数 `auto_min_dive` |
| `config/catch_scenarios.yaml` | 新增 `M6_stack_nodive`（gap=0.7m，`vert_mode: minimal`） |

---

## 2. 离线验证（大漏斗 eff_r=0.25，windkf+lead1，n=30/格）

| 侧风 w | gap1.0/a_dive3（标称） | gap1.0/最小(a=1.63) | gap0.7/免潜(a=0) | **gap0.6/免潜(a=0)** |
|---|---|---|---|---|
| 2 | 29/30 | 29/30 | 30/30 | 30/30 |
| 3 | 22/30 | 23/30 | 27/30 | **28/30** |
| 4 | 4/30 | 8/30 | 20/30 | **27/30** |
| 5 | 0/30 | 2/30 | 10/30 | **18/30** |

**读数**：在强侧风下，"小 gap + 免下潜"显著扩大容忍：
- w=4：**4/30 → 27/30**（约 7×）；w=5：**0/30 → 18/30**。
- 对比"gap1.0 但改成最小下潜"（a_dive 3→1.63）：仅小幅改善（w=4: 4→8/30）——
  说明**主要增益来自缩短 gap（减少漂移时间），下潜最小化是次要**，但两者方向一致、共同叠加。

---

## 3. SITL 验证（已跑通 ✅）

标称几何 A=4.5/B=3.5（有效 gap≈0.64m ≤ 0.833m），大漏斗，`auto_min_dive=true`：

```
B: WAIT_A done → TRANSLATE done → B: ALIGNED rel_xy=0.086m → release
PAYLOAD RELEASED at t=17.570s
B: DIVE plan a_dive=0.00(auto=True) t_c=0.355s v_rel=3.479 v_retain=4.044 feasible=True
*** STACK CAPTURED *** horiz=0.061m rel_v=2.292m/s z_mouth=-3.96
B phase=LAND ... caught=True     （两机均无 Failsafe，无异常）
```

**关键**：日志出现 **`a_dive=0.00(auto=True)`** —— B 不再下潜，而是**悬停等待**载荷落入漏斗，
且成功捕获并落地。对比旧逻辑（无条件 `a_dive=3`，`t_c=0.435s`）：
新逻辑 `t_c=0.355s`，**下落时间缩短 ~18%**、`v_rel` 从 2.96 升到 3.48（仍 < v_retain=4.04），
在横风下更稳。

> 注：SITL 的 Gazebo 载荷模型**未注入风**，所以横风增益只能在离线量化；SITL 验证的是
> **"免下潜"逻辑的功能正确性**（悬停即接、不误触发、正常落地）。

---

## 4. 结论与局限

**结论**
1. "**悬停优先 / 最小必要下潜**"已落到控制并在 SITL 跑通（`a_dive=0.00(auto=True)` → `STACK CAPTURED`）。
2. 离线显示该规则在**强横风**下把成功率提升数倍（w=4: 4→27/30），且与"大漏斗"收益叠加。
3. 默认开启（`auto_min_dive=true`）；需要旧行为时设 `auto_min_dive:=false` 或 `--vert open`。

**局限 / 下一步**
- SITL 无风：想直接在 SITL 验证横风增益，需给 `models/payload` 加风/阻力（Gazebo 里可加 `<wind>` 或外力插件）。
- 当前规则用**已知 gap 与 v_retain**；若 gap 估计有偏（A 高度估计延迟），可加一点安全裕量
  （如 `a_dive = minimal_dive(gap − 0.05, …)`）。
- 下一步：把**主动保持（锁扣/磁吸）**或**空心锥**做到模型层，彻底摆脱 `v_retain` 与横风漂移。
