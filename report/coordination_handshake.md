# 协同释放握手 + 意图共享（已实现并 SITL 验证）

> 承接 `report/coordination.md`：现状是"**单向、A 被动**"（B 用带延迟/噪声的 A 估计独自决定释放）。
> 本文实现**合理的双向释放协议**：**B 报就绪 → A 作释放权威（用自身精确状态）→ ack → B 下潜**，
> 并让 A 广播**意图**；默认关闭（`coord_mode='direct'`），非破坏。
>
> 复现：
> ```bash
> COORD=handshake bash run_m6_sitl.sh 70      # 协同握手
> bash run_m6_sitl.sh 70                       # 原 direct（默认）
> ```

---

## 0. 结论速览

- **协议已跑通**：SITL `COORD=handshake` → `A: 收到 B 就绪 → 释放权威发布 release@…` →
  `B: 收到 A 释放 ack@… → DIVE` → **`STACK CAPTURED horiz=0.065m`**、无 failsafe。
- **协调更合理**：释放决策改用 **A 的精确自身状态**（不再依赖 B 对 A 的带延迟/噪声估计）；
  双向就绪 + ack + 新鲜度检查，避免"B 以为对齐但其实 A 没就位"的误释放。
- **可回退**：`coord_mode='direct'` 保持原行为（默认）。

---

## 1. 协议设计

**话题**
| 话题 | 类型 | 发布者 | 内容 |
|---|---|---|---|
| `/drone_a/intent` | Float64MultiArray | A | `[t, 悬停x, 悬停y, 高度, vx, vy]` — A 的**意图**（B 可零延迟对准） |
| `/drone_b/ready` | Float64MultiArray | B | `[ready(0/1), rel_xy, spd_xy, stamp]` — B 就绪（**持续刷新、不对齐即置 0**） |
| `/drone_a/release_cmd` | Float64MultiArray | A | `[release(0/1), t_rel, stamp]` — A 的**释放 ack**（携带与 `/payload/release_at` 同一个 `t_rel`） |
| `/payload/release_at` | Float64 | A（handshake）/ B（direct） | 实际释放时刻 |
| `/coord/ping` / `/coord/pong` | Float64MultiArray | B→A / A→B | 时钟同步：`[t_b_send]` / `[t_b_send, t_a_recv]` |

**状态机**
```
B: ALIGN ──对齐+保持 align_hold_s──► 持续发 ready=1 ──收到 A 的 release_cmd──► DIVE
        └─未对齐─► 发 ready=0
A: 收 ready ──新鲜(≤ready_timeout)且为真 且 A自身已就位──► 发 /payload/release_at + release_cmd（只发一次）
```

---

## 2. 合理性（soundness）论证

1. **双向同意**：A **只在 B 报就绪时**释放；B **只在收到 A ack 后**下潜。任一方不配合都不动作。
2. **新鲜度/可撤销**：A 只认 `ready_timeout` 内的就绪消息；B 一旦不再对齐立即置 `ready=0`
   ⇒ 不会用过期/失效的就绪误释放。（ready 由 B 每拍刷新。）
3. **唯一释放**：A 内部 latch，`release_cmd` 只发一次 ⇒ 不重复释放。
4. **就位门限由"持有方"判定**：A 用**自己的精确状态**（位置/高度/速度）判断"已就位"，
   不再让"接收方对投送方的估计误差"进入释放决策——这是本协议的核心收益。
5. **延迟补偿一致**：A 统一施加 `release_lead`，并把同一个 `t_rel` 同时给 `/payload/release_at`
   与 `release_cmd` ⇒ A 与 B 对**释放时刻**的认知一致。
6. **可回退/非破坏**：`coord_mode='direct'`（默认）保持原有"B 直接决定释放"行为。
7. **无死锁/无竞态**：B 持续报就绪（可撤销），A 单次 ack；若 B 迟迟收不到 ack，仍保持对正并继续刷新就绪，
   不会卡死；超时（`handshake_timeout`）后不再认旧 ack。
8. **释放前一致性门限**：A 除“自身已就位”外，还要求 B 报告的 `rel_xy ≤ release_xy_tol`、`spd_xy ≤ release_spd_tol`
   （A 的**独立门限**，不只信 B 自己的阈值）才释放。
9. **时钟一致**：B 周期性 **往返测时** 估 `offset(A−B) = t_a_recv + rtt/2 − t_b_recv`（EMA 平滑），
   把 A ack 里的 `t_rel`（A 时钟）**换算到 B 时钟**（`t_rel − offset`）再用于 DIVE ⇒ 跨机时钟不一致也不影响释放时序。

---

## 3. SITL 验证（已跑通 ✅）

```
A: coord_mode=handshake（B 报就绪 → A 作释放权威）
B: coord_mode=handshake（报就绪 → 等 A 释放 ack）
B: ALIGNED rel_xy=0.051m …（持续报 ready）
A: 收到 B 就绪 → 释放权威发布 release@…s (lead=0.2)
payload_node: PAYLOAD RELEASED at t=17.500s
B: 收到 A 释放 ack@…s（rel_xy=0.051）→ DIVE
*** STACK CAPTURED *** horiz=0.065m rel_v=0.342m/s   无 failsafe
```

流程完整：B 就绪 → A 释放权威 → payload 释放 → B 收 ack 下潜 → 捕获。

**含时钟同步 + 释放门限（另一轮）**：
```
B: 时钟同步 offset(A−B)=-0.0000s rtt=0.5ms      ← 往返测时（同机≈0）
A: 收到 B 就绪 → 释放权威发布 release@…（A 的自身已就位 + B 对正达标）
B: 收到 A 释放 ack@…（rel_xy=0.063）→ DIVE
*** STACK CAPTURED *** horiz=0.055m rel_v=2.160m/s   无 failsafe
```

---

## 4. 与 direct 的对比（定性）

| 方面 | direct（原） | handshake（新） |
|---|---|---|
| 释放决策依据 | B 对 A 的**带延迟/噪声估计** | A 的**精确自身状态** |
| 一致性 | 单向、无 ack | 双向 ready + ack、共享 `t_rel` |
| 误释放风险 | "B 以为对齐" 即释放 | 需 B 就绪 **且** A 就位 **且** 消息新鲜 |
| 延迟补偿 | B 端 `release_lead` | A 端统一施加，A/B 共享 |
| 默认 | ✅（保持） | 可选（`COORD=handshake`） |

---

## 4b. 编队模式握手（M6-moving，已实现 ✅）

编队下单机决策同样升级为握手：
- ALIGN 只发 `/formation/start`（**不**报就绪）；进入 FORMATION 后才报就绪。
- A 的“就位”判定在编队下改为**速度匹配编队速度**（而非“悬停静止”）；A 的**意图**广播“当前编队参考点”（移动目标）。
- 释放 ack 是**原子事件**：B 收到即切 DIVE，**不再依赖瞬时对齐**
  （否则释放/分离会让对齐短暂抖动，B 会漏掉 ack 卡在 FORMATION——已修）。

```
B: ALIGNED rel_xy=0.095m → /formation/start
A: /formation/start → 编队巡航 vel=[0.5 0.]
B phase=FORMATION …（持续报就绪）
A: 收到 B 就绪 → 释放权威发布 release@…（A 速度已匹配编队速度）
B: 收到 A 释放 ack@…（原 phase=FORMATION）→ DIVE
*** STACK CAPTURED *** horiz=0.040m rel_v=3.315m/s   无 failsafe
```

## 4c. 意图升级：A 广播【预测落点】（已实现）

`/drone_a/intent` 从“悬停目标”扩为 **`[t, 目标x, 目标y, 高度, vx, vy, 落点x, 落点y]`**：
- **落点 = 目标点 + 风漂移**：A 用风估计 `wind_est` + 载荷阻力 `payload_drag_k` + 标称下落 `payload_fall_t`
  算水平漂移（`_horiz_drift`，含 linear/quadratic），把**预测落点**直接广播。
- B 开 `use_intent` 时**直接对齐落点**（相对偏差 `|落点−B|`），而不是“A 的投影”。
- 好处：横风下 B 对准**载荷将要去的地方**，而不是“A 现在在哪”；风估计可由 PX4 EKF 提供。

SITL 验证（`WIND_EST="0.0,0.0,0.0"` → 落点=目标，属无害 no-op）：
```
COORD=handshake WIND_EST="0.0,0.0,0.0" … → *** STACK CAPTURED *** horiz=0.017m   无异常
```

> 注：`WIND_EST` 必须写成**全 float**（`0.0,0.0,0.0`），否则 launch 向量被判为 INTEGER_ARRAY 而报类型错误。

## 4d. 安全层：不确定度感知 keep-out + 释放后清场（已实现 ✅）

**(1) 不确定度感知的 keep-out**：B 与 A 的最小高度间隔从常数 `min_ab_gap` 升级为
```
gap_eff = min_ab_gap + k·σ        （k=safety_k 默认 2；σ 为 A 相对位置误差）
```
- σ **在线估计**：接收到的 A 位置与平滑估计的残差做 EMA（`_sigma_est`）；可用 `rel_sigma_floor` 设下限。
- 所有 keep-out 处（CLIMB/WAIT_A/TRANSLATE/ALIGN/FORMATION/DONE）改用 `gap_eff`；
  **WAIT_A 会主动下降到满足 keep-out**（否则大 σ 下会卡在 WAIT）。
- 意义：**估计越不确定，B 离 A 越远**（保守安全）。

**(2) 释放后 A 定向清场**：A 订阅 `/payload/released`，释放后把悬停目标平移到
`hover + clear_offset`（默认 +2m 水平），**离开 B 的空域**，给 B 让出下潜/机动空间。

SITL 验证（`COORD=handshake SAFETY_FLOOR=0.20` → `gap_eff=1.2`）：
```
B: WAIT_A done (A_alt=4.36, clear=1.37) → TRANSLATE   ← 等 clear≥gap_eff 才横移
B phase=ALIGN pos_w=[-0.02 -0.02 -3.28]               ← B 保持更低
A: 收到 B 就绪 → 释放权威发布 release@…
A: /payload/released → 开始定向清场（离开 B 空域）
*** STACK CAPTURED *** horiz=0.010m   无 failsafe
```

## 5. 局限与下一步

已完成：**协同握手**、**意图广播**、**时钟同步（往返测时 + 换算 t_rel）**、**释放前一致性门限**。

待办：
- **意图升级**：从“悬停目标”扩到**完整预测轨迹 + 风速估计**（B 用 A 的未来状态预判释放点）。
- **编队模式（M6-moving）**的握手待接（当前只覆盖 stack 的 ALIGN）。
- **释放前落点余量预测**：结合风/阻力模型直接预测“落点余量”，不足则暂不释放（目前只用了 B 报的静态质量指标）。
- **安全层带不确定度**：`min_ab_gap + kσ`，释放后 A 定向清场。
