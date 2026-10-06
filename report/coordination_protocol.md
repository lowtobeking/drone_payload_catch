# C2：通信鲁棒的交接协议与时序误差界

> 研究 W3 / 论文贡献 C2。把 handshake + commit/abort + 时钟同步 从**经验常数**升级为**有界设计**。
> 复现：`python3 tools/coord_proto.py --n 200000`。

---

## 1. 协议与时间线

```
B: --ready(质量 q, σ)--> A
A: 校验门限 → 进入 commit 窗口(commit_hold_s) → 提交:
     发布 release_at = t_c + release_lead  (→ payload_node)
     发布 release_cmd = [1, t_rel, stamp]  (→ B)
   [t_c, t_rel] 内复核；失败 → release_abort (→ payload_node) + release_cmd=[-1] (→ B)
B: 收到 ack → 用 clock_offset 换算 t_rel → DIVE
payload: 到 t_rel 释放（若未被 abort）
```

单向延迟上界 `d_max`、抖动 `j`、时钟误差 `ε_clock`、各消息独立丢包 `p`。

---

## 2. 可导出的时序条件（命题）

| 编号 | 条件 | 含义 |
|---|---|---|
| T1 | `release_lead ≥ d_{A→payload} + j` | 释放执行误差 `δ_t ≤ release_lead`，`t_rel` 被履行 |
| **T2** | **`release_lead > d_max`** | **取消窗口 `W = release_lead − d_max > 0`**（否则失败无法拦截） |
| T3 | `handshake_timeout ≥ d_{A→B} + j` | B 等到 ack |
| T4 | `ready_timeout ≥ d_{B→A}` | A 拿到的 ready 新鲜 |
| T5 | `|t_rel^B − t_rel^A| ≤ ε_clock + j` | 时钟同步后的时序一致性 |

> 现行默认：`release_lead=0.2`、`commit_hold_s=0.2`、`handshake_timeout=1.0`、`ready_timeout=0.5`。
> ⇒ 只对 `d_max < 0.2s` 有正的取消窗口。

---

## 3. 取消捕获率（gate 在 lead 窗口内随机失败，无丢包）

`catch = P(t_fail + d ≤ release_lead)`，`d~U(0,d_max)`、`t_fail~U(0,lead)`。

| release_lead \ d_max | 0.05 | 0.10 | 0.20 | 0.40 |
|---|---|---|---|---|
| 0.10 | 0.749 | 0.499 | 0.249 | 0.125 |
| 0.20 | 0.876 | 0.752 | 0.499 | 0.250 |
| 0.50 | 0.950 | 0.901 | 0.801 | 0.600 |
| 1.00 | 0.975 | 0.950 | 0.901 | 0.800 |

**闭式规则**（`lead ≥ d_max` 时）：
```
catch = 1 − d_max / (2·release_lead)
⇒ 达到目标 catch 需   release_lead = d_max / ( 2·(1 − catch) )
```
| 目标 catch | 0.90 | 0.95 | 0.99 |
|---|---|---|---|
| `release_lead / d_max` | 5× | 10× | 50× |

> **发现 1**：取消能力的可靠性**远弱于**直观——要 90% 拦截需要 `lead≈5·d_max`；现行 `lead=0.2` 仅容忍 `d_max≈0.04s`。
> （这是"失败时刻均匀分布"的中性假设；若失败集中在提交后早期，覆盖率更高。）

---

## 4. 丢包下的协议结局（`release_lead=0.2`，各消息独立丢包 p）

| p | 未释放(安全) | 已释放 | **丢 cmd → B 未下潜** | 正常 | abort 丢失 |
|---|---|---|---|---|---|
| 0.00 | 0.000 | 1.000 | 0.0000 | 1.000 | 0.000 |
| 0.05 | 0.096 | 0.950 | **0.0468** | 0.903 | 0.050 |
| 0.10 | 0.189 | 0.901 | **0.0897** | 0.811 | 0.100 |
| 0.20 | 0.360 | 0.801 | **0.1600** | 0.641 | 0.198 |

> **发现 2（危险失效模式）**：`release_cmd` 丢失会导致**"已释放但 B 未下潜"**（捕获失败），
> 随 p=0.05/0.10 达 4.7%/9.0%。⇒ 协议需**冗余/确认重传**（或由 payload_node 侧确认；
> 项目已有 `/payload/released` 可作旁路确认）。
> 其余消息丢失多为**安全失败**（未释放/任务中止）。

---

## 5. 设计建议（可写进论文）

1. **取消窗口**：按 `release_lead = d_max/(2(1−catch))` 设计；给出"延迟预算—取消保证"曲线。
   - 若想容忍 `d_max=0.1s` 且 90% 拦截 → `lead≥0.5s`（这会推迟释放、消耗 C1 余量，需联合权衡）。
2. **丢包**：对 `release_cmd`/`abort` 用**冗余或 ack**；利用已有 `/payload/released` 做 B 侧旁路确认。
3. **常数**：把 `handshake_timeout`、`ready_timeout` 设为 `≥ d_max + j`，而非拍脑袋。
4. **与 C1 联合**：`release_lead` 越大 → 释放时刻离决策越远 → A/B 状态不确定性越大（C1 的 σ 与 `v_A·d`）。
   ⇒ **存在最优 `release_lead`**：权衡"取消可靠性"与"释放精度"。

---

## 6. 与代码的对应

| 协议要素 | 代码 |
|---|---|
| ready / ack / abort | `b_node._pub_ready`、`a_node._coord_tick`、`/payload/release_abort` |
| commit 窗口 | `a_node.commit_hold_s` |
| 时钟同步 | `/coord/ping|pong`（`b_node.clock_offset`） |
| 单次释放 | `a_node._released` 守卫 |
| 下游确认 | `/payload/released`（B 可旁路确认释放已发生） |

## 7. 下一步（W3）

- 在 SITL 注入 **`release_cmd` 丢包**，验证第 4 节的危险失效模式与冗余修复。
- 把 `release_lead` 做成 **`d_max` 自适应**（在线估延迟 → 设 lead）。
- C4（联合机动）与 C5（handover-CBF）随后。
