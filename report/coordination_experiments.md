# 协调实验主表（逐项消融，SITL）

> 论文实验章节。统一大漏斗 `FUNNEL_MOUTH=0.30`，SITL，`tools/bench_coord.py --grid ablation [--stress] --reps 3`。
> 指标：捕获率(95% CI)、水平脱靶 horiz、相对速度、`min‖A−B‖`、**就绪→释放时延**、协调异常、failsafe。

配置（自 baseline 逐项叠加）：
```
T0_direct : direct（B 单边决定释放，启发式，无 intent，启发式 keepout）
T1_auth   : + handshake（A 作释放权威）
T2_cert   : + 证书闸（相对 σ + 精确 Rice，C1）
T3_intent : + 意图共享（预测落点，C3）
T4_cbf    : + CBF 防碰（keepout_mode=cbf，C5）
```

---

## 1. 标称消融（无额外干扰）

| 配置 | 捕获率 | horiz mean/max | 就绪→释放(s) | failsafe | 协调异常 |
|---|---|---|---|---|---|
| T0_direct | 3/3 [44,100] | 0.027/0.040 | 0.21 | 0 | 0 |
| T1_auth | 3/3 [44,100] | 0.086/0.199 | 0.40–0.46 | 0 | 0 |
| T2_cert | 3/3 [44,100] | 0.031/0.041 | 0.40–0.41 | 0 | 0 |
| T3_intent | 3/3 [44,100] | 0.034/0.055 | 0.40–0.43 | 0 | 0 |
| T4_cbf | 3/3 [44,100] | 0.062/0.131 | 0.41–0.43 | 0 | 0 |

> 标称下**全部成功**：协调特性不降低标称性能（一致性），差异须在 stress 下显现。

---

## 2. Stress 消融（`rel_pos_sigma:=0.08 rel_latency:=0.15`）★主表

| 配置 | 捕获率 | horiz mean/max | **就绪→释放(s)** | σ_used | failsafe |
|---|---|---|---|---|---|
| T0_direct | 3/3 [44,100] | 0.017/0.025 | 0.20 | – | 0 |
| T1_auth | 3/3 [44,100] | 0.022/0.042 | **1.69 / 2.75 / 5.51**（均≈3.3） | 0.150 | 0 |
| T2_cert | 3/3 [44,100] | 0.041/0.056 | **0.41 / 1.26 / 0.49**（均≈0.72） | 0.05 | 0 |
| T3_intent | 3/3 [44,100] | 0.030/0.042 | **0.43 / 0.40 / 0.42**（均≈0.42） | 0.05 | 0 |
| T4_cbf | 3/3 [44,100] | 0.036/0.046 | 0.43 / 0.43 / 0.40（均≈0.42） | 0.05 | 0 |

**逐项边际收益（应力下）**：

| 增量 | 效果 |
|---|---|
| T0→T1（权威） | 释放决策由 A 用精确自身状态（安全/一致性↑），但启发式用**绝对 σ=0.150** ⇒ 阈值极小 ⇒ 等待 ~3.3s |
| **T1→T2（证书闸）** | 改用**相对 σ≈0.05** + 精确阈值 ⇒ 等待 ~0.72s ⇒ **时延 ↓4.6×** |
| **T2→T3（意图）** | 预测式对正 ⇒ 等待收敛到 ~0.42s ⇒ **再 ↓1.7×**，且方差小（0.40–0.43） |
| T3→T4（CBF） | 时延不变（≈0.42s）——CBF 是**安全**特性，**无性能代价** |

> **结论**：协调的收益在**应力下**显现，且**可逐项归因**——正是论文需要的"消融证据链"。
> 全部配置 0 failsafe、0 协调异常，说明安全层（C5/C2）不引入退化。

---

## 3. 专项实验（支撑贡献）

| 实验 | 命令 | 结论 |
|---|---|---|
| **证书闸 vs 启发式** | `bench_coord.py --grid gate --reps 2` | 噪声下时延 2.4s→0.42s（≈5×） |
| **T1 最优性** | `coord_optimal.py` | 中心球最优（释放概率 0.3448 > 方形 0.3191 > 偏心球 0.1715） |
| **T3 延迟 CBF** | `coord_cbf.py` | naive 在 d≥0.2s 违反（0.739<0.8）；robust 全程安全 |
| **C1 证书可行性** | `coord_prob.py` | 标准漏斗 rel_σ≥0.10 无法认证；大漏斗可 |
| **C3 state vs intent** | `coord_prob.py`（C3 段） | intent 延迟不变；state 随 v_A·d 崩 |
| **C4 联合机动** | `coord_maneuver.py` | v_A=2 默认 0.019 → 联合最优 0.114（≈6×） |
| **全协议不变量** | `validate_coord.py` | 就绪→释放→ack 顺序、单次释放、min‖A−B‖≥1.0、无 failsafe |

## 4. 复现

```bash
source ~/drone_payload_catch/env.sh
MODE=full bash run_m6_sitl.sh 70                                 # 研究特性全开（握手+证书闸+CBF+intent+T3）
python3 tools/bench_coord.py --grid ablation --reps 3            # 标称
python3 tools/bench_coord.py --grid ablation --stress --reps 3   # stress（主表）
python3 tools/coord_optimal.py --n 800000 --sigma0 0.25          # T1
python3 tools/coord_cbf.py                                       # T3
python3 tools/coord_prob.py --n 200000                           # C1/C3
python3 tools/coord_maneuver.py --wind 0 --n 12                  # C4
python3 tools/validate_coord.py                                  # 协议不变量
```

## 5. 局限

- 全部为 **SITL**（无真机）；相对定位为**真值替身 + 注入噪声**。
- 标称下各配置无差异；收益在 stress 下——需在论文中明确"应力条件"。
- 真机是最终门槛（见 `report/paper_outline.md` §8）。
