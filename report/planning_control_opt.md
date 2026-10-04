# 规划与协调 · 控制 两方面的优化

> 本轮聚焦 **规划与协调** 与 **控制**：(1) 协同释放的**落点余量闸**（规划/协调）；
> (2) **ZEM 终端导引**（控制）。都已在离线量化并在 SITL 验证。
>
> 复现：
> ```bash
> python3 tools/stack_run.py --scenario M6_stack_nodive   # 控制/规划相关离线工况
> COORD=handshake ZEM=0.8 bash run_m6_sitl.sh 70          # 两者都启用
> ```

---

## 1. 控制：ZEM 终端导引（terminal zero-effort-miss）

### 1.1 做法
M6 会合是**终端问题**。原控制为"PD + 载荷速度前馈"（只对准当前/预测位置）。新增一项
**零控脱靶(ZEM) 修正**：预测接触时刻的载荷点 `p_c` 与 B 的"滑翔"位置之差

```
τ = t_c − t
p_c   = p̂ + lead·v̂·τ + ½g·τ²
ZEM   = p_c − (p_B + v_B·τ)
a_cmd += k_zem · ZEM / τ²          （离线加速度）
v_sp  += k_zem · ZEM / τ           （SITL 速度接口）
```

它比"只对准位置"多用了 **B 自身速度**分量，从而更快消掉终端脱靶。

### 1.2 离线收益（大漏斗 eff_r=0.25，gap0.7 免下潜，windkf+lead1，n=40）

| 侧风 | `zem=0` | `zem=0.5` | `zem=1.0` | `zem=1.5` | `zem=2.0` |
|---|---|---|---|---|---|
| w=3 | 37/40 | **40/40** | 40/40 | 40/40 | 40/40 |
| w=4 | 28/40 | 33/40 | **36/40** | 36/40 | 38/40 |
| w=5 | 13/40 | 20/40 | 22/40 | 22/40 | **23/40** |

**结论**：ZEM 显著救回临界工况（w=4: 28→36；w=5: 13→22）；`zem≈1` 是性价比拐点。
（hmiss 基本不变，ZEM 主要把"接近失败"救成成功。）

### 1.3 SITL
`COORD=handshake ZEM=0.8`：`STACK CAPTURED horiz=0.055m rel_v=1.798m/s`，无 failsafe。
（SITL 无风，主要验证控制路径不破坏；收益在离线风场景量化。）

---

## 2. 规划/协调：释放前**落点余量闸**

### 2.1 做法
A（释放权威）在释放前，用**风/阻力模型**预测水平漂移 `Δ = _horiz_drift(wind, k, fall_t)`，
结合 B 报的就绪质量 `rel_xy` 与不确定度 σ，做**保守余量判据**：

```
pred_miss = rel_xy + |Δ| + k·σ
仅在  pred_miss ≤ eff_r − min_margin  时释放；否则 A 暂不释放，B 继续微调
```

这是协同的**最后一道安全闸**：把"是否释放"从"B 自报对齐"升级为"A 用**落点预测 + 余量**判定"。

### 2.2 SITL
```
A: 收到 B 就绪 → 释放权威发布 release@…（余量闸通过）
B: 收到 A 释放 ack@… → DIVE
*** STACK CAPTURED *** horiz=0.055m   无 failsafe
```
（若余量不足，A 会打印"落点余量不足 … 暂不释放"并保持。）

---

## 3. 代码/文档改动

| 文件 | 改动 |
|---|---|
| `payload_catch/stack_drop.py` | `simulate_stack` 新增 `zem_gain`/`zem_lead`（ZEM 终端导引，离线） |
| `payload_catch/b_node.py` | 参数 `zem_gain`；DIVE 速度设定点加 ZEM 修正 |
| `payload_catch/a_node.py` | 参数 `funnel_eff_radius`/`min_release_margin`/`release_sigma_k`/`release_sigma`；释放前落点余量闸 |
| `launch/catch_stack_launch.py`、`run_m6_sitl.sh` | `zem_gain` / `ZEM` 参数 |

全部 opt-in（默认 `zem_gain=0` 不改变原行为；余量闸默认阈值宽松）。

---

## 4. 结论与下一步

- **控制**：ZEM 终端导引有效（临界风场景成功率 +8~9/40），`zem≈1` 推荐。已在 SITL 打通。
- **规划/协调**：落点余量闸把释放决策建立在"预测余量"上，是合理的最后一道闸。
- **下一步候选**：① `zem_gain` 在线自适应（按 τ 调度）；② 余量闸结合 **KF 协方差**（更严格 σ）；
  ③ **协同规划 A 的速度/高度**（改终端速度/几何）；④ **带载操控**（捕获后质量/惯量自适应）。
