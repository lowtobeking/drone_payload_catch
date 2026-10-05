# 协同交接基准（Coordination Benchmark）

> 由 `python3 tools/bench_coord.py` 生成。统一大漏斗（`FUNNEL_MOUTH=0.30`）。
> 本次：5 个配置 × 1 次 / 每次 70s。
> `auth`=handshake（A 作释放权威）；`dir`=direct（B 单边）；`i`=intent；`s`=σ；`l`=latency(s)。
> `协调异常` = 非传感器瞬态的 HOLD + PULLBACK（已剔除 `estimator_reset`/`pos_stale`）。

| 配置 | 捕获率 (95% CI) | horiz mean/max | rel_v mean | min\|A-B\| | failsafe | 协调异常 |
|---|---|---|---|---|---|---|
| direct_clean | 1/1 = 100% [21,100] | 0.036/0.036 | 2.84 | 1.104 | 0 | 0 |
| auth_clean | 1/1 = 100% [21,100] | 0.009/0.009 | 2.64 | 1.064 | 0 | 0 |
| auth_noise | 1/1 = 100% [21,100] | 0.036/0.036 | 2.84 | 1.093 | 0 | 0 |
| auth_noise_intent | 1/1 = 100% [21,100] | 0.018/0.018 | 2.76 | 1.076 | 0 | 0 |
| auth_stress | 1/1 = 100% [21,100] | 0.034/0.034 | 2.91 | 1.022 | 0 | 0 |

## 明细（每次）

| 配置#rep | captured | horiz | rel_v | min\|A-B\| | failsafe | HOLD | PULLBACK | 就绪→释放(s) | σ_used |
|---|---|---|---|---|---|---|---|---|---|
| direct_clean#0 | ✅ | 0.036 | 2.84 | 1.104 | 0 | 0 | 0 | 0.21 | nan |
| auth_clean#0 | ✅ | 0.009 | 2.64 | 1.064 | 0 | 0 | 0 | 0.66 | 0.150 |
| auth_noise#0 | ✅ | 0.036 | 2.84 | 1.093 | 0 | 0 | 0 | 0.72 | 0.150 |
| auth_noise_intent#0 | ✅ | 0.018 | 2.76 | 1.076 | 0 | 0 | 0 | 0.41 | 0.150 |
| auth_stress#0 | ✅ | 0.034 | 2.91 | 1.022 | 0 | 0 | 0 | 1.81 | 0.150 |
