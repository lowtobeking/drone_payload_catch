# 物理与判据（references）

坐标：世界系 NED（x=北, y=东, z=下），高度 = −z。

## 1. 载荷抛体
```
p_p(τ) = p_r + v_r·τ + ½g·τ²,   v_p(τ) = v_r + g·τ        (τ = t − t_r)
```
- 释放瞬间载荷**继承 A 的速度** `v_r = v_A(t_r)`；
- 可选 linear/quadratic 阻力 + 风（`payload_model.py`）。

## 2. 会合规划（M1–M4）
在 `(t_r, τ_c)` 网格上最小化
```
J = w_time·t_r + w_accel·(峰值加速度/a_max) + w_vel·|Δv|²  (+ w_overshoot·过冲)
```
B 的会合轨迹：双积分器、两端位置/速度、`min∫|a|²dt` 的**解析三次多项式**；
终端速度不可行时退**软终端速度**。

## 3. M6 垂直堆叠投放（定点）
```
v_rel = √( 2·(g − a_dive)·gap )          # 接触相对速度下界（B 只能往下压）
v_retain = √(2·g·depth) / e               # 刚性漏斗保持速度（e=恢复系数）
eff_r = mouth_radius − object_radius      # 口内有效半径
```
- `a_dive=0`（悬停接）时 `v_rel=√(2g·gap)`；
- `gap ≤ v_retain²/(2g)` 时**免下潜**（横风下漂移最小）。

## 4. 圆形托盘（无杯深）
```
保持条件：e²·gap ≤ h        (h = 围边高 + 凹垫深)
eff_r   = 盘内半径 − 物半宽
```
- `e` 是生死线：裸塑料 e≈0.7 必弹飞；软泡棉 e≤0.15 稳定。

## 5. 释放证书（C1，决策层）
```
m ~ N(μ, σ_m² I₂)                       # 二维水平脱靶
P(‖m‖ ≤ R) = 1 − Q₁(μ/σ_m, R/σ_m)       # Rice / Marcum-Q
释放判据：‖δ̂‖ ≤ T(ε),  T 由 P(‖m‖≤r_eff−margin | μ=T)=1−ε 唯一确定
```
- **σ_m 必须是相对定位 σ**，非绝对（见 `uncertainty.py`）：
  ```
  σ_rel² = σ_A² + σ_B² − 2ρσ_Aσ_B + (l·σθ)² + σ_meas²
  ```
  `ρ`=公共误差相关系数（同源 RTK 大），`l`=杆臂，`σθ`=姿态误差。
- 相对传感器架构（RTK 双差/UWB）：`σ_rel` = 传感器 σ，与绝对 eph 无关。

## 6. 动力学（四旋翼聚合约束）
```
tanθ = a_x/(g − a_z) ≤ tanθ_max          # 倾角
(T/m) = √(a_x² + (g−a_z)²) ≤ T_max/m     # 推力
⇒ 水平权限 a_x ≤ (g − a_z)·tanθ_max       # 随下潜衰减
a_max = g·tanθ_max 是姿态约束的投影（6 m/s² ↔ ~31.5°）
```

## 7. 接触冲击（安全）
```
J = m_p·v_rel                            # 冲量
F_peak ≈ ½ m_p v_rel² / s                # 峰值力（缓冲行程 s）
ω ≈ J·d_off / I_B                        # 偏心角速度
可恢复：ω ≤ min(τ_max·t_rec/I_B, ω_max)
带载悬停：T_max ≥ (m_B + m_p)·g
```
