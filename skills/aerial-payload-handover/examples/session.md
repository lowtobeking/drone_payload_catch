# 使用示例（examples）

## 场景：评估"6cm/100g 方块、A 悬停 4.5m、B 在下方 3.5m 用圆形托盘接"是否可行

**用户**：我想用无人机 A 空投一个 6cm/100g 方块，B 在下方用塑料圆形托盘（带围边）接住，行不行？

**Agent（用本技能）**：

1. 明确输入：载荷 6cm/100g；gap≈1.0m；末端=托盘；相对定位 σ≈0.05m。
2. 选末端尺寸并选型：

```bash
bash scripts/size_tray.sh --diameter 0.30 --rim 0.05 --e 0.15 --obj 0.06 --gap 1.0
```
→ `eff_r=12cm`，`v_retain=6.60 m/s`，`e²gap=2.2cm ≤ 围边 5cm` → **PASS**。

3. 跑蒙特卡洛：

```bash
bash scripts/run_m6_mc.sh M6_stack_tray 300
```
→ 成功率 ≈ 99%（释放误差 σ=0.05）。

4. 评估释放误差敏感性 / 证书：

```bash
bash scripts/check_release_cert.sh
```
→ 结论：**相对定位 σ 必须远小于 eff_r**；绝对 σ 会让证书不可行。

5. （可选）动力学/接触与报告：

```bash
bash scripts/analyze_dynamics_contact.sh
bash scripts/make_report.sh
```

**Agent 回答**：可行。用**内径 30cm、围边 5cm、泡棉 e≤0.15** 的圆形托盘，定点投放离线成功率 ≈99%。
关键前提：① 泡棉实测 e≤0.20；② 相对定位 σ≪0.12m；③ 若要运动中交接，用大托盘(40cm)或主动锁扣。

---

## 场景：泡棉选型

**用户**：我从 1m 高把方块落到泡棉上，回弹约 2cm，够好吗？

**Agent**：
```bash
bash scripts/measure_foam_e.sh --h-drop 1.0 --rebounds 0.020 0.024 0.018
```
→ `e≈0.15`（PASS，目标 ≤0.20）；推荐 `TRAY_E=0.15`，`gap=1m` 回弹仅 2.3cm < 围边 5cm。

---

## 场景：为什么我不能保证 95% 捕获？

**用户**：我只有单点 GPS，绝对精度 ~0.15m，能保证吗？

**Agent**：用 `check_release_cert.sh` 显示，把绝对 σ（0.21m）当相对 σ 时，
证书 `T=0`（**不可行**）。需要：换成 **RTK/UWB 相对定位**（σ≈0.03m）或利用公共误差抵消（ρ 大）。
详见 `references/physics.md` §5。
