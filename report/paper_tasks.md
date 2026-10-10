# 论文任务追踪（Paper Task Tracker）

> 锚点文档：`report/paper_outline.md`（路线图 `research_roadmap.md`）。
> **这是活文档**：每次推进后更新状态、证据、日期。图例：✅ 完成 · 🔵 进行中 · ⏳ 待办 · ❌ 阻塞。
> 最后更新：**2026-10-09**

---

## 0. 总览

| 阶段 | 内容 | 状态 | 完成度（估） |
|---|---|---|---|
| W1 | 立论 / 相关工作 / 形式化 | ✅ | 100% |
| W2 | C1 概率证书 + 离线验证（含相对 σ 修正） | ✅ | 100% |
| W3 | C2/C3 消融 + 消融主表 | ✅ | ~95% |
| W4 | C5 handover-CBF + T1/T2/T3 | ✅ | ~95% |
| 附加 | 基准/负结果/统计+基线/感知/动力学 | ✅ | ~90% |
| **W5** | **真机搭建 + 实验**（唯一投稿硬门槛） | ❌ | **~15%**（仅软件接入层） |
| W6 | 写作 + 投稿 | ⏳ | ~10%（仅骨架） |
| — | 决策 D1–D4 | ⏳ | 阻塞 |

**一句话**：**理论+仿真+统计已基本就绪；卡在「真机（W5）」与「写作（W6）」，外加 4 个待定决策。**

---

## 1. 待决策（阻塞项，需作者拍板）

| ID | 决策 | 选项 | 影响 | 状态 |
|---|---|---|---|---|
| D1 | 目标 venue | 顶会(ICRA/IROS) / 期刊(RA-L/TRO) | 决定真机与理论深度 | ⏳ |
| D2 | 真机条件 | 室内动捕 / RTK-GNSS / UWB / 机载视觉 | 决定相对感知方案 | ⏳ |
| D3 | 机队 | 2 架 x500 级 / 其他 | 成本与周期 | ⏳ |
| D4 | 贡献聚焦 | C1 / C1+C3 / C1+C2+C3+C5 | 写作范围 | ⏳ |

> 建议先定 **D1+D2+D3**（三者共同决定真机怎么搭）。

---

## 2. 理论任务

| ID | 任务 | 状态 | 证据 / 产出 |
|---|---|---|---|
| T1.1 | 问题形式化（系统/通信/捕获事件/优化） | ✅ | `paper_outline.md` §4 |
| T1.2 | C1 概率捕获证书（精确 Rice / Marcum-Q） | ✅ | `coordination_probability.md`、`payload_catch/coord_cert.py`、`tools/coord_prob.py` |
| T1.3 | **相对不确定度模型**（修"绝对 σ 当相对 σ"） | ✅ | `payload_catch/uncertainty.py`、`report/rel_uncertainty.md` |
| T2.1 | T1 释放域最优性（中心球） | ✅ | `coordination_theory.md`、`tools/coord_optimal.py` |
| T2.2 | C2 协议时序界 `release_lead=d_max/(2(1−catch))` | ✅ | `coordination_protocol.md`、`tools/coord_proto.py` |
| T2.3 | C3 state vs intent（延迟容忍） | ✅ | `coordination_information.md` |
| T2.4 | C4 联合机动 | ✅ | `coordination_maneuver.md`、`tools/coord_maneuver.py` |
| T2.5 | C5 handover-CBF + T2 联合证书 + T3 延迟 CBF | ✅ | `coordination_cbf.md`、`coordination_theory.md`、`tools/coord_cbf.py` |
| T3.1 | 动力学聚合约束（倾角+推力）纳入形式化 | ✅ | `payload_catch/dynamics.py`、`report/dynamics_contact.md` |
| T3.2 | 接触冲击/可恢复性/带载余量纳入安全叙述 | ✅ | `payload_catch/impact.py`、`report/dynamics_contact.md` |
| T3.3 | 感知模型（相机）纳入 | ✅ | `payload_catch/perception.py`、`report/perception_study.md` |
| T3.4 | 证明/假设形式化为可发表物（符号统一、假设列表） | ⏳ | 待写作时补 |

---

## 3. 实验任务

| ID | 任务 | 状态 | 证据 / 产出 |
|---|---|---|---|
| E1.1 | 离线全工况（M1–M4） | ✅ | `tools/offline_run.py --all`（全 PASS） |
| E1.2 | M6 蒙特卡洛（大 N） | ✅ | `tools/stack_run.py --mc` |
| E1.3 | 消融主表（标称 + stress，含 CI） | ✅ | `coordination_experiments.md`、`tools/bench_coord.py` |
| E1.4 | **统计+基线**（Wilson CI + 配对 McNemar + 参数不确定性） | ✅ | `report/statistics.md`、`payload_catch/stats.py`、`tools/stats_report.py` |
| E1.5 | 感知影响量化（相机 vs 真值替身） | ✅ | `report/perception_study.md`、`tools/perception_study.py` |
| E1.6 | 托盘鲁棒（倾斜/侧风/难度扫描/MC） | ✅ | `report/m6_tray_robustness.md`、`report/m6_tray_sitl_*.md` |
| E2.1 | 复现 1–2 个**文献基线**做对比 | ⏳ | 尚未复现（评审会要） |
| E2.2 | SITL 大 N（≥20/档，附 CI，后台批量） | ⏳ | 脚本已附 CI（`mc_m6_sitl*.sh`），待跑 |
| E2.3 | 高保真/参数灵敏度（Pareto） | ⏳ | 未做 |

---

## 4. 真机任务（W5，硬门槛）

| ID | 任务 | 状态 | 证据 / 依赖 |
|---|---|---|---|
| H0.1 | 相对定位驱动（RTK→`/drone_a/state`） | ✅（软件） | `payload_catch/relnav*.py`；待真机标定 |
| H0.2 | 接触检测（力/微动 + 加速度/速度反转） | ✅（软件） | `payload_catch/contact_detect.py` |
| H0.3 | 真机 launch + bring-up 指南 | ✅（软件） | `launch/catch_real_launch.py`、`report/real_hardware_bringup.md` |
| H0.4 | 泡棉恢复系数 e 实测工具 | ✅（工具） | `tools/foam_drop_test.py`、`tools/gz_foam_drop_test.sh` |
| H1 | 硬件搭建（2×x500 + RTK + 释放机构 + 托盘/锁扣） | ⏳ | 依赖 D2/D3 |
| H2 | 地面标定（相对定位精度、杆臂、原点） | ⏳ | 见 `real_hardware_bringup.md` §1/§4 |
| H3 | 地面落物台（泡棉 e、释放机构行程、接触开关） | ⏳ | |
| H4 | 单机 B 定点捕获（接触→锁扣→带载悬停） | ⏳ | |
| H5 | 双机 M6（定点）→ M6-moving | ⏳ | |
| H6 | 验证 C1/C2/C5（至少）+ 记录指标（CI） | ⏳ | |
| **H7** | **真机 或 HIL 验证** | ⏳ | HIL 备选（真机不可行的兜底） |

> HIL（真飞控 + 仿真传感器）可作为 H1–H6 前的低成本桥。

---

## 5. 写作与投稿（W6）

| ID | 任务 | 状态 | 产出 |
|---|---|---|---|
| P1 | 题目 / 摘要 / 贡献清单定稿 | ⏳ | `paper_outline.md` §1–3（草案） |
| P2 | 方法与形式化章节（统一符号 + 假设） | ⏳ | |
| P3 | 实验章节（表格/图，含 CI） | ⏳ | 素材：`statistics.md`、`coordination_experiments.md`、各 `report/` |
| P4 | 相关工作（核实引用） | ⏳ | `survey_and_sim_report.md` §6 初稿 |
| P5 | 图表整理（矢量图 + 统一风格） | ⏳ | `report/figures/` |
| P6 | 初稿 | ⏳ | |
| P7 | 内审 / 修改 | ⏳ | |
| P8 | 投稿 | ⏳ | |

---

## 6. 工程/可复现（支撑项）

| ID | 任务 | 状态 | 证据 |
|---|---|---|---|
| R1 | 单一真值源配置 | ✅ | `config/catch_scenarios.yaml` |
| R2 | 一键复现（离线/SITL/报告） | ✅ | `SIM_COMMANDS.md`、`tools/*` |
| R3 | Word 仿真报告生成 | ✅ | `tools/make_docx_report.py` |
| R4 | Agent Skill 封装 | ✅ | `skills/aerial-payload-handover/` |
| R5 | 回归自检 + CI（把 `offline_run --all` / 自测固化） | ✅ | **`tools/run_checks.sh`**（`--quick`/全量，自动 source env，失败非 0；23/23）；`tests/`+`pytest.ini`；`Makefile` + pre-commit；`tools/sitl_check.sh`（SITL 验收）；`.github/workflows/checks.yml`（push/PR + 多 Python 版本；待首个 PR 验证） |
| R6 | 开源仓库同步 | ✅ | GitHub `lowtobeking/drone_payload_catch` |

---

## 7. 风险 / 阻塞

| 风险 | 影响 | 缓解 |
|---|---|---|
| 真机不可行 | 顶会/RA-L 难送审 | 兜底：基准/负结果论文；或 **HIL** |
| 相对定位精度不足（σ 需 ≪ eff_r≈0.12–0.17m） | 捕获率骤降、证书不可行 | RTK/UWB 双差 + 高 ρ；`rel_uncertainty.md` |
| 平台 failsafe（`v_p≈4.9 m/s`） | 高动态受限 | 修 B 的 EKF/磁罗盘 或 降速 |
| SITL 样本量小（N=5） | CI 宽、统计弱 | 用本仓 `stats.py` 报告 CI；SITL N≥20/档 |
| 写作时间 | 投稿延期 | 先定 D1，写作与真机并行 |

---

## 8. 变更日志（进度追踪）

| 日期 | 变更 |
|---|---|
| 2026-10-05 | 进度快照 v1（理论 C1–C5/T1–T3、系统、SITL 消融、基准）|
| 2026-10-09 | 补齐：相对不确定度模型、动力学/接触、感知/风保真、统计+基线、真机接入层、Agent Skill；建立本追踪文档 |
| 2026-10-09 | **离线自检套件**（参考 `drone_package_20260908`）：抽 `safety_logic.py`、`tools/test_{coord_cert,keepout,safety_logic,config,purity,compileall}.py`、`tools/smoke_acados.py`、`tools/sitl_check.sh`、一键 `tools/run_checks.sh`+`Makefile`+pre-commit（23/23，pytest 7）、`tests`+GitHub CI；修 `mpc_terminal` 自测解包 bug + 硬化 `sim_core`/`stack_drop` 自测 |
| 2026-10-09 | **起飞前自检**：`tools/preflight_check.py`（离线/`--sitl`/`--live`/`--logs` 日志门，致命 vs 告警分级）+ `tools/test_preflight.py`（纯逻辑单测）；`run_m6/m1_sitl.sh` 加 `PREFLIGHT=1` live+logs 门（默认关）；全量自检 24/24（pytest 8） |
| （下次） | 更新 D1–D4 决策、文献基线复现、SITL 大 N、真机/HIL |

---

## 9. 下一步（按价值）

1. **定 D1–D3**（venue/真机条件/机队）——解锁 W5 与写作深度。
2. **复现 1–2 个文献基线**（E2.1）——评审刚需，不依赖真机。
3. **SITL 大 N**（E2.2）——用已附 CI 的脚本后台跑。
4. **真机/HIL**（H1–H7）——硬门槛；HIL 先行。
5. **论文初稿**（P1–P6）——与真机并行推进。
