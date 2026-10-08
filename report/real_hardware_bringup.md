# 真机接入指南（Bring-up）

> 把已验证的仿真栈接到真机。**不改现有离线/SITL 层**，只在其上"补一层"：
> 真实相对定位（RTK/UWB/视觉）+ 真实载荷感知 + 释放/末端/接触 + 标定。
> 配套：`launch/catch_real_launch.py`、`run_m6_real.sh`、`payload_catch/relnav*.py`、
> `payload_catch/contact_detect.py`、`config/catch_scenarios.yaml` 的 `real:` 段。

---

## 0. 复用 vs 新增

| 层 | 复用（不动） | 新增（真机） |
|---|---|---|
| 算法 | 抛体/会合/M6/托盘/KF/协调 | — |
| 控制接口 | `px4_iface`（PX4 offboard） | — |
| 相对定位 | `/drone_a/state` 接口 | **relnav_node**（RTK 驱动，替换真值替身） |
| 载荷感知 | `/payload/state` 接口 + `BallisticDragKF` | 真实视觉/动捕/UWB tag（或弹道预测兜底） |
| 捕获 | 捕获判据/锁扣逻辑 | **contact_detect**（真实接触事件）+ 释放舵机/夹爪 |
| 坐标 | NED 约定 | **标定**（杆臂/原点/几何） |

---

## 1. 相对定位（RTK）——**成败单点**

### 1.1 原理与接口
`/drone_a/state = [t, pA_world_NED(3), vA_world_NED(3)]`。真机由 `relnav_node` 发布：
- RTK 大地坐标 → 公共本地 NED（小基线平面近似）；
- **杆臂补偿**：RTK 天线 ≠ 末端/托盘，用姿态把机体系偏移旋到世界系（`relnav.lever_arm_world`）；
- **相对化**：`p_A_world = p_B_px4_world + (p_A_ref − p_B_ref)`，**与 RTK 公共原点无关**。

b_node 的 `rel_*` 噪声参数置 0 即用它（真机不再注入替身噪声）。

### 1.2 启动（真机）
```bash
ros2 run payload_catch relnav_node --ros-args \
  -p source:=rtk -p a_rtk_topic:=/rtk/a -p b_rtk_topic:=/rtk/b \
  -p rtk_type:=navsatfix -p use_geodetic:=true \
  -p a_lever:="[0.0,0.0,0.2]" -p b_lever:="[0.0,0.0,-0.21]"
```
（或直接用 `catch_real_launch.py`。）

### 1.3 必须标定/验证
1. **杆臂**：天线安装偏心（机体系 x前/y右/z下），实测回填 `a_lever/b_lever`。
2. **公共原点/世界系**：`origin_lat/lon/alt`（全 0 取首个 fix）；B 的 PX4 本地原点在 relnav 世界系的
   偏移 `b_world_offset`。**做法**：把两机并排放在已知点，比较 relnav 输出与卷尺距离。
3. **精度/延迟**：静态并排测相对位置误差 σ、动态跑测延迟。**验收：σ ≪ `eff_r`（0.12–0.17m）**，
   建议 σ≤0.03m、延迟≤20ms。
4. **时间同步**：两机时间戳对齐（PTP/PPS 或 `/coord/ping|pong` 已实现）。

> 若 RTK 精度不足或室内无星：换 **UWB / 动捕 / 机载视觉**（同一接口，改 `rtk_type/source`）。
> 参考：离线/`tools/coord_*` 已量化"相对 σ 越小，成功/证书余量越大"。

---

## 2. 载荷状态感知（释放后）

仿真里 `/payload/state` 来自 Gazebo 真值。真机需要：
- **首选**：B 上视相机 / 动捕 / 载荷 UWB tag（实时位置+速度）；
- **兜底**：**弹道预测**（用 A 的释放点+速度，`payload_model`），低风/短下落可用；
- 接口不变：发 `/payload/state = [t, p(3), v(3)]`（世界 NED）。

> 注意：`track_payload` 在释放误差大时是决定性的（SITL 5/5 vs 1/5）——真机尽量上感知；
> 若只有弹道预测，需保证释放误差小 + 风小。

---

## 3. 释放机构 + 末端 + 接触检测

### 3.1 末端（已在仿真验证）
- 托盘：内径 30cm（`x500_tray`）/ 40cm（`x500_tray_big`，运动交接用），围边+泡棉；
- **泡棉 e 必须实测**：`python3 tools/foam_drop_test.py --h-drop 1.0 --rebounds ...`（见
  `report/m6_tray_robustness.md` §7），判据 **e≤0.20**；实测值回填 `TRAY_E`。
- 若要"接住→带走"稳：加**主动锁扣**（真机=夹爪/磁吸/闭爪；仿真已验证，见同报告 §6）。

### 3.2 释放机构
- A 端舵机/电磁铁/机械锁扣；触发由 `/payload/release_at`（已实现）映射到 actuator/GPIO。
- 释放点偏移 `payload_release_offset`（避开机体/桨）。

### 3.3 接触检测（替代软件判据）
`contact_detect.py` 融合：**外部力/微动/对射**（`/payload/contact`）、B 垂向加速度尖峰、相对速度反转。
`catch_real_launch.py` 已默认 `contact_detect:=true`，进入捕获窗口后无接触有 0.3s 兜底。
真机建议装一个**力开关或对射**，最可靠。

---

## 4. 标定清单（实测回填 `config.real`）

| 项 | 说明 | 方法 |
|---|---|---|
| `a_lever` / `b_lever` | 天线→末端 机体系偏移 | 卷尺 + 机体系定义 |
| `origin_*` / `b_world_offset` | 公共 NED 原点/机间原点 | 并排已知点比对 |
| `px4_z_bias` | PX4 z 与模型高度差 | 已知高度悬停比对 |
| `funnel_mount_height` | 托盘面/漏斗口相对 base_link | 实测 |
| `payload_release_offset` | 释放点相对 A | 实测（避桨/起落架） |
| `TRAY_E` / `v_retain` | 泡棉恢复系数/保持速度 | 落物试验（§3.1） |

---

## 5. Bring-up 顺序（低风险→高风险）

1. **地面**：RTK 相对定位静态精度；泡棉 e 落物；释放机构行程；接触开关触发。✅
2. **单机 A**：悬停 + 释放（载荷落网/垫）。
3. **单机 B**：定点捕获（人工/机械释放），验证接触检测→锁扣→带载悬停。
4. **双机 M6（定点）**：`catch_real_launch.py`（悬停投放）。
5. **双机编队（运动）**：大托盘 + 锁扣。
6. **指标记录**：捕获水平偏差、相对速度、min‖A−B‖、是否 failsafe。

---

## 6. 安全

- 安全员 + **RC 接管**；`/safety/kill_a|b`（飞行终止，已实现）；
- 围栏/高度限制（`safety_geofence_*`）；桨保护、隔离区；
- **平台边界**：SITL 里 B 在 `v_p≈4.9 m/s` 触发姿态 failsafe（平台层）——真机高动态需重新评估 EKF/姿态；
- 载荷未接住时自由落体砸地：需地面/网兜防护（真机应急未做，注意）。

---

## 7. 尚未解决 / 需真机补

- 真实感知下`/payload/state`的**延迟/遮挡**未在真机验证；
- 带载质量/惯量突变的自适应控制（未做）；
- 风/下洗/气动的真实标定（仿真偏乐观：模型泡棉 e≈0）；
- 适航/认证。

> 结论：**算法/控制/接口层已就绪，真机差的是"感知真实化 + 机构 + 标定"**（本文 1–4 节）。
> 其中 **RTK 相对定位精度 vs `eff_r`** 是唯一决定成败的关键。
