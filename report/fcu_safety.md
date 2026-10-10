# 飞控安全：参数体检 + 电池保护（对标 drone_package_20260908）

> 参考工程的安全核心是**飞控（PX4）参数体检**（`tools/fc_configure.py` 分组 + 知识库血泪教训）
> 与**失效保护分层**。本轮把这块补进本工程：飞行前查参数、飞行中管电池。
> 相关：`report/safety_control_review.md`（旧缺口清单）、`report/safety_supervisor.md`（kill）。

---

## 0. 参考工程提炼出的判据（写进 `payload_catch/fcu_params.py`）

**贯穿原则**：航向/位置精度**在飞行中验证之前**，不启用任何"自主飞一段距离"的失效动作
⇒ 失效保护选 **Land**（`NAV_RCL_ACT=3`、`COM_OBL_RC_ACT=4`），不选 Return。

| 关系 | 为什么 | 级别 |
|---|---|---|
| `MPC_THR_MIN < MPC_THR_HOVER` | 否则"油门拉到底也降不下来"（`THR_MIN≥THR_HOVER` 卡死） | fail |
| `COM_RCL_EXCEPT` 不含 bit2(4) | 含 4 → OFFBOARD 中 RC 失联被豁免＝拆掉最后一道闸 | fail |
| `RTL_RETURN_ALT ≤ GF_MAX_VER_DIST` | 否则 RTL 爬升撞破围栏、两个失效保护打架 | fail |
| 电池阈值 `EMERGEN ≤ CRIT ≤ LOW` | 阈值次序错 → 告警/动作错乱 | fail |
| `MPC_XY_VEL_MAX ≥ 2.0` | 否则 companion 指令被飞控裁掉 | fail |
| 围栏启用 `GF_MAX_* > 0` | `0`=未启用，无外层兜底 | fail（实机） |
| `COM_OBL_RC_ACT=4` / `NAV_RCL_ACT=3` | offboard/RC 失联 → Land（非 Return） | fail（实机） |
| `COM_LOW_BAT_ACT ∈ {2,3}` | `0`=None 会飞到没电 | fail（实机） |
| `COM_OF_LOSS_T ≤ 0.5` | offboard 失联响应超时 | warn |
| `SENS_BOARD_ROT` 与实物一致 | 参考机改错会**起飞直接翻** | warn |
| EKF2 源一致性（动捕 `0/11/0/5` vs RTK `7/0/1/0`） | 高度源混用会冲突 | fail（实机） |

## 1. 工具

| 工具 | 作用 |
|---|---|
| `payload_catch/fcu_params.py` | **纯逻辑**：`parse_param_dump` / `check_params(profile)` / `PRESETS`（分组预置）+ 自测 |
| `tools/preflight_params.py` | 飞行前检查：`--file`（param dump）/`--mavlink`（pymavlink 直连）/`--list`/`--dump`/`--json` |
| `tools/fcu_configure.py` | **配置器**（默认 dry-run，`--apply` 才写；写前读原值推断类型、写后回读校验） |
| `tools/test_fcu_params.py` | 纯逻辑单测（多格式解析 + 全部判据 + 预置自洽） |

接入 `preflight_check.py --params [--param-file F | --param-mavlink CONN] [--param-profile P]`。

## 2. 飞行中：电池保护

- `payload_catch/safety_logic.py`：`battery_land_reason()` / `battery_warn_reason()`（纯逻辑）。
- `payload_catch/px4_iface.py`：订阅 `/fmu/out/battery_status`（**1.16 已桥接**）；
  `_safety_update` 里 `warning≥2` 或 `remaining<0.07` → 直接 **LAND**（优先级高于其它恢复态）。
  参数 `battery_land_enable`(默认 True)、`battery_critical_remaining`(0.07)。
- `tools/live_probe.py`：起飞前也查电池（`warning≥2`/`remaining<0.07` → fail，低电 → warn）。

## 3. 验证

**离线**：`test_fcu_params`（解析/判据/预置）、`test_safety_logic`（电池→Land）、`test_live_probe`（电池门）全绿。

**SITL 实跑**（`PREFLIGHT=1`）：
- `live_probe` 电池项 `warning≤low ✅`、`remaining≥0.15 ✅`（min=1.00）——无误报。
- `tools/preflight_params.py --mavlink udpout:127.0.0.1:18570`（A）/`18571`（B）实读飞控参数 → `fail=0`。
  （修复：udpout 需**主动发 GCS 心跳**让 PX4 学到客户端地址，否则 no heartbeat。）
- 任务 `*** STACK CAPTURED ***`，0 failsafe。

```bash
# 飞行前（SITL）
PREFLIGHT=1 PREFLIGHT_PARAMS=1 bash run_m6_sitl.sh 70
# 手动
python3 tools/preflight_params.py --mavlink udpout:127.0.0.1:18570 --profile sitl
python3 tools/fcu_configure.py -g failsafe,limits_indoor            # dry-run
```

## 4. 现状 vs 缺口

本轮补上：**S1 飞控参数体检**、**S2 电池→Land**、**S3 失效保护/围栏预置 + 校验**（配置器）。
仍缺（见 `report/safety_control_review.md`）：真机/HIL 验证、接空失败物理防护、跨机冗余、
`SENS_BOARD_ROT` 等需实物核对的项只能在实机做。
