#!/usr/bin/env python3
"""fcu_params.py —— 飞控（PX4）安全参数**纯逻辑**：解析 / 判据 / 预置（无 ROS 依赖）。

参考 drone_package_20260908 的安全做法（`tools/fc_configure.py` 的分组 + 知识库）：

- **贯穿原则**：航向/位置精度**在飞行中验证之前**，不启用任何"自主飞一段距离"的失效动作
  ⇒ 失效保护选 **Land**（`NAV_RCL_ACT=3`、`COM_OBL_RC_ACT=4`），不选 Return。
- **关键关系**（血的教训）：
  - `MPC_THR_MIN < MPC_THR_HOVER`（否则"油门拉到底也降不下来"）
  - `RTL_RETURN_ALT ≤ GF_MAX_VER_DIST`（否则 RTL 爬升撞破围栏、两个失效保护打架）
  - `COM_RCL_EXCEPT` 不得含 bit2(=4)（否则 OFFBOARD 中 RC 失联被豁免＝拆掉最后一道闸）
  - 电池阈值次序 `EMERGEN ≤ CRIT ≤ LOW`
  - 失效保护/围栏不得为 0（`COM_LOW_BAT_ACT=0` 无动作、`GF_MAX_*=0` 未启用）

`check_params()` 是纯函数，`tools/test_fcu_params.py` 离线单测；`PRESETS` 供
`tools/fcu_configure.py`（dry-run 默认）写入飞控。
"""
from __future__ import annotations

import re
from dataclasses import dataclass

_PARAM_RE = re.compile(r'^[A-Z][A-Z0-9_]+$')


@dataclass
class Issue:
    name: str
    level: str        # ok | warn | fail
    detail: str = ''


# ----------------------------------------------------------- 分组预置（写入用）
# 值经参考工程实机验证；理由见 knowledge base。仅作建议，apply 前务必 dry-run。
PRESETS = {
    'failsafe': [
        ('COM_OBL_RC_ACT', 4, 'offboard 丢失动作 → 4(Land)；0(Position) 在无位置源时危险'),
        ('COM_OF_LOSS_T', 0.2, 'offboard 超时 1.0→0.2s（有线链路前提）'),
        ('NAV_RCL_ACT', 3, 'RC 失联 → 3(Land)；2(Return) 室内/未验证航向时更危险'),
        ('COM_RC_LOSS_T', 0.5, 'RC 超时确认'),
        ('COM_RCL_EXCEPT', 0, '不得含 bit2(4)：否则 OFFBOARD 中 RC 失联被豁免'),
    ],
    'thrust': [
        ('MPC_THR_MIN', 0.12, '推力下限；必须明显 < THR_HOVER，别设 0（快速下降电机停转）'),
        # MPC_THR_HOVER 需按实测标定（参考机 ≈0.30~0.32），此处不写死
    ],
    'limits_indoor': [
        ('MPC_XY_VEL_MAX', 2.0, '水平速度硬帽；须 ≥ companion 指令上限'),
        ('MPC_Z_VEL_MAX_UP', 1.5, '升速上限'),
        ('MPC_Z_VEL_MAX_DN', 1.0, '降速上限'),
        ('MPC_TILTMAX_AIR', 25.0, '最大倾角（度）'),
        ('MPC_XY_CRUISE', 1.0, '巡航速度'),
        ('MPC_TKO_SPEED', 0.5, '起飞速度'),
        ('MPC_LAND_SPEED', 0.3, '降落速度'),
    ],
    'geofence_indoor': [
        ('GF_MAX_HOR_DIST', 3.0, '水平围栏（离 home）——外层兜底'),
        ('GF_MAX_VER_DIST', 3.0, '垂直围栏；须 ≥ RTL_RETURN_ALT'),
    ],
    'geofence_outdoor': [
        ('GF_MAX_HOR_DIST', 12.0, '水平围栏，场地半径 15m 留 3m 余量'),
        ('GF_MAX_VER_DIST', 15.0, '垂直围栏；须 ≥ RTL_RETURN_ALT'),
        ('RTL_RETURN_ALT', 10.0, '返航高度压到垂直围栏以内，避免与围栏动作冲突'),
    ],
    'battery': [
        ('COM_LOW_BAT_ACT', 3, '低电动作 → 3(Land)；0(None) 飞到没电'),
        ('BAT1_LOW_THR', 0.15, '低电阈值'),
        ('BAT1_CRIT_THR', 0.07, '严重低电阈值'),
        ('BAT1_EMERGEN_THR', 0.05, '紧急低电阈值'),
    ],
    'ekf2_mocap': [
        ('EKF2_GPS_CTRL', 0, '动捕/视觉：禁用 GPS 位置'),
        ('EKF2_EV_CTRL', 11, '启用外部视觉（位置+高度+yaw）'),
        ('EKF2_BARO_CTRL', 0, '视觉高度源，关气压计（两套高度源别混）'),
        ('EKF2_MAG_TYPE', 5, 'None —— 航向由视觉给'),
    ],
    'ekf2_rtk': [
        ('EKF2_GPS_CTRL', 7, 'RTK：GPS 位置+速度+高度'),
        ('EKF2_EV_CTRL', 0, '关闭外部视觉'),
        ('EKF2_BARO_CTRL', 1, '开气压计'),
        ('EKF2_MAG_TYPE', 0, 'Automatic —— 靠磁罗盘定航向'),
    ],
}


# ----------------------------------------------------------- 参数 dump 解析
def parse_param_dump(text: str) -> dict:
    """解析 PX4 `param show/dump` 或 QGC `.params` 文本 → {NAME: float}。

    支持：`NAME VALUE` / `NAME,VALUE` / `NAME=VALUE` / `idx idx NAME VALUE`（取末两列）。
    """
    out: dict = {}
    for line in text.splitlines():
        s = line.strip()
        if not s or s.startswith('#'):
            continue
        if '=' in s:
            k, v = s.split('=', 1)
            k = k.strip()
            if _PARAM_RE.match(k):
                try:
                    out[k] = float(v.strip().split(',')[0])
                    continue
                except ValueError:
                    pass
        toks = re.split(r'[\s,]+', s)
        if len(toks) >= 2 and _PARAM_RE.match(toks[-2]):
            try:
                out[toks[-2]] = float(toks[-1])
            except ValueError:
                pass
    return out


# ----------------------------------------------------------- 判据（纯函数）
def check_params(p: dict, profile: str = 'sitl') -> list:
    """对飞控参数做安全检查。profile ∈ {sitl, indoor, outdoor}。"""
    R: list = []

    def add(name, cond, detail='', level='fail'):
        R.append(Issue(name, 'ok' if cond else level, '' if cond else detail))

    # ── 关键关系（任何 profile 都查）──
    if 'MPC_THR_MIN' in p and 'MPC_THR_HOVER' in p:
        add('MPC_THR_MIN < MPC_THR_HOVER',
            p['MPC_THR_MIN'] < p['MPC_THR_HOVER'] - 0.03,
            f"{p['MPC_THR_MIN']} vs {p['MPC_THR_HOVER']}（否则降不下来）")
    else:
        R.append(Issue('MPC_THR_MIN/HOVER', 'warn', '未提供，无法核验推力关系'))

    if 'COM_RCL_EXCEPT' in p:
        add('COM_RCL_EXCEPT 不豁免 OFFBOARD',
            (int(p['COM_RCL_EXCEPT']) & 4) == 0,
            f"值={int(p['COM_RCL_EXCEPT'])} 含 bit2 → OFFBOARD 中 RC 失联被豁免")

    if 'MPC_THR_MIN' in p:
        add('MPC_THR_MIN ≥ 0.05（防电机停转）', p['MPC_THR_MIN'] >= 0.05,
            f"{p['MPC_THR_MIN']}", 'warn')

    if 'MPC_XY_VEL_MAX' in p:
        add('MPC_XY_VEL_MAX ≥ 2.0（≥ 指令上限）', p['MPC_XY_VEL_MAX'] >= 2.0,
            f"{p['MPC_XY_VEL_MAX']}")

    if 'RTL_RETURN_ALT' in p and p.get('GF_MAX_VER_DIST', 0) > 0:
        add('RTL_RETURN_ALT ≤ GF_MAX_VER_DIST',
            p['RTL_RETURN_ALT'] <= p['GF_MAX_VER_DIST'],
            f"{p['RTL_RETURN_ALT']} > {p['GF_MAX_VER_DIST']}（RTL 会撞破围栏）")

    bat = ('BAT1_EMERGEN_THR', 'BAT1_CRIT_THR', 'BAT1_LOW_THR')
    if all(k in p for k in bat):
        add('电池阈值次序 EMERGEN ≤ CRIT ≤ LOW',
            p[bat[0]] <= p[bat[1]] <= p[bat[2]],
            f"{p[bat[0]]}/{p[bat[1]]}/{p[bat[2]]}")

    # ── 实机（indoor/outdoor）：失效保护/围栏/低电/EKF 源 ──
    if profile in ('indoor', 'outdoor'):
        add('围栏已启用 (GF_MAX_* > 0)',
            p.get('GF_MAX_HOR_DIST', 0) > 0 and p.get('GF_MAX_VER_DIST', 0) > 0,
            f"H={p.get('GF_MAX_HOR_DIST',0)} V={p.get('GF_MAX_VER_DIST',0)}")
        add('COM_OBL_RC_ACT = 4(Land)', int(p.get('COM_OBL_RC_ACT', 0)) == 4,
            f"={int(p.get('COM_OBL_RC_ACT',0))}")
        add('NAV_RCL_ACT = 3(Land)', int(p.get('NAV_RCL_ACT', 0)) == 3,
            f"={int(p.get('NAV_RCL_ACT',0))}")
        add('COM_LOW_BAT_ACT ∈ {2,3}', int(p.get('COM_LOW_BAT_ACT', 0)) in (2, 3),
            f"={int(p.get('COM_LOW_BAT_ACT',0))}（0=None 会飞到没电）")
        add('COM_OF_LOSS_T ≤ 0.5', p.get('COM_OF_LOSS_T', 9.0) <= 0.5,
            f"={p.get('COM_OF_LOSS_T')}", 'warn')
        if profile == 'indoor':
            ok = (int(p.get('EKF2_GPS_CTRL', -1)) == 0 and int(p.get('EKF2_EV_CTRL', -1)) == 11
                  and int(p.get('EKF2_BARO_CTRL', -1)) == 0
                  and int(p.get('EKF2_MAG_TYPE', -1)) == 5)
            add('EKF2 源=动捕(0/11/0/5)', ok, '与 ekf2_mocap 预置不一致')
        else:
            ok = (int(p.get('EKF2_GPS_CTRL', -1)) == 7 and int(p.get('EKF2_EV_CTRL', -1)) == 0
                  and int(p.get('EKF2_BARO_CTRL', -1)) == 1
                  and int(p.get('EKF2_MAG_TYPE', -1)) == 0)
            add('EKF2 源=RTK(7/0/1/0)', ok, '与 ekf2_rtk 预置不一致')
        # 提示（非硬判）
        if 'SENS_BOARD_ROT' in p:
            R.append(Issue('SENS_BOARD_ROT 与实物一致（参考机=0）', 'warn',
                           f"={int(p['SENS_BOARD_ROT'])}——改错会起飞直接翻"))
    return R


# ----------------------------------------------------------- 自测
def _selftest() -> bool:
    ok = True

    def chk(name, cond):
        nonlocal ok
        print(f'  {"✅" if cond else "❌"} {name}')
        ok = ok and bool(cond)

    p = parse_param_dump("MPC_THR_MIN 0.12\n1 1 MPC_THR_HOVER 0.30\nCOM_RCL_EXCEPT=0")
    chk('解析多格式', p.get('MPC_THR_MIN') == 0.12 and p.get('MPC_THR_HOVER') == 0.30
        and p.get('COM_RCL_EXCEPT') == 0)
    good = {'MPC_THR_MIN': 0.12, 'MPC_THR_HOVER': 0.30, 'COM_RCL_EXCEPT': 0,
            'MPC_XY_VEL_MAX': 2.0}
    chk('好参数无 fail', all(i.level != 'fail' for i in check_params(good, 'sitl')))
    bad = dict(good, MPC_THR_MIN=0.30)
    chk('THR_MIN=HOVER → fail',
        any(i.level == 'fail' and i.name.startswith('MPC_THR_MIN <') for i in check_params(bad)))
    return ok


if __name__ == '__main__':
    raise SystemExit(0 if _selftest() else 1)
