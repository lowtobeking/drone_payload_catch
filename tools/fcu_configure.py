#!/usr/bin/env python3
"""飞控安全参数**配置器**（PX4 / MAVLink）——对标 drone_package_20260908 的 `fc_configure.py`。

默认 **dry-run 只显示**；确认后加 `--apply` 才写入，写前备份原值、写后读回校验。

    python3 tools/fcu_configure.py --list
    python3 tools/fcu_configure.py -g failsafe,limits_indoor              # dry-run
    python3 tools/fcu_configure.py -g failsafe,limits_indoor --apply --mavlink udpout:127.0.0.1:18570

⚠️ 需要交互的三项（传感器校准 / 电调校准与电机测试 / RC 通道映射与 kill 开关）**不在本脚本内**，
   必须在 QGC 图形界面做（与参考工程一致）。
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from payload_catch.fcu_params import PRESETS   # noqa: E402


def apply_group(m, group: str) -> int:
    """写一组参数：先读原值（推断类型），再写、回读校验。返回失败数。"""
    fails = 0
    for name, target, why in PRESETS[group]:
        cur, ptype = None, None
        m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < 3:
            msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
            if msg and msg.param_id.strip('\x00') == name:
                cur, ptype = msg.param_value, msg.param_type
                break
        if ptype is None:
            print(f'  ❌ {name}: 读不到原值，跳过')
            fails += 1
            continue
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                             float(target), ptype)
        got = None
        t0 = time.time()
        while time.time() - t0 < 3:
            msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
            if msg and msg.param_id.strip('\x00') == name:
                got = msg.param_value
                break
        ok = got is not None and abs(float(got) - float(target)) < 1e-3
        print(f'  {"✅" if ok else "❌"} {name}: {cur} → {got} (目标 {target})  # {why}')
        fails += 0 if ok else 1
    return fails


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='飞控安全参数配置器（dry-run 默认）')
    ap.add_argument('-g', '--groups', default='',
                    help=f'逗号分隔的分组；可用：{",".join(PRESETS)}')
    ap.add_argument('--apply', action='store_true', help='真正写入（默认只预览）')
    ap.add_argument('--mavlink', default='udpout:127.0.0.1:18570')
    ap.add_argument('--list', action='store_true')
    args = ap.parse_args(argv)

    if args.list or not args.groups:
        for g, items in PRESETS.items():
            print(f'[{g}]')
            for n, v, why in items:
                print(f'  {n} = {v}   # {why}')
        if not args.list:
            print('\n用法：-g failsafe,limits_indoor [--apply]')
        return 0

    groups = [g.strip() for g in args.groups.split(',') if g.strip()]
    unknown = [g for g in groups if g not in PRESETS]
    if unknown:
        print(f'❌ 未知分组：{unknown}；可用：{list(PRESETS)}')
        return 2

    if not args.apply:
        print('=== DRY-RUN（只预览，不写入）===')
        for g in groups:
            print(f'[{g}]')
            for n, v, why in PRESETS[g]:
                print(f'  {n} = {v}   # {why}')
        print('\n确认无误后加 --apply 写入。')
        return 0

    try:
        from pymavlink import mavutil
    except ImportError:
        print('❌ 未安装 pymavlink')
        return 2
    m = mavutil.mavlink_connection(args.mavlink)
    if m.wait_heartbeat(timeout=10) is None:
        print(f'❌ {args.mavlink} 上无心跳')
        return 2
    print(f'=== APPLY（{args.mavlink}）===')
    fails = sum(apply_group(m, g) for g in groups)
    print(f'--- 完成：{"全部成功" if fails == 0 else f"{fails} 项失败"} ---')
    return 1 if fails else 0


if __name__ == '__main__':
    raise SystemExit(main())
