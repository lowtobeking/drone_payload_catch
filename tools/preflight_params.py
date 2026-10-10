#!/usr/bin/env python3
"""飞控安全参数**飞行前检查**（PX4）——对标 drone_package_20260908 的 `fc_configure.py` 体检项。

    python3 tools/preflight_params.py --file dump.txt --profile indoor
    python3 tools/preflight_params.py --mavlink udpout:127.0.0.1:18570 --profile outdoor
    python3 tools/preflight_params.py --list        # 列出分组预置值

来源：`--file`（PX4 `param show/dump` 或 QGC `.params` 文本）或 `--mavlink`（pymavlink 直连飞控）。
判据纯逻辑见 `payload_catch/fcu_params.py`；`--dump` 可把读到的参数存成文件再用 `--file` 复跑。
退出码：0=无 fail（可含 warn），1=有 fail。
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from payload_catch.fcu_params import PRESETS, check_params, parse_param_dump   # noqa: E402


def read_file(path: str) -> dict:
    with open(path, encoding='utf-8', errors='ignore') as f:
        return parse_param_dump(f.read())


def read_mavlink(conn: str, timeout: float = 30.0, idle: float = 2.0) -> dict:
    try:
        from pymavlink import mavutil
    except ImportError as e:                                   # noqa: F841
        raise RuntimeError('未安装 pymavlink（pip install pymavlink）') from e
    m = mavutil.mavlink_connection(conn)
    # 主动发 GCS 心跳，让 PX4 学到本客户端地址（udpout 场景必需）
    deadline = time.time() + 10.0
    hb = None
    while time.time() < deadline:
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0, 0)
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if hb is not None:
            break
    if hb is None:
        raise RuntimeError(f'{conn} 上 10s 内无心跳（端口/实例对不对？）')
    m.mav.param_request_list_send(m.target_system, m.target_component)
    out: dict = {}
    t0 = last = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
        if msg is not None:
            out[msg.param_id.strip('\x00')] = float(msg.param_value)
            last = time.time()
        elif out and (time.time() - last) > idle:
            break                                              # 空闲即认为收完
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='飞控安全参数飞行前检查')
    ap.add_argument('--file', help='参数 dump 文件（PX4 param show/dump 或 QGC .params）')
    ap.add_argument('--mavlink', help='MAVLink 连接串，如 udpout:127.0.0.1:18570')
    ap.add_argument('--profile', default='sitl', choices=['sitl', 'indoor', 'outdoor'])
    ap.add_argument('--dump', help='把读到的参数写入该文件')
    ap.add_argument('--list', action='store_true', help='列出分组预置值')
    ap.add_argument('--json', action='store_true')
    args = ap.parse_args(argv)

    if args.list:
        for g, items in PRESETS.items():
            print(f'[{g}]')
            for n, v, why in items:
                print(f'  {n} = {v}   # {why}')
        return 0

    if args.mavlink:
        params = read_mavlink(args.mavlink)
        src = f'mavlink:{args.mavlink}'
    elif args.file:
        params = read_file(args.file)
        src = args.file
    else:
        print('❌ 需要 --file 或 --mavlink（或 --list 看预置）')
        return 2

    if args.dump:
        with open(args.dump, 'w') as f:
            for k in sorted(params):
                f.write(f'{k} {params[k]}\n')
        print(f'已 dump {len(params)} 个参数 → {args.dump}')

    issues = check_params(params, args.profile)
    n_fail = sum(i.level == 'fail' for i in issues)
    n_warn = sum(i.level == 'warn' for i in issues)

    if args.json:
        print(json.dumps({'source': src, 'profile': args.profile,
                          'n_params': len(params),
                          'issues': [i.__dict__ for i in issues]}, ensure_ascii=False))
        return 1 if n_fail else 0

    print(f'=== 飞控参数检查（{src}，profile={args.profile}，{len(params)} 项）===')
    icon = {'ok': '✅', 'warn': '⚠️ ', 'fail': '❌'}
    for i in issues:
        print(f'  {icon[i.level]} {i.name}' + (f'   — {i.detail}' if i.detail else ''))
    print(f'--- 汇总：fail={n_fail} warn={n_warn} ---')
    return 1 if n_fail else 0


if __name__ == '__main__':
    raise SystemExit(main())
