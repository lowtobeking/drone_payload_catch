#!/usr/bin/env python3
"""起飞前自检（preflight）——在跑 SITL / 真机之前，先确认"能不能飞"。

    python3 tools/preflight_check.py                 # 离线静态检查（构建/依赖/配置）
    python3 tools/preflight_check.py --sitl          # SITL 严格：PX4/world/gz/agent/ROS 缺失即失败
    python3 tools/preflight_check.py --live          # 额外探测正在运行的 PX4 话题（ros2 CLI，只读）

约定：世界系 NED；本文件**不 import rclpy**（用 `ros2` CLI 子进程探测），
所以离线/无 ROS 机器也能跑。退出码：0=全通过（可含 warn），1=有 fail。

设计成"可单测"：静态检查逻辑是纯函数 `check_environment(env, root, ...)`，
`tools/test_preflight.py` 用假 env/假文件系统覆盖各种缺项组合。
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# 运行时（live）探测需要的 PX4 话题（1.16 命名）
LIVE_TOPICS = [
    '/fmu/out/vehicle_status_v1',
    '/fmu/out/vehicle_local_position',
    '/fmu/out/vehicle_attitude',
    '/fmu/out/failsafe_flags',
    '/fmu/out/estimator_status_flags',
    '/px4_1/fmu/out/vehicle_local_position',
]


@dataclass
class Res:
    name: str
    status: str          # ok | warn | fail
    detail: str = ''


def _has_dep(mod: str) -> bool:
    return importlib.util.find_spec(mod) is not None


def check_environment(env, root, *, strict_sitl: bool = False,
                      which=shutil.which, isfile=os.path.isfile,
                      isdir=os.path.isdir, has_dep=_has_dep) -> list:
    """纯函数：给定 env/文件系统探测函数，返回检查结果列表。

    strict_sitl=True 时，PX4/world/gz/agent/ROS 缺失算 fail（跑 SITL 必须全绿）；
    否则只算 warn（纯离线算法层不依赖它们）。
    """
    root = Path(root)
    R: list = []

    def add(name, cond, detail='', level='fail'):
        R.append(Res(name, 'ok' if cond else level, '' if cond else detail))

    offline_level = 'fail' if strict_sitl else 'warn'

    # ── 仓库文件 ──
    add('config/catch_scenarios.yaml', isfile(str(root / 'config' / 'catch_scenarios.yaml')),
        '单一真值源缺失')
    add('models/ 目录', isdir(str(root / 'models')), 'Gazebo 模型缺失')
    add('launch/ 目录', isdir(str(root / 'launch')), 'launch 缺失')

    # ── Python 依赖 ──
    for m in ('numpy', 'scipy', 'yaml'):
        add(f'python:{m}', has_dep(m), '未安装（离线算法层需要）')
    add('python:acados_template', has_dep('acados_template'),
        'MPC 工况不可用', 'warn')
    add('python:rclpy', has_dep('rclpy'), 'SITL/真机不可用', offline_level)
    add('python:px4_msgs', has_dep('px4_msgs'), 'SITL 不可用', offline_level)

    # ── 环境变量 ──
    px4 = str(env.get('PX4_DIR', '') or '')
    add('PX4_DIR 已设置', bool(px4), '未 source env.sh', offline_level)
    if px4:
        add('PX4 SITL 可执行',
            isfile(str(Path(px4) / 'build' / 'px4_sitl_default' / 'bin' / 'px4')),
            f'{px4}/build/.../bin/px4 缺失（未 build）', offline_level)
    world = str(env.get('SITL_WORLD', '') or '')
    add('SITL_WORLD 存在', bool(world) and isfile(world), world or '未设置', offline_level)
    add('RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
        env.get('RMW_IMPLEMENTATION') == 'rmw_fastrtps_cpp',
        str(env.get('RMW_IMPLEMENTATION', '(空)')) + '（PX4 话题可能静默不可见）',
        offline_level)
    add('GZ_SIM_RESOURCE_PATH 非空', bool(env.get('GZ_SIM_RESOURCE_PATH')),
        'Gazebo 找不到 PX4 模型', 'warn')
    acados = str(env.get('ACADOS_SOURCE_DIR', '') or '')
    add('ACADOS_SOURCE_DIR 存在', bool(acados) and isdir(acados),
        (acados or '未设置') + '（MPC 不可用 / libqpOASES 可能缺失）', 'warn')

    # ── 可执行 ──
    for b in ('gz', 'ros2', 'MicroXRCEAgent'):
        add(f'binary:{b}', which(b) is not None, '未在 PATH', offline_level)
    return R


def _run(cmd, timeout=8):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout
    except (OSError, subprocess.SubprocessError):
        return 1, ''


def live_checks(runner=_run) -> list:
    """探测正在运行的 PX4（只读 ros2 CLI）。假定已 source env 且 SITL 在跑。"""
    R: list = []
    rc, out = runner(['ros2', 'topic', 'list'], timeout=15)
    topics = set(out.split())
    for t in LIVE_TOPICS:
        R.append(Res(f'话题 {t}', 'ok' if t in topics else 'fail',
                     '' if t in topics else '未发布（PX4/agent 未就绪？）'))
    # 位置有效性
    rc, out = runner(['ros2', 'topic', 'echo', '/fmu/out/vehicle_local_position',
                      '--qos-reliability', 'best_effort', '--once'], timeout=10)
    xy = 'xy_valid: true' in out
    z = 'z_valid: true' in out
    dr = 'dead_reckoning: true' in out
    R.append(Res('EKF xy_valid', 'ok' if xy else 'fail', '' if xy else '水平位置无效'))
    R.append(Res('EKF z_valid', 'ok' if z else 'fail', '' if z else '垂直位置无效'))
    R.append(Res('非 dead_reckoning', 'ok' if (out and not dr) else 'fail',
                 '' if (out and not dr) else 'EKF 在死推算/无输出'))
    # 预检故障标志（failsafe_flags.local_position_invalid 应为 false）
    if '/fmu/out/failsafe_flags' in topics:
        _, ff = runner(['ros2', 'topic', 'echo', '/fmu/out/failsafe_flags',
                        '--qos-reliability', 'best_effort', '--once'], timeout=10)
        bad = 'local_position_invalid: true' in ff
        R.append(Res('failsafe_flags 无 local_position_invalid',
                     'fail' if bad else 'ok',
                     'local_position_invalid=true' if bad else ''))
    return R


def check_px4_logs(log_dir, drone_ids=(0, 1), isfile=os.path.isfile,
                   read_text=None) -> list:
    """起飞前扫 PX4 日志（grep -a 风格）。

    - `Ready for takeoff` 必须在（否则 fail）；
    - **致命**故障（`Gyro #0 fail: STALE` / `Arming denied`）→ fail；
    - 其余 `Preflight Fail` → **warn**（本项目 B 有固有健康告警，
      且 `Preflight Fail: ekf2 missing data` 是启动瞬态，不能当硬失败）。
    """
    log_dir = Path(log_dir)
    if read_text is None:
        def read_text(p):
            return Path(p).read_text(encoding='utf-8', errors='ignore')
    FATAL = ('Gyro #0 fail: STALE', 'Arming denied')
    R: list = []
    for i in drone_ids:
        p = log_dir / f'px4_{i}.log'
        if not isfile(str(p)):
            R.append(Res(f'px4_{i}.log 存在', 'warn', f'{p} 缺失（SITL 未起？）'))
            continue
        txt = read_text(str(p))
        rdy = 'Ready for takeoff' in txt
        R.append(Res(f'd{i} Ready for takeoff', 'ok' if rdy else 'fail',
                     '' if rdy else '未就绪'))
        fatal = [k for k in FATAL if k in txt]
        R.append(Res(f'd{i} 无致命故障', 'fail' if fatal else 'ok', '；'.join(fatal)))
        warns = sorted(set(re.findall(r'Preflight Fail[^\n]*', txt)))
        R.append(Res(f'd{i} Preflight 告警', 'warn' if warns else 'ok',
                     '；'.join(warns[:3])))
    return R


def _print(results: list) -> tuple:
    icon = {'ok': '✅', 'warn': '⚠️ ', 'fail': '❌'}
    for r in results:
        line = f'  {icon.get(r.status, "?")} {r.name}'
        if r.detail:
            line += f'   — {r.detail}'
        print(line)
    n_fail = sum(r.status == 'fail' for r in results)
    n_warn = sum(r.status == 'warn' for r in results)
    return n_fail, n_warn


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='起飞前自检（preflight）')
    ap.add_argument('--sitl', action='store_true', help='SITL 严格模式（PX4/ROS 缺失即 fail）')
    ap.add_argument('--live', action='store_true', help='额外探测运行中的 PX4 话题')
    ap.add_argument('--logs', nargs='?', const=os.path.expanduser('~/px4_logs'), default=None,
                    help='扫 PX4 日志门（默认 ~/px4_logs）：Ready 必须有、预检故障必须无')
    ap.add_argument('--json', action='store_true', help='输出 JSON')
    args = ap.parse_args(argv)

    mode = 'SITL' if args.sitl else '离线'
    extra = '+live' if args.live else ''
    extra += '+logs' if args.logs is not None else ''
    if not args.json:
        print(f'=== 起飞前自检（{mode}{extra}）===')
    results = check_environment(dict(os.environ), ROOT, strict_sitl=args.sitl)
    if args.live:
        results += live_checks()
    if args.logs is not None:
        results += check_px4_logs(args.logs)

    if args.json:
        import json
        print(json.dumps([r.__dict__ for r in results], ensure_ascii=False, indent=2))
        return 1 if any(r.status == 'fail' for r in results) else 0

    n_fail, n_warn = _print(results)
    print(f'--- 汇总：fail={n_fail} warn={n_warn} ---')
    if n_fail:
        print('❌ 起飞前自检未通过（修掉 fail 再起飞）')
        return 1
    print('✅ 起飞前自检通过' + ('（有 warn，注意）' if n_warn else ''))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
