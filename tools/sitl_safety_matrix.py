#!/usr/bin/env python3
"""安全层故障注入 SITL 矩阵（纯逻辑判定见 payload_catch/safety_matrix.py）。

    source env.sh && python3 tools/sitl_safety_matrix.py            # 全部场景（~10 min）
    python3 tools/sitl_safety_matrix.py --only baseline_no_false_trigger
    python3 tools/sitl_safety_matrix.py --list

每场景：跑 `run_m6_sitl.sh`（可带 LAUNCH_EXTRA / 中途动作）→ 读 `launch.log` →
用 `verdict()` 判 PASS/FAIL → 汇总写 `report/safety_injection_matrix.md`；任一 FAIL 退出非 0。
"""
import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from payload_catch.safety_matrix import SCENARIOS, verdict   # noqa: E402

LAUNCH_LOG = Path.home() / 'payload_catch_m6' / 'launch.log'
PX4_LOG_DIR = Path.home() / 'px4_logs'


def _run(cmd, **kw):
    return subprocess.run(cmd, **kw)


def kill_peer():
    """杀掉 A 节点（用独立 argv，避免 pkill 模式串出现在本进程命令行里）。"""
    for pat in ('payload_catch/lib/payload_catch/a_node', 'lib/payload_catch/a_node'):
        subprocess.run(['pkill', '-9', '-f', pat], check=False)


def kill_b():
    # --once 需等到有订阅者匹配；给足超时，失败也不阻断（判定看日志）
    try:
        _run(['ros2', 'topic', 'pub', '--once', '/safety/kill_b',
              'std_msgs/msg/Bool', '{data: true}'], stdout=subprocess.DEVNULL,
             stderr=subprocess.DEVNULL, timeout=45)
    except subprocess.TimeoutExpired:
        print('  (ros2 topic pub 超时，继续看日志判定)')


def run_scenario(sc):
    print(f'\n########## {sc.name} ##########')
    print(f'  why: {sc.why}')
    if LAUNCH_LOG.exists():
        LAUNCH_LOG.unlink()
    runlog = Path('/tmp') / f'pc_matrix_{sc.name}.log'
    env = {**os.environ}
    if sc.launch_extra:
        env['LAUNCH_EXTRA'] = sc.launch_extra
    rf = open(runlog, 'w')
    proc = subprocess.Popen(['bash', str(ROOT / 'run_m6_sitl.sh'), str(sc.sec)],
                            env=env, cwd=str(ROOT), stdout=rf, stderr=subprocess.STDOUT)
    # run_m6_sitl.sh 启动时会 rm -f $D/*.log；launch.log 只在"双机 READY 后"由 ros2 launch 创建，
    # 故以 "launch.log 出现" 作为"节点已启动"信号（比抓 run 脚本 stdout 稳）。
    if sc.action:
        ready = False
        for _ in range(50):
            time.sleep(3)
            if LAUNCH_LOG.exists():
                ready = True
                break
        print(f'  READY={ready}，等 {sc.action_after_ready_s:.0f}s 后执行 {sc.action}')
        if ready:
            time.sleep(sc.action_after_ready_s)
            if sc.action == 'kill_peer':
                kill_peer()
            elif sc.action == 'kill_b':
                kill_b()
        else:
            print('  ⚠️ launch.log 未出现 — 跳过中途动作')
    proc.wait()
    rf.close()

    text = LAUNCH_LOG.read_text(errors='ignore') if LAUNCH_LOG.exists() else ''
    # 平台 failsafe 也看 px4 日志
    for i in (0, 1):
        p = PX4_LOG_DIR / f'px4_{i}.log'
        if p.exists():
            text += '\n' + p.read_text(errors='ignore')
    ok, detail, matched = verdict(text, sc)
    print(f'  {"✅ PASS" if ok else "❌ FAIL"}  {detail}')
    return ok, detail


def main(argv=None):
    ap = argparse.ArgumentParser(description='安全层故障注入 SITL 矩阵')
    ap.add_argument('--only', help='只跑某个场景名')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--out', default=str(ROOT / 'report' / 'safety_injection_matrix.md'))
    args = ap.parse_args(argv)

    if args.list:
        for sc in SCENARIOS:
            print(f'{sc.name:32s} {sc.sec}s  extra="{sc.launch_extra}"  action={sc.action or "-"}')
        return 0

    scen = [sc for sc in SCENARIOS if (not args.only or sc.name == args.only)]
    results = []
    for sc in scen:
        ok, detail = run_scenario(sc)
        results.append((sc, ok, detail))

    lines = ['# 安全层故障注入 SITL 矩阵', '',
             '> 编排：`tools/sitl_safety_matrix.py`；判定：`payload_catch/safety_matrix.py`（纯逻辑）。',
             '> 对标参考工程 `safety_filter_SITL回归清单.md`（S27–S33）。', '',
             '| 场景 | 注入 | 期望 | 结果 | 证据/原因 |', '|---|---|---|---|---|']
    n_fail = 0
    for sc, ok, detail in results:
        n_fail += 0 if ok else 1
        lines.append(f'| `{sc.name}` | {sc.launch_extra or sc.action or "标称"} | {sc.why} | '
                     f'{"✅ PASS" if ok else "❌ FAIL"} | {detail} |')
    lines.append('')
    lines.append(f'**汇总：{len(results) - n_fail}/{len(results)} PASS**')
    Path(args.out).write_text('\n'.join(lines), encoding='utf-8')
    print(f'\n===== {len(results) - n_fail}/{len(results)} PASS → {args.out} =====')
    return 1 if n_fail else 0


if __name__ == '__main__':
    raise SystemExit(main())
