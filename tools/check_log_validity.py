#!/usr/bin/env python3
"""launch.log 遥测有效性检查（对标 drone_package_20260908/data_validity.py）。

    python3 tools/check_log_validity.py ~/payload_catch_m6/launch.log
    python3 tools/check_log_validity.py launch.log --json
    python3 tools/check_log_validity.py launch.log --min-run 8 --max-gap 4

判定（纯逻辑见 payload_catch/telemetry.py）：
    OVERFLOW / STALE / SILENCE → fail（退出码非 0）；HOVER → warn。
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from payload_catch import telemetry as T   # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='launch.log 遥测有效性检查')
    ap.add_argument('logs', nargs='+', help='一个或多个 launch.log')
    ap.add_argument('--min-run', type=int, default=T.MIN_CONST_RUN)
    ap.add_argument('--min-frac', type=float, default=T.MIN_CONST_FRAC)
    ap.add_argument('--max-gap', type=float, default=T.MAX_GAP_S)
    ap.add_argument('--overflow', type=float, default=T.OVERFLOW_ABS)
    ap.add_argument('--json', action='store_true')
    args = ap.parse_args(argv)

    text = ''
    for f in args.logs:
        try:
            with open(f, encoding='utf-8', errors='ignore') as fh:
                text += fh.read() + '\n'
        except OSError as e:
            print(f'⚠️  读取失败 {f}: {e}', file=sys.stderr)

    series = T.parse_log(text)
    issues = T.analyze(series, min_run=args.min_run, min_frac=args.min_frac,
                       max_gap=args.max_gap, overflow_abs=args.overflow)
    n_fail = sum(i.level == 'fail' for i in issues)
    n_warn = sum(i.level == 'warn' for i in issues)

    if args.json:
        print(json.dumps({'sources': {k: len(v) for k, v in series.items()},
                          'issues': [i.__dict__ for i in issues]}, ensure_ascii=False))
        return 1 if n_fail else 0

    print('=== 遥测源 ===')
    for src, samples in series.items():
        print(f'  {src}: {len(samples)} 帧')
    print('=== 有效性问题 ===')
    if not issues:
        print('  ✅ 无（各源数值在变化、无溢出、无长间隔）')
    for i in issues:
        icon = '❌' if i.level == 'fail' else '⚠️ '
        print(f'  {icon} [{i.code}] {i.source}: {i.detail}')
    print(f'--- 汇总：fail={n_fail} warn={n_warn} ---')
    return 1 if n_fail else 0


if __name__ == '__main__':
    raise SystemExit(main())
