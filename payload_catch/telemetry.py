#!/usr/bin/env python3
"""telemetry.py —— 遥测有效性判据（纯逻辑，无 ROS/Gazebo 依赖）。

对标 drone_package_20260908/Multi-UAV-simulation-full/tools/data_validity.py：
节点/健康通道停更时，日志会把**最后一个值一直重复**，从行数、`caught`、退出码上
**都看不出异常**——本模块用「冻结段 + 沉默间隔 + 溢出 + 兜底占比」把它揪出来。

适配本工程 `launch.log` 的周期行，如：
    [b_node-2] [INFO] [1791349046.38] [b_node]: B phase=HOLD safe=OK att=1.00
        pos_w=[ 0.21  0.01 -0.04] vel=[-0.03 -0.   -0.04] relA=None ... caught=False

失效签名：
  OVERFLOW  位置分量 |x| > 阈值（估计器发散，常见 ±2.1e9）
  STALE     `pos_w` 连续多拍完全不变（远轴通道死亡/冻结）
  SILENCE   同一源相邻日志时间戳间隔过大（节点死亡/卡死）
  HOVER     `safe=HOLD` 占比过高（悬停兜底主导，那段值合法但**不是测量**）
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

OVERFLOW_ABS = 1e6
MIN_CONST_RUN = 10        # 冻结段绝对下限（拍）
MIN_CONST_FRAC = 0.30     # 冻结段占该源样本比例下限
MAX_GAP_S = 5.0           # 相邻日志最大间隔（秒）
HOVER_FRAC = 0.5

_SRC = re.compile(r'\[(\w+)-\d+\]')
_TS = re.compile(r'\[(\d+\.\d+)\]')
_ARR = re.compile(r'(\w+)=\[\s*([^\]]*?)\s*\]')
_SCALAR = re.compile(r'(\w+)=([^\s\]]+)')
_ANSI = re.compile(r'\x1b\[[0-9;]*m')


@dataclass
class Sample:
    t: float
    pos: tuple | None = None
    vel: tuple | None = None
    fields: dict = field(default_factory=dict)


@dataclass
class Issue:
    code: str        # OVERFLOW | STALE | SILENCE | HOVER
    source: str
    level: str       # fail | warn
    detail: str = ''


def _strip_ansi(text: str) -> str:
    return _ANSI.sub('', text)


def _floats(s: str):
    try:
        return tuple(float(x) for x in s.split())
    except ValueError:
        return None


def parse_line(line: str):
    """解析一行 → (source, t, Sample)；不匹配返回 None。"""
    m = _SRC.search(line)
    if not m:
        return None
    src = m.group(1)
    tm = _TS.search(line)
    t = float(tm.group(1)) if tm else math.nan
    pos = vel = None
    for k, v in _ARR.findall(line):
        arr = _floats(v)
        if k == 'pos_w':
            pos = arr
        elif k == 'vel':
            vel = arr
    fields = {k: v for k, v in _SCALAR.findall(line)}
    return src, t, Sample(t, pos, vel, fields)


def parse_log(text: str) -> dict:
    """整段日志 → {source: [Sample, ...]}（保持出现顺序）。"""
    series: dict = {}
    for line in _strip_ansi(text).splitlines():
        r = parse_line(line)
        if r is None:
            continue
        src, _t, s = r
        series.setdefault(src, []).append(s)
    return series


def longest_const_run(vals) -> int:
    """最长「完全相同」连续段长度（空 → 0）。"""
    if not vals:
        return 0
    best = cur = 1
    for i in range(1, len(vals)):
        cur = cur + 1 if vals[i] == vals[i - 1] else 1
        best = max(best, cur)
    return best


def max_t_gap(times) -> float:
    """相邻时间戳最大间隔（<2 个样本 → 0）。"""
    ts = [t for t in times if t == t]          # 去 NaN
    if len(ts) < 2:
        return 0.0
    return max(b - a for a, b in zip(ts, ts[1:]))


def median_t_gap(times) -> float:
    """相邻时间戳间隔中位数（<2 个样本 → 0）；用于区分周期源 vs 事件源。"""
    ts = [t for t in times if t == t]
    if len(ts) < 2:
        return 0.0
    gaps = sorted(b - a for a, b in zip(ts, ts[1:]))
    n = len(gaps)
    return gaps[n // 2] if n % 2 else 0.5 * (gaps[n // 2 - 1] + gaps[n // 2])


def analyze(series: dict, *, min_run: int = MIN_CONST_RUN,
            min_frac: float = MIN_CONST_FRAC, max_gap: float = MAX_GAP_S,
            overflow_abs: float = OVERFLOW_ABS, hover_frac: float = HOVER_FRAC) -> list:
    """纯函数：对每个源的样本序列产出 Issue 列表。"""
    issues: list = []
    for src, samples in series.items():
        pos = [s.pos for s in samples if s.pos is not None]

        flat = [abs(x) for p in pos for x in p]
        if flat and max(flat) > overflow_abs:
            issues.append(Issue('OVERFLOW', src, 'fail',
                                f'|pos|max={max(flat):.2e} > {overflow_abs:.0e}'))

        if len(pos) >= min_run:
            run = longest_const_run(pos)
            if run >= min_run and run / len(pos) >= min_frac:
                issues.append(Issue('STALE', src, 'fail',
                                    f'pos_w 冻结 {run}/{len(pos)} 帧'))

        g = max_t_gap([s.t for s in samples])
        med = median_t_gap([s.t for s in samples])
        # 沉默检测只对**采样足够多的周期源**：事件型（如 payload_node 启动突刺+零星事件）
        # 样本少/中位间隔大，长间隔正常，不误报。
        if len(samples) >= min_run and g > max_gap and med < 0.5 * max_gap:
            issues.append(Issue('SILENCE', src, 'fail',
                                f'最大间隔 {g:.1f}s > {max_gap}s（常态 {med:.1f}s）'))

        if samples:
            hov = sum(1 for s in samples if s.fields.get('safe') == 'HOLD') / len(samples)
            if hov > hover_frac:
                issues.append(Issue('HOVER', src, 'warn',
                                    f'HOLD 占比 {hov:.0%}（兜底主导，值非测量）'))
    return issues


def validity_report(text: str, **kw) -> list:
    """日志文本 → Issue 列表（parse + analyze）。"""
    return analyze(parse_log(text), **kw)


# --------------------------------------------------------------- 自测
def _selftest() -> bool:
    ok = True

    def chk(name, cond):
        nonlocal ok
        print(f'  {"✅" if cond else "❌"} {name}')
        ok = ok and bool(cond)

    chk('longest_const_run', longest_const_run([1, 1, 1, 2, 2]) == 3
        and longest_const_run([]) == 0)
    line = ('[b_node-2] [INFO] [1791349046.38] [b_node]: B phase=HOLD safe=OK '
            'pos_w=[ 1.0 2.0 -3.0] vel=[0.1 0.2 0.3] caught=False')
    r = parse_line(line)
    chk('parse_line', r is not None and r[0] == 'b_node'
        and r[2].pos == (1.0, 2.0, -3.0) and r[2].fields.get('phase') == 'HOLD')
    return ok


if __name__ == '__main__':
    raise SystemExit(0 if _selftest() else 1)
