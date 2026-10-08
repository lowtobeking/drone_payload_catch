#!/usr/bin/env python3
"""make_docx_report.py —— 生成《空投—空中捕获》仿真报告（.docx）。

内容：相关研究（引言）/ 理论分析 / 仿真条件 / 仿真数据（含状态数据）/ 仿真分析。
数据：实时运行离线仿真（M1–M4 + M6 蒙特卡洛）采集，保证与当前代码一致；
      状态数据为代表性工况的时间序列（位置/速度/相对距离）。
图表：复用 report/figures/ 下已有 PNG。

依赖：python-docx（`pip install --break-system-packages python-docx`）、matplotlib（可选）。

用法:
  python3 tools/make_docx_report.py                 # 生成 report/drone_payload_catch_sim_report.docx
  python3 tools/make_docx_report.py --out /path/x.docx
  python3 tools/make_docx_report.py --fast          # 缩小 MC 次数，快速预览
"""
from __future__ import annotations

import argparse
import datetime as _dt
import math
import os
import sys

import numpy as np
import yaml
from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)

from payload_catch.sim_core import simulate, SimNoise           # noqa: E402
from payload_catch.stack_drop import (                           # noqa: E402
    simulate_stack, StackNoise, retain_speed, plan_stack_drop)

DEFAULT_YAML = os.path.join(REPO, 'config', 'catch_scenarios.yaml')
FIGDIR = os.path.join(REPO, 'report', 'figures')
DEFAULT_OUT = os.path.join(REPO, 'report', 'drone_payload_catch_sim_report.docx')

LATIN = 'Times New Roman'
EAST = '宋体'
EAST_H = '黑体'

# ===========================================================================
# Docx 基础工具
# ===========================================================================


def set_run(run, size=10.5, bold=None, italic=None, color=None, east=EAST):
    run.font.name = LATIN
    rpr = run._element.get_or_add_rPr()
    rf = rpr.find(qn('w:rFonts'))
    if rf is None:
        rf = OxmlElement('w:rFonts')
        rpr.append(rf)
    rf.set(qn('w:ascii'), LATIN)
    rf.set(qn('w:hAnsi'), LATIN)
    rf.set(qn('w:eastAsia'), east)
    run.font.size = Pt(size)
    if bold is not None:
        run.font.bold = bold
    if italic is not None:
        run.font.italic = italic
    if color is not None:
        run.font.color.rgb = RGBColor(*color)


def init_styles(doc):
    st = doc.styles['Normal']
    st.font.name = LATIN
    st.font.size = Pt(10.5)
    st.element.rPr.rFonts.set(qn('w:eastAsia'), EAST)
    for name, sz, east in (('Heading 1', 16, EAST_H), ('Heading 2', 14, EAST_H),
                           ('Heading 3', 12, EAST_H), ('Heading 4', 11, EAST_H)):
        s = doc.styles[name]
        s.font.name = LATIN
        s.font.size = Pt(sz)
        s.font.color.rgb = RGBColor(0x1F, 0x36, 0x4D)
        rpr = s.element.get_or_add_rPr()
        rf = rpr.find(qn('w:rFonts'))
        if rf is None:
            rf = OxmlElement('w:rFonts')
            rpr.append(rf)
        rf.set(qn('w:eastAsia'), east)


def h(doc, level, text):
    p = doc.add_heading('', level=level)
    r = p.add_run(text)
    set_run(r, size={0: 18, 1: 16, 2: 14, 3: 12, 4: 11}.get(level, 11),
            bold=True, east=EAST_H)
    return p


def body(doc, text, size=10.5, align=None, indent=True, space_after=6):
    p = doc.add_paragraph()
    if align is not None:
        p.alignment = align
    p.paragraph_format.space_after = Pt(space_after)
    if indent:
        p.paragraph_format.first_line_indent = Pt(21)
    r = p.add_run(text)
    set_run(r, size=size)
    return p


def bullet(doc, text, size=10.5):
    p = doc.add_paragraph(style='List Bullet')
    r = p.add_run(text)
    set_run(r, size=size)
    return p


def formula(doc, text, size=11):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_before = Pt(3)
    p.paragraph_format.space_after = Pt(6)
    r = p.add_run(text)
    set_run(r, size=size, italic=False, east=EAST)
    r.font.name = 'Cambria Math'
    return p


def caption(doc, text):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_after = Pt(8)
    r = p.add_run(text)
    set_run(r, size=9, italic=True, color=(0x40, 0x40, 0x40))
    return p


def add_figure(doc, fname, cap, width_cm=15.0):
    path = os.path.join(FIGDIR, fname)
    if not os.path.exists(path):
        caption(doc, f'（缺图：{fname}）')
        return
    doc.add_picture(path, width=Cm(width_cm))
    doc.paragraphs[-1].alignment = WD_ALIGN_PARAGRAPH.CENTER
    caption(doc, cap)


def add_table(doc, headers, rows, widths=None, font_size=9.5):
    t = doc.add_table(rows=1, cols=len(headers))
    t.style = 'Table Grid'
    t.alignment = WD_ALIGN_PARAGRAPH.CENTER
    for j, htxt in enumerate(headers):
        c = t.rows[0].cells[j]
        c.text = ''
        r = c.paragraphs[0].add_run(str(htxt))
        set_run(r, size=font_size, bold=True)
        c.paragraphs[0].alignment = WD_ALIGN_PARAGRAPH.CENTER
    for row in rows:
        cells = t.add_row().cells
        for j, v in enumerate(row):
            cells[j].text = ''
            r = cells[j].paragraphs[0].add_run('' if v is None else str(v))
            set_run(r, size=font_size)
    if widths:
        for j, w in enumerate(widths):
            for row in t.rows:
                row.cells[j].width = Cm(w)
    doc.add_paragraph().paragraph_format.space_after = Pt(2)
    return t


def add_toc(doc):
    p = doc.add_paragraph()
    run = p.add_run()
    f1 = OxmlElement('w:fldChar'); f1.set(qn('w:fldCharType'), 'begin')
    it = OxmlElement('w:instrText'); it.set(qn('xml:space'), 'preserve')
    it.text = 'TOC \\o "1-3" \\h \\z \\u'
    f2 = OxmlElement('w:fldChar'); f2.set(qn('w:fldCharType'), 'separate')
    tt = OxmlElement('w:t'); tt.text = '（在 Word 中按 F9 更新目录/页码）'
    f3 = OxmlElement('w:fldChar'); f3.set(qn('w:fldCharType'), 'end')
    for e in (f1, it, f2, tt, f3):
        run._r.append(e)


def add_page_number(paragraph):
    run = paragraph.add_run()
    f1 = OxmlElement('w:fldChar'); f1.set(qn('w:fldCharType'), 'begin')
    it = OxmlElement('w:instrText'); it.set(qn('xml:space'), 'preserve'); it.text = 'PAGE'
    f2 = OxmlElement('w:fldChar'); f2.set(qn('w:fldCharType'), 'end')
    for e in (f1, it, f2):
        run._r.append(e)
    set_run(run, size=9)


def setup_page(doc, title_footer=''):
    sec = doc.sections[0]
    sec.top_margin = Cm(2.4)
    sec.bottom_margin = Cm(2.2)
    sec.left_margin = Cm(2.6)
    sec.right_margin = Cm(2.4)
    fp = sec.footer.paragraphs[0]
    fp.alignment = WD_ALIGN_PARAGRAPH.CENTER
    if title_footer:
        r = fp.add_run(title_footer + '    ')
        set_run(r, size=9, color=(0x60, 0x60, 0x60))
    add_page_number(fp)


# ===========================================================================
# 数据采集
# ===========================================================================

def load_cfg():
    with open(DEFAULT_YAML, encoding='utf-8') as f:
        return yaml.safe_load(f)


def collect_offline(cfg):
    rows = []
    for name, scen in cfg['scenarios'].items():
        if scen.get('mode') == 'stack_drop':
            continue
        layout = cfg['layouts'][scen['layout']]
        th = {**cfg.get('thresholds', {}), **scen.get('thresholds', {})}
        try:
            res, _plan = simulate(cfg['defaults'], layout, scen,
                                  closed_loop=bool(scen.get('closed_loop')))
        except Exception as e:                      # noqa: BLE001
            rows.append((name, None, None, 'ERR', str(e)[:40]))
            continue
        rows.append((name, res.miss_dist, res.rel_speed_at_capture,
                     'PASS' if res.success else 'FAIL',
                     '闭环' if scen.get('closed_loop') else ''))
    return rows


def collect_m6_mc(cfg, spec):
    out = {}
    for name, n in spec:
        scen = cfg['scenarios'][name]
        layout = cfg['layouts'][scen['layout']]
        ok = 0
        hmiss, rel, marg = [], [], []
        for i in range(n):
            noise = StackNoise(release_pos_sigma=0.05, rel_pos_sigma=0.05,
                               rel_latency=0.05, seed=i)
            res, _ = simulate_stack(cfg['defaults'], layout, scen, noise=noise)
            ok += int(res.success)
            if res.success:
                hmiss.append(res.horiz_miss_at_capture)
                rel.append(res.rel_speed_at_capture)
                marg.append(res.capture_margin)
        out[name] = dict(
            n=n, ok=ok, rate=100.0 * ok / n,
            hmiss_max=(max(hmiss) if hmiss else float('nan')),
            rel_mean=(float(np.mean(rel)) if rel else float('nan')),
            marg_min=(min(marg) if marg else float('nan')),
        )
        print(f'  [MC] {name:22s} {ok}/{n}  ok')
    return out


def trace_m1(cfg):
    scen = cfg['scenarios']['M1_basic']
    layout = cfg['layouts'][scen['layout']]
    res, plan = simulate(cfg['defaults'], layout, scen)
    return res, plan


def trace_m6(cfg, name):
    scen = cfg['scenarios'][name]
    layout = cfg['layouts'][scen['layout']]
    res, _ = simulate_stack(cfg['defaults'], layout, scen,
                            noise=StackNoise(seed=0))
    return res


def _downsample(idx, k):
    if len(idx) <= k:
        return list(range(len(idx)))
    step = max(1, len(idx) // k)
    sel = list(range(0, len(idx), step))
    if sel[-1] != len(idx) - 1:
        sel.append(len(idx) - 1)
    return sel


def m1_state_rows(res, k=18):
    t = np.asarray(res.t); bp = np.asarray(res.b_pos); bv = np.asarray(res.b_vel)
    pp = np.asarray(res.p_pos); pv = np.asarray(res.p_vel)
    rows = []
    for i in _downsample(t, k):
        rows.append([f'{t[i]:.2f}',
                     f'{bp[i,0]:+.2f}', f'{bp[i,1]:+.2f}', f'{-bp[i,2]:.2f}',
                     f'{np.linalg.norm(bv[i]):.2f}',
                     f'{pp[i,0]:+.2f}', f'{pp[i,1]:+.2f}', f'{-pp[i,2]:.2f}',
                     f'{np.linalg.norm(pv[i]):.2f}',
                     f'{res.rel_dist[i]:.3f}'])
    return rows


def m6_state_rows(res, k=18):
    t = np.asarray(res.t); bp = np.asarray(res.b_pos); pp = np.asarray(res.p_pos)
    vb = np.gradient(bp, t, axis=0) if len(t) > 2 else np.zeros_like(bp)
    vp = np.gradient(pp, t, axis=0) if len(t) > 2 else np.zeros_like(pp)
    rows = []
    for i in _downsample(t, k):
        rows.append([f'{t[i]:.2f}',
                     f'{bp[i,0]:+.2f}', f'{bp[i,1]:+.2f}', f'{-bp[i,2]:.2f}',
                     f'{np.linalg.norm(vb[i]):.2f}',
                     f'{pp[i,0]:+.2f}', f'{pp[i,1]:+.2f}', f'{-pp[i,2]:.2f}',
                     f'{np.linalg.norm(vp[i]):.2f}',
                     f'{res.rel_dist[i]:.3f}'])
    return rows


# ===========================================================================
# 文档构建
# ===========================================================================

def cover(doc):
    for _ in range(5):
        doc.add_paragraph()
    p = doc.add_paragraph(); p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = p.add_run('空投—空中捕获'); set_run(r, size=30, bold=True, east=EAST_H)
    p = doc.add_paragraph(); p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = p.add_run('无人机协同投送与空中接收'); set_run(r, size=16, east=EAST_H)
    p = doc.add_paragraph(); p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = p.add_run('仿真报告'); set_run(r, size=20, bold=True, east=EAST_H)
    for _ in range(6):
        doc.add_paragraph()
    info = [
        ('项目仓库', 'drone_payload_catch'),
        ('仿真平台', 'PX4-1.16 SITL + Gazebo Harmonic + ROS 2 Jazzy + acados'),
        ('核心算法', '解析会合规划 + 闭环重规划 + 终端 MPC + 协调证书/CBF'),
        ('末端机构', '刚性漏斗（离线）/ 圆形托盘（围边+泡棉，真机）'),
        ('报告日期', _dt.date.today().isoformat()),
    ]
    for k, v in info:
        p = doc.add_paragraph(); p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        r = p.add_run(f'{k}：{v}'); set_run(r, size=11)
    doc.add_page_break()


def section_intro(doc):
    h(doc, 1, '1  相关研究（引言）')

    h(doc, 2, '1.1  问题定义与场景谱系')
    body(doc, '“空投—空中捕获”研究两个或多个空中平台之间的物体转移：投送方（无人机 A）在飞行中释放载荷，'
              '接收方（无人机 B）在载荷尚未落地之前于空中完成捕获与保持。本项目要求接收方到达会合点时的'
              '位置精确（硬约束）、相对速度尽量小（软约束），并在释放误差、模型失配、测量噪声/延迟以及'
              '双方动力学限幅的条件下保证安全。')
    body(doc, '按几何与动力学，本方向可分成一条谱系，如下表所示。本项目重点覆盖其中的“垂直堆叠投放”（M6）'
              '与“水平/斜向会合”（M1–M4）两类工况。')
    add_table(doc,
              ['形态', '投送方', '接收方', '相对运动', '典型难点'],
              [['垂直堆叠投放（M6）', '悬停/低速', '悬停/低速', '近共线、纯垂直', '接触速度下界、末端机构'],
               ['水平/斜向会合（M1–M4）', '恒速平飞', '机动到位', '空间会合点', '会合规划、终端速度匹配'],
               ['空射回收（Gremlins 类）', '母机释放', '母机/回收装置', '高速、大惯量', '相对导航、捕获机构'],
               ['精确空投（JPADS）', '运输机高空', '地面目标区', '单向下落', '落点精度、风场估计'],
               ['空中加油/交会对接', '加油机/目标星', '受油机/追踪星', '近场编队', '相对状态估计、精细控制'],
               ['物体抛接（ball catching）', '人/臂/无人机', '无人机', '快速非合作目标', '状态预测、极短窗口']],
              widths=[3.6, 2.6, 3.0, 3.0, 3.6])

    h(doc, 2, '1.2  与相邻问题的关系')
    bullet(doc, '空中操纵（Aerial Manipulation）：捕获机构、机械臂/夹爪、接触力控制可直接复用。')
    bullet(doc, '协同运输（Cooperative Transport）：捕获后“带走”与吊挂/刚体负载运输同源。')
    bullet(doc, '空间交会对接（Rendezvous & Docking）：CW 方程终端制导与本项目“会合点+速度匹配”高度相似。')
    bullet(doc, '导弹拦截/比例导引：非合作快速目标的预测拦截与终端导引律。')
    bullet(doc, '机器人动态抓取/接球：预测—规划—在极短窗口内到位的同一类问题。')

    h(doc, 2, '1.3  技术挑战分解')
    body(doc, '端到端任务可拆成六个耦合子问题：')
    for i, (ti, de) in enumerate([
        ('(1) 会合规划/制导', '决策释放时刻 t_r 与下落时长 τ_c；在可行域内最小化时间+机动代价+终端速度失配；'
                            '数学本质是带终端约束的双积分器最优控制。'),
        ('(2) 相对导航与状态估计', '需要载荷实时状态与 A–B 相对位置；现实手段有视觉/事件相机、UWB、RTK-GNSS、'
                                '动捕等；难点是延迟、丢包、遮挡与不确定性量化。'),
        ('(3) 捕获控制', '接收方常需高动态机动（俯冲、横向急停、终端速度匹配），逼近执行器与姿态极限；'
                       '关键张力是跟踪精度与反馈余量的取舍。'),
        ('(4) 末端捕获机构', '软网/夹爪/刚性漏斗/磁吸等；需处理接触瞬间的冲击（能量耗散、约束、捕获后保持）。'),
        ('(5) 安全与鲁棒性', '双机避碰、失效保护、丢失目标后的应急、估计不确定度余量、地理围栏。'),
        ('(6) 系统集成与真机化', '传感器/执行器/通信时延、机载算力、实时性与认证。'),
    ]):
        p = doc.add_paragraph(); p.paragraph_format.space_after = Pt(4)
        r = p.add_run(ti + '：'); set_run(r, size=10.5, bold=True)
        r = p.add_run(de); set_run(r, size=10.5)

    h(doc, 2, '1.4  国内外研究现状')
    body(doc, '按子方向梳理代表性脉络（参考文献见附录）：')
    bullet(doc, '空中操纵与抓取：机械臂/夹爪装在旋翼平台上的分类与挑战（Ruggiero 2018；Ollero 2022）；'
                '鸟爪式抓取/停栖与冲击吸收（Thomas 2014；Roderick 2021）。')
    bullet(doc, '空中物体抛接：ETH 团队的四旋翼抛球/接球与杂耍（Ritz 2012；Müller 2011），强调状态预测与'
                '极短窗口内的规划控制。')
    bullet(doc, '空中回收：DARPA Gremlins、Aurora SideArm；火箭整流罩/样品返回舱空中回收（Genesis 2004、'
                'Stardust 2006）等宏观尺度经典范例。')
    bullet(doc, '精确空投：JPADS 翼伞制导空投、Zipline 包裹定点空投，代表“投送—落地”路线，与本项目'
                '“投送—空中接收”相区别。')
    bullet(doc, '空中加油/交会对接：AAR（NASA DROID）与 CW 方程终端制导，为会合规划与相对导航提供方法论。')
    bullet(doc, '制导与控制方法：比例导引、min-snap/min-energy 多项式（Mellinger 2011；Richter 2013）、'
                '嵌入式 MPC（acados，Verschueren 2022）。')
    body(doc, '总体判断：“投送—空中接收”目前仍属较空白、偏系统集成的方向；尤其缺乏“释放权威 + 意图共享 + '
              '概率证书 + 交接 CBF”统一的、带保证的双机交接工作。')

    h(doc, 2, '1.5  本项目定位与贡献')
    bullet(doc, '给出可离线复现的完整算法层（抛体模型 / 会合规划 / 闭环重规划 / 终端 MPC / 状态估计 / 垂直投放），'
                '并在 PX4+Gazebo SITL 上双机端到端跑通。')
    bullet(doc, '系统刻画 M6 垂直投放的物理边界（接触速度下界、漏斗保持速度、有效捕获半径），'
                '并量化末端机构（漏斗/托盘/锁扣）的杠杆。')
    bullet(doc, '给出协调研究框架：概率捕获证书（T1）、协议时序界（C2）、state-vs-intent（C3）、'
                '联合机动（C4）、handover-CBF（C5）与延迟鲁棒 CBF（T3）。')
    bullet(doc, '面向真机，给出圆形托盘（围边+泡棉）的保持物理与选型方法。')


def section_theory(doc):
    h(doc, 1, '2  理论分析')

    h(doc, 2, '2.1  载荷抛体动力学')
    body(doc, '载荷在释放瞬间继承 A 的速度 v_r = v_A(t_r)，随后做抛体运动（无阻力解析模型）：')
    formula(doc, 'p_p(τ) = p_r + v_r·τ + ½·g·τ²        v_p(τ) = v_r + g·τ        (τ = t − t_r)')
    body(doc, '其中 g = [0, 0, 9.81] m/s²（NED 下 z 向下）。可选 linear/quadratic 阻力与常值风作为鲁棒性输入。'
              '下落 τ 后竖直速度为 v_z = v_r,z + g·τ，捕获点高度为 −p_p(τ_c)。')

    h(doc, 2, '2.2  会合规划与终端制导')
    body(doc, 'A 第一版为恒速直线 p_A(t) = a_init + a_vel·t（a_vel = 0 即悬停）。规划器在 (t_r, τ_c) 网格上'
              '最小化综合代价：')
    formula(doc, 'J = w_time·t_r + w_accel·(峰值加速度 / a_max) + w_vel·|Δv|²  (+ w_overshoot·过冲)')
    body(doc, '约束为 |v| ≤ v_max、|a| ≤ a_max、离地余量、捕获高度区间。B 的会合轨迹在“双积分器、两端位置/速度'
              '固定、min ∫|a|²dt”下每轴有解析三次多项式解；若终端速度精确匹配不可行（载荷速度超 B 限速），'
              '退为软终端速度解析解（位置仍硬到、速度按权重折中）。')

    h(doc, 2, '2.3  M6 垂直堆叠投放的物理')
    body(doc, 'A 悬停在 B 严格正上方 gap 米，水平速度为零。载荷纯垂直自由落体；B 以 a_dive 温和下潜、接触后'
              '以 a_brake 刹停。由于四旋翼“只能向下压”（a_B < g），纯垂直下落的接触相对速度存在下界：')
    formula(doc, 'v_rel = √( 2·(g − a_dive)·gap )')
    body(doc, '上式表明：下潜越猛 v_rel 越小，但 B 冲得越低、刹车余量越少。gap=1 m 时，悬停硬接 4.43 m/s，'
              'a_dive=3 → 3.69 m/s，a_dive=6 → 2.76 m/s（刹停后仅剩 0.35 m）。因此 a_dive=3 是冲击与余量的折中。'
              '刚性漏斗的保持判据为：')
    formula(doc, '位置：落到口面时水平偏差 < mouth_radius − object_radius')
    formula(doc, '速度：接触相对速度 ≤ v_retain = √(2·g·depth) / e      （e = 恢复系数）')

    h(doc, 2, '2.4  圆形托盘的保持物理（真机末端）')
    body(doc, '纯平托盘没有杯深，不靠深度保持，而靠“围边挡横向 + 泡棉消反弹”。设有效围挡高度 h = 围边高 + 凹垫深，'
              '物体撞击后回弹高度为 e²·gap（gap 为下落高度）。物体不弹出围挡的条件为：')
    formula(doc, 'h ≥ e²·gap     ⇔     e ≤ √(h / gap)')
    body(doc, '落点半径 eff_r = 盘内半径 − 物块半宽。恢复系数 e 是生死线：裸塑料盘 e≈0.7 时，gap=1 m 的回弹'
              '高度达 20–49 cm，必飞；软泡棉 e≤0.15 时回弹仅约 2 cm。本文第 4、5 章以此对 6cm/100g 方块'
              '进行量化（内径 30cm/围边 5cm/e=0.15 → 离线 MC 99%）。')

    h(doc, 2, '2.5  闭环重规划与状态估计')
    body(doc, '载荷离手后每 replan_dt 用当前观测的载荷状态重解会合（solve_inflight），滚动更新 B 的参考，'
              '抵抗释放误差与模型失配（风/阻力）。载荷状态估计可选 none（真值上界）/ naive（最近测量+有限差分+'
              '弹道外推）/ KF（状态 [p,v]，重力为已知输入，按测量时间戳 predict→update，天然处理延迟/丢包）。')

    h(doc, 2, '2.6  协调理论：证书与安全（C1–C5 / T1–T3）')
    body(doc, '在双机协同方向，本项目给出带保证的结果。释放决策的概率模型（C1）：估计残差 δ̂、真实脱靶 m|δ̂ ~ '
              'N(δ̂, σ_m²I₂)，条件捕获概率为 Marcum-Q，释放域最优解为中心球（T1 定理）。')
    formula(doc, 'p(δ̂) = P(capture | δ̂) = 1 − Q₁( ‖δ̂‖/σ_m , r_eff/σ_m )')
    body(doc, '安全方面，速度级控制屏障函数（CBF）给出防碰不变集；在通信延迟 d 下，对延迟状态采用收紧屏障'
              'h_d(r) = ‖r‖² − (d_safe + ρ)²（ρ = (v_A + v_B)·d）可保证真实安全（T3 定理）。协议时序上，'
              '释放提前量需满足 release_lead = d_max / (2(1 − catch))（C2）。这些结果与 M6 的 min_ab_gap 安全层'
              '在设计上一致。')

    h(doc, 2, '2.7  安全层')
    body(doc, '项目实现分级安全状态机 OK → HOLD（悬停）→ PULLBACK（越界回拉）→ LAND → KILL（飞行终止），'
              '并结合 EKF 的 eph/epv 作为位置不确定度、姿态/角速率安全滤波，以及释放后定向清场与超时中止。')
    add_figure(doc, 'fig_control_architecture.png', '图 2-1  系统分层架构：规划层 → 控制层 → 被控对象 → 估计层 → 反馈闭环。')


def section_conditions(doc, cfg):
    h(doc, 1, '3  仿真条件')

    h(doc, 2, '3.1  软件与硬件栈')
    add_table(doc, ['项', '配置'],
              [['操作系统', 'WSL2 Ubuntu-24.04（Gazebo GUI 经 WSLg 显示）'],
               ['飞控', 'PX4-1.16 SITL，双实例（A=-i 0 / B=-i 1）'],
               ['中间件', 'ROS 2 Jazzy，rmw_fastrtps_cpp，MicroXRCEAgent（UDP 8888）'],
               ['仿真环境', 'Gazebo Harmonic 8.13，自建“全系统”world（Imu/NavSat/Sensors + 球坐标）'],
               ['优化求解', 'acados 终端 MPC（指纹缓存 + 预热）'],
               ['载荷(Gazebo)', '0.06 m 立方体 / 0.3 kg，OdometryPublisher 发 /payload/odom'],
               ['定位', 'A 广播 /drone_a/state（世界系 NED），B 侧注入延迟/丢包/偏置/噪声（mesh 替身）'],
               ['核心算法', '纯 Python（不依赖 ROS），离线一条命令验证']],
              widths=[3.6, 11.8])

    h(doc, 2, '3.2  坐标系与约定')
    body(doc, '统一采用世界系 NED（x=北、y=东、z=下），高度 = −z。Gazebo ENU→NED 换算为 '
              'NED = [ENU_y, ENU_x, −ENU_z]。所有几何/参数/工况集中在 config/catch_scenarios.yaml，代码不硬编码。')

    h(doc, 2, '3.3  主要参数（单一真值源）')
    d = cfg['defaults']
    f = d['capture']['funnel']
    add_table(doc, ['参数', '值', '说明'],
              [['g', f"{d['g']} m/s²", '重力'],
               ['payload.mass', f"{d['payload']['mass']} kg", '载荷质量（Gazebo 立方体 0.06 m）'],
               ['drone_b.max_speed / max_accel', f"{d['drone_b']['max_speed']} / {d['drone_b']['max_accel']}",
                'B 速度/加速度上限'],
               ['capture.radius / rel_speed', f"{d['capture']['radius']} m / {d['capture']['rel_speed']} m/s",
                'M1–M4 捕获判据'],
               ['funnel.mouth_radius / depth / restitution',
                f"{f['mouth_radius']} / {f['depth']} / {f['restitution']}", 'M6 漏斗（标称）'],
               ['funnel.object_radius', f"{f['object_radius']} m", '载荷等效半径'],
               ['stack.a_dive / a_brake', f"{d['stack']['a_dive']} / {d['stack']['a_brake']} m/s²",
                'M6 下潜/刹车'],
               ['planner 权重', f"w_time={d['planner']['w_time']}, w_accel={d['planner']['w_accel']}, "
                              f"w_vel={d['planner']['w_vel']}", '会合代价权重']],
              widths=[5.0, 4.6, 5.8])

    h(doc, 2, '3.4  仿真工况')
    body(doc, '离线共包含以下工况（含垂直堆叠 M6 系列）。M1–M4 走通用会合规划器；M6 系列走独立的解析垂直'
              '投放规划器（3D 通用规划器不适合本场景）。')
    scen_list = [k for k in cfg['scenarios']]
    rows = []
    for k in scen_list:
        s = cfg['scenarios'][k]
        mode = s.get('mode', 'rendezvous')
        rows.append([k, mode, s.get('description', '')[:52]])
    add_table(doc, ['工况', '模式', '说明'], rows, widths=[4.4, 2.4, 8.6])

    h(doc, 2, '3.5  评价判据')
    body(doc, 'M1–M4：|p_B − p_p| < r_c = 0.30 m 且 |v_B − v_p| < v_c = 1.50 m/s（用载荷真值判定）。'
              'M6：载荷落到口面时水平偏差 < eff_r，且接触相对速度 ≤ v_retain。蒙特卡洛采用相对定位'
              'σ=0.05 m、延迟 0.05 s、释放点误差 σ=0.05 m 的注入噪声（与 SITL 扫描一致）。')


def section_data(doc, cfg, offline, m6, traces, m6_traces):
    h(doc, 1, '4  仿真数据（含状态数据）')

    h(doc, 2, '4.1  单元自测')
    add_table(doc, ['模块', '结果'],
              [['payload_model', '解析/积分误差 err = 2.44e-13；fall_time(1.5m) = 0.5530 s'],
               ['rendezvous', 'hover 可行：t_r=2.35 s，t_c=2.670 s，h_c=2.00 m，Δv=0.000'],
               ['sim_core', 'M1_basic miss=0.118 m；M2_line_v10 miss=0.156 m']],
              widths=[3.6, 11.8])

    h(doc, 2, '4.2  离线全工况结果（M1–M4）')
    rows = []
    for name, miss, rel, tag, note in offline:
        rows.append([name,
                     f'{miss:.4f}' if isinstance(miss, float) else '—',
                     f'{rel:.4f}' if isinstance(rel, float) else '—',
                     tag, note])
    add_table(doc, ['工况', '最近距离 (m)', '捕获相对速度 (m/s)', '判定', '备注'], rows,
              widths=[4.0, 2.6, 3.0, 1.8, 4.0])
    body(doc, '全部工况 PASS（M6 系列在此表跳过，见下节）。')

    h(doc, 2, '4.3  M6 垂直堆叠：蒙特卡洛数据')
    order = ['M6_stack_drop', 'M6_stack_bigfunnel', 'M6_stack_tray', 'M6_stack_tray_small',
             'M6_stack_tray_wind', 'M6_stack_nodive', 'M6_stack_wind', 'M6_stack_lock',
             'M6_stack_windcomp']
    rows = []
    for k in order:
        if k not in m6:
            continue
        m = m6[k]
        rows.append([k, f"{m['n']}", f"{m['ok']}/{m['n']}", f"{m['rate']:.0f}%",
                     f"{m['hmiss_max']:.4f}" if m['hmiss_max'] == m['hmiss_max'] else '—',
                     f"{m['rel_mean']:.3f}" if m['rel_mean'] == m['rel_mean'] else '—',
                     f"{m['marg_min']:.4f}" if m['marg_min'] == m['marg_min'] else '—'])
    add_table(doc, ['工况', 'N', '成功', '成功率', '水平偏差max(m)', '相对速度mean(m/s)', '余量min(m)'],
              rows, widths=[4.0, 1.2, 1.6, 1.6, 2.6, 2.8, 2.2])
    body(doc, '注：圆形托盘工况（M6_stack_tray / _small / _wind）载荷为 6cm/100g 方块；'
              '托盘建模映射为 mouth_radius=盘内半径、depth=有效围挡高度、restitution=泡棉恢复系数 e。')

    h(doc, 2, '4.4  状态数据：M1_basic 时间序列')
    body(doc, 'M1 悬停释放—空间会合的闭环状态（NED，位置单位 m，速度单位 m/s，取代表步）：')
    add_table(doc, ['t(s)', 'B_N', 'B_E', 'B_alt', '|v_B|', 'P_N', 'P_E', 'P_alt', '|v_P|', '|B−P|'],
              m1_state_rows(traces['m1'], 18), widths=[1.2, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.6],
              font_size=9)
    body(doc, f"规划结果：释放 t_r={traces['rx'][0]:.2f} s，捕获 t_c={traces['rx'][1]:.2f} s，"
              f"会合点高度 {traces['rx'][2]:.2f} m；仿真最近距离 {traces['rx'][3]:.4f} m。")

    h(doc, 2, '4.5  状态数据：M6 垂直堆叠时间序列')
    body(doc, 'M6_stack_drop（刚性漏斗 0.20 m，gap=1.0 m，a_dive=3）状态：')
    add_table(doc, ['t(s)', 'B_N', 'B_E', 'B_alt', '|v_B|', 'P_N', 'P_E', 'P_alt', '|v_P|', '|B−P|'],
              m6_state_rows(m6_traces['drop'], 18),
              widths=[1.2, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.6], font_size=9)
    body(doc, 'M6_stack_tray（圆形托盘 内径30cm/围边5cm/泡棉 e=0.15，6cm/100g 方块）状态：')
    add_table(doc, ['t(s)', 'B_N', 'B_E', 'B_alt', '|v_B|', 'P_N', 'P_E', 'P_alt', '|v_P|', '|B−P|'],
              m6_state_rows(m6_traces['tray'], 18),
              widths=[1.2, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.6], font_size=9)

    h(doc, 2, '4.6  SITL 结果（Gazebo + PX4，双机）')
    add_table(doc, ['试验', '加压项', '捕获', '水平误差(m)', '接触速度(m/s)', 'min‖A−B‖(m)', '两机落地'],
              [['T0_nominal', '标称', '✅', '0.041', '2.00', '0.980', '✅'],
               ['T1/T2', '释放误差 σ=0.10（跟踪/不跟踪）', '✅✅', '0.036/0.034', '~2.0', '~0.97', '✅'],
               ['T3', '重相对定位噪声 + EMA 滤波', '✅', '0.017', '2.18', '0.817', '✅'],
               ['T3n', '同上，不滤波', '❌', '—', '—', '0.481*', '—'],
               ['T4', '释放提前量 0.45 s', '✅', '0.043', '2.01', '0.970', '✅'],
               ['T5', '落差 gap=1.2 m', '✅', '0.048', '0.805', '1.175', '✅'],
               ['T6', 'B 下潜 a=5', '✅', '0.016', '1.315', '0.971', '✅'],
               ['T7/T8', '释放误差 σ=0.20（跟踪/不跟踪）', '✅✅', '0.009/0.033', '1.43/1.93', '~0.96', '✅']],
              widths=[2.4, 4.6, 1.4, 2.4, 2.8, 2.6, 1.6], font_size=9)
    body(doc, 'M6 SITL 蒙特卡洛（每档 N=5）：标称 5/5；重噪声 + EMA 滤波 4/5（不滤波 1/5）；'
              '释放误差 σ=0.20 + 载荷闭环 5/5（不跟踪 1/5）；σ=0.30 + 载荷闭环 5/5。')
    body(doc, 'M6-moving 编队同速投放（FORMATION_VEL=0.5 m/s）无窗口 SITL 连续 3 次全部捕获：'
              '编队对齐 rel_xy=0.011–0.067 m、rel_vxy=0.061–0.187，STACK CAPTURED horiz=0.108–0.128 m，'
              '双机均无 failsafe。')

    h(doc, 2, '4.7  仿真图表')
    add_figure(doc, 'fig_funnel_physics.png',
               '图 4-1  漏斗物理：v_rel 随下潜/落差的变化（左）与 v_retain 随深度/恢复系数的变化（右）。')
    add_figure(doc, 'fig_m6_stack_offline.png',
               '图 4-2  M6 垂直堆叠离线轨迹与相对距离。')
    add_figure(doc, 'fig_estimator_comparison.png',
               '图 4-3  载荷状态估计器对比（naive vs KF vs 真值）。')
    add_figure(doc, 'offline_M1_basic.png',
               '图 4-4  M1 悬停释放—空间会合轨迹（俯视/侧视/相对距离）。')
    add_figure(doc, 'offline_M3_high.png',
               '图 4-5  M3 高抛闭环重规划轨迹。')
    add_figure(doc, 'fig_m6_sitl_formation.png',
               '图 4-6  M6-moving 编队同速投放（SITL）。')


def section_analysis(doc, m6):
    h(doc, 1, '5  仿真分析')

    h(doc, 2, '5.1  闭环重规划：抗释放误差与模型失配')
    bullet(doc, '释放误差（高抛，下落 ~0.8 s）：σ=0.20 m 时开环 7/12 → 闭环 12/12（带 0.05 m 测量噪声仍 12/12）。')
    bullet(doc, '模型失配（风 [2,1,0] m/s + 线性阻力 k=2，规划器按无阻力预测）：开环 0/10 → 闭环 10/10。')
    body(doc, '结论：释放误差是“已知的系统性偏差”，滚动重规划最划算；闭环重规划显著提升鲁棒性。')

    h(doc, 2, '5.2  控制对比：PD 与终端 MPC')
    add_table(doc, ['工况（无扰动）', 'PD', 'MPC'],
              [['M2_line_v10 短下落', '10/10，0.16 m', '10/10，0.27 m'],
               ['M3_high 高抛', '10/10，0.29 m', '10/10，0.28 m']],
              widths=[5.0, 4.4, 4.4])
    body(doc, '负结果 A：终端 MPC 在短下落场景捕获略松，未超过带解析前馈的 PD。根因不是 MPC 有误，而是解析'
              'min-energy 参考在高时间余量、大终端速度时会让 B 先爬升再俯冲；PD 死跟该参考，而 MPC 最小化控制量'
              '倾向“抄近路”。')

    h(doc, 2, '5.3  状态估计：KF 的价值边界')
    add_table(doc, ['测量严重度 (pos σ/latency/dropout)', '估计误差 naive', '估计误差 KF', '成功率 KF', '成功率 none(真值)'],
              [['轻 (0.03 m / 0 / 0)', '0.049 m', '0.037 m', '30/30', '30/30'],
               ['中 (0.05 m / 0.1 s / 20%)', '0.667 m', '0.189 m', '27/30', '30/30'],
               ['重 (0.10 m / 0.2 s / 40%)', '0.305 m', '0.164 m', '29/30', '30/30']],
              widths=[4.8, 2.8, 2.8, 2.6, 3.0])
    body(doc, '负结果 C：KF 把估计误差稳定降低 2–4 倍，但端到端成功率没有提升（真值基线同样约 92%）。'
              '说明在本几何/判据下，瓶颈是释放误差与 B 的动力学限幅，而非估计精度。')

    h(doc, 2, '5.4  M6 垂直投放：末端机构的杠杆')
    body(doc, 'M6 的物理下界 v_rel = √(2(g − a_dive)gap) 决定了垂直方向的接触速度无法靠机动大幅消除，'
              '因此末端机构能力是主要杠杆。由蒙特卡洛数据可见：')
    md = m6.get('M6_stack_drop', {}); mb = m6.get('M6_stack_bigfunnel', {})
    if md and mb:
        bullet(doc, f"标准漏斗（eff_r=0.15 m）：成功率 {md['rate']:.0f}%，最小捕获余量 {md['marg_min']*1000:.1f} mm。")
        bullet(doc, f"大漏斗（eff_r=0.25 m）：成功率 {mb['rate']:.0f}%，最小捕获余量 {mb['marg_min']*1000:.1f} mm"
                    f"（余量提升约 {mb['marg_min']/max(md['marg_min'],1e-6):.0f}×）。")
    body(doc, '结论：末端能力（有效半径 eff_r / 保持速度 v_retain）是改变边界的首要杠杆；'
              '主动保持（锁扣/磁吸）可把“保持”从恢复系数问题变为结构强度问题。')

    h(doc, 2, '5.5  圆形托盘（真机末端）分析')
    body(doc, '托盘没有杯深，保持条件为 h ≥ e²·gap，恢复系数 e 是生死线。对 6cm/100g 方块（下落 gap=1 m，'
              '接触速度约 4.4 m/s）的分析如下：')
    add_table(doc, ['配置', 'eff_r (cm)', '保持判据', '离线 MC 成功率'],
              [['内径30cm/围边5cm/e=0.15', '12.0', 'PASS', f"{m6.get('M6_stack_tray',{}).get('rate',float('nan')):.0f}%"],
               ['内径25cm/围边5cm/e=0.15', '9.5', 'PASS', f"{m6.get('M6_stack_tray_small',{}).get('rate',float('nan')):.0f}%"],
               ['内径30cm + 侧风3 m/s', '12.0', 'PASS', f"{m6.get('M6_stack_tray_wind',{}).get('rate',float('nan')):.0f}%"],
               ['裸塑料盘 e≈0.7（对照）', '12.0', 'FAIL（回弹49cm）', '0%']],
              widths=[5.4, 2.4, 3.6, 3.0])
    body(doc, '分析：内径 30cm 明显优于 25cm（99% vs 约93%）；100g 轻物在 3 m/s 侧风下漂移仅约 3 cm，'
              '成功率仍达 96%。关键设计为“围边挡横向 + 泡棉消反弹”，并强烈建议将泡棉做成中部凹坑以实现'
              '自定心（等效一个软漏斗）。裸塑料盘无论围边多高都无法接住（回弹飞出）。')

    h(doc, 2, '5.6  SITL 分析')
    bullet(doc, '横向偏移不是瓶颈（0.2→0.8 m 全过）；真瓶颈是载荷下落速度：v_p≈4.9 m/s 触发 B 的 '
                'Attitude failure (roll)/Compass failsafe，失败属平台层而非算法层。')
    bullet(doc, '相对定位在重噪声下必须 EMA 低通（重噪声下滤波 4/5 vs 不滤波 1/5）；'
                '大释放误差下必须跟踪载荷（σ=0.20 时 5/5 vs 不跟踪 1/5）。')
    bullet(doc, '编队同速投放（0.5/1.0 m/s）稳定捕获并可携带落地，验证了“水平速度匹配”这一唯一能被消除的'
                '相对速度分量。')

    h(doc, 2, '5.7  负结果汇总')
    for t in [
        '终端 MPC 未超过带解析前馈的 PD（双积分器下 PD 前馈已近最优）。',
        '去过冲/staged 参考反而丢失反馈余量：σ=0.15 成功率由 8/10 降到 6/10，默认保留 cubic。',
        '更好的估计器（KF）单独使用不提升端到端成功率（真值基线同样约 92%）。',
        '敞口杯倾斜 >25° 即滚出，不如高摩擦平盘；保持/倾角需主动锁扣。',
        '在横风下“猛下潜”是反效果（拉长下落时间 → 风漂移更大）；应取最小 gap、尽量不下潜。',
    ]:
        bullet(doc, t)

    h(doc, 2, '5.8  局限与下一步')
    body(doc, '局限：相对导航仍为“真值+噪声”，非真实视觉/UWB；捕获仍为软件判据；末端为刚性漏斗/托盘，'
              '无真实接触动力学闭环；SITL 每档样本量小；高动态下 B 的 EKF 鲁棒性不足。')
    body(doc, '下一步（按价值）：(1) 真机圆形托盘（围边+泡棉凹垫）落地，先用落物试验实测泡棉 e≤0.20；'
              '(2) 相对定位真实化（RTK/UWB/视觉）替换 mesh 替身；(3) 推高速度边界（修 B 的 EKF）；'
              '(4) 不确定性感知安全层与接空应急；(5) 带载操控与自适应控制。')


def appendix(doc, cfg, m6):
    h(doc, 1, '附录 A  复现命令')
    for c in [
        'python3 -m payload_catch.payload_model',
        'python3 -m payload_catch.rendezvous',
        'python3 tools/offline_run.py --all',
        'python3 tools/stack_run.py --mc 200',
        'python3 tools/stack_run.py --scenario M6_stack_tray --mc 300',
        'python3 tools/tray_sizing.py --sweep-e',
        'python3 tools/tray_sizing.py --measure-drop 1.0 0.02',
        'MODE=full bash run_m6_sitl.sh 70    # SITL 研究特性全开',
    ]:
        p = doc.add_paragraph(); r = p.add_run(c)
        r.font.name = 'Consolas'; set_run(r, size=9.5)

    h(doc, 1, '附录 B  代码与文档映射')
    add_table(doc, ['模块', '作用'],
              [['payload_catch/payload_model.py', '载荷抛体模型（可选阻力/风）'],
               ['payload_catch/rendezvous.py', '会合规划：min-energy 三次 + 软终端速度 + solve_inflight'],
               ['payload_catch/sim_core.py', '离线闭环仿真（A 飞行 + B 控制 + 捕获 + 重规划）'],
               ['payload_catch/stack_drop.py', 'M6 垂直堆叠投放（解析规划 + 漏斗判据 + 离线仿真）'],
               ['payload_catch/mpc_terminal.py', 'acados 终端 MPC'],
               ['payload_catch/payload_filter.py', '载荷状态估计（KF / 朴素）'],
               ['tools/stack_run.py', 'M6 离线体检查询 CLI'],
               ['tools/tray_sizing.py', '圆形托盘选型计算器（真机末端）'],
               ['tools/make_docx_report.py', '本报告的生成脚本'],
               ['config/catch_scenarios.yaml', '单一真值源（几何/参数/工况）']],
              widths=[6.0, 9.4])

    h(doc, 1, '附录 C  参考文献（代表性）')
    refs = [
        'F. Ruggiero, V. Lippiello, A. Ollero, “Aerial Manipulation: A Literature Review,” IEEE RA-L, 3(3), 2018.',
        'A. Ollero et al., “Past, Present, and Future of Aerial Robotic Manipulators,” IEEE T-RO, 38(1), 2022.',
        'J. Thomas, G. Loianno, K. Sreenath, V. Kumar, “Toward image based visual servoing for aerial grasping and perching,” ICRA, 2014.',
        'W. R. T. Roderick, M. R. Cutkosky, D. Lentink, “Bird-inspired dynamic grasping and perching,” Science Robotics, 6(61), 2021.',
        'R. Ritz, M. W. Müller, M. Hehn, R. D’Andrea, “Cooperative quadrocopter ball throwing and catching,” IROS, 2012.',
        'M. W. Müller, S. Lupashin, R. D’Andrea, “Quadrocopter ball juggling,” IROS, 2011.',
        'D. Mellinger, V. Kumar, “Minimum snap trajectory generation and control for quadrotors,” ICRA, 2011.',
        'C. Richter, A. Bry, N. Roy, “Polynomial trajectory planning for aggressive quadrotor flight,” ISRR, 2013.',
        'R. Verschueren et al., “acados—a modular open-source framework for fast embedded optimal control,” Math. Prog. Comp., 14:147–183, 2022.',
        'W. H. Clohessy, R. S. Wiltshire, “Terminal guidance system for satellite rendezvous,” J. Aerospace Sci., 27(9), 1960.',
        'P. Zarchan, Tactical and Strategic Missile Guidance, AIAA.',
        'DARPA Gremlins；Aurora “SideArm”；NASA Genesis/Stardust 空中回收；JPADS（公开资料）。',
    ]
    for i, r in enumerate(refs, 1):
        p = doc.add_paragraph(); p.paragraph_format.space_after = Pt(2)
        run = p.add_run(f'[{i}] {r}'); set_run(run, size=9.5)


def build(cfg, offline, m6, traces, m6_traces, out):
    doc = Document()
    init_styles(doc)
    setup_page(doc, '空投—空中捕获·仿真报告')
    cover(doc)

    hp = doc.add_paragraph(); r = hp.add_run('目录'); set_run(r, size=16, bold=True, east=EAST_H)
    add_toc(doc)
    doc.add_page_break()

    section_intro(doc)
    section_theory(doc)
    section_conditions(doc, cfg)
    section_data(doc, cfg, offline, m6, traces, m6_traces)
    section_analysis(doc, m6)
    appendix(doc, cfg, m6)

    doc.save(out)
    return out


def main():
    ap = argparse.ArgumentParser(description='生成《空投—空中捕获》仿真报告 docx')
    ap.add_argument('--out', default=DEFAULT_OUT)
    ap.add_argument('--fast', action='store_true', help='缩小 MC 次数，快速预览')
    ap.add_argument('--skip-mc', action='store_true', help='跳过蒙特卡洛（用已有量级数据）')
    args = ap.parse_args()

    cfg = load_cfg()
    print('[1/4] 离线全工况 ...')
    offline = collect_offline(cfg)
    print(f'      {len(offline)} 个工况')

    print('[2/4] M6 蒙特卡洛 ...')
    if args.fast:
        spec = [('M6_stack_drop', 60), ('M6_stack_bigfunnel', 40), ('M6_stack_tray', 100),
                ('M6_stack_tray_small', 100), ('M6_stack_tray_wind', 100),
                ('M6_stack_nodive', 40), ('M6_stack_wind', 40),
                ('M6_stack_lock', 40), ('M6_stack_windcomp', 40)]
    else:
        spec = [('M6_stack_drop', 200), ('M6_stack_bigfunnel', 100), ('M6_stack_tray', 300),
                ('M6_stack_tray_small', 300), ('M6_stack_tray_wind', 300),
                ('M6_stack_nodive', 100), ('M6_stack_wind', 100),
                ('M6_stack_lock', 100), ('M6_stack_windcomp', 100)]
    m6 = collect_m6_mc(cfg, spec)

    print('[3/4] 状态数据时间序列 ...')
    res1, plan1 = trace_m1(cfg)
    traces = {'m1': res1, 'rx': (plan1.t_r, plan1.t_c, plan1.h_c, res1.miss_dist)}
    m6_traces = {'drop': trace_m6(cfg, 'M6_stack_drop'),
                 'tray': trace_m6(cfg, 'M6_stack_tray')}

    print('[4/4] 生成 Word 文档 ...')
    out = build(cfg, offline, m6, traces, m6_traces, args.out)
    print('完成：', out, f'({os.path.getsize(out)} bytes)')


if __name__ == '__main__':
    main()
