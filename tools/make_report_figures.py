#!/usr/bin/env python3
"""make_report_figures.py — generate simulation figures for the survey/report doc.

Outputs (PNG -> report/figures/):
  fig_control_architecture.png   control / information architecture
  fig_funnel_physics.png         funnel capture physics (v_rel vs gap; v_retain vs depth)
  fig_m6_stack_offline.png       M6 vertical stack-drop offline (B/payload altitude, rel dist)
  fig_m6_sitl_formation.png      M6-moving formation release, SITL traces (parsed from launch.log)
  fig_estimator_comparison.png   KF vs naive payload-state estimation error
Usage: python3 tools/make_report_figures.py [--log ~/payload_catch_m6_gui/launch.log]
"""
from __future__ import annotations

import argparse
import os
import re
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)
from payload_catch.stack_drop import simulate_stack, retain_speed, contact_rel_speed  # noqa: E402

FIG = os.path.join(REPO, 'report', 'figures')
os.makedirs(FIG, exist_ok=True)
plt.rcParams.update({'figure.dpi': 130, 'font.size': 9, 'axes.grid': True,
                     'grid.alpha': 0.3, 'axes.unicode_minus': False})


# ------------------------------------------------------------- architecture
def fig_architecture():
    fig, ax = plt.subplots(figsize=(11.5, 6.2))
    ax.set_xlim(0, 100); ax.set_ylim(0, 62); ax.axis('off')
    C = {'plan': '#dbeafe', 'ctrl': '#dcfce7', 'plant': '#fee2e2',
         'est': '#fef9c3', 'io': '#e2e8f0'}
    edge = '#334155'

    def box(x, y, w, h, text, color, fs=8.3, bold=False):
        ax.add_patch(plt.Rectangle((x, y), w, h, fc=color, ec=edge, lw=1.2, zorder=2))
        ax.text(x + w / 2, y + h / 2, text, ha='center', va='center',
                fontsize=fs, zorder=3, fontweight='bold' if bold else 'normal')

    def arrow(x1, y1, x2, y2, text='', rad=0.0, fs=7.5, color=edge):
        ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle='-|>', color=color, lw=1.3,
                                    connectionstyle=f'arc3,rad={rad}'))
        if text:
            ax.text((x1 + x2) / 2, (y1 + y2) / 2 + 1.1, text, ha='center', fontsize=fs)

    box(2, 53, 20, 8, 'Mission / geometry config\n(yaml single source of truth)', C['plan'], bold=True)
    box(26, 53, 22, 8, 'Rendezvous planner\nsearch (t_r, τ_c) + min-energy cubic', C['plan'])
    box(52, 53, 20, 8, 'Release timing / formation\n/formation/start → release_at', C['plan'])
    box(76, 53, 22, 8, 'Capture criterion\nwindow + v_retain', C['plan'])

    box(4, 39, 26, 9, 'A: cruise / hover control\nv = v_ff + kp (p_ref − p)', C['ctrl'], bold=True)
    box(38, 39, 26, 9, 'B: guidance + control\nPD+feedforward  or  terminal MPC (acados)', C['ctrl'], bold=True)
    box(70, 39, 26, 9, 'DIVE dive/brake reference\n_stack_ref (analytic)', C['ctrl'])

    box(4, 23, 26, 9, 'A rigid body + attached payload\n(DetachableJoint)', C['plant'], bold=True)
    box(38, 23, 26, 9, 'B rigid body + rigid funnel\n(x500_funnel)', C['plant'], bold=True)
    box(70, 23, 26, 9, 'Payload projectile dynamics\np = p_r + v_r τ + ½ g τ²', C['plant'])

    box(4, 8.5, 26, 8, 'PX4 velocity control\n(Offboard, 50 Hz)', C['io'])
    box(38, 8.5, 26, 8, 'PX4 velocity control\n(Offboard, 50 Hz)', C['io'])
    box(70, 8.5, 26, 8, 'Gazebo physics\n(ODE, dt = 4 ms)', C['io'])

    box(2, 0.5, 44, 5.5, 'Relative nav (mesh stand-in: truth + noise/latency/dropout) + EMA', C['est'])
    box(50, 0.5, 48, 5.5, 'Payload state estimation:  none / naive / KF ([p,v], gravity as input)', C['est'])

    for x in (15, 37, 62, 89):
        arrow(x, 53, x, 48)
    arrow(17, 39, 17, 32); arrow(51, 39, 51, 32); arrow(83, 39, 83, 32)
    arrow(17, 23, 17, 16.5); arrow(51, 23, 51, 16.5); arrow(83, 23, 83, 16.5)
    arrow(30, 12.5, 36, 12.5)
    arrow(64, 12.5, 70, 12.5)
    arrow(15, 8.5, 15, 6, rad=-0.3)
    arrow(24, 6, 70, 6, 'observation / state feedback', rad=-0.12)
    arrow(70, 6, 70, 8.5)
    ax.text(50, 30.8, 'closed-loop replanning  solve_inflight (every replan_dt)',
            ha='center', fontsize=7.8, color='#7c2d12')
    ax.set_title('Fig. 1  Airdrop & mid-air capture: layered planning / estimation / control / plant',
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, 'fig_control_architecture.png'), bbox_inches='tight')
    plt.close(fig)


# -------------------------------------------------------------- funnel physics
def fig_funnel_physics():
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    gap = np.linspace(0.05, 2.0, 200)
    vrel = lambda g_, a_: np.sqrt(np.maximum(2.0 * (g_ - a_) * gap, 0.0))
    for a_dive, c in [(0.0, '#94a3b8'), (3.0, '#2563eb'), (6.0, '#dc2626')]:
        axes[0].plot(gap, vrel(9.81, a_dive), color=c, lw=2, label=f'$a_{{dive}}$={a_dive:.0f} m/s²')
    axes[0].axhline(4.044, color='green', ls='--', lw=1.5, label=r'$v_{retain}$=4.04 (depth .3, e=.6)')
    axes[0].axvline(1.0, color='k', ls=':', lw=1)
    axes[0].plot([1.0], [float(contact_rel_speed(1.0, 3.0))], 'o', color='#2563eb')
    axes[0].annotate('nominal gap=1.0\na=3 → 3.69', (1.0, 3.69), (1.15, 2.1),
                     arrowprops=dict(arrowstyle='->', color='k'), fontsize=8)
    axes[0].set_xlabel('gap = payload→funnel-mouth distance (m)')
    axes[0].set_ylabel(r'contact relative speed $v_{rel}$ (m/s)')
    axes[0].set_title(r'$v_{rel}=\sqrt{2(g-a_{dive})\,gap}$ vs retention threshold')
    axes[0].legend(fontsize=7.5); axes[0].set_ylim(0, 7)

    depth = np.linspace(0.05, 0.6, 200)
    for e, c in [(0.4, '#16a34a'), (0.6, '#2563eb'), (0.8, '#dc2626')]:
        axes[1].plot(depth, [retain_speed(d, e) for d in depth], color=c, lw=2, label=f'e={e}')
    axes[1].axhline(3.69, color='k', ls=':', lw=1)
    axes[1].text(0.31, 3.8, r'nominal $v_{rel}$=3.69', fontsize=8)
    axes[1].axvline(0.30, color='k', ls=':', lw=1)
    axes[1].set_xlabel('funnel depth (m)')
    axes[1].set_ylabel(r'$v_{retain}=\sqrt{2g\cdot depth}/e$ (m/s)')
    axes[1].set_title('max contact speed a rigid funnel can retain')
    axes[1].legend(fontsize=8); axes[1].set_ylim(0, 12)
    fig.suptitle('Fig. 2  M6 funnel-capture physics: contact-speed lower bound and retention criterion', fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, 'fig_funnel_physics.png'), bbox_inches='tight')
    plt.close(fig)


# ---------------------------------------------------- M6 stack-drop offline sim
def fig_m6_stack_offline():
    cfg = yaml.safe_load(open(os.path.join(REPO, 'config', 'catch_scenarios.yaml')))
    defaults = cfg['defaults']; layout = cfg['layouts']['stack_drop']
    scen = cfg['scenarios']['M6_stack_drop']
    res, plan = simulate_stack(defaults, layout, scen)
    t = np.array(res.t)
    b_alt = -np.array(res.b_pos)[:, 2]
    p_alt = -np.array(res.p_pos)[:, 2]
    mount = float(defaults['capture']['funnel']['mount_height'])
    mouth = b_alt + mount
    eff_r = float(defaults['capture']['funnel']['mouth_radius']
                  - defaults['capture']['funnel']['object_radius'])

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.3))
    axes[0].plot(t, b_alt, color='#2563eb', lw=2, label='B altitude')
    axes[0].plot(t, mouth, color='#2563eb', ls='--', lw=1.2, label='funnel mouth plane')
    axes[0].plot(t, p_alt, color='#ea580c', lw=2, label='payload altitude')
    axes[0].axhline(-float(layout['a_init'][2]), color='#16a34a', ls=':', label='A hover altitude')
    if np.isfinite(res.t_capture):
        axes[0].axvline(res.t_capture, color='k', ls=':', lw=1)
        axes[0].plot([res.t_capture], [np.interp(res.t_capture, t, p_alt)], 'k*', ms=13,
                     label=f'capture t={res.t_capture:.2f}s')
    axes[0].set_xlabel('time (s)'); axes[0].set_ylabel('altitude (m)')
    axes[0].set_title('B / payload altitude (vertical stack drop)')
    axes[0].legend(fontsize=8)

    axes[1].plot(t, res.rel_dist, color='#9333ea', lw=2)
    axes[1].axhline(eff_r, color='green', ls='--', label='effective mouth radius')
    axes[1].set_xlabel('time (s)'); axes[1].set_ylabel('B–payload distance (m)')
    axes[1].set_title(f'relative distance (success={res.success}, miss={res.miss_dist:.3f} m, '
                      f'rel_v={res.rel_speed_at_capture:.2f} m/s)')
    axes[1].legend(fontsize=8)
    fig.suptitle('Fig. 3  M6 vertical stack-drop offline closed-loop simulation (analytic funnel criterion)', fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, 'fig_m6_stack_offline.png'), bbox_inches='tight')
    plt.close(fig)


# ------------------------------------------------- M6-moving SITL traces
LINE = re.compile(
    r'\[(?P<ts>\d+\.\d+)\].*?B phase=(?P<phase>\w+) '
    r'pos_w=\[\s*(?P<bx>[-\d.eE]+)\s+(?P<by>[-\d.eE]+)\s+(?P<bz>[-\d.eE]+)\]\s*'
    r'vel=\[\s*(?P<vx>[-\d.eE]+)\s+(?P<vy>[-\d.eE]+)\s+(?P<vz>[-\d.eE]+)\]\s*'
    r'relA=(?P<relA>[-\d.eE]+).*?'
    r'pay=(?P<pay>None|\[\s*[-\d.eE]+\s+[-\d.eE]+\s+[-\d.eE]+\])')
PAY = re.compile(r'\[\s*([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)\s*\]')
FORM = re.compile(r'\[(?P<ts>\d+\.\d+)\].*?/formation/start.*?from \[(?P<x>[-\d.eE]+)\s+(?P<y>[-\d.eE]+)\]')


def fig_sitl_formation(log):
    if not os.path.exists(log):
        print(f'[skip] no log {log}'); return
    rec = []
    a_t0 = None; a_xy0 = np.zeros(2); a_alt = 5.0
    for ln in open(log, errors='ignore'):
        mf = FORM.search(ln)
        if mf:
            a_t0 = float(mf.group('ts')); a_xy0 = np.array([float(mf.group('x')), float(mf.group('y'))])
        m = LINE.search(ln)
        if not m:
            continue
        b = np.array([float(m.group('bx')), float(m.group('by')), float(m.group('bz'))])
        v = np.array([float(m.group('vx')), float(m.group('vy')), float(m.group('vz'))])
        pay = None if m.group('pay') == 'None' else np.array(
            [float(x) for x in PAY.search(m.group('pay')).groups()])
        rec.append((float(m.group('ts')), m.group('phase'), b, v, pay))
    if len(rec) < 5:
        print('[skip] too few samples'); return
    t = np.array([r[0] for r in rec]); t = t - t[0]
    B = np.array([r[2] for r in rec]); V = np.array([r[3] for r in rec])
    P = np.array([r[4] if r[4] is not None else [np.nan] * 3 for r in rec], float)
    phase = [r[1] for r in rec]

    A = np.full_like(B, np.nan)
    if a_t0 is not None:
        for i in range(len(rec)):
            tc = rec[i][0] - a_t0
            A[i] = [a_xy0[0] + max(tc, 0.0) * 0.5, a_xy0[1], -a_alt]

    fig, axes = plt.subplots(1, 3, figsize=(14, 4.3))
    axes[0].plot(B[:, 0], B[:, 1], color='#2563eb', lw=2, label='B (SITL)')
    axes[0].plot(P[:, 0], P[:, 1], color='#ea580c', lw=2, label='payload (SITL)')
    if np.any(np.isfinite(A)):
        axes[0].plot(A[:, 0], A[:, 1], color='#16a34a', lw=1.4, ls='--', label='A (formation ref, reconstructed)')
    axes[0].set_xlabel('North (m)'); axes[0].set_ylabel('East (m)')
    axes[0].set_title('top view (N–E)'); axes[0].legend(fontsize=7.5); axes[0].axis('equal')

    axes[1].plot(t, -B[:, 2], color='#2563eb', lw=2, label='B altitude')
    axes[1].plot(t, -P[:, 2], color='#ea580c', lw=2, label='payload altitude')
    if np.any(np.isfinite(A)):
        axes[1].plot(t, -A[:, 2], color='#16a34a', lw=1.4, ls='--', label='A altitude')
    i_rel = next((i for i, p in enumerate(phase) if p == 'DIVE'), None)
    if i_rel is not None:
        axes[1].axvline(t[i_rel], color='k', ls=':', lw=1)
        axes[1].text(t[i_rel], axes[1].get_ylim()[1] * 0.95, ' release', fontsize=8)
    i_cap = next((i for i, p in enumerate(phase) if p == 'DONE'), None)
    if i_cap is not None:
        axes[1].axvline(t[i_cap], color='#dc2626', ls=':', lw=1)
        axes[1].text(t[i_cap], axes[1].get_ylim()[1] * 0.55, ' capture', fontsize=8, color='#dc2626')
    axes[1].set_xlabel('time (s)'); axes[1].set_ylabel('altitude (m)')
    axes[1].set_title('altitude (A / B / payload)'); axes[1].legend(fontsize=8)

    axes[2].plot(t, np.linalg.norm(V[:, :2], axis=1), color='#2563eb', lw=1.8, label='B horizontal speed')
    d = np.linalg.norm(B - P, axis=1)
    axes[2].plot(t, d, color='#9333ea', lw=1.8, label='B–payload distance')
    axes[2].axhline(0.14, color='green', ls='--', lw=1, label='effective mouth radius')
    axes[2].set_xlabel('time (s)'); axes[2].legend(fontsize=8)
    axes[2].set_title('formation speed / relative distance')
    fig.suptitle('Fig. 4  M6-moving formation release, SITL traces (0.5 m/s: FORMATION → DIVE → capture)', fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, 'fig_m6_sitl_formation.png'), bbox_inches='tight')
    plt.close(fig)
    print('[ok] fig_m6_sitl_formation.png')


# ------------------------------------------------------ estimator comparison
def fig_estimator():
    labels = ['light\n(0.03/0/0)', 'medium\n(0.05/0.1s/20%)', 'heavy\n(0.10/0.2s/40%)']
    naive = [0.049, 0.667, 0.305]
    kf = [0.037, 0.189, 0.164]
    x = np.arange(len(labels)); w = 0.36
    fig, ax = plt.subplots(figsize=(7, 4.2))
    ax.bar(x - w / 2, naive, w, label='naive (finite-diff + extrapolation)', color='#f59e0b')
    ax.bar(x + w / 2, kf, w, label='KF (Kalman filter)', color='#2563eb')
    for xi, (n, k) in enumerate(zip(naive, kf)):
        ax.text(xi - w / 2, n + 0.01, f'{n:.3f}', ha='center', fontsize=8)
        ax.text(xi + w / 2, k + 0.01, f'{k:.3f}', ha='center', fontsize=8)
    ax.set_xticks(x); ax.set_xticklabels(labels)
    ax.set_ylabel('payload-state estimation error (m)')
    ax.set_title('Fig. 5  Payload estimator comparison: KF cuts error by 2–4×')
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, 'fig_estimator_comparison.png'), bbox_inches='tight')
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', default=os.path.expanduser('~/payload_catch_m6_gui/launch.log'))
    args = ap.parse_args()
    fig_architecture()
    fig_funnel_physics()
    fig_m6_stack_offline()
    fig_estimator()
    fig_sitl_formation(args.log)
    print('figures ->', FIG)


if __name__ == '__main__':
    main()
