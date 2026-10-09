#!/usr/bin/env bash
# mc_m6_tray_sitl.sh — 圆形托盘（真机末端）SITL 蒙特卡洛：每配置重复 N 次换种子。
#   FUNNEL_TYPE=tray（x500_tray + payload_100g）；结果写 report/m6_tray_sitl_mc.md。
#
# 用法（建议后台）：
#   cd ~/drone_payload_catch
#   RUN_S=70 NREP=3 setsid bash tools/mc_m6_tray_sitl.sh > /tmp/m6_tray_mc.out 2>&1 &
set +u
RUN_S="${RUN_S:-70}"
NREP="${NREP:-3}"
REPO="$HOME/drone_payload_catch"
OUT="${OUT:-/tmp/m6_tray_mc_results.txt}"
REPORT="$REPO/report/m6_tray_sitl_mc.md"
: > "$OUT"

# name | LAUNCH_EXTRA（种子由 NREP 循环注入）
configs=(
  "C0_nominal|"
  "C1_relnav_heavy_lpf|rel_pos_sigma:=0.15 rel_latency:=0.15 rel_jitter:=0.05 rel_dropout:=0.30 rel_bias:=0.05 payload_meas_sigma:=0.05 payload_meas_latency:=0.08"
  "C2_err0.20_track|release_xy_sigma:=0.20"
)

{
echo "# 圆形托盘（真机末端）SITL 蒙特卡洛结果（每档 N=$NREP，逐次换种子）"
echo ""
echo "模型：\`FUNNEL_TYPE=tray\`（x500_tray + payload_100g）；eff_r=0.12m，v_retain=6.60 m/s。"
echo ""
printf '| 配置 | 成功率 | 水平误差(成功均值) | 接触速度(成功均值) | min A-B 最低 |\n'
printf '|---|---|---|---|---|\n'
} > "$OUT"

for c in "${configs[@]}"; do
  IFS='|' read -r name extra <<< "$c"
  ok=0; hz_all=""; rv_all=""; mr_all=""
  for rep in $(seq 1 "$NREP"); do
    ex="$extra rel_seed:=$rep release_seed:=$rep"
    lr="/tmp/m6traymc_${name}_${rep}.log"
    ready=0
    for attempt in 1 2; do
      sleep 6
      FUNNEL_TYPE=tray A_HOVER="0.0,0.0,-4.5" B_STANDBY="0.0,0.0,-3.5" LAUNCH_EXTRA="$ex" \
        bash "$REPO/run_m6_sitl.sh" "$RUN_S" > "$lr" 2>&1
      grep -qa "双机 READY" "$lr" && { ready=1; break; }
    done
    L="$HOME/payload_catch_m6/launch.log"
    cap=$(grep -ac "STACK CAPTURED" "$L" 2>/dev/null)
    hz=$(grep -aoE "horiz=[0-9.]+m" "$L" 2>/dev/null | tail -1 | sed 's/horiz=//;s/m//')
    rv=$(grep -aoE "rel_v=[0-9.]+m/s" "$L" 2>/dev/null | tail -1 | sed 's/rel_v=//;s#m/s##')
    mr=$(grep -aoE "min_relA=[0-9.]+" "$L" 2>/dev/null | sed 's/min_relA=//' | sort -n | head -1)
    if [ "${cap:-0}" -ge 1 ] && [ "$ready" -eq 1 ]; then
      ok=$((ok+1)); hz_all="$hz_all $hz"; rv_all="$rv_all $rv"
    fi
    [ -n "$mr" ] && mr_all="$mr_all $mr"
    echo "  rep$rep: captured=$cap ready=$ready hz=$hz rel_v=$rv min_relA=$mr" >> "$OUT"
  done
  hzm=$(python3 -c "import statistics as s; v=[float(x) for x in '$hz_all'.split()]; print(f'{s.mean(v):.3f}' if v else '—')" 2>/dev/null)
  rvm=$(python3 -c "import statistics as s; v=[float(x) for x in '$rv_all'.split()]; print(f'{s.mean(v):.3f}' if v else '—')" 2>/dev/null)
  mrm=$(python3 -c "import statistics as s; v=[float(x) for x in '$mr_all'.split()]; print(f'{min(v):.3f}' if v else '—')" 2>/dev/null)
  ci=$(cd "$REPO" 2>/dev/null && python3 -c "from payload_catch.stats import wilson_ci as w; p,lo,hi=w($ok,$NREP); print(f'{100*p:.0f}% [{100*lo:.0f},{100*hi:.0f}]')" 2>/dev/null)
  printf '| %s | %s/%s %s | %s m | %s m/s | %s m |\n' "$name" "$ok" "$NREP" "${ci:-}" "$hzm" "$rvm" "$mrm" >> "$OUT"
  echo "== $name: $ok/$NREP mean_hz=$hzm mean_rv=$rvm min_minrelA=$mrm" >> "$OUT"
done
echo "MC-DONE" >> "$OUT"
cp "$OUT" "$REPORT"
echo "tray mc done -> $REPORT"
