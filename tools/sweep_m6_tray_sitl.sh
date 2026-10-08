#!/usr/bin/env bash
# sweep_m6_tray_sitl.sh — 圆形托盘（真机末端）SITL 难度扫描。
#   FUNNEL_TYPE=tray（x500_tray + payload_100g）；逐档加压，结果写 report/m6_tray_sitl_results.md。
#
# 用法（建议后台）：
#   cd ~/drone_payload_catch
#   RUN_S=70 setsid bash tools/sweep_m6_tray_sitl.sh > /tmp/m6_tray_sweep.out 2>&1 &
set +u
RUN_S="${RUN_S:-70}"
REPO="$HOME/drone_payload_catch"
OUT="${OUT:-/tmp/m6_tray_sweep_results.txt}"
REPORT="$REPO/report/m6_tray_sitl_results.md"
: > "$OUT"

# name | A_HOVER | B_STANDBY | LAUNCH_EXTRA
trials=(
  "TR0_nominal|0.0,0.0,-4.5|0.0,0.0,-3.5|"
  "TR1_err0.10_track|0.0,0.0,-4.5|0.0,0.0,-3.5|release_xy_sigma:=0.10"
  "TR2_relnav_heavy_lpf|0.0,0.0,-4.5|0.0,0.0,-3.5|rel_pos_sigma:=0.15 rel_latency:=0.15 rel_jitter:=0.05 rel_dropout:=0.30 rel_bias:=0.05 payload_meas_sigma:=0.05 payload_meas_latency:=0.08"
  "TR3_relnav_heavy_nolpf|0.0,0.0,-4.5|0.0,0.0,-3.5|rel_pos_sigma:=0.15 rel_latency:=0.15 rel_jitter:=0.05 rel_dropout:=0.30 rel_bias:=0.05 payload_meas_sigma:=0.05 payload_meas_latency:=0.08 est_lpf_alpha:=1.0"
  "TR4_gap1.2|0.0,0.0,-4.7|0.0,0.0,-3.5|"
  "TR5_dive5|0.0,0.0,-4.5|0.0,0.0,-3.5|a_dive:=5.0"
  "TR6_err0.20_track|0.0,0.0,-4.5|0.0,0.0,-3.5|release_xy_sigma:=0.20"
  "TR7_err0.20_notrack|0.0,0.0,-4.5|0.0,0.0,-3.5|release_xy_sigma:=0.20 track_payload:=false"
)

{
echo "# 圆形托盘（真机末端）SITL 难度扫描结果"
echo ""
echo "模型：\`FUNNEL_TYPE=tray\`（x500_tray + payload_100g）；盘内径30cm/围边5cm/泡棉e=0.15；"
echo "eff_r=0.12m，v_retain=6.60 m/s（a_dive 自动=0，悬停接）。"
echo "每档重启 gz + 2×PX4，运行 ${RUN_S}s。"
echo ""
printf '| 试验 | 捕获 | 水平误差 | 接触相对速度 | 对正 rel_xy | min A-B | 双机落地 |\n'
printf '|---|---|---|---|---|---|---|\n'
} > "$OUT"

for t in "${trials[@]}"; do
  IFS='|' read -r name az bz extra <<< "$t"
  echo "===== $name  A=$az B=$bz extra=[$extra] =====" >> "$OUT"
  ready=0
  for attempt in 1 2; do
    sleep 6
    FUNNEL_TYPE=tray A_HOVER="$az" B_STANDBY="$bz" LAUNCH_EXTRA="$extra" \
      bash "$REPO/run_m6_sitl.sh" "$RUN_S" > "/tmp/m6tray_${name}.log" 2>&1
    grep -qa "双机 READY" "/tmp/m6tray_${name}.log" && { ready=1; break; }
    echo "  (attempt $attempt: 双机未 READY，重试)" >> "$OUT"
  done
  L="$HOME/payload_catch_m6/launch.log"
  cap=$(grep -ac "STACK CAPTURED" "$L" 2>/dev/null)
  hz=$(grep -aoE "horiz=[0-9.]+m" "$L" 2>/dev/null | tail -1)
  rv=$(grep -aoE "rel_v=[0-9.]+m/s" "$L" 2>/dev/null | tail -1)
  al=$(grep -aoE "ALIGNED rel_xy=[0-9.]+m" "$L" 2>/dev/null | tail -1 | sed 's/ALIGNED //')
  mr=$(grep -aoE "min_relA=[0-9.]+" "$L" 2>/dev/null | sed 's/min_relA=//' | sort -n | head -1)
  l0=$(grep -ac "Landing detected" "$HOME/px4_logs/px4_0.log" 2>/dev/null)
  l1=$(grep -ac "Landing detected" "$HOME/px4_logs/px4_1.log" 2>/dev/null)
  ok=$([ "${cap:-0}" -ge 1 ] 2>/dev/null && echo "✅" || echo "❌")
  lls=$([ "${l0:-0}" -ge 1 ] 2>/dev/null && [ "${l1:-0}" -ge 1 ] 2>/dev/null && echo "✅✅" || echo "—")
  [ "$ready" -eq 0 ] && ok="❌(env)"
  printf '| %s | %s | %s | %s | %s | %s m | %s |\n' \
    "$name" "$ok" "${hz:-—}" "${rv:-—}" "${al:-—}" "${mr:-—}" "$lls" >> "$OUT"
  echo "  captured=$cap $hz $rv $al min_relA=${mr} land0=${l0} land1=${l1} ready=$ready" >> "$OUT"
done
echo "SWEEP-DONE" >> "$OUT"
cp "$OUT" "$REPORT"
echo "tray sweep done -> $REPORT"
