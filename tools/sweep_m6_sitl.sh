#!/usr/bin/env bash
# sweep_m6_sitl.sh — M6 垂直堆叠投放的 SITL 难度扫描（每档重启 gz+2×PX4，逐档加压）。
#
# 用法（后台，结果写 report/m6_sitl_results.md）：
#   cd ~/drone_payload_catch
#   setsid bash tools/sweep_m6_sitl.sh > /tmp/m6_sweep.out 2>&1 &
#
# 每档变量：A_HOVER / B_STANDBY / LAUNCH_EXTRA（透传给 catch_stack_launch.py）。
set +u
RUN_S="${RUN_S:-95}"
REPO="$HOME/drone_payload_catch"
OUT="${OUT:-/tmp/m6_sweep_results.txt}"
REPORT="$REPO/report/m6_sitl_results.md"
: > "$OUT"

# name | A_HOVER | B_STANDBY | LAUNCH_EXTRA
trials=(
  "T0_nominal|0.0,0.0,-4.5|0.0,0.0,-3.5|"
  "T1_err0.10_track|0.0,0.0,-4.5|0.0,0.0,-3.5|release_xy_sigma:=0.10"
  "T2_err0.10_notrack|0.0,0.0,-4.5|0.0,0.0,-3.5|release_xy_sigma:=0.10 track_payload:=false"
  "T3_relnav_heavy_lpf|0.0,0.0,-4.5|0.0,0.0,-3.5|rel_pos_sigma:=0.15 rel_latency:=0.15 rel_jitter:=0.05 rel_dropout:=0.30 rel_bias:=0.05 payload_meas_sigma:=0.05 payload_meas_latency:=0.08"
  "T3n_relnav_heavy_nolpf|0.0,0.0,-4.5|0.0,0.0,-3.5|rel_pos_sigma:=0.15 rel_latency:=0.15 rel_jitter:=0.05 rel_dropout:=0.30 rel_bias:=0.05 payload_meas_sigma:=0.05 payload_meas_latency:=0.08 est_lpf_alpha:=1.0"
  "T4_lead0.45|0.0,0.0,-4.5|0.0,0.0,-3.5|release_lead:=0.45"
  "T5_gap1.2|0.0,0.0,-4.7|0.0,0.0,-3.5|"
  "T6_dive5|0.0,0.0,-4.5|0.0,0.0,-3.5|a_dive:=5.0"
  "T7_err0.20_track|0.0,0.0,-4.5|0.0,0.0,-3.5|release_xy_sigma:=0.20"
  "T8_err0.20_notrack|0.0,0.0,-4.5|0.0,0.0,-3.5|release_xy_sigma:=0.20 track_payload:=false"
)

echo "# M6 SITL 难度扫描结果" > "$OUT"
echo "" >> "$OUT"
printf '| 试验 | 捕获 | 水平误差 | 接触相对速度 | 对正 rel_xy | min A-B | A落地 | B落地 |\n' >> "$OUT"
printf '|---|---|---|---|---|---|---|---|\n' >> "$OUT"

for t in "${trials[@]}"; do
  IFS='|' read -r name az bz extra <<< "$t"
  echo "===== $name  A=$az B=$bz extra=[$extra] =====" >> "$OUT"
  ok_ready=0
  for attempt in 1 2; do
    sleep 6
    A_HOVER="$az" B_STANDBY="$bz" LAUNCH_EXTRA="$extra" \
      bash "$REPO/run_m6_sitl.sh" "$RUN_S" > "/tmp/m6sweep_${name}.log" 2>&1
    if grep -qa "双机 READY" "/tmp/m6sweep_${name}.log"; then ok_ready=1; break; fi
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
  ok=$([ "$cap" -ge 1 ] 2>/dev/null && echo "✅" || echo "❌")
  l0s=$([ "${l0:-0}" -ge 1 ] 2>/dev/null && echo "✅" || echo "—")
  l1s=$([ "${l1:-0}" -ge 1 ] 2>/dev/null && echo "✅" || echo "—")
  [ "$ok_ready" -eq 0 ] && ok="❌(env)"
  printf '| %s | %s | %s | %s | %s | %s m | %s | %s |\n' \
    "$name" "$ok" "${hz:-—}" "${rv:-—}" "${al:-—}" "${mr:-—}" "$l0s" "$l1s" >> "$OUT"
  echo "  captured=$cap $hz $rv $al min_relA=${mr} landA=${l0} landB=${l1} ready=$ok_ready" >> "$OUT"
done
echo "SWEEP-DONE" >> "$OUT"
cp "$OUT" "$REPORT"
echo "m6 sweep done -> $REPORT"
