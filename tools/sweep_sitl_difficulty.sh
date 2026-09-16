#!/bin/bash
# sweep_difficulty.sh — 逐步加压：B 偏移 / A 高度 递增，记录是否捕获
# 后台运行: setsid bash /tmp/sweep_difficulty.sh &  结果写 /tmp/sweep_results.txt
# ⚠️ 所有向量字符串必须全 float（launch 要求序列元素类型一致）
OUT=/tmp/sweep_results.txt
: > "$OUT"
RUN_S="${RUN_S:-45}"
levels=(
  "L0a_gentle_off0.2_A3.0|0.2|0.0,0.0,-3.0"
  "L0b_gentle_off0.2_A3.0|0.2|0.0,0.0,-3.0"
  "L1_off0.5_A3.0|0.5|0.0,0.0,-3.0"
  "L2_off0.8_A3.0|0.8|0.0,0.0,-3.0"
  "L3_off0.2_A4.0|0.2|0.0,0.0,-4.0"
  "L4_off0.5_A4.0|0.5|0.0,0.0,-4.0"
)
for lv in "${levels[@]}"; do
  IFS='|' read -r name off az <<< "$lv"
  echo "===== $name  (B_off=$off A_z=$az) =====" >> "$OUT"
  A_HOVER="$az" B_STANDBY="$off,0.0,-2.8" B_OFFSET="$off,0.0,0.0" \
    B_POSE_ENU="0,$off,0,0,0,0" CTRL=mpc \
    bash "$HOME/drone_payload_catch/run_m1_sitl.sh" "$RUN_S" \
    > "/tmp/sweep_${name}.log" 2>&1
  L="$HOME/payload_catch_sitl/launch.log"
  cap=$(grep -ac "CAPTURED" "$L" 2>/dev/null)
  capd=$(grep -aoE "\*\*\* CAPTURED \*\*\* d=[0-9.]+m rel_v=[0-9.]+m/s" "$L" 2>/dev/null | tail -1)
  plan=$(grep -aE "PLAN ok|PLAN infeasible" "$L" 2>/dev/null | tail -1 | sed 's/.*b_node\]: //')
  safe=$(grep -ac "Failsafe activated" "$HOME/px4_logs/px4_1.log" 2>/dev/null)
  echo "  captured=$cap  $capd" >> "$OUT"
  echo "  plan: $plan   px4_1_failsafe=$safe" >> "$OUT"
done
echo "SWEEP-DONE" >> "$OUT"
