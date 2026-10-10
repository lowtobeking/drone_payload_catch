#!/usr/bin/env bash
# tools/sitl_check.sh — M6 SITL 端到端验收（headless）。
#
# 委托已验证的 run_m6_sitl.sh 跑一遍，再按关键事件给出"通过/失败"判定与退出码，
# 补上"离线有 run_checks.sh、SITL 只能靠人肉 grep"的缺口。
#
# 需要完整 SITL 环境（`source ~/drone_payload_catch/env.sh`：PX4-1.16 + Gazebo + ROS）。
# 耗时数分钟，**不进 CI**（CI 只跑纯 Python 的 run_checks.sh --quick）。
#
#   source ~/drone_payload_catch/env.sh
#   bash tools/sitl_check.sh 70                       # 默认 M6 定点
#   FORMATION_VEL="0.5,0.0,0.0" bash tools/sitl_check.sh 70   # M6-moving
#   MODE=full bash tools/sitl_check.sh 70             # 研究特性全开
#
# 判定：出现 `STACK CAPTURED` 且无 `Failsafe activated` ⇒ 通过（退出码 0）。
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_S="${1:-70}"
OUT="${SITL_CHECK_OUT:-$HOME/payload_catch_m6/sitl_check.log}"
LAUNCH="$HOME/payload_catch_m6/launch.log"
mkdir -p "$(dirname "$OUT")"

echo "### run_m6_sitl.sh $RUN_S（输出 → $OUT）"
bash "$ROOT/run_m6_sitl.sh" "$RUN_S" > "$OUT" 2>&1 || true

cap=$( { grep -ac 'STACK CAPTURED' "$OUT" 2>/dev/null || true; } )
cap2=$( { grep -ac 'STACK CAPTURED' "$LAUNCH" 2>/dev/null || true; } )
fs=$( { grep -acE 'Failsafe activated|Flight termination active' "$OUT" 2>/dev/null || true; } )
fs2=$( { grep -acE 'Failsafe activated|Flight termination active' "$LAUNCH" 2>/dev/null || true; } )
captured=$(( ${cap:-0} + ${cap2:-0} ))
failsafe=$(( ${fs:-0} + ${fs2:-0} ))

echo "---------------- 判定 ----------------"
echo "  STACK CAPTURED 命中 : $captured"
echo "  Failsafe 命中       : $failsafe"
if [[ "$captured" -ge 1 && "$failsafe" -eq 0 ]]; then
  echo "✅ SITL-CHECK 通过"
  exit 0
fi
echo "❌ SITL-CHECK 失败（详见 $OUT 与 $LAUNCH）"
exit 1
