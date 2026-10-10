#!/usr/bin/env bash
# tools/run_checks.sh — 一键离线自检（参考 drone_package_20260908 的 *_check.sh 风格）
#
#   bash tools/run_checks.sh            # 全量：纯模块自测 + tools/test_*.py + acados + offline --all
#   bash tools/run_checks.sh --quick    # 快速：只跑纯模块自测 + tools/test_*.py（秒级，pre-commit）
#
# 退出码：0=全通过，1=有失败（失败项列在末尾）。改代码后必跑。
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 1
QUICK=0
[[ "${1:-}" == "--quick" ]] && QUICK=1

# ── 环境（acados/ROS）仅当存在时 source，且吞掉它的 echo ──
if [[ -f "$ROOT/env.sh" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT/env.sh" >/dev/null 2>&1 || true
fi

PASS=0
FAIL=0
FAILED=()

run() {
  local name="$1"; shift
  echo "───────────────────────────────────────────────"
  echo "▶ $name"
  if "$@"; then
    echo "✅ $name"
    PASS=$((PASS + 1))
  else
    echo "❌ $name"
    FAIL=$((FAIL + 1))
    FAILED+=("$name")
  fi
}

echo "==============================================="
echo "离线自检  ROOT=$ROOT  $([[ $QUICK -eq 1 ]] && echo '(quick)' || echo '(full)')"
echo "==============================================="

# 1) 纯算法模块自测（不依赖 ROS，秒级）
for m in payload_model rendezvous sim_core payload_filter stack_drop relnav \
         contact_detect uncertainty dynamics impact perception stats safety_logic; do
  run "module:$m" python3 -m "payload_catch.$m"
done

# 2) tools/test_*.py（纯逻辑单元测试）
shopt -s nullglob
for t in tools/test_*.py; do
  run "test:$(basename "$t")" python3 "$t"
done

# 3) acados 链 + 离线集成体检 + pytest（full）
if [[ "$QUICK" -eq 0 ]]; then
  run "module:mpc_terminal" python3 -m payload_catch.mpc_terminal
  run "smoke:acados" python3 tools/smoke_acados.py
  run "offline:--all" python3 tools/offline_run.py --all
  if python3 -c 'import pytest' >/dev/null 2>&1; then
    run "pytest" python3 -m pytest -q
  fi
fi

echo "==============================================="
echo "自检汇总：PASS=$PASS  FAIL=$FAIL"
if [[ "$FAIL" -gt 0 ]]; then
  printf '失败项：%s\n' "${FAILED[*]}"
  echo "❌ 有失败，请修复后重跑"
  exit 1
fi
echo "✅ 全部通过"
