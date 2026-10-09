#!/usr/bin/env bash
# 相对不确定度 + C1 释放证书。用法: bash scripts/check_release_cert.sh [eps]
set +u
REPO="${DRONE_PAYLOAD_CATCH:-$HOME/drone_payload_catch}"; cd "$REPO" || exit 1
echo "== 相对不确定度模型自测 =="; python3 -m payload_catch.uncertainty
echo; echo "== 证书可行性 / 释放阈值（写 report/rel_uncertainty.md）=="
exec python3 tools/rel_sigma.py "$@"
