# 开发者快捷入口（等价于 tools/run_checks.sh）
.PHONY: help check check-quick test smoke offline

help:
	@echo "make check        # 全量离线自检（+acados +offline --all +pytest）"
	@echo "make check-quick  # 秒级离线自检（纯模块 + tools/test_*.py）"
	@echo "make test         # pytest -q"
	@echo "make smoke        # acados MPC 链冒烟（需 source env.sh）"
	@echo "make offline      # tools/offline_run.py --all"

check:
	bash tools/run_checks.sh

check-quick:
	bash tools/run_checks.sh --quick

test:
	python3 -m pytest -q

smoke:
	python3 tools/smoke_acados.py

offline:
	python3 tools/offline_run.py --all
