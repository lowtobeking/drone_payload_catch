#!/usr/bin/env python3
"""全仓语法编译守卫：所有 `.py` 必须能编译（不 import，故无需任何依赖）。

    python3 tools/test_compileall.py

比"逐模块 import"更广（覆盖 ROS 节点、launch、全部 tools），且不受依赖缺失
影响——用来挡住低级语法错误进入主分支 / CI。
"""
import os
import py_compile
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SKIP_PARTS = {'__pycache__', '.git', 'build', 'install', 'log'}

files = [p for p in ROOT.rglob('*.py')
         if not any(part in SKIP_PARTS for part in p.parts)]

FAIL = []
print(f'=== 编译 {len(files)} 个 .py 文件 ===')
for p in files:
    try:
        py_compile.compile(str(p), doraise=True)
    except py_compile.PyCompileError as e:
        FAIL.append((p.relative_to(ROOT), str(e)))

if FAIL:
    for rel, err in FAIL:
        print(f'  ❌ {rel}\n     {err.splitlines()[-1] if err else ""}')
    print(f'\n❌ {len(FAIL)} 个文件编译失败')
    sys.exit(1)

print(f'✅ test_compileall 全部通过（{len(files)} 个文件）')
