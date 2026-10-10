"""pytest 入口：把 `tools/test_*.py` 离线自检收进标准测试框架。

这样仓库声明的 `pytest` / `colcon test` 真正有东西可跑（此前是空缺），
同时不改变参考工程"独立 `tools/test_*.py`、可单独运行"的风格。

    pytest -q
"""
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = sorted((ROOT / 'tools').glob('test_*.py'))


@pytest.mark.parametrize('script', SCRIPTS, ids=lambda p: p.name)
def test_offline_script(script):
    """每个 tools/test_*.py 必须能在纯 Python 环境下全绿退出。"""
    result = subprocess.run([sys.executable, str(script)], cwd=str(ROOT),
                            capture_output=True, text=True)
    assert result.returncode == 0, (
        f'{script.name} 退出码 {result.returncode}\n'
        f'--- stdout ---\n{result.stdout}\n--- stderr ---\n{result.stderr}')


def test_scripts_discovered():
    """防止 glob 失效（脚本被改名/移走后测试却"全绿"）。"""
    assert SCRIPTS, 'tools/ 下未发现任何 test_*.py'
