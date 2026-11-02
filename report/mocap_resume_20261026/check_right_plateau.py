"""Offline check of right-turn acceptance at angle wrap and invalid plateaus."""
import importlib.util
from pathlib import Path
path=Path(__file__).resolve().parent.parent/"mocap_right_20261026/yaw_plateau.py"
spec=importlib.util.spec_from_file_location("right_plateau",path)
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
assert module.stable_right([-120.1,-119.9]*40,179.9)
assert module.stable_right([59.9,60.1]*40,0)
assert not module.stable_right([-60.1,-59.9]*40,0)
assert not module.stable_right([58.,62.]*40,0)
assert not module.stable_right([60.]*20,0)
assert not module.stable_right([float("nan")]*80,0)
print("6 right-turn plateau and angle-wrap checks passed")
