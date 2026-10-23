from pathlib import Path
import json
import shutil
files=[str(p) for p in Path("/opt/ros/humble/lib").glob("*quat*")]
files +=[str(p) for p in Path("/opt/ros/humble/lib").glob("*vrpn*")]
print(json.dumps(dict(gpp=shutil.which("g++"),libraries=files),indent=2))
