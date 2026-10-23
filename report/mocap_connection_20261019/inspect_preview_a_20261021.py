from pathlib import Path
import json
import subprocess

def run(argv):
    p = subprocess.run(argv, capture_output=True, text=True, timeout=12, shell=False)
    print(json.dumps({"argv": argv, "exit": p.returncode, "stdout": p.stdout, "stderr": p.stderr}, ensure_ascii=False))

for name in ["/proc/device-tree/serial-number", "/proc/sys/kernel/random/boot_id",
             "/opt/ros/humble/share/vrpn_mocap/launch/client.launch.yaml",
             "/opt/ros/humble/share/vrpn_mocap/config/client.yaml"]:
    p = Path(name)
    print(json.dumps({"path": name, "text": p.read_text(errors="replace").replace("\u0000", "") if p.exists() else None}))
print("CURRENT", str(Path("/home/nvidia/payload_catch_deploy/current").resolve()))
run(["/usr/bin/pgrep", "-af", "vrpn|mocap|solo_validation|a_node|b_node"])
run(["/usr/bin/find", "/opt/ros/humble/share/vrpn_mocap", "-maxdepth", "3", "-type", "f"])
run(["/usr/bin/python3", "-c", "import numpy,scipy,yaml; print(numpy.__version__,scipy.__version__)"])
