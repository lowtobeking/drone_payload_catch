from pathlib import Path
import subprocess
p=subprocess.run(["/usr/bin/dpkg","-L","ros-humble-vrpn-mocap"],capture_output=True,text=True,timeout=8,shell=False)
print(p.stdout,p.stderr)
for root in [Path("/opt/ros/humble/include/vrpn_mocap"),Path("/home/nvidia/ros2_ws/src")]:
    if root.exists():
        for path in root.rglob("*"):
            if path.is_file() and ("vrpn" in str(path).lower()) and path.suffix in [".hpp",".h",".cpp"]:
                print("SOURCE",path)
                print(path.read_text(errors="replace")[:16000])
