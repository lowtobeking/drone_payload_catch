from pathlib import Path
import subprocess
roots=[Path('/home/nvidia/ros2_ws/src'),Path('/home/nvidia/ros2_ws/install')]
for root in roots:
    print('ROOT',root,'EXISTS',root.is_dir(),flush=True)
    if not root.is_dir(): continue
    for p in sorted(root.iterdir()):
        if any(x in p.name.lower() for x in ('mocap','vrpn')):
            print('PACKAGE',p,flush=True)
            files=list(p.rglob('*.cpp'))+list(p.rglob('*.py'))+list(p.rglob('*.yaml'))+list(p.rglob('README*'))
            for f in files[:16]:
                if f.is_file() and f.stat().st_size<45000:
                    print('FILE',f,flush=True)
                    print(f.read_text(encoding='utf-8',errors='replace'),flush=True)
for p in [Path('/opt/ros/humble/share/vrpn_mocap/package.xml'),Path('/opt/payload_catch/managed_20261016/px4_msgs_ws/src/px4_msgs/msg/VehicleOdometry.msg')]:
    if p.is_file(): print('FILE',p,p.read_text(encoding='utf-8'),flush=True)
