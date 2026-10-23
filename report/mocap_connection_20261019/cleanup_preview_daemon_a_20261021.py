"""Stop only the exact CLI daemon PID created by this preview inspection."""
from pathlib import Path
import json
import os
import signal
import subprocess
import time
pid=7351
expected=[b"/usr/bin/python3",b"-c",b"from ros2cli.daemon.daemonize import main; main()",b"--name",b"ros2-daemon",b"--ros-domain-id",b"191",b"--rmw-implementation",b"rmw_fastrtps_cpp"]
proc=Path("/proc")/str(pid)
def alive():
    return proc.exists() and (proc/"stat").read_text().split()[2]!="Z"
if alive():
    actual=[v for v in (proc/"cmdline").read_bytes().split(b"\0") if v]
    if actual!=expected:raise RuntimeError("PID identity changed; not stopping")
    root=Path("/home/nvidia/payload_catch_mocap_preview_20261021")
    script=root/"stop_owned_cli_daemon.sh"
    script.write_text("#!/bin/bash\nset -eo pipefail\nsource /opt/ros/humble/setup.bash\nexport ROS_DOMAIN_ID=191\nexport ROS_LOCALHOST_ONLY=1\nexec /opt/ros/humble/bin/ros2 daemon stop\n",encoding="utf-8")
    subprocess.run(["/bin/bash","-n",str(script)],check=True,timeout=3)
    p=subprocess.run(["/bin/bash",str(script)],capture_output=True,text=True,timeout=8,shell=False)
    print(json.dumps(dict(cli_exit=p.returncode,stdout=p.stdout,stderr=p.stderr)))
    for _ in range(60):
        if not alive():break
        time.sleep(.05)
    if alive():
        actual=[v for v in (proc/"cmdline").read_bytes().split(b"\0") if v]
        if actual!=expected:raise RuntimeError("PID identity changed")
        os.kill(pid,signal.SIGKILL)
        for _ in range(60):
            if not alive():break
            time.sleep(.05)
    if alive():raise RuntimeError("owned daemon still active")
    script.unlink()
print(json.dumps(dict(owned_daemon_pid=pid,stopped=True)))
