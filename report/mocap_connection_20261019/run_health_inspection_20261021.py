from pathlib import Path
import ast
import shlex
import subprocess
import sys
work=Path(__file__).resolve().parent
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
base=[r"C:/Windows/System32/OpenSSH/ssh.exe","-o","BatchMode=yes","-o","ConnectTimeout=6","-o","StrictHostKeyChecking=yes","-o","HostKeyAlias=192.168.155.186","-i",r"C:/Users/caoli/.ssh/id_ed25519","nvidia@192.168.1.4"]
remote="/home/nvidia/payload_catch_a_solo_20261019/mocap_check_20261021"
def run(args,name,data=None,timeout=40):
    p=subprocess.run(base+[shlex.join(args)],cwd=work,input=data,capture_output=True,timeout=timeout,shell=False)
    (work/(name+".stdout.log")).write_bytes(p.stdout)
    (work/(name+".stderr.log")).write_bytes(p.stderr)
    print(p.stdout.decode("utf-8",errors="replace"),flush=True)
    print(p.stderr.decode("utf-8",errors="replace"),file=sys.stderr,flush=True)
    if p.returncode:raise SystemExit(p.returncode)
source=(work/"sample_flight_health_20261021.py").read_text(encoding="utf-8")
ast.parse(source)
uploader="from pathlib import Path; Path("+repr(remote+"/sample_health.py")+").write_bytes("+repr(source.encode("utf-8"))+")"
run(["/usr/bin/python3","-"],"health_upload",uploader.encode("utf-8"))
shell='#!/bin/bash\nset -eo pipefail\nsource /home/nvidia/payload_catch_deploy/current/src/deploy/runtime_env.sh\nsource /home/nvidia/payload_catch_deploy/current/ws/install/setup.bash\nexport PYTHONDONTWRITEBYTECODE=1\nexec /usr/bin/timeout --signal=TERM --kill-after=3s 25s /usr/bin/python3 '+remote+'/sample_health.py\n'
uploader="from pathlib import Path; Path("+repr(remote+"/run_health.sh")+").write_bytes("+repr(shell.encode("utf-8"))+")"
run(["/usr/bin/python3","-"],"health_shell_upload",uploader.encode("utf-8"))
run(["/bin/bash","-n",remote+"/run_health.sh"],"health_shell_syntax")
run(["/usr/bin/python3","-c","import ast,pathlib; ast.parse(pathlib.Path("+repr(remote+"/sample_health.py")+").read_text(encoding='utf-8'))"],"health_python_syntax")
run(["/bin/bash",remote+"/run_health.sh"],"flight_health_20261021",timeout=40)
