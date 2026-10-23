"""Windows SSH orchestration; all work files and logs stay in WSL project."""
from pathlib import Path
import json
import shlex
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[2]
WORK=ROOT/"report/a_solo_deployment_20261019"
REMOTE="/home/nvidia/payload_catch_a_solo_20261019"
HOST="nvidia@192.168.155.186"
KEY=str(Path.home()/".ssh/id_ed25519")
SSH=[r"C:/Windows/System32/OpenSSH/ssh.exe","-o","BatchMode=yes","-o","ConnectTimeout=6",
     "-o","ConnectionAttempts=1","-o","StrictHostKeyChecking=yes","-o","ServerAliveInterval=10",
     "-o","ServerAliveCountMax=3","-i",KEY]
SCP=[r"C:/Windows/System32/OpenSSH/scp.exe","-o","BatchMode=yes","-o","ConnectTimeout=6",
     "-o","StrictHostKeyChecking=yes","-i",KEY]
RELEASE=(WORK/"release_id.txt").read_text(encoding="utf-8").strip()
def run(args,name,timeout=60,input=None):
    proc=subprocess.run(args,cwd=WORK,timeout=timeout,capture_output=True,
        text=True,encoding="utf-8",errors="replace",shell=False,input=input)
    (WORK/(name+".stdout.log")).write_text(proc.stdout,encoding="utf-8")
    (WORK/(name+".stderr.log")).write_text(proc.stderr,encoding="utf-8")
    print(name,"exit",proc.returncode,flush=True)
    if proc.returncode:
        print(proc.stdout,flush=True);print(proc.stderr,file=sys.stderr,flush=True)
        raise SystemExit(proc.returncode)
    return proc.stdout
def remote(args,name,timeout=60,input=None):
    return run(SSH+[HOST,shlex.join(args)],name,timeout,input)
def main():
    phase=sys.argv[1]
    if phase=="probe":
        data=remote(["/usr/bin/python3","-"],"before_probe",input=(WORK/"remote_probe.py").read_text(encoding="utf-8"))
        (WORK/"before.json").write_text(json.dumps(json.loads(data),indent=2)+"\n",encoding="utf-8")
        print(data,flush=True)
    elif phase=="upload":
        remote(["mkdir","-p",REMOTE],"mkdir")
        names=("payload-catch-a-solo.tar.gz","installer.py","validate_onboard.sh","build_onboard.sh","switch_onboard.py","mark_validated.py")
        run(SCP+[str(WORK/name) for name in names]+[HOST+":"+REMOTE+"/"],"upload",timeout=120)
    elif phase=="build":
        print(remote(["/bin/bash",REMOTE+"/build_onboard.sh",RELEASE],"build",timeout=300),flush=True)
    elif phase=="validate":
        print(remote(["/bin/bash",REMOTE+"/validate_onboard.sh",RELEASE],"onboard_validation",timeout=240),flush=True)
    elif phase=="switch":
        print(remote(["/usr/bin/python3",REMOTE+"/switch_onboard.py",RELEASE],"switch",timeout=60),flush=True)
    elif phase=="after":
        data=remote(["/usr/bin/python3","-"],"after_probe",input=(WORK/"remote_probe.py").read_text(encoding="utf-8"))
        (WORK/"after.json").write_text(json.dumps(json.loads(data),indent=2)+"\n",encoding="utf-8")
        print(data,flush=True)
    else:raise SystemExit("phase: probe/upload/build/validate/switch/after")
if __name__=="__main__":main()
