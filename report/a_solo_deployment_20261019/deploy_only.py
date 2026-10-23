"""Windows orchestrator: select the built A release without running mission/tests."""
from pathlib import Path
import json
import shlex
import subprocess
import sys

sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
WORK=Path(__file__).resolve().parent
REMOTE="/home/nvidia/payload_catch_a_solo_20261019"
HOST="nvidia@192.168.155.186"
RELEASE=(WORK/"release_id.txt").read_text(encoding="utf-8").strip()
SSH=[r"C:/Windows/System32/OpenSSH/ssh.exe","-o","BatchMode=yes","-o","ConnectTimeout=6",
     "-o","StrictHostKeyChecking=yes","-o","ServerAliveInterval=10","-o","ServerAliveCountMax=3",
     "-i",str(Path.home()/".ssh/id_ed25519"),HOST]

def remote(argv,name,input=None):
    p=subprocess.run(SSH+[shlex.join(argv)],cwd=WORK,timeout=45,capture_output=True,
                     text=True,encoding="utf-8",errors="replace",input=input,shell=False)
    (WORK/(name+".stdout.log")).write_text(p.stdout,encoding="utf-8")
    (WORK/(name+".stderr.log")).write_text(p.stderr,encoding="utf-8")
    if p.returncode:
        print(p.stdout); print(p.stderr,file=sys.stderr)
        raise SystemExit(p.returncode)
    return p.stdout

def probe(name):
    data=json.loads(remote(["/usr/bin/python3","-"],name,
                    (WORK/"remote_probe.py").read_text(encoding="utf-8")))
    (WORK/(name+".json")).write_text(json.dumps(data,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    return data

def main():
    before=probe("deploy_only_before_probe")
    script=(WORK/"select_files_only.py").read_text(encoding="utf-8")
    # tee receives the script bytes as stdin; no shell interpolation of the source.
    remote(["/usr/bin/tee",REMOTE+"/select_files_only.py"],"upload_select_only",script)
    result=json.loads(remote(["/usr/bin/python3",REMOTE+"/select_files_only.py",RELEASE],"select_only"))
    after=probe("deploy_only_after_probe")
    for key in ("hostname","serial","boot_id","host_profile_sha256","boot_config_sha256","pwm33","services"):
        assert before[key]==after[key],key+" changed unexpectedly"
    assert Path(after["current"]).name==RELEASE
    assert after["previous"]==before["current"]
    assert not after["mission_processes"] and all(after["locks_available"].values())
    result["post_deploy_readonly_verification_passed"]=True
    result["no_remaining_mission_or_test_processes"]=True
    result["rollback_command"]="/usr/bin/python3 "+REMOTE+"/select_files_only.py "+RELEASE+" --rollback"
    (WORK/"deployment_status.json").write_text(json.dumps(result,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    print(json.dumps(result,indent=2,ensure_ascii=False))

if __name__=="__main__":
    main()
