"""Read-only A deployment identity and baseline; no flight/parameter commands."""
import ast
import json
from pathlib import Path
import subprocess
WORK=Path(__file__).resolve().parent
SSH=["C:/Windows/System32/OpenSSH/ssh.exe","-F","NUL","-b","192.168.1.2","-o","BatchMode=yes","-o","ConnectTimeout=8","-o","StrictHostKeyChecking=yes","-o","HostKeyAlias=192.168.155.186","-i",str(Path.home()/".ssh/id_ed25519"),"nvidia@192.168.1.5"]
CODE="""from pathlib import Path
import hashlib,json,os,subprocess
serial=Path('/proc/device-tree/serial-number').read_bytes().rstrip(bytes([0])).decode()
assert serial=='1612425621719'
base=Path('/home/nvidia/payload_catch_deploy')
def cmd(args):
 p=subprocess.run(args,capture_output=True,text=True,timeout=10)
 return dict(exit=p.returncode,stdout=p.stdout,stderr=p.stderr)
files={}
for name in ['host.runtime.yaml','runtime.yaml','a_solo.yaml','ROLE']:
 p=base/name
 if p.exists():files[name]=dict(sha256=hashlib.sha256(p.read_bytes()).hexdigest(),bytes=p.stat().st_size)
services=cmd(['systemctl','show','payload-catch-agent.service','payload-catch-mav-relay.service','payload-catch-wifi-peer.service','-p','Id','-p','ActiveState','-p','MainPID','-p','NRestarts'])
processes=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit() or int(p.name)==os.getpid():continue
 try:
  args=(p/'cmdline').read_bytes().replace(bytes([0]),b' ').decode(errors='replace')
  if any(token in args for token in ['payload_catch.a_node','payload_catch.b_node','solo_validation_node','a_solo_runtime.py','mocap_ground','mocap_source_node','mocap_odometry_node']):
   processes.append(dict(pid=int(p.name),args=args))
 except (OSError,PermissionError):pass
result=dict(serial=serial,hostname=cmd(['hostname'])['stdout'].strip(),boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),architecture=os.uname().machine,
 current=str((base/'current').resolve()),previous=str((base/'previous').resolve()),files=files,services=services,
 releases=sorted(p.name for p in (base/'releases').iterdir()),task_processes=processes,
 network=cmd(['ip','-br','address']),sudo=cmd(['sudo','-n','true']))
print(json.dumps(result,indent=2))
"""
def run_probe(label):
 ast.parse(CODE)
 p=subprocess.run(SSH+["/usr/bin/python3 -"],input=CODE.encode(),cwd=WORK,capture_output=True,timeout=35,shell=False)
 (WORK/(label+".stdout.log")).write_bytes(p.stdout)
 (WORK/(label+".stderr.log")).write_bytes(p.stderr)
 if p.returncode:
  raise RuntimeError(p.stderr.decode(errors="replace"))
 result=json.loads(p.stdout)
 (WORK/(label+".json")).write_text(json.dumps(result,indent=2),encoding="utf-8")
 print(json.dumps(result,indent=2))
 return result
if __name__=="__main__":
 run_probe("before")
