"""Deploy exact source bundle to A; build and test only, never start a mission."""
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import sys
WORK=Path(__file__).resolve().parent
PROJECT=WORK.parents[1]
sys.path.insert(0,str(WORK))
from probe_a import SSH
BEFORE=json.loads((WORK/"before.json").read_text(encoding="utf-8"))
REMOTE="/home/nvidia/payload_catch_a_deploy_20261104_fusion_default"
BUNDLE=WORK/"source.tar.gz"
with tarfile.open(BUNDLE) as archive:
    manifest=json.load(archive.extractfile("FLEET_MANIFEST.json"))
release_id=manifest["release_id"]
release="/home/nvidia/payload_catch_deploy/releases/"+release_id
local=dict(remote=REMOTE,release=release,release_id=release_id,bundle_sha256=hashlib.sha256(BUNDLE.read_bytes()).hexdigest(),
    git_commit="307056e",before_current=BEFORE["current"],before_previous=BEFORE["previous"],activated=False)
(WORK/"manifest.json").write_text(json.dumps(local,indent=2),encoding="utf-8")
def call(command,label,timeout=35,data=None):
    p=subprocess.run(command,cwd=WORK,capture_output=True,input=data,timeout=timeout,shell=False)
    (WORK/(label+".stdout.log")).write_bytes(p.stdout)
    (WORK/(label+".stderr.log")).write_bytes(p.stderr)
    print(label,"exit",p.returncode,flush=True)
    if p.returncode:
        print(p.stdout.decode(errors="replace"),flush=True)
        print(p.stderr.decode(errors="replace"),flush=True)
        raise RuntimeError(label+" failed")
    return p
def python(code,label):
    ast.parse(code)
    return call(SSH+["/usr/bin/python3 -"],label,data=code.encode())
create="from pathlib import Path\nassert Path('/proc/device-tree/serial-number').read_bytes().rstrip(bytes([0])).decode()=='1612425621719'\nassert Path('/proc/sys/kernel/random/boot_id').read_text().strip()=="+repr(BEFORE["boot"])+"\np=Path("+repr(REMOTE)+");p.mkdir(exist_ok=False)\n"
python(create,"remote_directory")
scp=["C:/Windows/System32/OpenSSH/scp.exe","-o","BindAddress=192.168.1.2","-o","BatchMode=yes","-o","ConnectTimeout=8","-o","StrictHostKeyChecking=yes","-o","HostKeyAlias=192.168.155.186","-i",str(Path.home()/".ssh/id_ed25519")]
call(scp+[str(BUNDLE),"nvidia@192.168.1.5:"+REMOTE+"/source.tar.gz"],"upload_bundle",timeout=70)
call(scp+[str(PROJECT/"tools/fleet_deploy.py"),"nvidia@192.168.1.5:"+REMOTE+"/fleet_deploy.py"],"upload_installer",timeout=30)
shell="""#!/bin/bash
set -eo pipefail
stage=/home/nvidia/payload_catch_a_deploy_20261104_fusion_default
base=/home/nvidia/payload_catch_deploy
source "$base/current/src/deploy/runtime_env.sh"
source "$base/current/ws/install/setup.bash"
export PYTHONDONTWRITEBYTECODE=1
/usr/bin/python3 "$stage/fleet_deploy.py" install --bundle "$stage/source.tar.gz" --role a --base "$base"
"""
remote_code="from pathlib import Path\nimport hashlib,subprocess,json\nroot=Path("+repr(REMOTE)+")\nassert hashlib.sha256((root/'source.tar.gz').read_bytes()).hexdigest()=="+repr(local["bundle_sha256"])+"\np=root/'build.sh';p.write_text("+repr(shell)+",encoding='utf-8')\nsubprocess.run(['/bin/bash','-n',str(p)],check=True,timeout=5)\n"
python(remote_code,"verify_upload")
unit="payload-catch-deploy-build-20261104_fusion_default"
local["build_unit"]=unit
(WORK/"manifest.json").write_text(json.dumps(local,indent=2),encoding="utf-8")
p=call(SSH+["sudo -n systemd-run --unit="+unit+" --property=User=nvidia --property=WorkingDirectory="+REMOTE+" --property=RuntimeMaxSec=600 --property=StandardOutput=file:"+REMOTE+"/build.stdout.log --property=StandardError=file:"+REMOTE+"/build.stderr.log /bin/bash "+REMOTE+"/build.sh"],"launch_build")
print(json.dumps(local,indent=2),flush=True)
