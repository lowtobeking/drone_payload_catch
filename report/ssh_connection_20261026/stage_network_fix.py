"""Stage checked installer via Python stdin and run under a rollback timer."""
import ast
import base64
from pathlib import Path
import subprocess
import sys
WORK=Path(__file__).resolve().parent
SSH=["C:/Windows/System32/OpenSSH/ssh.exe","-F","NUL","-b","10.41.10.50","-o","BatchMode=yes","-o","ConnectTimeout=8","-o","StrictHostKeyChecking=yes","-o","HostKeyAlias=192.168.155.186","-i",str(Path.home()/".ssh/id_ed25519"),"nvidia@10.41.10.100","/usr/bin/python3 -"]
def main():
    code=(WORK/"network_transaction.py").read_bytes()
    ast.parse(code)
    bootstrap=("import ast,base64,subprocess\nfrom pathlib import Path\n"
        +"data=base64.b64decode("+repr(base64.b64encode(code).decode())+")\n"
        +"ast.parse(data)\np=Path('/home/nvidia/payload_catch_network_transaction_20261026_v3.py')\n"
        +"with p.open('xb') as handle: handle.write(data)\n"
        +"proc=subprocess.run(['sudo','-n','/usr/bin/python3',str(p),'install'],timeout=60)\n"
        +"raise SystemExit(proc.returncode)\n")
    ast.parse(bootstrap)
    p=subprocess.run(SSH,input=bootstrap.encode(),cwd=WORK,capture_output=True,timeout=75,shell=False)
    with (WORK/"network_transaction.stdout.log").open("ab") as handle:handle.write(p.stdout)
    with (WORK/"network_transaction.stderr.log").open("ab") as handle:handle.write(p.stderr)
    print(p.stdout.decode("utf-8",errors="replace"),flush=True)
    print(p.stderr.decode("utf-8",errors="replace"),file=sys.stderr,flush=True)
    sys.exit(p.returncode)
if __name__=="__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
