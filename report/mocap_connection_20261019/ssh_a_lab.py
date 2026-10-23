from pathlib import Path
import shlex
import subprocess
import sys
work=Path(__file__).resolve().parent
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")
script=work/sys.argv[1]
argv=["/usr/bin/python3","-"] if script.suffix==".py" else ["/bin/bash","-s"]
args=[r"C:/Windows/System32/OpenSSH/ssh.exe","-o","BatchMode=yes","-o","ConnectTimeout=6",
      "-o","StrictHostKeyChecking=yes","-o","HostKeyAlias=192.168.155.186",
      "-i",str(Path.home()/".ssh/id_ed25519"),"nvidia@192.168.1.4",shlex.join(argv)]
p=subprocess.run(args,input=script.read_text(encoding="utf-8"),cwd=work,timeout=85,
                 capture_output=True,text=True,encoding="utf-8",errors="replace",shell=False)
(work/(script.stem+".stdout.log")).write_text(p.stdout,encoding="utf-8")
(work/(script.stem+".stderr.log")).write_text(p.stderr,encoding="utf-8")
print(p.stdout)
print(p.stderr,file=sys.stderr)
raise SystemExit(p.returncode)
