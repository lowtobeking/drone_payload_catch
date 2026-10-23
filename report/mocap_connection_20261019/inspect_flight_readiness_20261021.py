from pathlib import Path
import json
import subprocess
serial=Path("/proc/device-tree/serial-number").read_bytes().rstrip(bytes([0])).decode()
if serial!="1612425621719":
    raise SystemExit("A identity mismatch")
base=Path("/home/nvidia/payload_catch_deploy")
print("IDENTITY",serial,"BOOT",Path("/proc/sys/kernel/random/boot_id").read_text().strip(),flush=True)
for path in [base/"current",base/"a_solo.yaml",base/"current/src/config/jetson_a.runtime.yaml",base/"current/src/deploy/runtime_env.sh"]:
    print("FILE",str(path),"RESOLVED",str(path.resolve()),flush=True)
    if path.is_file():
        print(path.read_text(encoding="utf-8"),flush=True)
commands=[
["systemctl","show","payload-catch-agent.service","payload-catch-mav-relay.service","-p","Id","-p","ActiveState","-p","ExecStart"],
["systemctl","cat","payload-catch-mav-relay.service"],
["ss","-ulnp"],
["ps","-eo","pid,ppid,args"],
["ip","route"],
]
for args in commands:
    p=subprocess.run(args,capture_output=True,text=True,encoding="utf-8",errors="replace",timeout=10,shell=False)
    print(json.dumps({"argv":args,"exit":p.returncode,"stdout":p.stdout,"stderr":p.stderr}),flush=True)
