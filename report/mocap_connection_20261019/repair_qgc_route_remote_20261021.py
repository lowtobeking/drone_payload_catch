"""Only add a route to current laptop; retain all existing routes and interfaces."""
from pathlib import Path
import json
import subprocess
import sys
import time
serial=Path("/proc/device-tree/serial-number").read_bytes().rstrip(bytes([0])).decode()
if serial!="1612425621719":
    raise SystemExit("Wrong device")
uuid="5bf25dec-358c-4a85-9eed-ee5b19a254ca"
dev="wlP1p1s0"
work=Path("/home/nvidia/payload_catch_a_solo_20261019/mocap_check_20261021")
state=work/"qgc_peer_route_backup.json"
def run(args):
    p=subprocess.run(args,cwd=work,timeout=20,capture_output=True,text=True,encoding="utf-8",errors="replace",shell=False)
    print(json.dumps({"argv":args,"exit":p.returncode,"stdout":p.stdout,"stderr":p.stderr}),flush=True)
    if p.returncode:
        raise RuntimeError("Command failed")
    return p.stdout.strip()
mode=sys.argv[1]
if mode=="prepare":
    active=run(["nmcli","-g","GENERAL.CON-UUID","device","show",dev])
    if active!=uuid:
        raise SystemExit("Unexpected active Wi-Fi profile")
    before=run(["nmcli","-g","ipv4.routes","connection","show","uuid",uuid])
    data={"profile":uuid,"routes_before":before,"added":False}
    state.write_text(json.dumps(data,indent=2),encoding="utf-8")
    if "192.168.1.124/32" not in before:
        data["added"]=True
        state.write_text(json.dumps(data,indent=2),encoding="utf-8")
        try:
            run(["sudo","-n","nmcli","connection","modify","uuid",uuid,"+ipv4.routes","192.168.1.124/32 0.0.0.0 10"])
            run(["sudo","-n","nmcli","device","reapply",dev])
        except Exception:
            run(["sudo","-n","nmcli","connection","modify","uuid",uuid,"ipv4.routes",before])
            run(["sudo","-n","nmcli","device","reapply",dev])
            raise
    for attempt in range(8):
        route=json.loads(run(["ip","-j","route","get","192.168.1.124"]))
        if route[0]["dev"]==dev:
            break
        time.sleep(0.25)
    if route[0]["dev"]!=dev:
        run(["ip","-j","route","show","table","all"])
        run(["nmcli","-g","ipv4.routes","connection","show","uuid",uuid])
        run(["sudo","-n","ip","route","add","192.168.1.124/32","dev",dev,"metric","10","proto","static"])
        data["manual_runtime_route"]=True
        state.write_text(json.dumps(data,indent=2),encoding="utf-8")
    route=json.loads(run(["ip","-j","route","get","192.168.1.124"]))
    if route[0]["dev"]!=dev:
        raise SystemExit("Route still incorrect; client must rollback")
elif mode=="rollback":
    data=json.loads(state.read_text(encoding="utf-8"))
    if data.get("manual_runtime_route"):
        existing=json.loads(run(["ip","-j","route","show","192.168.1.124/32","dev",dev]))
        if existing:
            run(["sudo","-n","ip","route","del","192.168.1.124/32","dev",dev,"metric","10","proto","static"])
    if data["added"]:
        run(["sudo","-n","nmcli","connection","modify","uuid",uuid,"ipv4.routes",data["routes_before"]])
        run(["sudo","-n","nmcli","device","reapply",dev])
elif mode=="commit":
    data=json.loads(state.read_text(encoding="utf-8"))
    data["verified_normal_ssh"]=True
    state.write_text(json.dumps(data,indent=2),encoding="utf-8")
else:
    raise SystemExit("Unknown mode")
print("ROUTE_"+mode.upper()+"_OK",flush=True)
