"""Read-only Wi-Fi capabilities and target BSSID scan."""
import json
import subprocess
result={}
for name,args in (
    ("link",["iw","dev","wlP1p1s0","link"]),
    ("regulatory",["iw","reg","get"]),
    ("capabilities",["iw","list"]),
    ("supplicant",["wpa_supplicant","-v"]),
    ("scan",["nmcli","-t","-f","BSSID,SSID,FREQ,SECURITY,SIGNAL","dev","wifi","list","ifname","wlP1p1s0","--rescan","no"]),
):
    try:
        p=subprocess.run(args,cwd="/home/nvidia",timeout=12,capture_output=True,text=True,encoding="utf-8",errors="replace",shell=False)
        output=p.stdout
        if name=="capabilities":
            output="\n".join(line for line in output.splitlines() if any(t in line for t in ("MHz","Supported Ciphers","SAE","Device supports","Wiphy","Authentication")))
        result[name]=dict(exit=p.returncode,stdout=output,stderr=p.stderr)
    except FileNotFoundError as error:
        result[name]=dict(error=str(error))
print(json.dumps(result,indent=2))
