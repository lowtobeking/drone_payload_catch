"""Read public upstream VRPN source as reference; do not install or execute it."""
from pathlib import Path
import hashlib
import json
import urllib.request
WORK=Path(__file__).resolve().parent
root=WORK/"vrpn_reference"
root.mkdir(exist_ok=True)
records=[]
for filename in ("vrpn_BaseClass.C","vrpn_Shared.C"):
    url="https://raw.githubusercontent.com/vrpn/vrpn/version_07.35/"+filename
    with urllib.request.urlopen(url,timeout=15) as stream:
        data=stream.read(2_000_000)
    (root/filename).write_bytes(data)
    records.append({"url":url,"filename":filename,"sha256":hashlib.sha256(data).hexdigest()})
    text=data.decode("utf-8")
    lines=text.splitlines()
    anchors=("vrpn_BaseClassUnique::handle_ping","vrpn_BaseClassUnique::initiate_ping_cycle") if filename=="vrpn_BaseClass.C" else ("vrpn_gettimeofday(",)
    for i,line in enumerate(lines):
        if any(a in line for a in anchors):
            print(filename,i+1,"\n"+"\n".join(lines[max(0,i-2):i+55]))
(root/"manifest.json").write_text(json.dumps(records,indent=2),encoding="utf-8")
