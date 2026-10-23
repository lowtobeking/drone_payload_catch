from pathlib import Path
import json
root=Path("/home/nvidia/payload_catch_mocap_preview_20261021")
print("CURRENT",str(Path("/home/nvidia/payload_catch_deploy/current").resolve()))
print("BOOT",Path("/proc/sys/kernel/random/boot_id").read_text().strip())
for path in Path("/proc").iterdir():
    if not path.name.isdigit():continue
    try:
        argv=(path/"cmdline").read_bytes().split(b"\0")
        text=[v.decode(errors="replace") for v in argv if v]
        joined=" ".join(text)
        if str(root) in joined or ("ros2" in joined and ("daemon" in joined or "191" in joined)):
            print(json.dumps(dict(pid=int(path.name),argv=text)))
    except (FileNotFoundError,PermissionError,ProcessLookupError):pass
for path in sorted(root.glob("*/logs/*/processes.json")):
    print("PROCESS_RECORD",path.read_text())
