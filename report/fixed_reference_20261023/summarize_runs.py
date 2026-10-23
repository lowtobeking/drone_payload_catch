"""Record chronological FC restart/recovery evidence from bounded runs."""
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parent
rows=[]
for p in sorted(ROOT.glob("fixed_reference_20*")):
    row={"run":p.name}
    for n in ("configure_reboot_requested.json","configure_reboot_verified.json","restore_reboot_requested.json","restore_reboot_verified.json","parameter_lease_result.json","validation.json"):
        f=p/n
        if f.exists():row[n]=json.loads(f.read_text(encoding="utf-8"))
    for n in ("guard.log","ground_test.stdout.log","ground_test.stderr.log"):
        f=p/n
        if f.exists():row[n+"_bytes"]=f.stat().st_size
    rows.append(row)
(ROOT/"run_history.json").write_text(json.dumps(rows,indent=2),encoding="utf-8")
print(json.dumps([dict(run=r["run"],validation=r.get("validation.json",{}).get("error","pending"),lease=r.get("parameter_lease_result.json")) for r in rows],indent=2))
