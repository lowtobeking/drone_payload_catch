"""Write the acceptance marker only when invoked after successful checks."""
from pathlib import Path
import datetime
import json
import re
import sys
release=sys.argv[1]
assert re.fullmatch(r"[0-9a-f]{64}",release)
work=Path(__file__).resolve().parent
source=Path("/home/nvidia/payload_catch_deploy/releases")/release/"src"
sys.path.insert(0,str(source))
from tools.fleet_deploy import verify
assert verify(source)["release_id"]==release
marker=dict(release=release,passed=True,validated_at=datetime.datetime.now(datetime.timezone.utc).isoformat())
(work/"onboard_validated.json").write_text(json.dumps(marker,indent=2)+"\n",encoding="utf-8")
print(json.dumps(marker),flush=True)
