"""Verify existing checkpoint with ZIP slash normalization; do not rerun drone work."""
import hashlib
import json
from pathlib import Path
import zipfile
ROOT=Path(__file__).resolve().parents[2]
CLOSE=ROOT/"report/session_pause_20261026"
snapshot=sorted(CLOSE.glob("progress_checkpoint_*.zip"))[-1]
checks={}
with zipfile.ZipFile(snapshot) as archive:
    if archive.testzip():raise RuntimeError("CRC verification failed")
    for name in archive.namelist():
        path=ROOT.joinpath(*name.replace("\\","/").split("/"))
        actual=hashlib.sha256(path.read_bytes()).hexdigest()
        archived=hashlib.sha256(archive.read(name)).hexdigest()
        if actual!=archived:raise RuntimeError("Entry mismatch "+name)
        checks[name]=actual
manifest=dict(path=snapshot.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(snapshot.read_bytes()).hexdigest(),
    entries=checks,verified=True,raw_logs_remain_in_original_directories=True)
(CLOSE/"checkpoint_manifest.json").write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
guide=ROOT/"docs/A机动捕PX4验收进度与继续入口_20261026.md"
audit=ROOT/"report/mocap_yaw_20261026/after_20261026T033215Z/comparison.json"
state=json.loads(audit.read_text(encoding="utf-8"))
if not state["passed"] or state["remaining_task_processes"]:raise RuntimeError("Restoration evidence not passed")
(CLOSE/"pause_state.json").write_text(json.dumps(dict(paused_by_user=True,
    progress_document=guide.relative_to(ROOT).as_posix(),no_automatic_continuation=True,
    last_independent_audit=audit.relative_to(ROOT).as_posix(),
    current_fusion_running=False,remaining_task_processes=[],network_helper_retained=True,
    snapshot_verified=True),ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
print(json.dumps(dict(saved=True,document=guide.relative_to(ROOT).as_posix(),
    snapshot=snapshot.name,entries=len(checks),sha256=manifest["sha256"],
    restoration_verified=True,no_more_drone_operations=True),ensure_ascii=False,indent=2))
