"""Remove only extra final blank lines flagged by git, preserving original bytes in report."""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent/"format_backups"
OUT.mkdir(exist_ok=True)
for name in ("config/mocap_a.solo_flight.yaml","report/preflight_20261028/STATUS.md","tools/test_mocap_flight_bridge.py"):
    path=ROOT/name
    data=path.read_bytes()
    (OUT/name.replace("/","__")).write_bytes(data)
    ending=b"\r\n" if b"\r\n" in data else b"\n"
    path.write_bytes(data.rstrip(b"\r\n")+ending)
print("Trimmed EOF blank lines in 3 files; originals preserved")
