"""Capture proof validation time after reading a concurrently renewed proof."""
import ast
import hashlib
import json
from pathlib import Path

PROJECT=Path(__file__).resolve().parents[2]
ROOT=PROJECT/"report/mocap_right_translation_20261026"
path=PROJECT/"payload_catch/mocap_odometry_node.py"
content=path.read_text(encoding="utf-8")
old='''        issues = fixed_parameter_issues(policy, mono, self.fixed_boot,
'''
new='''        # The guardian can renew the proof while this file is being read.
        # Check its age against a clock read after the file read completes.
        proof_mono = time.monotonic_ns() // 1000
        issues = fixed_parameter_issues(policy, proof_mono, self.fixed_boot,
'''
assert content.count(old)==1
backup=ROOT/"mocap_odometry_node.before_proof_clock_fix.py"
assert not backup.exists()
backup.write_text(content,encoding="utf-8",newline="\n")
changed=content.replace(old,new,1)
ast.parse(changed)
path.write_text(changed,encoding="utf-8",newline="\n")
(ROOT/"proof_clock_fix.json").write_text(json.dumps({"file":"payload_catch/mocap_odometry_node.py","before_sha256":hashlib.sha256(content.encode()).hexdigest(),"after_sha256":hashlib.sha256(changed.encode()).hexdigest(),"change":"read proof before taking proof-validation monotonic time","max_age_us":2000000,"future_timestamp_rejection_preserved":True,"field_failure_reason":"stale/future; exact cause not observable in prior logs"},indent=2),encoding="utf-8")
print("Proof read/check ordering corrected; thresholds and guard unchanged")
