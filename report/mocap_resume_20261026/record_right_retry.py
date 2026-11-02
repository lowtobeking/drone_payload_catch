"""Record the exact first-attempt failure and correction without changing evidence."""
import ast
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent/"mocap_right_translation_20261026"
first=ROOT/"lift_reference_20261026T062808Z"
validation=json.loads((first/"validation.json").read_text(encoding="utf-8"))
audit=json.loads((ROOT/"after_20261026T063228Z/comparison.json").read_text(encoding="utf-8"))
assert not validation["passed"] and audit["passed"]
record={"first_run":first.name,"first_validation":validation,"post_audit_passed":audit["passed"],
    "cause":"experiment preparation renamed forwarded counter to righted; no baseline/motion prompt issued",
    "correction":"restore forwarded schema key; five heading/right-projection checks passed",
    "production_bridge_or_guards_changed":False,"failed_remote_stage_preserved":True}
(ROOT/"retry_correction.json").write_text(json.dumps(record,indent=2),encoding="utf-8")
path=Path(__file__).resolve().parent/"save_right_translation.py"
content=path.read_text(encoding="utf-8")
old='"all_observed_flags_ev_xyz_yaw_no_mag":valid_flags,"latched_bridge_faults":faults,'
new=old+'\n    "retry_correction":read(ROOT/"retry_correction.json"),'
assert content.count(old)==1
content=content.replace(old,new,1)
marker='用户指定验证向右移动。本轮{run.name}，地面动作验收**{state}**。'
newmarker=marker+'\n\n首轮062808Z在基准前因验收脚本误将转发计数字段forwarded写为righted而中止，未发出移动提示。该误改已修正，五组方向投影检查通过；原失败记录与独立恢复审计保留在retry_correction.json，不改写为成功。只修验收工具，生产桥接及保护未改。'
assert content.count(marker)==1
content=content.replace(marker,newmarker,1)
ast.parse(content)
path.write_text(content,encoding="utf-8",newline="\n")
print("First attempt failure and exact correction saved; final save script parsed")
