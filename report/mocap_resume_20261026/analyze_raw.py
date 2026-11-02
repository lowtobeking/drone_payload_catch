"""Summarize raw pose repeats without altering source evidence."""
import json
from pathlib import Path

def analyze(root):
    samples=[json.loads(line) for line in (root/"raw.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]
    groups=[]
    start=0
    for idx in range(1,len(samples)+1):
        same=idx<len(samples) and samples[idx]["position"]==samples[start]["position"] and samples[idx]["quaternion_xyzw"]==samples[start]["quaternion_xyzw"]
        if same:
            continue
        elapsed=(samples[idx-1]["receive_monotonic_us"]-samples[start]["receive_monotonic_us"])/1e6
        if elapsed>=0.1:
            groups.append({"first_index":start,"last_index":idx-1,"frames":idx-start,"span_s":elapsed,"server_span_s":(samples[idx-1]["server_stamp_us"]-samples[start]["server_stamp_us"])/1e6,"pose":samples[start]})
        start=idx
    result={"frames":len(samples),"duration_s":(samples[-1]["receive_monotonic_us"]-samples[0]["receive_monotonic_us"])/1e6,"max_receive_gap_s":max((b["receive_monotonic_us"]-a["receive_monotonic_us"])/1e6 for a,b in zip(samples,samples[1:])),"identical_groups_ge_100ms":groups,"identical_groups_gt_500ms":sum(g["span_s"]>0.5 for g in groups),"first":samples[0],"last":samples[-1]}
    (root/"raw_analysis.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    print(json.dumps({"directory":str(root),"frames":result["frames"],"duration_s":result["duration_s"],"max_receive_gap_s":result["max_receive_gap_s"],"identical_groups_ge_100ms":len(groups),"identical_groups_gt_500ms":result["identical_groups_gt_500ms"]}))
if __name__=="__main__":
    analyze(sorted(Path(__file__).resolve().parent.glob("raw_turn_20*"))[-1])
