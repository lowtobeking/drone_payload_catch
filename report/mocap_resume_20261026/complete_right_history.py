"""Include every right-translation attempt and proof-order correction in the final archive."""
import ast
from pathlib import Path
WORK=Path(__file__).resolve().parent
path=WORK/"save_right_translation.py"
content=path.read_text(encoding="utf-8")
anchor='passed=bool(validation["passed"] and valid_flags and not faults)'
insert='''attempt_history=[]
for attempt in sorted(ROOT.glob("lift_reference_20*")):
    if not (attempt/"validation.json").exists():
        continue
    matches=[p for p in ROOT.glob("after_*/comparison.json") if read(p)["test"]==str(attempt)]
    attempt_history.append({"run":attempt.name,"validation":read(attempt/"validation.json"),
        "audits":[read(p) for p in matches]})
'''
assert content.count(anchor)==1
content=content.replace(anchor,insert+anchor,1)
anchor2='"retry_correction":read(ROOT/"retry_correction.json"),'
assert content.count(anchor2)==1
content=content.replace(anchor2,anchor2+'\n    "attempt_history":attempt_history,"proof_clock_fix":read(ROOT/"proof_clock_fix.json"),',1)
anchor3='只修验收工具，生产桥接及保护未改。'
new='''该首轮只修验收工具。第二轮063332Z完成baseline后，因fixed reference parameter readback stale/future锁存停止，未保存合格right_move；完整日志与独立1127参数恢复审计保留。本次另修项目桥接mocap_odometry_node.py的参数证明校验顺序：读完文件后再获取单调时钟，避免并发刷新时将新证明误判为未来。旧顺序竞态已离线复现，7项检查通过，实际未来/超时/身份/磁融合冲突及锁存保护保留。现场第二轮缺少故障瞬间证明时间，不能断言唯一根因已证实；本轮成功不抹去此前失败。2秒证明时限、源时间与新鲜度门禁、冻结保护均未取消。修复在本地源码及本轮独立测试目录，正式current未切换。'''
assert content.count(anchor3)==1
content=content.replace(anchor3,new,1)
ast.parse(content)
path.write_text(content,encoding="utf-8",newline="\n")
print("All attempts and proof-clock correction will be included in final results")
