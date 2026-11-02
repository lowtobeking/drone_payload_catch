"""Write a current concise progress entry from the completed right-move result."""
import json
from pathlib import Path
PROJECT=Path(__file__).resolve().parents[2]
result=json.loads((PROJECT/"report/mocap_right_translation_20261026/final_summary.json").read_text(encoding="utf-8"))
assert result["ground_test_passed"] and result["audit"]["passed"]
assert len(result["attempt_history"])==3
text="""# 2026-10-26 A机验收进度

已验证：静态XYZ与航向融合、水平左转、水平右转、前移停稳、向右平移停稳，以及此前的抬高稳定片段。

| 本日动作 | 动捕 | PX4 | 数据一致性 |
| --- | ---: | ---: | --- |
| 左转 | 61.93° | 61.72° | 航向变化差0.21° |
| 右转 | 58.27° | 57.83° | 航向变化差0.44° |
| 前移 | 76.45cm | 76.52cm | 水平位移向量差0.83mm |
| 右移 | 90.90cm | 90.84cm | 水平位移向量差1.88mm |
| 抬高片段 | 70.08cm | 69.97cm | 高度变化差1.07mm |

上述位移是动捕与PX4的一致性，物理距离未经独立标定，不代表绝对定位精度。右移时机头约变化7.82°，低于本次10°地面门槛，不宣称严格纯平移。

本轮参数恢复及1127项参数独立审计通过，测试任务无残留，当前没有持续融合。原有部署和通信服务保持。

待完成：放回后的高度稳定段/全程，运动中的偏差与时延、源时间问题、安装偏移、午前冻结根因、物理跟踪失效/恢复、飞行桥接与接管及起飞条件验收。本次地面通过不放行飞行。

右移首轮验收计数误改、第二轮参数证明stale/future失败均完整保留；项目桥接的文件读取/校验取时并发竞态已修正，7项离线检查通过，原有时间保护阈值保留。该修复已用于本次独立实验，正式current未切换。

[详细验收表、恢复状态与继续入口](docs/A机动捕PX4验收进度与继续入口_20261026.md)
"""
(PROJECT/"2026-10-26_A机验收进度.md").write_text(text,encoding="utf-8",newline="\n")
assert "向右平移真实融合（最新）" in (PROJECT/"docs/A机动捕PX4验收进度与继续入口_20261026.md").read_text(encoding="utf-8")
assert "向右平移真实融合（最新）" in (PROJECT/"MEMORY.md").read_text(encoding="utf-8")
print(json.dumps({"saved":True,"run":result["run"],"attempts_preserved":len(result["attempt_history"]),"flags_samples":result["fusion_flags_samples"],"test_parameters_restored":result["audit"]["test_parameters_restored"],"remaining_task_processes":result["audit"]["remaining_task_processes"]}))
