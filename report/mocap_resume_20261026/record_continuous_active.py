"""Record that the manually started continuous ground fusion is active."""
import json
from pathlib import Path
PROJECT=Path(__file__).resolve().parents[2]
report=PROJECT/"report/mocap_continuous_20261026"
active=json.loads((report/"active_session.json").read_text(encoding="utf-8"))
line=f"""\n## 2026-10-26 持续动捕-PX4地面融合已启动

用户要求直接融合 PX4 与动捕。本次手动会话已在 A 机运行：{active["unit"]}，正式 current 未切换、无开机自启。EV位置/高度/航向融合启用，磁融合关闭，持续门禁要求 A 身份正确、未解锁、已着陆、动捕新鲜且单一发布者。实时检查已见动捕转发、PX4 EV XYZ/yaw 有效、磁融合关闭、flight_approval=false。

这是无桨地面会话，不是飞行放行。安全停止脚本位于 report/mocap_continuous_20261026/stop_continuous.py；停止后守护恢复 EV0/HGT1/MAG0 并重启核验。证据与会话说明见 docs/A机持续动捕融合地面会话_20261026.md。
"""
summary=PROJECT/"2026-10-26_A机验收进度.md"
summary.write_text(summary.read_text(encoding="utf-8")+line,encoding="utf-8",newline="\n")
with (PROJECT/"MEMORY.md").open("a",encoding="utf-8",newline="\n") as f:
    f.write(line)
print(json.dumps(active,ensure_ascii=False))
