"""Fix the displayed stop command in the continuous-session document."""
from pathlib import Path
PROJECT=Path(__file__).resolve().parents[2]
path=PROJECT/"docs/A机持续动捕融合地面会话_20261026.md"
content=path.read_text(encoding="utf-8")
start=content.index("PowerShell 执行：\n")+len("PowerShell 执行：\n")
end=content.index("\n\n停止请求",start)
command="& 'C:/Users/caoli/AppData/Local/Programs/Python/Python312/python.exe' -B '//wsl.localhost/Ubuntu-22.04/home/caolihao/drone_payload_catch/report/mocap_continuous_20261026/stop_continuous.py'"
content=content[:start]+command+content[end:]
path.write_text(content,encoding="utf-8",newline="\n")
print("Stop command fixed")
