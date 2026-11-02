"""Save active continuous-session details and a safe stop helper."""
from datetime import datetime
import ast
import json
from pathlib import Path
import subprocess
PROJECT=Path(__file__).resolve().parents[2]
REPORT=PROJECT/"report/mocap_continuous_20261026"
run=sorted(REPORT.glob("lift_reference_20*"))[-1]
manifest=json.loads((run/"manifest.json").read_text(encoding="utf-8"))
status={
    "saved_at":datetime.now().astimezone().isoformat(),
    "run":run.name,
    "remote":manifest["remote"],
    "remote_logs":manifest["remote_logs"],
    "unit":manifest["unit"],
    "current_switched":False,
    "autostart":False,
    "scope":"manual continuous disarmed ground fusion",
    "flight_approval":False,
    "temporary_parameters":{"EKF2_EV_CTRL":11,"EKF2_HGT_REF":3,"EKF2_MAG_TYPE":5,"UXRCE_DDS_SYNCT":1},
    "stop_contract":"touch remote_logs/stop.request; session stops source/bridge then guard restores EV0/HGT1/MAG0 and reboots FC to verify",
    "device_serial":"1612425621719",
}
(REPORT/"active_session.json").write_text(json.dumps(status,indent=2),encoding="utf-8")
stop=REPORT/"stop_continuous.py"
stop.write_text('''"""Stop the exact continuous session and wait for parameter restoration."""
import ast
import json
from pathlib import Path
import subprocess
WORK=Path(__file__).resolve().parent
manifest=json.loads((WORK/"active_session.json").read_text(encoding="utf-8"))
code="from pathlib import Path\\nPath("+repr(manifest["remote_logs"]+"/stop.request") + ").touch()\\nprint('stop requested')\\n"
ast.parse(code)
ssh=["C:/Windows/System32/OpenSSH/ssh.exe","-F","NUL","-b","192.168.1.4","-o","BatchMode=yes","-o","ConnectTimeout=8","-o","HostKeyAlias=192.168.155.186","-o","StrictHostKeyChecking=yes","-i",str(Path.home()/".ssh/id_ed25519"),"nvidia@192.168.1.5","/usr/bin/python3 -"]
proc=subprocess.run(ssh,input=code.encode(),cwd=WORK,capture_output=True,timeout=20,shell=False)
print(proc.stdout.decode("utf-8",errors="replace"),end="")
print(proc.stderr.decode("utf-8",errors="replace"),end="")
raise SystemExit(proc.returncode)
''',encoding="utf-8",newline="\n")
ast.parse(stop.read_text(encoding="utf-8"))
doc=PROJECT/"docs/A机持续动捕融合地面会话_20261026.md"
doc.write_text(f"""# A机持续动捕融合地面会话（2026-10-26）

本次用户要求直接融合 PX4 与动捕，已在 A 机启动手动地面会话。

- 会话：{run.name}
- 机载临时单元：{manifest["unit"]}
- 作用范围：A 机无桨、未解锁的地面融合
- 动捕：FZMotion planeA，VRPN 192.168.1.111:3883
- 输入处理：使用 Jetson 接收时间，源报文时间偏差仍未作为飞行时序依据
- PX4：EV 位置、EV 高度、EV 航向融合；磁力计航向和磁力计3D融合关闭
- 当前参数：EKF2_EV_CTRL=11、EKF2_HGT_REF=3、EKF2_MAG_TYPE=5、UXRCE_DDS_SYNCT=1
- 状态门禁：持续检查 A 机身份、未解锁、已着陆、动捕新鲜度、PX4融合标志、参数证明和单一发布者
- 当前不提供：解锁、飞行设定点、电机/PWM、常驻开机自启、飞行放行
- 正式部署 current 未切换，原有 Agent、relay 和 Wi-Fi 辅助服务未停止

安全停止和回退：
PowerShell 执行：
& 'C:/Users/caoli/AppData/Local/Programs/Python/Python312/python.exe' -B '//wsl.localhost/Ubuntu-22.04/home/caolihao/drone_payload_catch\report\mocap_resume_20261026\stop_continuous.py'

停止请求会先结束动捕源和桥接，再由守护恢复 EKF2_EV_CTRL=0、EKF2_HGT_REF=1、EKF2_MAG_TYPE=0 并重启飞控核验。不要直接杀守护进程，也不要在会话运行时解锁。

原始会话日志：{manifest["remote_logs"]}；本地证据目录：{run}。本会话不等于飞行放行。
""",encoding="utf-8",newline="\n")
print(json.dumps(status,indent=2))
