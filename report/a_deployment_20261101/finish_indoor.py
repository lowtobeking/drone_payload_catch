"""Verify indoor configuration independently and save deployment documentation."""
import ast
import json
from pathlib import Path
import subprocess
from probe_a import SSH

WORK = Path(__file__).resolve().parent
ROOT = WORK.parents[1]
remote = """from pathlib import Path
import hashlib,json,sys
base=Path('/home/nvidia/payload_catch_deploy')
sys.path.insert(0,str(base/'current/src'))
from payload_catch.solo_validation import load_config
cfg=load_config(base/'a_solo.yaml')
assert cfg.positioning=='external_local' and not cfg.gps
assert cfg.height_m==1.0 and cfg.hover_s==10.0
assert not cfg.site_checked and not cfg.payload_removed
assert hashlib.sha256((base/'host/runtime.yaml').read_bytes()).hexdigest()=='1752e0550ea832ca9a8f6459b17f2c14651bebfe70922e4b10ff790bf5a771ac'
print(json.dumps(dict(positioning=cfg.positioning,gps_required=cfg.gps,height_m=cfg.height_m,hover_s=cfg.hover_s,host_runtime_unchanged=True)))
"""
ast.parse(remote)
p = subprocess.run(SSH + ["/usr/bin/python3 -"], input=remote.encode(), cwd=WORK, capture_output=True, timeout=35, shell=False)
(WORK/"indoor_verify.stdout.log").write_bytes(p.stdout)
(WORK/"indoor_verify.stderr.log").write_bytes(p.stderr)
if p.returncode:
    raise RuntimeError(f"Remote readback exit={p.returncode}: {p.stderr.decode(errors='replace')}")
result = json.loads(p.stdout)
(WORK/"indoor_verification.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
doc = ROOT/"docs/A机单机起降部署_20261101.md"
s = doc.read_text(encoding="utf-8")
start = s.index("## 当前配置限制")
end = s.index("## 回退", start)
replacement = """## 当前配置限制

用户明确指定室内使用动捕后，已将 A 机 /home/nvidia/payload_catch_deploy/a_solo.yaml 的 positioning 从 outdoor_gps 改为 external_local，GPS 不再是本任务的起飞条件。本地 config/a_solo.example.yaml 同步选择室内模式。实际配置独立读回通过，仅定位模式改变，height_m=1.0、hover_s=10.0、site_checked=false、payload_removed=false 保留。

室内模式仍要求 PX4 输出有效、新鲜的融合本地位置、速度和航向，自检通过并由操作员明确开始。动捕当前不可用，因此尚不能启动实际起飞。未启动动捕桥接、任务、解锁或执行器，未修改 PX4 参数。host/runtime.yaml 哈希保持不变。单机配置相关的26项测试通过。

现场配置备份：/home/nvidia/payload_catch_a_deploy_20261101/a_solo.before_indoor.yaml。恢复配置应在着陆、未解锁且无任务运行时进行；程序版本回退不会自动恢复现场 a_solo.yaml。

"""
doc.write_text(s[:start]+replacement+s[end:],encoding="utf-8")
note = """\n## 2026-11-01 室内动捕配置更正

- 按用户要求，A机现场 a_solo.yaml 与本地单机示例选择 external_local，不要求 GPS；任务仍为相对起飞1米、稳定后悬停10秒、降落。
- 现场配置只改变 positioning，保留 site_checked=false、payload_removed=false；备份 /home/nvidia/payload_catch_a_deploy_20261101/a_solo.before_indoor.yaml。
- 独立SSH读回通过，host/runtime.yaml 未变，26项单机逻辑测试通过。
- 未启动任务/桥接/解锁/执行器，未修改 PX4 参数。动捕缺失或 PX4 融合本地定位无效时仍禁止起飞。本次不是实飞验收。
"""
with (ROOT/"MEMORY.md").open("a",encoding="utf-8") as f:
    f.write(note)
with (WORK/"A机部署记录.md").open("a",encoding="utf-8") as f:
    f.write(note)
print(json.dumps(result,indent=2))
print("Deployment documentation and MEMORY updated; 26 solo tests passed.")
