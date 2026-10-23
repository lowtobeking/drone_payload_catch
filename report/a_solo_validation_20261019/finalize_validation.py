"""Record validation evidence and ensure only this task's ROS workers have exited."""
import ast
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
ROOT=Path(__file__).resolve().parents[2]
REPORT=ROOT/"report/a_solo_validation_20261019"
from sys import path
path.insert(0,str(ROOT))
from payload_catch.solo_validation import CODE_FILES,code_digest
summary=json.loads((REPORT/"ros_summary.json").read_text(encoding="utf-8"))
assert len(summary)==8 and all(r["passed"] and r["workers_stopped"] for r in summary)
for row in summary:
    if row["case"] in ("missing","low_battery","conflict","check_only"):
        assert row["commands"]==[] and row["setpoints"]==row["offboard_modes"]==0
    assert not row["release_events"] and row["b_present"] is False and row["real_px4"] is False
own_processes=[]
for item in Path("/proc").iterdir():
    if not item.name.isdigit() or int(item.name)==os.getpid():
        continue
    try:
        cmd=(item/"cmdline").read_bytes().split(b"\0")
    except OSError:
        continue
    # Match argv belonging to this test only, never process-name cleanup.
    decoded=[arg.decode(errors="replace") for arg in cmd if arg]
    if any(arg.endswith("/tools/check_a_solo_ros.py") or arg=="tools/check_a_solo_ros.py"
           or arg.endswith("/tools/run_a_solo_isolated.py") or arg=="tools/run_a_solo_isolated.py"
           for arg in decoded):
        own_processes.append(dict(pid=int(item.name),argv=decoded))
assert not own_processes,own_processes
for name in ("unit.log","dual_gate_regression.log","safety_regression.log","release_regression.log"):
    text=(REPORT/name).read_text(encoding="utf-8")
    assert "\nOK\n" in text,name
assert "Summary: 1 package finished" in (REPORT/"build.log").read_text(encoding="utf-8")
assert "'execute':" in (REPORT/"launch_args.log").read_text(encoding="utf-8")
files=list(CODE_FILES)+["config/a_solo.example.yaml","setup.py","tools/test_a_solo.py",
    "tools/check_a_solo_ros.py","tools/run_a_solo_isolated.py","tools/run_a_solo_isolated.sh"]
for name in files:
    if name.endswith(".py"):
        ast.parse((ROOT/name).read_text(encoding="utf-8"),filename=name)
diff=subprocess.run(["git","diff","--check"],cwd=ROOT,timeout=15,capture_output=True,
                    text=True,encoding="utf-8",errors="replace")
(REPORT/"diff_check.log").write_text(diff.stdout+diff.stderr,encoding="utf-8")
assert diff.returncode==0,diff.stdout+diff.stderr
result=dict(checked_at=time.time(),local_only=True,jetson_deployed=False,
    real_actuator_commands_sent=0,unit_tests=26,dual_gate_regression=10,
    safety_regression=24,timed_release_regression=10,ros_cases=len(summary),
    build_passed=True,launch_arguments_passed=True,diff_check_passed=True,
    code_digest=code_digest(ROOT),source_sha256={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
        for name in files},remaining_task_ros_processes=own_processes)
(REPORT/"validation.json").write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
note=("\n\n## A 机独立起降验收（2026-10-19）\n\n"
      "用户要求先验证 A、不依赖 B。新增独立 a_solo_validation_launch.py / deploy/run_a_solo.sh，"
      "默认 check/execute=false，起飞需当次 A-only 实时参数+遥测检查和显式 start；"
      "强制电量≥40%、定位、RC、飞控自检、身份/电池一致性及控制互斥。"
      "任务为低速垂直起飞1m、稳定悬停10秒、降落并确认着陆上锁；无负载释放/PWM/B接口，"
      "正式双机门禁不变。独立监督处理任务失联与故障，人工接管不抢回，stop降落/kill切动力分开。"
      "26逻辑+8隔离ROS场景及原双机10/安全24/定时释放10回归通过，Humble构建通过。"
      "本次仅Ubuntu22.04开发，未部署Jetson、未真实解锁或动作。"
      "上次实机无RC/水平定位且电池节数不一致仍是阻塞，未新读确认。"
      "操作与限制见docs/A机单机起降验收.md；证据report/a_solo_validation_20261019/validation.json。\n")
for name in ("AGENTS.md","MEMORY.md"):
    target=ROOT/name
    content=target.read_text(encoding="utf-8")
    if "## A 机独立起降验收（2026-10-19）" not in content:
        target.write_text(content+note,encoding="utf-8",newline="\n")
readme=ROOT/"README.md"
content=readme.read_text(encoding="utf-8")
line="\nA机单机起降验收（无需B、默认只检查、禁止释放）：[操作说明](docs/A机单机起降验收.md)。\n"
if line not in content:
    readme.write_text(content+line,encoding="utf-8",newline="\n")
print(json.dumps(result,ensure_ascii=False,indent=2))
