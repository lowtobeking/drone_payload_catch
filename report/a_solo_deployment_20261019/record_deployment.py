"""Record file-only deployment outcome locally; no aircraft connection."""
from pathlib import Path
import json
root=Path(__file__).resolve().parents[2]
work=Path(__file__).resolve().parent
status=json.loads((work/"deployment_status.json").read_text(encoding="utf-8"))
assert status["status"]=="deployed_not_run"
assert status["post_deploy_readonly_verification_passed"] is True
release=status["release"]
heading="A 机单机版实机部署（2026-10-19，仅部署不运行）"
note=(
    "\n\n## "+heading+"\n\n"
    "用户最新指示：先部署，不用跑。已在 A 机 nvidia@192.168.155.186 完成 ARM64 构建，"
    "current 已切换至 "+release+"；previous 保留旧版 13c1e49da415339aa236e19547c63a809de15e55235584917a700e964dd1702a。"
    "入口 /home/nvidia/payload_catch_deploy/current/src/deploy/run_a_solo.sh；"
    "配置 /home/nvidia/payload_catch_deploy/a_solo.yaml 保留 site_checked=false/payload_removed=false。"
    "此次未运行测试、ROS任务、飞行或PWM，未设置开机自启。文件/进程复核确认无任务和测试进程残留，"
    "服务、PWM、boot配置和host/runtime.yaml不变。"
    "单元测试此前通过；机载隔离ROS验收因新建report目录归属导致PermissionError，尚未通过。"
    "此前中断的目录修复未执行到源码，部署的是已编译的b4453dd版本。"
    "保留起飞门禁，不代表具备真实起飞条件。"
    "部署报告及精确回退命令见 report/a_solo_deployment_20261019/A机仅部署记录.md。\n"
)
for name in ("AGENTS.md","MEMORY.md"):
    p=root/name
    text=p.read_text(encoding="utf-8")
    if heading not in text:
        with p.open("a",encoding="utf-8") as handle:
            handle.write(note)
fence=chr(96)*3
report=(
    "# A机单机版部署记录\n\n"
    "2026-10-19，按用户要求仅部署，不运行。\n\n"
    "- 设备：nvidia@192.168.155.186，nvidia-desktop，ARM64 / Ubuntu22.04 / ROS2 Humble。\n"
    "- 当前目录：/home/nvidia/payload_catch_deploy/current。\n"
    "- 当前版本："+release+"。\n"
    "- 回退版本：13c1e49da415339aa236e19547c63a809de15e55235584917a700e964dd1702a。\n"
    "- 配置：/home/nvidia/payload_catch_deploy/a_solo.yaml，现场确认字段仍为false。\n\n"
    "ARM编译通过。文件完整性、版本链接和部署前后只读检查通过。"
    "无任务或测试进程残留；启动配置、通信服务、主机运行配置及PWM状态未改变。"
    "未运行任务或执行器，未添加开机自启。\n\n"
    "单机入口用于A机自检、起飞1m、稳定悬停10秒、降落上锁验收，不含B机协调或舵机投放。"
    "完整工程其他入口保留，未启动。\n\n"
    "此前26项单机逻辑、7项参数协议、10项起飞门禁及飞控参数测试通过。"
    "机载隔离ROS测试因新建日志目录权限失败，尚未通过；本次未重新运行，未生成验收通过标记。"
    "本部署记录不能替代实机飞行验收。\n\n"
    "## 回退\n\n"
    "在A机SSH终端执行下列命令，仅恢复版本链接，不运行任务：\n\n"
    +fence+"bash\n"+status["rollback_command"]+"\n"+fence+"\n\n"
    "部署前指针保存在A机 /home/nvidia/payload_catch_a_solo_20261019/deploy_only_before.json。"
    "旧版代码及构建产物均保留。\n\n"
    "## 证据\n\n"
    "- deployment_status.json：最终部署状态。\n"
    "- deploy_only_before_probe.json / deploy_only_after_probe.json：前后只读检查。\n"
    "- select_only.stdout.log / select_only.stderr.log：远端切换结果。\n"
    "- build.stdout.log / build.stderr.log：机载编译日志。\n"
    "- onboard_validation.stdout.log / onboard_validation.stderr.log：此前验收失败记录。\n"
)
(work/"A机仅部署记录.md").write_text(report,encoding="utf-8")
archive=work/"diagnostic_helpers"
archive.mkdir(exist_ok=True)
for name in ("complete_validation_workflow.py","fix_test_report_owner.py"):
    p=work/name
    if p.is_file():
        target=archive/name
        assert not target.exists()
        p.rename(target)
print("Deployment report and project memory updated; obsolete helpers archived")
