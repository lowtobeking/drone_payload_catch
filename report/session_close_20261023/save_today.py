"""Save today's source/evidence index and an independently verified local snapshot."""
from datetime import datetime,timezone
import ast
import hashlib
import json
from pathlib import Path
import shutil
import zipfile
ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parents[1]
assert str(PROJECT)=="/home/caolihao/drone_payload_catch"
lift=PROJECT/"report/mocap_lift_20261023"
# Preserve corrected helper as diagnostic evidence; no further remote calls.
helper=lift/"stop_session.py"
s=helper.read_text(encoding="utf-8")
s=s.replace("p/chr(99).__add__(chr(119)).__add__(chr(100))", "p/'cwd'")
ast.parse(s)
helper.write_text(s,encoding="utf-8")
for directory,names in (
    ("mocap_lift_20261023",("prepare_lift.py",)),
    ("mocap_z_20261023",("prepare_z.py","prepare_tests.py"))):
    base=PROJECT/"report"/directory
    target=base/"diagnostic_helpers";target.mkdir(exist_ok=True)
    for name in names:
        src=base/name;dest=target/name
        assert src.resolve().parent==base and dest.resolve().parent==target
        if src.exists():
            assert not dest.exists(),dest
            shutil.move(str(src),str(dest))
xyz=json.loads((PROJECT/"report/mocap_z_20261023/final_summary.json").read_text(encoding="utf-8"))
manual=json.loads((lift/"final_summary.json").read_text(encoding="utf-8"))
assert xyz["passed"] and manual["post_audit_passed"]
assert manual["restored"]["restored"] and manual["restored"]["restore_restart_verified"]
assert not manual["remaining_task_processes"]
record="""# 2026-10-23 工作收尾与下次继续入口

用户要求结束今日工作。本轮采集和本任务SSH命令均已退出，没有设置监控或自动继续任务；没有关机、断电或修改SSH服务、网卡、路由、时钟。既有Agent与QGC转发服务保持运行。

## 今日已完成

1. 建立固定实验室坐标参考，关闭磁融合，实机验证动捕XY与航向约26秒，PX4位置/速度/可控航向均有效。
2. 接入动捕Z并作为高度参考，保留气压计辅助，实机XYZ+航向共同有效约26秒；稳定20秒中Z输入与PX4平均差约0.64毫米，仅表示静态数据接近，不代表物理定位精度。
3. 软件断流后停止发送并锁存；完整参数先备份，恢复原参数与飞控重启读回已核验。
4. 本次人工抬高验证只完成水平基准。用户暂不能抬高并要求结束，没有抬高或放回数据；不得记为动态通过。程序原始记录为manual interaction window expired，随后自动恢复，记录保留。
5. 当前EV_CTRL=0、HGT_REF=1、MAG_TYPE=0、DDS SYNCT=1，当前没有常驻动捕融合或飞行任务。本轮所有机载测试进程已退出，正式current仍旧发布b4453dd。新增源码在Ubuntu22.04工程，A机独立暂存由各manifest定位，未擅自切正式current。

1127参数已独立重新读取，唯一额外差异为CAL_BARO0_OFF的自动相对校准候选值变化，保留完整原始差异，未强行覆盖。Jetson本轮未重启；飞控按测试/恢复配置曾单独重启。

## 已保存位置

- docs/A机固定动捕航向实机验证_20261023.md
- docs/A机动捕XYZ与航向融合_20261023.md
- docs/A机动捕高度抬高验证_20261023.md
- report/fixed_reference_20261023、report/mocap_z_20261023、report/mocap_lift_20261023：源码、配置、快照、原始日志、失败证据及校验清单。
- MEMORY.md：最新进度和待办。

## 下次继续

先确认无桨、未解锁、planeA跟踪及SSH/QGC；读取最新MEMORY，重新采当次基准。在同一连续融合记录里人工抬高已知距离并放回，核对Z方向、变化幅度与返回误差。之后做前/右平移和转向；测量IMU安装偏移、起降平面、延迟及物理遮挡/恢复；飞行常驻桥接和失效/人工接管保护仍待。现有桥接是有界地面工具，解锁即停发，不能直接飞行。

本目录快照包含源代码、配置、文档、测试入口和主要JSON验收/清单；原始大日志保留原报告目录，没有删除。未创建提交、推送或外部上传。
"""
(ROOT/"2026-10-23_工作收尾与继续入口.md").write_text(record,encoding="utf-8")
with (PROJECT/"MEMORY.md").open("a",encoding="utf-8") as f:
    f.write("\n\n## 2026-10-23 用户结束今日工作（最后状态）\n\n"+
        "用户暂不能抬高，随后明确先结束、保存今日成果并退出工作状态。手动验证只完成baseline，原始程序因手动窗口到期退出，EV0/HGT1/MAG0/SYNCT1已恢复且飞控重启核验；after_20261023T103137Z独立1127参数审计通过，仅自动气压计校准候选差异，本轮机载进程0。已完成XY/yaw与XYZ/yaw静止融合仍有效，动态抬高待测，当前没有常驻融合。Jetson/SSH/网络及既有服务未变，没有自动继续任务。成果快照及继续入口report/session_close_20261023，用户再次要求继续前不再设备操作。\n")
paths=set()
for name in ("payload_catch","config","tools","launch","deploy","docs"):
    for p in (PROJECT/name).rglob("*"):
        if p.is_file() and "__pycache__" not in p.parts and p.suffix not in (".pyc",".pyo"):
            paths.add(p)
paths.update(p for p in PROJECT.glob("*.md") if p.is_file())
paths.add(ROOT/"2026-10-23_工作收尾与继续入口.md")
for task in ("fixed_reference_20261023","mocap_z_20261023","mocap_lift_20261023","mocap_fusion_20261023"):
    for p in (PROJECT/"report"/task).rglob("*.json"):
        if p.name in ("validation.json","final_summary.json","manifest.json","comparison.json",
                      "parameter_lease_result.json","baseline.json","identity.json","local_ekf_validation.json",
                      "configure_reboot_verified.json","restore_reboot_verified.json"):
            paths.add(p)
entries=[]
stamp=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
archive=ROOT/("source_and_results_"+stamp+".zip")
with zipfile.ZipFile(archive,"x",compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for p in sorted(paths):
        data=p.read_bytes();rel=p.relative_to(PROJECT).as_posix()
        entries.append(dict(path=rel,size=len(data),sha256=hashlib.sha256(data).hexdigest()))
        z.writestr(rel,data)
with zipfile.ZipFile(archive,"r") as z:
    assert z.testzip() is None
    for e in entries:
        assert hashlib.sha256(z.read(e["path"])).hexdigest()==e["sha256"]
manifest=dict(archive=archive.name,files=len(entries),bytes=archive.stat().st_size,
    archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),archive_verified=True,
    raw_logs_preserved_in_project=True,entries=entries)
(ROOT/"snapshot_manifest.json").write_text(json.dumps(manifest,indent=2),encoding="utf-8")
print(json.dumps({k:v for k,v in manifest.items() if k!="entries"},indent=2))
print("No remote commands or processes started by this archival operation")
