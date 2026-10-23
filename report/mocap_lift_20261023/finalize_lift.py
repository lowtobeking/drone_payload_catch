"""Record manual endpoint and continuous-fusion evidence with independent rollback audit."""
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parents[1]
run=sorted(ROOT.glob("lift_reference_20*"))[-1]
post=sorted(ROOT.glob("after_20*"))[-1]
v=json.loads((run/"validation.json").read_text(encoding="utf-8"))
audit=json.loads((post/"comparison.json").read_text(encoding="utf-8"))
rows=[json.loads(line) for line in (run/"samples.jsonl").read_text(encoding="utf-8").splitlines()]
p=v.get("phases",{})
summary=dict(ground_test_passed=v["passed"],post_audit_passed=audit["passed"],flight_approval=False,
    run=run.name,post=post.name,phases=p,lift=v.get("lift"),error=v.get("error"),
    all_disarmed=all(r["data"]["arming_state"]==1 for r in rows if r["kind"]=="status"),
    all_landed=all(r["data"]["landed"] for r in rows if r["kind"]=="land"),
    restored=v.get("lease"),parameter_differences=audit["parameter_differences"],
    remaining_task_processes=audit["remaining_task_processes"])
if "baseline" in p and "returned" in p:
    start=p["baseline"]["monotonic"]-4
    end=p["returned"]["monotonic"]
    local=[r["data"] for r in rows if r["kind"]=="local" and start<=r["monotonic"]<=end]
    flags=[r["data"] for r in rows if r["kind"]=="flags" and start<=r["monotonic"]<=end]
    summary["continuous"] = dict(seconds=end-start,local_samples=len(local),flag_samples=len(flags),
        all_position_velocity_heading_valid=all(all(d[k] for k in
            ("xy_valid","v_xy_valid","z_valid","v_z_valid","heading_good_for_control")) for d in local),
        all_ev_xyz_yaw_active=all(all(d[k] for k in ("cs_ev_pos","cs_ev_yaw","cs_ev_hgt")) for d in flags),
        any_magnetic_fusion=any(d["cs_mag_hdg"] or d["cs_mag_3d"] for d in flags),
        any_dead_reckoning=any(d["dead_reckoning"] for d in local))
summary["passed"]=(v["passed"] and audit["passed"] and
    summary.get("continuous",{}).get("all_ev_xyz_yaw_active",False) and
    summary.get("continuous",{}).get("all_position_velocity_heading_valid",False) and
    not summary.get("continuous",{}).get("any_magnetic_fusion",True))
(ROOT/"final_summary.json").write_text(json.dumps(summary,indent=2),encoding="utf-8")
status="通过" if summary["passed"] else "未通过/未完成"
doc=f"""# A机动捕高度手动抬高连续验证（2026-10-23）

本轮结果：**{status}**。仅无桨未解锁手持地面检查，无解锁、模式、设定点、电机或PWM指令。固定实验室参考，禁止跨运行重取原点来制造一致。完整结果在report/mocap_lift_20261023/final_summary.json。

## 测量

{json.dumps(v.get("lift",{}),indent=2,ensure_ascii=False)}

input_up_m为转换后的动捕抬高量；px4_up_m为同次PX4抬高量；两者正值都表示向上。return_*是放回后相对初始Z差值。用四秒稳定段均值计算，每组样本数、Z幅度及XYZ/Yaw均值见各phase文件。人工移动的真实距离仍需用户测量，不能把两条数据接近当物理绝对精度。

连续融合状态：{json.dumps(summary.get("continuous",{}),ensure_ascii=False)}。这不代替精确延迟、完整安装姿态和物理遮挡验证。

## 安全及恢复

用户现场确认无桨、未解锁、水平放稳、planeA跟踪且无物块/牵扯机构。查看PX4 1.16 LandDetector源码确认未解锁时landed保持true，因此未放宽原着陆门禁；不把该标志解释为人工抬高后仍物理接地。连续身份、未解锁、landed、新鲜参数证明/DDS时间同步/动捕间隙/磁GNSS冲突检查保留。

临时EV_CTRL11/HGT_REF3/MAG_TYPE5/SYNCT1；全参数先备份，独立守护150秒lease；原生源120秒/桥接115秒，人工阶段总96秒有界窗口。先存基准，再收到人工抬高完成标记后重新采五秒（最后四秒均值），再放回同样采集；超时/故障停发回退。没有通过重发旧位姿或重置坐标延长窗口。

恢复及审计：{json.dumps(v.get("lease"),ensure_ascii=False)}。
全参数额外差异：{json.dumps(audit["parameter_differences"],ensure_ascii=False)}，自动气压计校准候选保留，不强行覆写。Jetson boot与正式current、既有服务未改，机载本轮进程审计{audit["remaining_task_processes"]}。飞控按参数要求单独重启，约82秒恢复；网络、时钟、自启不修改。

本轮错误（如有）：{v.get("error")}。

## 证据

report/mocap_lift_20261023/{run.name}/validation.json、baseline.json、elevated.json、returned.json、samples.jsonl、manifest.json及守护和重启记录。
恢复审计report/mocap_lift_20261023/{post.name}/comparison.json。
"""
(PROJECT/"docs/A机动捕高度抬高验证_20261023.md").write_text(doc,encoding="utf-8")
with (PROJECT/"MEMORY.md").open("a",encoding="utf-8") as f:
    f.write("\n\n## 2026-10-23 人工抬高连续融合验证（最新）\n\n"+
        "用户确认无桨未解锁水平放稳/planeA跟踪无牵扯，执行"+run.name+"。结果"+
        json.dumps(summary,ensure_ascii=False)+"\n"+
        "原landed/未解锁/新鲜时间及参数门禁保留，PX4源码未解锁landed强制true，不代表手持飞机物理接地。未解锁/执行器/网络时钟自启修改；当前测试结束不常驻融合。详见docs/A机动捕高度抬高验证_20261023.md。\n")
print(json.dumps(summary,indent=2))
