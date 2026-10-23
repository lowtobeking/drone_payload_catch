"""Record current status, compare exact parameter snapshots and clean owned edit helpers."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
WORK = Path(__file__).resolve().parent
before_path = WORK / "parameters_20261023T060446Z/parameters_backup.json"
after_path = WORK / "parameters_20261023T062624Z/parameters_backup.json"
before, after = [json.loads(p.read_text(encoding="utf-8")) for p in (before_path, after_path)]
assert before["complete"] and after["complete"]
assert before["firmware"] == after["firmware"]
assert before["version"]["uid"] == after["version"]["uid"]
changes = {k: {"before": before["parameters"].get(k), "after": after["parameters"].get(k)}
           for k in set(before["parameters"]) | set(after["parameters"])
           if before["parameters"].get(k) != after["parameters"].get(k)}
audit = json.loads((WORK / "final_audit.json").read_text(encoding="utf-8"))
assert not audit["remaining_task_processes"]
assert not changes, changes
result = {"passed": True, "parameter_count": len(before["parameters"]),
          "parameter_changes": changes,
          "before": str(before_path.relative_to(ROOT)),
          "after": str(after_path.relative_to(ROOT)),
          "remaining_task_processes": audit["remaining_task_processes"],
          "flight_release_unchanged": audit["current"].endswith(
              "b4453dd12c8482d808613769fb6c781bd7debff566633bf0fc4827a6d2654d37")}
(WORK / "final_verification.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
memory = ROOT / "MEMORY.md"
entry = """
## 2026-10-23 动捕→PX4 地面桥接实现与诊断（最新）

用户确认仍无桨/未解锁、偏移未测，随后明确暂时无法操作动捕服务器，先完成桥接和诊断验证。Ubuntu22.04新增 mocap_odometry/mocap_odometry_node/mocap_source_node、原生显式planeA接收器、config/mocap_a.bridge.yaml和独立ROS入口。默认disabled、scope=ground_only、确认项false；无真实PX4发布。支持FRD任意朝向、xyzw→wxyz、未知速度NaN；源/接收双时间、排队/倒退/过期/断流锁存，真实输出要求A身份、未解锁已着陆、稳定DDS时间同步及显式启用。它是限时地面工具，不是可直接飞行的常驻服务。

PX4只读确认1.16.0；完整1127参数前后快照一致（含类型/索引）；EKF2_EV_CTRL=0、UXRCE_DDS_SYNCT=1。源时间仍相对Jetson快约12小时并有数秒偏差，映射/不确定度未验证，不自动用到达时间猜源偏移。23逻辑+10转换回归+ROS域192localhost合成检查通过，日志report/mocap_fusion_20261023。

A独立诊断暂存到/home/nvidia/payload_catch_mocap_fusion_20261023/diagnostic_*，精确哈希看manifest。域191localhost，18秒raw1919/odometry333，同戳333对换算通过，最大算术差5.91e-8m（不代表物理精度）。出现一次原生接收间隙277642us（该处源间隔17012us），超过150ms，停发锁存正确；validation.passed=false，连续健康未通过，不能掩盖/放宽阈值。网络或进程调度根因未定位，安装/杠杆臂/服务器时序/跟踪失效仍待验收。最终本地又补严格sensor类型和真实输出竞争发布者检查，隔离回归通过，不把A历史暂存当最终发布。

没有写参数、切current、修改网络或自启、解锁/飞行/PWM。current仍b4453dd，原Agent1019和relay1023保持active；final_audit确认本轮机载进程无残留。后续先解决源时间、偏移及接收间隙，再无桨验证实际里程计输入/EKF。详见docs/A机动捕PX4桥接与诊断_20261023.md；最终参数与进程核对report/mocap_fusion_20261023/final_verification.json。
"""
text = memory.read_text(encoding="utf-8")
if "## 2026-10-23 动捕→PX4 地面桥接实现与诊断（最新）" not in text:
    memory.write_text(text.rstrip() + "\n\n" + entry.strip() + "\n", encoding="utf-8")
preview = ROOT / "docs/A机动捕转换预览.md"
text = preview.read_text(encoding="utf-8")
notice = ("\n> 2026-10-23 更新：已新增独立的 PX4 里程计地面桥接和诊断验证，"
          "默认禁用真实输出；当前仍未完成真实 EKF 融合。"
          "本页介绍的 mocap_preview 仍仅为 PoseStamped 预览。"
          "最新代码、实机间隙失败证据与待办见 "
          "[A机动捕PX4桥接与诊断](A机动捕PX4桥接与诊断_20261023.md)。\n")
if "2026-10-23 更新" not in text:
    head, rest = text.split("\n", 1)
    text = head + "\n" + notice + rest
text = text.replace("下一步先补测安装关系/原点偏移及源时序/跟踪失效，再实现带明确启用门禁的 PX4 VehicleOdometry 发布、时间映射和协方差处理，备份并核对实际 EKF 参数，进行无桨地面融合验收。当前版本不支持切换配置启用真实转发，未完成 PX4 发送或融合。",
                    "原预览节点仍不支持真实转发。2026-10-23 已另行实现默认禁用的 VehicleOdometry 地面桥接，并完成参数备份和诊断；后续先解决安装/原点、源时间/跟踪失效及实测接收间隙，再验收真实 PX4 输入与 EKF 融合。")
preview.write_text(text, encoding="utf-8")
doc = ROOT / "docs/A机动捕PX4桥接与诊断_20261023.md"
text = doc.read_text(encoding="utf-8")
text += ("\n## 最终状态复核\n\n前后两次完整 1127 参数快照逐项一致，"
         "正式 release 保持不变，最终只读检查确认本轮机载进程无残留；"
         "已有 DDS Agent 与 MAVLink 中继继续运行。证据见 "
         "[final_verification.json](../report/mocap_fusion_20261023/final_verification.json)。\n")
doc.write_text(text, encoding="utf-8")
# Only exact helper files created for edits in this task; no recursive cleanup.
for name in ("update_entries.py", "add_replay_test.py"):
    path = WORK / name
    assert path.resolve().parent == WORK.resolve()
    if path.is_file():
        print("remove owned edit helper", path.relative_to(ROOT))
        path.unlink()
cache = ROOT / "payload_catch/__pycache__/mocap_odometry.cpython-310.pyc"
if cache.is_file():
    print("remove generated cache", cache.relative_to(ROOT))
    cache.unlink()
print(json.dumps(result, indent=2))
