"""Record qualified latest result, preserve evidence, remove exact task-owned edit helpers."""
from pathlib import Path
import ast
import hashlib
import json
ROOT=Path(__file__).resolve().parents[2]
WORK=ROOT/"report/mocap_fusion_20261023"
note="""

## 2026-10-23 航向候选地面静态测试通过（最新，覆盖“航向可控仍false”）

用户确认A无桨未解锁放稳、planeA跟踪、QGC连通，并说明无北向标定。SSH仍.123→.101/serial1612425621719/boot a006295d。只读heading_probe_20261023T081303Z采集1586对，动捕labFRD yaw约-1.274543°、PX4磁估计yaw-47.109135°，差-45.834592°/std0.024870°。候选将Rz(delta)同时用于位置和完整姿态，对齐当次PX4参考并输出NED；**north_verified=false，仅地面候选，不宣称真实北向标定，不能直接用于双机共享轴**。配置要求来源、相同旋转、同boot和10分钟启动有效期，默认严格/普通FRD配置不变。

首次heading_candidate_20261023T080951Z源间隙>150ms锁存，forwarded0/未改参数，证据保留；只调整源先预热4秒200帧，再启动桥接，阈值不放宽。第二次heading_candidate_20261023T081425Z通过：cs_ev_pos/cs_ev_yaw 22样本跨度21.210s，启用5秒后2206个位置/速度/航向可控同时有效，末10s全有效且dead_reckoning=false；桥接accepted3685/forwarded1117/rejected0。**静态候选通过不是飞行放行或动态/绝对方向验收**；启动间隙根因及长期稳定仍未证明。新增6航向单元及ROS位置姿态一致/过期拒绝/解锁停发，通过原23+9+10回归。

全程观测未解锁着陆、无飞行/PWM指令。EV_CTRL临时9后已自动恢复0，SYNCT始终1，1127项参数数值类型索引前后0差异；current仍b4453dd，Agent1011/relay1013正常，任务进程0。没有常驻融合。最新完整参数parameters_20261023T081620Z；审计receive_final_audit.json。

保存前移基准heading_movement_baseline.json：raw约[1.8785946,-.1999071,.2916852]，候选EV约[1.2082929,1.4522768,-.2916850]，PX4约[1.2080616,1.4526118,1.1031359]，yaw-.8219225rad。高度仍气压计且静止长期明显漂移，**不要把PX4 z与EV z当同基准或宣称高度验收**。下一步先人工前移/右移/转向，先比逐姿态融合位移再记录连续动态；若快照超时/重启须显式重采同基准，不能混参考相减。其后高度、刚体偏移、时延、遮挡断流/保护和持续飞行桥接，供电false/电池等既有门禁仍待解决。报告docs/A机航向参考与静态融合验证_20261023.md；汇总heading_candidate_summary.json，候选完整配置在通过run目录candidate_config.yaml。
"""
p=ROOT/"MEMORY.md"
s=p.read_text(encoding="utf-8")
assert "航向候选地面静态测试通过（最新" not in s
p.write_text(s+note,encoding="utf-8")
p=ROOT/"docs/A机动捕接收时间地面融合_20261023.md"
s=p.read_text(encoding="utf-8")
idx=s.index("\n")
s=s[:idx+1]+"""
> 后续更新：候选旋转将动捕对齐至当次 PX4 磁航向估计后，静态位置、速度、航向可控标志均通过。真实北向、动态、偏移与高度仍未标定/验收，参数已回退，不是飞行放行。详见 [航向候选静态验证](A机航向参考与静态融合验证_20261023.md)。本文以下“航向未通过”保留为前一轮局部FRD测试的结果。
"""+s[idx+1:]
p.write_text(s,encoding="utf-8")
checked=[]
for relative in ["payload_catch/mocap_odometry.py","payload_catch/mocap_odometry_node.py",
    "tools/test_mocap_heading_candidate.py","tools/check_mocap_odometry_ros.py",
    "report/mocap_fusion_20261023/heading_probe_runtime_a.py",
    "report/mocap_fusion_20261023/run_heading_probe_on_a.py",
    "report/mocap_fusion_20261023/run_heading_candidate_on_a.py",
    "report/mocap_fusion_20261023/receive_ground_runtime_a.py",
    "report/mocap_fusion_20261023/receive_parameter_guard_a.py",
    "report/mocap_fusion_20261023/summarize_heading_candidate.py"]:
    ast.parse((ROOT/relative).read_text(encoding="utf-8"),relative)
    checked.append(relative)
removed=[]
for name in ["prepare_heading_probe.py","implement_heading_candidate.py","prepare_heading_test.py",
             "refine_heading_startup.py","extend_heading_ros.py"]:
    path=WORK/name
    assert path.parent.resolve()==WORK.resolve()
    if path.is_file():
        removed.append(dict(path=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        path.unlink()
(WORK/"heading_local_check.json").write_text(json.dumps(dict(
    syntax_checked=checked,removed_task_owned_helpers=removed),indent=2),encoding="utf-8")
print("Latest result documented; exact one-off editing helpers removed.")
