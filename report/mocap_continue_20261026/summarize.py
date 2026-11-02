"""Summarize static evidence honestly; retain raw incomplete manual verdict."""
import bisect
import json
import math
from pathlib import Path
import statistics
WORK=Path(__file__).resolve().parent
run=sorted(WORK.glob("lift_reference_20*"))[-1]
validation=json.loads((run/"validation.json").read_text())
post=sorted(WORK.glob("after_20*"))[-1]
audit=json.loads((post/"comparison.json").read_text())
with (run/"samples.jsonl").open(encoding="utf-8") as handle:
    rows=[json.loads(line) for line in handle]
baseline=validation["phases"]["baseline"]
start=baseline["monotonic"]-4
inputs=[r for r in rows if r["kind"]=="input" and r["monotonic"]>=start]
end=inputs[-1]["monotonic"]
local=[r for r in rows if r["kind"]=="local" and start<=r["monotonic"]<=end]
flags=[r for r in rows if r["kind"]=="flags" and start<=r["monotonic"]<=end]
required=("xy_valid","v_xy_valid","z_valid","v_z_valid","heading_good_for_control")
flags_good=lambda d: all(d[k] for k in ("cs_ev_pos","cs_ev_yaw","cs_ev_hgt")) and not d["cs_mag_hdg"] and not d["cs_mag_3d"]
itimes=[r["monotonic"] for r in inputs]
errors=[]
for row in local:
    index=bisect.bisect_left(itimes,row["monotonic"])
    choices=[i for i in (index-1,index) if 0<=i<len(inputs)]
    index=min(choices,key=lambda i:abs(itimes[i]-row["monotonic"]))
    if abs(itimes[index]-row["monotonic"])>.04:continue
    inp=inputs[index]["data"]
    p=row["data"]
    w,x,y,z=inp["q"]
    yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    errors.append({
        "xy_m":math.hypot(p["x"]-inp["position"][0],p["y"]-inp["position"][1]),
        "z_m":abs(p["z"]-inp["position"][2]),
        "yaw_deg":abs(math.degrees(math.atan2(math.sin(p["heading"]-yaw),math.cos(p["heading"]-yaw))))})
bridge=[r for r in rows if r["kind"]=="bridge"]
good=(bool(local) and bool(flags) and all(all(r["data"][k] for k in required) and not r["data"]["dead_reckoning"] for r in local)
      and all(flags_good(r["data"]) for r in flags))
result={
    "run":run.name,"post":post.name,"static_common_xyz_yaw_valid":good,
    "interval_start_monotonic":start,"interval_end_monotonic":end,
    "observed_static_span_s":end-start,
    "local_samples":len(local),"input_samples":len(inputs),"flag_samples":len(flags),
    "all_disarmed":all(r["data"]["arming_state"]==1 for r in rows if r["kind"]=="status"),
    "all_reported_landed":all(r["data"]["landed"] for r in rows if r["kind"]=="land"),
    "input_estimate_pair_comparison":{"pairs":len(errors),"max_match_age_s":.04,
        "means":{key:statistics.mean(e[key] for e in errors) for key in ("xy_m","z_m","yaw_deg")},
        "maxima":{key:max(e[key] for e in errors) for key in ("xy_m","z_m","yaw_deg")},
        "meaning":"Static input/estimator agreement; not physical accuracy or dynamic latency."},
    "baseline":baseline,"last_bridge_status":bridge[-1]["data"],
    "manual_lift_completed":False,"manual_validation_passed":validation["passed"],
    "raw_error":validation.get("error"),
    "operator_response":"Temporarily unable to lift; exact runtime SIGTERM sent during/near automatic timeout cleanup.",
    "parameter_restoration":validation["lease"],"post_audit_passed":audit["passed"],
    "parameter_differences":audit["parameter_differences"],
    "current_fusion_running":False,"flight_approval":False,
}
(WORK/"final_summary.json").write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
doc=f"""# A机动捕XYZ与航向融合续验（2026-10-26）

用户确认无桨、未解锁、水平放稳且无悬挂物块/牵扯机构。本轮在A机Jetson运行，通过Wi-Fi SSH操作；没有飞行、设定点、解锁或PWM输出。

## 本轮结果

静态XYZ与航向共同有效：{good}，观察区间约{result['observed_static_span_s']:.2f}秒。局部位置样本{len(local)}、动捕输入{len(inputs)}、EKF标志{len(flags)}；区间内位置/速度/可控航向有效、dead_reckoning为false，EV位置/高度/航向开启，磁航向与3D磁融合关闭。

按接收单调时钟匹配附近样本（最大40ms，静止条件下比较）：
- XY平均差：{result['input_estimate_pair_comparison']['means']['xy_m']*1000:.3f}毫米。
- Z平均差：{result['input_estimate_pair_comparison']['means']['z_m']*1000:.3f}毫米。
- 航向平均差：{result['input_estimate_pair_comparison']['means']['yaw_deg']:.4f}度。

这仅说明静态输入与估计接近，不是物理定位精度、动态时延或飞行验收。

人工抬高：**未完成**。用户回答暂时无法完成，只保存了水平baseline，没有elevated/returned样本。原始程序结论passed=false，错误为{validation.get('error')}；正常退出信号按精确运行路径发送，不把该结果改为动态通过。

## 配置与边界

复用2026-10-23有界地面程序；只更新SSH地址/源绑定为当前A机.5、电脑.4及本轮日志路径。原程序和证据未覆盖。完整1127参数先备份；临时EV_CTRL11、HGT_REF3、MAG_TYPE5、SYNCT1。固定实验室[Y,X,-Z]和完整姿态变换，原点0不重置；非地理北向标定。FZMotion/VRPN planeA约120Hz，原始时间仍快约12小时3秒，本轮接收时间仅用于明确授权的地面验证。

真实安装偏移、采样时间/延迟、物理遮挡和飞行常驻桥接未验收。配置中安装/杠杆臂/时间/跟踪失效确认仍false；该桥接解锁即停，不能直接飞行。基准z约-0.292m是实验室刚体原点坐标，不是离地高度。

## 恢复与退出

EV_CTRL0、HGT_REF1、MAG_TYPE0、SYNCT1已恢复并完成飞控重启读回。全1127参数独立审计passed={audit['passed']}；唯一差异：
{json.dumps(audit['parameter_differences'],ensure_ascii=False)}

CAL_BARO0_OFF符合此前PX4启动自动相对气压计校准候选，未强行覆写；不是全参数完全一致。Jetson boot不变，原Agent1155/relay1156和正式current b4453dd不变；本轮采集/源/桥接/守护进程已退出，没有常驻融合。保留上一个任务要求的Wi-Fi网络辅助服务，不改网络或自启。

## 继续入口与证据

- 本轮：report/mocap_continue_20261026/{run.name}/。
- 恢复审计：report/mocap_continue_20261026/{post.name}/comparison.json。
- 汇总：report/mocap_continue_20261026/final_summary.json。
- 下次先重新确认现场条件、身份和planeA，然后准备好连续人工抬高/放回再启动有界验证。必须采当次新基准，不跨运行拼接。
"""
root=WORK.parents[1]
(root/"docs/A机动捕XYZ融合续验_20261026.md").write_text(doc,encoding="utf-8")
memory=root/"MEMORY.md"
text=memory.read_text(encoding="utf-8")+f"""

## 2026-10-26 动捕XYZ与航向实机续验（最新）

用户确认无桨未解锁水平放稳，Wi-Fi .4→A .5，serial1612425621719/boot3f7e18cb保持。8秒原生planeA960帧、约120Hz/invalid0/maxgap21.518ms，源时间快约12h3.11s，继续明确receive_time_ground。复用10月23有界地面工具，只改当前IP/日志，不放宽门禁。1127参数先备份，临时MAG5/EV11/HGT3/SYNCT1和飞控重启核验；静态XYZ/航向共同有效{result['observed_static_span_s']:.2f}s、local{len(local)}、input{len(inputs)}、flags{len(flags)}，不等于动态或绝对精度。

人工抬高询问后用户回答暂时无法完成；仅baseline，未抬高/放回，原始passed=false/{validation.get('error')}，正常退出信号按精确进程发送。EV0/HGT1/MAG0/SYNCT1已回退重启读回，独立1127参数审计passed={audit['passed']}；仅CAL_BARO0_OFF自动校准候选差异，原始详情保留。Jetson/current/Agent1155/relay1156不变，本轮进程0残留，不常驻融合、无解锁/飞行/PWM/网络修改。文档docs/A机动捕XYZ融合续验_20261026.md；完整证据report/mocap_continue_20261026/final_summary.json。动态手动验证仍待，下一次准备好连续动作后重新采基准再测。
"""
memory.write_text(text,encoding="utf-8")
print(json.dumps({k:v for k,v in result.items() if k not in ("last_bridge_status","baseline")},ensure_ascii=False,indent=2))
