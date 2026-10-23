from pathlib import Path
import json
work=Path(__file__).resolve().parent
project=work.parents[1]
run=work/"preview_on_a_20261021T091329Z"
result=json.loads((run/"capture.json").read_text(encoding="utf-8"))
deployment=json.loads((run/"deployment.json").read_text(encoding="utf-8"))
processes=json.loads((run/"processes.json").read_text(encoding="utf-8"))
raw={}
matched=0
max_error=0.
for line in (run/"poses.jsonl").read_text(encoding="utf-8").splitlines():
    data=json.loads(line)
    if data["kind"]=="raw":raw[data["stamp_ns"]]=data
    elif data["stamp_ns"] in raw:
        source=raw[data["stamp_ns"]]["position"]
        expected=[source[1],source[0],-source[2]]
        max_error=max(max_error,max(abs(a-b) for a,b in zip(expected,data["position"])))
        matched+=1
assert result["passed"] and matched>100 and max_error<1e-12
assert all(item["exited"] and item["exit"]==0 for item in processes["cleanup"])
validation=dict(passed=True,matched_same_stamp_positions=matched,max_position_transform_error_m=max_error,
                original_flight_release_unchanged=True,isolated_domain=191,forwarded_to_px4=False,
                manual_movement_recheck_complete=False,server_timing_verified=False)
(run/"validation.json").write_text(json.dumps(validation,indent=2),encoding="utf-8")
summary="""# A 机 planeA 实时转换预览（2026-10-21）

已通过 SSH 在 A 机 Jetson 上运行。飞行发布版本仍为 b4453dd12c8482d808613769fb6c781bd7debff566633bf0fc4827a6d2654d37。
未切换 current、未运行飞行/PWM、未向 PX4 注入定位、未改飞控参数/网络/自启服务。

## 实测

- 192.168.1.111:3883，planeA 原生探针1178帧/约120Hz/无无效或非递增时间戳。
- ROS隔离域191、localhost-only，18秒采集原始2139条、转换2125条。
- 最后周期状态 accepted=2013/rejected=0/stale=false；周期状态早于采集结束。
- 原生子进程退出统计2236帧/invalid0，ROS源published2214/rejected0。统计窗口不同，不可据此直接推算丢包率。
- 最后原始XYZ约[1.074527,-0.791781,0.291146]m，转换XYZ约[-0.791760,1.074515,-0.291126]m；两者为各自最后样本，非同一时间戳。
- 转换滚转/俯仰/航向约[0.195,-0.504,-3.323]度；航向相对实验室所选轴，不是地理北。
- 同时间戳位置转换逐项核对见validation.json。
- 源、转换、采集及原生子进程均已退出。额外查询进程daemon191 PID7351已通过ros2 daemon stop关闭。
- 旧SSH保活28664断线，按此前要求恢复ssh -N会话43619。

## 接收器处理

服务器仍广播数字开头名称450_1，安装的vrpn_mocap1.1.0自动发现后因非法ROS命名空间退出。
实际参数列表没有trackers，传入该参数无效。

当前预览复用原生VRPN API，只构造planeA@192.168.1.111:3883，避开其他刚体自动发现。
11个源码/配置文件经哈希核对，C++/Python/Bash语法检查通过后暂存于：
REMOTE_ROOT

全部电脑侧源码与日志在Ubuntu22.04工程report/mocap_connection_20261019。
原生C++接收器、Python ROS源只用于当前隔离预览；源强制localhost/domain191且最多运行26秒，不是PX4发送器。
ROS PoseStamped使用本机接收时间，服务器时间戳仅做递增检查；不能据此宣称服务器采样/跟踪有效性已验收。
本次源时间数值与A系统墙钟约差12小时，尚未核实时钟语义/同步，不能直接作为PX4采样时间。

## 下一阶段与重复验证

候选安装矩阵、杠杆臂和服务器时序均未验收。本次仅证明接收与转换在Jetson实时运行。
还需人工移动核对前/右/上/水平转向，量距离比例与刚体原点到机体/飞控原点偏移，验证跟踪丢失，再做PX4发送与地面融合。

电脑侧可用Windows Python执行同目录deploy_preview_a_20261021.py，先核对身份/语法/哈希和未解锁着陆状态，再限时采集并清理。
A侧手动重复预览需先运行run_disarm.sh，只有通过后才运行run_preview.sh，两脚本第一个参数为本目录，后者第二参数为唯一的新日志目录。
网络地址变化时先重新只读核实，不擅自改Wi-Fi/DHCP/路由。
预览停止即退出诊断，不涉及飞行配置回退；没有安装系统服务或选择为当前飞行发布。
"""
summary=summary.replace("REMOTE_ROOT",deployment["remote"])
(run/"A机动捕预览运行记录.md").write_text(summary,encoding="utf-8")
doc=project/"docs/A机动捕转换预览.md"
text=doc.read_text(encoding="utf-8")
marker="## 2026-10-21 A 机实时预览已验证"
if marker not in text:
    text+="\n\n"+marker+"\n\n已在A机Jetson独立ROS191域验证planeA接收与转换，18秒原始2139/转换2125条，未向PX4发送。vrpn_mocap因旧450_1自动发现失败，当前预览改用专用planeA原生接收器。安装、杠杆臂和服务器时序仍未验收。详见../report/mocap_connection_20261019/preview_on_a_20261021T091329Z/A机动捕预览运行记录.md。\n"
    doc.write_text(text,encoding="utf-8",newline="\n")
memory=project/"MEMORY.md"
marker="2026-10-21 A机实时动捕转换预览完成"
text=memory.read_text(encoding="utf-8")
if marker not in text:
    text+="\n"+marker+"：A=192.168.1.4/serial1612425621719/boot846ff451；planeA约120Hz正常，旧450_1仍广播导致vrpn_mocap自动发现崩溃，实参不支持trackers。仅预览改用原生planeA专用接收+ROS源，哈希/语法检查部署到/home/nvidia/payload_catch_mocap_preview_20261021/9695526c87f980d0。18秒raw2139/preview2125，状态rejected0、无过期；同戳位置转换核对通过。domain191+localhost隔离，无fmu输入发布，无解锁/PWM/参数/网络修改，current仍b4453dd。预览与原生子进程退出，查询daemon191 PID7351已正常停止。旧SSH28664断线，新ssh-N保活43619保留。候选安装、杠杆臂、服务器时序未验收；源时间与A墙钟约差12小时，不可直接用于PX4采样时间。证据report/mocap_connection_20261019/preview_on_a_20261021T091329Z；未完成PX4发送或融合。\n"
    memory.write_text(text,encoding="utf-8",newline="\n")
temporary=["prepare_native_preview_20261021.py","prepare_preview_retry_20261021.py","read_vrpn_upstream_20261021.py"]
for name in temporary:
    path=work/name
    assert path.parent==work and path.name in temporary
    if path.is_file():path.unlink()
print(json.dumps(validation,indent=2))
print("REPORT",str(run/"A机动捕预览运行记录.md"))
