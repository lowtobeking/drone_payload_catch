#!/usr/bin/env python3
"""Real A-only nodes against a FAKE PX4 in an isolated, loopback-only netns."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from dataclasses import asdict
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def component():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from payload_catch.solo_validation_node import SoloNode, test_isolation
    test_isolation()
    role, cfg, receipt, mode, identity = sys.argv[2:]
    args = ["--ros-args", "--log-level", "warn", "-p", "config:="+cfg,
        "-p", "receipt:="+receipt, "-p", "source_root:="+str(ROOT),
        "-p", "execute:="+mode]
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SoloNode(role, test_context={"identity":json.loads(identity)})
    stop = False
    def stopping(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGTERM, stopping)
    try:
        deadline = time.monotonic()+55
        while not stop and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def case(name, report):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from px4_msgs.msg import (VehicleCommand, VehicleCommandAck, TrajectorySetpoint,
                            OffboardControlMode)
    from payload_catch.px4_iface import qos_in
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from payload_catch.solo_validation import SoloConfig, SCOPE, code_digest
    from payload_catch.solo_validation_node import KINDS, TOPIC, bus_qos
    from tools.test_a_solo import packet, receipt as fake_receipt
    monitor = QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
    telemetry_qos = qos_in()
    cfg = SoloConfig(hostname="FAKE_A",board_serial="FAKE_BOARD_"+name,site_checked=True,
        payload_removed=True,positioning="external_local",height_m=.5,hover_s=2.,
        climb_speed=.5,takeoff_timeout_s=10.)
    identity = dict(hostname=cfg.hostname,board_serial=cfg.board_serial,boot_id="NETNS_FAKE_BOOT")
    workers, handles = [], []
    result = dict(case=name, mock_only=True, real_px4=False, b_present=False,
                  commands=[], phases=[], setpoints=0, offboard_modes=0, release_events=[])
    latest = {}
    armed, nav = False, 0
    pos, vel, target = [0.,0.,0.], [0.,0.,0.], [0.,0.,0.]
    def receive_command(msg):
        nonlocal armed, nav
        assert (msg.target_system,msg.target_component,msg.source_system,msg.source_component)==(1,1,255,191)
        assert msg.command in (176,400,21,185)
        result["commands"].append(msg.command)
        if msg.command == 176:
            assert msg.param1 == 1 and msg.param2 == 6
            nav = 14
        elif msg.command == 400:
            assert nav == 14 and msg.param1 == 1
            armed = True
        elif msg.command == 21:
            nav = 18
        elif msg.command == 185:
            armed, nav = False, 13
        ack = VehicleCommandAck()
        ack.timestamp, ack.command, ack.result = int(time.time()*1e6), msg.command, 0
        ack.target_system, ack.target_component = 255, 191
        ackpub.publish(ack)
    def receive_setpoint(msg):
        nonlocal target
        target = [float(x) for x in msg.velocity]
        assert all(math.isfinite(x) for x in target)
        assert math.hypot(*target[:2]) <= cfg.xy_speed+.0001
        assert abs(target[2]) <= cfg.climb_speed+.0001
        result["setpoints"] += 1
    def receive_mode(msg):
        assert msg.velocity and not msg.position and not msg.attitude and not msg.body_rate
        result["offboard_modes"] += 1
    def state(msg):
        nonlocal latest
        data=json.loads(msg.data)
        if data.get("config_digest") != cfg.digest:
            return
        latest = data
        assert data["release_allowed"] is False
        if not result["phases"] or result["phases"][-1] != data["phase"]:
            result["phases"].append(data["phase"])
            print(name, data["phase"], data["reason"], flush=True)
    def request(action, **extra):
        data={k:latest[k] for k in ("scope","session","config_digest","code_digest")}
        import uuid
        data.update(id=uuid.uuid4().hex,stamp=time.time(),action=action,**extra)
        requestpub.publish(String(data=json.dumps(data)))
    rclpy.init(args=[])
    node = Node("a_solo_fake_px4_"+name)
    pubs = {key:node.create_publisher(cls, "/fmu/out/"+suffix,telemetry_qos)
            for key,(cls,suffix) in KINDS.items()}
    ackpub = node.create_publisher(VehicleCommandAck,"/fmu/out/vehicle_command_ack",telemetry_qos)
    requestpub = node.create_publisher(String, TOPIC+"request",bus_qos())
    node.create_subscription(VehicleCommand,"/fmu/in/vehicle_command",receive_command,monitor)
    node.create_subscription(TrajectorySetpoint,"/fmu/in/trajectory_setpoint",receive_setpoint,monitor)
    node.create_subscription(OffboardControlMode,"/fmu/in/offboard_control_mode",receive_mode,monitor)
    node.create_subscription(String, TOPIC+"state",state,bus_qos())
    for topic in ("/payload/authorized_release","/payload/release_event"):
        node.create_subscription(String,topic,lambda msg:result["release_events"].append(msg.data),bus_qos())
    foreign = node.create_publisher(VehicleCommand,"/fmu/in/vehicle_command",monitor) if name=="conflict" else None
    try:
        with tempfile.TemporaryDirectory(prefix="a-solo-",dir=report) as temp:
            cfgpath, receiptpath = Path(temp)/"config.yaml",Path(temp)/"receipt.json"
            import yaml
            cfgpath.write_text(yaml.safe_dump(asdict(cfg)),encoding="utf-8")
            d=fake_receipt(cfg)
            d.update(identity=identity,checked_at=time.time(),code_digest=code_digest(ROOT))
            receiptpath.write_text(json.dumps(d),encoding="utf-8")
            for role in ("supervisor","mission"):
                log=(report/(name+"_"+role+".log")).open("w",encoding="utf-8")
                handles.append(log)
                process=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),"--component",
                    role,str(cfgpath),str(receiptpath),"false" if name=="check_only" else "true",
                    json.dumps(identity)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
                workers.append(process)
            result["worker_pids"]=[p.pid for p in workers]
            started=False
            injected=False
            last_telemetry=0.
            t0=previous=time.monotonic()
            while time.monotonic()-t0 < 35:
                now=time.monotonic()
                dt=min(.05,now-previous)
                previous=now
                if any(p.poll() is not None for p in workers) and not (name=="task_loss" and injected):
                    raise RuntimeError("component exited; inspect logs")
                if armed and nav==14:
                    vel=list(target)
                    for k in range(3):pos[k]+=vel[k]*dt
                elif armed and nav==18:
                    vel=[0.,0.,.4]
                    pos[2]+=vel[2]*dt
                    if pos[2]>=0:
                        pos[2],armed=0.,False
                        vel=[0.,0.,0.]
                if now-last_telemetry>=.05:
                    last_telemetry=now
                    for key,(cls,_) in KINDS.items():
                        if name=="missing" and key=="att":
                            continue
                        msg=cls()
                        msg.timestamp=int(time.time()*1e6)
                        if key in ("land",):
                            msg.landed=not armed or pos[2]>-.02
                        else:
                            fixture=packet(key,msg.timestamp)
                            for attr,value in vars(fixture).items():
                                if hasattr(msg,attr):
                                    setattr(msg,attr,value)
                            if key=="status":
                                msg.system_id,msg.component_id=1,1
                                msg.arming_state,msg.nav_state=(2 if armed else 1),nav
                                msg.pre_flight_checks_pass=not armed
                            elif key=="lpos":
                                msg.x,msg.y,msg.z=pos
                                msg.vx,msg.vy,msg.vz=vel
                                msg.heading=.2
                            elif key=="batt":
                                msg.voltage_v,msg.cell_count=16.2,4
                                msg.remaining=.39 if name=="low_battery" else .8
                        pubs[key].publish(msg)
                rclpy.spin_once(node,timeout_sec=.005)
                if latest.get("phase")=="READY" and not started:
                    request("start",confirm="A_ONLY_TAKEOFF")
                    started=True
                if latest.get("phase")=="FLY" and not injected:
                    if name=="operator_stop":
                        request("stop",reason="mock operator stop")
                    elif name=="task_loss":
                        workers[1].terminate()
                    elif name=="manual":
                        nav=3
                    injected=True
                phase=latest.get("phase")
                if name in ("normal","operator_stop","task_loss") and phase=="DONE":
                    break
                if name=="manual" and phase=="MANUAL":
                    # Prove late task traffic cannot take back control.
                    count=len(result["commands"])
                    until=time.monotonic()+.6
                    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
                    assert len(result["commands"])==count
                    break
                if name in ("missing","low_battery","conflict","check_only") and now-t0>6:
                    assert not result["commands"] and not result["setpoints"] and not result["offboard_modes"]
                    break
            else:
                raise AssertionError("A-only test timed out in "+str(latest))
            if name in ("normal","operator_stop","task_loss"):
                assert latest["phase"]=="DONE" and not armed
                assert 176 in result["commands"] and 400 in result["commands"] and 21 in result["commands"]
                assert max(abs(pos[0]),abs(pos[1]))<.01
            if name=="normal":
                assert "FLY" in result["phases"]
                assert result["setpoints"]>20
            assert not result["release_events"]
            assert not any(topic.startswith("/px4_1/") for topic,_ in node.get_topic_names_and_types())
            result.update(passed=True,final_phase=latest.get("phase"),landed=not armed,final_position=pos)
    finally:
        for process in workers:
            if process.poll() is None:process.terminate()
        for process in workers:
            try:process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        for handle in handles:handle.close()
        result["workers_stopped"]=all(p.poll() is not None for p in workers)
        node.destroy_node()
        rclpy.shutdown()
        (report/(name+".json")).write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
    return result


def main():
    from payload_catch.solo_validation_node import test_isolation
    test_isolation()
    report=ROOT/"report/a_solo_validation_20261019"
    results=[]
    cases = sys.argv[2:] if len(sys.argv)>1 and sys.argv[1]=="--cases" else (\
        "missing","low_battery","conflict","check_only","normal","operator_stop","task_loss","manual")
    for name in cases:
        results.append(case(name,report))
    (report/"ros_summary.json").write_text(json.dumps(results,indent=2)+"\n",encoding="utf-8")
    print("A_SOLO_ROS",len(results),"cases passed; no B or real hardware; all workers stopped",flush=True)


if __name__=="__main__":
    component() if len(sys.argv)>1 and sys.argv[1]=="--component" else main()
