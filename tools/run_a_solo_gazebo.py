#!/usr/bin/env python3
"""Gazebo/PX4 A-only demo. Loopback-only netns; never connects to a real FC."""
from pathlib import Path
import os,sys,json,time,subprocess,signal,pwd,hashlib,socket
from dataclasses import asdict
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
REPORT=ROOT/"report/a_solo_gazebo_20261019"
IDENTITY=dict(hostname="GAZEBO_SITL_A",board_serial="GAZEBO_ONLY_NO_BOARD",
              boot_id="ISOLATED_GAZEBO_"+Path("/proc/sys/kernel/random/boot_id").read_text().strip())

class GazeboClock:
    """Raw PX4/Gazebo simulation clock, only inside the isolated SITL context."""
    def __init__(self):
        from gz.transport13 import Node
        from gz.msgs10.clock_pb2 import Clock
        self.node=Node()
        self.stamp=0.
        def update(msg):
            self.stamp=msg.sim.sec+msg.sim.nsec*1e-9
        if not self.node.subscribe(Clock,"/world/default/clock",update):
            raise RuntimeError("Gazebo clock subscription failed")
    def now(self):
        return self.stamp


def save(path,data):
    path.write_text(json.dumps(data,indent=2,ensure_ascii=False,default=str)+"\n",encoding="utf-8")

def component():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from payload_catch.solo_validation_node import SoloNode,test_isolation
    test_isolation()
    clock=GazeboClock()
    role=sys.argv[2]
    rclpy.init(args=["--ros-args","-p","config:="+str(REPORT/"config.yaml"),
        "-p","receipt:="+str(REPORT/"simulation_receipt.json"),"-p","source_root:="+str(ROOT),
        "-p","execute:=true"],signal_handler_options=SignalHandlerOptions.NO)
    node=SoloNode(role,test_context={"identity":IDENTITY,"source_time":clock.now})
    stop=False
    def stopping(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGTERM,stopping)
    try:
        while not stop:rclpy.spin_once(node,timeout_sec=.02)
    finally:node.destroy_node();rclpy.shutdown()

def worker():
    def stopping(*_):raise KeyboardInterrupt("Gazebo demo stopping")
    signal.signal(signal.SIGTERM,stopping)
    signal.signal(signal.SIGINT,stopping)
    import rclpy,yaml
    from rclpy.node import Node
    from std_msgs.msg import String
    from pymavlink import mavutil
    from payload_catch.solo_validation_node import test_isolation,AHealth,TOPIC,bus_qos
    from payload_catch.solo_validation import SoloConfig,preflight_errors,parameter_issues,SCOPE,code_digest
    test_isolation()
    REPORT.mkdir(exist_ok=True,parents=True)
    run=REPORT/("run_"+time.strftime("%H%M%S"))
    run.mkdir()
    save(REPORT/"latest_run.json",dict(path=str(run),pid=os.getpid(),netns=os.readlink("/proc/self/ns/net")))
    processes=[];handles=[];manifest=[]
    def launch(name,argv,**kw):
        handle=(run/(name+".log")).open("w",encoding="utf-8");handles.append(handle)
        process=subprocess.Popen(argv,cwd=kw.pop("cwd",run),stdout=handle,stderr=subprocess.STDOUT,
            start_new_session=True,**kw)
        processes.append(process)
        manifest.append(dict(name=name,pid=process.pid,argv=argv,log=str(run/(name+".log"))))
        save(run/"processes.json",manifest)
        print("START",name,process.pid,flush=True)
        return process
    cfg=SoloConfig(hostname=IDENTITY["hostname"],board_serial=IDENTITY["board_serial"],
        site_checked=True,payload_removed=True,positioning="external_local")
    (REPORT/"config.yaml").write_text(yaml.safe_dump(asdict(cfg)),encoding="utf-8")
    base=Path.home()/"drone_package_20260908";px4=Path(os.environ["PX4_DIR"])
    env=dict(os.environ)
    params=dict(COM_RC_IN_MODE=0,COM_RCL_EXCEPT=0,RC_MAP_KILL_SW=6,RC_MAP_FLTMODE=5,
        COM_OBL_RC_ACT=4,COM_OF_LOSS_T=.5,NAV_RCL_ACT=3,COM_RC_LOSS_T=1,
        COM_LOW_BAT_ACT=3,BAT1_N_CELLS=4,BAT_EMERGEN_THR=.05,BAT_CRIT_THR=.07,BAT_LOW_THR=.15,
        CBRK_FLIGHTTERM=0,MPC_THR_MIN=.12,MPC_THR_HOVER=.7,UXRCE_DDS_SYNCT=0,
        MPC_TKO_SPEED=.3,MPC_LAND_SPEED=.3,MPC_Z_VEL_MAX_UP=1,MPC_Z_VEL_MAX_DN=.5,
        MPC_TILTMAX_AIR=25,MPC_XY_VEL_MAX=2,COM_DISARM_LAND=2,
        GF_MAX_HOR_DIST=3,GF_MAX_VER_DIST=3,GF_ACTION=4,RTL_RETURN_ALT=2,
        EKF2_GPS_CTRL=0,EKF2_EV_CTRL=11,EKF2_BARO_CTRL=0,EKF2_MAG_TYPE=5,RC_CHAN_CNT=8,
        RC_MAP_ROLL=1,RC_MAP_PITCH=2,RC_MAP_THROTTLE=3,RC_MAP_YAW=4)
    params.update({f"RC{i}_{k}":v for i in range(1,9) for k,v in (("MIN",1000),("MAX",2000),("TRIM",1500),("DZ",10),("REV",1))})
    params.update({"COM_FLTMODE"+str(i):2 for i in range(1,7)})
    env.update({"PX4_PARAM_"+k:str(v) for k,v in params.items()})
    env.update(PX4_GZ_STANDALONE="1",PX4_SYS_AUTOSTART="4001",PX4_SIM_MODEL="gz_x500",
        PX4_GZ_MODEL_POSE="0,0,0,0,0,0",PX4_GZ_MODELS=str(px4/"Tools/simulation/gz/models"),
        PX4_GZ_WORLDS=str(px4/"Tools/simulation/gz/worlds"),PX4_GZ_FOLLOW="1")
    # Copy world to keep all outputs here and show a closer view of the single A.
    world=(base/"gz_overrides/worlds/default.sdf").read_text()
    world=world.replace("-8 -6 8 0 0.55 0.6","-3 -4 2.6 0 0.35 0.9")
    worldpath=run/"a_solo.sdf";worldpath.write_text(world,encoding="utf-8")
    node=None;link=None;gznode=None
    result=dict(gazebo=True,real_px4=True,real_hardware=False,b_present=False,
        simulation_rc=True,simulation_external_local=True,passed=False,phases=[],task_phases=[],max_height=0.,commands=[])
    try:
        launch("gazebo",["gz","sim","-r",str(worldpath)],env=env)
        agentenv=dict(env,LD_LIBRARY_PATH=":".join(str(base/p) for p in ("Micro-XRCE-DDS-Agent/build_localdeps","Micro-XRCE-DDS-Agent/build/temp_install/fastrtps-2.12/lib","Micro-XRCE-DDS-Agent/build/temp_install/fastcdr-1.1.1/lib"))+":"+env.get("LD_LIBRARY_PATH",""))
        launch("xrce",[str(base/"Micro-XRCE-DDS-Agent/build_localdeps/MicroXRCEAgent"),"udp4","-p","8888"],env=agentenv)
        launch("px4",[str(px4/"build/px4_sitl_default/bin/px4"),
            "-d","-i","0","-w",str(run),str(px4/"build/px4_sitl_default/etc")],env=env)
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO);node=Node("gazebo_a_solo_observer")
        clock=GazeboClock();health=AHealth(node,source_time=clock.now,defer_sim_delivery=True);latest={};last_task={}
        from gz.transport13 import Node as GzNode
        from gz.msgs10.pose_v_pb2 import Pose_V
        from px4_msgs.msg import VehicleOdometry
        from payload_catch.px4_iface import qos_out
        import numpy as np
        from scipy.spatial.transform import Rotation
        gznode=GzNode();gzpose=None;gzpose_at=0.;gzstamp=None
        def pose_callback(msg):
            nonlocal gzpose,gzpose_at,gzstamp
            stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
            if gzstamp is not None and stamp<=gzstamp:return
            for pose in msg.pose:
                if pose.name=="x500_0":
                    gzpose=pose;gzpose_at=time.monotonic();gzstamp=stamp;break
        if not gznode.subscribe(Pose_V,"/world/default/pose/info",pose_callback):
            raise RuntimeError("Gazebo pose subscription failed")
        evpub=node.create_publisher(VehicleOdometry,"/fmu/in/vehicle_visual_odometry",qos_out())
        ned=np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
        frd=np.diag([1.,-1.,-1.])
        def receive(msg):
            nonlocal latest
            latest=json.loads(msg.data)
            phase=latest.get("phase")
            if not result["phases"] or result["phases"][-1]!=phase:
                result["phases"].append(phase);print("A_PHASE",phase,latest.get("reason"),flush=True)
        def task(msg):
            nonlocal last_task
            last_task=json.loads(msg.data);phase=last_task.get("phase")
            if not result["task_phases"] or result["task_phases"][-1]!=phase:
                result["task_phases"].append(phase);print("TASK_PHASE",phase,flush=True)
        node.create_subscription(String,TOPIC+"state",receive,bus_qos())
        node.create_subscription(String,TOPIC+"task",task,bus_qos())
        pub=node.create_publisher(String,TOPIC+"request",bus_qos())
        from px4_msgs.msg import VehicleCommand
        from payload_catch.px4_iface import qos_in
        node.create_subscription(VehicleCommand,"/fmu/in/vehicle_command",
            lambda msg:result["commands"].append(dict(command=msg.command,param1=msg.param1,stamp=msg.timestamp)),qos_out())
        # This is SIMULATED RC: processed by PX4 rc_update, never a ROS fake telemetry publisher.
        link=mavutil.mavlink_connection("udpout:127.0.0.1:18570",source_system=254,source_component=190)
        last_send=0;last_ev_stamp=None
        def spin():
            nonlocal last_send,last_ev_stamp
            now=time.monotonic()
            if now-last_send>=.05:
                link.mav.rc_channels_override_send(1,1,1500,1500,1000,1500,1500,1000,1500,1500)
                if int(now*20)%20==0:
                    link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0,0)
                last_send=now
            if gzpose is not None and gzstamp!=last_ev_stamp and now-gzpose_at<.2:
                last_ev_stamp=gzstamp
                m=VehicleOdometry();m.timestamp=m.timestamp_sample=int(clock.now()*1e6)
                m.pose_frame=1;m.velocity_frame=1
                p=gzpose.position;q=gzpose.orientation
                m.position=[float(p.y),float(p.x),float(-p.z)]
                enu_q=Rotation.from_quat([q.x,q.y,q.z,q.w])
                quat=Rotation.from_matrix(ned@enu_q.as_matrix()@frd).as_quat()
                m.q=[float(quat[3]),float(quat[0]),float(quat[1]),float(quat[2])]
                m.velocity=[float("nan")]*3;m.angular_velocity=[float("nan")]*3
                m.position_variance=[.001]*3;m.orientation_variance=[.001]*3
                m.velocity_variance=[float("nan")]*3;m.quality=100
                evpub.publish(m)
                result["gz_max_z"]=max(result.get("gz_max_z",0.),p.z)
            rclpy.spin_once(node,timeout_sec=.01)
            while link.recv_match(blocking=False):pass
        deadline=time.monotonic()+100;last_print=0;good=None;last_errors=None
        while time.monotonic()<deadline:
            spin()
            snapshot,telemetry=health.observe()
            errors=preflight_errors(snapshot,telemetry,cfg)
            if time.monotonic()-last_print>5:
                last_print=time.monotonic()
                print("GAZEBO_PREFLIGHT",errors,flush=True)
                save(run/"observed.json",dict(errors=errors,telemetry=telemetry,
                    source_now=clock.now(),invalid=list(snapshot.invalid_data),
                    raw={k:{slot.lstrip("_"):str(getattr(m,slot)) for slot in m.__slots__}
                         for k,m in health.raw.items()}))
            if errors!=last_errors:
                print("PREFLIGHT_CHANGE",round(clock.now(),3),errors,flush=True)
                last_errors=errors
            if errors:good=None
            elif good is None:good=time.monotonic()
            elif time.monotonic()-good>3:break
            if any(p.poll() is not None for p in processes):raise RuntimeError("simulation process exited")
        else:raise RuntimeError("Gazebo PX4 preflight blocked; inspect observed.json and px4.log")
        # Query the complete real SITL parameter list while keeping RC alive.
        pre=launch("parameters",[sys.executable,str(Path(__file__).resolve()),"--params"])
        while pre.poll() is None:spin()
        if pre.returncode:raise RuntimeError("SITL live parameter read failed")
        actual=json.loads((REPORT/"live_parameters.json").read_text())
        rows=parameter_issues(actual,cfg)
        save(run/"parameter_checks.json",rows)
        errors=[r for r in rows if r["level"]=="fail"]
        if errors:raise RuntimeError("SITL parameter checks: "+str(errors))
        receipt=dict(schema=SCOPE,scope=SCOPE,passed=True,checked_at=time.time(),drone_ids=[0],
            system_id=1,battery_min=.4,rc=True,gps=cfg.gps,release_allowed=False,
            config_digest=cfg.digest,code_digest=code_digest(ROOT),identity=IDENTITY,
            params_checked=[0],param_profile="indoor",simulation_only=True,
            params_sha256=hashlib.sha256(json.dumps(actual,sort_keys=True).encode()).hexdigest())
        save(REPORT/"simulation_receipt.json",receipt)
        for role in ("supervisor","mission"):
            launch(role,[sys.executable,str(Path(__file__).resolve()),"--component",role],cwd=ROOT,env=env)
        started=False;start_at=None;origin=None;deadline=time.monotonic()+100
        print("GUI DEMO: waiting READY, then 10s viewing delay before takeoff",flush=True)
        while time.monotonic()<deadline:
            spin()
            _,telemetry=health.observe()
            if origin is None and telemetry.get("source_ok"):origin=telemetry["position"][2]
            if origin is not None:result["max_height"]=max(result["max_height"],origin-telemetry["position"][2])
            save(run/"live_state.json",dict(state=latest,task=last_task,telemetry=telemetry))
            if latest.get("phase")=="READY" and not started:
                if start_at is None:start_at=time.monotonic()+10
                if time.monotonic()>=start_at:
                    import uuid
                    data={k:latest[k] for k in ("scope","session","config_digest","code_digest")}
                    data.update(id=uuid.uuid4().hex,stamp=time.time(),action="start",confirm="A_ONLY_TAKEOFF")
                    pub.publish(String(data=json.dumps(data)));started=True
                    print("START_REQUEST_SENT",flush=True)
            if latest.get("phase")=="DONE":
                result.update(final=telemetry);break
            if latest.get("phase") in ("CANCELLED","FAILED","MANUAL","KILL"):
                raise RuntimeError("A-only stopped: "+str(latest))
            if any(p.poll() is not None for p in processes if p is not pre):raise RuntimeError("worker exited")
        else:raise RuntimeError("A mission timed out: "+str(latest))
        if (result["max_height"]<.8 or result.get("gz_max_z",0.)<.8
                or "HOVER" not in result["task_phases"]
                or latest.get("reason")!="LAND_REQUESTED"):
            raise RuntimeError("flight trajectory incomplete")
        result["passed"]=True
        print("GAZEBO_A_SOLO_SUCCESS",json.dumps(result),flush=True)
        save(run/"result.json",result)
        # Keep the landed model visible briefly. Own all processes and bounded lifetime.
        until=time.monotonic()+45
        while time.monotonic()<until:spin()
    except Exception as exc:
        result["passed"]=False
        result["error"]=str(exc)
        print("GAZEBO_ERROR",str(exc),flush=True)
        raise
    finally:
        if link:link.close()
        if node:
            try:
                node.destroy_node()
                if rclpy.ok():rclpy.shutdown()
            except Exception as exc:print("ROS_CLEANUP",exc,flush=True)
        # Gazebo may put server/GUI into different groups in our launch session.
        owned={}
        sessions={p.pid for p in processes}
        for path in Path("/proc").iterdir():
            if path.name.isdigit():
                try:
                    pid=int(path.name)
                    if os.getsid(pid) in sessions:
                        owned[pid]=(path/"stat").read_text().split(") ",1)[1].split()[19]
                except (OSError,ProcessLookupError):pass
        save(run/"owned_at_cleanup.json",owned)
        for pid in owned:
            try:os.kill(pid,signal.SIGTERM)
            except ProcessLookupError:pass
        for p in reversed(processes):
            if p.poll() is None:
                try:os.killpg(p.pid,signal.SIGTERM)
                except ProcessLookupError:pass
        for p in reversed(processes):
            try:p.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
        for pid,start in owned.items():
            path=Path("/proc")/str(pid)
            try:
                if (path/"stat").read_text().split(") ",1)[1].split()[19]==start:
                    os.kill(pid,signal.SIGKILL)
            except (OSError,ProcessLookupError):pass
        for h in handles:h.close()
        result["workers_stopped"]=all(p.poll() is not None for p in processes)
        save(run/"result.json",result)
        print("CLEANUP",result["workers_stopped"],flush=True)

def root_main():
    if os.geteuid()!=0:raise SystemExit("root needed to create netns only; use sudo -n")
    REPORT.mkdir(parents=True,exist_ok=True)
    os.chown(REPORT,ROOT.stat().st_uid,ROOT.stat().st_gid)
    user=pwd.getpwuid(ROOT.stat().st_uid).pw_name
    env=dict(os.environ,PAYLOAD_CATCH_HOST_NETNS=os.readlink("/proc/self/ns/net"),
        PAYLOAD_CATCH_TEST_USER=user,PAYLOAD_CATCH_TEST_ROOT=str(ROOT))
    logfile=REPORT/("launcher_"+time.strftime("%H%M%S")+".log")
    with logfile.open("w",encoding="utf-8") as handle:
        p=subprocess.Popen(["unshare","--net","--","/bin/bash",str(ROOT/"tools/run_a_solo_gazebo.sh")],
            cwd=ROOT,env=env,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
        print("GAZEBO_RUNNER_PID",p.pid,"LOG",logfile,flush=True)
        try:code=p.wait(timeout=420)
        except BaseException:
            # Do not kill arbitrary processes; this group belongs to this launcher.
            os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=15);raise
        finally:os.chown(logfile,ROOT.stat().st_uid,ROOT.stat().st_gid)
    print(logfile.read_text(),flush=True)
    return code

if __name__=="__main__":
    if "--component" in sys.argv:component()
    elif "--params" in sys.argv:
        from payload_catch.solo_validation_node import test_isolation
        from tools.a_solo_preflight import live_parameters
        test_isolation()
        save(REPORT/"live_parameters.json",live_parameters("udpout:127.0.0.1:14580"))
    elif "--worker" in sys.argv:worker()
    else:raise SystemExit(root_main())
