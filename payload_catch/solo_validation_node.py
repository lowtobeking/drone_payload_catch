"""A-only ROS adapter. Separate mission and command-supervisor processes."""
from __future__ import annotations
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import (VehicleStatus, VehicleLocalPosition, VehicleAttitude,
    BatteryStatus, FailsafeFlags, EstimatorStatusFlags, SensorGps,
    ManualControlSetpoint, VehicleLandDetected, VehicleCommand,
    VehicleCommandAck, OffboardControlMode, TrajectorySetpoint)
from .preflight_health import HealthAccumulator, MAX_AGE
from .safety_runtime import SourceWatch, flight_reasons
from .px4_iface import qos_in, qos_out
from .solo_validation import (SCOPE, ACTIVE, SoloSupervisor, SoloMission,
    load_config, code_digest, read_receipt, receipt_errors, hardware_errors,
    preflight_errors)

KINDS = dict(status=(VehicleStatus, "vehicle_status_v1"),
    lpos=(VehicleLocalPosition, "vehicle_local_position"),
    att=(VehicleAttitude, "vehicle_attitude"), batt=(BatteryStatus, "battery_status"),
    failsafe=(FailsafeFlags, "failsafe_flags"), est=(EstimatorStatusFlags, "estimator_status_flags"),
    gps=(SensorGps, "vehicle_gps_position"), rc=(ManualControlSetpoint, "manual_control_setpoint"),
    land=(VehicleLandDetected, "vehicle_land_detected"))
TOPIC = "/a_validation/"
CONTROL = dict(vehicle_command="a_solo_supervisor", offboard_control_mode="a_solo_mission",
               trajectory_setpoint="a_solo_mission")
OTHER_CONTROL = ("vehicle_attitude_setpoint", "vehicle_rates_setpoint",
                 "actuator_motors", "actuator_servos", "manual_control_input")


def bus_qos():
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


def local_identity():
    serial = Path("/proc/device-tree/serial-number")
    return dict(hostname=socket.gethostname(),
        board_serial=serial.read_bytes().rstrip(b"\0").decode() if serial.exists() else "",
        boot_id=Path("/proc/sys/kernel/random/boot_id").read_text().strip())


def check_runtime(root, profile):
    """Read-only existing host check, never changes services or PX4 parameters."""
    if not profile:
        return ["host_profile not supplied"]
    result = subprocess.run(["sudo", "-n", "/usr/bin/env",
        "ROS_DOMAIN_ID="+os.environ.get("ROS_DOMAIN_ID", "0"),
        "RMW_IMPLEMENTATION="+os.environ.get("RMW_IMPLEMENTATION", ""),
        "ROS_LOCALHOST_ONLY="+os.environ.get("ROS_LOCALHOST_ONLY", "0"),
        "/usr/bin/python3", str(Path(root)/"tools/a_solo_runtime.py"), "--profile", profile],
        cwd=root, timeout=25, capture_output=True, text=True, encoding="utf-8", errors="replace")
    return [] if result.returncode == 0 else [
        "host runtime check failed: " + result.stdout.strip() + " " + result.stderr.strip()]


def test_isolation():
    """Test injection has no ROS/CLI bypass and requires no path to real hardware."""
    if (os.geteuid() == 0 or os.environ.get("ROS_LOCALHOST_ONLY") != "1"
            or not os.environ.get("PAYLOAD_CATCH_HOST_NETNS")
            or os.readlink("/proc/self/ns/net") == os.environ["PAYLOAD_CATCH_HOST_NETNS"]
            or {name for _, name in socket.if_nameindex()} != {"lo"}):
        raise RuntimeError("mock context requires unprivileged loopback-only separate netns")


class AHealth:
    def __init__(self, node, *, source_time=None, defer_sim_delivery=False):
        self.source_time = source_time or time.time
        self.debug_node = node
        self.pending_sim = None
        if defer_sim_delivery:
            test_isolation()
            self.pending_sim = deque(maxlen=256)
        self.acc = HealthAccumulator()
        self.raw = {}
        # PX4 1.16 publishes land state on change, and a 1 Hz heartbeat.
        # A 0.5 s timeout would reject a healthy detector every other half second.
        self.watch = {k: SourceWatch(1.5 if k == "land" else MAX_AGE[k]) for k in KINDS}
        self.subscriptions = [node.create_subscription(cls, "/fmu/out/"+suffix,
            lambda m, k=k: self.receive(k, m), qos_in()) for k, (cls, suffix) in KINDS.items()]

    def receive(self, key, msg, *, queued=False):
        wall, mono = self.source_time(), time.monotonic()
        # Gazebo clock and DDS telemetry use different transports. A message
        # can arrive before the corresponding clock tick. In isolated SITL,
        # wait for that tick; never refresh health with a future timestamp.
        if (not queued and self.pending_sim is not None
                and 0 < msg.timestamp/1e6-wall <= .1):
            self.pending_sim.append((mono,key,msg))
            return
        accepted = self.watch[key].accept(msg.timestamp/1e6, (), wall, mono)
        if accepted:
            self.raw[key] = msg
            if key in self.acc.samples:
                self.acc.record(key, msg, mono)
        elif not self.watch[key].valid and key in self.acc.samples:
            if self.pending_sim is not None:
                self.debug_node.get_logger().warning(
                    f"SIM_SOURCE_REJECT {key} stamp={msg.timestamp/1e6:.6f} now={wall:.6f} queued={queued}")
            # Invalid source timestamps must poison the observation window.
            from types import SimpleNamespace
            self.acc.record(key, SimpleNamespace(timestamp=0), mono)

    def observe(self):
        wall, mono = self.source_time(), time.monotonic()
        if self.pending_sim is not None:
            waiting = list(self.pending_sim)
            self.pending_sim.clear()
            for at,key,msg in waiting:
                if msg.timestamp/1e6 <= wall or mono-at >= .1:
                    self.receive(key,msg,queued=True)
                else:
                    self.pending_sim.append((at,key,msg))
            # Delivery may have advanced while draining the queue.
            wall, mono = self.source_time(), time.monotonic()
        fresh = {k: w.fresh(wall, mono) for k, w in self.watch.items()}
        status, land = self.raw.get("status"), self.raw.get("land")
        p, att, batt = self.raw.get("lpos"), self.raw.get("att"), self.raw.get("batt")
        get = lambda obj, key, default: getattr(obj, key, default)
        q = list(get(att, "q", [math.nan]*4))
        tilt = math.inf
        if len(q) == 4 and all(math.isfinite(v) for v in q):
            tilt = math.degrees(math.acos(max(-1., min(1., 1-2*(q[1]**2+q[2]**2)))))
        snapshot = self.acc.snapshot(mono)
        telemetry = dict(status_fresh=fresh["status"], land_fresh=fresh["land"],
            identity_ok=fresh["status"] and get(status, "system_id", -1) == 1
                and get(status, "component_id", -1) == 1,
            source_ok=all(fresh[k] for k in ("status","lpos","att","batt","failsafe","est","rc","land")),
            armed=get(status, "arming_state", -1) == 2,
            nav=get(status, "nav_state", -1), landed=get(land, "landed", False),
            position=[get(p, k, math.nan) for k in ("x","y","z")],
            velocity=[get(p, k, math.nan) for k in ("vx","vy","vz")],
            heading=get(p, "heading", math.nan), tilt=tilt,
            reset_counters=[get(p, k, -1) for k in
                ("xy_reset_counter","z_reset_counter","vxy_reset_counter","vz_reset_counter","heading_reset_counter")],
            cells=get(batt, "cell_count", -1), voltage=get(batt, "voltage_v", math.nan))
        if not math.isfinite(telemetry["heading"]) or -1 in telemetry["reset_counters"]:
            telemetry["source_ok"] = False
        return snapshot, telemetry


class SoloNode(Node):
    def __init__(self, role, *, test_context=None):
        super().__init__("a_solo_"+role)
        self.role = role
        for name, default in (("config", ""), ("receipt", ""), ("source_root", ""),
                              ("host_profile", ""), ("execute", False)):
            self.declare_parameter(name, default)
        param = lambda key: self.get_parameter(key).value
        self.cfg = load_config(param("config"))
        self.execute = param("execute")
        self.root = Path(param("source_root")).resolve()
        self.identity = local_identity()
        self.fingerprint = code_digest(self.root)
        self.receipt_path = param("receipt")
        self.runtime_errors = []
        if test_context is not None:
            test_isolation()
            self.identity = test_context["identity"]
        else:
            # Executing copied/stale installed code with a new source receipt is forbidden.
            for name in ("solo_validation", "solo_validation_node", "preflight_health", "safety_runtime", "px4_iface"):
                module = sys.modules["payload_catch."+name]
                if Path(module.__file__).read_bytes() != (self.root/"payload_catch"/(name+".py")).read_bytes():
                    self.runtime_errors.append("installed/source code mismatch: "+name)
            if self.execute:
                try:
                    self.runtime_errors += check_runtime(self.root, param("host_profile"))
                except (OSError, subprocess.SubprocessError) as exc:
                    self.runtime_errors.append("host runtime check error: "+str(exc))
        self.wall_time = time.time
        self.source_time = test_context.get("source_time", time.time) if test_context else time.time
        self.health = AHealth(self, source_time=self.source_time,
            defer_sim_delivery=test_context is not None and "source_time" in test_context)
        self.peer_watch = SourceWatch(.5)
        self.peer_seq = -1
        self.peer_session = None
        self.peer = {}
        self.stream_at = None
        self.session = uuid.uuid4().hex
        self.core = SoloSupervisor(self.cfg, execute=self.execute) if role == "supervisor" else SoloMission(self.cfg)
        self.state_pub = self.create_publisher(String, TOPIC+("state" if role == "supervisor" else "task"), bus_qos())
        self.request_pub = self.create_publisher(String, TOPIC+"request", bus_qos())
        self.create_subscription(String, TOPIC+("task" if role == "supervisor" else "state"),
                                 self.receive_peer, bus_qos())
        self.request_ids = set()
        self.ack_watch = SourceWatch(.5)
        self.graph_errors = ["control publisher graph not observed"]
        self.last_graph = -math.inf
        self.counter = 0
        self.last_phase = None
        self.request_counter = 0
        if role == "supervisor":
            self.command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_out())
            self.create_subscription(String, TOPIC+"request", self.receive_request, bus_qos())
            self.create_subscription(VehicleCommandAck, "/fmu/out/vehicle_command_ack", self.receive_ack, qos_in())
        else:
            self.mode_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_out())
            self.trajectory_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_out())
        self.timer = self.create_timer(.05, self.tick)

    def envelope_ok(self, data):
        return (isinstance(data, dict) and data.get("scope") == SCOPE
            and data.get("config_digest") == self.cfg.digest
            and data.get("code_digest") == self.fingerprint)

    def receive_peer(self, msg):
        try:
            data = json.loads(msg.data)
            if not self.envelope_ok(data) or type(data.get("seq")) is not int:
                return
            session = data.get("session")
            if self.role == "supervisor":
                if session != self.session:
                    return
            elif self.peer_session is not None and session != self.peer_session:
                # A supervisor restart never resumes a previous arm attempt.
                self.core.stop_latched = True
                return
            if data["seq"] <= self.peer_seq:
                return
            if not self.peer_watch.accept(data.get("stamp"), (), self.wall_time(), time.monotonic()):
                return
            self.peer_seq = data["seq"]
            self.peer_session, self.peer = session, data
        except (ValueError, TypeError, OverflowError):
            return

    def static_errors(self, wall):
        return (self.cfg.site_errors(self.identity) + self.runtime_errors + self.graph_errors
            + receipt_errors(read_receipt(self.receipt_path), self.cfg, self.identity,
                             self.fingerprint, wall))

    def graph_check(self):
        errors = []
        for suffix, owner in CONTROL.items():
            infos = self.get_publishers_info_by_topic("/fmu/in/"+suffix)
            if len(infos) != 1 or infos[0].node_name != owner or infos[0].node_namespace != "/":
                errors.append("missing/foreign/duplicate control publisher: "+suffix)
        for suffix in OTHER_CONTROL:
            if self.get_publishers_info_by_topic("/fmu/in/"+suffix):
                errors.append("competing control publisher: "+suffix)
        self.graph_errors = errors

    def receive_request(self, msg):
        try:
            data = json.loads(msg.data)
            if (not self.envelope_ok(data) or data.get("session") != self.session
                    or data.get("id") in self.request_ids
                    or not isinstance(data.get("id"), str)
                    or len(data["id"]) != 32
                    or type(data.get("stamp")) not in (float, int)
                    or not 0 <= self.wall_time()-data["stamp"] <= .5):
                return
            self.request_ids.add(data["id"])
            # No bounded eviction/replay of old requests within this short session.
            action, now = data.get("action"), time.monotonic()
            if action == "start" and data.get("confirm") == "A_ONLY_TAKEOFF":
                self.graph_check()
                snapshot, telemetry = self.health.observe()
                errors = self.static_errors(self.wall_time()) + preflight_errors(snapshot, telemetry, self.cfg)
                if not self.peer_watch.fresh(self.wall_time(), now):
                    errors.append("task lease expired")
                self.core.start(now, telemetry, errors)
            elif action in ("stop", "land"):
                self.core.stop(str(data.get("reason", "operator requested landing"))[:200], now)
            elif action == "kill" and data.get("confirm") == "CUT_MOTOR_POWER":
                self.core.stop("explicit operator motor power cut", now, kill=True)
        except (ValueError, TypeError, OverflowError):
            return

    def receive_ack(self, msg):
        ticket = self.core.ticket
        if ticket is None or ticket.attempts < 1:
            return
        now = time.monotonic()
        if (not self.ack_watch.accept(msg.timestamp/1e6, (), self.source_time(), now)
                or msg.timestamp/1e6 < self.source_time()-(now-ticket.created)-.05):
            return
        self.core.acknowledge(msg.command, msg.result, msg.target_system, msg.target_component)

    def request(self, action, reason):
        if self.peer_session is None:
            return
        self.request_counter += 1
        data = dict(scope=SCOPE, config_digest=self.cfg.digest, code_digest=self.fingerprint,
            session=self.peer_session, id=uuid.uuid4().hex, stamp=self.wall_time(),
            action=action, reason=reason)
        self.request_pub.publish(String(data=json.dumps(data)))

    def send_command(self, ticket, now):
        msg = VehicleCommand()
        msg.timestamp = int(self.source_time()*1e6)
        msg.command = ticket.command
        msg.target_system, msg.target_component = 1, 1
        msg.source_system, msg.source_component = 255, 191
        msg.from_external = True
        if ticket.action == "offboard":
            msg.param1, msg.param2 = 1., 6.
        elif ticket.action in ("arm", "kill"):
            msg.param1 = 1.
        self.command_pub.publish(msg)
        ticket.sent(now)

    def send_setpoint(self, velocity, yaw):
        stamp = int(self.source_time()*1e6)
        mode = OffboardControlMode()
        mode.timestamp, mode.velocity = stamp, True
        target = TrajectorySetpoint()
        target.timestamp = stamp
        target.position = [math.nan]*3
        target.velocity = [float(x) for x in velocity]
        target.acceleration = [math.nan]*3
        target.jerk = [math.nan]*3
        target.yaw, target.yawspeed = float(yaw), math.nan
        self.trajectory_pub.publish(target)
        self.mode_pub.publish(mode)

    def tick(self):
        now, wall = time.monotonic(), self.wall_time()
        if now-self.last_graph > .25:
            self.graph_check()
            self.last_graph = now
        snapshot, telemetry = self.health.observe()
        peer_fresh = self.peer_watch.fresh(wall, now)
        if self.role == "supervisor":
            ticket = self.core.tick(now, snapshot, telemetry, before_errors=self.static_errors(wall),
                task_ok=peer_fresh and self.peer.get("healthy") is True and not self.graph_errors,
                streamed=peer_fresh and self.peer.get("streamed") is True)
            if ticket:
                self.send_command(ticket, now)
            data = self.core.state(self.session, wall)
            data.update(config_digest=self.cfg.digest, code_digest=self.fingerprint)
            phase = self.core.phase
        else:
            phase = self.peer.get("phase", "CHECK") if peer_fresh else "STALE"
            health_errors = (flight_reasons(snapshot, require_rc=True)
                             + hardware_errors(telemetry, self.cfg) + self.graph_errors)
            healthy = not health_errors
            streamed = False
            if self.execute and peer_fresh and self.peer.get("permit") is True and phase in ACTIVE:
                errors = list(health_errors)
                if phase in ("ARM","FLY") and telemetry["status_fresh"] and telemetry["nav"] != 14:
                    errors.append("PX4/RC mode takeover")
                if phase != "FLY":
                    errors += self.static_errors(wall)
                    errors += preflight_errors(snapshot, telemetry, self.cfg,
                        ground=phase != "ARM", checks=not (phase == "ARM" and telemetry["armed"]))
                if errors:
                    self.core.stop_latched = True
                    self.request("land", "; ".join(errors))
                else:
                    velocity, land = self.core.velocity(now, phase, self.peer["origin"],
                        telemetry["position"], telemetry["velocity"])
                    if land:
                        self.request("land", self.core.last_phase)
                    elif velocity is not None:
                        self.send_setpoint(velocity, self.peer["yaw"])
                        streamed = True
            elif self.execute and not peer_fresh and self.core.last_phase in ("PRESTREAM","TAKEOFF","HOVER"):
                self.core.stop_latched = True
                self.request("land", "supervisor state lease expired")
            self.counter += 1
            data = dict(scope=SCOPE, config_digest=self.cfg.digest, code_digest=self.fingerprint,
                session=self.peer_session, seq=self.counter, stamp=wall,
                healthy=healthy and not self.core.stop_latched, streamed=streamed,
                phase=self.core.last_phase, health_errors=health_errors, release_allowed=False)
        self.state_pub.publish(String(data=json.dumps(data, allow_nan=False)))
        if phase != self.last_phase:
            self.get_logger().info("A_SOLO "+phase+" "+(self.core.reason if self.role=="supervisor" else ""))
            self.last_phase = phase


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("role", choices=["supervisor", "mission"])
    args, ros_args = parser.parse_known_args()
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = SoloNode(args.role)
    stopping = None
    def stop(*_):
        nonlocal stopping
        if stopping is None:
            stopping = time.monotonic()
            if args.role == "supervisor":
                node.core.stop("process shutdown requested", stopping)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
            if stopping is not None:
                if (args.role != "supervisor" or node.core.phase not in ("LAND", "KILL")
                        or time.monotonic()-stopping > 32):
                    break
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
