"""A-only, no-payload flight acceptance. No ROS, B state or release authority."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
import time
from pathlib import Path

from .preflight_health import BATTERY_MIN, evaluate
from .safety_runtime import CommandTicket, flight_reasons

SCOPE = "a_solo_no_payload_v1"
ACTIVE = {"PRESTREAM", "MODE", "ARM", "FLY"}
TERMINAL = {"DONE", "CANCELLED", "MANUAL", "KILL", "FAILED"}
CODE_FILES = ("payload_catch/solo_validation.py", "payload_catch/solo_validation_node.py",
              "payload_catch/preflight_health.py", "payload_catch/safety_runtime.py",
              "payload_catch/fcu_params.py", "payload_catch/px4_iface.py",
              "payload_catch/host_runtime.py", "tools/a_solo_preflight.py",
              "tools/preflight_params.py", "launch/a_solo_validation_launch.py",
              "deploy/run_a_solo.sh", "tools/a_solo_control.py", "tools/jetson_runtime.py", "tools/a_solo_runtime.py")


@dataclass(frozen=True)
class SoloConfig:
    scope: str = SCOPE
    hostname: str = "FILL_ME"
    board_serial: str = "FILL_ME"
    site_checked: bool = False
    payload_removed: bool = False
    expected_cells: int = 4
    positioning: str = "outdoor_gps"
    height_m: float = 1.0
    hover_s: float = 10.0
    climb_speed: float = 0.3
    xy_speed: float = 0.2
    radius_m: float = 2.0
    max_height_m: float = 2.0
    max_tilt_deg: float = 25.0
    takeoff_timeout_s: float = 20.0
    max_flight_s: float = 60.0

    def validate(self):
        if self.scope != SCOPE or self.positioning not in ("outdoor_gps", "external_local"):
            raise ValueError("wrong single-aircraft scope / positioning")
        if type(self.site_checked) is not bool or type(self.payload_removed) is not bool:
            raise ValueError("site_checked/payload_removed must be explicit booleans")
        if type(self.expected_cells) is not int or not 2 <= self.expected_cells <= 12:
            raise ValueError("expected_cells must describe actual battery")
        limits = dict(height_m=(0.5, 2.0), hover_s=(2, 30), climb_speed=(0.1, 0.5),
                      xy_speed=(0.05, 0.5), radius_m=(0.5, 3),
                      max_height_m=(0.8, 3), max_tilt_deg=(10, 35),
                      takeoff_timeout_s=(10, 40), max_flight_s=(30, 120))
        for key, (lo, hi) in limits.items():
            val = getattr(self, key)
            if type(val) not in (float, int) or not math.isfinite(val) or not lo <= val <= hi:
                raise ValueError(f"{key} outside acceptance range")
        if self.max_height_m < self.height_m + 0.3:
            raise ValueError("height ceiling must leave margin above target")
        if self.max_flight_s < self.takeoff_timeout_s + self.hover_s + 5:
            raise ValueError("max_flight_s too short")
        return self

    @property
    def gps(self):
        return self.positioning == "outdoor_gps"

    @property
    def digest(self):
        return hashlib.sha256(json.dumps(asdict(self), sort_keys=True).encode()).hexdigest()

    def site_errors(self, identity):
        errors = []
        if not self.site_checked or not self.payload_removed:
            errors.append("site inspection and unloaded mechanism must be confirmed")
        for key in ("hostname", "board_serial"):
            if getattr(self, key) in ("", "FILL_ME") or getattr(self, key) != identity.get(key):
                errors.append(key + " mismatch / not configured")
        if not identity.get("boot_id"):
            errors.append("missing boot identity")
        return errors


def load_config(path):
    import yaml
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("solo config must be a mapping")
    return SoloConfig(**raw).validate()


def code_digest(root):
    h = hashlib.sha256()
    for name in CODE_FILES:
        h.update(name.encode() + b"\0" + (Path(root) / name).read_bytes() + b"\0")
    return h.hexdigest()


def receipt_errors(d, cfg, identity, fingerprint, wall):
    if not isinstance(d, dict):
        return ["missing A-only preflight receipt"]
    expected = dict(schema=SCOPE, scope=SCOPE, passed=True, drone_ids=[0],
                    system_id=1, battery_min=BATTERY_MIN, rc=True, gps=cfg.gps,
                    release_allowed=False, config_digest=cfg.digest,
                    code_digest=fingerprint, identity=identity,
                    params_checked=[0], param_profile="outdoor" if cfg.gps else "indoor")
    out = [key + " mismatch" for key, val in expected.items() if d.get(key) != val]
    at = d.get("checked_at")
    if type(at) not in (int, float) or not math.isfinite(at) or not 0 <= wall-at <= 60:
        out.append("preflight expired / invalid time")
    if (not isinstance(d.get("params_sha256"), str) or len(d["params_sha256"]) != 64
            or any(c not in "0123456789abcdef" for c in d["params_sha256"])):
        out.append("missing live parameter fingerprint")
    return out


def read_receipt(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return None


def hardware_errors(telemetry, cfg):
    """Extra A identity and raw battery consistency beyond common health rules."""
    out = []
    if not telemetry.get("source_ok", False):
        out.append("PX4 source timestamp stale / invalid")
    if not telemetry.get("identity_ok"):
        out.append("A SYSID1/component1 not freshly verified")
    if not telemetry.get("land_fresh"):
        out.append("land detector missing/stale")
    voltage = telemetry.get("voltage")
    if (type(voltage) not in (int, float) or not math.isfinite(voltage)
            or not 3.0 * cfg.expected_cells <= voltage <= 4.3 * cfg.expected_cells):
        out.append("battery voltage inconsistent")
    return out


def preflight_errors(snapshot, telemetry, cfg, *, ground=True, checks=True):
    out = [r["name"] for r in evaluate(snapshot, require_gps=cfg.gps, require_rc=True)
           if r["status"] == "fail" and (checks or r["name"] != "PX4 pre_flight_checks_pass")]
    out += hardware_errors(telemetry, cfg)
    if ground and (telemetry.get("armed") or not telemetry.get("landed")):
        out.append("must start landed and disarmed")
    return out


def parameter_issues(parameters, cfg):
    """Shared strict PX4 checks plus the limits used in this vertical acceptance."""
    from .fcu_params import check_params
    rows = [x.__dict__ for x in check_params(parameters,
        "outdoor" if cfg.gps else "indoor", require_complete=True)]
    exact = dict(MAV_SYS_ID=1, CBRK_FLIGHTTERM=0)
    for name, expected in exact.items():
        actual = parameters.get(name)
        rows.append(dict(name=name, level="ok" if actual == expected else "fail",
                         detail=f"expected {expected}, got {actual}"))
    bounds = dict(MPC_THR_MIN=(.05,.5), MPC_THR_HOVER=(.1,.7),
        MPC_TKO_SPEED=(.1,.5), MPC_LAND_SPEED=(.1,.5),
        MPC_Z_VEL_MAX_UP=(cfg.climb_speed,1.5), MPC_Z_VEL_MAX_DN=(.1,1.),
        MPC_TILTMAX_AIR=(10.,cfg.max_tilt_deg), COM_DISARM_LAND=(.1,5.))
    for name, (lo,hi) in bounds.items():
        actual = parameters.get(name)
        valid = type(actual) in (int,float) and math.isfinite(actual) and lo <= actual <= hi
        rows.append(dict(name=name, level="ok" if valid else "fail",
                         detail=f"acceptance range [{lo}, {hi}], got {actual}"))
    return rows


class SoloSupervisor:
    """Independent monotonic deadlines, single attempt, observed ACK/mode/landing."""
    def __init__(self, cfg, *, execute=False):
        self.cfg = cfg.validate()
        self.execute = execute
        self.phase = "CHECK"
        self.reason = "waiting for A-only preflight"
        self.good_since = None
        self.origin = None
        self.yaw = None
        self.started = None
        self.ticket = None
        self.stream_since = None
        self.armed_seen = False
        self.arm_attempted = False
        self.land_since = None
        self.land_confirm_since = None
        self.start_consumed = False
        self.reset_counters = None
        self.progress = ""
        self.seq = 0

    @property
    def permit(self):
        return self.execute and self.phase in ACTIVE

    def stop(self, reason, now, *, kill=False):
        self.reason = reason
        if self.phase in ("DONE", "KILL"):
            return
        if kill and self.execute:
            self.phase = "KILL"
            self.ticket = CommandTicket("kill", 185, 13, now)
        elif self.phase == "MANUAL":
            return
        elif self.execute and (self.armed_seen or self.arm_attempted):
            if self.phase != "LAND":
                self.phase = "LAND"
                self.land_since = now
                self.ticket = CommandTicket("land", 21, 18, now)
        else:
            self.phase = "CANCELLED"
            self.ticket = None

    def start(self, now, telemetry, errors):
        if (not self.execute or self.phase != "READY" or self.start_consumed
                or errors or not telemetry.get("landed") or telemetry.get("armed")):
            return False
        self.start_consumed = True
        self.phase = "PRESTREAM"
        self.started = now
        self.origin = list(telemetry["position"])
        self.yaw = telemetry["heading"]
        self.reset_counters = telemetry.get("reset_counters")
        self.reason = "operator start accepted; zero setpoint prestream"
        return True

    def acknowledge(self, command, result, target_system, target_component):
        if self.ticket:
            return self.ticket.acknowledge(command, result, target_system, target_component)
        return False

    def tick(self, now, snapshot, telemetry, *, before_errors, task_ok, streamed=False):
        armed = telemetry.get("armed", False)
        nav = telemetry.get("nav", -1)
        self.armed_seen |= armed and self.phase in ACTIVE | {"LAND"}
        if self.phase in ("CHECK", "READY"):
            errors = list(before_errors) + preflight_errors(snapshot, telemetry, self.cfg)
            if not task_ok:
                errors.append("task heartbeat missing / invalid")
            if errors:
                self.good_since = None
                self.phase = "CHECK"
                self.reason = "; ".join(errors)
            else:
                if self.good_since is None:
                    self.good_since = now
                self.phase = "READY" if now-self.good_since >= 3 else "CHECK"
                self.reason = "ready; explicit start required" if self.phase == "READY" else "healthy window"
            return None

        if self.phase in ACTIVE:
            # Once PX4/pilot leaves our observed Offboard, never fight mode takeover.
            if self.phase in ("ARM", "FLY") and telemetry.get("status_fresh") and nav != 14:
                self.phase = "MANUAL"
                self.reason = "PX4/RC mode takeover; no automatic mode retries"
                self.ticket = None
                return None
            errors = flight_reasons(snapshot, require_rc=True) + hardware_errors(telemetry, self.cfg)
            if not task_ok:
                errors.append("task lease expired")
            if self.phase != "FLY":
                errors += before_errors
                errors += preflight_errors(snapshot, telemetry, self.cfg,
                                           ground=self.phase != "ARM",
                                           checks=not (self.phase == "ARM" and armed))
            if self.origin is not None:
                p = telemetry.get("position", [])
                if len(p) != 3 or not all(math.isfinite(x) for x in p):
                    errors.append("nonfinite position")
                elif (math.hypot(p[0]-self.origin[0], p[1]-self.origin[1]) > self.cfg.radius_m
                      or self.origin[2]-p[2] > self.cfg.max_height_m
                      or p[2]-self.origin[2] > 0.3):
                    errors.append("local acceptance boundary exceeded")
                if telemetry.get("reset_counters") != self.reset_counters:
                    errors.append("estimator coordinate reset")
                if telemetry.get("tilt", math.inf) > self.cfg.max_tilt_deg:
                    errors.append("tilt acceptance limit exceeded")
            if now-self.started > self.cfg.max_flight_s:
                errors.append("absolute mission timeout")
            if errors:
                self.stop("; ".join(dict.fromkeys(errors)), now)

        if self.phase == "PRESTREAM":
            if not streamed:
                self.stream_since = None
            elif self.stream_since is None:
                self.stream_since = now
            if self.stream_since is not None and now-self.stream_since >= 2:
                self.phase = "MODE"
                self.ticket = CommandTicket("offboard", 176, 14, now)
            if now-self.started > 10:
                self.stop("prestream timeout", now)
        if self.phase in ("MODE", "ARM") and self.ticket:
            self.progress = self.ticket.update(now, status_fresh=telemetry.get("status_fresh", False),
                                               nav_state=nav, armed=armed)
            if self.ticket.failure:
                self.stop(self.ticket.failure, now)
            elif self.phase == "MODE" and self.progress == "MODE_CONFIRMED":
                self.phase = "ARM"
                self.ticket = CommandTicket("arm", 400, 14, now)
            elif self.phase == "ARM" and self.progress == "MODE_CONFIRMED" and armed:
                self.phase = "FLY"
                self.ticket = None
                self.reason = "armed + Offboard confirmed"
            elif now-self.started > 12:
                self.stop("arming / mode confirmation timeout", now)
        if self.phase == "LAND":
            if (telemetry.get("status_fresh") and telemetry.get("land_fresh")
                    and telemetry.get("landed") and not armed):
                if self.land_confirm_since is None:
                    self.land_confirm_since = now
                if now-self.land_confirm_since >= 1:
                    # Normal result requires command receipt/mode, never timer-only.
                    if self.ticket and self.ticket.ack_result == 0 and self.ticket.mode_confirmed:
                        self.phase = "DONE"
                        self.progress = "LANDED_DISARMED_CONFIRMED"
                        self.ticket = None
            else:
                self.land_confirm_since = None
            if self.phase == "LAND" and now-self.land_since > 30:
                self.phase = "FAILED"
                self.reason = "landing not confirmed; PX4/RC intervention required"
                self.ticket = None
        if self.phase in ("LAND", "KILL") and self.ticket:
            self.progress = self.ticket.update(now, status_fresh=telemetry.get("status_fresh", False),
                                               nav_state=nav, armed=armed,
                                               landed=telemetry.get("landed", False))
            # A deliberate pilot-controlled mode cancels LAND retries.
            if self.phase == "LAND" and telemetry.get("status_fresh") and armed and nav in (0,1,2,3,5,10,13,15):
                self.phase = "MANUAL"
                self.reason = "RC/PX4 takeover during landing"
                self.ticket = None
        if self.execute and self.ticket and self.ticket.due(now):
            if self.phase == "ARM":
                self.arm_attempted = True
            return self.ticket
        return None

    def state(self, session, wall):
        self.seq += 1
        return dict(scope=SCOPE, session=session, seq=self.seq, stamp=wall,
                    phase=self.phase, execute=self.execute, permit=self.permit, origin=self.origin, yaw=self.yaw,
                    reason=self.reason, progress=self.progress, release_allowed=False)


class SoloMission:
    """Take off vertically, settle, hover, request PX4 LAND; no release interface."""
    def __init__(self, cfg):
        self.cfg = cfg
        self.flight_started = None
        self.settled_since = None
        self.hover_since = None
        self.last_phase = "WAIT"
        self.stop_latched = False

    def velocity(self, now, phase, origin, position, velocity):
        if self.stop_latched:
            return None, False
        if phase in ("PRESTREAM", "MODE", "ARM"):
            self.last_phase = "PRESTREAM"
            return [0.0, 0.0, 0.0], False
        if phase != "FLY":
            self.last_phase = phase
            return None, False
        if self.flight_started is None:
            self.flight_started = now
        target = [origin[0], origin[1], origin[2]-self.cfg.height_m]
        error = [a-b for a,b in zip(target, position)]
        if not all(math.isfinite(x) for x in error + list(velocity)):
            self.stop_latched = True
            return None, True
        xy = error[:2]
        scale = min(1.0, self.cfg.xy_speed/max(math.hypot(*xy), 1e-9))
        command = [xy[0]*scale, xy[1]*scale,
                   max(-self.cfg.climb_speed, min(self.cfg.climb_speed, error[2]))]
        stable = math.hypot(*error[:2]) <= .15 and abs(error[2]) <= .15 and math.sqrt(sum(v*v for v in velocity)) <= .15
        if not stable:
            self.settled_since = None
            # Hover means continuously stable, not cumulative visits to target.
            self.hover_since = None
        elif self.settled_since is None:
            self.settled_since = now
        elif now-self.settled_since >= 2 and self.hover_since is None:
            self.hover_since = now
        self.last_phase = "HOVER" if self.hover_since is not None else "TAKEOFF"
        done = self.hover_since is not None and now-self.hover_since >= self.cfg.hover_s
        timeout = self.hover_since is None and now-self.flight_started > self.cfg.takeoff_timeout_s
        if done or timeout:
            self.stop_latched = True
            self.last_phase = "LAND_REQUESTED" if done else "TAKEOFF_TIMEOUT"
            return None, True
        return command, False
