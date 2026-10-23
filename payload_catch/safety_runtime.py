"""ROS-free runtime safety: source freshness, flight health, command confirmation."""
from __future__ import annotations
import math
from dataclasses import dataclass
from .preflight_health import MAX_AGE, FAILSAFE_FATAL, EST_BAD

LEASE_S = 0.5
SOURCE_S = 0.5
FAULT_LAND_S = 3.0
CMD_RETRY_S = 0.3
CMD_ATTEMPTS = 8


class SourceWatch:
    def __init__(self, timeout=SOURCE_S):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('source timeout must be positive')
        self.timeout = timeout
        self.stamp = None
        self.received = None
        self.valid = False

    def accept(self, stamp, values, source_now, monotonic_now):
        try:
            if math.isfinite(stamp) and self.stamp is not None and stamp<=self.stamp:
                return False # duplicate/reordered data must not refresh freshness
            good = (math.isfinite(stamp) and 0 <= source_now - stamp <= self.timeout
                    and all(math.isfinite(float(v)) for v in values)
                    and (self.stamp is None or stamp > self.stamp))
        except (TypeError, ValueError, OverflowError):
            good = False
        if not good:
            self.valid = False
            return False
        self.stamp, self.received, self.valid = stamp, monotonic_now, True
        return True

    def fresh(self, source_now, monotonic_now):
        return (self.valid and self.received is not None and self.stamp is not None
                and 0 <= monotonic_now - self.received <= self.timeout
                and 0 <= source_now - self.stamp <= self.timeout)


def flight_reasons(s, *, require_rc=False):
    """40% and pre_flight_checks_pass are deliberately not inflight conditions."""
    out = []
    for key in ('lpos','att','status','failsafe','est','batt'):
        age = s.ages.get(key, math.inf)
        if getattr(s,'n_'+key) < 1 or not math.isfinite(age) or not 0 <= age <= MAX_AGE[key]:
            out.append(key + '_stale_or_missing')
    if s.invalid_data: out.append('invalid_data')
    if not s.att_valid: out.append('attitude_invalid')
    if not (s.xy_valid_frac == s.z_valid_frac == s.v_valid_frac == 1): out.append('navigation_invalid')
    if s.dead_reckoning_frac or s.heading_good_frac != 1: out.append('navigation_degraded')
    if not (math.isfinite(s.max_eph) and 0 <= s.max_eph <= .5
            and math.isfinite(s.max_epv) and 0 <= s.max_epv <= .5): out.append('position_uncertain')
    if any(s.failsafe.get(k,True) for k,_ in FAILSAFE_FATAL): out.append('failsafe_fault')
    if not (s.est.get('cs_tilt_align') and s.est.get('cs_yaw_align')): out.append('estimator_unaligned')
    if any(s.est.get(k,True) for k in EST_BAD): out.append('estimator_fault')
    if not s.batt_connected or not math.isfinite(s.batt_remaining_min) or not 0 <= s.batt_remaining_min <= 1:
        out.append('battery_unknown')
    elif s.batt_warning_max >= 2 or s.batt_remaining_min < .07:
        out.append('battery_critical')
    if require_rc and (not getattr(s,'rc_valid',False) or getattr(s,'n_rc',0)<1
                       or not 0<=s.ages.get('rc',math.inf)<=MAX_AGE['rc']):
        out.append('rc_missing')
    return out


def final_setpoint(state, velocity, acceleration):
    """Safety modes override ALL task feedforward, including position publishers."""
    if state == 'HOLD': return [0.,0.,0.], None
    if state != 'OK': return [0.,0.,0.], None
    if not all(math.isfinite(float(v)) for v in velocity):
        raise ValueError('nonfinite velocity')
    if acceleration is not None and not all(math.isfinite(float(v)) for v in acceleration):
        raise ValueError('nonfinite acceleration')
    return list(velocity), acceleration


@dataclass
class CommandTicket:
    action: str
    command: int
    expected_mode: int
    created: float
    attempts: int = 0
    last_sent: float = -math.inf
    ack_result: int | None = None
    mode_confirmed: bool = False
    failure: str = ''

    def acknowledge(self, command, result, target_system, target_component):
        if command != self.command or target_system != 255 or target_component != 191:
            return False
        self.ack_result = int(result)
        if result in (2,3,4,6): self.failure = f'ACK_REJECTED({result})'
        return True

    def update(self, now, *, status_fresh, nav_state, armed, landed=False):
        if status_fresh and nav_state == self.expected_mode:
            self.mode_confirmed = True
        # Completion needs ACK + observed mode. Never infer motor RPM from mode.
        if self.ack_result == 0 and self.mode_confirmed:
            return 'LANDED_CONFIRMED' if self.action == 'land' and landed and not armed else 'MODE_CONFIRMED'
        if self.failure: return self.failure
        if now - self.created > CMD_RETRY_S * (CMD_ATTEMPTS + 3):
            self.failure = 'EXECUTION_UNCONFIRMED'; return self.failure
        return 'PENDING'

    def due(self, now):
        return (not self.failure and not (self.ack_result == 0 and self.mode_confirmed)
                and self.attempts < CMD_ATTEMPTS and now - self.last_sent >= CMD_RETRY_S)

    def sent(self, now):
        self.attempts += 1; self.last_sent = now


def landing_order(requested, positions, armed, landed):
    """Always sequence stacked A/B; unknown separation must not authorize A first."""
    targets = set(requested)
    # FC local origins are independent. Do not infer safe separation from them.
    # Only a fresh land detector AND fresh disarmed status release A to land.
    if 0 in targets and not landed.get(1,False):
        targets.add(1); return {0:'hold',1:'land'}
    return {i:'land' for i in targets}
