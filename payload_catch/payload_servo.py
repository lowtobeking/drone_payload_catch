"""ROS-free, one-shot latch interlock. All timestamps use synchronized Unix time."""
from dataclasses import dataclass, asdict
import hashlib
import json
import math
import os
from pathlib import Path
from .safety_runtime import SourceWatch


@dataclass(frozen=True)
class ServoConfig:
    enabled: bool = False
    calibrated: bool = False
    confirmation_mode: str = 'sensor'  # sensor or explicitly selected timed action
    neutral_us: int = 1500
    open_us: int = 1700
    period_ns: int = 20000000
    release_delay_s: float = 1.5
    delay_reference: str = 'local_command_direct'
    max_output_s: float = 2.0
    delay_tolerance_s: float = .02
    feedback_timeout_s: float = .15
    owner: int = 0

    def validate(self):
        if self.confirmation_mode not in ('sensor','timed'):
            raise ValueError('confirmation_mode must be sensor or timed')
        # Continuous-rotation pulses control direction/speed, not latch position.
        if (type(self.enabled) is not bool or type(self.calibrated) is not bool
                or type(self.period_ns) is not int or type(self.owner) is not int
                or type(self.neutral_us) is not int or type(self.open_us) is not int
                or not 1200 <= self.neutral_us <= 1800
                or not 1200 <= self.open_us <= 1800 or self.neutral_us == self.open_us
                or self.period_ns != 20000000 or self.owner not in (0, 1)):
            raise ValueError('invalid servo pulse / period / authority')
        for value, lower, upper in ((self.release_delay_s, 0, 3.0),
                                   (self.max_output_s, 0, 5.0),
                                   (self.delay_tolerance_s, 0, .02),
                                   (self.feedback_timeout_s, .02, .2)):
            if type(value) not in (int, float) or not math.isfinite(value) or not lower < value <= upper:
                raise ValueError('invalid servo timing calibration')
        if self.max_output_s <= self.release_delay_s + self.delay_tolerance_s + .1:
            raise ValueError('PWM time limit must cover opening delay and feedback margin')
        if self.delay_reference not in ('gui_apply', 'unmeasured_direct', 'operator_direct', 'local_command_direct'):
            raise ValueError('unknown release delay reference')
        if self.calibrated and (self.delay_reference not in ('operator_direct','local_command_direct')
                                or (self.confirmation_mode=='sensor' and self.delay_reference!='local_command_direct')):
            raise ValueError('direct output requires local-command calibration before flight; old ramp timing is invalid')


def servo_signature(config):
    return hashlib.sha256(json.dumps(asdict(config),sort_keys=True,allow_nan=False).encode()).hexdigest()


class AttemptRecord:
    """Persist BEFORE motion. Restart never automatically clears a consumed latch."""
    def __init__(self, path):
        self.path = Path(path)

    def consumed(self):
        return self.path.exists()

    def reserve(self, envelope):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        try:
            data = json.dumps(envelope, allow_nan=False).encode()
            with os.fdopen(fd, 'wb') as stream:
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
            directory = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        except BaseException:
            # Even a partial record blocks a repeat; never delete it on failure.
            raise


class ServoInterlock:
    def __init__(self, config, backend, record, *, mock=False):
        config.validate()
        self.config, self.backend, self.record, self.mock = config, backend, record, mock
        self.supervisor_watch = SourceWatch(.2)
        self.feedback_watch = SourceWatch(config.feedback_timeout_s)
        self.auth_watch = SourceWatch(.05)
        self.supervisor = self.feedback = None
        self.session = None
        self.command_wall = self.command_mono = self.planned_at = None
        self.scheduled_command_at = self.output_written_at = self.detached_at = None
        self.authorization_nonce = None
        self.retained_seen = False
        self.physical_released = False
        self.action_completed_at = self.estimated_detach_at = self.output_written_mono = None
        self.reason = ''
        self.state = 'WAITING' if config.enabled and config.calibrated else 'DISABLED'
        if config.enabled and not config.calibrated:
            self.state = 'UNCALIBRATED'
        if record.consumed():
            self.fault('previous release attempt: ground reset required')

    def fault(self, reason):
        if self.state != 'FAULT':
            self.reason = reason
        self.state = 'FAULT'

    def abort(self):
        # Completed, sensor-confirmed release has already stopped its output.
        # Normal landing/stop cannot undo it or turn success into an abort fault.
        # Existing timing/rollback faults remain latched; attempts stay consumed.
        if (self.state == 'RELEASED' and self.physical_released) or self.state=='ACTION_COMPLETE':
            return
        self.fault('release aborted / operator stop; ground reset required')

    def on_supervisor(self, data, wall, mono):
        try:
            if not isinstance(data['session'], str) or not data['session']:
                raise ValueError('missing session')
            if not self.supervisor_watch.accept(data['stamp'], [], wall, mono):
                return
            if self.session is not None and self.session != data['session']:
                self.fault('supervisor restarted')
            self.session, self.supervisor = data['session'], data
            if any(data.get('actions', {}).get(str(i)) in ('hold', 'land', 'kill', 'manual')
                   for i in (0, 1)):
                self.abort()
        except (KeyError, TypeError, ValueError):
            self.supervisor_watch.valid = False

    def on_feedback(self, data, wall, mono):
        if self.config.confirmation_mode=='timed':return  # No sensor evidence exists in this mode.
        try:
            if data.get('session') != self.session or self.session is None:
                raise ValueError('feedback session mismatch')
            if any(type(data[k]) is not bool for k in ('locked', 'present', 'detached', 'healthy')):
                raise ValueError('feedback must contain sensor booleans')
            if not self.feedback_watch.accept(data['stamp'], [], wall, mono):
                return
            self.feedback = data
            retained = data['locked'] and data['present'] and not data['detached']
            detached = data['detached'] and not data['present'] and not data['locked']
            if not data['healthy'] or (data['detached'] and not detached):
                self.fault('latch sensor unhealthy / inconsistent')
            elif detached and not self.physical_released:
                detached_at = data['detached_at']
                if (not math.isfinite(detached_at) or not 0 <= data['stamp'] - detached_at <= self.config.feedback_timeout_s):
                    raise ValueError('detachment requires fresh measured edge time')
                self.detached_at = float(detached_at)
                # Report physical loss even after an abort. PWM success never sets this.
                if self.retained_seen:
                    self.physical_released = True
                if self.command_wall is None:
                    self.fault('unexpected payload detachment')
                elif detached_at < self.command_wall:
                    self.fault('detachment predates command')
                else:
                    self.state = 'RELEASED' if self.state != 'FAULT' else 'FAULT'
                    if abs(detached_at - self.planned_at) > self.config.delay_tolerance_s:
                        self.fault('physical release outside calibrated timing window')
            elif retained and self.command_wall is None:
                self.retained_seen = True
            elif self.command_wall is None:
                self.fault('payload not positively retained')
        except (KeyError, TypeError, ValueError):
            self.feedback_watch.valid = False

    def ready(self, wall, mono):
        d = self.feedback or {}
        return (self.state in ('WAITING', 'READY') and self.config.enabled
                and self.config.calibrated and not self.record.consumed()
                and self.backend.healthy() and self.supervisor_watch.fresh(wall, mono)
                and (self.config.confirmation_mode=='timed' or
                     (self.feedback_watch.fresh(wall, mono)
                      and d.get('healthy') is True and d.get('locked') is True
                      and d.get('present') is True and d.get('detached') is False)))

    def authorize(self, data, wall, mono):
        try:
            if not self.ready(wall, mono):
                return False
            if (data['session'] != self.session or data['owner'] != self.config.owner
                    or not isinstance(data['nonce'], str) or not data['nonce']):
                return False
            if not self.auth_watch.accept(data['stamp'], [data['planned_at'], data['command_at']], wall, mono):
                return False
            c = self.config
            if (abs(data['planned_at'] - data['command_at'] - c.release_delay_s) > 1e-5
                    or not 0 <= wall - data['command_at'] <= .03):
                return False
            s = self.supervisor or {}
            if (s.get('release_permitted') is not True
                    or not all(s.get('permits', {}).get(str(i)) is True for i in (0, 1))
                    or not all(s.get('actions', {}).get(str(i)) == 'run' for i in (0, 1))):
                return False
            self.record.reserve(data)
            # Preserve command time even if write acknowledgement fails: outcome uncertain.
            self.command_wall, self.command_mono = wall, mono
            self.planned_at = data['planned_at']
            self.scheduled_command_at = data['command_at']
            self.authorization_nonce = data['nonce']
            # The persisted reservation/IPC must not silently consume the timing budget.
            receipt = self.backend.open(c.open_us, latest_start_at=data['command_at'] + c.delay_tolerance_s)
            if receipt is not None:
                start, end = receipt['write_started_at'], receipt['write_completed_at']
                if (not all(type(v) in (int,float) and math.isfinite(v) for v in (start,end))
                        or not self.command_wall <= start <= end
                        or abs(end-data['command_at']) > c.delay_tolerance_s):
                    raise ValueError('PWM write missed synchronized release window')
                self.output_written_at = end
                self.output_written_mono = mono+(end-wall)
            elif not self.mock:
                raise ValueError('real PWM requires a timestamped write acknowledgement')
            if self.config.confirmation_mode=='timed':
                # Mock has no write receipt; real hardware must supply one above.
                if self.output_written_at is None:
                    self.output_written_at=wall;self.output_written_mono=mono
                self.estimated_detach_at=self.output_written_at+c.release_delay_s
            self.state = 'COMMAND_SENT'
            return True
        except (KeyError, TypeError, ValueError, OSError, RuntimeError) as exc:
            self.fault('release command failed: ' + str(exc))
            return False

    def hardware_ready(self, wall, mono):
        # During the measured opening delay the payload remains attached. Dropping
        # readiness immediately after a command would make the pair abort itself.
        if self.state in ('RELEASED','ACTION_COMPLETE'):
            return True
        if self.state == 'COMMAND_SENT':
            return (self.backend.healthy() and self.supervisor_watch.fresh(wall, mono)
                    and (self.config.confirmation_mode=='timed' or
                         (self.feedback_watch.fresh(wall, mono) and self.feedback.get('healthy') is True)))
        return self.ready(wall, mono)

    def tick(self, wall, mono):
        if self.state in ('DISABLED', 'UNCALIBRATED', 'FAULT', 'RELEASED', 'ACTION_COMPLETE'):
            return
        if not self.backend.healthy():
            self.fault('PWM worker unavailable')
        elif self.command_wall is not None:
            if (not self.supervisor_watch.fresh(wall, mono)
                    or self.supervisor.get('release_permitted') is not True
                    or not all(self.supervisor.get('permits',{}).get(str(i)) is True for i in (0,1))
                    or not all(self.supervisor.get('actions',{}).get(str(i))=='run' for i in (0,1))):
                self.fault('safety permission lost during opening')
            elif self.config.confirmation_mode=='timed':
                elapsed=mono-self.output_written_mono
                if elapsed<0 or elapsed>self.config.release_delay_s+self.config.delay_tolerance_s+.03:
                    self.fault('timed action scheduler missed completion window')
                elif elapsed>=self.config.release_delay_s:
                    self.action_completed_at=wall
                    self.state='ACTION_COMPLETE'  # Never sets physical_released or detached_at.
            elif not self.feedback_watch.fresh(wall, mono):
                self.fault('physical feedback lost during opening')
            elif mono - self.command_mono > self.config.release_delay_s + self.config.delay_tolerance_s + .03:
                self.fault('physical detachment not confirmed')
        elif self.state == 'READY' and not self.ready(wall, mono):
            self.fault('latch / safety heartbeat lost after ready')
        elif self.ready(wall, mono):
            self.state = 'READY'
