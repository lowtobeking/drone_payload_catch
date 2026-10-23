#!/usr/bin/env python3
"""Bench-only newline JSON agent, streamed over SSH; no ROS or boot changes."""
import argparse
from collections import deque
try:
    import fcntl
except ImportError:  # Windows GUI may use Controller + MockBackend for demo only.
    fcntl = None
import importlib.util
import json
import math
import os
from pathlib import Path
import select
import signal
import sys
import time
import uuid

# ES08MD conservative bench starting profile, not a manufacturer frequency rating.
# The old DS041MG profile is accepted only for readback/stop and explicit migration.
PERIOD_NS = 20_000_000  # 50 Hz
LEGACY_PERIOD_NS = 3_003_003
MIN_US, MAX_US = 1200, 1800  # Operator-requested bench limits; enforced independently of GUI.
LEASE_S = 2.0  # Output watchdog remains unchanged.
SESSION_IDLE_S = 30.0  # Reclaim silent sessions without equating short stalls with SSH failure.


class CommandRejected(ValueError):
    """Stop output and report an invalid/stale command without destroying SSH."""


class MockBackend:
    def __init__(self):
        self.mode = False
        self.pulse = 0
        self.events = []
    def status(self):
        return dict(mode=self.mode, enabled=self.pulse != 0, pulse_us=self.pulse,
                    period_ns=PERIOD_NS, pin=33, mock=True, profile_ready=self.mode, servo_model='EMAX ES08MD II')
    def prepare(self):
        self.mode = True
    def output(self, pulse):
        if not self.mode or type(pulse) is not int or not MIN_US <= pulse <= MAX_US:
            raise ValueError('invalid pulse or mode')
        self.pulse = pulse
        self.events.append(pulse)
    def stop(self):
        self.pulse = 0
    def restore(self):
        self.stop()
        self.mode = False
    def check(self):
        pass
    def close(self):
        self.stop()


class JetsonBackend:
    def __init__(self):
        if os.geteuid() != 0:
            raise RuntimeError('sudo -n required for verified pin33 runtime')
        self.lock = open('/run/pwm-pin33-config.lock', 'a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.base = Path('/home/nvidia/pwm_pin33_safety')
            helper = self.base / 'pin33_pwm_safe.py'
            for path in (self.base, helper, self.base/'pin33_register_original.json'):
                st = path.stat()
                if path.is_symlink() or st.st_uid != 0 or st.st_mode & 0o022:
                    raise RuntimeError('unsafe PWM helper/baseline ownership')
            spec = importlib.util.spec_from_file_location('pin33_safe', helper)
            self.safe = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.safe)
            # A reboot invalidates write authorization, not read-only SSH access.
            compatible=Path('/proc/device-tree/compatible').read_bytes()
            if b'nvidia,p3768-0000+p3767-0001\0' not in compatible or b'nvidia,tegra234\0' not in compatible:
                raise RuntimeError('unverified board; connection refused')
            self.baseline_current=(Path('/proc/sys/kernel/random/boot_id').read_text().strip()
                                   ==self.safe.original()['boot_id'])
            if self.baseline_current:self.safe.verify_board()
            self.safe.boot_unchanged()
            self.check()
            self.status()  # Refuse unknown exported channels or ownership.
        except BaseException:
            self.lock.close()
            raise

    def check(self):
        if (self.base/'flight_pwm_state.json').exists():
            raise RuntimeError('flight PWM runtime is active')
        blocked = ('a_node', 'b_node', 'payload_servo', 'safety_supervisor')
        for path in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                if int(path.parent.name) == os.getpid():
                    continue
                args = path.read_bytes().split(b'\0')
                names = [Path(a.decode(errors='ignore')).name for a in args if a]
                if any(name in blocked or any(name == 'payload_catch.'+b for b in blocked) for name in names):
                    raise RuntimeError('flight task is running; bench PWM refused')
            except (FileNotFoundError, ProcessLookupError, PermissionError):
                continue

    def require_current_baseline(self):
        if not self.baseline_current:
            raise CommandRejected('设备已重启，PWM安全基线过期：仅可查看；禁止配置、输出或恢复，须独立重新核验')

    def owned(self):
        s = self.safe
        state = json.loads(s.STATE.read_text())
        if (state['boot_id'] != s.original()['boot_id'] or state['chip'] != str(s.chip())
                or state.get('purpose') != 'operator_requested_PWM_mode_hold'
                or state.get('exported') is not True):
            raise RuntimeError('PWM belongs to another owner')
        return state

    def status(self):
        s = self.safe
        value = s.register()
        pwm = s.chip()/'pwm0'
        if not self.baseline_current:
            # Never reinterpret a previous boot's ownership or update its record.
            if pwm.exists() or value != s.original()['value']:
                raise RuntimeError('baseline expired and hardware is not idle/original; independent inspection required')
            return dict(mode=False,enabled=False,pulse_us=0,period_ns=PERIOD_NS,pin=33,mock=False,
                        profile_ready=False,control_ready=False,baseline_current=False,
                        control_block_reason='设备已重启，PWM基线过期：仅可查看，禁止输出；须重新核验',
                        servo_model='EMAX ES08MD II')
        if not s.STATE.exists():
            if pwm.exists() or value != 0x54:
                raise RuntimeError('unknown pin/PWM state; do not take over')
            return dict(mode=False, enabled=False, pulse_us=0, period_ns=PERIOD_NS, pin=33, mock=False, profile_ready=False, servo_model='EMAX ES08MD II')
        self.owned()
        if value != 0x405 or not pwm.exists():
            raise RuntimeError('owned mode does not match hardware')
        period = int((pwm/'period').read_text())
        duty = int((pwm/'duty_cycle').read_text())
        enabled = (pwm/'enable').read_text().strip() == '1'
        if period not in (PERIOD_NS, LEGACY_PERIOD_NS) or (pwm/'polarity').read_text().strip() != 'normal':
            raise RuntimeError('PWM period/polarity changed')
        lower, upper = (MIN_US, MAX_US) if period == PERIOD_NS else (1300, 1800)
        if enabled and not lower*1000 <= duty <= upper*1000:
            raise RuntimeError('PWM pulse outside bench range')
        return dict(mode=True, enabled=enabled, pulse_us=duty/1000 if enabled else 0,
                    period_ns=period, pin=33, mock=False, profile_ready=period == PERIOD_NS, servo_model='EMAX ES08MD II')

    def prepare(self):
        self.require_current_baseline()
        current = self.status()
        if current['enabled']:
            raise RuntimeError('stop output before configuring ES08MD profile')
        self.check()
        if current['mode']:
            # Only this explicit prepare action migrates the disabled legacy channel.
            pwm = self.safe.chip()/'pwm0'
            self.safe.write(pwm/'enable', 0)
            self.safe.write(pwm/'duty_cycle', 0)
            self.safe.write(pwm/'period', PERIOD_NS)
            self.status()
            return
        s = self.safe
        try:
            s.prepare()
            state = json.loads(s.STATE.read_text())
            state.update(purpose='operator_requested_PWM_mode_hold', output_enabled=False, auto_restore=False)
            s.STATE.write_text(json.dumps(state, indent=2))
            os.chmod(s.STATE, 0o600)
            s.write(s.chip()/'pwm0'/'period', PERIOD_NS)
            s.register((s.original()['value'] & ~s.MASK) | s.PWM_BITS)
            text = Path('/sys/kernel/debug/pinctrl/2430000.pinmux/pinconf-groups').read_text()
            entry = text.split('(soc_gpio21_ph0):', 1)[1].split('\n133 ', 1)[0]
            if not all(x in entry for x in ('function=gp', 'tristate=0', 'enable-input=0', 'gpio-mode=1')):
                raise RuntimeError('kernel pinmux readback mismatch')
            self.status()
        except BaseException:
            if s.STATE.exists():
                s.restore()
            raise

    def output(self, pulse):
        self.require_current_baseline()
        if type(pulse) is not int or not MIN_US <= pulse <= MAX_US:
            raise ValueError('pulse must be 1200..1800us')
        self.owned()
        s = self.safe
        if s.register() != 0x405:
            raise RuntimeError('pinmux changed')
        pwm = s.chip()/'pwm0'
        if int((pwm/'period').read_text()) != PERIOD_NS:
            raise RuntimeError('period changed')
        s.write(pwm/'duty_cycle', pulse*1000)
        if int((pwm/'duty_cycle').read_text()) != pulse*1000:
            raise RuntimeError('duty readback failed')
        if (pwm/'enable').read_text().strip() != '1':
            s.write(pwm/'enable', 1)
        if (pwm/'enable').read_text().strip() != '1':
            raise RuntimeError('enable readback failed')

    def stop(self):
        if not self.baseline_current:
            self.status()  # Confirm unexported/original; no writes on this boot.
            return
        if not self.safe.STATE.exists():
            return
        self.owned()
        pwm = self.safe.chip()/'pwm0'
        # Always attempt disable before metadata/readback work.
        self.safe.write(pwm/'enable', 0)
        self.safe.write(pwm/'duty_cycle', 0)
        if (pwm/'enable').read_text().strip() != '0' or int((pwm/'duty_cycle').read_text()) != 0:
            raise RuntimeError('cannot confirm output stopped')
        state = self.owned()
        state.update(output_enabled=False)
        state.pop('pulse_us', None)
        self.safe.STATE.write_text(json.dumps(state, indent=2))

    def restore(self):
        self.require_current_baseline()
        self.stop()
        self.safe.restore()

    def close(self):
        try:
            self.stop()
        finally:
            self.lock.close()


class Controller:
    def __init__(self, backend, now):
        self.backend = backend
        self.session = uuid.uuid4().hex
        self.tokens = deque(maxlen=16)
        self.seq = -1
        self.last_ping = now
        self.running = False
        self.target = self.current = None
        self.deadline = None
        self.last_tick = now
        self.reason = ''
        self.expired = False
        self.heartbeat_lost = False

    def stop(self, reason='stopped'):
        self.running = False
        self.target = None
        self.backend.stop()
        self.reason = reason

    def status(self, now):
        token = uuid.uuid4().hex
        self.tokens.append((token, now))
        return dict(type='status', session=self.session, token=token, seq=self.seq,
                    running=self.running, target_us=self.target, reason=self.reason,
                    lease_s=LEASE_S, session_idle_s=SESSION_IDLE_S,
                    heartbeat_lost=self.heartbeat_lost, ping_age_s=max(0,now-self.last_ping),
                    **self.backend.status())

    def command(self, d, now):
        if self.expired:
            raise RuntimeError('connection lease expired; reconnect')
        if d.get('session') != self.session or type(d.get('seq')) is not int or d['seq'] <= self.seq:
            raise CommandRejected('old session / duplicate command')
        action = d.get('action')
        # Stop always bypasses challenge age, but still needs this session.
        if action not in ('stop', 'close') and not any(
                token == d.get('token') and 0 <= now-stamp <= 1.5 for token,stamp in self.tokens):
            raise CommandRejected('stale command; wait for fresh status')
        self.seq = d['seq']
        if action == 'ping':
            self.last_ping = now
            if self.heartbeat_lost:
                self.reason = 'heartbeat recovered; pulses remain stopped; apply target explicitly'
            self.heartbeat_lost = False
        elif action == 'prepare':
            if self.running:
                raise CommandRejected('stop before changing mode')
            self.backend.prepare()
            self.reason = 'PWM mode ready; no pulses'
        elif action == 'set':
            if self.heartbeat_lost or now-self.last_ping > LEASE_S:
                raise CommandRejected('heartbeat not healthy; wait for fresh ping then explicitly apply target')
            pulse, duration = d.get('pulse_us'), d.get('seconds')
            if type(pulse) is not int or not MIN_US <= pulse <= MAX_US:
                raise CommandRejected('pulse must be integer 1200..1800us')
            if type(duration) not in (int, float) or not math.isfinite(duration) or not (duration == 0 or 1 <= duration <= 60):
                raise CommandRejected('duration 1..60s, or explicit continuous=0')
            hardware = self.backend.status()
            if not hardware['mode'] or not hardware.get('profile_ready', False):
                raise CommandRejected('prepare ES08MD 50Hz PWM mode first')
            self.backend.check()
            # Apply the actual target immediately, including the very first pulse.
            self.backend.output(pulse)
            self.current = float(pulse)
            self.target = pulse
            self.running = True
            self.last_tick = now
            self.deadline = now+duration if duration else None
            self.reason = 'output active; direct pulse applied'
        elif action in ('stop', 'close'):
            self.stop('pulses stopped; PWM mode retained')
        elif action == 'restore':
            self.stop()
            self.backend.restore()
            self.reason = 'GPIO baseline restored'
        else:
            raise CommandRejected('unknown action')

    def tick(self, now):
        if now-self.last_ping > LEASE_S:
            if not self.heartbeat_lost:
                self.stop('SSH/UI heartbeat lost; pulses stopped; session waiting for recovery')
                self.heartbeat_lost = True
            if now-self.last_ping > SESSION_IDLE_S:
                self.expired = True
                self.reason = 'SSH/UI heartbeat absent for 30s; idle session closed; pulses stopped'
            self.last_tick = now
            return
        if self.running:
            if self.deadline is not None and now >= self.deadline:
                self.stop('timer elapsed; PWM mode retained')
                return
            # The target has already been written; fixed PWM runs in hardware.
            pass
        self.last_tick = now


def run_agent(mock=False):
    backend = MockBackend() if mock else JetsonBackend()
    controller = Controller(backend, time.monotonic())
    def interrupted(*args):
        raise KeyboardInterrupt()
    for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(sig, interrupted)
    def emit(data):
        print(json.dumps(data, allow_nan=False), flush=True)
    data = bytearray()
    last_status = last_check = -1e9
    try:
        while True:
            now = time.monotonic()
            controller.tick(now)
            if controller.expired:
                emit(controller.status(now))
                break
            if now-last_check > .5:
                backend.check()
                last_check = now
            if now-last_status >= .2:
                emit(controller.status(now))
                last_status = now
            readable, _, _ = select.select([sys.stdin.buffer], [], [], .02)
            if readable:
                chunk = os.read(sys.stdin.fileno(), 4096)
                if not chunk:
                    break
                data.extend(chunk)
                if len(data) > 8192:
                    raise ValueError('command buffer too large')
                while b'\n' in data:
                    line, _, rest = data.partition(b'\n')
                    data = bytearray(rest)
                    request = json.loads(line)
                    if not isinstance(request, dict):
                        raise ValueError('command must be object')
                    try:
                        controller.command(request, time.monotonic())
                    except CommandRejected as exc:
                        controller.stop('command rejected; pulses stopped: '+str(exc))
                        emit(dict(type='error', error=str(exc), recoverable=True))
                    emit(controller.status(time.monotonic()))
                    if request.get('action') == 'close':
                        return
    except (Exception, KeyboardInterrupt) as exc:
        try:
            controller.stop('error: '+str(exc))
            emit(dict(type='error', error=str(exc)))
        except Exception as stop_error:
            print(json.dumps(dict(type='error', error=str(exc)+'; STOP UNCONFIRMED: '+str(stop_error))), flush=True)
        if not isinstance(exc, KeyboardInterrupt):
            raise
    finally:
        backend.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--mock', action='store_true')
    run_agent(parser.parse_args().mock)
