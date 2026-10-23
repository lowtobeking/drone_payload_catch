"""Temporary Orin NX pin33 PWM, with a separate process heartbeat and rollback.

No boot files, Jetson-IO overlays or fan PWM are modified. Requires the previously
captured, root-owned baseline from pwm_pin33_safety. Never automatically drives the latch closed.
"""
import argparse
import fcntl
import hashlib
import json
import math
import mmap
import multiprocessing as mp
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

BASE = Path('/home/nvidia/pwm_pin33_safety')
ADDRESS = 0x02434040
MASK = 3 | (1 << 4) | (1 << 6) | (1 << 10)
PWM_BITS = 1 | (1 << 10)


class JetsonPWM:
    def __init__(self):
        self.lock = None
        self.owned = False
        self.original = None
        self.chip = None
        self.journal = BASE / 'flight_pwm_state.json'

    def verify(self):
        if os.geteuid() != 0:
            raise RuntimeError('PWM runtime requires explicitly provisioned root service')
        for p in (BASE, BASE / 'pin33_register_original.json', BASE / 'manifest.json'):
            st = p.stat()
            if p.is_symlink() or st.st_uid != 0 or st.st_mode & 0o022:
                raise RuntimeError('PWM baseline must be root-owned and not writable by others')
        comp = Path('/proc/device-tree/compatible').read_bytes()
        if b'nvidia,p3768-0000+p3767-0001\0' not in comp or b'nvidia,tegra234\0' not in comp:
            raise RuntimeError('unsupported board: expected verified Orin NX 8GB carrier')
        self.original = json.loads((BASE / 'pin33_register_original.json').read_text())
        boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        if (self.original['address'] != ADDRESS or self.original['value'] != 0x54
                or self.original['boot_id'] != boot_id):
            raise RuntimeError('baseline / boot changed; ground revalidation required')
        for item in json.loads((BASE / 'manifest.json').read_text()).values():
            if item['source'].startswith('/boot/'):
                if hashlib.sha256(Path(item['source']).read_bytes()).hexdigest() != item['sha256']:
                    raise RuntimeError('boot configuration changed')
        found = [p for p in Path('/sys/class/pwm').glob('pwmchip*')
                 if '/32c0000.pwm/' in str(p.resolve())]
        if len(found) != 1 or (found[0] / 'npwm').read_text().strip() != '1':
            raise RuntimeError('pin33 controller cannot be identified')
        self.chip = found[0]

    def register(self, value=None):
        fd = os.open('/dev/mem', (os.O_RDWR if value is not None else os.O_RDONLY) | os.O_SYNC)
        try:
            with mmap.mmap(fd, 4096, flags=mmap.MAP_SHARED,
                           prot=mmap.PROT_READ | (mmap.PROT_WRITE if value is not None else 0),
                           offset=ADDRESS & ~4095) as mem:
                current = struct.unpack_from('<I', mem, ADDRESS & 4095)[0]
                target = (self.original['value'] & ~MASK) | PWM_BITS
                if value is not None:
                    if current not in (self.original['value'], target) or value not in (self.original['value'], target):
                        raise RuntimeError('unexpected pinmux value; refusing overwrite')
                    struct.pack_into('<I', mem, ADDRESS & 4095, value)
                    if struct.unpack_from('<I', mem, ADDRESS & 4095)[0] != value:
                        raise RuntimeError('pinmux readback failed')
                    return value
                return current
        finally:
            os.close(fd)

    def write(self, name, value):
        path = self.chip / 'pwm0' / name
        path.write_text(str(value))
        if path.read_text().strip() != str(value):
            raise RuntimeError('PWM readback failed: ' + name)

    def acquire(self):
        self.lock = open('/run/pwm-pin33-config.lock', 'a')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.verify()

    def prepare(self, period_ns):
        self.acquire()
        pwm = self.chip / 'pwm0'
        if self.journal.exists() or (BASE / 'temporary_state.json').exists() or pwm.exists():
            raise RuntimeError('PWM owned / recovery pending; refusing takeover')
        if period_ns != 20000000 or self.register() != self.original['value']:
            raise RuntimeError('incorrect period or pin not at baseline')
        gpio = subprocess.run(['gpioinfo', 'gpiochip0'], capture_output=True, text=True,
                              check=True, timeout=2).stdout
        line = next(x for x in gpio.splitlines() if '"PH.00"' in x)
        if 'unused' not in line or '[used]' in line:
            raise RuntimeError('PH.00 already in use')
        # Journal before export allows explicit recovery after power/process interruption.
        fd = os.open(self.journal, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as stream:
            json.dump(dict(boot_id=self.original['boot_id'], chip=str(self.chip)), stream)
            stream.flush()
            os.fsync(stream.fileno())
        self.owned = True
        (self.chip / 'export').write_text('0')
        deadline = time.monotonic() + 1
        while not (pwm / 'period').exists():
            if time.monotonic() > deadline:
                raise RuntimeError('PWM export timed out')
            time.sleep(.01)
        self.write('enable', 0)
        self.write('duty_cycle', 0)
        self.write('polarity', 'normal')
        self.write('period', period_ns)
        self.register((self.original['value'] & ~MASK) | PWM_BITS)
        text = Path('/sys/kernel/debug/pinctrl/2430000.pinmux/pinconf-groups').read_text()
        entry = text.split('(soc_gpio21_ph0):', 1)[1].split('\n133 ', 1)[0]
        if not all(x in entry for x in ('function=gp', 'tristate=0', 'enable-input=0', 'gpio-mode=1')):
            raise RuntimeError('kernel pinmux readback mismatch')

    def open(self, pulse_us):
        if not self.owned or type(pulse_us) is not int or not 1200 <= pulse_us <= 1800:
            raise RuntimeError('invalid open pulse or ownership')
        self.write('duty_cycle', pulse_us * 1000)
        self.write('enable', 1)

    def close(self):
        failures = []
        if self.owned:
            pwm = self.chip / 'pwm0'
            # Each cleanup step is attempted even when a previous one fails.
            operations = []
            if pwm.exists():
                operations.extend((lambda: self.write('enable', 0), lambda: self.write('duty_cycle', 0)))
            operations.append(lambda: self.register(self.original['value']))
            if pwm.exists():
                operations.append(lambda: (self.chip / 'unexport').write_text('0'))
            for operation in operations:
                try:
                    operation()
                except Exception as exc:
                    failures.append(str(exc))
            if not failures and self.register() == self.original['value'] and not pwm.exists():
                self.journal.unlink()
                self.owned = False
        if self.lock is not None:
            self.lock.close()
            self.lock = None
        if failures:
            raise RuntimeError('rollback incomplete: ' + '; '.join(failures))

    def recover(self):
        self.acquire()
        if self.journal.exists():
            data = json.loads(self.journal.read_text())
            if data['boot_id'] != self.original['boot_id'] or data['chip'] != str(self.chip):
                raise RuntimeError('recovery baseline mismatch')
            self.owned = True
        elif self.register()!=self.original['value'] or (self.chip/'pwm0').exists():
            self.close()
            raise RuntimeError('no owned runtime journal: refusing to restore another configuration')
        self.close()


def _worker(connection, period_ns, max_output_s=2.0):
    backend = JetsonPWM()
    def stop(signum, frame):
        raise RuntimeError('PWM worker terminated')
    signal.signal(signal.SIGTERM, stop)
    opened_at = None
    last_parent = time.monotonic()
    try:
        if (type(max_output_s) not in (int, float) or not math.isfinite(max_output_s)
                or not 0 < max_output_s <= 5):
            raise ValueError('invalid maximum PWM on-time')
        backend.prepare(period_ns)
        last_parent = time.monotonic()
        connection.send(('ready', ''))
        while True:
            if connection.poll(.02):
                command, value, sent_at = connection.recv()
                now = time.monotonic()
                if not 0 <= now - sent_at <= .25:
                    raise RuntimeError('stale parent command')
                last_parent = now
                if command == 'close':
                    break
                # Reject an expired actuator even if a fresh command just arrived.
                if opened_at is not None and now-opened_at >= max_output_s:
                    raise RuntimeError('maximum PWM on-time exceeded')
                if command == 'open':
                    if opened_at is not None:
                        raise RuntimeError('duplicate open command')
                    pulse = value.get('pulse_us') if isinstance(value,dict) else value
                    latest = value.get('latest_start_at') if isinstance(value,dict) else None
                    if type(pulse) is not int or not 1200 <= pulse <= 1800:
                        raise ValueError('opening target must be integer 1200..1800us')
                    if latest is not None and (type(latest) not in (int,float)
                            or not math.isfinite(latest) or time.time() > latest):
                        raise RuntimeError('PWM start deadline expired; no output')
                    # This brackets sysfs writes, NOT an electrically measured first edge.
                    started = time.time()
                    opened_at = time.monotonic()
                    backend.open(pulse)
                    receipt = dict(write_started_at=started,write_completed_at=time.time())
                elif command != 'ping':
                    raise RuntimeError('unknown PWM command')
                connection.send((command, receipt if command == 'open' else ''))
            now = time.monotonic()
            if now-last_parent > .25:
                raise RuntimeError('parent heartbeat expired')
            if opened_at is not None:
                if now-opened_at >= max_output_s:
                    raise RuntimeError('maximum PWM on-time exceeded')
    except (Exception, EOFError) as exc:
        try:
            connection.send(('error', str(exc)))
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        try:
            backend.close()
            connection.send(('closed', ''))
        except Exception as exc:
            try:
                connection.send(('error', str(exc)))
            except (BrokenPipeError, EOFError, OSError):
                pass
        connection.close()


class PWMWorker:
    def __init__(self, period_ns, *, max_output_s=2.0):
        ctx = mp.get_context('spawn')
        self.connection, child = ctx.Pipe()
        self.process = ctx.Process(target=_worker, args=(child, period_ns, max_output_s))
        self.process.start()
        child.close()
        self.last_ack = time.monotonic()
        self.error = ''
        self.closed = False
        if not self.connection.poll(5) or self.connection.recv()[0] != 'ready':
            self.close()
            raise RuntimeError('PWM prepare failed; inspect rollback journal')

    def healthy(self):
        try:
            while self.connection.poll():
                event, reason = self.connection.recv()
                if event in ('error', 'closed'):
                    self.error = reason or 'worker stopped'
                self.last_ack = time.monotonic()
        except (EOFError, OSError):
            self.error = 'worker disconnected'
        return (not self.closed and not self.error and self.process.is_alive()
                and time.monotonic() - self.last_ack <= .25)

    def heartbeat(self):
        if not self.healthy():
            raise RuntimeError(self.error or 'PWM worker heartbeat lost')
        self.connection.send(('ping', None, time.monotonic()))

    def open(self, pulse_us, *, latest_start_at=None):
        if type(pulse_us) is not int or not 1200 <= pulse_us <= 1800:
            raise ValueError('opening target must be integer 1200..1800us')
        if not self.healthy():
            raise RuntimeError('PWM worker not ready')
        self.connection.send(('open', dict(pulse_us=pulse_us,latest_start_at=latest_start_at), time.monotonic()))
        deadline = time.monotonic() + .04
        while time.monotonic() < deadline:
            if self.connection.poll(max(0, deadline - time.monotonic())):
                event, reason = self.connection.recv()
                self.last_ack = time.monotonic()
                if event == 'open':
                    return reason
                if event in ('error', 'closed'):
                    raise RuntimeError(reason or event)
        raise RuntimeError('PWM write acknowledgement timed out; outcome uncertain')

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            if self.process.is_alive():
                self.connection.send(('close', None, time.monotonic()))
                self.process.join(.5)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(.5)
            if self.process.is_alive():
                raise RuntimeError('PWM worker still running; explicit recovery required')
            while self.connection.poll():
                try:
                    event, reason = self.connection.recv()
                except EOFError:
                    break
                if event == 'error' and 'rollback' in reason:
                    raise RuntimeError(reason)
        finally:
            self.connection.close()
            if not self.process.is_alive() and (BASE / 'flight_pwm_state.json').exists():
                JetsonPWM().recover()


class MockPWM:
    def __init__(self):
        self.pulses = []
        self.closed = False
    def healthy(self):
        return not self.closed
    def heartbeat(self):
        pass
    def open(self, pulse_us, *, latest_start_at=None):
        if self.closed:
            raise RuntimeError('mock PWM closed')
        self.pulses.append(pulse_us)
    def close(self):
        self.closed = True


def main():
    parser = argparse.ArgumentParser(description='Explicit ground recovery of owned PWM33 runtime')
    parser.add_argument('action', choices=['restore'])
    parser.parse_args()
    JetsonPWM().recover()


if __name__ == '__main__':
    main()
