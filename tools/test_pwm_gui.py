#!/usr/bin/env python3
"""Offline protocol/lease/fault tests and an actual mock agent subprocess."""
import json
from pathlib import Path
import select
import subprocess
import sys
import time
import unittest
from unittest.mock import Mock
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from tools.pwm_remote_agent import Controller,MockBackend,JetsonBackend,PERIOD_NS,LEGACY_PERIOD_NS,SESSION_IDLE_S
from tools.pwm_angle import angle_to_pulse,pulse_to_angle,angle_limits
import tempfile
from types import SimpleNamespace


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.backend=MockBackend();self.control=Controller(self.backend,100)
        self.seq=0
    def command(self,action,now=100,**values):
        status=self.control.status(now)
        self.seq+=1
        d=dict(action=action,session=status['session'],token=status['token'],seq=self.seq,**values)
        self.control.command(d,now)
        return d
    def prepare(self):self.command('prepare')
    def test_connect_is_idle_and_set_requires_prepared_mode(self):
        self.assertFalse(self.control.status(100)['enabled'])
        with self.assertRaises(ValueError):self.command('set',pulse_us=1520,seconds=10)
        self.assertFalse(self.backend.events)
    def test_direct_first_pulse_and_duration_stop_keep_mode(self):
        self.prepare();self.command('set',pulse_us=1600,seconds=1)
        self.assertEqual(self.backend.events,[1600])
        self.assertEqual(self.backend.pulse,1600)
        for step in range(1,11):
            self.control.tick(100+step*.05)
            self.assertEqual(self.backend.pulse,1600)
        self.control.tick(101)
        self.assertEqual(self.backend.pulse,0)
        self.assertTrue(self.backend.mode)
    def test_continuous_requires_renewed_ui_heartbeat(self):
        self.prepare();self.command('set',pulse_us=1520,seconds=0)
        self.command('ping',101.5);self.control.tick(102)
        self.assertTrue(self.control.running)
        self.control.tick(103.6)
        self.assertFalse(self.control.running)
        self.assertEqual(self.backend.pulse,0)
        self.assertTrue(self.backend.mode)
        self.assertFalse(self.control.expired)
        with self.assertRaises(ValueError):self.command('set',103.7,pulse_us=1520,seconds=10)
        self.command('ping',103.8)
        self.assertFalse(self.control.heartbeat_lost)
        self.assertEqual(self.backend.pulse,0)
        self.control.tick(104)
        self.assertEqual(self.backend.pulse,0)
        self.command('set',104,pulse_us=1500,seconds=0)
        self.assertTrue(self.control.running)
    def test_stop_disables_without_reposition_or_restore(self):
        self.prepare();self.command('set',pulse_us=1600,seconds=10)
        self.control.tick(100.05)
        history=list(self.backend.events)
        self.command('stop',100.1)
        self.assertEqual(self.backend.events,history)
        self.assertTrue(self.backend.mode)
        self.assertFalse(self.backend.status()['enabled'])
    def test_restore_is_explicit(self):
        self.prepare();self.command('restore')
        self.assertFalse(self.backend.mode)
    def test_rejects_invalid_pulses_and_durations(self):
        self.prepare()
        for pulse in (1199,1801,999,2001,True,1520.5,'1520'):
            with self.assertRaises(ValueError):self.command('set',pulse_us=pulse,seconds=10)
        for duration in (-1,.5,61,True,float('nan'),float('inf')):
            with self.assertRaises(ValueError):self.command('set',pulse_us=1520,seconds=duration)
        self.assertFalse(self.backend.events)
    def test_fresh_server_challenge_and_session_required(self):
        self.prepare();s=self.control.status(100)
        d=dict(action='set',session=s['session'],seq=20,token=s['token'],pulse_us=1520,seconds=10)
        with self.assertRaises(ValueError):self.control.command(d,101.6)
        d.update(session='old',token=self.control.status(101.6)['token'])
        with self.assertRaises(ValueError):self.control.command(d,101.6)
        self.assertFalse(self.backend.events)
    def test_duplicate_and_reordered_commands_cannot_restart_output(self):
        self.prepare();old=self.command('set',pulse_us=1520,seconds=10)
        self.command('stop')
        with self.assertRaises(ValueError):self.control.command(old,100)
        self.assertFalse(self.backend.status()['enabled'])
    def test_stop_works_even_if_challenge_expired(self):
        self.prepare();self.command('set',pulse_us=1520,seconds=0)
        self.control.command(dict(action='stop',session=self.control.session,seq=99,token='old'),101.9)
        self.assertEqual(self.backend.pulse,0)
    def test_backend_failure_surfaces_and_caller_can_stop(self):
        self.prepare();self.command('set',pulse_us=1600,seconds=10)
        self.backend.output=Mock(side_effect=OSError('readback failed'))
        with self.assertRaises(OSError):self.command('set',100.05,pulse_us=1700,seconds=0)
        self.control.stop('error')
        self.assertEqual(self.backend.pulse,0)


    def test_long_idle_session_expires_but_short_stall_only_stops_output(self):
        self.prepare();self.command('set',pulse_us=1500,seconds=0)
        self.control.tick(102.1)
        self.assertFalse(self.control.expired)
        self.assertTrue(self.control.heartbeat_lost)
        self.assertFalse(self.control.running)
        self.assertEqual(self.backend.pulse,0)
        self.control.tick(100+SESSION_IDLE_S+.1)
        self.assertTrue(self.control.expired)

    def test_continuous_hold_exceeds_old_timer_and_changes_target_without_recentering(self):
        self.prepare();self.command('set',pulse_us=1550,seconds=0)
        for step in range(1,2401):
            now=100+step*.05
            if step % 8 == 0:self.command('ping',now)
            self.control.tick(now)
        self.assertTrue(self.control.running)
        self.assertIsNone(self.control.deadline)
        self.assertEqual(self.backend.pulse,1550)
        history=len(self.backend.events)
        self.command('set',220,pulse_us=1450,seconds=0)
        self.assertEqual(len(self.backend.events),history+1)
        self.assertEqual(self.backend.pulse,1450)
        self.control.tick(220.05)
        self.assertEqual(self.backend.pulse,1450)
        self.command('stop',220.1)
        self.assertEqual(self.backend.pulse,0)
        self.assertTrue(self.backend.mode)

    def test_legacy_frequency_cannot_start_before_explicit_prepare(self):
        self.prepare()
        status=self.backend.status
        self.backend.status=lambda:dict(status(),profile_ready=False,period_ns=LEGACY_PERIOD_NS)
        with self.assertRaisesRegex(ValueError,'50Hz'):
            self.command('set',pulse_us=1500,seconds=0)
        self.assertFalse(self.backend.events)


class AngleTests(unittest.TestCase):
    def test_datasheet_nominal_mapping_and_quantization(self):
        for angle,pulse in ((-9,1400),(0,1500),(9,1600)):
            self.assertEqual(angle_to_pulse(str(angle)),pulse)
            self.assertAlmostEqual(pulse_to_angle(pulse),angle)
        self.assertEqual(angle_to_pulse(5),1556)
        self.assertEqual(angle_to_pulse(1.23),1514)
        self.assertEqual(angle_limits(),(-9,9))
        self.assertEqual(angle_limits(True),(-45,45))
        for angle in (-45,-12.3,0,18.2,45):
            self.assertLessEqual(abs(pulse_to_angle(angle_to_pulse(angle,True))-angle),.045001)
    def test_expansion_uses_datasheet_90deg_total_not_180deg(self):
        with self.assertRaises(ValueError):angle_to_pulse(45)
        self.assertEqual(angle_to_pulse(45,True),2000)
        self.assertEqual(angle_to_pulse(-45,True),1000)
        self.assertAlmostEqual(pulse_to_angle(2000)-pulse_to_angle(1000),90)
        for angle in (45.01,-45.01,90,180):
            with self.assertRaises(ValueError):angle_to_pulse(angle,True)
    def test_nonfinite_and_out_of_range_even_before_rounding_are_rejected(self):
        for angle in ('nan','inf','-inf','bad','',True,9.001,-9.001):
            with self.assertRaises(ValueError):angle_to_pulse(angle)


class PeriodMigrationTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        self.chip=Path(temp.name);self.pwm=self.chip/'pwm0';self.pwm.mkdir()
        for name,value in dict(enable=0,duty_cycle=0,period=LEGACY_PERIOD_NS,polarity='normal').items():
            (self.pwm/name).write_text(str(value))
        self.backend=object.__new__(JetsonBackend)
        self.backend.baseline_current=True
        self.backend.safe=SimpleNamespace(chip=lambda:self.chip,write=lambda p,v:p.write_text(str(v)))
        self.backend.status=lambda:dict(mode=True,enabled=(self.pwm/'enable').read_text()=='1')
        self.backend.check=Mock()
    def test_prepare_changes_only_disabled_channel_period_and_never_outputs(self):
        self.backend.prepare()
        self.assertEqual(int((self.pwm/'period').read_text()),PERIOD_NS)
        self.assertEqual(int((self.pwm/'enable').read_text()),0)
        self.assertEqual(int((self.pwm/'duty_cycle').read_text()),0)
    def test_enabled_channel_must_be_stopped_first(self):
        (self.pwm/'enable').write_text('1')
        with self.assertRaisesRegex(RuntimeError,'stop output'):
            self.backend.prepare()
        self.assertEqual(int((self.pwm/'period').read_text()),LEGACY_PERIOD_NS)
    def test_output_refuses_legacy_frequency(self):
        self.backend.owned=Mock();self.backend.safe.register=lambda:0x405
        with self.assertRaisesRegex(RuntimeError,'period changed'):
            self.backend.output(1500)
        self.assertEqual(int((self.pwm/'enable').read_text()),0)


class ExpiredBaselineTests(unittest.TestCase):
    def setUp(self):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        self.chip=Path(tmp.name)
        self.backend=object.__new__(JetsonBackend)
        self.backend.baseline_current=False
        self.backend.safe=SimpleNamespace(chip=lambda:self.chip,register=lambda:0x54,
            original=lambda:dict(value=0x54),STATE=self.chip/'old_state.json',
            write=Mock(side_effect=AssertionError('unexpected hardware write')),
            restore=Mock(side_effect=AssertionError('unexpected GPIO restore')))
        self.backend.safe.STATE.write_text('{"boot_id":"previous boot","exported":true}')
        self.saved=self.backend.safe.STATE.read_bytes()
    def test_idle_connection_status_and_cleanup_are_readonly(self):
        status=self.backend.status()
        self.assertFalse(status['control_ready']);self.assertFalse(status['enabled'])
        self.assertFalse(status['baseline_current'])
        self.backend.stop()
        self.backend.safe.write.assert_not_called()
        self.assertEqual(self.backend.safe.STATE.read_bytes(),self.saved)
    def test_all_actuation_paths_remain_blocked(self):
        for action in (self.backend.prepare,lambda:self.backend.output(1500),self.backend.restore):
            with self.assertRaisesRegex(ValueError,'基线过期'):action()
        self.backend.safe.write.assert_not_called();self.backend.safe.restore.assert_not_called()
        self.assertEqual(self.backend.safe.STATE.read_bytes(),self.saved)
    def test_unknown_exported_channel_is_not_taken_over(self):
        (self.chip/'pwm0').mkdir()
        with self.assertRaisesRegex(RuntimeError,'not idle'):self.backend.status()
        self.backend.safe.write.assert_not_called()
    def test_nonoriginal_pin_is_not_treated_as_verified_idle(self):
        self.backend.safe.register=lambda:0x405
        with self.assertRaisesRegex(RuntimeError,'not idle'):self.backend.status()
        self.backend.safe.write.assert_not_called()


class ProcessTests(unittest.TestCase):
    def start(self):
        p=subprocess.Popen([sys.executable,str(ROOT/'tools/pwm_remote_agent.py'),'--mock'],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
        self.addCleanup(lambda:p.kill() if p.poll() is None else None)
        self.addCleanup(lambda:[s.close() for s in (p.stdin,p.stdout,p.stderr) if not s.closed])
        return p
    def read(self,p,timeout=3):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if select.select([p.stdout],[],[],.1)[0]:
                line=p.stdout.readline()
                if not line:raise AssertionError('agent exited: '+p.stderr.read())
                return json.loads(line)
        self.fail('agent reply timeout')
    def send(self,p,status,seq,action,**values):
        p.stdin.write(json.dumps(dict(action=action,session=status['session'],token=status['token'],seq=seq,**values))+'\n')
        p.stdin.flush()
    def test_actual_pipe_heartbeat_loss_stops_pulses_keeps_connection_and_needs_manual_restart(self):
        p=self.start();s=self.read(p)
        self.assertFalse(s['enabled']);self.assertTrue(s['mock'])
        self.send(p,s,1,'prepare')
        while not s['mode']:s=self.read(p)
        self.send(p,s,2,'set',pulse_us=1520,seconds=0)
        while not s['enabled']:s=self.read(p)
        while not s.get('heartbeat_lost'):s=self.read(p)
        self.assertIsNone(p.poll())
        self.assertFalse(s['enabled']);self.assertTrue(s['mode'])
        self.assertIn('heartbeat',s['reason'])
        self.send(p,s,3,'ping')
        while s.get('heartbeat_lost'):s=self.read(p)
        self.assertFalse(s['enabled'])
        self.send(p,s,4,'set',pulse_us=1500,seconds=0)
        while not s['enabled']:s=self.read(p)
        self.send(p,s,5,'close');p.wait(timeout=3)
        self.assertEqual(p.returncode,0,p.stderr.read())

    def test_rejected_stale_command_stops_output_without_destroying_session(self):
        p=self.start();s=self.read(p)
        self.send(p,s,1,'prepare')
        while not s['mode']:s=self.read(p)
        self.send(p,s,2,'set',pulse_us=1500,seconds=0)
        while not s['enabled']:s=self.read(p)
        stale=dict(s,token='invalid-stale-challenge')
        self.send(p,stale,3,'ping')
        while s.get('type')!='error':s=self.read(p)
        self.assertTrue(s['recoverable'])
        while s.get('type')!='status':s=self.read(p)
        self.assertFalse(s['enabled']);self.assertIsNone(p.poll())
        self.send(p,s,4,'ping');s=self.read(p)
        self.send(p,s,5,'close');p.wait(timeout=3)
        self.assertEqual(p.returncode,0,p.stderr.read())

    def test_eof_terminates_without_waiting_for_duration(self):
        p=self.start();s=self.read(p)
        self.send(p,s,1,'prepare')
        while not s['mode']:s=self.read(p)
        self.send(p,s,2,'set',pulse_us=1520,seconds=60)
        while not s['enabled']:s=self.read(p)
        p.stdin.close();p.wait(timeout=2)
        self.assertEqual(p.returncode,0,p.stderr.read())
    def test_invalid_message_ends_session_and_reports_error(self):
        p=self.start();self.read(p)
        p.stdin.write('[]\n');p.stdin.flush()
        p.wait(timeout=2)
        self.assertNotEqual(p.returncode,0)
        self.assertIn('command must be object',p.stdout.read())


if __name__=='__main__':unittest.main()
