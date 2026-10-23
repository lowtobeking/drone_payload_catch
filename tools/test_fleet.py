#!/usr/bin/env python3
"""Offline fleet leases, deployment integrity, role isolation and rollback."""
import argparse
import importlib.util
import io
import json
import os
from pathlib import Path
import socket
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch
import yaml
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from payload_catch.fleet_runtime import PeerLease,load_fleet,dds_profile
from tools import fleet_deploy as deploy


def valid_config(directory):
    d=yaml.safe_load((ROOT/'config/fleet.example.yaml').read_text())
    d.update(mission_id='bench',geometry_confirmed=True)
    d['hosts']={'a':dict(hostname='jetson-a',ip='10.41.10.101',system_id=1),
                'b':dict(hostname='jetson-b',ip='10.41.10.102',system_id=2)}
    d['launch'].update(a_lever=[0.,0.,0.],b_lever=[0.,0.,0.],b_offset=[5.,0.,0.])
    (directory/'payload_servo.yaml').write_bytes((ROOT/'config/payload_servo.yaml').read_bytes())
    path=directory/'fleet.yaml';path.write_text(yaml.safe_dump(d))
    return d,path


class LeaseTests(unittest.TestCase):
    def setUp(self):self.lease=PeerLease('a','code-and-config','boot-a')
    def hello(self,**changes):
        d=dict(role='b',fingerprint='code-and-config',boot='boot-b',seq=1,stamp=100,
               local_ok=True,seen_peer_boot='boot-a',peer_rx_age_s=.01);d.update(changes);return d
    def test_bidirectional_echo_required(self):
        self.assertFalse(self.lease.ready(100,10,True))
        self.lease.accept(self.hello(seen_peer_boot=None),100,10)
        self.assertFalse(self.lease.ready(100,10,True))
        self.lease.accept(self.hello(seq=2,stamp=100.01),100.01,10.01)
        self.assertTrue(self.lease.ready(100.01,10.01,True))
    def test_timeout_and_duplicate_do_not_extend_lease(self):
        self.lease.accept(self.hello(),100,10)
        self.assertFalse(self.lease.accept(self.hello(),100.2,10.2))
        self.assertFalse(self.lease.ready(100.31,10.31,True))
    def test_role_release_restart_mismatch_latches_fault(self):
        for changes in ({'role':'a'},{'fingerprint':'other'},{'boot':'restarted'}):
            lease=PeerLease('a','code-and-config','boot-a');lease.accept(self.hello(),100,10)
            self.assertFalse(lease.accept(self.hello(seq=2,stamp=100.01,**changes),100.01,10.01))
            self.assertFalse(lease.ready(100.01,10.01,True));self.assertTrue(lease.fault)
    def test_clock_skew_and_invalid_data_cannot_authorize(self):
        for changes in ({'stamp':99.9},{'stamp':100.1},{'stamp':float('nan')},{'seq':'2'}):
            lease=PeerLease('a','code-and-config','boot-a')
            self.assertFalse(lease.accept(self.hello(**changes),100,10))
            self.assertFalse(lease.ready(100,10,True))
    def test_one_way_failure_expires_reverse_acknowledgement(self):
        self.lease.accept(self.hello(),100,10)
        self.assertTrue(self.lease.ready(100,10,True))
        # Peer still transmits, but has stopped receiving us. Old boot echo alone
        # must not keep our side ready.
        self.lease.accept(self.hello(seq=2,stamp=100.01,peer_rx_age_s=.4),100.01,10.01)
        self.assertFalse(self.lease.ready(100.01,10.01,True))

    def test_local_and_remote_graph_must_be_healthy(self):
        self.lease.accept(self.hello(),100,10)
        self.assertFalse(self.lease.ready(100,10,False))
        self.lease.accept(self.hello(seq=2,stamp=100.01,local_ok=False),100.01,10.01)
        self.assertFalse(self.lease.ready(100.01,10.01,True))


class ConfigTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.d,self.path=valid_config(Path(self.tmp.name))
    def test_template_is_not_flight_ready(self):
        with self.assertRaises(ValueError):load_fleet(ROOT/'config/fleet.example.yaml')
    def test_domain_zero_needs_explicit_measured_confirmation(self):
        self.d['domain_id']=0
        self.path.write_text(yaml.safe_dump(self.d))
        with self.assertRaises(ValueError):load_fleet(self.path)
        self.d['domain_zero_confirmed']=True
        self.path.write_text(yaml.safe_dump(self.d))
        self.assertEqual(load_fleet(self.path)[0]['domain_id'],0)
        self.d['geometry_confirmed']=False
        self.path.write_text(yaml.safe_dump(self.d))
        with self.assertRaises(ValueError):load_fleet(self.path)

    def test_config_fingerprint_changes_with_servo_or_geometry(self):
        _,_,before=load_fleet(self.path)
        self.d['launch']['a_hover'][2]=-5
        self.path.write_text(yaml.safe_dump(self.d))
        self.assertNotEqual(before,load_fleet(self.path)[2])
        before=load_fleet(self.path)[2]
        servo=self.path.parent/'payload_servo.yaml'
        d=yaml.safe_load(servo.read_text());d['payload_servo']['ros__parameters']['open_us']=1610
        servo.write_text(yaml.safe_dump(d))
        self.assertNotEqual(before,load_fleet(self.path)[2])
    def test_duplicate_addresses_bad_ids_and_missing_geometry_rejected(self):
        for edit in ('ip','system_id','geometry'):
            d,path=valid_config(self.path.parent)
            if edit=='geometry':d['geometry_confirmed']=False
            else:d['hosts']['b'][edit]=d['hosts']['a'][edit]
            path.write_text(yaml.safe_dump(d))
            with self.assertRaises(ValueError):load_fleet(path)
    def test_profile_whitelists_interface_and_configures_peers(self):
        import xml.etree.ElementTree as ET
        tree=ET.fromstring(dds_profile('10.41.10.101',['10.41.10.101','10.41.10.102']))
        ns={'f':'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
        self.assertEqual(tree.find('.//f:interfaceWhiteList/f:address',ns).text,'10.41.10.101')
        self.assertEqual(tree.find('.//f:useBuiltinTransports',ns).text,'false')


class DeployTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.home=Path(self.tmp.name);self.bundle=self.home/'release.tar.gz'
        self.release_id=deploy.bundle(ROOT,self.bundle)
    def test_bundle_includes_worktree_pwm_and_split_launch(self):
        files,manifest=deploy.read_bundle(self.bundle)
        self.assertIn('payload_catch/payload_servo.py',files)
        self.assertIn('launch/catch_real_a_launch.py',files)
        self.assertIn('deploy/run_fleet.sh',files)
        self.assertEqual(manifest['release_id'],self.release_id)
        self.assertNotIn('report',files)
    def test_tamper_and_traversal_rejected_before_extraction(self):
        for name in ('../escape','/absolute','payload_catch/a.py'):
            path=self.home/('bad'+str(len(name))+'.tar.gz')
            with tarfile.open(path,'w:gz') as archive:
                info=tarfile.TarInfo(name);info.size=3;archive.addfile(info,io.BytesIO(b'bad'))
            with self.assertRaises((ValueError,KeyError)):deploy.read_bundle(path)
    def test_stage_only_cannot_activate_and_role_cannot_change(self):
        base=self.home/'deploy'
        deploy.install(argparse.Namespace(base=base,bundle=self.bundle,role='a',no_build=True))
        self.assertFalse((base/'current').exists())
        with self.assertRaises(FileNotFoundError):deploy.select(base,self.release_id)
        with self.assertRaises(ValueError):deploy.install(argparse.Namespace(base=base,bundle=self.bundle,role='b',no_build=True))
    def test_activation_and_rollback_preserve_previous_source(self):
        base=self.home/'deploy'
        deploy.install(argparse.Namespace(base=base,bundle=self.bundle,role='a',no_build=True))
        first=base/'releases'/self.release_id
        def mark_built(path):
            (path/'BUILT').write_text(json.dumps(dict(host=socket.gethostname(),architecture=os.uname().machine)))
        mark_built(first)
        deploy.activate(argparse.Namespace(base=base,release=self.release_id,command='activate'))
        files,_=deploy.read_bundle(self.bundle)
        files['README.md']+=b'\nsecond release\n'
        manifest=deploy.make_manifest(files);second_id=manifest['release_id']
        second=base/'releases'/second_id
        second.mkdir();(second/'src').mkdir()
        for name,content in files.items():
            path=second/'src'/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(content)
        (second/'src/FLEET_MANIFEST.json').write_text(json.dumps(manifest))
        (second/'ROLE').write_text('a\n');mark_built(second)
        deploy.activate(argparse.Namespace(base=base,release=second_id,command='activate'))
        self.assertEqual((base/'previous').resolve(),first)
        deploy.activate(argparse.Namespace(base=base,command='rollback'))
        self.assertEqual((base/'current').resolve(),first)
        self.assertTrue(second.exists())
    def test_running_mission_prevents_version_switch(self):
        import fcntl
        base=deploy.base_dir(self.home/'deploy')
        with open(base/'mission.lock','a') as handle:
            fcntl.flock(handle,fcntl.LOCK_EX)
            with self.assertRaises(BlockingIOError):deploy.ensure_stopped(base)


if __name__=='__main__':unittest.main()
