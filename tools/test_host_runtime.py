#!/usr/bin/env python3
"""Offline checks that adoption never tolerates a different controller/domain."""
import copy
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from payload_catch.host_runtime import load_profile,validate_observed,environment_text


class HostRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.d=load_profile(ROOT/'config/jetson_a.runtime.yaml')
        d=self.d
        self.s=dict(hostname=d['hostname'],overlay_exists=True,px4_msgs_commit=d['px4_msgs_commit'],
            px4_msgs_clean=True,environment_domain='0',environment_rmw='rmw_fastrtps_cpp',ros_localhost_only='0',
            interfaces={'eno1':[d['network']['local_ip']],'mesh':[d['network']['mesh_ip']]},
            udp_owners={'8888':[1003],'14550':[1002]},agent_pids=[1003],
            agent=dict(pid=1003,active='active',enabled='enabled',argv=[d['agent']['executable'],'udp4','-p','8888'],
                environment={'ROS_DOMAIN_ID':'0','RMW_IMPLEMENTATION':'rmw_fastrtps_cpp','LD_LIBRARY_PATH':d['managed_prefix']+'/lib'}),
            relay=dict(pid=1002,active='active',enabled='enabled',argv=['/usr/bin/python3',d['relay']['script'],'--fc-ip',d['network']['fc_ip'],'--gcs-ip',d['relay']['ground_station_ip'],'--port','14550'],
                environment={'MAV_RELAY_FC':d['network']['fc_ip']},script_sha256=d['relay']['sha256']))

        self.s.update(managed_tree_protected=True,agent_bundle_intact=True,
            legacy_services={n:dict(active='inactive',enabled='masked') for n in d['legacy_units']})

    def test_legacy_service_reactivation_or_unprotected_runtime_fails(self):
        for mutation in ({'managed_tree_protected':False},{'agent_bundle_intact':False},
            {'legacy_services':{'microxrce-agent.service':dict(active='active',enabled='enabled')}}):
            s=copy.deepcopy(self.s);s.update(mutation)
            self.assertTrue(validate_observed(self.d,s))

    def test_valid_existing_services_are_adopted(self):
        self.assertEqual(validate_observed(self.d,self.s),[])

    def test_terminal_domain_42_or_agent_mismatch_fails(self):
        for target in ('terminal','agent'):
            s=copy.deepcopy(self.s)
            if target=='terminal':s['environment_domain']='42'
            else:s['agent']['environment']['ROS_DOMAIN_ID']='42'
            self.assertTrue(validate_observed(self.d,s))

    def test_missing_duplicate_or_foreign_port_owner_fails(self):
        for owners in ([],[4321],[1003,4321]):
            s=copy.deepcopy(self.s);s['udp_owners']['8888']=owners
            self.assertTrue(validate_observed(self.d,s))
        self.s['agent_pids']=[1003,4321]
        self.assertTrue(validate_observed(self.d,self.s))

    def test_stopped_or_wrong_agent_transport_fails(self):
        for mutation in ({'active':'inactive'},{'argv':['MicroXRCEAgent','serial','--dev','/dev/ttyTHS1']}):
            s=copy.deepcopy(self.s);s['agent'].update(mutation)
            self.assertTrue(validate_observed(self.d,s))

    def test_message_drift_and_local_modification_fail(self):
        for mutation in ({'px4_msgs_commit':'0'*40},{'px4_msgs_clean':False},{'overlay_exists':False}):
            s=copy.deepcopy(self.s);s.update(mutation)
            self.assertTrue(validate_observed(self.d,s))

    def test_relay_wrong_fc_gcs_or_modified_source_fails(self):
        for field in ('fc','gcs','hash'):
            s=copy.deepcopy(self.s)
            if field=='fc':s['relay']['argv'][3]='192.168.77.2'
            elif field=='gcs':s['relay']['argv'][5]='192.168.1.1'
            else:s['relay']['script_sha256']='0'*64
            self.assertTrue(validate_observed(self.d,s))

    def test_host_and_network_identity_fail_closed(self):
        for mutation in ({'hostname':'other'},{'interfaces':{'eno1':['10.41.10.101']}}):
            s=copy.deepcopy(self.s);s.update(mutation)
            self.assertTrue(validate_observed(self.d,s))

    def test_fleet_domain_or_mesh_mismatch_fails(self):
        fleet=dict(domain_id=0,hosts={'a':dict(hostname=self.d['hostname'],system_id=1,ip='192.168.1.101')})
        self.assertEqual(validate_observed(self.d,self.s,fleet=fleet),[])
        fleet['domain_id']=42
        self.assertTrue(validate_observed(self.d,self.s,fleet=fleet))
        fleet['domain_id']=0;fleet['hosts']['a']['ip']='192.168.1.102'
        self.assertTrue(validate_observed(self.d,self.s,fleet=fleet))

    def test_environment_is_explicit_and_shell_quoted(self):
        text=environment_text(self.d,ROOT/'config/jetson_a.runtime.yaml')
        self.assertIn('export ROS_DOMAIN_ID=0\n',text)
        self.assertIn('export ROS_LOCALHOST_ONLY=0\n',text)
        self.assertIn('unset FASTRTPS_DEFAULT_PROFILES_FILE',text)
        d=copy.deepcopy(self.d);d['px4_msgs_setup']='/tmp/a $(touch unsafe)/setup.bash'
        self.assertIn("PX4_MSGS_SETUP='/tmp/a $(touch unsafe)/setup.bash'",environment_text(d,'profile.yaml'))


if __name__=='__main__':unittest.main()
