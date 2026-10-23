#!/usr/bin/env python3
"""Evaluate both role launches with a fake host; no processes or hardware start."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch_ros.utilities import evaluate_parameters
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from tools.test_fleet import valid_config


def load(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'launch'/name)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


with tempfile.TemporaryDirectory() as directory:
    fleet,path=valid_config(Path(directory))
    from payload_catch.fleet_runtime import load_fleet
    digest=load_fleet(path)[2]
    for role in ('a','b'):
        context=LaunchContext()
        module=load('catch_real_launch.py')
        description=load('catch_real_'+role+'_launch.py').generate_launch_description()
        context.launch_configurations['deployment_role']='both' # Role wrappers must resist this.
        for action in description.entities:
            if isinstance(action,DeclareLaunchArgument):action.execute(context)
            elif isinstance(action,OpaqueFunction) and len(context.launch_configurations)==1:action.execute(context)
        assert context.launch_configurations['deployment_role']==role
        context.launch_configurations.update(fleet_config=str(path),release_id='a'*64)
        host=fleet['hosts'][role]
        with patch.dict(os.environ,ROS_DOMAIN_ID='42',ROS_LOCALHOST_ONLY='0',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',FLEET_CONFIG_DIGEST=digest,FLEET_ROLE=role),\
             patch.object(module.socket,'gethostname',return_value=host['hostname']),\
             patch.object(module.subprocess,'check_output',return_value=json.dumps([{'addr_info':[{'local':host['ip']}]}])):
            nodes=module._nodes(context)
        # Inspect substitutions through their perform method instead of repr.
        from launch.utilities import perform_substitutions
        def executable(node):
            v=node.node_executable
            return v if isinstance(v,str) else perform_substitutions(context,v)
        executables=[executable(node) for node in nodes]
        expected={'fleet_link','safety_supervisor','release_guard','a_node','payload_servo'} if role=='a' else {'fleet_link','relnav_node','b_node'}
        assert set(executables)==expected,(role,executables)
        task=next(n for n in nodes if executable(n)==role+'_node')
        p=evaluate_parameters(context,task._Node__parameters)[0]
        assert p['require_fleet_link'] is True and p['drone_id']==(0 if role=='a' else 1)
        assert p['preflight_require_rc'] is True
    print('Split launch PASS: fixed roles, disjoint ownership, required communication and existing preflight gates')
