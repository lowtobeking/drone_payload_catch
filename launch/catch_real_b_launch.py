"""Distributed role B; same fleet config on both hosts."""
import importlib.util
from pathlib import Path

def generate_launch_description():
    spec=importlib.util.spec_from_file_location('fleet_real',Path(__file__).with_name('catch_real_launch.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    description=module.generate_launch_description('b')
    # The role wrapper is fixed even if callers pass deployment_role:=both.
    from launch.actions import OpaqueFunction
    def force_role(context):
        context.launch_configurations['deployment_role']='b'
    description.entities.insert(0,OpaqueFunction(function=force_role))
    return description
