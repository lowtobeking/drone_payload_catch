#!/usr/bin/env python3
"""Evaluate real launch parameters without starting nodes or accessing PWM."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
spec = importlib.util.spec_from_file_location('real_launch', ROOT / 'launch/catch_real_launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
context = LaunchContext()
for action in module.generate_launch_description().entities:
    if isinstance(action, DeclareLaunchArgument):
        action.execute(context)


def params(node):
    return evaluate_parameters(context, node._Node__parameters)


nodes = module._nodes(context)
assert len(nodes) == 6
assert params(nodes[1])[0]['release_delay_s'] == 1.5
assert params(nodes[4])[0]['release_lead'] == 1.7
assert params(nodes[5])[0]['require_hardware_status'] is True
assert params(nodes[4])[0]['release_confirmation_mode']=='timed'
assert params(nodes[5])[0]['release_confirmation_mode']=='timed'
with tempfile.TemporaryDirectory() as directory:
    config_path = Path(directory) / 'servo.yaml'
    base = yaml.safe_load((ROOT / 'config/payload_servo.yaml').read_text())
    config = base['payload_servo']['ros__parameters']
    config.update(enabled=True, calibrated=True, delay_reference='local_command_direct')
    context.launch_configurations['servo_config'] = str(config_path)
    config_path.write_text(yaml.safe_dump(base))
    nodes = module._nodes(context)
    assert params(nodes[1])[0]['release_delay_s'] == 1.5
    assert abs(params(nodes[4])[0]['release_lead'] - 1.7) < 1e-9
    assert abs(params(nodes[5])[0]['release_lead'] - 1.7) < 1e-9
    # A new direct-output measurement must propagate to both aircraft and guard.
    changed=dict(config,release_delay_s=.63)
    config_path.write_text(yaml.safe_dump({'payload_servo': {'ros__parameters': changed}}))
    nodes=module._nodes(context)
    assert params(nodes[1])[0]['release_delay_s']==.63
    assert abs(params(nodes[4])[0]['release_lead']-.83)<1e-9
    assert abs(params(nodes[5])[0]['release_lead']-.83)<1e-9
    config_path.write_text(yaml.safe_dump(base))
    for changes in ({'calibrated': False}, {'backend': 'mock'}, {'open_us': 2100}, {'delay_reference': 'gui_apply'}, {'delay_reference': 'unmeasured_direct'}):
        bad = dict(config)
        bad.update(changes)
        config_path.write_text(yaml.safe_dump({'payload_servo': {'ros__parameters': bad}}))
        try:
            module._nodes(context)
        except ValueError:
            pass
        else:
            raise AssertionError(changes)
    config_path.write_text(yaml.safe_dump(base))
    context.launch_configurations['coord_mode'] = 'direct'
    try:
        module._nodes(context)
    except ValueError:
        pass
    else:
        raise AssertionError('uncoordinated real PWM release allowed')
print('Launch evaluation PASS: disabled default, shared delay compensation, calibration/mock/pulse/mode gates')
