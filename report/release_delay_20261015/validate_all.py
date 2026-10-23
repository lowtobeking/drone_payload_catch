from pathlib import Path
import subprocess,json,hashlib
root=Path('/home/caolihao/drone_payload_catch')
report=root/'report/release_delay_20261015'
checks={}
for name,command in [('ros',['bash','report/release_delay_20261015/validate_ros.sh']),
                     ('quick',['bash','tools/run_checks.sh','--quick']),
                     ('diff',['git','diff','--check'])]:
    result=subprocess.run(command,cwd=root,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    (report/(name+'.log')).write_text(result.stdout)
    checks[name]={'exit_code':result.returncode,'log':name+'.log'}
    print(name,'PASS' if result.returncode==0 else 'FAIL',flush=True)
    if result.returncode:print(result.stdout[-6000:],flush=True)
files=['payload_catch/payload_servo.py','payload_catch/servo_pwm.py','payload_catch/release_guard.py',
       'payload_catch/payload_servo_node.py','payload_catch/b_node.py','launch/catch_real_launch.py',
       'config/payload_servo.yaml','tools/test_payload_servo.py','tools/test_safety_runtime.py']
(report/'validation.json').write_text(json.dumps(dict(checks=checks,hardware_access=False,
    observed_delay_s=1.5,reference='gui_apply',target_us=1700,neutral_us=1500,slew_us_s=100,
    flight_enabled=False,flight_calibrated=False,
    sha256={f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in files}),indent=2))
raise SystemExit(any(check['exit_code'] for check in checks.values()))
