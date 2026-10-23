#!/usr/bin/env python3
"""Capture pin31 as INPUT or analyze saved edges. NEVER configures/outputs PWM.

For pin33->pin31 loopback only after disconnecting servo signal AND servo power.
GPIO timestamps estimate digital timing, not voltage/noise or servo position.
"""
import argparse
import json
import math
from pathlib import Path
import re
import statistics
import subprocess


def describe(values):
    return None if not values else dict(count=len(values),min=min(values),
                                       median=statistics.median(values),max=max(values))


def analyze_edges(text):
    """gpiomon -F '%s %n %e': seconds, nanoseconds, edge(1=rise/0=fall)."""
    widths=[];periods=[];events=[];anomalies=[]
    last_stamp=last_rise=pending_rise=None
    last_edge=None
    for number,line in enumerate(text.splitlines(),1):
        if not line.strip():continue
        fields=line.split()
        if len(fields)!=3:raise ValueError('invalid edge record at line '+str(number))
        seconds,nanoseconds,edge=map(int,fields)
        if seconds<0 or not 0<=nanoseconds<1_000_000_000 or edge not in (0,1):
            raise ValueError('invalid timestamp/edge at line '+str(number))
        stamp=seconds*1_000_000_000+nanoseconds
        if last_stamp is not None and stamp<=last_stamp:
            raise ValueError('timestamps do not increase at line '+str(number))
        if last_edge==edge:anomalies.append('repeated edge at line '+str(number))
        if edge==1:
            if last_rise is not None and last_edge==0:
                periods.append((stamp-last_rise)/1000)
            pending_rise=last_rise=stamp
        elif pending_rise is not None:
            widths.append((stamp-pending_rise)/1000)
            pending_rise=None
        # A capture can start with a falling edge or end with a rising edge.
        last_stamp,last_edge=stamp,edge
        events.append([seconds,nanoseconds,edge])
    median_period=statistics.median(periods) if periods else None
    result=dict(edge_count=len(events),high_time_us=describe(widths),period_us=describe(periods),
                frequency_hz=1_000_000/median_period if median_period else None,
                anomalies=anomalies,
                timing_samples_usable=len(widths)>=10 and len(periods)>=9 and not anomalies,
                limitation='Kernel GPIO event timing; no voltage/noise or mechanical position measurement.')
    if not events:result['reason']='No edges: PWM may be disabled, wiring absent, or input path invalid; do not infer servo failure.'
    elif not result['timing_samples_usable']:result['reason']='Too few clean samples or missing/repeated edges; investigate capture quality before judging PWM.'
    return result


def capture_input(seconds):
    comp=Path('/proc/device-tree/compatible').read_bytes()
    if b'nvidia,p3768-0000+p3767-0001\0' not in comp:
        raise RuntimeError('not the verified Orin NX carrier/module')
    info=subprocess.run(['gpioinfo','gpiochip0'],capture_output=True,text=True,check=True).stdout
    line=next(s for s in info.splitlines() if re.match(r'\s*line\s+106:',s))
    if '"PQ.06"' not in line or 'unused' not in line or 'input' not in line or '[used]' in line:
        raise RuntimeError('pin31/PQ.06 not an unused input; do not take over')
    pinconf=Path('/sys/kernel/debug/pinctrl/2430000.pinmux/pinconf-groups').read_text()
    block=next(b for b in re.split(r'(?m)(?=^\d+ \()',pinconf)
               if '(soc_gpio33_pq6):' in b.split('\n',1)[0])
    if not all(x in block for x in ('tristate=1','enable-input=1','gpio-mode=0','function=rsvd0')):
        raise RuntimeError('pin31 is not in the verified input configuration')
    cmd=['gpiomon','-b','-n','1000','-F','%s %n %e','gpiochip0','106']
    process=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE,text=True)
    timed_out=False
    try:
        stdout,stderr=process.communicate(timeout=seconds)
    except subprocess.TimeoutExpired:
        timed_out=True;process.terminate()
        try:stdout,stderr=process.communicate(timeout=1)
        except subprocess.TimeoutExpired:
            process.kill();stdout,stderr=process.communicate(timeout=1)
    finally:
        if process.poll() is None:
            process.kill();process.communicate(timeout=1)
    if not timed_out and process.returncode!=0:
        raise RuntimeError('gpiomon failed: '+stderr.strip())
    result=analyze_edges(stdout)
    result.update(input_pin=31,chip='gpiochip0',line=106,requested_seconds=seconds,
                  event_limit=1000,raw_edges=stdout,
                  pwm_output_actions='NONE; capture tool only requests pin31 INPUT and releases it on exit')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    mode=p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--analyze',type=Path,help='analyze a saved gpiomon edge file; no hardware')
    mode.add_argument('--capture',action='store_true',help='temporarily request pin31 INPUT only')
    p.add_argument('--servo-isolated',action='store_true',help='operator confirms servo signal AND power disconnected')
    p.add_argument('--seconds',type=float,default=3)
    p.add_argument('--output',type=Path,help='optional result JSON file')
    args=p.parse_args()
    if args.capture:
        if not args.servo_isolated:p.error('servo signal AND power must be disconnected before loopback')
        if not math.isfinite(args.seconds) or not .5<=args.seconds<=10:p.error('capture time must be 0.5..10 seconds')
        result=capture_input(args.seconds)
    else:result=analyze_edges(args.analyze.read_text(encoding='utf-8'))
    text=json.dumps(result,ensure_ascii=False,indent=2)+'\n'
    if args.output:args.output.write_text(text,encoding='utf-8')
    print(text,end='')


if __name__=='__main__':main()
