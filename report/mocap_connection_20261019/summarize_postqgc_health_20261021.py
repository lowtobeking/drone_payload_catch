from pathlib import Path
import json
work=Path(__file__).resolve().parent
d=json.loads((work/'flight_health_postqgc_20261021.stdout.log').read_text(encoding='utf-8'))
r=d['raw']
print(json.dumps({
 'identity':d['identity'],
 'battery':{k:r['batt'].get(k) for k in ('remaining','voltage_v','cell_count','faults','warning')},
 'status':{k:r['status'].get(k) for k in ('arming_state','nav_state','failsafe','gcs_connection_lost','pre_flight_checks_pass','power_input_valid')},
 'rc':{k:r['rc'].get(k) for k in ('valid','data_source')},
 'position':{k:r['lpos'].get(k) for k in ('xy_valid','v_xy_valid','z_valid','v_z_valid','heading_good_for_control','dead_reckoning')},
 'ev_fusion':{k:r['est'].get(k) for k in ('cs_ev_pos','cs_ev_vel','cs_ev_yaw','cs_ev_hgt')},
 'active_failsafe_flags':{k:v for k,v in r['failsafe'].items() if v is True},
 'site_errors':d['site_errors'],
 'configured_positioning':d['configured_positioning'],
 'configured_gate_errors':d['configured_gate_errors'],
 'telemetry':d['telemetry'],
 'publishers':d['publishers'],
},ensure_ascii=False,indent=2))
