#!/usr/bin/env python3
"""A-only gates, actual command/state confirmation, leases, boundaries and protocol."""
import json
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as N
import unittest
from dataclasses import replace
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from payload_catch.preflight_health import (HealthAccumulator, evaluate, approval_reason,
                                           FAILSAFE_FATAL, EST_BAD)
from payload_catch.solo_validation import (SoloConfig, SoloSupervisor, SoloMission, SCOPE,
    preflight_errors, receipt_errors, hardware_errors, parameter_issues)
from tools.a_solo_preflight import decode_parameter

IDENTITY = dict(hostname="mock-a", board_serial="test-board", boot_id="test-boot")


def config():
    return SoloConfig(hostname="mock-a", board_serial="test-board",
        site_checked=True, payload_removed=True, positioning="external_local")


def packet(key, timestamp):
    d = dict(timestamp=timestamp)
    if key == "lpos":
        d.update(x=0., y=0., z=0., vx=0., vy=0., vz=0., xy_valid=True, z_valid=True,
            v_xy_valid=True, v_z_valid=True, heading_good_for_control=True,
            dead_reckoning=False, eph=.1, epv=.1)
    elif key == "att":
        d["q"] = [1., 0., 0., 0.]
    elif key == "status":
        d["pre_flight_checks_pass"] = True
    elif key == "failsafe":
        d.update({k:False for k, _ in FAILSAFE_FATAL})
        d["offboard_control_signal_lost"] = True
    elif key == "est":
        d.update({k:False for k in EST_BAD}, cs_tilt_align=True, cs_yaw_align=True)
    elif key == "batt":
        d.update(remaining=.8, warning=0, connected=True)
    elif key == "gps":
        d.update(eph=.1, epv=.1, satellites_used=25)
    elif key == "rc":
        d.update(valid=True, data_source=1)
    return N(**d)


def snapshot(**changes):
    acc = HealthAccumulator()
    for t in (1., 1.1):
        for key in acc.samples:
            m = packet(key, int(t*1e6))
            for name, val in changes.get(key, {}).items():
                setattr(m, name, val)
            acc.record(key, m, t)
    return acc.snapshot(1.1)


def telemetry(**changes):
    d = dict(identity_ok=True, source_ok=True, status_fresh=True, land_fresh=True,
        landed=True, armed=False, nav=0, cells=4, voltage=16.,
        position=[0., 0., 0.], velocity=[0., 0., 0.], heading=.2, tilt=0.,
        reset_counters=[0, 0, 0, 0, 0])
    d.update(changes)
    return d


def receipt(cfg=None):
    cfg = cfg or config()
    return dict(schema=SCOPE, scope=SCOPE, passed=True, checked_at=100.,
        drone_ids=[0], system_id=1, battery_min=.4, rc=True, gps=cfg.gps,
        release_allowed=False, config_digest=cfg.digest, code_digest="code",
        identity=IDENTITY, params_checked=[0],
        param_profile="outdoor" if cfg.gps else "indoor", params_sha256="a"*64)


def ready():
    core = SoloSupervisor(config(), execute=True)
    for now in (1., 4.01):
        core.tick(now, snapshot(), telemetry(), before_errors=[], task_ok=True)
    assert core.phase == "READY"
    assert core.start(4.02, telemetry(), [])
    return core


def fly():
    core = ready()
    for now in (4.1, 6.11):
        ticket = core.tick(now, snapshot(), telemetry(), before_errors=[], task_ok=True, streamed=True)
    assert ticket.action == "offboard"
    ticket.sent(6.11)
    core.acknowledge(176, 0, 255, 191)
    ticket = core.tick(6.12, snapshot(), telemetry(nav=14), before_errors=[], task_ok=True, streamed=True)
    assert ticket.action == "arm"
    ticket.sent(6.12)
    core.acknowledge(400, 0, 255, 191)
    core.tick(6.13, snapshot(status={"pre_flight_checks_pass":False}),
        telemetry(nav=14, armed=True, landed=False), before_errors=[], task_ok=True, streamed=True)
    assert core.phase == "FLY"
    return core


class SoloTests(unittest.TestCase):
    def test_only_a_needed_no_b(self):
        s = snapshot()
        self.assertFalse(preflight_errors(s, telemetry(), config()))
        self.assertFalse(ready().phase == "CHECK")

    def test_missing_critical_data_always_blocks(self):
        s = HealthAccumulator().snapshot(1)
        self.assertTrue(preflight_errors(s, telemetry(), config()))
        for key in ("lpos","att","status","failsafe","est","batt","rc"):
            good = snapshot()
            setattr(good, "n_"+key, 0)
            self.assertTrue(preflight_errors(good, telemetry(), config()), key)

    def test_battery_exact_40_and_nan(self):
        for val, good in ((.39,False), (.4,True), (1.,True), (math.nan,False), (1.1,False)):
            self.assertEqual(not preflight_errors(snapshot(batt={"remaining":val}), telemetry(), config()), good)

    def test_rc_and_local_position_cannot_be_disabled(self):
        for bad in (dict(rc={"valid":False}), dict(rc={"data_source":2}),
                    dict(lpos={"xy_valid":False}), dict(lpos={"dead_reckoning":True})):
            self.assertTrue(preflight_errors(snapshot(**bad), telemetry(), config()))

    def test_gps_optional_only_with_configured_external_position(self):
        cfg = replace(config(), positioning="outdoor_gps")
        self.assertTrue(preflight_errors(snapshot(gps={"satellites_used":0}), telemetry(), cfg))
        self.assertFalse(preflight_errors(snapshot(gps={"satellites_used":0}), telemetry(), config()))

    def test_voltage_and_fc_identity_not_guessed(self):
        self.assertFalse(hardware_errors(telemetry(cells=1), config()))
        for change in (dict(voltage=4.), dict(voltage=math.nan),
                       dict(identity_ok=False), dict(source_ok=False), dict(land_fresh=False)):
            self.assertTrue(hardware_errors(telemetry(**change), config()))

    def test_defaults_cannot_authorize_site(self):
        self.assertTrue(SoloConfig().site_errors(IDENTITY))
        self.assertFalse(config().site_errors(IDENTITY))

    def test_strict_config_bounds(self):
        for change in (dict(height_m=20),dict(hover_s=math.nan),
                       dict(site_checked="true"), dict(expected_cells=True)):
            with self.assertRaises(ValueError):
                replace(config(), **change).validate()

    def test_scoped_receipt_never_unlocks_dual_mission(self):
        self.assertTrue(approval_reason(receipt(), now=100, require_rc=True))
        self.assertFalse(receipt_errors(receipt(), config(), IDENTITY, "code", 100))
        for change in (dict(drone_ids=[0,1]),dict(checked_at=39),
                       dict(checked_at=101),dict(rc=False),dict(params_checked=[]),
                       dict(code_digest="old"),dict(config_digest="old"),dict(release_allowed=True),
                       dict(identity={**IDENTITY,"boot_id":"old"}),dict(params_sha256="g"*64)):
            self.assertTrue(receipt_errors({**receipt(),**change}, config(), IDENTITY, "code", 100))

    def test_check_mode_never_emits_or_starts(self):
        core = SoloSupervisor(config())
        for t in (1.,4.1):
            self.assertIsNone(core.tick(t,snapshot(),telemetry(),before_errors=[],task_ok=True))
        self.assertFalse(core.start(4.2, telemetry(), []))
        core.stop("test", 4.3, kill=True)
        self.assertIsNone(core.tick(4.4,snapshot(),telemetry(),before_errors=[],task_ok=True))

    def test_explicit_start_and_no_automatic_restart(self):
        core = ready()
        core.stop("abort", 4.1)
        self.assertEqual(core.phase,"CANCELLED")
        for t in (5.,9.,20.):
            self.assertIsNone(core.tick(t,snapshot(),telemetry(),before_errors=[],task_ok=True))
        self.assertFalse(core.start(21.,telemetry(),[]))

    def test_prestream_must_be_continuous(self):
        core = ready()
        for t, streamed in ((4.1,True),(5.,False),(6.,True),(7.,True)):
            self.assertIsNone(core.tick(t,snapshot(),telemetry(),before_errors=[],task_ok=True,streamed=streamed))
        ticket = core.tick(8.01,snapshot(),telemetry(),before_errors=[],task_ok=True,streamed=True)
        self.assertEqual(ticket.action,"offboard")

    def test_ack_and_observed_mode_and_actual_armed_required(self):
        core = ready()
        for t in (4.1,6.11):
            ticket=core.tick(t,snapshot(),telemetry(),before_errors=[],task_ok=True,streamed=True)
        ticket.sent(6.11)
        self.assertFalse(core.acknowledge(176,0,1,1))
        self.assertEqual(core.phase,"MODE")
        core.acknowledge(176,0,255,191)
        core.tick(6.12,snapshot(),telemetry(nav=0),before_errors=[],task_ok=True,streamed=True)
        self.assertEqual(core.phase,"MODE")
        ticket=core.tick(6.13,snapshot(),telemetry(nav=14),before_errors=[],task_ok=True,streamed=True)
        ticket.sent(6.13)
        core.acknowledge(400,0,255,191)
        core.tick(6.14,snapshot(),telemetry(nav=14),before_errors=[],task_ok=True,streamed=True)
        self.assertEqual(core.phase,"ARM")
        core.tick(6.15,snapshot(status={"pre_flight_checks_pass":False}),
                  telemetry(nav=14,armed=True,landed=False),before_errors=[],task_ok=True)
        self.assertEqual(core.phase,"FLY")

    def test_rejected_mode_does_not_attempt_arm(self):
        core=ready()
        for t in (4.1,6.11):
            ticket=core.tick(t,snapshot(),telemetry(),before_errors=[],task_ok=True,streamed=True)
        ticket.sent(6.11)
        core.acknowledge(176,2,255,191)
        self.assertIsNone(core.tick(6.12,snapshot(),telemetry(),before_errors=[],task_ok=True))
        self.assertEqual(core.phase,"CANCELLED")
        self.assertFalse(core.arm_attempted)

    def test_arming_unknown_result_lands_instead_of_rearming(self):
        core=fly()
        core.phase="ARM"
        core.stop("uncertain arm", 7)
        self.assertEqual(core.phase,"LAND")
        self.assertFalse(core.start(8,telemetry(),[]))

    def test_task_loss_and_sensor_fault_land(self):
        for task_ok, snap in ((False,snapshot()),(True,snapshot(rc={"valid":False})),
                              (True,snapshot(lpos={"eph":10.}))):
            core=fly()
            ticket=core.tick(7,snap,telemetry(armed=True,nav=14,landed=False),
                             before_errors=[],task_ok=task_ok)
            self.assertEqual(core.phase,"LAND")
            self.assertEqual(ticket.command,21)

    def test_operator_stop_during_flight_lands(self):
        core=fly()
        core.stop("operator stop", 7)
        self.assertEqual(core.phase,"LAND")
        self.assertFalse(core.permit)

    def test_explicit_kill_is_separate_and_latched(self):
        core=fly()
        core.stop("power cut",7,kill=True)
        ticket=core.tick(7.01,snapshot(),telemetry(armed=True,nav=14,landed=False),
                         before_errors=[],task_ok=True)
        self.assertEqual((core.phase,ticket.command),("KILL",185))
        core.stop("land must not override kill",7.1)
        self.assertEqual(core.phase,"KILL")

    def test_manual_takeover_never_reasserts_offboard(self):
        core=fly()
        self.assertIsNone(core.tick(7,snapshot(),telemetry(armed=True,nav=3,landed=False),
                                   before_errors=[],task_ok=True))
        self.assertEqual(core.phase,"MANUAL")
        core.stop("late task request",7.1)
        self.assertEqual(core.phase,"MANUAL")
        self.assertFalse(core.start(8,telemetry(),[]))

    def test_fence_tilt_reset_or_absolute_timeout_land(self):
        for changes, t in ((dict(position=[2.1,0,0]),7),(dict(position=[0,0,-2.1]),7),
            (dict(tilt=26),7),(dict(reset_counters=[1,0,0,0,0]),7),({},70)):
            core=fly()
            core.tick(t,snapshot(),telemetry(armed=True,nav=14,landed=False,**changes),
                      before_errors=[],task_ok=True)
            self.assertEqual(core.phase,"LAND")

    def test_timer_alone_never_means_landed(self):
        core=fly()
        core.stop("normal",7)
        ticket=core.tick(7.01,snapshot(),telemetry(armed=True,nav=14,landed=False),
                         before_errors=[],task_ok=True)
        ticket.sent(7.01)
        core.acknowledge(21,0,255,191)
        core.tick(8,snapshot(),telemetry(armed=True,nav=18,landed=False),before_errors=[],task_ok=True)
        self.assertEqual(core.phase,"LAND")
        for t in (9,10.1):
            core.tick(t,snapshot(),telemetry(armed=False,nav=18,landed=True),
                      before_errors=[],task_ok=True)
        self.assertEqual(core.phase,"DONE")

    def test_missing_landing_ack_is_explicit_failure(self):
        core=fly()
        core.stop("normal",7)
        for t in (8,10,20,38):
            core.tick(t,snapshot(),telemetry(nav=18),before_errors=[],task_ok=True)
        self.assertEqual(core.phase,"FAILED")

    def test_mission_bounded_velocity_and_continuous_hover(self):
        cfg=config()
        mission=SoloMission(cfg)
        command,land=mission.velocity(0,"FLY",[0,0,0],[1,1,0],[0,0,0])
        self.assertLessEqual(math.hypot(*command[:2]),cfg.xy_speed+1e-9)
        self.assertEqual(command[2],-cfg.climb_speed)
        for t in (1,3.1,8):
            self.assertFalse(mission.velocity(t,"FLY",[0,0,0],[0,0,-1],[0,0,0])[1])
        self.assertFalse(mission.velocity(9,"FLY",[0,0,0],[.2,0,-1],[0,0,0])[1])
        self.assertIsNone(mission.hover_since)
        self.assertTrue(mission.velocity(21,"FLY",[0,0,0],[0,0,-1],[0,0,0])[1])
        self.assertEqual(mission.last_phase,"TAKEOFF_TIMEOUT")

    def test_20cm_takeoff_velocity_tapers_to_target(self):
        mission=SoloMission(replace(config(),height_m=.2))
        initial,land=mission.velocity(0,"FLY",[0,0,0],[0,0,0],[0,0,0])
        near,_=mission.velocity(.1,"FLY",[0,0,0],[0,0,-.16],[0,0,0])
        self.assertFalse(land)
        self.assertAlmostEqual(initial[2],-.3)
        self.assertAlmostEqual(near[2],-.06)

    def test_mission_hover_finishes_and_never_resumes(self):
        mission=SoloMission(config())
        for t in (0,2.1):
            self.assertFalse(mission.velocity(t,"FLY",[0,0,0],[0,0,-1],[0,0,0])[1])
        self.assertTrue(mission.velocity(12.2,"FLY",[0,0,0],[0,0,-1],[0,0,0])[1])
        self.assertEqual(mission.velocity(13,"FLY",[0,0,0],[0,0,0],[0,0,0]),(None,False))


    def test_live_fcu_acceptance_limits_are_required(self):
        from payload_catch.fcu_params import PRESETS
        parameters = {}
        for group in ("failsafe","thrust","limits_indoor","geofence_indoor","battery","ekf2_mocap"):
            parameters.update({name:value for name,value,_ in PRESETS[group]})
        parameters.update(MPC_THR_HOVER=.3, GF_ACTION=4, RTL_RETURN_ALT=2., SENS_BOARD_ROT=0,
            COM_RC_IN_MODE=0, RC_MAP_KILL_SW=8, RC_MAP_FLTMODE=5, MAV_SYS_ID=1,
            BAT1_N_CELLS=4, CBRK_FLIGHTTERM=0, COM_DISARM_LAND=2.)
        parameters.update({"COM_FLTMODE"+str(i):0 for i in range(1,7)})
        self.assertFalse([r for r in parameter_issues(parameters,config()) if r["level"]=="fail"])
        for changes in (dict(MPC_LAND_SPEED=3.),dict(MPC_TKO_SPEED=2.),
                        dict(COM_DISARM_LAND=0),dict(CBRK_FLIGHTTERM=121212),
                        dict(MAV_SYS_ID=2)):
            self.assertTrue([r for r in parameter_issues({**parameters,**changes},config())
                             if r["level"]=="fail"])

    def test_integer_bytewise_decoding_matches_actual_px4(self):
        for value in (0,1,4,-1,121212):
            raw=struct.unpack("<f",struct.pack("<i",value))[0]
            self.assertEqual(decode_parameter(raw,6,True),value)
        self.assertEqual(decode_parameter(.3,9,True),.3)
        self.assertEqual(decode_parameter(4.,6,False),4)
        with self.assertRaises(ValueError):
            decode_parameter(4.5,6,False)
        with self.assertRaises(ValueError):
            decode_parameter(math.nan,9,True)


if __name__ == "__main__":
    unittest.main()
