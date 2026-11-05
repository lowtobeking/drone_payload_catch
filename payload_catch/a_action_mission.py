"""ROS-free A-only takeoff, translation, settle, empty-claw action plan."""
import math


class AActionMission:
    def __init__(self, geometry):
        self.geometry = geometry.validate()
        self.phase = 'WAIT'
        self.origin = None
        self.stable_since = None
        self.action_sent = False
        self.stop_latched = False
        self.phase_started = None
        self.flight_started = None
        self.translate_started = None
        self.failure_reason = ''

    def start(self, origin_ned_m):
        expected = self.geometry.expected_px4_start_ned_m
        if (len(origin_ned_m) != 3 or
                any(not math.isfinite(x) for x in origin_ned_m) or
                any(abs(origin_ned_m[i]-expected[i]) > self.geometry.start_tolerance_m[i]
                    for i in range(3))):
            raise ValueError('A outside measured mocap start region')
        self.origin = tuple(origin_ned_m)
        self.phase = 'TAKEOFF'

    def _abort(self, reason):
        self.failure_reason = reason
        self.phase = 'ABORT'
        self.stop_latched = True
        return None, 'land'

    def tick(self, now, position_ned_m, velocity_ned_m, *, flight_healthy):
        if self.stop_latched or self.origin is None:
            return None, None
        if not math.isfinite(now):
            return self._abort('invalid monotonic time')
        if self.flight_started is None:
            self.flight_started = self.phase_started = now
        if now < self.flight_started or now < self.phase_started:
            return self._abort('monotonic time reversed')
        g = self.geometry
        if (not flight_healthy or len(position_ned_m) != 3 or len(velocity_ned_m) != 3
                or any(not math.isfinite(x) for x in (*position_ned_m, *velocity_ned_m))
                or not g.inside_corridor(self.origin, position_ned_m)):
            return self._abort('health lost or flight corridor exceeded')
        elapsed = now - self.phase_started
        if ((self.phase == 'TAKEOFF' and elapsed > g.takeoff_timeout_s)
                or (self.phase == 'TRANSLATE' and now - self.translate_started > g.translate_timeout_s)
                or (self.phase == 'SETTLE' and elapsed > g.settle_timeout_s)
                or (self.phase == 'PWM_ACTION' and elapsed > g.pwm_output_s + 1.0)):
            return self._abort(self.phase + ' timeout')
        end = g.target_ned_m(self.origin)
        takeoff = (self.origin[0], self.origin[1], end[2])
        target = takeoff if self.phase == 'TAKEOFF' else end
        error = [target[i]-position_ned_m[i] for i in range(3)]
        xy_dist = math.hypot(*error[:2])
        speed = math.sqrt(sum(v*v for v in velocity_ned_m))
        stable = (xy_dist <= g.waypoint_xy_tolerance_m
                  and abs(error[2]) <= g.waypoint_height_tolerance_m
                  and speed <= g.waypoint_speed_tolerance_m_s)
        if self.phase == 'TAKEOFF' and stable:
            self.phase = 'TRANSLATE'
            self.phase_started = now
            self.translate_started = now
            self.stable_since = None
        elif self.phase == 'TRANSLATE' and stable:
            self.phase = 'SETTLE'
            self.phase_started = now
            self.stable_since = now
        elif self.phase == 'SETTLE':
            if not stable:
                self.phase = 'TRANSLATE'
                self.stable_since = None
                self.phase_started = now
            elif now-self.stable_since >= g.settle_before_pwm_s:
                self.phase = 'PWM_ACTION'
                self.phase_started = now
                self.action_sent = True
                return [0., 0., 0.], 'open'
        elif self.phase == 'PWM_ACTION':
            if not stable:
                return self._abort('position or speed lost during PWM action')
            return [0., 0., 0.], None
        if self.phase != 'TAKEOFF':
            target = end
            error = [target[i]-position_ned_m[i] for i in range(3)]
        xy = [1.0*error[0], 1.0*error[1]]
        scale = min(1., g.max_horizontal_speed_m_s/max(math.hypot(*xy), 1e-9))
        return [xy[0]*scale, xy[1]*scale,
                max(-g.max_vertical_speed_m_s,
                    min(g.max_vertical_speed_m_s, 1.5*error[2]))], None

    def action_complete(self, confirmed):
        if self.phase != 'PWM_ACTION':
            raise ValueError('PWM action was not requested')
        self.phase = 'LAND_REQUESTED' if confirmed else 'ABORT'
        self.stop_latched = True
        return 'land'
