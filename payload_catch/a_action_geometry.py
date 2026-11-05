"""ROS-free geometry and limits for the A-only empty-claw action trial."""
from dataclasses import dataclass
import math
from pathlib import Path

import yaml


@dataclass(frozen=True)
class AActionGeometry:
    expected_mocap_start_mm: tuple[float, float, float]
    start_tolerance_m: tuple[float, float, float]
    travel_mocap_x_m: float
    height_above_takeoff_m: float
    settle_before_pwm_s: float
    max_horizontal_speed_m_s: float
    max_vertical_speed_m_s: float
    waypoint_xy_tolerance_m: float
    waypoint_height_tolerance_m: float
    waypoint_speed_tolerance_m_s: float
    corridor_cross_track_m: float
    corridor_end_margin_m: float
    max_height_above_takeoff_m: float
    pwm_open_us: int
    pwm_output_s: float
    payload_attached: bool
    takeoff_timeout_s: float
    translate_timeout_s: float
    settle_timeout_s: float

    @property
    def expected_px4_start_ned_m(self):
        x, y, z = self.expected_mocap_start_mm
        # Current fixed_lab_navigation: local N/E/D = mocap Y/X/-Z.
        # -0.16 m marker-to-body vertical offset adds +0.16 m to D.
        return y / 1000.0, x / 1000.0, -z / 1000.0 + 0.16

    def target_ned_m(self, origin_ned_m):
        north, east, down = origin_ned_m
        return north, east + self.travel_mocap_x_m, down - self.height_above_takeoff_m

    def inside_corridor(self, origin_ned_m, position_ned_m):
        north, east, down = position_ned_m
        start_n, start_e, start_d = origin_ned_m
        end_e = start_e + self.travel_mocap_x_m
        margin = self.corridor_end_margin_m
        return (abs(north - start_n) <= self.corridor_cross_track_m
                and min(start_e, end_e) - margin <= east <= max(start_e, end_e) + margin
                and start_d - self.max_height_above_takeoff_m <= down <= start_d + .20)

    def validate(self):
        values = (*self.expected_mocap_start_mm, *self.start_tolerance_m,
                  self.travel_mocap_x_m, self.height_above_takeoff_m,
                  self.settle_before_pwm_s, self.max_horizontal_speed_m_s,
                  self.max_vertical_speed_m_s, self.waypoint_xy_tolerance_m,
                  self.waypoint_height_tolerance_m, self.waypoint_speed_tolerance_m_s,
                  self.corridor_cross_track_m, self.corridor_end_margin_m,
                  self.max_height_above_takeoff_m, self.pwm_output_s,
                  self.takeoff_timeout_s, self.translate_timeout_s,
                  self.settle_timeout_s)
        if not all(type(x) in (int, float) and math.isfinite(x) for x in values):
            raise ValueError('geometry contains nonfinite values')
        if (self.travel_mocap_x_m != -2.0 or self.height_above_takeoff_m != 1.0
                or self.payload_attached is not False or self.pwm_open_us != 1700
                or not 1.4 <= self.pwm_output_s <= 1.6
                or self.max_height_above_takeoff_m < self.height_above_takeoff_m + .2):
            raise ValueError('A-only empty-claw trial differs from the approved action')
        if any(x <= 0 for x in self.start_tolerance_m):
            raise ValueError('start tolerance must be positive')
        if not (10 <= self.takeoff_timeout_s <= 40
                and 8 <= self.translate_timeout_s <= 40
                and self.settle_before_pwm_s < self.settle_timeout_s <= 20):
            raise ValueError('invalid action phase timeout')
        return self


def load_action_geometry(path):
    raw = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if raw.pop('schema', None) != 'a_action_solo_v1':
        raise ValueError('wrong A action schema')
    for key in ('expected_mocap_start_mm', 'start_tolerance_m'):
        if not isinstance(raw.get(key), list) or len(raw[key]) != 3:
            raise ValueError(key + ' must have three values')
        raw[key] = tuple(raw[key])
    return AActionGeometry(**raw).validate()
