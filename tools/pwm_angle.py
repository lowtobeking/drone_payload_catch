"""ES08MD II nominal relative angles from the supplied datasheet, page 2.

1000..2000us gives 90 +/-10 degrees TOTAL travel; 1500us is neutral.
Nominal linear interpolation is a command estimate, never position feedback.
Actual shaft angles/direction/installed mechanical limits need measurement.
900..2100us is the specified maximum pulse range (110 +/-10 deg total);
this bench intentionally stays inside the 1000..2000us normal travel range.
"""
import math

REFERENCE_US = 1500
US_PER_DEGREE = 1000.0 / 90.0
DEFAULT_LIMITS = (1400, 1600)  # Nominal +/-9 deg software bench limit.
EXTENDED_LIMITS = (1000, 2000)  # Normal nominal +/-45 deg, explicit confirmation.


def pulse_to_angle(pulse):
    value = float(pulse)
    if isinstance(pulse, bool) or not math.isfinite(value):
        raise ValueError('脉宽必须为有限数值')
    return (value - REFERENCE_US) / US_PER_DEGREE


def angle_limits(expanded=False):
    return tuple(pulse_to_angle(p) for p in (EXTENDED_LIMITS if expanded else DEFAULT_LIMITS))


def angle_to_pulse(angle, expanded=False):
    value = float(angle)
    if isinstance(angle, bool) or not math.isfinite(value):
        raise ValueError('角度必须为有限数值')
    lower, upper = angle_limits(expanded)
    if not lower <= value <= upper:
        raise ValueError('当前允许标称相对角度：%g° 至 %g°' % (lower, upper))
    return round(REFERENCE_US + value * US_PER_DEGREE)
