"""Pure circular yaw plateau check; negative NED yaw is a left turn."""
import math
def wrap_degrees(angle):
    return math.degrees(math.atan2(math.sin(math.radians(angle)),math.cos(math.radians(angle))))
def stable_left(values,baseline):
    if len(values)<60 or not all(math.isfinite(x) for x in values):
        return False
    deltas=[wrap_degrees(x-baseline) for x in values]
    return max(deltas)-min(deltas)<3 and -120<sum(deltas)/len(deltas)<-30
