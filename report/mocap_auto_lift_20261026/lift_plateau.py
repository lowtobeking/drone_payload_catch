"""Pure bounded plateau detector for manual lift evidence; never fabricates phases."""
import math
def stable_phase(z_values, baseline_z, phase):
    if phase not in ("elevated","returned"):
        raise ValueError("Unknown manual phase")
    if len(z_values)<60 or not all(math.isfinite(z) for z in z_values):
        return False
    if max(z_values)-min(z_values)>.015:
        return False
    mean=sum(z_values)/len(z_values)
    if phase=="elevated":
        return .15<baseline_z-mean<1.2
    return abs(mean-baseline_z)<.06
