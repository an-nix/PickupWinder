import sys, os
root = os.path.abspath(os.path.join(os.path.dirname(__file__), "src", "rpi"))
if root not in sys.path:
    sys.path.insert(0, root)
from motion.ramp_config import compute_ramp_times, RampConfig
from motion.multi_axis_segment_generator import AxisMotionConfig, MultiAxisSegmentGenerator
accel, cruise, decel = compute_ramp_times(target_rpm=15.0, duration_s=10.0, max_accel_steps_per_s2=5000, max_decel_steps_per_s2=5000, steps_per_rev=6400)
rc = RampConfig(axis_id=1, steps_per_rev=6400, target_rpm=15.0, accel_s=accel, cruise_s=cruise, decel_s=decel)
gen = MultiAxisSegmentGenerator([AxisMotionConfig(1, rc)])
for _, s in zip(range(10), gen):
    if s.steps[0] > 0:
        interval = (s.duration_us * 80) / s.steps[0]
        print(f"duration={s.duration_us} steps={s.steps[0]} interval={interval}")
