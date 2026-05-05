import sys, os
root = os.path.abspath(os.path.join(os.path.dirname(__file__), "src", "rpi"))
if root not in sys.path:
    sys.path.insert(0, root)
from motion.ramp_config import compute_ramp_times, RampConfig
from motion.multi_axis_segment_generator import AxisMotionConfig, MultiAxisSegmentGenerator
accel, cruise, decel = compute_ramp_times(target_rpm=10.0, duration_s=10.0, max_accel_steps_per_s2=5000, max_decel_steps_per_s2=5000, steps_per_rev=6400)
print(f"10 RPM times: accel={accel} cruise={cruise} decel={decel}")
accel15, cruise15, decel15 = compute_ramp_times(target_rpm=15.0, duration_s=10.0, max_accel_steps_per_s2=5000, max_decel_steps_per_s2=5000, steps_per_rev=6400)
print(f"15 RPM times: accel={accel15} cruise={cruise15} decel={decel15}")
rc10 = RampConfig(axis_id=1, steps_per_rev=6400, target_rpm=10.0, accel_s=accel, cruise_s=cruise, decel_s=decel)
gen10 = MultiAxisSegmentGenerator([AxisMotionConfig(1, rc10)])
steps10 = sum(s.steps[0] for _, s in zip(range(100), gen10))

rc15 = RampConfig(axis_id=1, steps_per_rev=6400, target_rpm=15.0, accel_s=accel15, cruise_s=cruise15, decel_s=decel15)
gen15 = MultiAxisSegmentGenerator([AxisMotionConfig(1, rc15)])
steps15 = sum(s.steps[0] for _, s in zip(range(100), gen15))

print(f"100 segments (400ms) steps generated: 10rpm={steps10} 15rpm={steps15}")
