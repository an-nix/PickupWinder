import sys
import os
root = os.path.abspath(os.path.join(os.path.dirname(__file__), "src", "rpi"))
if root not in sys.path:
    sys.path.insert(0, root)

from motion.move import HomingMove

move = HomingMove(
    name="test",
    axis_id=1,
    steps_per_rev=6400,
    approach_rpm=100.0,
    search_rpm=20.0,
    backoff_steps=200,
    max_approach_steps=6400*20,
)

approach = move._make_approach_move()
print("Approach Config:", approach._config.axis_configs[0].ramp)
b_move = move._make_backoff_move()
print("Backoff Config:", b_move._config.axis_configs[0].ramp)
s_move = move._make_search_move()
print("Search Config:", s_move._config.axis_configs[0].ramp)

gen = approach.segments()
blocks = []
for _ in range(10):
    blocks.append(next(gen))

print("Approach first 10 segment steps:")
for b in blocks:
    print(b)
