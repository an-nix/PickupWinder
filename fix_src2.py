import re

with open("src/WinderApp.cpp", "r") as f:
    lines = f.readlines()

new_lines = []
skip_mode = False
in_pause = False

for i, line in enumerate(lines):
    # skip the block:
    if "if (_pauseParkPending && _lateral.getState() == LatState::HOMED" in line:
        skip_mode = True
        continue
    if skip_mode:
        if "}" in line and i > 0 and "}" in lines[i-1]:
            pass
        if "        }" in line:
            # check if we exited the block, it has a simple structure
            pass
        # simpler: just skip next 7 lines
        continue
        
