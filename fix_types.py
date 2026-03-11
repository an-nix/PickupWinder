import pathlib, re
f = pathlib.Path(r"c:\temp\PickupWinder\include\Types.h")
t = f.read_text(encoding="utf-8")
# Remove stray semicolon after first windingStateName closing brace + entire duplicate old function
pattern = re.compile(
    r'(default:\s*return "UNKNOWN";\s*\n\s*\})\s*\};\s*\n\ninline const char\* windingStateName.*?default:\s*return "UNKNOWN";\s*\n\s*\}',
    re.DOTALL
)
t2 = pattern.sub(r'\1\n}', t)
if t2 != t:
    f.write_text(t2, encoding="utf-8")
    print("Fixed Types.h - removed duplicate windingStateName")
else:
    print("No match - checking file...")
    # Find all occurrences
    for m in re.finditer(r'inline const char\* windingStateName', t):
        print(f"  Found at pos {m.start()}")
