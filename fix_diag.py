"""
Migrate all Serial.println/printf calls to Diag::info/warn/error[f].
Also fixes literal newlines inside string literals (compile bug from previous scripts).
"""
import re, pathlib, sys

ROOT = pathlib.Path(r"c:\temp\PickupWinder")

# Files to migrate (Diag.cpp itself keeps raw Serial calls)
TARGETS = [
    "src/LateralController.cpp",
    "src/WinderApp.cpp",
    "src/StepperController.cpp",
    "src/WebInterface.cpp",
    "src/LinkSerial.cpp",
    "src/main.cpp",
]

ERROR_KEYS = ("erreur", "error", "fault", "fail", "impossible", "invalid", "unable", "fehler")
WARN_KEYS  = ("avertissement", "avert", "warn", "warning")

def detect_level(s: str) -> str:
    s = s.lower()
    if any(k in s for k in ERROR_KEYS): return "error"
    if any(k in s for k in WARN_KEYS):  return "warn"
    return "info"

# ── String-literal-aware character scanner ────────────────────────────────────

def fix_literal_newlines(text: str) -> str:
    """Replace raw newline characters inside C string literals with \\n."""
    out, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        if c != '"':
            out.append(c); i += 1; continue
        # Enter string literal
        out.append('"'); i += 1
        while i < n:
            c = text[i]
            if c == '\\':
                out.append(c); i += 1
                if i < n: out.append(text[i]); i += 1
            elif c == '"':
                out.append('"'); i += 1; break
            elif c == '\n':
                out.append('\\n'); i += 1          # fix the bug
            elif c == '\r':
                i += 1                              # skip CR
            else:
                out.append(c); i += 1
    return ''.join(out)

def find_close_paren(text: str, pos: int) -> int:
    """Find the matching ')' for an '(' at pos, skipping strings."""
    depth, i, n = 0, pos, len(text)
    while i < n:
        c = text[i]
        if c == '"':
            i += 1
            while i < n:
                c2 = text[i]
                if c2 == '\\': i += 2; continue
                if c2 == '"':  i += 1; break
                i += 1
        elif c == '(': depth += 1; i += 1
        elif c == ')':
            depth -= 1; i += 1
            if depth == 0: return i - 1
        else: i += 1
    return -1

def first_string_arg(args: str) -> str:
    """Return the first comma-delimited arg (the format string)."""
    depth, i = 0, 0
    while i < len(args):
        c = args[i]
        if c == '"':
            i += 1
            while i < len(args):
                c2 = args[i]
                if c2 == '\\': i += 2; continue
                if c2 == '"':  i += 1; break
                i += 1
        elif c in '([': depth += 1; i += 1
        elif c in ')]': depth -= 1; i += 1
        elif c == ',' and depth == 0:
            return args[:i].strip()
        else: i += 1
    return args.strip()

def strip_trailing_newline(fmt: str) -> str:
    """Remove a trailing \\n from a C string literal: '"...\\n"' -> '"..."'."""
    if fmt.startswith('"') and fmt.endswith('"'):
        inner = fmt[1:-1]
        inner = re.sub(r'\\n$', '', inner)
        return f'"{inner}"'
    return fmt

# ── Main replacement logic ────────────────────────────────────────────────────

def migrate(text: str) -> str:
    text = fix_literal_newlines(text)
    out, i, n = [], 0, len(text)

    while i < n:
        # Detect Serial.println( or Serial.printf(
        for kw, is_printf in (("Serial.printf(", True), ("Serial.println(", False)):
            if text[i:i+len(kw)] == kw:
                paren_open  = i + len(kw) - 1   # index of '('
                paren_close = find_close_paren(text, paren_open)
                if paren_close == -1:
                    break  # malformed — skip
                args = text[paren_open+1:paren_close]

                if is_printf:
                    fmt = first_string_arg(args)
                    rest = args[len(fmt)+1:].lstrip(',').lstrip() if ',' in args[len(fmt):] else ''
                    level = detect_level(fmt)
                    fn = f"Diag::{level}f"
                    fmt_clean = strip_trailing_newline(fmt.strip())
                    if rest.strip():
                        out.append(f"{fn}({fmt_clean},\n{' '*len(fn)} {rest})")
                    else:
                        out.append(f"{fn}({fmt_clean})")
                else:
                    level = detect_level(args)
                    out.append(f"Diag::{level}({args})")

                # Consume to end of statement (past the ';')
                j = paren_close + 1
                while j < n and text[j] in ' \t': j += 1
                if j < n and text[j] == ';': j += 1
                out.append(';')
                i = j
                break
        else:
            out.append(text[i]); i += 1

    return ''.join(out)

def ensure_include(text: str, header: str) -> str:
    """Add #include "header" after the first #include block if not present."""
    inc = f'#include "{header}"'
    if inc in text:
        return text
    # Insert after the last existing #include line
    lines = text.splitlines(keepends=True)
    last_inc = -1
    for idx, line in enumerate(lines):
        if line.strip().startswith('#include'):
            last_inc = idx
    if last_inc >= 0:
        lines.insert(last_inc + 1, inc + '\n')
    else:
        lines.insert(0, inc + '\n')
    return ''.join(lines)

# ── Process all target files ──────────────────────────────────────────────────

for rel in TARGETS:
    fp = ROOT / rel
    if not fp.exists():
        print(f"SKIP  (not found): {rel}")
        continue
    original = fp.read_text(encoding="utf-8")
    migrated = migrate(original)
    migrated = ensure_include(migrated, "Diag.h")
    if migrated != original:
        fp.write_text(migrated, encoding="utf-8")
        n_serial = original.count("Serial.println") + original.count("Serial.printf")
        print(f"OK    {rel:45s}  ({n_serial} calls migrated)")
    else:
        print(f"NOOP  {rel}  (no Serial calls found)")
