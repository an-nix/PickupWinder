#!/usr/bin/env python3
"""
Simple eQEP reader for BeagleBone.

Supports:
- kernel counter framework (Debian 12 / kernel 6.12):
    /sys/bus/counter/devices/counterX/countY/count
- legacy sysfs paths with eqep .../position

Note: the counter framework requires writing 1 to countY/enable before
reading. This script handles that automatically.

Usage: python3 read_eqep.py [--interval 0.1] [--once]
"""

import time
import argparse
import glob
import os


def find_counter_entries():
    """Return list of (count_path, enable_path) for counter framework devices."""
    entries = []
    for count_path in sorted(glob.glob('/sys/bus/counter/devices/counter*/count*/count')):
        if os.path.isfile(count_path):
            enable_path = os.path.join(os.path.dirname(count_path), 'enable')
            entries.append((count_path, enable_path if os.path.isfile(enable_path) else None))
    return entries


def find_legacy_positions():
    """Return list of (position_path, None) for legacy eQEP sysfs."""
    matches = []
    for path in glob.glob('/sys/devices/**/eqep*', recursive=True):
        pos = os.path.join(path, 'position')
        if os.path.isfile(pos):
            matches.append((pos, None))
    for path in glob.glob('/sys/devices/**/position', recursive=True):
        if 'eqep' in path and (path, None) not in matches:
            matches.append((path, None))
    return sorted(set(matches))


def enable_counter(enable_path):
    """Write 1 to the counter enable sysfs file."""
    try:
        with open(enable_path, 'w') as f:
            f.write('1\n')
        return True
    except PermissionError:
        print(f'  [!] Permission refusée pour activer {enable_path}')
        print('      Relancez avec sudo ou ajoutez votre utilisateur au groupe gpio.')
        return False
    except Exception as e:
        print(f'  [!] Impossible d\'activer {enable_path}: {e}')
        return False


def read_value(path):
    try:
        with open(path, 'r') as f:
            return f.read().strip()
    except Exception:
        return None


def main():
    parser = argparse.ArgumentParser(description='Lire eQEP via sysfs (BeagleBone)')
    parser.add_argument('--interval', '-i', type=float, default=0.1,
                        help='Intervalle en secondes entre lectures (défaut: 0.1)')
    parser.add_argument('--once', action='store_true',
                        help='Lire une fois puis quitter')
    args = parser.parse_args()

    # Early diagnostic print so we know the script started and PID
    print(f"[start] read_eqep.py starting at {time.strftime('%Y-%m-%d %H:%M:%S')} pid={os.getpid()}", flush=True)

    # Discover devices — counter framework preferred on kernel 6.x
    counter_entries = find_counter_entries()
    print(f"[diag] counter_entries found={len(counter_entries)}", flush=True)
    # Avoid expensive recursive legacy search when the counter framework is present
    if counter_entries:
        legacy_entries = []
        print("[diag] skipping legacy search because counter devices present", flush=True)
    else:
        legacy_entries = find_legacy_positions()
        print(f"[diag] legacy_entries found={len(legacy_entries)}", flush=True)
    all_entries = counter_entries + legacy_entries  # (count_path, enable_path_or_None)

    if not all_entries:
        print('Aucun périphérique eQEP trouvé dans sysfs.')
        print('Chemins recherchés:')
        print('  counter: /sys/bus/counter/devices/counterX/countY/count')
        print('  legacy:  /sys/devices/.../eqep.../position')
        print()
        print('Vérifications:')
        print('  dmesg | grep -i eqep')
        print('  ls /sys/bus/counter/devices/')
        return 2

    print('Périphériques eQEP trouvés:')
    for count_path, enable_path in all_entries:
        print(f'  {count_path}')

    # Enable all counter framework devices (kernel 6.x requires explicit enable)
    print()
    enabled_any = False
    for count_path, enable_path in counter_entries:
        current = read_value(os.path.join(os.path.dirname(count_path), 'enable'))
        if current == '0':
            print(f'Activation du compteur: {enable_path}')
            if enable_counter(enable_path):
                enabled_any = True
        else:
            enabled_any = True  # already enabled

    if counter_entries and not enabled_any:
        return 1

    print()

    if args.once:
        for count_path, _ in all_entries:
            v = read_value(count_path)
            print(f'{count_path}: {v}')
        return 0

    last = {p: None for p, _ in all_entries}
    # Heartbeat: print an "alive" line every N seconds so user knows script is running
    heartbeat_interval = 5.0
    last_alive = time.time()
    try:
        while True:
            now = time.time()
            if now - last_alive >= heartbeat_interval:
                print(f"[alive] {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(now))} pid={os.getpid()} devices={len(all_entries)}", flush=True)
                last_alive = now

            for count_path, _ in all_entries:
                v = read_value(count_path)
                if v is None:
                    print(f'{count_path}: <read error>', flush=True)
                    continue
                prev = last[count_path]
                if prev is None:
                    delta = ''
                else:
                    try:
                        delta = f'Δ={int(v) - int(prev):+d}'
                    except Exception:
                        delta = ''
                print(f'{count_path}: {v}  {delta}', flush=True)
                last[count_path] = v
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print('\nInterrompu.')
        return 0


if __name__ == '__main__':
    raise SystemExit(main())
