#!/usr/bin/env python3
"""
Simple eQEP reader for BeagleBone.

Supports:
- legacy sysfs paths with eqep .../position
- kernel counter framework (Debian 12 / kernel 6.12):
    /sys/bus/counter/devices/counterX/countY/count

Usage: python3 read_eqep.py [--interval 0.1]
"""

import time
import argparse
import glob
import os


def find_eqep_positions_legacy():
    # Search common sysfs locations for files named "position" under a parent path containing 'eqep'.
    matches = []
    for path in glob.glob('/sys/devices/**/eqep*', recursive=True):
        pos = os.path.join(path, 'position')
        if os.path.isfile(pos):
            matches.append(pos)
    # also try a looser search
    for path in glob.glob('/sys/devices/**/position', recursive=True):
        if 'eqep' in path and path not in matches:
            matches.append(path)
    return sorted(set(matches))


def find_eqep_positions_counter():
    # Counter framework paths used by recent kernels (ti-eqep-cnt driver).
    matches = []
    for path in glob.glob('/sys/bus/counter/devices/counter*/count*/count'):
        if os.path.isfile(path):
            matches.append(path)
    return sorted(set(matches))


def find_all_positions():
    # Prefer counter framework first on modern kernels, then legacy fallback.
    counter = find_eqep_positions_counter()
    legacy = find_eqep_positions_legacy()
    return counter + [p for p in legacy if p not in counter]


def read_value(path):
    try:
        with open(path, 'r') as f:
            return f.read().strip()
    except Exception:
        return None


def main():
    parser = argparse.ArgumentParser(description='Lire eQEP via sysfs (BeagleBone)')
    parser.add_argument('--interval', '-i', type=float, default=0.1, help='Intervalle en secondes entre lectures')
    parser.add_argument('--once', action='store_true', help='Lire une fois puis quitter')
    args = parser.parse_args()

    positions = find_all_positions()
    if not positions:
        print('Aucun périphérique eQEP trouvé dans sysfs.')
        print('Chemins recherchés:')
        print(' - legacy: /sys/devices/.../eqep.../position')
        print(' - counter: /sys/bus/counter/devices/counterX/countY/count')
        return 2

    print('Périphériques eQEP trouvés:')
    for p in positions:
        print(' -', p)

    if args.once:
        for p in positions:
            print(p, read_value(p))
        return 0

    last = {p: None for p in positions}
    try:
        while True:
            for p in positions:
                v = read_value(p)
                if v is None:
                    print(p, '<read error>')
                    continue
                if last[p] is None:
                    delta = ''
                else:
                    try:
                        delta = str(int(v) - int(last[p]))
                    except Exception:
                        delta = ''
                print(f'{p}: {v}  Δ={delta}')
                last[p] = v
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print('\nInterrompu par l\'utilisateur')
        return 0


if __name__ == '__main__':
    raise SystemExit(main())
