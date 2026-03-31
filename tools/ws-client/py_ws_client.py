#!/usr/bin/env python3
"""Python WebSocket client for PickupWinder telemetry and auto-calibration."""
import argparse
import json
import sys
import time

try:
    from websocket import create_connection
except ImportError:
    print('Install websocket-client: pip install websocket-client')
    sys.exit(1)


def parse_args():
    p = argparse.ArgumentParser(description='PickupWinder WebSocket client')
    p.add_argument('--host', default='192.168.4.1', help='Device IP or hostname')
    p.add_argument('--port', type=int, default=80, help='Device HTTP port')
    p.add_argument('--calib', action='store_true', help='Start auto calibration on connect')
    p.add_argument('--rpm', type=int, default=80, help='Auto-calibration spindle RPM')
    p.add_argument('--repeats', type=int, default=3, help='Auto-calibration repeats')
    p.add_argument('--apply', action='store_true', help='Auto-apply suggested offset')
    p.add_argument('--log', help='CSV file path for telemetry logging')
    return p.parse_args()


def main():
    cfg = parse_args()
    url = f'ws://{cfg.host}:{cfg.port}/ws'
    print('Connecting to', url)
    ws = create_connection(url, timeout=5)
    print('Connected.')

    if cfg.calib:
        val = f'{cfg.rpm},{cfg.repeats},{1 if cfg.apply else 0}'
        cmd = {'cmd': 'auto_calib', 'val': val}
        ws.send(json.dumps(cmd))
        print('Sent auto_calib:', val)

    logf = None
    if cfg.log:
        logf = open(cfg.log, 'w', encoding='utf-8', newline='')
        header = 'ts,turns,target,rpm,hz,latPos,latProgress,activeTpp,latScale,calibInProgress,calibCurrent,calibTotal,calibMeasuredTPP,calibSuggestedOffset' + '\n'
        logf.write(header)

    try:
        while True:
            msg = ws.recv()
            if msg is None:
                break
            try:
                data = json.loads(msg)
            except json.JSONDecodeError:
                print('Invalid JSON:', msg)
                continue

            ts = int(time.time() * 1000)
            turns = data.get('turns')
            target = data.get('target')
            rpm = data.get('rpm')
            hz = data.get('hz')
            latPos = data.get('latPos')
            latProgress = data.get('latProgress')
            activeTpp = data.get('activeTpp')
            latScale = data.get('latScale')
            calibInProgress = data.get('calibInProgress')
            calibCurrent = data.get('calibCurrent')
            calibTotal = data.get('calibTotal')
            calibMeasuredTPP = data.get('calibMeasuredTPP')
            calibSuggestedOffset = data.get('calibSuggestedOffset')

            line = f'{ts},{turns},{target},{rpm},{hz},{latPos},{latProgress},{activeTpp},{latScale},{calibInProgress},{calibCurrent},{calibTotal},{calibMeasuredTPP},{calibSuggestedOffset}'
            print(line)
            if logf:
                logf.write(line + '\n')
                logf.flush()

    except KeyboardInterrupt:
        print('Interrupted. Closing connection.')
    finally:
        if logf:
            logf.close()
        ws.close()


if __name__ == '__main__':
    main()
