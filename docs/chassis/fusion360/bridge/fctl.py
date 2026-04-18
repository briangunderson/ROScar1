#!/usr/bin/env python3
"""fctl — CLI client for the ROScar Fusion 360 Bridge add-in.

Usage (once the bridge add-in is loaded in a running Fusion):

    python fctl.py ping
    python fctl.py status
    python fctl.py run D:/localrepos/ROScar1/docs/chassis/fusion360/roscar_v2_chassis.py
    python fctl.py close
    python fctl.py screenshot D:/tmp/chassis.png --width 1920 --height 1080

Each command:
  1. Writes <uuid>.json into ~/roscar_fusion_bridge/inbox
  2. Waits for ~/roscar_fusion_bridge/outbox/<uuid>.json to appear
  3. Prints the "data" payload (or the error) and exits.

No Fusion imports here — this is a plain Python 3 script that talks to
the add-in over the filesystem. Safe to run from any environment.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
import uuid

BRIDGE_DIR = os.path.join(os.path.expanduser('~'), 'roscar_fusion_bridge')
INBOX_DIR = os.path.join(BRIDGE_DIR, 'inbox')
OUTBOX_DIR = os.path.join(BRIDGE_DIR, 'outbox')
HEARTBEAT_FILE = os.path.join(BRIDGE_DIR, 'heartbeat')

HEARTBEAT_MAX_AGE_SEC = 10


def _check_alive() -> tuple[bool, str]:
    """Return (alive, reason). Alive means the heartbeat file was touched
    recently enough that we can expect the add-in to respond.
    """
    if not os.path.exists(HEARTBEAT_FILE):
        return (False,
                f'heartbeat file missing at {HEARTBEAT_FILE}. '
                f'Is Fusion running with the ROScar Bridge add-in enabled?')
    try:
        age = time.time() - os.path.getmtime(HEARTBEAT_FILE)
    except Exception as e:
        return (False, f'could not stat heartbeat: {e}')
    if age > HEARTBEAT_MAX_AGE_SEC:
        return (False,
                f'heartbeat is stale ({age:.0f}s old, max '
                f'{HEARTBEAT_MAX_AGE_SEC}s). The add-in may have crashed '
                f'or Fusion may be frozen.')
    return (True, f'heartbeat age {age:.1f}s')


def send(cmd: dict, timeout: float = 60) -> dict:
    """Send a command, block until the add-in responds (or we time out).

    Raises RuntimeError if the bridge is not alive or the response times
    out. Returns the parsed JSON response (already unwrapped from the
    {"ok": ..., "data": ...} envelope on success).
    """
    alive, reason = _check_alive()
    if not alive:
        raise RuntimeError(f'Bridge not alive: {reason}')

    os.makedirs(INBOX_DIR, exist_ok=True)
    os.makedirs(OUTBOX_DIR, exist_ok=True)

    cmd_id = uuid.uuid4().hex[:12]
    inbox_final = os.path.join(INBOX_DIR, f'{cmd_id}.json')
    inbox_tmp = inbox_final + '.tmp'
    outbox_final = os.path.join(OUTBOX_DIR, f'{cmd_id}.json')

    with open(inbox_tmp, 'w', encoding='utf-8') as f:
        json.dump(cmd, f)
    os.replace(inbox_tmp, inbox_final)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if os.path.exists(outbox_final):
            with open(outbox_final, 'r', encoding='utf-8') as f:
                resp = json.load(f)
            try:
                os.remove(outbox_final)
            except Exception:
                pass
            return resp
        time.sleep(0.1)

    # Clean up our un-processed inbox file if the add-in never got to it.
    try:
        if os.path.exists(inbox_final):
            os.remove(inbox_final)
    except Exception:
        pass
    raise TimeoutError(
        f'No response after {timeout}s. '
        f'Command id was {cmd_id}; last heartbeat age '
        f'{time.time() - os.path.getmtime(HEARTBEAT_FILE):.0f}s.')


def _handle(resp: dict) -> int:
    if resp.get('ok'):
        data = resp.get('data', {})
        # For readable terminal output, pretty-print dicts/lists; for
        # scalars, print bare.
        if isinstance(data, (dict, list)):
            print(json.dumps(data, indent=2, default=str))
        else:
            print(data)
        return 0
    sys.stderr.write('FUSION ERROR: ')
    sys.stderr.write(resp.get('error', '(no message)') + '\n')
    if 'traceback' in resp:
        sys.stderr.write(resp['traceback'] + '\n')
    return 2


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        prog='fctl',
        description='CLI for the ROScar Fusion 360 Bridge add-in.')
    sub = p.add_subparsers(dest='action', required=True)

    sub.add_parser('ping', help='Check that the bridge is alive.')
    sub.add_parser('status', help='Report Fusion version + open docs.')

    p_run = sub.add_parser('run', help='Run a Fusion script by path.')
    p_run.add_argument('path')
    p_run.add_argument('--timeout', type=float, default=300,
                       help='Max seconds to wait (default 300).')

    p_close = sub.add_parser('close', help='Close all open documents.')
    p_close.add_argument('--save', action='store_true',
                         help='Save documents before closing (default no).')

    p_shot = sub.add_parser('screenshot', help='Save viewport image.')
    p_shot.add_argument('path')
    p_shot.add_argument('--width', type=int, default=1600)
    p_shot.add_argument('--height', type=int, default=1000)

    args = p.parse_args(argv)

    if args.action == 'ping':
        cmd = {'action': 'ping'}
        timeout = 10.0
    elif args.action == 'status':
        cmd = {'action': 'status'}
        timeout = 10.0
    elif args.action == 'run':
        cmd = {'action': 'run_script', 'path': os.path.abspath(args.path)}
        timeout = float(args.timeout)
    elif args.action == 'close':
        cmd = {'action': 'close_all', 'save': bool(args.save)}
        timeout = 30.0
    elif args.action == 'screenshot':
        cmd = {
            'action': 'screenshot',
            'path': os.path.abspath(args.path),
            'width': args.width,
            'height': args.height,
        }
        timeout = 30.0
    else:
        p.error(f'Unknown action: {args.action}')
        return 2

    try:
        resp = send(cmd, timeout=timeout)
    except (RuntimeError, TimeoutError) as e:
        sys.stderr.write(f'{e}\n')
        return 1
    return _handle(resp)


if __name__ == '__main__':
    sys.exit(main())
