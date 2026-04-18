"""
ROScar Fusion 360 Bridge — filesystem-dropbox add-in.

Purpose
-------
Lets Claude (or any external process) drive Fusion 360 without the user
having to click through the Scripts / Add-Ins dialog each time. The
add-in loads automatically at Fusion startup, polls a known directory
for command JSON files, executes them on the Fusion main thread, and
writes the result back as a JSON file in a sibling directory.

Protocol
--------
Directories (all under ``<home>/roscar_fusion_bridge``):

    inbox/           # client writes <uuid>.json here; add-in consumes
    outbox/          # add-in writes <uuid>.json here; client consumes
    heartbeat        # add-in touches every few seconds; liveness probe
    log/bridge.log   # add-in's own diagnostic log (append-only)

Request JSON (what the client writes to ``inbox/<uuid>.json``):

    {"action": "ping"}
    {"action": "status"}
    {"action": "run_script", "path": "/absolute/path/to/script.py"}
    {"action": "close_all", "save": false}
    {"action": "screenshot", "path": "/abs/out.png", "width": 1600, "height": 1000}

Response JSON (add-in writes ``outbox/<uuid>.json``):

    {"ok": true,  "data": {...}}
    {"ok": false, "error": "...", "traceback": "..."}

Thread model
------------
The socket/filesystem polling happens on a background thread, but
Fusion's API calls (document manipulation, viewport I/O, etc.) must
happen on the main thread. We use ``Application.registerCustomEvent``
/ ``fireCustomEvent`` to marshal command execution back onto the main
thread. The poller thread only handles I/O.
"""

import adsk.core
import adsk.fusion
import importlib.util
import io
import json
import os
import sys
import threading
import time
import traceback

ADDIN_NAME = 'ROScar Bridge'
BRIDGE_DIR = os.path.join(os.path.expanduser('~'), 'roscar_fusion_bridge')
INBOX_DIR = os.path.join(BRIDGE_DIR, 'inbox')
OUTBOX_DIR = os.path.join(BRIDGE_DIR, 'outbox')
HEARTBEAT_FILE = os.path.join(BRIDGE_DIR, 'heartbeat')
LOG_DIR = os.path.join(BRIDGE_DIR, 'log')
LOG_FILE = os.path.join(LOG_DIR, 'bridge.log')

EVENT_ID = 'com.roscar.bridge.command'
POLL_INTERVAL = 0.5        # seconds between inbox scans
HEARTBEAT_INTERVAL = 2.0   # seconds between heartbeat touches

# Module-level globals set in run(), cleared in stop().
_app = None
_ui = None
_custom_event = None
_event_handler = None
_stop_event = threading.Event()
_poll_thread = None


# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
def _log(msg):
    """Append a timestamped line to the bridge log."""
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        line = f'{time.strftime("%Y-%m-%d %H:%M:%S")} {msg}\n'
        with open(LOG_FILE, 'a', encoding='utf-8') as f:
            f.write(line)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Command dispatch (runs on MAIN thread via CustomEvent)
# ---------------------------------------------------------------------------
class _CommandEventHandler(adsk.core.CustomEventHandler):
    def notify(self, args):
        cmd_id = None
        try:
            payload = json.loads(args.additionalInfo)
            cmd_id = payload['id']
            cmd = payload['cmd']
            _log(f'dispatch id={cmd_id} cmd={cmd}')
            result = _dispatch(cmd)
            _write_result(cmd_id, {'ok': True, 'data': result})
        except Exception as e:
            tb = traceback.format_exc()
            _log(f'dispatch FAILED id={cmd_id}: {e}\n{tb}')
            if cmd_id is not None:
                _write_result(cmd_id, {
                    'ok': False, 'error': str(e), 'traceback': tb})


def _dispatch(cmd):
    action = cmd.get('action')
    if action == 'ping':
        return {'msg': 'pong', 'version': _app.version}
    if action == 'status':
        return _cmd_status()
    if action == 'run_script':
        return _cmd_run_script(cmd['path'])
    if action == 'close_all':
        return _cmd_close_all(save=cmd.get('save', False))
    if action == 'screenshot':
        return _cmd_screenshot(
            cmd['path'], cmd.get('width', 1600), cmd.get('height', 1000))
    raise ValueError(f'Unknown action: {action!r}')


def _cmd_status():
    docs = []
    for i in range(_app.documents.count):
        d = _app.documents.item(i)
        docs.append(d.name)
    return {
        'version': _app.version,
        'documentCount': _app.documents.count,
        'documents': docs,
        'activeDocument': (_app.activeDocument.name
                           if _app.activeDocument else None),
    }


def _cmd_run_script(path):
    path = os.path.abspath(path)
    if not os.path.exists(path):
        raise FileNotFoundError(path)

    # Force a fresh import every time — otherwise module-level globals
    # leak between runs.
    mod_name = 'roscar_bridge_target_script'
    if mod_name in sys.modules:
        del sys.modules[mod_name]
    spec = importlib.util.spec_from_file_location(mod_name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[mod_name] = mod

    stdout_cap = io.StringIO()
    stderr_cap = io.StringIO()
    old_out, old_err = sys.stdout, sys.stderr
    sys.stdout, sys.stderr = stdout_cap, stderr_cap

    try:
        spec.loader.exec_module(mod)
        # Cooperative no-modal-dialog protocol: if the target script
        # defines a SHOW_MSGBOX module-level flag, set it to False before
        # invoking run() so the script prints its summary instead of
        # blocking on a modal dialog. Scripts that don't expose the flag
        # still work — they just behave as usual (and the bridge will
        # hang until a human dismisses the dialog).
        if hasattr(mod, 'SHOW_MSGBOX'):
            mod.SHOW_MSGBOX = False
        if hasattr(mod, 'run'):
            mod.run({})
    finally:
        sys.stdout, sys.stderr = old_out, old_err

    result = {
        'ran': path,
        'stdout': stdout_cap.getvalue(),
        'stderr': stderr_cap.getvalue(),
    }

    # Pull out well-known debug globals if the script exposes them.
    # These are strictly optional — keeps the bridge generic.
    if hasattr(mod, 'VERSION'):
        result['VERSION'] = mod.VERSION
    if hasattr(mod, '_clog'):
        try:
            result['_clog'] = list(mod._clog)
        except Exception:
            pass
    if hasattr(mod, '_frame_positions'):
        # _frame_positions entries may contain Fusion Occurrence objects
        # (not JSON serialisable). Extract name + intended + actual only.
        try:
            rows = []
            for entry in mod._frame_positions:
                if len(entry) >= 3:
                    name, intended, occ = entry[:3]
                    try:
                        tr = occ.transform.translation
                        got = (tr.x, tr.y, tr.z)
                    except Exception:
                        got = None
                    rows.append({
                        'name': name,
                        'intended': list(intended) if intended else None,
                        'got': list(got) if got else None,
                    })
                elif len(entry) == 2:
                    name, intended = entry
                    rows.append({
                        'name': name,
                        'intended': list(intended) if intended else None,
                    })
            result['_frame_positions'] = rows
        except Exception as e:
            result['_frame_positions_error'] = str(e)
    return result


def _cmd_close_all(save=False):
    closed = 0
    # Iterate by index 0 because closing shrinks the collection.
    guard = 0
    while _app.documents.count > 0 and guard < 50:
        _app.documents.item(0).close(save)
        closed += 1
        guard += 1
    return {'closed': closed}


def _cmd_screenshot(path, width, height):
    path = os.path.abspath(path)
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    vp = _app.activeViewport
    if vp is None:
        raise RuntimeError('No active viewport')
    vp.saveAsImageFile(path, int(width), int(height))
    return {'path': path, 'width': width, 'height': height}


# ---------------------------------------------------------------------------
# Filesystem I/O
# ---------------------------------------------------------------------------
def _write_result(cmd_id, result):
    """Atomic write to outbox. Client watches outbox for <cmd_id>.json."""
    try:
        os.makedirs(OUTBOX_DIR, exist_ok=True)
        final = os.path.join(OUTBOX_DIR, f'{cmd_id}.json')
        tmp = final + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(result, f, indent=2, default=str)
        os.replace(tmp, final)
    except Exception as e:
        _log(f'_write_result failed for {cmd_id}: {e}')


def _poll_loop():
    """Background thread. Polls the inbox and fires CustomEvents for each
    new command. All Fusion API work happens on the main thread inside
    the CustomEvent handler.
    """
    try:
        os.makedirs(INBOX_DIR, exist_ok=True)
        os.makedirs(OUTBOX_DIR, exist_ok=True)
    except Exception as e:
        _log(f'_poll_loop mkdir failed: {e}')
        return

    last_heartbeat = 0.0
    while not _stop_event.is_set():
        try:
            now = time.time()
            if now - last_heartbeat >= HEARTBEAT_INTERVAL:
                try:
                    with open(HEARTBEAT_FILE, 'w') as f:
                        f.write(f'{now:.0f}\n')
                    last_heartbeat = now
                except Exception as e:
                    _log(f'heartbeat failed: {e}')

            # Scan inbox. sorted() gives deterministic order; files are
            # named <uuid>.json so order doesn't really matter but it's
            # nice for debugging.
            try:
                names = sorted(os.listdir(INBOX_DIR))
            except FileNotFoundError:
                names = []
            for name in names:
                if not name.endswith('.json'):
                    continue
                if name.endswith('.tmp.json') or name.endswith('.json.tmp'):
                    continue
                full = os.path.join(INBOX_DIR, name)
                try:
                    with open(full, 'r', encoding='utf-8') as f:
                        cmd = json.load(f)
                    cmd_id = os.path.splitext(name)[0]
                    # Remove BEFORE firing to avoid duplicate processing
                    # if event dispatch is slow and we poll again.
                    os.remove(full)
                    payload = {'id': cmd_id, 'cmd': cmd}
                    _app.fireCustomEvent(EVENT_ID, json.dumps(payload))
                    _log(f'enqueued id={cmd_id}')
                except Exception as e:
                    _log(f'failed to process inbox file {name}: {e}')
                    try: os.remove(full)
                    except Exception: pass
        except Exception as e:
            _log(f'_poll_loop iteration error: {e}')
        time.sleep(POLL_INTERVAL)


# ---------------------------------------------------------------------------
# Add-in lifecycle
# ---------------------------------------------------------------------------
def run(context):
    """Called by Fusion when the add-in starts."""
    global _app, _ui, _custom_event, _event_handler, _poll_thread
    try:
        _app = adsk.core.Application.get()
        _ui = _app.userInterface

        os.makedirs(BRIDGE_DIR, exist_ok=True)
        os.makedirs(INBOX_DIR, exist_ok=True)
        os.makedirs(OUTBOX_DIR, exist_ok=True)
        os.makedirs(LOG_DIR, exist_ok=True)

        _custom_event = _app.registerCustomEvent(EVENT_ID)
        _event_handler = _CommandEventHandler()
        _custom_event.add(_event_handler)

        _stop_event.clear()
        _poll_thread = threading.Thread(
            target=_poll_loop, name='ROScarBridgePoll', daemon=True)
        _poll_thread.start()

        _log(f'{ADDIN_NAME} started. Bridge dir: {BRIDGE_DIR}')
    except Exception as e:
        tb = traceback.format_exc()
        _log(f'startup failed: {e}\n{tb}')
        if _ui:
            _ui.messageBox(f'{ADDIN_NAME} startup failed:\n{e}\n{tb}')


def stop(context):
    """Called by Fusion when the add-in unloads (including Fusion quit)."""
    global _custom_event, _event_handler
    try:
        _stop_event.set()
        if _poll_thread and _poll_thread.is_alive():
            _poll_thread.join(timeout=2.0)
        if _custom_event and _event_handler:
            try:
                _custom_event.remove(_event_handler)
            except Exception:
                pass
        if _app and _custom_event:
            try:
                _app.unregisterCustomEvent(EVENT_ID)
            except Exception:
                pass
        _custom_event = None
        _event_handler = None
        _log(f'{ADDIN_NAME} stopped.')
    except Exception as e:
        _log(f'stop failed: {e}')
        if _ui:
            _ui.messageBox(
                f'{ADDIN_NAME} stop error:\n{e}\n{traceback.format_exc()}')
