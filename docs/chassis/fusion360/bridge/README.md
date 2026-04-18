# ROScar Fusion 360 Bridge

A filesystem-dropbox add-in that lets Claude (or any external process)
drive Fusion 360 without having to click through the **Scripts and
Add-Ins** dialog every iteration. The add-in loads automatically when
Fusion starts, polls `~/roscar_fusion_bridge/inbox` for JSON command
files, executes them on Fusion's main thread, and writes results back
to `~/roscar_fusion_bridge/outbox`.

Setup is a one-time manual step. After that, Claude can run scripts,
take screenshots, check status, and close documents from Bash — no more
"please run the script and send me a screenshot" back-and-forth.

## One-time install

1. **Open Fusion 360.**
2. Press **`Shift+S`** (or go to **Utilities → Scripts and Add-Ins**).
3. Click the **Add-Ins** tab.
4. Click the small **`+`** icon next to "My Add-Ins" → **Script or
   Add-In from Device**.
5. In the file picker, navigate to
   `D:\localrepos\ROScar1\docs\chassis\fusion360\bridge\` and
   click **Select Folder**.
6. The add-in "ROScar Bridge" now appears in the list. Select it.
7. Tick **Run on Startup**, then click **Run**.

A file called `~/roscar_fusion_bridge/heartbeat` (on Windows:
`C:\Users\<you>\roscar_fusion_bridge\heartbeat`) will start getting
touched every ~2 seconds. That's the add-in's liveness signal.

## Sanity check

From any shell on the same machine:

```bash
python D:/localrepos/ROScar1/docs/chassis/fusion360/bridge/fctl.py ping
```

Expected output:

```json
{
  "msg": "pong",
  "version": "2.0.xxxxx"
}
```

If you instead see `Bridge not alive: heartbeat file missing ...`,
the add-in isn't loaded. Re-check the install steps above and make sure
Fusion is actually running.

## Commands

| Command | What it does |
|---------|--------------|
| `ping` | Returns pong + Fusion version |
| `status` | Fusion version, open document count, active document name |
| `run <path>` | Imports the script at `<path>` and calls its `run(context)`. Captures stdout/stderr. If the script exposes `VERSION`, `_clog`, or `_frame_positions` at module scope, they come back in the response. |
| `close [--save]` | Closes every open document. `--save` keeps changes (default: discard). |
| `screenshot <out.png> [--width W] [--height H]` | Saves the current viewport to `out.png`. Defaults 1600×1000. |

## Typical iteration (what Claude does)

```bash
BRIDGE=D:/localrepos/ROScar1/docs/chassis/fusion360/bridge
CHASSIS=D:/localrepos/ROScar1/docs/chassis/fusion360/roscar_v2_chassis.py

# Close any leftover documents from the last run
python $BRIDGE/fctl.py close

# Re-run the chassis script
python $BRIDGE/fctl.py run $CHASSIS

# Grab a viewport image
python $BRIDGE/fctl.py screenshot D:/tmp/chassis_latest.png --width 1920 --height 1200
```

The `run` response includes `stdout`, `_clog`, and `_frame_positions`
when they exist, so Claude can see exactly what the script produced
without needing a screenshot or message-box transcript.

## Troubleshooting

**`heartbeat file missing`** — Fusion isn't running, or the add-in
didn't load. Open Fusion, open the Add-Ins dialog, and make sure
"ROScar Bridge" shows a green running indicator.

**`heartbeat is stale`** — The add-in loaded but something has blocked
Fusion's main thread (a modal dialog, a long script, a hang). Dismiss
any open dialogs in Fusion and retry.

**`No response after Ns`** — The command was accepted but didn't finish
in time. Increase `--timeout`, or check `~/roscar_fusion_bridge/log/bridge.log`
for exceptions. A STEP-heavy script can take 30–60 seconds.

**Errors from my script show up as `FUSION ERROR: ...`** — the add-in
caught an exception raised by your script's `run(context)`. The full
traceback is printed to stderr. Check `bridge.log` for the same trace
with a timestamp.

## What lives where

```
<repo>/docs/chassis/fusion360/bridge/
├── roscar_bridge.py         # the add-in Fusion loads
├── roscar_bridge.manifest   # Fusion manifest (runOnStartup=true)
├── fctl.py                  # plain Python CLI; no Fusion imports
└── README.md                # this file

~/roscar_fusion_bridge/      # created at first run; gitignored
├── inbox/                   # client writes <uuid>.json here
├── outbox/                  # add-in writes <uuid>.json here
├── heartbeat                # touched every ~2s by add-in
└── log/bridge.log           # add-in's own diagnostic log
```

## Safety notes

- Only one Fusion instance should have the add-in enabled — the poller
  thread has no locking, so two add-ins on the same user account would
  race for inbox files.
- The add-in does not validate `action` values against a whitelist by
  design — it's a personal-machine tool, not a public service. Don't
  expose `~/roscar_fusion_bridge/inbox/` to untrusted writers.
- `run_script` does `importlib.util.spec_from_file_location` + exec on
  whatever path you point at. Same trust assumption: personal use only.
