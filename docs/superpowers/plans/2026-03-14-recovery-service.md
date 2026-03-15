# Recovery Service Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A standalone recovery HTTP service on port 9999 that can restart the roscar-web stack when the main dashboard is unresponsive.

**Architecture:** Single Python script using only stdlib (`http.server`, `subprocess`). Runs as its own systemd service, completely independent of ROS2. Inline HTML (no external files to break).

**Tech Stack:** Python 3 stdlib only. systemd for lifecycle.

---

## File Structure

| File | Action | Responsibility |
|------|--------|---------------|
| `scripts/roscar-recovery.py` | Create | Standalone HTTP server: serves recovery page, handles start/stop/restart actions |
| `scripts/roscar-recovery.service` | Create | systemd unit file (independent of roscar-web) |
| `scripts/setup_rpi.sh` | Modify | Add recovery service installation to RPi setup |
| `CLAUDE.md` | Modify | Document the recovery service |

---

## Chunk 1: Recovery Service

### Task 1: Create the recovery server script

**Files:**
- Create: `scripts/roscar-recovery.py`

- [ ] **Step 1: Write roscar-recovery.py**

Single-file Python HTTP server with inline HTML. Key design:
- Port 9999, binds 0.0.0.0
- `GET /` — serves inline HTML page with status + buttons
- `GET /status` — JSON: `{"service": "active|failed|inactive", "uptime": "...", "timestamp": "..."}`
- `POST /restart` — runs `systemctl restart roscar-web`, returns JSON result
- `POST /start` — runs `systemctl start roscar-web`, returns JSON result
- `POST /stop` — runs `systemctl stop roscar-web`, returns JSON result
- HTML page auto-refreshes status every 5s via fetch
- Link to `http://<same-host>:8888/aio.html`
- Only POST for mutations (no accidental restarts)

```python
#!/usr/bin/env python3
"""
ROScar Recovery Service — lightweight admin page for restarting the web stack.
Runs on port 9999 with zero ROS2 dependencies.
"""

import http.server
import json
import subprocess
import socket
from urllib.parse import urlparse
from datetime import datetime

PORT = 9999
SERVICE = "roscar-web"

def get_service_status():
    """Query systemd for roscar-web service state."""
    info = {"service": "unknown", "sub_state": "unknown", "uptime": "N/A",
            "timestamp": datetime.now().isoformat(), "hostname": socket.gethostname()}
    try:
        result = subprocess.run(
            ["systemctl", "is-active", SERVICE],
            capture_output=True, text=True, timeout=5)
        info["service"] = result.stdout.strip()
    except Exception:
        pass
    try:
        result = subprocess.run(
            ["systemctl", "show", SERVICE, "--property=SubState,ActiveEnterTimestamp"],
            capture_output=True, text=True, timeout=5)
        for line in result.stdout.strip().splitlines():
            if line.startswith("SubState="):
                info["sub_state"] = line.split("=", 1)[1]
            elif line.startswith("ActiveEnterTimestamp="):
                ts = line.split("=", 1)[1].strip()
                info["uptime"] = ts if ts else "N/A"
    except Exception:
        pass
    return info

def service_action(action):
    """Run systemctl start/stop/restart."""
    try:
        result = subprocess.run(
            ["sudo", "systemctl", action, SERVICE],
            capture_output=True, text=True, timeout=30)
        return {"success": result.returncode == 0,
                "message": result.stderr.strip() or f"{action} completed"}
    except subprocess.TimeoutExpired:
        return {"success": False, "message": f"{action} timed out (30s)"}
    except Exception as e:
        return {"success": False, "message": str(e)}

HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ROScar Recovery</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: -apple-system, system-ui, sans-serif; background: #1a1a2e;
         color: #e0e0e0; display: flex; justify-content: center; align-items: center;
         min-height: 100vh; }
  .card { background: #16213e; border-radius: 12px; padding: 2rem;
          max-width: 420px; width: 90%; box-shadow: 0 4px 24px rgba(0,0,0,0.4); }
  h1 { font-size: 1.3rem; margin-bottom: 1.5rem; text-align: center; }
  .status-row { display: flex; justify-content: space-between; align-items: center;
                padding: 0.5rem 0; border-bottom: 1px solid #ffffff10; }
  .status-row:last-child { border-bottom: none; }
  .label { color: #888; font-size: 0.85rem; }
  .badge { padding: 0.2rem 0.7rem; border-radius: 6px; font-weight: 600;
           font-size: 0.85rem; }
  .badge.active { background: #0f5132; color: #75d9a3; }
  .badge.failed { background: #5c1a1a; color: #f77; }
  .badge.inactive, .badge.unknown { background: #444; color: #aaa; }
  .actions { display: flex; gap: 0.5rem; margin-top: 1.5rem; }
  button { flex: 1; padding: 0.7rem; border: none; border-radius: 8px;
           font-size: 0.95rem; font-weight: 600; cursor: pointer;
           transition: opacity 0.2s; }
  button:disabled { opacity: 0.5; cursor: wait; }
  .btn-restart { background: #e2b714; color: #1a1a2e; }
  .btn-start { background: #2d8a4e; color: #fff; }
  .btn-stop { background: #8a2d2d; color: #fff; }
  .dashboard-link { display: block; text-align: center; margin-top: 1.2rem;
                    color: #5dade2; text-decoration: none; font-size: 0.9rem; }
  .dashboard-link:hover { text-decoration: underline; }
  .toast { position: fixed; bottom: 1.5rem; left: 50%; transform: translateX(-50%);
           padding: 0.6rem 1.2rem; border-radius: 8px; font-size: 0.85rem;
           opacity: 0; transition: opacity 0.3s; pointer-events: none; }
  .toast.show { opacity: 1; }
  .toast.ok { background: #0f5132; color: #75d9a3; }
  .toast.err { background: #5c1a1a; color: #f77; }
  .spinner { display: none; margin: 0.3rem auto; width: 18px; height: 18px;
             border: 2px solid #555; border-top-color: #5dade2;
             border-radius: 50%; animation: spin 0.8s linear infinite; }
  @keyframes spin { to { transform: rotate(360deg); } }
</style>
</head>
<body>
<div class="card">
  <h1>ROScar Recovery</h1>
  <div id="info">
    <div class="status-row">
      <span class="label">Service</span>
      <span id="badge" class="badge unknown">...</span>
    </div>
    <div class="status-row">
      <span class="label">Since</span>
      <span id="uptime" style="font-size:0.85rem">—</span>
    </div>
    <div class="status-row">
      <span class="label">Host</span>
      <span id="hostname" style="font-size:0.85rem">—</span>
    </div>
  </div>
  <div class="spinner" id="spinner"></div>
  <div class="actions">
    <button class="btn-start" onclick="act('start')">Start</button>
    <button class="btn-restart" onclick="act('restart')">Restart</button>
    <button class="btn-stop" onclick="act('stop')">Stop</button>
  </div>
  <a class="dashboard-link" id="dashlink" href="/aio.html">Open AIO Dashboard →</a>
</div>
<div class="toast" id="toast"></div>
<script>
const badge = document.getElementById('badge');
const uptime = document.getElementById('uptime');
const hostname = document.getElementById('hostname');
const spinner = document.getElementById('spinner');
const toast = document.getElementById('toast');
const dashlink = document.getElementById('dashlink');

// Point dashboard link at port 8888 on same host
dashlink.href = `http://${location.hostname}:8888/aio.html`;

async function refresh() {
  try {
    const r = await fetch('/status');
    const d = await r.json();
    badge.textContent = d.service;
    badge.className = 'badge ' + (d.service === 'active' ? 'active' :
                                   d.service === 'failed' ? 'failed' : 'inactive');
    uptime.textContent = d.uptime || '—';
    hostname.textContent = d.hostname || '—';
  } catch { badge.textContent = 'error'; badge.className = 'badge failed'; }
}

let toastTimer;
function showToast(msg, ok) {
  toast.textContent = msg;
  toast.className = 'toast show ' + (ok ? 'ok' : 'err');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => toast.className = 'toast', 3000);
}

async function act(action) {
  document.querySelectorAll('button').forEach(b => b.disabled = true);
  spinner.style.display = 'block';
  try {
    const r = await fetch('/' + action, { method: 'POST' });
    const d = await r.json();
    showToast(d.message, d.success);
    // Wait a moment then refresh status
    setTimeout(refresh, 2000);
  } catch (e) { showToast('Request failed: ' + e.message, false); }
  finally {
    spinner.style.display = 'none';
    document.querySelectorAll('button').forEach(b => b.disabled = false);
  }
}

refresh();
setInterval(refresh, 5000);
</script>
</body>
</html>"""


class RecoveryHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/" or path == "":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode())
        elif path == "/status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(get_service_status()).encode())
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        path = urlparse(self.path).path
        if path in ("/start", "/stop", "/restart"):
            action = path.lstrip("/")
            result = service_action(action)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # Silence per-request logging


if __name__ == "__main__":
    server = http.server.HTTPServer(("0.0.0.0", PORT), RecoveryHandler)
    print(f"Recovery service listening on port {PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.shutdown()
```

- [ ] **Step 2: Commit**

```bash
git add scripts/roscar-recovery.py
git commit -m "feat(recovery): add standalone recovery HTTP server on port 9999"
```

---

### Task 2: Create the systemd service file

**Files:**
- Create: `scripts/roscar-recovery.service`

- [ ] **Step 1: Write roscar-recovery.service**

Modeled after `roscar-web.service` but much simpler — no ROS2, no DDS, no network probing. Just Python.

```ini
[Unit]
Description=ROScar1 Recovery Service (port 9999)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=brian
ExecStart=/usr/bin/python3 /home/brian/ROScar1/scripts/roscar-recovery.py
Restart=always
RestartSec=3
StartLimitIntervalSec=60
StartLimitBurst=10

TimeoutStopSec=5
KillSignal=SIGINT

StandardOutput=journal
StandardError=journal
SyslogIdentifier=roscar-recovery

[Install]
WantedBy=multi-user.target
```

Note: The `User=brian` means `sudo systemctl` in the script will require passwordless sudo for the `systemctl` commands. We need a sudoers entry.

- [ ] **Step 2: Commit**

```bash
git add scripts/roscar-recovery.service
git commit -m "feat(recovery): add systemd unit for recovery service"
```

---

### Task 3: Add sudoers rule for passwordless service control

**Files:**
- Create: `scripts/roscar-recovery-sudoers`

- [ ] **Step 1: Write sudoers drop-in file**

This file gets installed to `/etc/sudoers.d/roscar-recovery` on the Pi. It allows user `brian` to start/stop/restart only the `roscar-web` service without a password.

```
# Allow roscar-recovery service to manage roscar-web without password
brian ALL=(ALL) NOPASSWD: /usr/bin/systemctl start roscar-web, /usr/bin/systemctl stop roscar-web, /usr/bin/systemctl restart roscar-web
```

- [ ] **Step 2: Commit**

```bash
git add scripts/roscar-recovery-sudoers
git commit -m "feat(recovery): add sudoers rule for passwordless web service control"
```

---

### Task 4: Update setup_rpi.sh with recovery service installation

**Files:**
- Modify: `scripts/setup_rpi.sh`

- [ ] **Step 1: Read current setup_rpi.sh to find the roscar-web service install section**

Find where `roscar-web.service` is installed and add a parallel block for the recovery service + sudoers file.

- [ ] **Step 2: Add recovery service install block**

After the roscar-web.service install section, add:

```bash
# --- Recovery Service (port 9999) ---
echo "Installing recovery service..."
sudo cp "$SCRIPT_DIR/roscar-recovery-sudoers" /etc/sudoers.d/roscar-recovery
sudo chmod 440 /etc/sudoers.d/roscar-recovery
sudo visudo -c  # validate sudoers syntax
chmod +x "$SCRIPT_DIR/roscar-recovery.py"
sudo cp "$SCRIPT_DIR/roscar-recovery.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable roscar-recovery
sudo systemctl start roscar-recovery
echo "Recovery service running at http://$(hostname -I | awk '{print $1}'):9999"
```

- [ ] **Step 3: Commit**

```bash
git add scripts/setup_rpi.sh
git commit -m "feat(recovery): add recovery service to RPi setup script"
```

---

### Task 5: Update CLAUDE.md

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: Add recovery service section**

Add a new section after the Web Dashboard section documenting the recovery service: what it does, what port, how to access, how to install, systemd commands.

- [ ] **Step 2: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: add recovery service to CLAUDE.md"
```

---

### Task 6: Deploy to Pi

- [ ] **Step 1: Push branch and deploy**

SSH to Pi, pull the changes, and run the setup steps:

```bash
# From dev machine — push branch
git push origin claude/confident-antonelli

# SSH to Pi
ssh roscar
cd ~/ROScar1
git fetch origin
git checkout claude/confident-antonelli  # or merge into master

# Install recovery service
chmod +x scripts/roscar-recovery.py
sudo cp scripts/roscar-recovery-sudoers /etc/sudoers.d/roscar-recovery
sudo chmod 440 /etc/sudoers.d/roscar-recovery
sudo visudo -c
sudo cp scripts/roscar-recovery.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable roscar-recovery
sudo systemctl start roscar-recovery
```

- [ ] **Step 2: Verify recovery service is running**

```bash
curl http://localhost:9999/status
# Expected: {"service": "active", ...}
```

- [ ] **Step 3: Verify restart button works**

```bash
curl -X POST http://localhost:9999/restart
# Expected: {"success": true, "message": "restart completed"}
```

- [ ] **Step 4: Verify from browser**

Navigate to `http://roscar:9999/` — should see status page with working buttons.
