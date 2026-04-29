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
    info = {
        "service": "unknown",
        "sub_state": "unknown",
        "uptime": "N/A",
        "timestamp": datetime.now().isoformat(),
        "hostname": socket.gethostname(),
    }
    try:
        result = subprocess.run(
            ["systemctl", "is-active", SERVICE],
            capture_output=True, text=True, timeout=5,
        )
        info["service"] = result.stdout.strip()
    except Exception:
        pass
    try:
        result = subprocess.run(
            ["systemctl", "show", SERVICE,
             "--property=SubState,ActiveEnterTimestamp"],
            capture_output=True, text=True, timeout=5,
        )
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
            capture_output=True, text=True, timeout=30,
        )
        return {
            "success": result.returncode == 0,
            "message": result.stderr.strip() or f"{action} completed",
        }
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
      <span id="uptime" style="font-size:0.85rem">&mdash;</span>
    </div>
    <div class="status-row">
      <span class="label">Host</span>
      <span id="hostname" style="font-size:0.85rem">&mdash;</span>
    </div>
  </div>
  <div class="spinner" id="spinner"></div>
  <div class="actions">
    <button class="btn-start" onclick="act('start')">Start</button>
    <button class="btn-restart" onclick="act('restart')">Restart</button>
    <button class="btn-stop" onclick="act('stop')">Stop</button>
  </div>
  <a class="dashboard-link" id="dashlink" href="/aio.html">Open AIO Dashboard &rarr;</a>
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
dashlink.href = 'http://' + location.hostname + ':8888/aio.html';

async function refresh() {
  try {
    const r = await fetch('/status');
    const d = await r.json();
    badge.textContent = d.service;
    badge.className = 'badge ' + (d.service === 'active' ? 'active' :
                                   d.service === 'failed' ? 'failed' : 'inactive');
    uptime.textContent = d.uptime || '\u2014';
    hostname.textContent = d.hostname || '\u2014';
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
        if path in ("/", ""):
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
