#!/usr/bin/env python3
"""WAVE ROVER browser keyboard controller. Python 3.9+, standard library only.

Run on the Pi; open the page locally through an SSH tunnel. Default is DRY RUN.
No imports from, or changes to, rover_wifi.py. No network configuration changes.
"""

import argparse
import json
import secrets
import signal
import threading
import time
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

POWER = 0.30
PERIOD = 0.10
LEASE = 0.45
HTTP_TIMEOUT = 0.25  # socket timeout, not a hard end-to-end delivery guarantee
STOP = (0.0, 0.0)
DIRECTIONS = {
    "KeyW": (POWER, POWER),
    "KeyS": (-POWER, -POWER),
    "KeyA": (-POWER, POWER),
    "KeyD": (POWER, -POWER),
}


class RoverTransport:
    def __init__(self, live=False):
        self.live = live
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.previous = None

    def send(self, pair):
        command = {"T": 1, "L": pair[0], "R": pair[1]}
        if not self.live:
            if pair != self.previous:
                print("DRY RUN:", json.dumps(command), flush=True)
            self.previous = pair
            return
        query = urllib.parse.urlencode({"json": json.dumps(command)})
        # One worker owns this transport: movement and STOP never run in parallel.
        with self.opener.open("http://192.168.4.1/js?" + query,
                              timeout=HTTP_TIMEOUT) as response:
            # Receiving an HTTP response is not confirmation that wheels stopped.
            if response.status != 200:
                raise OSError("Rover HTTP status %s" % response.status)


class Controller:
    def __init__(self, transport, clock=time.monotonic):
        self.transport = transport
        self.clock = clock
        self.cv = threading.Condition()
        self.session = ""
        self.seq = -1
        self.armed = False
        self.pair = STOP
        self.deadline = 0.0
        self.reason = "Disarmed — click Arm, then hold one direction key."
        self.error = ""
        self.stop_ok = False
        self.inflight = False
        self.closing = False
        self.quit = False
        self.worker = None

    def _disarm(self, reason):
        self.armed = False
        self.pair = STOP
        self.reason = reason
        self.cv.notify_all()

    def _expire(self):
        if self.armed and self.clock() >= self.deadline:
            self._disarm("Browser heartbeat expired — STOP requested; re-arm required.")

    def new_session(self):
        with self.cv:
            self.session = secrets.token_urlsafe(32)
            self.seq = -1
            self._disarm("Page opened — disarmed. Only this newest page can control.")
            return self.session

    def _status(self):
        return {"armed": self.armed, "reason": self.reason, "error": self.error,
                "stop_response": self.stop_ok, "mode": "LIVE" if self.transport.live else "DRY RUN"}

    def status(self, token):
        with self.cv:
            if not secrets.compare_digest(token, self.session) or not self.session:
                raise ValueError("Page replaced or invalid session. Reload this page.")
            self._expire()
            return self._status()

    def command(self, token, seq, action, keys):
        with self.cv:
            if not secrets.compare_digest(token, self.session) or not self.session:
                raise ValueError("Page replaced or invalid session. Reload this page.")
            if type(seq) is not int or seq < 0 or action not in ("arm", "state", "stop"):
                raise ValueError("Invalid command")
            if (not isinstance(keys, list) or len(keys) > 4
                    or any(not isinstance(k, str) or k not in DIRECTIONS for k in keys)
                    or len(set(keys)) != len(keys)):
                raise ValueError("Invalid keys")
            self._expire()  # late packets cannot revive an expired lease
            if seq <= self.seq:
                return self._status()  # an older request cannot undo a newer STOP
            self.seq = seq
            if self.closing:
                self._disarm("Controller is shutting down.")
            elif action == "stop":
                self._disarm("STOP requested — release keys and click Arm to continue.")
            elif action == "arm":
                if keys or self.pair != STOP or not self.stop_ok or self.error:
                    self._disarm("Wait for a STOP response, release keys, then try Arm again.")
                else:
                    self.armed = True
                    self.deadline = self.clock() + LEASE
                    self.reason = "Armed — hold W, A, S or D. Release to stop."
            elif self.armed:
                self.deadline = self.clock() + LEASE
                self.pair = DIRECTIONS[keys[0]] if len(keys) == 1 else STOP
                self.reason = "Moving — release to stop." if self.pair != STOP else "Armed — stopped."
            self.cv.notify_all()
            return self._status()

    def step(self):
        """One serialized send, also usable with a fake transport in tests."""
        with self.cv:
            self._expire()
            pair = self.pair if self.armed else STOP
            self.inflight = True
            # A requested STOP isn't a delivered STOP; neither is an in-flight move.
            if pair != STOP:
                self.stop_ok = False
        try:
            self.transport.send(pair)
        except Exception as exc:
            with self.cv:
                self.stop_ok = False
                self.error = str(exc)[:200]
                self._disarm("Rover request failed — retrying STOP; re-arm required.")
        else:
            with self.cv:
                self.error = ""
                self.stop_ok = pair == STOP
        finally:
            with self.cv:
                self.inflight = False
                self.cv.notify_all()

    def _run(self):
        while True:
            with self.cv:
                if self.quit:
                    return
                before = (self.session, self.armed, self.pair, self.closing)
            self.step()
            with self.cv:
                after = (self.session, self.armed, self.pair, self.closing)
                # If release/STOP arrived during HTTP, send the next STOP now.
                if before == after and not self.quit:
                    self.cv.wait(PERIOD)

    def start(self):
        self.worker = threading.Thread(target=self._run, name="rover-sender", daemon=True)
        self.worker.start()

    def close(self):
        with self.cv:
            self.closing = True
            self._disarm("Shutting down — STOP requested.")
        # Give the worker time to retry STOP. Hard process/power loss skips cleanup.
        end = self.clock() + 1.5
        with self.cv:
            while not self.stop_ok and self.clock() < end:
                self.cv.wait(min(0.10, max(0, end - self.clock())))
            self.quit = True
            self.cv.notify_all()
        if self.worker:
            self.worker.join(timeout=1.0)
        if self.worker and self.worker.is_alive():
            print("STOP sender did not exit. Check the rover and motor power.", flush=True)
        elif not self.stop_ok:
            print("STOP delivery failed. Check the rover and motor power.", flush=True)
        else:
            print("STOP HTTP response received (or simulated in dry run).", flush=True)


PAGE = r'''<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>WAVE ROVER keyboard</title>
<style>
body{background:#102128;color:#eef6f5;font:18px system-ui;max-width:700px;margin:50px auto;padding:24px}
h1{font-size:30px}button{font:inherit;padding:14px 24px;border-radius:8px;border:0;cursor:pointer;margin:8px 8px 8px 0}
#stop{background:#ef5757;color:#fff}#arm{background:#89dbc3}#status{padding:20px;background:#1c343d;border-radius:10px;min-height:55px}
kbd{background:#294751;padding:5px 10px;border-radius:5px}small{color:#b4cbce} .keys{line-height:2.7}
</style>
<h1>WAVE ROVER · <span id="mode">connecting</span></h1>
<p>Keep this page focused. Hold one key to move; release any direction key to stop.</p>
<div class="keys"><kbd>W</kbd> Forward &nbsp; <kbd>S</kbd> Backward<br>
<kbd>A</kbd> Turn left &nbsp; <kbd>D</kbd> Turn right<br><kbd>Space</kbd> Emergency STOP</div>
<button id="arm">Arm controls</button><button id="stop">STOP / Disarm</button>
<p id="status" role="status">Connecting…</p>
<small>Power 0.30 · browser updates 100 ms · Pi lease 450 ms<br>
Focus loss, hidden tab, connection error and Space disarm the controls. Click Arm to resume.<br>
Opening a new controller page disarms and replaces this one.<br>
STOP needs the Pi-to-rover Wi-Fi link. Keep motor power accessible.</small>
<script>
const token = __TOKEN__;
const held = new Set();
const directions = new Set(['KeyW','KeyA','KeyS','KeyD']);
const statusBox = document.querySelector('#status');
let seq = 0, armed = false, pending = 0, polling = false;
function show(s, allowArm = false) {
  // A poll or delayed response must never re-arm this browser after local STOP.
  if (s.armed && !armed && !allowArm) {stop(); return;}
  armed = s.armed;
  if (!armed) held.clear();
  document.querySelector('#mode').textContent = s.mode;
  statusBox.textContent = s.reason + (s.error ? ' ' + s.error : '');
}
async function send(action) {
  const n = ++seq;
  const body = JSON.stringify({seq:n, action, keys:[...held]});
  const abort = new AbortController();
  const timer = setTimeout(() => abort.abort(), 800);
  pending++;
  try {
    const r = await fetch('/control', {method:'POST', cache:'no-store',
      headers:{'Content-Type':'application/json','X-Rover-Token':token},
      body, signal:abort.signal, keepalive:action==='stop'});
    const s = await r.json();
    if (!r.ok) throw new Error(s.error || 'Request failed');
    if (n === seq) show(s, action === 'arm');
  } catch (e) {
    if (n === seq) {
      held.clear(); armed = false;
      statusBox.textContent = 'Connection lost — STOP requested. Check rover; re-arm after reconnecting.';
      if (action !== 'stop') void send('stop');
    }
  } finally {clearTimeout(timer); pending--;}
}
function stop() {held.clear(); armed = false; void send('stop');}
document.querySelector('#stop').addEventListener('click', stop);
document.querySelector('#arm').addEventListener('click', () => {
  if (held.size || document.hidden || !document.hasFocus() || pending) return;
  void send('arm');
});
window.addEventListener('keydown', e => {
  if (e.code === 'Space' || e.code === 'Escape') {
    e.preventDefault(); if (!e.repeat) stop(); return;
  }
  if ((e.ctrlKey || e.altKey || e.metaKey) && armed) {stop(); return;}
  if (!directions.has(e.code)) return;
  e.preventDefault();
  if (!armed || e.repeat || held.has(e.code)) return;
  held.add(e.code); void send('state');
});
window.addEventListener('keyup', e => {
  if (!directions.has(e.code)) return;
  e.preventDefault();
  // Clear all keys: releasing a key cannot resume a different held direction.
  held.clear(); if (armed) void send('state');
});
window.addEventListener('blur', stop);
window.addEventListener('pagehide', stop);
document.addEventListener('visibilitychange', () => {if (document.hidden) stop();});
async function poll() {
  if (polling || pending || armed) return;
  polling = true;
  const before = seq;
  const abort = new AbortController();
  const timer = setTimeout(() => abort.abort(), 800);
  try {
    const r = await fetch('/status', {cache:'no-store', headers:{'X-Rover-Token':token}, signal:abort.signal});
    const s = await r.json();
    if (!r.ok) throw new Error(s.error || 'Unavailable');
    if (before === seq) show(s);
  } catch (e) {if (before === seq) statusBox.textContent = 'Controller unavailable. Reconnect or reload this page.';}
  finally {clearTimeout(timer); polling = false;}
}
setInterval(() => {if (armed && !pending) void send('state'); else void poll();}, 100);
void poll();
</script></html>'''


def make_handler(controller, port):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(1.0)

        def log_message(self, *_args):
            pass

        def valid_host(self):
            return self.headers.get("Host") in ("127.0.0.1:%d" % port, "localhost:%d" % port)

        def reply(self, code, body, content_type="application/json"):
            data = body.encode() if isinstance(body, str) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if not self.valid_host():
                self.reply(403, {"error": "Use localhost through the SSH tunnel."})
            elif self.path == "/":
                token = controller.new_session()
                self.reply(200, PAGE.replace("__TOKEN__", json.dumps(token)), "text/html; charset=utf-8")
            elif self.path == "/status":
                try:
                    self.reply(200, controller.status(self.headers.get("X-Rover-Token", "")))
                except ValueError as exc:
                    self.reply(403, {"error": str(exc)})
            else:
                self.reply(404, {"error": "Not found"})

        def do_POST(self):
            if not self.valid_host() or self.path != "/control":
                self.reply(403, {"error": "Invalid endpoint"})
                return
            origin = self.headers.get("Origin")
            if origin and origin not in ("http://localhost:%d" % port, "http://127.0.0.1:%d" % port):
                self.reply(403, {"error": "Invalid origin"})
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 1024:
                    raise ValueError("Invalid body size")
                body = json.loads(self.rfile.read(size))
                if not isinstance(body, dict):
                    raise ValueError("Expected object")
                result = controller.command(self.headers.get("X-Rover-Token", ""),
                                            body.get("seq"), body.get("action"), body.get("keys"))
            except (ValueError, TypeError) as exc:
                self.reply(400, {"error": str(exc)})
                return
            self.reply(200, result)
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live", action="store_true", help="send real motor commands; default is dry run")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error("port must be 1024..65535")
    controller = Controller(RoverTransport(args.live))
    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(controller, args.port))
    except OSError as exc:
        parser.exit(1, "Cannot open local controller port: %s\nTry --port 18765 if the port is occupied.\n" % exc)
    server.daemon_threads = True
    server.timeout = 0.1
    exiting = threading.Event()

    def on_signal(_signum, _frame):
        exiting.set()

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_signal)
    controller.start()
    print("%s — open http://localhost:%d through an SSH tunnel." %
          ("LIVE MOTORS" if args.live else "DRY RUN (no rover requests)", args.port), flush=True)
    try:
        while not exiting.is_set():
            server.handle_request()
    finally:
        controller.close()
        server.server_close()


if __name__ == "__main__":
    main()
