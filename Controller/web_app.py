"""Mobile web control panel for the OpenManipulator-X digital twin.

Runs an HTTP server inside the same process as the Tk GUI and exposes every
control the desktop panel has, laid out for a phone. Stdlib only - no extra
dependencies beyond what requirements.txt already pulls in.

Threading contract, which is the whole reason this file is structured the way
it is: Tk is not thread-safe, so the HTTP threads never touch a widget. They
only ever do one of two things:

  * schedule work onto the Tk thread with `root.after(0, ...)`, reusing the
    exact same `on_*` handlers the desktop buttons call, so the web UI and
    the desktop UI can never drift apart in behaviour; or
  * read `app.web_state`, a plain dict rebuilt ON the Tk thread every 100 ms
    and swapped in under a lock.

Anything that mutates `app.target` still goes through `app.lock`, same as the
keyboard, gamepad and slider paths.
"""

import hmac
import json
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote

WEB_PORT = 8080
WEB_BIND = "0.0.0.0"       # reachable from the phone; see SECURITY note in README
STATE_PERIOD_MS = 100      # how often the Tk thread republishes web_state

# Shared-secret gate, off unless OMX_WEB_TOKEN is set in the environment.
# On a private lab LAN it can stay off. It must NOT stay off once the panel is
# reachable from the internet through a tunnel: this endpoint moves a physical
# arm, and an unauthenticated public URL means anyone who finds it can too.
# Opening  http://host:8080/?token=SECRET  once sets a cookie, so the phone
# does not have to carry the token in every later request.
WEB_TOKEN = os.environ.get("OMX_WEB_TOKEN", "").strip()
TOKEN_COOKIE = "omx_token"

# A held jog button on a phone is only safe if losing the phone stops the arm.
# The browser re-sends "press" every JOG_HEARTBEAT_MS while a pad is held; if
# a key goes this long without a refresh (screen lock, wifi drop, tab closed,
# finger slid off the button) the watchdog releases it.
JOG_WATCHDOG_S = 0.45
JOG_HEARTBEAT_MS = 150


def lan_ip():
    """Best-effort LAN address to print in the console.

    Uses a UDP socket to a public address purely to ask the routing table
    which local interface would be used - no packet is actually sent."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
        finally:
            s.close()
    except OSError:
        return "127.0.0.1"


# --------------------------------------------------------------------------
# State snapshot (built on the Tk thread)
# --------------------------------------------------------------------------

def _btn_state(widget):
    """'normal'/'disabled' for a Tk button, so the phone greys out exactly
    the same controls the desktop panel does."""
    try:
        return str(widget["state"])
    except Exception:
        return "normal"


def cpu_temp_c():
    """Pi/Linux CPU temperature, or None where the kernel doesn't expose it."""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return round(int(f.read().strip()) / 1000.0, 1)
    except (OSError, ValueError):
        return None


def build_snapshot(app, g):
    """Assemble the full UI state as plain JSON-able data.

    MUST run on the Tk thread - it reads Tk variables and widget states.
    `g` is the digital_twin_gui module (passed in to avoid a circular import).
    """
    with app.lock:
        target = [float(v) for v in app.target]
        ticks = dict(app.present_ticks)
        frame_count = len(app.frames)
        frame_dur = float(app.frames[-1][0]) if app.frames else 0.0

    joints = []
    for label, ctrl_idx, dxl_id, lo, hi, kind in g.JOINT_SPECS:
        # Slider bounds are widened/narrowed at runtime by the hardware's own
        # EEPROM limits, so read the live range rather than the constant.
        live_lo, live_hi = app.joint_limits.get(ctrl_idx, (lo, hi))
        joints.append({
            "label": label, "idx": ctrl_idx, "id": dxl_id, "kind": kind,
            "lo": float(live_lo), "hi": float(live_hi),
            "value": target[ctrl_idx],
        })

    feedback = []
    for label, ctrl_idx, dxl_id, lo, hi, kind in g.JOINT_SPECS:
        tick = ticks.get(dxl_id)
        row = {"label": label, "id": dxl_id, "tick": "-", "value": "-", "deg": "-"}
        if tick is not None:
            row["tick"] = str(tick)
            if kind == "gripper":
                val = g.tick_to_gripper(tick, lo, hi,
                                        app.gripper_tick_at_lo, app.gripper_tick_at_hi)
                row["value"] = f"{val:+.4f} m"
            else:
                rad = g.tick_to_rad(tick)
                row["value"] = f"{rad:+.3f} rad"
                row["deg"] = f"{g.math.degrees(rad):+.1f}"
        feedback.append(row)

    try:
        recordings = sorted(f for f in os.listdir(g.RECORDINGS_DIR) if f.endswith(".json"))
    except OSError:
        recordings = []

    return {
        "joints": joints,
        "feedback": feedback,
        "flags": {
            "connected": bool(app.hw.connected),
            "torque": bool(app.hw.torque_on),
            "mirror": bool(app.mirror_var.get()),
            "cartesian": bool(app.cartesian_jog_var.get()),
            "loop": bool(app.loop_var.get()),
            "gamepad": bool(app.gamepad_var.get()),
            "recording": bool(app.recording),
            "playing": bool(app.playing),
            "homing": bool(app.homing),
            "calibrating": bool(app.calibrating),
        },
        "status": {
            "hw": app.status_var.get(),
            "home": app.home_status_var.get(),
            "record": app.record_status_var.get(),
            "ik": app.ik_status_var.get(),
            "tune": app.tune_status_var.get(),
            "gamepad": app.gamepad_status_var.get(),
            "gamepad_raw": app.gamepad_raw_var.get(),
            "mirror_note": app.mirror_note_var.get(),
            "ee": app.ee_readout_var.get(),
        },
        # Mirroring the desktop panel's own enable/disable logic rather than
        # re-deriving it keeps the two UIs in lockstep as that logic evolves.
        "enabled": {
            "connect": _btn_state(app.connect_btn),
            "disconnect": _btn_state(app.disconnect_btn),
            "torque": _btn_state(app.torque_btn),
            "diagnose": _btn_state(app.diag_btn),
            "calibrate": _btn_state(app.calib_btn),
            "record": _btn_state(app.record_btn),
            "play": _btn_state(app.play_btn),
            "home": _btn_state(app.home_btn),
            "tune_apply": _btn_state(app.tune_apply_btn),
            "tune_read": _btn_state(app.tune_read_btn),
        },
        "labels": {
            "record": app.record_btn["text"],
            "play": app.play_btn["text"],
        },
        "tuning": {
            "target": app.tune_target_var.get(),
            "targets": list(app.tune_target_combo["values"]),
            "p": app.tune_p_var.get(),
            "d": app.tune_d_var.get(),
            "vel": app.tune_vel_var.get(),
            "acc": app.tune_acc_var.get(),
        },
        "ik": {axis: var.get() for axis, var in app.ik_entries.items()},
        "recording_info": {"frames": frame_count, "duration": frame_dur},
        "recordings": recordings,
        "keys": {
            "joint": sorted(g.KEY_BINDINGS),
            "cartesian": sorted(g.CARTESIAN_KEY_BINDINGS),
        },
        # Host telemetry - the whole point of teleop is that nobody is standing
        # next to the machine, so its own health has to be visible remotely.
        "host": {
            "name": socket.gethostname(),
            "ip": app.web_host_ip,
            "port": app.web_host_port,
            "auth": bool(WEB_TOKEN),
            "cpu_c": cpu_temp_c(),
            "uptime_s": int(time.monotonic() - app.web_started_at),
            "viewer": bool(g.OPTS.viewer),
            "panel": bool(g.OPTS.panel),
            "record_hz": g.RECORD_RATE,
            "feedback_hz": g.HW_FEEDBACK_RATE,
        },
    }


# --------------------------------------------------------------------------
# Actions (dispatched onto the Tk thread)
# --------------------------------------------------------------------------

def _safe_recording_name(name):
    """Confine a web-supplied filename to RECORDINGS_DIR.

    basename() strips any directory part, so "../../.bashrc" collapses to
    ".bashrc"; forcing the .json suffix then keeps this endpoint from being
    able to write anywhere else or to any other file type."""
    name = os.path.basename(str(name or "")).strip()
    if not name:
        raise ValueError("empty filename")
    if not name.endswith(".json"):
        name += ".json"
    return name


def _set_joint(app, idx, value, g):
    lo, hi = app.joint_limits.get(idx, (-3.15, 3.15))
    value = max(lo, min(hi, float(value)))
    for var, label, ctrl_idx, scale in app.scale_vars:
        if ctrl_idx == idx:
            var.set(value)      # drags the desktop slider to match
            break
    app._on_slider(idx, value)


def perform(app, g, action, body):
    """Run one web action. Returns a small dict; the real feedback the phone
    sees is the next /api/state poll, exactly like the desktop panel's own
    status lines."""
    tk_call = lambda fn: app.root.after(0, fn)

    # --- hardware ---------------------------------------------------------
    if action == "connect":
        tk_call(app.on_connect)
    elif action == "disconnect":
        tk_call(app.on_disconnect)
    elif action == "torque":
        tk_call(app.on_enable_torque)
    elif action == "estop":
        # Deliberately NOT routed through root.after: an e-stop must not
        # queue behind whatever else the Tk thread is doing. on_estop only
        # flips torque off through the hardware lock, no widget access.
        threading.Thread(target=app.on_estop, daemon=True).start()
    elif action == "diagnose":
        tk_call(app.on_diagnose_gripper)
    elif action == "calibrate":
        tk_call(app.on_calibrate_gripper)
    elif action == "home":
        tk_call(app.on_go_home)

    # --- joints and jogging ----------------------------------------------
    elif action == "joint":
        idx, value = int(body["idx"]), float(body["value"])
        tk_call(lambda: _set_joint(app, idx, value, g))
    elif action == "jog":
        key = str(body.get("key", ""))
        if key not in g.KEY_BINDINGS and key not in g.CARTESIAN_KEY_BINDINGS:
            return {"ok": False, "error": "unknown jog key"}
        if body.get("state") == "press":
            # Same set the physical keyboard feeds, so the sim loop applies
            # web jog through the identical rate-limited, clamped path.
            app.keys_held.add(key)
            app.web_jog_deadline[key] = time.monotonic() + JOG_WATCHDOG_S
        else:
            app.keys_held.discard(key)
            app.web_jog_deadline.pop(key, None)
    elif action == "jog_release_all":
        for key in list(app.web_jog_deadline):
            app.keys_held.discard(key)
            app.web_jog_deadline.pop(key, None)
    elif action == "cartesian":
        want = bool(body.get("value"))
        def _set_cart():
            app.cartesian_jog_var.set(want)
            app.on_toggle_cartesian_jog()
        tk_call(_set_cart)

    # --- inverse kinematics ----------------------------------------------
    elif action == "ik":
        def _solve():
            for axis in ("X", "Y", "Z"):
                app.ik_entries[axis].set(str(body.get(axis.lower(), "")))
            app.on_solve_ik()
        tk_call(_solve)

    # --- mirror -----------------------------------------------------------
    elif action == "mirror":
        want = bool(body.get("value"))
        def _set_mirror():
            app.mirror_var.set(want)
            app.on_toggle_mirror()
        tk_call(_set_mirror)

    # --- teach by demonstration ------------------------------------------
    elif action == "record":
        tk_call(app.on_toggle_record)
    elif action == "play":
        tk_call(app.on_toggle_play)
    elif action == "loop":
        want = bool(body.get("value"))
        tk_call(lambda: app.loop_var.set(want))
    elif action == "clear":
        tk_call(app.on_clear_recording)
    elif action == "save":
        name = _safe_recording_name(body.get("name"))
        os.makedirs(g.RECORDINGS_DIR, exist_ok=True)
        path = os.path.join(g.RECORDINGS_DIR, name)
        n = app.save_recording_to(path)
        return {"ok": True, "saved": name, "frames": n}
    elif action == "load":
        name = _safe_recording_name(body.get("name"))
        path = os.path.join(g.RECORDINGS_DIR, name)
        if not os.path.isfile(path):
            return {"ok": False, "error": "no such recording"}
        tk_call(lambda: app.load_recording_from(path))

    # --- live gain tuning -------------------------------------------------
    elif action == "gains_apply":
        def _apply():
            app.tune_target_var.set(str(body.get("target", app.tune_target_var.get())))
            app.tune_p_var.set(str(body.get("p", "")))
            app.tune_d_var.set(str(body.get("d", "")))
            app.tune_vel_var.set(str(body.get("vel", "")))
            app.tune_acc_var.set(str(body.get("acc", "")))
            app.on_apply_gains()
        tk_call(_apply)
    elif action == "gains_read":
        tk_call(app.on_read_gains)

    # --- gamepad ----------------------------------------------------------
    elif action == "gamepad_connect":
        tk_call(app.on_connect_gamepad)
    elif action == "gamepad_enable":
        want = bool(body.get("value"))
        def _set_gp():
            app.gamepad_var.set(want)
            app.on_toggle_gamepad()
        tk_call(_set_gp)

    else:
        # Raised, not returned: an unrecognised endpoint is a client error,
        # so it should come back as HTTP 400 rather than a 200 carrying a
        # failure the caller has to remember to inspect.
        raise ValueError(f"unknown action '{action}'")

    return {"ok": True}


# --------------------------------------------------------------------------
# HTTP server
# --------------------------------------------------------------------------

class _Handler(BaseHTTPRequestHandler):
    app = None
    gui = None
    server_version = "OpenManipulatorX-Web"

    def log_message(self, *args):
        pass    # a polling phone would otherwise flood the console

    def _send(self, code, body, ctype="application/json", set_token_cookie=False):
        payload = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        if set_token_cookie:
            # HttpOnly so page scripts can't read it back out; SameSite=Lax so
            # another site can't drive the arm with the browser's cookie.
            self.send_header("Set-Cookie",
                             f"{TOKEN_COOKIE}={WEB_TOKEN}; Path=/; Max-Age=31536000; "
                             "HttpOnly; SameSite=Lax")
        self.end_headers()
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError):
            pass    # phone navigated away mid-response

    def _authorised(self):
        """True when no token is configured, or the request carries it.

        Checked against a constant-time compare so a wrong token cannot be
        recovered by timing the response."""
        if not WEB_TOKEN:
            return True
        supplied = ""
        query = self.path.split("?", 1)
        if len(query) == 2:
            for part in query[1].split("&"):
                if part.startswith("token="):
                    supplied = unquote(part[len("token="):])
        if not supplied:
            for chunk in (self.headers.get("Cookie") or "").split(";"):
                name, _, value = chunk.strip().partition("=")
                if name == TOKEN_COOKIE:
                    supplied = value
        return hmac.compare_digest(supplied, WEB_TOKEN)

    def _deny(self):
        self._send(401, json.dumps({"error": "unauthorised"}))

    def do_GET(self):
        if not self._authorised():
            self._deny()
            return
        path = self.path.split("?", 1)[0]
        if path == "/":
            self._send(200, PAGE_HTML, "text/html; charset=utf-8",
                       set_token_cookie=WEB_TOKEN and "token=" in self.path)
        elif path == "/api/state":
            with self.app.web_state_lock:
                state = self.app.web_state
            self._send(200, json.dumps(state))
        elif path == "/manifest.webmanifest":
            self._send(200, json.dumps({
                "name": "OpenManipulator-X", "short_name": "OMX",
                "display": "standalone", "background_color": "#12151a",
                "theme_color": "#12151a", "start_url": "/",
            }), "application/manifest+json")
        else:
            self._send(404, json.dumps({"error": "not found"}))

    def do_POST(self):
        if not self._authorised():
            self._deny()
            return
        path = self.path.split("?", 1)[0]
        if not path.startswith("/api/"):
            self._send(404, json.dumps({"error": "not found"}))
            return
        action = path[len("/api/"):]
        try:
            length = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(length) or b"{}") if length else {}
            result = perform(self.app, self.gui, action, body)
        except Exception as exc:      # never let one bad request kill the UI
            self._send(400, json.dumps({"ok": False, "error": str(exc)}))
            return
        self._send(200, json.dumps(result))


def start(app, gui_module, port=WEB_PORT, bind=WEB_BIND):
    """Start the web panel alongside the Tk GUI.

    Returns the ThreadingHTTPServer, or None if the port could not be bound -
    a busy port must not stop the desktop app from coming up."""
    app.web_state = {}
    app.web_state_lock = threading.Lock()
    app.web_jog_deadline = {}
    app.web_started_at = time.monotonic()
    app.web_host_ip = lan_ip()
    app.web_host_port = port

    _Handler.app = app
    _Handler.gui = gui_module

    def publish():
        """Rebuild the snapshot on the Tk thread and expire stale jog keys."""
        now = time.monotonic()
        for key, deadline in list(app.web_jog_deadline.items()):
            if now > deadline:
                app.keys_held.discard(key)
                app.web_jog_deadline.pop(key, None)
        try:
            snapshot = build_snapshot(app, gui_module)
        except Exception as exc:
            snapshot = {"error": str(exc)}
        with app.web_state_lock:
            app.web_state = snapshot
        if not app.stop_event.is_set():
            app.root.after(STATE_PERIOD_MS, publish)

    try:
        httpd = ThreadingHTTPServer((bind, port), _Handler)
    except OSError as exc:
        print(f"[web] could not start on {bind}:{port} - {exc}", flush=True)
        return None

    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    app.root.after(STATE_PERIOD_MS, publish)

    # flush=True: stdout is block-buffered when redirected to a file or pipe,
    # which would otherwise hide the address you need until the app exits.
    print(f"[web] mobile control panel: http://{lan_ip()}:{port}/  (also http://localhost:{port}/)",
          flush=True)
    if WEB_TOKEN:
        print(f"[web] token auth ENABLED - open http://{lan_ip()}:{port}/?token=... once "
              "to store the cookie.", flush=True)
    else:
        print("[web] NOTE: no authentication - anyone on this network can move the arm. "
              "Set OMX_WEB_TOKEN before exposing this beyond a trusted LAN.", flush=True)
    return httpd


# --------------------------------------------------------------------------
# The page. Single self-contained document - a phone on a lab network may
# have no route to the internet, so nothing is loaded from a CDN.
# --------------------------------------------------------------------------

PAGE_CSS = """
*,*::before,*::after{box-sizing:border-box}
:root{
  --bg:#0f1216; --card:#181d25; --card-2:#1f2630; --line:#2b3441;
  --txt:#e8edf4; --muted:#8b97a8; --dim:#5f6b7d;
  --red:#e8402a; --red-dim:#8f2a1c; --amber:#f5a524; --green:#20c997; --cyan:#22b8cf;
  --r:14px; --tap:48px;
  --safe-b:env(safe-area-inset-bottom,0px);
}
html,body{margin:0;padding:0;background:var(--bg);color:var(--txt);
  font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif;
  -webkit-text-size-adjust:100%;overscroll-behavior-y:none}
body{padding-bottom:calc(72px + var(--safe-b))}

/* ---- header ---- */
header{position:sticky;top:0;z-index:20;background:rgba(15,18,22,.94);
  backdrop-filter:blur(10px);border-bottom:1px solid var(--line);
  padding:10px 14px calc(10px) 14px}
.hrow{display:flex;align-items:center;gap:10px}
.brand{font-weight:700;font-size:15px;letter-spacing:.02em;flex:1;min-width:0}
.brand small{display:block;font-weight:500;font-size:11px;color:var(--dim);
  letter-spacing:.06em;text-transform:uppercase}
.estop{flex:none;background:var(--red);color:#fff;border:0;border-radius:10px;
  font-weight:800;font-size:12px;letter-spacing:.04em;padding:0 14px;height:44px;
  box-shadow:0 2px 0 var(--red-dim);cursor:pointer}
.estop:active{transform:translateY(2px);box-shadow:none}
.pills{display:flex;gap:6px;margin-top:8px;flex-wrap:wrap}
/* Says, in words, what the arm will actually do if you touch something. The
   pills above are precise but assume you know what "TORQUE" means; an operator
   who does not needs to be told plainly whether the real arm is live. */
.statebar{margin-top:8px;font-size:12.5px;line-height:1.45;padding:8px 10px;
  border-radius:9px;border:1px solid var(--line);background:var(--card)}
.statebar b{font-weight:700}
.statebar.live{border-color:var(--amber);background:rgba(245,165,36,.12)}
.statebar.live b{color:var(--amber)}
.statebar.idle{border-color:var(--line)}
.statebar.sim{border-color:var(--line);color:var(--muted)}
.pill{font-size:12px;font-weight:600;padding:5px 10px;border-radius:99px;
  border:1px solid var(--line);color:var(--muted);background:var(--card);white-space:nowrap}
.pill.on{color:#04150f;background:var(--green);border-color:transparent}
.pill.warn{color:#1a1204;background:var(--amber);border-color:transparent}
.pill.live{color:#fff;background:var(--red);border-color:transparent}

/* ---- layout ---- */
main{padding:14px}
.panel{display:none} .panel.show{display:block}
.card{background:var(--card);border:1px solid var(--line);border-radius:var(--r);
  padding:14px;margin-bottom:12px}
.card h2{margin:0 0 2px;font-size:13px;letter-spacing:.06em;text-transform:uppercase;color:var(--cyan)}
.card p.hint{margin:0 0 12px;font-size:12px;color:var(--dim);line-height:1.45}
.status{font-size:12px;color:var(--muted);line-height:1.5;margin-top:10px;
  padding-top:10px;border-top:1px solid var(--line);word-wrap:break-word}

/* ---- sliders ---- */
.jrow{margin-bottom:16px}
.jhead{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:7px}
.jname{font-size:13px;font-weight:600}
.jname span{color:var(--dim);font-weight:500}
.jval{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:13px;color:var(--cyan)}
/* Full tap-target height: the visible track stays slim, the grabbable strip
   around it is what gets to 44px so a thumb can actually catch it. */
input[type=range]{-webkit-appearance:none;appearance:none;width:100%;height:44px;
  background:transparent;margin:0}
input[type=range]::-webkit-slider-runnable-track{height:8px;border-radius:99px;background:var(--card-2);border:1px solid var(--line)}
input[type=range]::-moz-range-track{height:8px;border-radius:99px;background:var(--card-2);border:1px solid var(--line)}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:26px;height:26px;
  border-radius:50%;background:var(--txt);border:3px solid var(--red);margin-top:-9px}
input[type=range]::-moz-range-thumb{width:26px;height:26px;border-radius:50%;
  background:var(--txt);border:3px solid var(--red)}
input[type=range]:disabled{opacity:.4}

/* ---- buttons ---- */
button{font-family:inherit}
.btn{display:flex;align-items:center;justify-content:center;gap:6px;
  min-height:var(--tap);padding:0 14px;border-radius:11px;border:1px solid var(--line);
  background:var(--card-2);color:var(--txt);font-size:13px;font-weight:600;
  cursor:pointer;-webkit-tap-highlight-color:transparent;width:100%}
.btn:active{background:#2a323e}
.btn[disabled]{opacity:.35;pointer-events:none}
.btn.primary{background:var(--red);border-color:transparent;color:#fff}
.btn.ghost{background:transparent}
.btn.live{background:var(--red);border-color:transparent;color:#fff}
.grid{display:grid;gap:8px}
.g2{grid-template-columns:1fr 1fr}
.g3{grid-template-columns:repeat(3,1fr)}

/* ---- jog pads ---- */
.pad{display:grid;gap:8px;grid-template-columns:1fr auto 1fr;align-items:center;
  margin-bottom:10px}
.pad .lbl{text-align:center;font-size:12px;font-weight:700;color:var(--muted);
  letter-spacing:.05em;min-width:74px}
.jogbtn{min-height:56px;font-size:20px;font-weight:700;border-radius:12px;
  border:1px solid var(--line);background:var(--card-2);color:var(--txt);
  cursor:pointer;-webkit-tap-highlight-color:transparent;touch-action:none;user-select:none}
.jogbtn:disabled{opacity:.35}
.jogbtn.held{background:var(--red);border-color:transparent;color:#fff}

/* ---- toggles ---- */
.tog{display:flex;align-items:center;justify-content:space-between;gap:12px;
  min-height:var(--tap);padding:6px 0;border-top:1px solid var(--line)}
.tog:first-of-type{border-top:0}
.tog .t{font-size:13px;font-weight:600}
.tog .s{font-size:11px;color:var(--dim);margin-top:2px;line-height:1.4}
.sw{flex:none;width:52px;height:31px;border-radius:99px;background:var(--card-2);
  border:1px solid var(--line);position:relative;cursor:pointer;transition:background .15s}
.sw::after{content:"";position:absolute;top:3px;left:3px;width:23px;height:23px;
  border-radius:50%;background:var(--muted);transition:transform .15s,background .15s}
.sw.on{background:var(--red);border-color:transparent}
.sw.on::after{transform:translateX(21px);background:#fff}

/* ---- inputs / table ---- */
label.f{display:block;font-size:11px;color:var(--muted);margin-bottom:5px;
  letter-spacing:.04em;text-transform:uppercase}
input[type=text],input[type=number],select{width:100%;height:var(--tap);padding:0 12px;
  border-radius:11px;border:1px solid var(--line);background:var(--card-2);
  color:var(--txt);font-size:15px;font-family:ui-monospace,SFMono-Regular,Menlo,monospace}
input:focus,select:focus{outline:2px solid var(--cyan);outline-offset:-1px}
select{font-family:inherit;appearance:none}
table{width:100%;border-collapse:collapse;font-size:12px}
th{text-align:left;color:var(--dim);font-weight:600;font-size:10px;
  letter-spacing:.07em;text-transform:uppercase;padding:0 0 7px}
td{padding:7px 0;border-top:1px solid var(--line);
  font-family:ui-monospace,SFMono-Regular,Menlo,monospace}
td.n{font-family:inherit;font-weight:600}
.mono{font-family:ui-monospace,SFMono-Regular,Menlo,monospace}

/* ---- tab bar ---- */
nav{position:fixed;left:0;right:0;bottom:0;z-index:20;display:flex;
  background:rgba(15,18,22,.96);backdrop-filter:blur(10px);
  border-top:1px solid var(--line);padding-bottom:var(--safe-b)}
nav button{flex:1;background:none;border:0;color:var(--dim);padding:9px 2px 8px;
  font-size:10px;font-weight:600;letter-spacing:.03em;cursor:pointer;
  -webkit-tap-highlight-color:transparent}
nav button .ic{display:block;font-size:19px;margin-bottom:2px;line-height:1}
nav button.on{color:var(--red)}
.toast{position:fixed;left:50%;transform:translateX(-50%);bottom:calc(80px + var(--safe-b));
  background:var(--card-2);border:1px solid var(--line);color:var(--txt);
  padding:11px 16px;border-radius:11px;font-size:13px;z-index:40;
  opacity:0;transition:opacity .2s;pointer-events:none;max-width:86vw;text-align:center}
.toast.show{opacity:1}
.offline{background:var(--red);color:#fff;text-align:center;padding:7px;
  font-size:12px;font-weight:700;display:none}
.offline.show{display:block}
/* Camera. The wrap keeps a 16:9 box whether or not a frame ever arrives, so
   the page does not jump when the feed appears, disappears, or reconnects. */
.camwrap{position:relative;width:100%;aspect-ratio:16/9;background:#0b0e12;
  border:1px solid var(--line);border-radius:8px;overflow:hidden}
.camwrap img{width:100%;height:100%;object-fit:contain;display:none}
.camwrap img.live{display:block}
.camoff{position:absolute;inset:0;display:flex;align-items:center;
  justify-content:center;text-align:center;padding:12px;color:var(--dim);
  font-size:13px;line-height:1.5}
.camwrap img.live + .camoff{display:none}
"""


PAGE_BODY = """
<div class="offline" id="offline">CONNECTION TO ROBOT LOST</div>
<header>
  <div class="hrow">
    <div class="brand">OpenManipulator-X<small>Digital Twin Control</small></div>
    <button class="estop" id="estop">TORQUE OFF<br>E-STOP</button>
  </div>
  <div class="pills" id="pills"></div>
  <div class="statebar sim" id="statebar">Connecting...</div>
</header>

<main>
  <!-- ============ CONTROL ============ -->
  <section class="panel show" id="p-control">
    <div class="card">
      <h2>Camera</h2>
      <div class="camwrap">
        <img id="cam" alt="Live camera feed">
        <div class="camoff" id="cam-off">Camera offline</div>
      </div>
      <div class="status" id="s-cam"></div>
      <button class="btn" id="cam-btn">Pause feed</button>
    </div>
    <div class="card">
      <h2>Joint Control</h2>
      <p class="hint">Drag to set each joint target. The twin always follows; the real arm follows too once torque is on.</p>
      <div id="joints"></div>
      <button class="btn" id="home">Home Position</button>
      <div class="status" id="s-home"></div>
    </div>
  </section>

  <!-- ============ JOG ============ -->
  <section class="panel" id="p-jog">
    <div class="card">
      <h2>Jog</h2>
      <p class="hint">Hold a button to move continuously - release to stop. Mirrors the desktop keyboard jog exactly.</p>
      <div class="tog">
        <div><div class="t">Cartesian mode</div>
             <div class="s">Drive the end-effector in X/Y/Z via IK instead of individual joints.</div></div>
        <div class="sw" id="sw-cartesian"></div>
      </div>
      <div id="pads" style="margin-top:14px"></div>
      <div class="status" id="s-jog">Hold to jog. Releasing, locking the screen or losing wifi stops the arm.</div>
    </div>
  </section>

  <!-- ============ IK ============ -->
  <section class="panel" id="p-ik">
    <div class="card">
      <h2>Inverse Kinematics</h2>
      <p class="hint">Type an end-effector target in meters and solve for the joint angles.</p>
      <div class="status" style="margin-top:0;border-top:0;padding-top:0" id="s-ee"></div>
      <div class="grid g3" style="margin:12px 0">
        <div><label class="f">X</label><input type="text" inputmode="decimal" id="ik-x"></div>
        <div><label class="f">Y</label><input type="text" inputmode="decimal" id="ik-y"></div>
        <div><label class="f">Z</label><input type="text" inputmode="decimal" id="ik-z"></div>
      </div>
      <div class="grid g2">
        <button class="btn ghost" id="ik-here">Use current</button>
        <button class="btn primary" id="ik-solve">Solve &amp; Move</button>
      </div>
      <div class="status" id="s-ik"></div>
    </div>
  </section>

  <!-- ============ TEACH ============ -->
  <section class="panel" id="p-teach">
    <div class="card">
      <h2>Teach by Demonstration</h2>
      <p class="hint">Turn torque off, hand-guide the arm, and record its own encoder feedback. Playback runs at the speed you taught it.</p>
      <div class="grid g2" style="margin-bottom:8px">
        <button class="btn" id="rec">Record</button>
        <button class="btn" id="play">Play</button>
      </div>
      <div class="tog">
        <div><div class="t">Loop playback</div><div class="s">Repeat until stopped.</div></div>
        <div class="sw" id="sw-loop"></div>
      </div>
      <div class="status" id="s-rec"></div>
    </div>
    <div class="card">
      <h2>Recordings</h2>
      <label class="f">Save current as</label>
      <div class="grid g2" style="grid-template-columns:1fr auto;margin-bottom:12px">
        <input type="text" id="rec-name" placeholder="demo-1">
        <button class="btn" id="rec-save" style="width:auto;padding:0 20px">Save</button>
      </div>
      <label class="f">Load saved</label>
      <select id="rec-list"></select>
      <div class="grid g2" style="margin-top:8px">
        <button class="btn" id="rec-load">Load</button>
        <button class="btn ghost" id="rec-clear">Clear</button>
      </div>
    </div>
  </section>

  <!-- ============ SETUP ============ -->
  <section class="panel" id="p-setup">
    <div class="card">
      <h2>Hardware</h2>
      <p class="hint">U2D2 /dev/ttyUSB0 @ 1,000,000 bps, protocol 2.0. Nothing moves until torque is enabled.</p>
      <div class="grid g2" style="margin-bottom:8px">
        <button class="btn" id="hw-connect">Connect</button>
        <button class="btn" id="hw-disconnect">Disconnect</button>
      </div>
      <button class="btn primary" id="hw-torque" style="margin-bottom:8px">Enable Torque</button>
      <div class="grid g2">
        <button class="btn ghost" id="hw-diag">Diagnose Gripper</button>
        <button class="btn ghost" id="hw-calib">Calibrate Gripper</button>
      </div>
      <div class="status" id="s-hw"></div>
    </div>

    <div class="card">
      <h2>Live Motor Feedback</h2>
      <table><thead><tr><th>Joint</th><th>Tick</th><th>Value</th><th>Deg</th></tr></thead>
        <tbody id="fb"></tbody></table>
      <div class="tog" style="margin-top:12px;border-top:1px solid var(--line)">
        <div><div class="t">Mirror mode</div>
             <div class="s">The REAL arm drives the twin - stops sending commands.</div></div>
        <div class="sw" id="sw-mirror"></div>
      </div>
      <div class="status" id="s-mirror"></div>
    </div>

    <div class="card">
      <h2>Position Gain Tuning</h2>
      <p class="hint">Applies live with torque on. Vibrating at rest: lower P in ~200 steps. Stuttering while moving: adjust Vel.</p>
      <label class="f">Joint</label>
      <select id="tn-target" style="margin-bottom:12px"></select>
      <div class="grid g2" style="margin-bottom:8px">
        <div><label class="f">P gain</label><input type="text" inputmode="numeric" id="tn-p"></div>
        <div><label class="f">D gain</label><input type="text" inputmode="numeric" id="tn-d"></div>
      </div>
      <div class="grid g2" style="margin-bottom:12px">
        <div><label class="f">Profile vel</label><input type="text" inputmode="numeric" id="tn-vel"></div>
        <div><label class="f">Profile acc</label><input type="text" inputmode="numeric" id="tn-acc"></div>
      </div>
      <div class="grid g2">
        <button class="btn ghost" id="tn-read">Read current</button>
        <button class="btn primary" id="tn-apply">Apply</button>
      </div>
      <div class="status" id="s-tune"></div>
    </div>

    <div class="card">
      <h2>Host</h2>
      <p class="hint">The machine physically wired to the arm.</p>
      <table><tbody id="hostinfo"></tbody></table>
    </div>

    <div class="card">
      <h2>Gamepad Teleop</h2>
      <p class="hint">A pad plugged into the host machine, not the phone. Left stick base/shoulder, right stick wrist/elbow, LB/RB gripper.</p>
      <button class="btn" id="gp-connect" style="margin-bottom:8px">Connect Gamepad</button>
      <div class="tog">
        <div><div class="t">Enable gamepad</div><div class="s">Uses the Cartesian mode toggle above.</div></div>
        <div class="sw" id="sw-gamepad"></div>
      </div>
      <div class="status" id="s-gp"></div>
    </div>
  </section>
</main>

<nav>
  <button class="on" data-tab="control"><span class="ic">&#9707;</span>Control</button>
  <button data-tab="jog"><span class="ic">&#10021;</span>Jog</button>
  <button data-tab="ik"><span class="ic">&#8982;</span>IK</button>
  <button data-tab="teach"><span class="ic">&#9210;</span>Teach</button>
  <button data-tab="setup"><span class="ic">&#9881;</span>Setup</button>
</nav>
<div class="toast" id="toast"></div>
"""


PAGE_JS = """
const $ = id => document.getElementById(id);
let dragging = null;      // joint index currently under the finger
let editing  = null;      // text field currently focused
let lastOk   = Date.now();
let CURF     = {};      // newest flags, for handlers that must know the state

function toast(msg){
  const t = $('toast'); t.textContent = msg; t.classList.add('show');
  clearTimeout(t._t); t._t = setTimeout(()=>t.classList.remove('show'), 1800);
}

async function post(action, body, timeoutMs){
  const ctl = timeoutMs ? new AbortController() : null;
  const timer = ctl ? setTimeout(()=>ctl.abort(), timeoutMs) : null;
  try{
    const r = await fetch('/api/'+action, {method:'POST',
      headers:{'Content-Type':'application/json'}, body:JSON.stringify(body||{}),
      signal: ctl ? ctl.signal : undefined});
    const j = await r.json();
    if(j && j.ok === false && j.error) toast(j.error);
    return j;
  }catch(e){ return null; }
  finally{ if(timer) clearTimeout(timer); }
}

/* ---------- latency-tolerant joint sends ----------
   A slider drag fires 'input' ~60x/second. Posting each one queued a request
   per event, so on a slow link the arm replayed the ENTIRE drag path long
   after the finger stopped - the "moves late, then moves again and again"
   problem.

   Instead: at most ONE request per joint in flight, and only ever the NEWEST
   value. Intermediate positions are discarded, not queued - the operator cares
   where the slider IS, never where it passed through. This makes the send rate
   adapt to the link automatically: a fast link sends often, a slow one sends
   rarely, and neither builds a backlog.

   The timeout matters too: without it one stalled request would block that
   joint's sends forever, and the arm would stop responding with no error. */
const JSEND = {};
function sendJoint(idx, value){
  const st = JSEND[idx] || (JSEND[idx] = {pending:null, inflight:false});
  st.pending = value;                 // newest wins, overwrites any older one
  if(!st.inflight) pumpJoint(idx);
}
function pumpJoint(idx){
  const st = JSEND[idx];
  if(st.pending === null){ st.inflight = false; return; }
  const v = st.pending;
  st.pending = null;
  st.inflight = true;
  post('joint', {idx:idx, value:v}, 2000).then(()=>pumpJoint(idx));
}

/* ---------- tabs ---------- */
document.querySelectorAll('nav button').forEach(b=>{
  b.onclick = ()=>{
    document.querySelectorAll('nav button').forEach(x=>x.classList.toggle('on', x===b));
    document.querySelectorAll('.panel').forEach(p=>
      p.classList.toggle('show', p.id === 'p'+'-'+b.dataset.tab));
    window.scrollTo(0,0);
  };
});

/* ---------- jog pads ---------- */
/* [label, key that decreases, key that increases] - the same letters the
   desktop keyboard jog uses, so the server drives one code path. */
const JOINT_PADS = [['BASE','d','a'],['SHOULDER','s','w'],['ELBOW','k','i'],
                    ['WRIST','j','l'],['GRIPPER','u','o']];
const CART_PADS  = [['X','f','r'],['Y','g','t'],['Z','h','y']];
let padsMode = null;

function buildPads(cartesian){
  if(padsMode === cartesian) return;
  padsMode = cartesian;
  const rows = cartesian ? CART_PADS : JOINT_PADS;
  $('pads').innerHTML = rows.map(([lbl,neg,pos]) =>
    '<div class="pad">' +
      '<button class="jogbtn" data-key="'+neg+'">&minus;</button>' +
      '<div class="lbl">'+lbl+'</div>' +
      '<button class="jogbtn" data-key="'+pos+'">+</button>' +
    '</div>').join('');
  $('pads').querySelectorAll('.jogbtn').forEach(bindJog);
}

function bindJog(btn){
  const key = btn.dataset.key;
  let beat = null;
  const start = ev => {
    ev.preventDefault();
    if(btn.disabled || beat) return;
    btn.classList.add('held');
    post('jog', {key, state:'press'});
    // Re-assert while held: the server releases any key it stops hearing
    // about, so a dropped connection cannot leave the arm driving.
    beat = setInterval(()=>post('jog', {key, state:'press'}), """ + str(JOG_HEARTBEAT_MS) + """);
  };
  const stop = () => {
    if(!beat) return;
    clearInterval(beat); beat = null;
    btn.classList.remove('held');
    post('jog', {key, state:'release'});
  };
  btn.addEventListener('pointerdown', start);
  ['pointerup','pointercancel','pointerleave'].forEach(e=>btn.addEventListener(e, stop));
  btn._stop = stop;
}

function releaseAll(){
  document.querySelectorAll('.jogbtn').forEach(b=>b._stop && b._stop());
  post('jog_release_all', {});
}
document.addEventListener('visibilitychange', ()=>{ if(document.hidden) releaseAll(); });
window.addEventListener('blur', releaseAll);
window.addEventListener('pagehide', releaseAll);

/* ---------- static wiring ---------- */
$('estop').onclick = ()=>{ releaseAll(); post('estop',{}); toast('E-STOP - torque off'); };
$('home').onclick  = ()=>post('home',{});
$('hw-connect').onclick    = ()=>post('connect',{});
$('hw-disconnect').onclick = ()=>post('disconnect',{});
/* Enabling torque energises a physical arm that can move and, on release,
   drop. It is the one control here with a real-world consequence that cannot
   be undone by clicking again, so it asks first. Disabling is NEVER confirmed:
   turning power off is a safety action and must stay instant. */
$('hw-torque').onclick     = ()=>{
  if(!CURF.torque && !confirm(
      'Enable torque?\n\n'
    + 'The arm will power up and hold its position. It may move as it takes up '
    + 'slack, and controls will drive the real arm from then on.\n\n'
    + 'Check the area around the arm is clear.')) return;
  post('torque',{});
};
$('hw-diag').onclick       = ()=>post('diagnose',{});
$('hw-calib').onclick      = ()=>post('calibrate',{});
$('rec').onclick   = ()=>post('record',{});
$('play').onclick  = ()=>post('play',{});
$('rec-clear').onclick = ()=>post('clear',{});
$('tn-read').onclick   = ()=>post('gains_read',{});

$('ik-solve').onclick = ()=>post('ik',
  {x:$('ik-x').value, y:$('ik-y').value, z:$('ik-z').value});
$('ik-here').onclick = ()=>{
  // "Current EE pos: x=0.287 y=0.000 z=0.183" -> prefill the three fields
  const m = ($('s-ee').textContent||'').match(/x=(-?[\\d.]+)\\s+y=(-?[\\d.]+)\\s+z=(-?[\\d.]+)/);
  if(m){ $('ik-x').value=m[1]; $('ik-y').value=m[2]; $('ik-z').value=m[3]; }
};
$('tn-apply').onclick = ()=>post('gains_apply', {target:$('tn-target').value,
  p:$('tn-p').value, d:$('tn-d').value, vel:$('tn-vel').value, acc:$('tn-acc').value});
$('rec-save').onclick = async ()=>{
  const name = ($('rec-name').value||'').trim();
  if(!name){ toast('Name the recording first'); return; }
  const r = await post('save', {name});
  if(r && r.ok) toast('Saved '+r.saved+' ('+r.frames+' frames)');
};
$('rec-load').onclick = ()=>{
  const name = $('rec-list').value;
  if(!name){ toast('No recording selected'); return; }
  post('load', {name});
};

$('sw-cartesian').onclick = e=>post('cartesian', {value:!e.target.classList.contains('on')});
$('sw-mirror').onclick    = e=>post('mirror',    {value:!e.target.classList.contains('on')});
$('sw-loop').onclick      = e=>post('loop',      {value:!e.target.classList.contains('on')});
$('sw-gamepad').onclick   = e=>post('gamepad_enable', {value:!e.target.classList.contains('on')});
$('gp-connect').onclick   = ()=>post('gamepad_connect',{});

['ik-x','ik-y','ik-z','tn-p','tn-d','tn-vel','tn-acc','rec-name'].forEach(id=>{
  $(id).addEventListener('focus', ()=>editing = id);
  $(id).addEventListener('blur',  ()=>{ if(editing===id) editing=null; });
});

/* ---------- render ---------- */
function pill(txt, cls){ return '<span class="pill '+(cls||'')+'">'+txt+'</span>'; }

function render(s){
  const f = s.flags, st = s.status, en = s.enabled;
  CURF = f;

  const sb = $('statebar');
  if(!f.connected){
    sb.className = 'statebar sim';
    sb.innerHTML = '<b>Not connected.</b> Controls move the 3D model only \u2014 '
                 + 'the real arm will not move. Go to Setup and press Connect.';
  }else if(!f.torque){
    sb.className = 'statebar idle';
    sb.innerHTML = '<b>Connected, power OFF.</b> The arm is limp and may sag. '
                 + 'Controls move the model only. Press Enable Torque in Setup '
                 + 'to drive the real arm.';
  }else{
    sb.className = 'statebar live';
    sb.innerHTML = '<b>Arm is LIVE.</b> It is powered and holding position \u2014 '
                 + 'anything you move here moves the real arm. E-STOP cuts power '
                 + 'instantly (the arm will drop).';
  }

  $('pills').innerHTML =
      pill(f.connected ? 'CONNECTED' : 'SIM ONLY', f.connected ? 'on' : '')
    + pill(f.torque ? 'TORQUE ON' : 'TORQUE OFF', f.torque ? 'warn' : '')
    + (f.recording ? pill('REC '+s.recording_info.frames, 'live') : '')
    + (f.playing   ? pill('PLAYING', 'live') : '')
    + (f.mirror    ? pill('MIRROR', 'warn') : '')
    + (f.homing    ? pill('HOMING', 'warn') : '')
    + (f.calibrating ? pill('CALIBRATING', 'warn') : '')
    + (f.cartesian ? pill('CARTESIAN', '') : '');

  /* joints */
  const box = $('joints');
  if(box.children.length !== s.joints.length){
    box.innerHTML = s.joints.map(j =>
      '<div class="jrow"><div class="jhead">' +
        '<div class="jname">'+j.label.replace(/ - /,' <span>')+'</span></div>' +
        '<div class="jval" id="jv'+j.idx+'"></div></div>' +
        '<input type="range" id="js'+j.idx+'" min="'+j.lo+'" max="'+j.hi+
        '" step="0.001"></div>').join('');
    s.joints.forEach(j=>{
      const sl = $('js'+j.idx);
      sl.addEventListener('pointerdown', ()=>dragging = j.idx);
      ['pointerup','pointercancel'].forEach(e=>sl.addEventListener(e, ()=>{
        if(dragging===j.idx) dragging = null; }));
      sl.addEventListener('input', ()=>{
        $('jv'+j.idx).textContent = (+sl.value).toFixed(3);
        sendJoint(j.idx, +sl.value);
      });
      // On release, make sure the exact final value lands even if the last
      // in-flight request carried a slightly older one.
      ['pointerup','pointercancel'].forEach(e=>sl.addEventListener(e, ()=>{
        sendJoint(j.idx, +sl.value); }));
    });
  }
  s.joints.forEach(j=>{
    const sl = $('js'+j.idx);
    // Never fight the finger: skip the joint being dragged right now.
    if(dragging === j.idx) return;
    if(+sl.min !== j.lo) sl.min = j.lo;
    if(+sl.max !== j.hi) sl.max = j.hi;
    sl.value = j.value;
    $('jv'+j.idx).textContent = j.value.toFixed(3);
  });

  /* jog */
  buildPads(f.cartesian);
  const jogLocked = f.mirror || f.recording || f.playing || f.homing;
  document.querySelectorAll('.jogbtn').forEach(b=>b.disabled = jogLocked);

  /* feedback */
  $('fb').innerHTML = s.feedback.map(r =>
    '<tr><td class="n">'+r.label.split(' - ')[1]+'</td><td>'+r.tick+
    '</td><td>'+r.value+'</td><td>'+r.deg+'</td></tr>').join('');

  /* toggles */
  $('sw-cartesian').classList.toggle('on', f.cartesian);
  $('sw-mirror').classList.toggle('on', f.mirror);
  $('sw-loop').classList.toggle('on', f.loop);
  $('sw-gamepad').classList.toggle('on', f.gamepad);

  /* buttons follow the desktop panel's own enable logic */
  $('hw-connect').disabled    = en.connect    !== 'normal';
  $('hw-disconnect').disabled = en.disconnect !== 'normal';
  $('hw-torque').disabled     = en.torque     !== 'normal';
  $('hw-diag').disabled       = en.diagnose   !== 'normal';
  $('hw-calib').disabled      = en.calibrate  !== 'normal';
  $('rec').disabled           = en.record     !== 'normal';
  $('play').disabled          = en.play       !== 'normal';
  $('home').disabled          = en.home       !== 'normal';
  $('tn-apply').disabled      = en.tune_apply !== 'normal';
  $('tn-read').disabled       = en.tune_read  !== 'normal';
  $('rec').textContent  = s.labels.record.replace(/[^\\x20-\\x7e]/g,'').trim() || 'Record';
  $('play').textContent = s.labels.play.replace(/[^\\x20-\\x7e]/g,'').trim() || 'Play';
  $('rec').classList.toggle('live', f.recording);
  $('play').classList.toggle('live', f.playing);
  $('hw-torque').textContent = f.torque ? 'Torque is ON' : 'Enable Torque';

  /* recordings list */
  const sel = $('rec-list');
  const want = s.recordings.join('|');
  if(sel._want !== want){
    sel._want = want;
    const keep = sel.value;
    sel.innerHTML = s.recordings.length
      ? s.recordings.map(n=>'<option>'+n+'</option>').join('')
      : '<option value="">(none saved yet)</option>';
    if(s.recordings.includes(keep)) sel.value = keep;
  }

  /* tuning fields - only when the user is not typing in them */
  const tsel = $('tn-target');
  if(tsel.options.length !== s.tuning.targets.length){
    tsel.innerHTML = s.tuning.targets.map(t=>'<option>'+t+'</option>').join('');
    tsel.value = s.tuning.target;
  }
  if(editing !== 'tn-p')   $('tn-p').value   = s.tuning.p;
  if(editing !== 'tn-d')   $('tn-d').value   = s.tuning.d;
  if(editing !== 'tn-vel') $('tn-vel').value = s.tuning.vel;
  if(editing !== 'tn-acc') $('tn-acc').value = s.tuning.acc;
  if(editing !== 'ik-x')   $('ik-x').value   = s.ik.X;
  if(editing !== 'ik-y')   $('ik-y').value   = s.ik.Y;
  if(editing !== 'ik-z')   $('ik-z').value   = s.ik.Z;

  /* host telemetry */
  const h = s.host || {};
  const up = n => { const m=Math.floor(n/60), sec=n%60, hr=Math.floor(m/60);
                    return hr+'h '+String(m%60).padStart(2,'0')+'m '+String(sec).padStart(2,'0')+'s'; };
  $('hostinfo').innerHTML = [
    ['Host',     h.name || '-'],
    ['Address',  (h.ip||'-')+':'+(h.port||'-')],
    ['Auth',     h.auth ? 'token required' : 'NONE - trusted network only'],
    ['CPU temp', h.cpu_c != null ? h.cpu_c+' °C' : 'n/a'],
    ['Uptime',   h.uptime_s != null ? up(h.uptime_s) : '-'],
    ['Rates',    (h.feedback_hz||0)+' Hz feedback / '+(h.record_hz||0)+' Hz record'],
    ['3D viewer',h.viewer ? 'on' : 'off (headless)'],
  ].map(([k,v])=>'<tr><td class="n">'+k+'</td><td>'+v+'</td></tr>').join('');

  /* status lines */
  $('s-hw').textContent     = st.hw;
  $('s-home').textContent   = st.home;
  $('s-rec').textContent    = st.record;
  $('s-ik').textContent     = st.ik;
  $('s-tune').textContent   = st.tune;
  $('s-ee').textContent     = st.ee;
  $('s-mirror').textContent = st.mirror_note;
  $('s-gp').textContent     = st.gamepad + (st.gamepad_raw ? '\\n' + st.gamepad_raw : '');
}

/* ---------- poll ---------- */
async function tick(){
  try{
    const r = await fetch('/api/state', {cache:'no-store'});
    const s = await r.json();
    if(!s.error){ render(s); lastOk = Date.now(); }
  }catch(e){ /* handled by the staleness check below */ }
  $('offline').classList.toggle('show', Date.now() - lastOk > 2000);
}
setInterval(tick, 200);
tick();
"""

# Deliberately a SEPARATE <script> tag from PAGE_JS. If anything in here throws
# or fails to parse, the browser still runs the arm panel script - the camera
# cannot take the controls down with it. Same isolation rule as the separate
# service and the separate nginx location.
CAMERA_JS = """
(function(){
  try{
    var img = document.getElementById('cam');
    var off = document.getElementById('cam-off');
    var st  = document.getElementById('s-cam');
    var btn = document.getElementById('cam-btn');
    if(!img || !off || !st || !btn) return;

    var paused = false, streaming = false;

    function setOff(msg){
      streaming = false;
      img.classList.remove('live');
      img.removeAttribute('src');
      off.textContent = msg;
      st.textContent = '';
    }

    function startStream(){
      if(paused || streaming) return;
      streaming = true;
      // Cache-buster: without it a reconnect can be served the dead response
      // from the previous attempt.
      img.src = '/camera/stream.mjpg?t=' + Date.now();
      img.classList.add('live');
    }

    img.onerror = function(){
      setOff('Camera feed interrupted - retrying...');
    };

    function poll(){
      fetch('/camera/status', {cache:'no-store'})
        .then(function(r){ return r.ok ? r.json() : null; })
        .then(function(d){
          if(!d || !d.available){
            setOff(d && d.detail ? ('Camera offline - ' + d.detail)
                                 : 'Camera offline');
            return;
          }
          st.textContent = d.detail || '';
          if(!paused) startStream();
        })
        .catch(function(){
          // Camera service down, nginx 502, or no /camera/ route at all.
          // Purely cosmetic - the arm panel keeps working.
          setOff('Camera service unavailable');
        });
    }

    btn.onclick = function(){
      paused = !paused;
      btn.textContent = paused ? 'Resume feed' : 'Pause feed';
      if(paused){ setOff('Feed paused'); } else { poll(); }
    };

    setOff('Connecting to camera...');
    poll();
    setInterval(poll, 5000);
  }catch(e){ /* never let the camera break the panel */ }
})();
"""


PAGE_HTML = ("<!doctype html><html lang=\"en\"><head>"
             "<meta charset=\"utf-8\">"
             "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1,"
             "maximum-scale=1,viewport-fit=cover\">"
             "<meta name=\"theme-color\" content=\"#0f1216\">"
             "<meta name=\"mobile-web-app-capable\" content=\"yes\">"
             "<link rel=\"manifest\" href=\"/manifest.webmanifest\">"
             "<title>OpenManipulator-X</title>"
             "<style>" + PAGE_CSS + "</style></head><body>"
             + PAGE_BODY +
             "<script>" + PAGE_JS + "</script>"
             "<script>" + CAMERA_JS + "</script></body></html>")
