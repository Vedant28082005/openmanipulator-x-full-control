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

import base64
import hashlib
import hmac
import json
import os
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote

WEB_PORT = 8080
WEB_BIND = "0.0.0.0"       # reachable from the phone; see SECURITY note in README

# 3D viewer static assets. REPO_ROOT holds the URDF and the meshes/ directory
# it references; VENDOR_DIR holds a locally-vendored Three.js + urdf-loader,
# fetched once at build time rather than from a CDN, so the viewer keeps
# working with the Pi on an isolated network and no internet at all - the same
# reason the rest of this app is stdlib-only.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MESHES_DIR = os.path.join(REPO_ROOT, "meshes")
URDF_PATH = os.path.join(REPO_ROOT, "open_manipulator_x.xml").replace(".xml", ".urdf")
VENDOR_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static", "vendor")

_STATIC_MIME = {
    ".js": "text/javascript; charset=utf-8",
    ".stl": "model/stl",
    ".urdf": "application/xml",
}
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
            # Whether a pad is actually open, as distinct from the enable
            # toggle. The panel used to show only the toggle, so an unplugged
            # or wedged pad still read as live.
            "gamepad_present": app.gamepad is not None,
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
            "pickplace": app.pickplace_status_var.get(),
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
            "run_pick_place": _btn_state(app.pickplace_run_btn),
        },
        "pickplace": {
            name: (pose is not None) for name, pose in app.action_poses.items()
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
    """Apply one joint target, clamped to that joint's limits. Reached only
    over the websocket now - the old POST-per-slider-input route is gone, and
    with it the queueing that made dragging a slider on a slow link replay
    minutes later."""
    lo, hi = app.joint_limits.get(idx, (-3.15, 3.15))
    value = max(lo, min(hi, float(value)))
    for var, label, ctrl_idx, scale in app.scale_vars:
        if ctrl_idx == idx:
            var.set(value)      # keeps the desktop slider in step
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

    # --- quick actions: gripper presets, taught Pick && Place --------------
    elif action == "gripper_preset":
        closed = bool(body.get("closed"))
        tk_call(lambda: app.on_gripper_preset(closed))
    elif action == "capture_pose":
        name = str(body.get("name", ""))
        if name not in ("hover", "pickup", "place"):
            return {"ok": False, "error": "unknown pose"}
        tk_call(lambda: app.on_capture_pose(name))
    elif action == "run_pick_place":
        tk_call(app.on_run_pick_place)

    # --- joints -------------------------------------------------------------
    elif action == "joint":
        idx, value = int(body["idx"]), float(body["value"])
        tk_call(lambda: _set_joint(app, idx, value, g))

    # --- jogging -----------------------------------------------------------
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

# ---------------------------------------------------------------- websocket
#
# RFC 6455, implemented here rather than pulled in as a dependency because
# this file is deliberately stdlib-only - see the module docstring. Only the
# subset the panel needs: a text-frame channel, ping/pong, and a clean close.
#
# Why a socket at all, when polling worked: the joint sliders were removed
# from the panel because POST-per-input queued disastrously on a slow link -
# the arm replayed the whole drag long after the finger stopped. HTTP gives a
# client no way to see that backlog building. A socket does: bufferedAmount is
# exactly "how much have I written that has not gone out yet", so the client
# can drop stale input instead of queueing it. That is what makes putting the
# sliders back safe.

# The RFC 6455 magic string. Transposing even one character still
# produces a plausible-looking base64 accept value, which a hand-rolled
# test client will happily ignore and every real browser will reject -
# so this is checked against the spec's own test vector in the suite.
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
WS_BROADCAST_HZ = 20.0          # state pushes per second to each open socket
WS_PING_SECONDS = 15.0          # keepalive, and how fast a dead peer is noticed

_ws_clients = set()
_ws_lock = threading.Lock()


def _ws_frame(payload: bytes, opcode: int = 0x1) -> bytes:
    """Server->client frame. Never masked, per the spec."""
    header = bytearray([0x80 | opcode])
    n = len(payload)
    if n < 126:
        header.append(n)
    elif n < (1 << 16):
        header.append(126)
        header += struct.pack(">H", n)
    else:
        header.append(127)
        header += struct.pack(">Q", n)
    return bytes(header) + payload


def _ws_read_frame(rfile):
    """Reads one client frame. Returns (opcode, payload) or (None, None) at
    end of stream. Client frames are always masked; an unmasked one is a
    protocol violation and closes the connection."""
    hdr = rfile.read(2)
    if not hdr or len(hdr) < 2:
        return None, None
    b0, b1 = hdr[0], hdr[1]
    opcode = b0 & 0x0F
    masked = bool(b1 & 0x80)
    n = b1 & 0x7F
    if n == 126:
        n = struct.unpack(">H", rfile.read(2))[0]
    elif n == 127:
        n = struct.unpack(">Q", rfile.read(8))[0]
    if n > (1 << 20):           # 1 MB: nothing this protocol sends is close
        return None, None
    if not masked:
        return None, None
    mask = rfile.read(4)
    data = bytearray(rfile.read(n))
    for i in range(n):
        data[i] ^= mask[i & 3]
    return opcode, bytes(data)


class _WSClient:
    """One open socket. send() is serialised and never raises upward - a
    broadcast to a peer that has gone away must not disturb the others."""

    def __init__(self, handler):
        self.handler = handler
        self.wfile = handler.wfile
        self.lock = threading.Lock()
        self.alive = True

    def send(self, payload: bytes, opcode: int = 0x1):
        if not self.alive:
            return False
        with self.lock:
            try:
                self.wfile.write(_ws_frame(payload, opcode))
                self.wfile.flush()
                return True
            except (OSError, ValueError):
                self.alive = False
                return False

    def close(self):
        self.alive = False
        try:
            self.wfile.write(_ws_frame(b"", 0x8))
            self.wfile.flush()
        except Exception:       # noqa: BLE001 - already going away
            pass


def ws_broadcast_loop(app):
    """Pushes the state snapshot to every open socket. One thread for all of
    them rather than one per client: the snapshot is built once by the Tk
    thread anyway, and fanning it out here keeps the cost flat as viewers are
    added."""
    period = 1.0 / WS_BROADCAST_HZ
    last_ping = time.monotonic()
    while not app.stop_event.is_set():
        start = time.monotonic()
        with _ws_lock:
            clients = list(_ws_clients)
        if clients:
            with app.web_state_lock:
                snapshot = app.web_state
            try:
                payload = json.dumps({"t": "state", "s": snapshot}).encode()
            except (TypeError, ValueError):
                payload = None
            if payload is not None:
                for c in clients:
                    if not c.send(payload):
                        with _ws_lock:
                            _ws_clients.discard(c)
            if start - last_ping >= WS_PING_SECONDS:
                last_ping = start
                for c in clients:
                    c.send(b"", 0x9)     # ping; a dead peer fails the write
        time.sleep(max(0.0, period - (time.monotonic() - start)))


class _Handler(BaseHTTPRequestHandler):
    app = None
    gui = None
    server_version = "OpenManipulatorX-Web"
    # BaseHTTPRequestHandler defaults to HTTP/1.0, and a browser will refuse a
    # websocket upgrade that comes back as 1.0 - which is exactly how this
    # first failed: a raw socket client accepted the 101 and Chrome did not.
    # Safe to raise because every response path here sets Content-Length
    # (_send and _serve_static_file are the only two), and it lets the polling
    # client reuse one connection instead of reconnecting several times a
    # second.
    protocol_version = "HTTP/1.1"

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

    def _serve_static_file(self, root, rel_path):
        """Serves rel_path from under root, or 404. Rejects anything that
        would resolve outside root ("..", absolute paths, symlink escapes) -
        the URDF/mesh/vendor routes are the only places this process opens a
        file by a name that arrives in the request, so this is the one spot
        traversal actually needs blocking."""
        root = os.path.realpath(root)
        target = os.path.realpath(os.path.join(root, rel_path))
        if os.path.commonpath([root, target]) != root or not os.path.isfile(target):
            self._send(404, json.dumps({"error": "not found"}))
            return
        ext = os.path.splitext(target)[1].lower()
        ctype = _STATIC_MIME.get(ext, "application/octet-stream")
        with open(target, "rb") as f:
            body = f.read()
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        # Meshes and vendored libraries never change at runtime; the browser
        # can keep them indefinitely and re-fetch only after a restart.
        self.send_header("Cache-Control", "public, max-age=86400")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _ws_handshake(self):
        """Completes the RFC 6455 upgrade and serves this socket until it
        closes. Runs on the connection's own thread, courtesy of
        ThreadingHTTPServer, so a long-lived socket costs one thread and
        blocks nothing else."""
        key = self.headers.get("Sec-WebSocket-Key")
        if not key:
            self._send(400, json.dumps({"error": "bad websocket handshake"}))
            return
        accept = base64.b64encode(
            hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
        self.send_response(101)
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()
        try:
            self.wfile.flush()
        except OSError:
            return

        client = _WSClient(self)
        with _ws_lock:
            _ws_clients.add(client)
        # Send one snapshot immediately so the panel paints without waiting
        # for the next broadcast tick.
        with self.app.web_state_lock:
            snapshot = self.app.web_state
        try:
            client.send(json.dumps({"t": "state", "s": snapshot}).encode())
        except (TypeError, ValueError):
            pass

        try:
            while client.alive and not self.app.stop_event.is_set():
                opcode, payload = _ws_read_frame(self.rfile)
                if opcode is None or opcode == 0x8:      # closed
                    break
                if opcode == 0x9:                        # ping -> pong
                    client.send(payload, 0xA)
                    continue
                if opcode != 0x1:                        # only text carries commands
                    continue
                try:
                    msg = json.loads(payload.decode("utf-8"))
                except (ValueError, UnicodeDecodeError):
                    continue
                self._ws_handle(client, msg)
        except (OSError, ValueError):
            pass
        finally:
            with _ws_lock:
                _ws_clients.discard(client)
            client.close()
            # The socket was hijacked for the websocket; there is no next
            # request on it, so stop the keep-alive loop from waiting for one.
            self.close_connection = True

    def _ws_handle(self, client, msg):
        """One decoded client message. Commands go through the SAME perform()
        the HTTP API uses, so a control can never behave differently depending
        on which transport it arrived over."""
        kind = msg.get("t")
        if kind == "ping":
            client.send(json.dumps({"t": "pong", "seq": msg.get("seq")}).encode())
            return
        if kind != "cmd":
            return
        action = str(msg.get("action", ""))
        body = msg.get("body") or {}
        try:
            result = perform(self.app, self.gui, action, body)
        except Exception as exc:      # noqa: BLE001 - mirror do_POST's guard
            result = {"ok": False, "error": str(exc)}
        # Echo the sequence number back so the client can measure round-trip
        # latency and tell "still catching up" from "idle".
        if msg.get("seq") is not None:
            payload = {"t": "ack", "seq": msg["seq"]}
            if isinstance(result, dict) and result.get("ok") is False:
                payload["error"] = result.get("error")
            client.send(json.dumps(payload).encode())

    def do_GET(self):
        if not self._authorised():
            self._deny()
            return
        path = self.path.split("?", 1)[0]
        if (path == "/ws"
                and (self.headers.get("Upgrade") or "").lower() == "websocket"):
            self._ws_handshake()
            return
        if path == "/":
            # Landing page: pick a role. Deliberately unauthenticated - it is
            # a menu, and the guest half is meant to be reachable without a
            # password. Nothing here reads or touches robot state.
            self._send(200, LANDING_HTML, "text/html; charset=utf-8",
                       set_token_cookie=WEB_TOKEN and "token=" in self.path)
        elif path in ("/view", "/control"):
            # The SAME panel either way. The role only drives what the UI
            # offers; it is not the security boundary. That boundary is
            # nginx refusing POST /api/* without controller credentials, so a
            # guest who edits window.OMX_ROLE in a console gains buttons that
            # return 401 and nothing else.
            role = "controller" if path == "/control" else "guest"
            self._send(200, PAGE_HTML.replace("__OMX_ROLE__", role),
                       "text/html; charset=utf-8",
                       set_token_cookie=WEB_TOKEN and "token=" in self.path)
        elif path == "/api/state":
            with self.app.web_state_lock:
                state = self.app.web_state
            self._send(200, json.dumps(state))
        elif path == "/model/robot.urdf":
            self._serve_static_file(REPO_ROOT, "open_manipulator_x.urdf")
        elif path.startswith("/model/meshes/"):
            self._serve_static_file(MESHES_DIR, path[len("/model/meshes/"):])
        elif path.startswith("/static/vendor/"):
            self._serve_static_file(VENDOR_DIR, path[len("/static/vendor/"):])
        elif path == "/manifest.webmanifest":
            self._send(200, json.dumps({
                "name": "OpenManipulator-X", "short_name": "OMX",
                "display": "standalone", "background_color": "#111214",
                "theme_color": "#111214", "start_url": "/",
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

    threading.Thread(target=ws_broadcast_loop, args=(app,),
                     daemon=True, name="ws-broadcast").start()

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
/* Palette follows ISA-101 (ANSI/ISA-101.01-2015, the high-performance-HMI
   standard for process/robot control): a desaturated grayscale base, with
   color spent ONLY on states that need attention - amber for caution, red
   for alarm/critical, blue for "operator action available". A live number or
   a card title is not a deviation, so neither gets a decorative accent color
   any more; --heading and --txt (both neutral) replaced what used to be
   var(--cyan) on those. Every state that carries color also carries a text
   label (the pills already said "TORQUE ON", not just showing a colored dot),
   which is the other ISA-101 requirement - roughly 8% of men have red-green
   color vision deficiency, so color alone must never be the only signal. */
:root{
  --bg:#111214; --card:#1a1b1f; --card-2:#212226; --line:#34363c;
  --txt:#e7e8ea; --muted:#9a9da5; --dim:#6b6e76; --heading:#aeb1b8;
  --red:#e2412c; --red-dim:#7a2a20; --amber:#f2a531; --green:#2f9e6e; --info:#4d8fdb;
  --r:8px; --tap:48px;
  --safe-b:env(safe-area-inset-bottom,0px);
}
html,body{margin:0;padding:0;background:var(--bg);color:var(--txt);
  font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif;
  -webkit-text-size-adjust:100%;overscroll-behavior-y:none}
body{padding-bottom:calc(72px + var(--safe-b))}

/* ---- header ---- */
header{position:sticky;top:0;z-index:20;background:var(--bg);
  border-bottom:1px solid var(--line);
  padding:10px 14px calc(10px) 14px}
.hrow{display:flex;align-items:center;gap:10px}
.brand{font-weight:700;font-size:15px;letter-spacing:.02em;flex:1;min-width:0}
.brand small{display:block;font-weight:500;font-size:11px;color:var(--dim);
  letter-spacing:.06em;text-transform:uppercase}
.estop{flex:none;background:var(--red);color:#fff;border:0;border-radius:8px;
  font-weight:800;font-size:12px;letter-spacing:.04em;padding:0 14px;height:44px;
  box-shadow:0 2px 0 var(--red-dim);cursor:pointer}
.estop:active{transform:translateY(2px);box-shadow:none}
.pills{display:flex;gap:6px;margin-top:8px;flex-wrap:wrap}
/* Says, in words, what the arm will actually do if you touch something. The
   pills above are precise but assume you know what "TORQUE" means; an operator
   who does not needs to be told plainly whether the real arm is live. */
.statebar{margin-top:8px;font-size:12.5px;line-height:1.45;padding:8px 10px;
  border-radius:8px;border:1px solid var(--line);background:var(--card)}
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
.card h2{margin:0 0 2px;font-size:13px;letter-spacing:.06em;text-transform:uppercase;color:var(--heading)}
.card p.hint{margin:0 0 12px;font-size:12px;color:var(--dim);line-height:1.45}
.status{font-size:12px;color:var(--muted);line-height:1.5;margin-top:10px;
  padding-top:10px;border-top:1px solid var(--line);word-wrap:break-word}


/* ---- sliders ---- */
.jrow{margin-bottom:16px}
.jhead{display:flex;justify-content:space-between;align-items:baseline;margin-bottom:7px}
.jname{font-size:13px;font-weight:600}
.jname span{color:var(--dim);font-weight:500}
.jval{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:13px;color:var(--txt)}
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
  min-height:var(--tap);padding:0 14px;border-radius:8px;border:1px solid var(--line);
  background:var(--card-2);color:var(--txt);font-size:13px;font-weight:600;
  cursor:pointer;-webkit-tap-highlight-color:transparent;width:100%}
.btn:active{background:#2a323e}
.btn[disabled]{opacity:.35;pointer-events:none}
.btn.primary{background:var(--red);border-color:transparent;color:#fff}
.btn.ghost{background:transparent}
.btn.live{background:var(--red);border-color:transparent;color:#fff}
.btn.captured{border-color:var(--green);color:var(--green)}
.grid{display:grid;gap:8px}
.g2{grid-template-columns:1fr 1fr}
.g3{grid-template-columns:repeat(3,1fr)}

/* ---- jog pads ---- */
.pad{display:grid;gap:8px;grid-template-columns:1fr auto 1fr;align-items:center;
  margin-bottom:10px}
.pad .lbl{text-align:center;font-size:12px;font-weight:700;color:var(--muted);
  letter-spacing:.05em;min-width:74px}
.jogbtn{min-height:56px;font-size:20px;font-weight:700;border-radius:8px;
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
/* No pad plugged in: the toggle is meaningless, so it reads as unavailable
   rather than sitting there looking operable. */
.sw.disabled{opacity:.35;pointer-events:none}

/* ---- inputs / table ---- */
label.f{display:block;font-size:11px;color:var(--muted);margin-bottom:5px;
  letter-spacing:.04em;text-transform:uppercase}
input[type=text],input[type=number],select{width:100%;height:var(--tap);padding:0 12px;
  border-radius:8px;border:1px solid var(--line);background:var(--card-2);
  color:var(--txt);font-size:15px;font-family:ui-monospace,SFMono-Regular,Menlo,monospace}
input:focus,select:focus{outline:2px solid var(--info);outline-offset:-1px}
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
  background:var(--bg);
  border-top:1px solid var(--line);padding-bottom:var(--safe-b)}
nav button{flex:1;background:none;border:0;color:var(--dim);padding:9px 2px 8px;
  font-size:10px;font-weight:600;letter-spacing:.03em;cursor:pointer;
  -webkit-tap-highlight-color:transparent}
nav button .ic{display:block;font-size:19px;margin-bottom:2px;line-height:1}
nav button.on{color:var(--red)}
.toast{position:fixed;left:50%;transform:translateX(-50%);bottom:calc(80px + var(--safe-b));
  background:var(--card-2);border:1px solid var(--line);color:var(--txt);
  padding:11px 16px;border-radius:8px;font-size:13px;z-index:40;
  opacity:0;transition:opacity .2s;pointer-events:none;max-width:86vw;text-align:center}
.toast.show{opacity:1}
.offline{background:var(--red);color:#fff;text-align:center;padding:7px;
  font-size:12px;font-weight:700;display:none}
.offline.show{display:block}

/* ---- responsive: tablet and desktop ----------------------------------
   Everything above is mobile-first and correct on a phone. On a wider screen
   the same single column stretches each slider across the full width, which
   puts the label at one edge and the value at the other and makes a 24-inch
   monitor harder to use than a handset.

   Past 720px the cards flow into an auto-fitting grid with a capped measure,
   so they sit side by side instead of stretching. align-items:start stops a
   short card being padded out to match a tall neighbour.

   The tab bar stays fixed at the bottom rather than moving to the top: it
   comes after <main> in the document, so making it static would drop it below
   the content. Instead it becomes a centred floating bar, which reads as
   deliberate on a desktop rather than as a phone UI stretched wide. */
@media (min-width:720px){
  main{max-width:1000px;margin:0 auto;padding:18px}
  .panel.show{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));
    gap:14px;align-items:start}
  .card{margin-bottom:0}
  header{padding:12px 18px}
  .hrow,.pills,.statebar{max-width:1000px;margin-left:auto;margin-right:auto}
  nav{left:50%;right:auto;transform:translateX(-50%);width:min(600px,94vw);
    border:1px solid var(--line);border-bottom:0;border-radius:10px 10px 0 0}
  nav button{padding:11px 2px 10px;font-size:11px}
  .toast{bottom:calc(92px + var(--safe-b))}
}
@media (min-width:1180px){
  main{max-width:1300px;padding:22px}
  .hrow,.pills,.statebar{max-width:1300px}
  /* Once there is room, let the live feed use two columns - it is the one
     card whose usefulness scales with size. :has() degrades silently to the
     normal one-column card on a browser that lacks it. */
  .panel.show > .card:has(.camwrap){grid-column:span 2}
}
/* A mouse has no 48px finger, but shrinking targets would hurt touch laptops
   and tablets, so sizes are left alone; only the pointer feedback changes. */
@media (hover:hover) and (pointer:fine){
  .btn:hover{background:#2a323e}
  nav button:hover{color:var(--txt)}
  .jogbtn:hover{background:#2a323e}
}
/* Camera. The wrap keeps a 16:9 box whether or not a frame ever arrives, so
   the page does not jump when the feed appears, disappears, or reconnects. */
.camwrap{position:relative;width:100%;aspect-ratio:16/9;background:#0a0b0d;
  border:1px solid var(--line);border-radius:8px;overflow:hidden}
.camwrap img{width:100%;height:100%;object-fit:contain;display:none}
.camwrap img.live{display:block}
.camoff{position:absolute;inset:0;display:flex;align-items:center;
  justify-content:center;text-align:center;padding:12px;color:var(--dim);
  font-size:13px;line-height:1.5}
.camwrap img.live + .camoff{display:none}
/* 3D digital twin. Same fixed-box pattern as the camera, own class so the
   two are never coupled - the viewer can fail to load WebGL/the model without
   touching camera markup or vice versa. */
.viewer3d-wrap{position:relative;width:100%;aspect-ratio:4/3;background:#0a0b0d;
  border:1px solid var(--line);border-radius:8px;overflow:hidden}
.viewer3d-wrap canvas{width:100%;height:100%;display:block;touch-action:none}
.viewer3d-off{position:absolute;inset:0;display:flex;align-items:center;
  justify-content:center;text-align:center;padding:12px;color:var(--dim);
  font-size:13px;line-height:1.5;background:#0a0b0d}
.viewer3d-off.hide{display:none}
.credit{text-align:center;font-size:11px;color:var(--dim);letter-spacing:.02em;
  padding:4px 0 2px}
/* Guest mode. Hidden rather than removed from the DOM on purpose: render()
   and the static wiring both look elements up by id every poll, and deleting
   them would turn every tick into a null dereference. This is presentation
   only - the actual boundary is nginx refusing POST /api/* without controller
   credentials, so unhiding these in devtools buys a guest nothing but 401s. */
body.guest [data-ctl]{display:none !important}
.viewbar{background:var(--info);color:#04121f;text-align:center;padding:7px 12px;
  font-size:12px;font-weight:700;letter-spacing:.02em}
.viewbar a{color:#04121f;text-decoration:underline}
"""


PAGE_BODY = """
<div class="viewbar" id="viewbar" style="display:none">VIEW ONLY &mdash; you are signed in as a guest. <a href="/control">Sign in to control</a></div>
<div class="offline" id="offline">CONNECTION TO ROBOT LOST</div>
<header>
  <div class="hrow">
    <div class="brand">OpenManipulator-X<small>Digital Twin Control</small></div>
    <button class="estop" data-ctl id="estop">TORQUE OFF<br>E-STOP</button>
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
      <h2>3D View</h2>
      <p class="hint">Digital twin, visual only — drag to orbit, scroll or pinch to zoom.</p>
      <div class="viewer3d-wrap">
        <canvas id="viewer3d"></canvas>
        <div class="viewer3d-off" id="viewer3d-off">Loading 3D view...</div>
      </div>
      <div class="status" id="s-viewer3d"></div>
    </div>
    <div class="card" data-ctl>
      <h2>Joint Control</h2>
      <p class="hint">Drag to set each joint target. The twin always follows; the real arm follows too once torque is on. Sent over a realtime socket, which drops intermediate positions rather than queueing them &mdash; so a slow link costs you resolution, never a delayed replay.</p>
      <div id="joints"></div>
      <button class="btn" id="home">Home Position</button>
      <div class="status" id="s-home"></div>
    </div>
  </section>

  <!-- ============ JOG ============ -->
  <section class="panel" data-ctl id="p-jog">
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
  <section class="panel" data-ctl id="p-ik">
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
  <section class="panel" data-ctl id="p-teach">
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
      <h2>Quick Actions</h2>
      <p class="hint">Open/Close move the gripper to its calibrated ends in one press. Pick &amp; Place is ROBOTIS's own flagship OpenManipulator-X demo, taught for your bench rather than guessed: jog to a hover height above the object and tap Capture Hover, then the same for Pickup (lowered onto it) and Place (drop-off). Run plays hover → pickup → close → hover → place → open → hover, eased the same way Home Position moves.</p>
      <div class="grid g2" style="margin-bottom:8px">
        <button class="btn" id="grip-open">Open Gripper</button>
        <button class="btn" id="grip-close">Close Gripper</button>
      </div>
      <div class="grid g3" style="margin-bottom:8px">
        <button class="btn ghost" id="cap-hover">Capture Hover</button>
        <button class="btn ghost" id="cap-pickup">Capture Pickup</button>
        <button class="btn ghost" id="cap-place">Capture Place</button>
      </div>
      <button class="btn primary" id="run-pickplace" disabled>Run Pick &amp; Place</button>
      <div class="status" id="s-pickplace"></div>
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
    <div class="card" data-ctl>
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

    <div class="card" data-ctl>
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

    <div class="card" data-ctl>
      <h2>Gamepad Teleop</h2>
      <p class="hint">A pad plugged into the host machine, not the phone. Left stick base/shoulder, right stick wrist/elbow, LB/RB gripper. <b>Y</b> homes the arm, <b>X</b> plays the saved recording, <b>A</b> turns torque on, <b>B</b> is E-STOP (torque off). B works even with the toggle below off, since stopping must always be possible; A needs it ticked, since energising the arm is the direction that can hurt. Plugging a pad in is detected automatically — Rescan is only needed if that misses it.</p>
      <button class="btn" id="gp-connect" style="margin-bottom:8px">Rescan for Gamepad</button>
      <div class="tog">
        <div><div class="t">Enable gamepad</div><div class="s">Uses the Cartesian mode toggle above.</div></div>
        <div class="sw" id="sw-gamepad"></div>
      </div>
      <div class="status" id="s-gp"></div>
    </div>
  </section>
  <div class="credit">Built by Dr. Ravi Kant &amp; Vedant Sutariya</div>
</main>

<nav>
  <button class="on" data-tab="control"><span class="ic">&#9707;</span>Control</button>
  <button data-ctl data-tab="jog"><span class="ic">&#10021;</span>Jog</button>
  <button data-ctl data-tab="ik"><span class="ic">&#8982;</span>IK</button>
  <button data-ctl data-tab="teach"><span class="ic">&#9210;</span>Teach</button>
  <button data-tab="setup"><span class="ic">&#9881;</span>Setup</button>
</nav>
<div class="toast" id="toast"></div>
"""


PAGE_JS = """
const $ = id => document.getElementById(id);
let dragging = null;      // joint index currently under the finger

/* Role, injected per-request by the server (/view vs /control). Applied as a
   body class so a stylesheet does the hiding; no element is removed, so every
   lookup in the rest of this file still resolves. */
const GUEST = (window.OMX_ROLE === 'guest');
if(GUEST){
  document.body.classList.add('guest');
  const vb = document.getElementById('viewbar');
  if(vb) vb.style.display = 'block';
}
let editing  = null;      // text field currently focused
let lastOk   = Date.now();
let CURF     = {};      // newest flags, for handlers that must know the state

function toast(msg){
  const t = $('toast'); t.textContent = msg; t.classList.add('show');
  clearTimeout(t._t); t._t = setTimeout(()=>t.classList.remove('show'), 1800);
}

/* How long without a successful poll before we treat the link as dead and stop
   accepting commands. Two poll periods plus slack. */
const STALE_MS = 2500;
const POLL_MS  = 200;
const REQ_TIMEOUT_MS = 4000;

function linkStale(){ return Date.now() - lastOk > STALE_MS; }

/* ---------- realtime socket ----------
   Controllers get a websocket; guests stay on polling, since they have no
   commands to send and no latency to care about. If the socket will not open
   or drops, everything falls back to the HTTP path automatically - the panel
   must not become unusable because a proxy somewhere refuses upgrades.

   BACKPRESSURE is the whole point. bufferedAmount is the bytes written to the
   socket that have not yet gone out. On a healthy link it sits at 0; on a
   congested one it climbs. Checking it before sending is a direct measurement
   of "am I outrunning the link", which HTTP could not give us - and it is why
   the joint sliders are safe to have back. Over the limit we DROP the update
   rather than queue it: the operator cares where the slider is now, never
   where it passed through, so the newest value simply replaces the last one
   we failed to send. */
const WS_MAX_BUFFERED = 8192;
let ws = null, wsReady = false, wsSeq = 0, wsRtt = null;
let wsRetry = 800;

function wsConnect(){
  if(GUEST) return;                       // guests have nothing to send
  let url;
  try{
    url = (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws';
    ws = new WebSocket(url);
  }catch(e){ return; }

  ws.onopen = ()=>{ wsReady = true; wsRetry = 800; };

  ws.onmessage = ev=>{
    let m; try{ m = JSON.parse(ev.data); }catch(e){ return; }
    if(m.t === 'state' && m.s && !m.s.error){ render(m.s); lastOk = Date.now(); }
    else if(m.t === 'ack'){
      if(m.error) toast(m.error);
      if(m.seq === wsSeq) wsRtt = Date.now() - wsSentAt;
    }
  };

  const bye = ()=>{
    wsReady = false; ws = null;
    // Back off, but stay bounded: a controller who walks out of wifi range
    // should reconnect promptly on return, not after a long exponential wait.
    setTimeout(wsConnect, wsRetry);
    wsRetry = Math.min(wsRetry * 2, 5000);
  };
  ws.onclose = bye;
  ws.onerror = ()=>{ try{ ws.close(); }catch(e){} };
}

let wsSentAt = 0;
function wsSend(action, body){
  if(!wsReady || !ws || ws.readyState !== 1) return false;
  // Congested: drop this update instead of adding to the queue.
  if(ws.bufferedAmount > WS_MAX_BUFFERED) return false;
  wsSeq++;
  wsSentAt = Date.now();
  try{
    ws.send(JSON.stringify({t:'cmd', action:action, body:body||{}, seq:wsSeq}));
    return true;
  }catch(e){ return false; }
}

/* Joint targets: socket if we have one, otherwise nothing. Deliberately NOT
   falling back to POST here - an HTTP fallback for a continuous drag is
   exactly the queueing behaviour this replaced. Discrete buttons still use
   post(); only the continuous stream is socket-only. */
function sendJoint(idx, value){
  if(!wsSend('joint', {idx:idx, value:value})){
    // Nothing sent: either no socket or the link is congested. The next
    // 'input' event carries a newer value, so simply skipping is correct.
    return false;
  }
  return true;
}

async function post(action, body){
  /* Refuse to queue commands at a robot we are not currently talking to.
     A backgrounded or frozen tab used to accumulate presses - the fetches sat
     unsent, and when the connection came back the arm replayed every button
     the operator had pressed while staring at a dead screen. Dropping them is
     the only safe option: a command issued 40 seconds ago against a pose the
     arm has since left is not one you want executed.

     E-STOP is exempt. If the link is dead it will fail anyway, but a panic
     button must always be allowed to try. */
  if(action !== 'estop' && linkStale()){
    toast('Not connected - command ignored');
    return null;
  }
  /* Every request gets a deadline. Without one a stalled fetch hangs for the
     browser's own (very long) timeout and then lands late, which is the same
     replay problem by another route. */
  const ctl = new AbortController();
  const timer = setTimeout(()=>ctl.abort(), REQ_TIMEOUT_MS);
  try{
    const r = await fetch('/api/'+action, {method:'POST',
      headers:{'Content-Type':'application/json'}, body:JSON.stringify(body||{}),
      signal: ctl.signal});
    const j = await r.json();
    if(j && j.ok === false && j.error) toast(j.error);
    return j;
  }catch(e){ return null; }
  finally{ clearTimeout(timer); }
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
      'Enable torque?\\n\\n'
    + 'The arm will power up and hold its position. It may move as it takes up '
    + 'slack, and controls will drive the real arm from then on.\\n\\n'
    + 'Check the area around the arm is clear.')) return;
  post('torque',{});
};
$('hw-diag').onclick       = ()=>post('diagnose',{});
$('hw-calib').onclick      = ()=>post('calibrate',{});
$('rec').onclick   = ()=>post('record',{});
$('play').onclick  = ()=>post('play',{});
$('rec-clear').onclick = ()=>post('clear',{});
$('grip-open').onclick  = ()=>post('gripper_preset', {closed:false});
$('grip-close').onclick = ()=>post('gripper_preset', {closed:true});
$('cap-hover').onclick  = ()=>post('capture_pose', {name:'hover'});
$('cap-pickup').onclick = ()=>post('capture_pose', {name:'pickup'});
$('cap-place').onclick  = ()=>post('capture_pose', {name:'place'});
$('run-pickplace').onclick = ()=>post('run_pick_place', {});
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

  /* joints - built once, then value-synced every frame */
  const box = $('joints');
  if(box && box.children.length !== s.joints.length){
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
        if(dragging===j.idx) dragging = null;
        // Always land the exact final value, even if the last few 'input'
        // events were dropped for congestion.
        sendJoint(j.idx, +sl.value);
      }));
      sl.addEventListener('input', ()=>{
        $('jv'+j.idx).textContent = (+sl.value).toFixed(3);
        sendJoint(j.idx, +sl.value);
      });
    });
  }
  if(box){
    s.joints.forEach(j=>{
      const sl = $('js'+j.idx);
      if(!sl) return;
      // Never fight the finger: skip the joint being dragged right now.
      if(dragging === j.idx) return;
      if(+sl.min !== j.lo) sl.min = j.lo;
      if(+sl.max !== j.hi) sl.max = j.hi;
      sl.value = j.value;
      const lab = $('jv'+j.idx);
      if(lab) lab.textContent = j.value.toFixed(3);
    });
  }


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
  // Grey the toggle out when no pad is actually present, so the panel can
  // never show "enabled" for a pad that is unplugged or gone.
  $('sw-gamepad').classList.toggle('disabled', !f.gamepad_present);

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
  $('run-pickplace').disabled = en.run_pick_place !== 'normal';
  // Captured poses get a visual check so it's obvious at a glance which of
  // the three waypoints still need teaching before Run will do anything.
  ['hover','pickup','place'].forEach(function(name){
    var btn = $('cap-'+name);
    var got = !!(s.pickplace && s.pickplace[name]);
    btn.classList.toggle('captured', got);
    btn.textContent = 'Capture ' + name[0].toUpperCase() + name.slice(1) + (got ? ' \u2713' : '');
  });
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
  $('s-pickplace').textContent = st.pickplace;
  $('s-gp').textContent     = st.gamepad + (st.gamepad_raw ? '\\n' + st.gamepad_raw : '');
}

/* ---------- poll ---------- */
/* Self-scheduling rather than setInterval. setInterval fires on a fixed
   cadence whether or not the previous poll finished, so a slow link stacked
   requests, and a tab that the browser froze woke up and delivered the whole
   backlog at once - the "everything happens later, all together" behaviour.
   Chaining the next poll only after the current one settles makes overlap
   structurally impossible. */
let tickTimer = null;
function scheduleTick(ms){
  clearTimeout(tickTimer);
  tickTimer = setTimeout(tick, ms);
}

async function tick(){
  /* A hidden tab is throttled hard by every mobile browser, so polling it is
     wasted work on both ends. Stop entirely and resume on the way back; the
     stale guard then blocks commands for the one round trip it takes to get a
     fresh state, which is exactly the window in which the UI is still showing
     the operator something out of date. */
  if(document.hidden){ scheduleTick(1000); return; }

  const ctl = new AbortController();
  const timer = setTimeout(()=>ctl.abort(), REQ_TIMEOUT_MS);
  try{
    const r = await fetch('/api/state', {cache:'no-store', signal: ctl.signal});
    const s = await r.json();
    if(!s.error){ render(s); lastOk = Date.now(); }
  }catch(e){ /* surfaced by the staleness check below */ }
  finally{ clearTimeout(timer); }

  $('offline').classList.toggle('show', linkStale());
  scheduleTick(POLL_MS);
}

document.addEventListener('visibilitychange', ()=>{
  if(!document.hidden){
    // Back on screen: resync immediately rather than waiting out the slow
    // hidden-tab cadence, so controls unblock as soon as possible.
    scheduleTick(0);
  }
});

tick();
wsConnect();
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

# A THIRD separate <script> tag, same reasoning as CAMERA_JS: whatever goes
# wrong in here - WebGL unsupported, the model 404s, a vendored module fails
# to parse - must never stop the arm-control script or the camera script from
# running. Uses dynamic import() specifically so a broken/missing module
# rejects a promise this code catches, rather than throwing at parse time the
# way a static top-level `import` statement would.
VIEWER_JS = """
(function(){
  try{
    var wrap = document.querySelector('.viewer3d-wrap');
    var canvas = document.getElementById('viewer3d');
    var off = document.getElementById('viewer3d-off');
    var st = document.getElementById('s-viewer3d');
    if(!wrap || !canvas || !off) return;

    var robot = null, scene, camera, renderer, controls, group;
    // idx in /api/state's joints array -> URDF joint name. The gripper's
    // mirror partner (gripper_right_joint) is a <mimic> of this one in the
    // URDF, which urdf-loader resolves on its own - setting the left joint
    // is enough to move both.
    var JOINT_NAMES = ['joint1','joint2','joint3','joint4','gripper_left_joint'];

    function setOff(msg){ off.textContent = msg; off.classList.remove('hide'); }
    function setOn(){ off.classList.add('hide'); }

    async function boot(){
      if(!window.WebGLRenderingContext){
        setOff('3D view needs WebGL, which this browser does not support.');
        return;
      }
      var THREE, OrbitControls, URDFLoaderCtor;
      try{
        THREE = await import('/static/vendor/three.module.min.js');
        OrbitControls = (await import('/static/vendor/OrbitControls.js')).OrbitControls;
        URDFLoaderCtor = (await import('/static/vendor/URDFLoader.js')).default;
      }catch(e){
        setOff('3D view failed to load its viewer library.');
        return;
      }

      scene = new THREE.Scene();
      scene.background = new THREE.Color(0x0a0b0d);
      scene.add(new THREE.AmbientLight(0xffffff, 0.7));
      var dl = new THREE.DirectionalLight(0xffffff, 0.9);
      dl.position.set(1, 2, 1.5);
      scene.add(dl);
      scene.add(new THREE.GridHelper(1, 10, 0x2b3441, 0x1c222b));

      camera = new THREE.PerspectiveCamera(45, 4/3, 0.01, 10);
      camera.position.set(0.55, 0.4, 0.55);

      try{
        renderer = new THREE.WebGLRenderer({canvas: canvas, antialias: true});
      }catch(e){
        setOff('3D view: WebGL context could not be created.');
        return;
      }
      renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));

      controls = new OrbitControls(camera, renderer.domElement);
      controls.enableDamping = true;
      controls.target.set(0, 0.15, 0);
      controls.update();

      // URDF/ROS is Z-up; three.js's usual convention (and OrbitControls'
      // default up vector) is Y-up. Rotating the whole model onto that axis
      // once, here, is simpler than fighting the camera/controls convention
      // at every subsequent step.
      group = new THREE.Group();
      group.rotation.x = -Math.PI / 2;
      scene.add(group);

      var loader = new URDFLoaderCtor();
      loader.load('/model/robot.urdf', function(result){
        robot = result;
        group.add(robot);
        setOn();
        if(st) st.textContent = 'Digital twin only - does not run the physics simulation.';
      }, undefined, function(){
        setOff('3D view: could not load the robot model.');
      });

      function resize(){
        var w = wrap.clientWidth, h = wrap.clientHeight;
        if(!w || !h) return;
        camera.aspect = w / h;
        camera.updateProjectionMatrix();
        renderer.setSize(w, h, false);
      }
      new ResizeObserver(resize).observe(wrap);
      resize();

      (function animate(){
        requestAnimationFrame(animate);
        controls.update();
        renderer.render(scene, camera);
      })();
    }

    function poll(){
      fetch('/api/state', {cache:'no-store'})
        .then(function(r){ return r.ok ? r.json() : null; })
        .then(function(s){
          if(!s || !robot || !s.joints) return;
          s.joints.forEach(function(j, i){
            var name = JOINT_NAMES[i];
            if(name) robot.setJointValue(name, j.value);
          });
        })
        .catch(function(){ /* transient - next poll retries, cosmetic only */ });
    }

    boot();
    setInterval(poll, 150);
  }catch(e){ /* never let the 3D view break the panel or the camera */ }
})();
"""



LANDING_CSS = """
*{box-sizing:border-box}
html,body{margin:0;padding:0;min-height:100%;background:#111214;color:#e7e8ea;
  font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif;
  -webkit-text-size-adjust:100%}
.wrap{min-height:100vh;min-height:100dvh;display:flex;flex-direction:column;
  align-items:center;justify-content:center;padding:24px 18px;gap:26px}
.brand{text-align:center}
.brand h1{margin:0;font-size:clamp(20px,5vw,30px);font-weight:700;letter-spacing:.01em}
.brand p{margin:6px 0 0;font-size:clamp(11px,2.6vw,13px);color:#6b6e76;
  letter-spacing:.10em;text-transform:uppercase}
.roles{display:grid;gap:14px;width:100%;max-width:760px;
  grid-template-columns:repeat(auto-fit,minmax(260px,1fr))}
.role{background:#1a1b1f;border:1px solid #34363c;border-radius:8px;padding:20px;
  display:flex;flex-direction:column;gap:10px;text-decoration:none;color:inherit;
  transition:border-color .15s,background .15s}
.role h2{margin:0;font-size:13px;letter-spacing:.06em;text-transform:uppercase;color:#aeb1b8}
.role .lead{font-size:clamp(15px,3.4vw,17px);font-weight:600}
.role ul{margin:2px 0 0;padding-left:18px;font-size:13px;line-height:1.65;color:#9a9da5}
.role .go{margin-top:auto;padding-top:14px;font-size:13px;font-weight:600}
.role.view .go{color:#4d8fdb}
.role.ctl  .go{color:#e2412c}
.role.ctl{border-color:#4a3130}
.note{max-width:760px;font-size:12px;line-height:1.6;color:#6b6e76;text-align:center}
.credit{font-size:11px;color:#6b6e76;letter-spacing:.02em}
@media (hover:hover) and (pointer:fine){
  .role:hover{background:#212226;border-color:#4a4d55}
}
"""

LANDING_BODY = """
<div class="wrap">
  <div class="brand">
    <h1>OpenManipulator-X</h1>
    <p>Robot Arm Control</p>
  </div>

  <div class="roles">
    <a class="role view" href="/view">
      <h2>Guest</h2>
      <div class="lead">View only</div>
      <ul>
        <li>Live camera feed</li>
        <li>3D digital twin</li>
        <li>Joint positions and motor feedback</li>
        <li>Pi health: temperature, uptime, rates</li>
      </ul>
      <div class="go">Continue as guest &rarr;</div>
    </a>

    <a class="role ctl" href="/control">
      <h2>Controller</h2>
      <div class="lead">Full control</div>
      <ul>
        <li>Everything a guest can see</li>
        <li>Jog, inverse kinematics, gripper</li>
        <li>Teach, record and play back motions</li>
        <li>Torque, E-STOP and tuning</li>
      </ul>
      <div class="go">Sign in to control &rarr;</div>
    </a>
  </div>

  <p class="note">Guests cannot move the arm. Control requires a username and
  password, and that is enforced by the server &mdash; not by hiding buttons.</p>

  <div class="credit">Built by Dr. Ravi Kant &amp; Vedant Sutariya</div>
</div>
"""

LANDING_HTML = ("<!doctype html><html lang=\"en\"><head>"
                "<meta charset=\"utf-8\">"
                "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1,"
                "viewport-fit=cover\">"
                "<meta name=\"theme-color\" content=\"#111214\">"
                "<title>OpenManipulator-X</title>"
                "<style>" + LANDING_CSS + "</style></head><body>"
                + LANDING_BODY + "</body></html>")


PAGE_HTML = ("<!doctype html><html lang=\"en\"><head>"
             "<meta charset=\"utf-8\">"
             "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1,"
             "maximum-scale=1,viewport-fit=cover\">"
             "<meta name=\"theme-color\" content=\"#111214\">"
             "<meta name=\"mobile-web-app-capable\" content=\"yes\">"
             "<link rel=\"manifest\" href=\"/manifest.webmanifest\">"
             "<title>OpenManipulator-X</title>"
             "<style>" + PAGE_CSS + "</style></head><body>"
             + PAGE_BODY +
             "<script>window.OMX_ROLE=\"__OMX_ROLE__\";</script>"
             "<script>" + PAGE_JS + "</script>"
             "<script>" + CAMERA_JS + "</script>"
             "<script>" + VIEWER_JS + "</script></body></html>")
