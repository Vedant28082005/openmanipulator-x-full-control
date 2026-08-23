#!/usr/bin/env python3
"""Standalone MJPEG streamer for the Raspberry Pi AI Camera (IMX500).

Deliberately a SEPARATE process from digital_twin_gui.py, and deliberately
running under the SYSTEM python rather than the arm app's venv:

  - picamera2/libcamera are system packages. Pulling them into the venv would
    mean rebuilding it with --system-site-packages, which drags every system
    module into the control loop's import namespace for no benefit.
  - Process isolation is the actual requirement. A camera that is unplugged,
    wedged, or throwing inside libcamera can only ever kill THIS process. The
    arm keeps running, because nothing here shares an interpreter with it.

Never blocks on the camera: if the sensor is missing or fails to open, the
HTTP server still comes up and reports that plainly, retrying in the
background so a hot-plugged camera starts working without a restart.

Endpoints
  /stream.mjpg   multipart MJPEG, or 503 when there is no camera
  /snapshot.jpg  single frame, or 503
  /status        JSON: {"available": bool, "detail": str, "clients": int}
"""
import io
import json
import os
import socketserver
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

BIND = os.environ.get("OMX_CAM_BIND", "127.0.0.1")
PORT = int(os.environ.get("OMX_CAM_PORT", "8091"))
WIDTH = int(os.environ.get("OMX_CAM_WIDTH", "1280"))
HEIGHT = int(os.environ.get("OMX_CAM_HEIGHT", "720"))
RETRY_SECONDS = float(os.environ.get("OMX_CAM_RETRY", "10"))
# How long the sensor keeps running after the last viewer leaves. Long enough
# that a page reload or a tab switch does not tear the pipeline down and pay
# the ~1s restart cost again; short enough that walking away stops the drain.
IDLE_LINGER_SECONDS = float(os.environ.get("OMX_CAM_LINGER", "10"))
# The sensor happily produces 30 fps, but that is ~7 Mbit/s of MJPEG - far
# too much to push through a phone tunnel. Cap it; a viewer can ask for
# more with ?fps=N when watching over the LAN.
DEFAULT_FPS = float(os.environ.get("OMX_CAM_FPS", "12"))


class FrameBuffer:
    """Holds the newest JPEG. Readers wait on a Condition rather than polling,
    so a slow client can never make the encoder wait - it just misses frames."""

    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()
        self.seq = 0

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.seq += 1
            self.condition.notify_all()

    def wait_for_frame(self, last_seq, timeout=5.0):
        with self.condition:
            if self.seq == last_seq:
                self.condition.wait(timeout)
            return self.frame, self.seq


class CameraWorker(threading.Thread):
    """Owns the camera. Any failure is caught, reported, and retried - it must
    never raise out of this thread, or the HTTP server would lose its only
    source of status information.

    Capture is DEMAND-DRIVEN. The sensor and the MJPEG encoder only run while
    something is actually watching. Left running unconditionally this process
    measured ~47% of a core with zero viewers connected - by a wide margin the
    largest consumer on the Pi, and enough to hold it at its thermal limit for
    no benefit whatsoever."""

    daemon = True

    def __init__(self, buffer):
        super().__init__(name="camera")
        self.buffer = buffer
        self.available = False       # a camera exists and could be started
        self.streaming = False       # the sensor is actually running right now
        self.detail = "starting"
        self._stop = threading.Event()
        self._demand = threading.Event()   # set while a viewer wants frames
        self._viewers = 0
        self._viewer_lock = threading.Lock()
        self._last_demand = 0.0

    # -- viewer accounting, called from the HTTP threads -------------------

    def add_viewer(self):
        with self._viewer_lock:
            self._viewers += 1
            self._last_demand = time.monotonic()
            self._demand.set()

    def remove_viewer(self):
        with self._viewer_lock:
            self._viewers = max(0, self._viewers - 1)
            self._last_demand = time.monotonic()

    def poke(self):
        """A one-off frame request (snapshot). Keeps the sensor alive for the
        linger window without holding a viewer slot."""
        with self._viewer_lock:
            self._last_demand = time.monotonic()
            self._demand.set()

    def _wanted(self):
        with self._viewer_lock:
            if self._viewers > 0:
                return True
            return (time.monotonic() - self._last_demand) < IDLE_LINGER_SECONDS

    def run(self):
        while not self._stop.is_set():
            try:
                self._session()
            except Exception as exc:               # noqa: BLE001 - see docstring
                self.available = False
                self.detail = "%s: %s" % (type(exc).__name__, exc)
                print("[camera] %s" % self.detail, flush=True)
            if self._stop.wait(RETRY_SECONDS):
                break

    def _session(self):
        from picamera2 import Picamera2
        from picamera2.encoders import MJPEGEncoder
        from picamera2.outputs import FileOutput

        cams = Picamera2.global_camera_info()
        if not cams:
            self.available = False
            self.streaming = False
            self.detail = "no camera detected"
            print("[camera] no camera detected; retrying", flush=True)
            return

        model = cams[0].get("Model", "camera")
        self.available = True
        self.detail = "%s @ %dx%d (idle)" % (model, WIDTH, HEIGHT)

        picam = Picamera2()
        try:
            config = picam.create_video_configuration(main={"size": (WIDTH, HEIGHT)})
            picam.configure(config)

            while not self._stop.is_set():
                # Idle: the camera is configured and ready, but the sensor and
                # encoder are stopped. This is the whole point - running them
                # with nobody watching measured ~47% of a core and helped hold
                # the board at its thermal limit.
                if not self._wanted():
                    self._demand.clear()
                    # Wait on the event rather than polling, so an idle camera
                    # costs one blocked thread and no CPU at all.
                    self._demand.wait(timeout=1.0)
                    continue

                output = FileOutput(_BufferWriter(self.buffer))
                picam.start_recording(MJPEGEncoder(), output)
                self.streaming = True
                self.detail = "%s @ %dx%d" % (model, WIDTH, HEIGHT)
                print("[camera] streaming (viewer connected)", flush=True)
                try:
                    while not self._stop.is_set() and self._wanted():
                        time.sleep(0.25)
                finally:
                    picam.stop_recording()
                    self.streaming = False
                    self.detail = "%s @ %dx%d (idle)" % (model, WIDTH, HEIGHT)
                    with self.buffer.condition:
                        # Wake any reader still blocked on wait_for_frame so it
                        # notices streaming stopped instead of hanging for its
                        # full timeout.
                        self.buffer.condition.notify_all()
                    print("[camera] idle (no viewers) - sensor stopped", flush=True)
        finally:
            self.available = False
            self.streaming = False
            try:
                picam.close()
            except Exception:                      # noqa: BLE001
                pass

    def stop(self):
        self._stop.set()


class _BufferWriter(io.BufferedIOBase):
    """picamera2's FileOutput expects a file-like object."""

    def __init__(self, buffer):
        self.buffer_ = buffer

    def writable(self):
        return True

    def write(self, buf):
        self.buffer_.write(bytes(buf))
        return len(buf)


FRAMES = FrameBuffer()
WORKER = None
CLIENTS = threading.Semaphore(8)      # cap concurrent viewers


class Handler(BaseHTTPRequestHandler):
    server_version = "OMXCamera"
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):
        pass                                        # a polling panel would flood

    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0].rstrip("/") or "/"
        if path in ("/status", "/"):
            self._json(200, {
                "available": bool(WORKER and WORKER.available),
                "streaming": bool(WORKER and WORKER.streaming),
                "detail": WORKER.detail if WORKER else "not started",
                "width": WIDTH, "height": HEIGHT,
            })
        elif path == "/snapshot.jpg":
            # A single frame still needs the sensor spun up; poke demand and
            # let the linger window stop it again afterwards.
            if WORKER:
                WORKER.poke()
            frame, _ = FRAMES.wait_for_frame(-1, timeout=6.0)
            if not (WORKER and WORKER.available) or frame is None:
                self._json(503, {"error": "camera unavailable",
                                 "detail": WORKER.detail if WORKER else ""})
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(frame)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(frame)
        elif path == "/stream.mjpg":
            self._stream(self._requested_fps())
        else:
            self._json(404, {"error": "not found"})

    def _requested_fps(self):
        """?fps=N, clamped to something sane. Bad input falls back to default
        rather than 500 erroring - a mistyped query should not break the feed."""
        if "?" not in self.path:
            return DEFAULT_FPS
        for part in self.path.split("?", 1)[1].split("&"):
            if part.startswith("fps="):
                try:
                    return max(1.0, min(30.0, float(part[4:])))
                except ValueError:
                    break
        return DEFAULT_FPS

    def _stream(self, max_fps):
        if not (WORKER and WORKER.available):
            self._json(503, {"error": "camera unavailable",
                             "detail": WORKER.detail if WORKER else ""})
            return
        if not CLIENTS.acquire(blocking=False):
            self._json(503, {"error": "too many viewers"})
            return
        # Registering here is what actually starts the sensor, and the matching
        # remove_viewer in the finally block is what lets it stop again.
        WORKER.add_viewer()
        try:
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-store, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            seq = -1
            min_interval = 1.0 / max_fps
            next_due = 0.0
            while True:
                frame, seq = FRAMES.wait_for_frame(seq, timeout=5.0)
                if frame is None or not (WORKER and WORKER.available):
                    break
                # Drop frames rather than queue them: a viewer on a slow link
                # should see current reality at a lower rate, not a growing
                # backlog of stale frames.
                now = time.monotonic()
                if now < next_due:
                    continue
                next_due = now + min_interval
                self.wfile.write(b"--FRAME\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(
                    ("Content-Length: %d\r\n\r\n" % len(frame)).encode())
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass                                    # viewer navigated away
        finally:
            WORKER.remove_viewer()
            CLIENTS.release()


class ThreadedHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    global WORKER
    WORKER = CameraWorker(FRAMES)
    WORKER.start()
    server = ThreadedHTTPServer((BIND, PORT), Handler)
    print("[camera] http://%s:%d/  (stream.mjpg, snapshot.jpg, status)"
          % (BIND, PORT), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        WORKER.stop()


if __name__ == "__main__":
    sys.exit(main())
