# Prompt for Claude Code running on the Raspberry Pi

Copy everything below the line into Claude Code on the Pi. It is written to be
self-contained: it does not assume the Pi's Claude Code can see this desktop
session or the conversation that produced the `teleop` branch.

---

## Context

I have a Raspberry Pi 5 (8 GB) running the latest 64-bit Raspberry Pi OS
(Bookworm). I want to turn it into the portable brain for a **ROBOTIS
OpenManipulator-X** robot arm so I can teleoperate the arm from my phone.

The plan: the Pi is the only thing I carry besides the arm. The arm connects to
the Pi over a **U2D2 USB-to-DYNAMIXEL adapter** (`/dev/ttyUSB0`, 1,000,000 bps,
protocol 2.0, five XM430-W350 motors at IDs 11-15). The Pi runs the control
software and serves a mobile web control panel. My phone is the interface.

The software is already written and tested on an Ubuntu desktop. It lives here,
on the **`teleop` branch** (not `main`):

    https://github.com/Vedant28082005/openmanipulator-x-full-control
    branch: teleop

### What the software already does

`Controller/digital_twin_gui.py` is the application. It runs a MuJoCo
simulation of the arm that stays synchronised with the real hardware, plus:

- Per-joint sliders, keyboard jog, gamepad teleop, inverse kinematics
- Teach-by-demonstration: hand-guide the arm, record, replay
- Mirror mode (the real arm drives the simulated twin)
- Hardware safety: automatic Overload recovery, hardware-read joint limits,
  current-limited gripper, torque never engaging at a stale target
- A **desktop Tk control panel** and a **MuJoCo 3D viewer** window

`Controller/web_app.py` serves a **mobile web control panel** on port 8080 with
every one of those controls laid out for a phone, plus a permanently visible
E-STOP and live host telemetry. It starts automatically with the app. It is
stdlib-only (no Flask/FastAPI). Both UIs drive the same state and stay in sync.

It has flags for running without a screen:

    --headless     no 3D viewer, no desktop panel (web control only)
    --no-viewer    keep the Tk panel, drop the expensive 3D window
    --no-panel     hide the Tk panel, keep everything else
    --port / --bind
    --no-web

It reads `OMX_WEB_TOKEN` from the environment. When set, the web panel requires
that token (via `?token=...` once, then a cookie).

### What is already in the repo for you

    deploy/install_rpi.sh    Idempotent installer - apt packages, venv,
                             dialout group, FTDI udev rule, token generation,
                             systemd service install and verification
    deploy/omx@.service      systemd template unit; runs the app headless under
                             Xvfb so no monitor is needed

## Your tasks

### 1. Install and verify

```bash
git clone -b teleop https://github.com/Vedant28082005/openmanipulator-x-full-control.git ~/openmanipulator-x-full-control
cd ~/openmanipulator-x-full-control
./deploy/install_rpi.sh
```

Then confirm, and show me the evidence for each:

- The venv imports `mujoco`, `numpy`, `pygame`, `dynamixel_sdk`, `tkinter`
- The MuJoCo model loads
- `systemctl status omx@$USER` is active
- `curl -s "http://127.0.0.1:8080/api/state?token=$TOKEN"` returns JSON with a
  `host` block
- The panel loads on my phone at the LAN address the installer prints

**MuJoCo on ARM64 is the main risk.** If `pip install mujoco` has no aarch64
wheel for the Pi's Python version, do not silently give up: report exactly what
failed, then try (a) a slightly older `mujoco` release that does ship an
aarch64 wheel, or (b) a different Python via `pyenv`. Tell me which you used
and why. Note the app never renders on the Pi in `--headless` mode, so we only
need MuJoCo's physics, not its GL stack.

### 2. Confirm the arm actually works

With the arm powered and the U2D2 plugged in:

- `ls -l /dev/ttyUSB0` exists and is group `dialout`
- `cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer` reads **1**, not 16
  (the udev rule in the installer sets this; without it every serial read costs
  16 ms and the 60 Hz record loop cannot keep up)
- From the phone panel: **Connect**, confirm all five motors appear in the live
  feedback table with sensible ticks, then **Enable Torque** and move one joint
  a small amount

Be careful and go slowly here — this moves real hardware. Ask me before
enabling torque the first time.

### 3. Remote access beyond the LAN

I originally wanted to host this on Render or Vercel. **That cannot work** and I
understand why: those run code in a datacenter with no path to a USB serial port
on the Pi, and Vercel is serverless so it cannot hold the persistent 30 Hz
control loop. The control software must run on the Pi. What I need is a
**tunnel** that gives the Pi's port 8080 a public address.

Set up **one** of these and explain the trade-off you chose:

- **Tailscale** (recommended). A private mesh VPN. The Pi and my phone join my
  tailnet and the panel is reachable at the Pi's tailnet IP from anywhere, with
  no public exposure at all. `curl -fsSL https://tailscale.com/install.sh | sh`
  then `sudo tailscale up`. With this, also set `OMX_ARGS` to bind
  `0.0.0.0` (the tailnet interface needs it) and keep the token on.
- **Cloudflare Tunnel**. Gives a real public HTTPS URL with no port forwarding.
  Only acceptable if you put **Cloudflare Access** in front of it — a public URL
  that moves a robot arm with only a shared token is a bad idea.

Do **not** set up plain port forwarding on my router.

Whichever you choose, make it start on boot alongside `omx@$USER`, and show me
the resulting URL.

### 4. Startup behaviour

The service should already autostart. Verify by rebooting:

```bash
sudo reboot
```

After it comes back, confirm without me touching anything: service active, web
panel reachable, tunnel up. Show me `journalctl -u omx@$USER -b` for the boot.

### 5. Seeing the Tk panel remotely (optional, ask me first)

In `--headless` the Tk panel exists inside Xvfb but nothing displays it. If I
want it visible remotely, install `x11vnc` pointed at the Xvfb display so I can
VNC in and see the desktop panel and the 3D viewer. Only do this if I say yes —
it is another listening service to secure.

## Constraints

- **Never enable torque without asking me.** It energises a physical arm.
- Keep the token in `/etc/omx.env` (mode 600). Do not print it into any file
  that could get committed, and do not commit it.
- If you change application code, commit to the `teleop` branch with a clear
  message and tell me what changed and why. Do not push to `main`.
- Prefer editing `deploy/install_rpi.sh` over running one-off commands, so the
  setup stays reproducible on the next Pi.
- Tell me plainly when something does not work. I would rather hear "MuJoCo has
  no aarch64 wheel and here are the options" than get a confident summary that
  hides a broken step.

## How to know it all worked

I should be able to: take the Pi and the arm somewhere with only power, have
the Pi boot unattended, open a URL on my phone from a different network, see
live motor feedback, and jog the arm — with an E-STOP always on screen.
