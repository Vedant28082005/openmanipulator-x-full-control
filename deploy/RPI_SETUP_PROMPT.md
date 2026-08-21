# Prompt for Claude Code on the Raspberry Pi

Copy everything below the horizontal rule into Claude Code running on the Pi.
It is deliberately self-contained — it assumes no access to the desktop session
or the conversation that produced the `teleop` branch.

---

# Set up my Raspberry Pi 5 as the controller for a robot arm

## What I'm building

I have a **ROBOTIS OpenManipulator-X** robot arm and a **Raspberry Pi 5 (8 GB)**
running the latest 64-bit Raspberry Pi OS (Bookworm). I want the Pi to become
the arm's portable brain so I can teleoperate it from my phone.

The end state: I carry only the Pi and the arm. I give the Pi power, it boots
with no monitor or keyboard, and I open a URL on my phone to drive the arm —
with live motor feedback and an always-visible E-STOP.

### Physical setup

```
  Robot arm (5x DYNAMIXEL XM430-W350, IDs 11,12,13,14,15)
        |
        |  TTL daisy chain
        v
  U2D2 USB-to-DYNAMIXEL adapter  ──USB──>  Raspberry Pi 5
        ^                                       |
        |                                       | web panel :8080
   12V power to the arm                         v
   (separate from the Pi's supply)          my phone
```

The arm needs its own 12 V supply. The Pi cannot power it over USB — the U2D2
carries data only.

### The software already exists and is tested

Everything is written and working on an Ubuntu desktop. Your job is to get it
running on the Pi, not to write it. It lives on the **`teleop` branch** — not
`main`, which does not have any of this:

```
https://github.com/Vedant28082005/openmanipulator-x-full-control
branch: teleop
```

**What it is:** `Controller/digital_twin_gui.py` (~1900 lines) runs a MuJoCo
physics simulation of the arm that stays synchronised with the real hardware
over the U2D2. It provides joint sliders, keyboard jog, gamepad teleop, inverse
kinematics, teach-by-demonstration recording/playback, mirror mode, and hardware
safety features (automatic Overload recovery, joint limits read from each
motor's own EEPROM, current-limited gripper, torque never engaging at a stale
target). It has a Tk desktop panel and a MuJoCo 3D viewer window.

`Controller/web_app.py` (~1100 lines) serves a **mobile web control panel** on
port 8080 exposing every one of those controls, laid out for a phone. It is
stdlib-only — no Flask, no FastAPI, no npm. It starts automatically with the
app. Both UIs drive the same state and stay in sync live.

**Do not rewrite either file.** If something does not work on ARM, fix the
specific thing and tell me what you changed.

---

## Step 1 — Clone

```bash
cd ~
git clone -b teleop https://github.com/Vedant28082005/openmanipulator-x-full-control.git
cd openmanipulator-x-full-control
```

Confirm you are on the right branch and see the deploy directory:

```bash
git branch --show-current      # must print: teleop
ls deploy/                     # install_rpi.sh, omx@.service, RPI_SETUP_PROMPT.md
```

If `deploy/` is missing you are on `main`. Run `git checkout teleop`.

The repo path matters: the systemd unit expects
`/home/<user>/openmanipulator-x-full-control`. If you clone elsewhere, edit
`WorkingDirectory` and both `ExecStart` paths in `deploy/omx@.service`.

## Step 2 — Run the installer, then read what it did

```bash
./deploy/install_rpi.sh
```

It is idempotent — safe to re-run after a `git pull`. It performs the steps in
Step 3 below. **Read those steps anyway** so you can diagnose it if it fails
partway; do not just re-run it hoping for a different result.

## Step 3 — What the installer does, and how to do it by hand

Do these manually only if the installer fails at that step.

### 3a. System packages

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    python3-venv python3-dev python3-tk xvfb git curl libgl1 libglib2.0-0
```

- `python3-tk` — the Tk control panel. Not preinstalled, and **the app cannot
  start without it** even in headless mode (explained in 3e).
- `xvfb` — a virtual X display, so the app runs with no monitor.
- `libgl1`, `libglib2.0-0` — MuJoCo links these even when not rendering.

### 3b. Virtualenv

Raspberry Pi OS marks its system Python as externally-managed (PEP 668), so a
bare `pip install` is refused. A venv is required, not optional. Name it `opmx`
— that name is already in the repo's `.gitignore`:

```bash
cd ~/openmanipulator-x-full-control
python3 -m venv opmx
./opmx/bin/python -m pip install --upgrade pip
./opmx/bin/python -m pip install -r requirements.txt
```

`requirements.txt` is: `mujoco`, `pygame`, `numpy`, `dynamixel-sdk`.

Verify all four plus Tk import inside the venv:

```bash
./opmx/bin/python -c "
import mujoco, numpy, pygame, dynamixel_sdk, tkinter
print('mujoco', mujoco.__version__, '| numpy', numpy.__version__,
      '| pygame', pygame.__version__, '| tk', tkinter.TkVersion)"
```

> **This is the step most likely to fail.** MuJoCo on ARM64 may not have a
> prebuilt aarch64 wheel for the Pi's Python version, and building from source
> on a Pi is painful. If it fails: **tell me exactly what the error was**, then
> try, in order — (1) a slightly older `mujoco` release that does ship an
> aarch64 wheel, pinning it in `requirements.txt`; (2) a different Python
> version via `pyenv`. Report which you used and why. Note we only need
> MuJoCo's **physics**, never its renderer, since the Pi runs headless — so a
> version with a broken GL stack is still fine.

Then confirm the robot model loads:

```bash
./opmx/bin/python -c "
import mujoco
m = mujoco.MjModel.from_xml_path('open_manipulator_x.xml')
print('model OK: nq=%d nu=%d nbody=%d' % (m.nq, m.nu, m.nbody))"
```

Expected: `nq=6 nu=6 nbody=8`. If the XML is missing or stale, regenerate it
with `./opmx/bin/python urdf-xml.py` — but the committed one should be fine,
so investigate before regenerating.

### 3c. Serial port permission

```bash
sudo usermod -aG dialout $USER
```

Log out and back in for interactive use — a running shell keeps the group list
it was created with, so `groups` will look stale until you do. The systemd
service does not care; it gets the group directly via `SupplementaryGroups`.

Check the adapter is present:

```bash
ls -l /dev/ttyUSB0        # should be crw-rw---- root dialout
```

If it is absent, the U2D2 is not detected — check `dmesg | tail -20` and the
USB cable. If it appears as `ttyUSB1`, either fix the cause or change
`DEVICENAME` (see Step 5).

### 3d. FTDI latency timer — do not skip this

The U2D2's FTDI chip defaults to a **16 ms** USB latency timer. I measured this
on the desktop: every 5-motor sync-read took 16.0 ms with almost no variance,
against a 16.7 ms budget for the 60 Hz recording loop. The bus itself is not
the bottleneck — at 1 Mbaud the actual transfer is well under a millisecond.

```bash
echo 'SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"' \
  | sudo tee /etc/udev/rules.d/99-dynamixel-latency.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer     # must read 1, not 16
```

This also improves teleop responsiveness generally, not just recording.

### 3e. Configuration file

The installer creates `/etc/omx.env` (mode 600) with a generated token:

```ini
OMX_WEB_TOKEN=<32 random characters>
OMX_ARGS=--headless --bind 0.0.0.0
```

**`OMX_WEB_TOKEN`** gates the web panel. Anyone with it can move the arm. Open
`http://<pi>:8080/?token=<token>` once on the phone and it stores a cookie, so
later visits need no token in the URL.

**`OMX_ARGS`** are the app's flags:

| Flag | Effect |
|---|---|
| `--headless` | No 3D viewer, no Tk panel. Right for a carried Pi. |
| `--no-viewer` | Keep the Tk panel, drop the expensive 3D window. |
| `--no-panel` | Hide the Tk panel only. |
| `--port N` | Web panel port (default 8080). |
| `--bind ADDR` | `127.0.0.1` accepts only local + tunnelled traffic. |
| `--no-web` | Don't serve the web panel at all. |

> **Why Xvfb is needed even in `--headless` mode.** Every cross-thread call in
> this app is scheduled through Tk's event loop (`root.after`), so Tk must
> exist and be running even when no window is displayed. `--headless`
> `withdraw()`s the window rather than skipping Tk. Tk cannot initialise
> without an X display, hence `xvfb-run`. It costs a few MB and eliminates the
> entire "works on the desk, dies without a monitor" class of failure. Do not
> try to remove this by refactoring Tk out — that is a large change with no
> benefit here.

### 3f. systemd service

`deploy/omx@.service` is a **template** unit (note the `@`), instantiated per
user. The installer copies it to `/etc/systemd/system/omx@.service` and enables
`omx@<user>.service`.

```bash
sudo cp deploy/omx@.service /etc/systemd/system/omx@.service
sudo systemctl daemon-reload
sudo systemctl enable --now omx@$USER.service
systemctl status omx@$USER --no-pager
```

Two details in that unit that matter:

- `StartLimitBurst` / `StartLimitIntervalSec` are in `[Unit]`, **not**
  `[Service]`. systemd silently ignores them in `[Service]`, which would leave
  an unguarded crash-loop repeatedly re-energising a physical arm. Do not
  "tidy" them into `[Service]`.
- `ExecStart` uses `$OMX_ARGS` with a single `$`, which word-splits. Do not
  change it to `${OMX_ARGS}` — that passes the whole string as one argument
  and the app will reject it.

## Step 4 — Verify the install

Show me evidence for each, don't just assert it:

```bash
systemctl is-active omx@$USER                        # active
TOKEN=$(sudo grep -oP 'OMX_WEB_TOKEN=\K.*' /etc/omx.env)
curl -s "http://127.0.0.1:8080/api/state?token=$TOKEN" | head -c 400
hostname -I                                          # the address for my phone
```

The JSON must contain a `host` block with `viewer: false`, `panel: false`,
`auth: true`, plus CPU temp and uptime. Then confirm the panel loads on my
phone at `http://<pi-ip>:8080/?token=<token>`.

Sanity-check that auth actually works:

```bash
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8080/api/state          # 401
curl -s -o /dev/null -w '%{http_code}\n' "http://127.0.0.1:8080/api/state?token=$TOKEN"  # 200
```

## Step 5 — Only modify these if my hardware differs

All at the top of `Controller/digital_twin_gui.py` under `HARDWARE CONFIG`:

| Constant | Default | Change when |
|---|---|---|
| `DEVICENAME` | `/dev/ttyUSB0` | The adapter enumerates elsewhere. Prefer a stable `/dev/serial/by-id/...` path if you have more than one USB serial device. |
| `BAUDRATE` | `1000000` | Only if the motors were reconfigured. |
| `DXL_IDS` | `[11,12,13,14,15]` | Motor IDs differ. |
| `GRIPPER_DEG_CLOSED` / `GRIPPER_DEG_OPEN` | `125.2` / `270.4` | **Physically measured on my specific arm**, not calculated — the servo horn's assembly index is not portable. Same arm, so leave them alone; the panel's *Calibrate Gripper* can re-measure if the gripper misbehaves. |
| `POSITION_D_GAIN` | `1000` | Joints vibrate at rest (raise) or buzz (lower). Tunable live from the panel — don't edit the file to experiment. |
| `PROFILE_VELOCITY` | `70` | Motion stutters. Also tunable live. |

In `Controller/web_app.py`, `WEB_PORT` and `WEB_BIND` are defaults only —
override them via `OMX_ARGS` rather than editing the file.

**Do not commit the token or any `/etc/omx.env` content.**

## Step 6 — Remote access beyond my LAN

I originally wanted to host this on Render or Vercel. **That cannot work**, and
I understand why: both run code in a datacenter with no path to a USB serial
port on the Pi, and Vercel is serverless so it cannot hold the persistent 30 Hz
control loop. The control software must run on the Pi. What I need is a
**tunnel** giving the Pi's port 8080 a remote address.

Set up **one** of these, and explain the trade-off you chose:

- **Tailscale (recommended).** A private mesh VPN — the Pi and my phone join my
  tailnet, and nothing is publicly exposed at all.
  ```bash
  curl -fsSL https://tailscale.com/install.sh | sh
  sudo tailscale up
  tailscale ip -4
  ```
  Keep `--bind 0.0.0.0` so the tailnet interface is served, and keep the token on.

- **Cloudflare Tunnel.** Real public HTTPS URL, no port forwarding. Acceptable
  **only** behind Cloudflare Access — a public URL that moves a robot arm should
  not be protected by a shared token alone.

**Do not set up plain router port forwarding.** Make whichever you choose start
on boot alongside `omx@$USER`, and show me the resulting URL.

## Step 7 — Bring up the actual arm

Power the arm, plug in the U2D2, then from the phone panel:

1. **Connect** — pings all five motors, reads their positions and EEPROM limits.
2. Check the live feedback table shows all five with plausible ticks.
3. **Ask me before enabling torque.** Then enable it and move one joint slightly.

Go slowly. This is real hardware that can hit things, and the arm drops when
de-energised.

## Step 8 — Prove it survives a reboot

```bash
sudo reboot
```

Afterwards, with me touching nothing: service active, panel reachable, tunnel
up. Show me `journalctl -u omx@$USER -b --no-pager | tail -40`.

## Optional — seeing the desktop panel remotely (ask me first)

In `--headless` the Tk panel exists inside Xvfb but nothing displays it. If I
want it, install `x11vnc` against the Xvfb display so I can VNC in and see the
Tk panel and 3D viewer. Only if I say yes — it's another listening service to
secure.

---

## Ground rules

- **Never enable motor torque without asking me first.** It energises a
  physical arm that can move and drop.
- Keep the token in `/etc/omx.env` (mode 600). Never print it into a file that
  could be committed.
- If you change application code, commit to **`teleop`** with a clear message
  explaining what and why. Never push to `main`.
- Prefer editing `deploy/install_rpi.sh` over one-off commands, so the setup
  stays reproducible on the next Pi.
- **Tell me plainly when something doesn't work.** "MuJoCo has no aarch64 wheel,
  here are the two options" is far more useful to me than a confident summary
  with a broken step hidden inside it.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `PermissionError: /dev/ttyUSB0` | Not in `dialout`, or a shell that predates `usermod`. Log out and back in. |
| Service restarts repeatedly then stops | Hit the start limit — by design. `journalctl -u omx@$USER -n 50` for the real error. |
| `_tkinter.TclError: no display name` | Not running under `xvfb-run`. Check `ExecStart`. |
| Web panel returns 401 everywhere | Token mismatch. Re-read `/etc/omx.env`; restart after editing it. |
| Panel loads but Connect fails | Arm unpowered, U2D2 unplugged, or another program holds the port (only one at a time). |
| Recording feels coarse | `latency_timer` is still 16. See 3d. |
| `pip install mujoco` fails | See the aarch64 note in 3b. |
