# OpenManipulator-X Full Control

A full digital-twin control system for the ROBOTIS OpenManipulator-X: a MuJoCo
simulation that stays synchronized with the real robot arm over U2D2/DYNAMIXEL,
with joint sliders, keyboard/gamepad teleop, inverse kinematics (including a
click-to-pick workspace sphere), teach-by-demonstration recording, and a set
of hardware-safety features built from real debugging on physical hardware
(overload recovery, hardware-measured joint limits, gripper stop calibration).

This started as a fork of [Flux-15/openmanipulator-mujoco-sim](https://github.com/Flux-15/openmanipulator-mujoco-sim)
(MuJoCo model conversion + a basic gamepad-only simulation control script) and
grew into a standalone hardware+simulation control application — see
[Credits](#credits) below.

## Features

- **Live digital twin** — a MuJoCo simulation that mirrors (or drives) the
  real arm in real time, with hardware sync entirely opt-in: connect, then
  explicitly enable torque before anything physically moves.
- **Multiple control methods**, all converging on the same target state:
  - Per-joint sliders (5 joints: base, shoulder, elbow, wrist, gripper)
  - Keyboard jog — joint-space (`A/D W/S I/K J/L U/O`) or Cartesian
    end-effector jog (`R/F T/G Y/H`), works whether the control panel or the
    3D viewer window has focus
  - Gamepad teleop — proportional analog-stick control (joint or Cartesian
    mode), with a live raw-axis readout for calibrating to your specific pad
  - Inverse kinematics — type an X/Y/Z target, or **click a point directly
    on a translucent "workspace sphere" in the 3D view** and it fills in the
    coordinates for you
- **Mirror mode** — reverse the data flow so the *real* arm drives the
  simulated twin (e.g. to visualize hand-guided motion)
- **Teach by demonstration** — hand-guide the arm (torque off), record the
  motion from the arm's own encoder feedback, then play it back at the same
  speed it was taught, looped if you like. Save/load recordings as JSON.
- **Live motor feedback table** — present position of all 5 motors, updated
  at 30 Hz via a single sync-read bus transaction
- **Hardware safety, built from real incidents on this hardware:**
  - Automatic **Overload-error recovery** — polls each motor's hardware
    error register and auto-reboots + restores settings, instead of you
    having to reboot manually in DYNAMIXEL Wizard
  - **Hardware-measured joint limits** — reads each arm motor's own EEPROM
    min/max position limit on connect and clamps the digital twin to it,
    not just whatever the model file claims
  - **Gripper current-based position control** with a conservative current
    limit, so a bad position command stalls softly against an obstruction
    instead of tripping Overload
  - Torque never engages at a stale target — every "enable torque" path
    re-reads the arm's actual present position first, so energizing can't
    cause a lunge across whatever gap the sliders happened to be at
  - Thread-safe hardware access — all serial I/O is centralized behind a
    single lock, so the continuous feedback/command loop can't corrupt reads
    from concurrent UI actions (Diagnose, Calibrate, etc.)

## Demo control flow

```
DYNAMIXEL Wizard 2.0  --------\
  (setup / diagnostics)        \
                                 U2D2 (USB) <---> 5x XM430-W350 (IDs 11-15)
digital_twin_gui.py  ----------/
  (sliders / keyboard / gamepad / IK / record-playback)
       |
       v
  MuJoCo simulation (open_manipulator_x.xml)
```

Only one program can hold the serial port at a time — use the **Disconnect**
button in the GUI to free it up for DYNAMIXEL Wizard and back.

## Hardware

- ROBOTIS **OpenManipulator-X** (5x DYNAMIXEL XM430-W350)
- **U2D2** USB-to-DYNAMIXEL adapter
- Default config (from a DYNAMIXEL Wizard 2.0 scan): port `/dev/ttyUSB0`,
  baudrate `1,000,000` bps, protocol `2.0`, motor IDs `11` (base/joint1),
  `12` (shoulder/joint2), `13` (elbow/joint3), `14` (wrist/joint4),
  `15` (gripper). Edit the constants at the top of
  `Controller/digital_twin_gui.py` if your setup differs.
- The gripper's tick↔meters mapping (`GRIPPER_DEG_CLOSED` /
  `GRIPPER_DEG_OPEN`) is **measured, not calculated** — see
  [Gripper calibration](#gripper-calibration) below if you're setting this
  up on a different physical unit.

## System compatibility

| | |
|---|---|
| **OS** | Developed and tested on **Ubuntu 24.04 LTS** (Linux). Should work on other Linux distros with the same dependencies; not tested on Windows/macOS (the `/dev/ttyUSB0`-style serial path and `dialout` group permission model are Linux-specific — Windows would need a COM port path and different SDK setup). |
| **Python** | Tested with **Python 3.14**. Older versions (3.9–3.12) should also work and may have broader pre-built wheel availability for `mujoco`/`numpy` on some platforms. |
| **Display** | Needs a working X11/Wayland display — the MuJoCo viewer (GLFW/OpenGL) and the Tkinter control panel are both native GUI windows. Does not run headless without a virtual display (e.g. `Xvfb`). |
| **System packages** | `python3-tk` (Tkinter — not always preinstalled on minimal systems: `sudo apt install python3-tk`). OpenGL/Mesa drivers for the MuJoCo renderer (present by default on most desktop Ubuntu installs). |
| **Serial permissions** | Your user needs to be in the `dialout` group to access `/dev/ttyUSB0` without `sudo`: `sudo usermod -aG dialout $USER` (then log out/in). |
| **Python packages** | `mujoco`, `numpy`, `pygame`, `dynamixel-sdk` — see `requirements.txt`. |

## Installation

```bash
git clone https://github.com/Vedant28082005/openmanipulator-x-full-control.git
cd openmanipulator-x-full-control
pip install -r requirements.txt
```

## Quick start

1. **Generate the MuJoCo model** from the URDF (only needed once, or after
   editing the URDF):
   ```bash
   python urdf-xml.py
   ```
   This compiles `open_manipulator_x.urdf` into `open_manipulator_x.xml` and
   post-processes it to add the position actuators, damping, the
   `implicitfast` integrator, the IK end-effector site, and the workspace
   sphere — MuJoCo's URDF importer doesn't preserve any of those on its own,
   so this step matters (see comments in `urdf-xml.py` for why).

2. **Run the digital twin GUI:**
   ```bash
   python Controller/digital_twin_gui.py
   ```
   This opens two windows: a Tkinter control panel and a MuJoCo 3D viewer.
   It starts in **simulation-only mode** — no hardware is touched until you
   explicitly click **Connect**.

## Usage guide

### Simulation only (no hardware)

Drag the joint sliders, or jog with the keyboard (`A/D W/S I/K J/L U/O`,
works from either window). Tick **Cartesian jog mode** to switch the same
keys to end-effector X/Y/Z jog. Type coordinates into the IK panel, or
select-and-drag on the blue **workspace sphere** in the 3D view (MuJoCo's
built-in object-selection — check the viewer's own on-screen control legend,
usually ctrl+double-click-drag, since the exact gesture is version-dependent)
and hit **Solve & Move**.

### With the real arm

1. **Connect** — opens the serial port, pings all 5 motors, reads their
   current position (so the twin snaps to match reality instead of jumping),
   and reads each arm motor's hardware position limits.
2. **Enable Torque** — re-syncs the target to the arm's *actual* current
   position first, then energizes. The arm should not move at all the moment
   torque comes on.
3. Now the sliders/keyboard/gamepad/IK all drive the real arm too, at 20 Hz.
4. **TORQUE OFF (E-STOP)** is always live, regardless of what mode you're in.
5. **Disconnect** frees the serial port (e.g. to switch to DYNAMIXEL Wizard)
   without closing the app.

### Gamepad teleop

Click **Connect Gamepad**, then **Enable Gamepad**. Left stick = base/shoulder
(or X/Y in Cartesian mode), right stick = wrist/elbow (or Z), LB/RB =
gripper close/open. Stick deflection is proportional to speed. If the mapping
doesn't match your pad, the live raw axis/button readout in the panel will
show you which index to fix in the `GAMEPAD_AXIS_*` / `GAMEPAD_BTN_*`
constants at the top of `digital_twin_gui.py`.

### Mirror mode

Tick the checkbox to make the *real* arm drive the twin instead of the other
way around (useful while hand-guiding). Command output is suppressed while
it's on. Turning it off re-syncs the twin's target to reality before
resuming normal control, so you don't get a jump.

### Teach by demonstration (record & playback)

1. **Connect**, then **● Record** (offers to disable torque for you, with a
   confirmation — support the arm, it drops when de-energized).
2. Hand-guide the arm through the motion.
3. **■ Stop Recording**, then **▶ Play** to replay it (torque re-enables
   automatically, easing in from wherever the arm currently is rather than
   snapping to the recording's start).
4. **Save.../Load...** to keep recordings as JSON in `recordings/`. **Loop**
   for repeated playback.

Recording and playback each take exclusive control of the arm while active —
mirror mode, Calibrate, manual torque toggling, and the sliders are all
locked out and restored automatically when you stop.

### Gripper calibration

The default gripper open/close ticks in `GRIPPER_DEG_CLOSED` /
`GRIPPER_DEG_OPEN` were measured on the specific unit this was built against
(via DYNAMIXEL Wizard's live position readout while manually opening/closing
the gripper). On a different physical gripper, either remeasure those two
numbers the same way, or use the **Calibrate Gripper** button, which
gently drives the gripper into each mechanical stop (safe: it's in
current-based position control, so it stalls against the current limit
instead of over-torquing) and measures the real travel automatically.

### Diagnostics

**Diagnose Gripper** reads back the gripper's actual live control-table
state (operating mode, torque enable, current limit, present current,
hardware error) — useful for root-causing "it's not moving" instead of
guessing.

## Project structure

```
open_manipulator_x.urdf       Robot description (source of truth for geometry)
urdf-xml.py                   Converts the URDF to MuJoCo XML + post-processes
                               it (actuators, damping/integrator, IK site,
                               workspace sphere - see comments for why each
                               step is needed)
open_manipulator_x.xml        Generated MuJoCo model (regenerate with the
                               script above; don't hand-edit)
Controller/
  digital_twin_gui.py          The main application (this project's core)
  controller_cntroll.py        Original gamepad-only sim control script
  controller_test.py           Gamepad axis/button test utility
meshes/, STL/                  Robot visual/collision geometry
recordings/                    Saved teach-by-demonstration recordings (gitignored)
```

## Roadmap ideas

- Cartesian jog is already implemented; recorded-motion editing (trim/scale/splice)
  and a waypoint-macro library are natural next steps
- Live current/load graphing per motor
- Camera + basic vision integration
- Self-collision warnings (contacts are currently disabled entirely in sim)

## Credits

- Base MuJoCo model conversion and initial gamepad simulation control:
  [Flux-15/openmanipulator-mujoco-sim](https://github.com/Flux-15/openmanipulator-mujoco-sim)
- Digital twin control system, hardware integration, safety engineering,
  IK/workspace sphere, teach-by-demonstration, and gamepad teleop built on
  top of that base.
- Robot hardware/firmware reference: [ROBOTIS OpenManipulator-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/)
  and the official [open_manipulator_libs](https://github.com/ROBOTIS-GIT/open_manipulator)
  source (used to find the real gripper drive scheme and control-table values).

## License

MIT — see [LICENSE](LICENSE). Note: the robot description, mesh files, and
the two original controller scripts were sourced from the Flux-15 repo above,
which does not carry its own license file — see the note at the bottom of
[LICENSE](LICENSE) if you plan to redistribute this further.

## Safety

This project drives real motorized hardware. Read through the safety
behaviors described above before connecting to a physical arm, keep the
E-STOP accessible, and don't leave torque engaged on an unsupported/raised
arm unattended.
