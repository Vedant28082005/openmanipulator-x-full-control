"""
Digital-twin control GUI for OpenManipulator-X.

Sliders drive the MuJoCo simulation in real time. Hardware sync to the
real robot (via U2D2) is optional and off by default - you must
explicitly Connect and then Enable Torque before the real motors move.

Hardware config (from DYNAMIXEL Wizard 2.0 scan):
    Port:      /dev/ttyUSB0
    Baudrate:  1,000,000 bps
    Protocol:  2.0
    Motor IDs: 11 (joint1/base), 12 (joint2/shoulder), 13 (joint3/elbow),
               14 (joint4/wrist), 15 (gripper)
"""
import json
import math
import os
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import mujoco
import mujoco.viewer
import numpy as np
import pygame

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import web_app
from dynamixel_sdk import (PortHandler, PacketHandler, GroupSyncWrite, GroupSyncRead,
                           DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD)

# ==========================================
# HARDWARE CONFIG
# ==========================================
DEVICENAME = "/dev/ttyUSB0"
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

# joint1, joint2, joint3, joint4, gripper (single motor drives both fingers)
DXL_IDS = [11, 12, 13, 14, 15]

ADDR_OPERATING_MODE = 11
ADDR_CURRENT_LIMIT = 38
ADDR_MIN_POSITION_LIMIT = 48
ADDR_MAX_POSITION_LIMIT = 52
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_POSITION_D_GAIN = 80
ADDR_POSITION_I_GAIN = 82
ADDR_POSITION_P_GAIN = 84
ADDR_GOAL_CURRENT = 102
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132
LEN_GOAL_POSITION = 4

OPERATING_MODE_CURRENT_BASED_POSITION = 5

TICKS_PER_REV = 4096
CENTER_TICK = 2048
GRIPPER_ID = 15

# Per DYNAMIXEL's own docs and the ROBOTIS community forum, the
# Profile Velocity/Acceleration combination is what determines vibration
# and noise during motion - and if the profile's speed cap is LOWER than
# the rate the goal position is actually being pushed forward at (which is
# what continuous jogging does), the servo can never catch up to a
# receding goal: it keeps re-triggering its acceleration ramp on every
# 20 Hz update instead of settling into one smooth trapezoid. That
# mismatch was the direct cause of the "shaky/vibrating" feel during jogging.
#   old PROFILE_VELOCITY=40 raw -> 0.96 rad/s cap, but jog commands up to
#   1.5 rad/s (JOINT_SPEED / GAMEPAD_JOINT_RATE) - the servo was literally
#   incapable of keeping pace with its own goal.
# 150 raw -> ~3.6 rad/s cap, ~2.4x headroom over the fastest jog rate, so
# the servo is never the bottleneck. Acceleration scaled up by the same
# ratio so the ramp itself doesn't become the new mismatch.
#   40  raw = 0.96 rad/s = 0.64x the max jog rate -> servo LAGS behind its
#                          own goal and re-accelerates constantly (original bug)
#   70  raw = 1.68 rad/s = 1.12x -> still travelling when the next goal
#                          arrives, so motion is continuous  <-- what we want
#   150 raw = 3.60 rad/s = 2.40x -> finishes each 33 ms step early and sits
#                          idle waiting, i.e. stop/start micro-stutter
# ROBOTIS: "when a DYNAMIXEL receives an updated Goal Position while it is
# moving toward the previous one, velocity is adjusted smoothly" - that only
# holds while it is STILL MOVING, hence matching the cap to the jog rate.
PROFILE_VELOCITY = 70
PROFILE_ACCELERATION = 40

# Position-loop PID gains for the ARM joints (11-14). The XM430 ships with
# P=800, I=0, and - critically - **D=0**, i.e. NO damping in the position
# loop at all. ROBOTIS's own open_manipulator firmware never overrides
# these, so this arm has been running undamped. Per ROBOTIS's PID tuning
# guide (robotis.us/pid-tuning-for-dynamixel), D gain "adds damping by
# reacting to how fast the error changes... reduces overshoot and improves
# settling near the target" - with D=0 a joint carrying real load hunts
# around its goal instead of settling, which is why IDs 11 (base yaw
# carrying the whole arm) and 12 (shoulder holding it against gravity)
# vibrate at rest while the lighter elbow/wrist don't.
#
# TUNING, if the defaults below aren't right for your arm:
#   still vibrating  -> raise POSITION_D_GAIN (try +400 at a time)
#   buzzing / harsh  -> too much D, lower it; D amplifies sensor noise
#   sagging or soft  -> raise POSITION_P_GAIN back toward/above 800
# P is left at the factory 800 deliberately: lowering it is the other
# documented way to kill oscillation, but it also weakens the arm's hold
# against gravity, so damping first is the safer lever.
POSITION_P_GAIN = 800   # factory default
POSITION_I_GAIN = 0     # factory default
POSITION_D_GAIN = 1000  # factory default is 0 - this is the fix

# Gripper: these are ROBOTIS's own official values, taken directly from
# open_manipulator_libs (github.com/ROBOTIS-GIT/open_manipulator, noetic
# branch, open_manipulator_libs/src/open_manipulator.cpp addTool("gripper", ...)
# and src/dynamixel.cpp GripperDynamixel::setOperatingMode). The gripper is
# NOT a separately-calibrated joint - it's driven by the exact same
# radian->tick formula as the arm joints, just through a linear
# meter<->radian coefficient:
#     joint_radian = gripper_value_m / GRIPPER_COEFFICIENT
# Full official travel is +-0.010 m, i.e. +-(0.010/0.015) =~ +-0.667 rad
# =~ +-435 ticks.
#
# The SCALE above is official, but the ORIGIN is not portable: ROBOTIS's
# formula assumes tick 2048 == gripper zero, which only holds if the servo
# horn was assembled at that index. It isn't on this robot, so instead of
# any calculated origin, the endpoints below are DIRECTLY MEASURED: jogged
# to each mechanical stop in DYNAMIXEL Wizard and read off its position
# display. This is ground truth and overrides both the coefficient-based
# guess and the auto-calibration sweep's estimate.
GRIPPER_COEFFICIENT = -0.015   # meter per radian (official ROBOTIS scale, informational only)
GRIPPER_GOAL_CURRENT = 200     # official ROBOTIS value (dynamixel.cpp: `const uint32_t current = 200;`)
GRIPPER_PROFILE_VELOCITY = 200      # official ROBOTIS value
GRIPPER_PROFILE_ACCELERATION = 20   # official ROBOTIS value

GRIPPER_DEG_CLOSED = 125.2   # measured in DYNAMIXEL Wizard
GRIPPER_DEG_OPEN = 270.4     # measured in DYNAMIXEL Wizard
GRIPPER_TICK_CLOSED = round(GRIPPER_DEG_CLOSED * TICKS_PER_REV / 360)
GRIPPER_TICK_OPEN = round(GRIPPER_DEG_OPEN * TICKS_PER_REV / 360)

# Auto-calibration sweep: drive gently into each mechanical stop to measure
# the gripper's true travel, instead of trusting a calculated range.
GRIPPER_CALIB_CURRENT = 100     # gentler than normal for the sweep - soft stall
GRIPPER_CALIB_SWEEP = 900       # ticks to command past any plausible stop
GRIPPER_CALIB_TIMEOUT = 4.0     # seconds per direction
GRIPPER_CALIB_STILL_TICKS = 2   # movement below this counts as "not moving"
GRIPPER_CALIB_STILL_SAMPLES = 6 # consecutive still samples (~0.3 s) = stalled
GRIPPER_CALIB_MIN_SPAN = 60     # reject implausibly small measured travel
GRIPPER_CALIB_MARGIN = 15       # back off this many ticks from each hard stop

# Record / playback (teach by demonstration).
RECORD_MIN_DELTA = 1e-4     # skip frames where nothing meaningfully moved
HW_FEEDBACK_RATE = 30.0     # Hz for the normal serial feedback/goal-write loop
RECORD_RATE = 60.0          # Hz while recording - 2x, for finer-grained capture.
                            # Recording samples the arm's encoders on that same
                            # loop, so the loop rate IS the capture rate. It's
                            # affordable because recording skips the goal-write
                            # half of the cycle (the arm drives the twin, not the
                            # reverse), so the extra cycles cost only sync-reads.
PLAYBACK_APPROACH_TIME = 2.5  # seconds to ease from the current pose into frame 0
PLAYBACK_APPROACH_RATE = 50.0 # Hz for that easing ramp
RECORDINGS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "recordings")

# Home position - the arm's resting pose. [joint1, joint2, joint3, joint4,
# gripper_left, gripper_right] matching self.target's layout (gripper_right
# mirrors gripper_left, same as everywhere else in this file).
HOME_POSITION = np.array([1.589, 0.330, -0.107, 1.663, 0.0066, 0.0066])
HOME_APPROACH_TIME = 2.5    # seconds to ease into home, same feel as playback's approach ramp
HOME_APPROACH_RATE = 50.0   # Hz for that easing ramp

GRIPPER_ACTION_SETTLE = 1.0  # seconds to hold after a scripted gripper close/open before moving on

# Runtime options, set from the command line in __main__. Defaults keep the
# desktop behaviour exactly as it was.
class Options:
    viewer = True       # show the MuJoCo 3D window
    panel = True        # show the Tk control panel
    web = True          # serve the mobile web panel
    port = None         # None -> web_app.WEB_PORT
    bind = None         # None -> web_app.WEB_BIND


OPTS = Options()


# Control-panel window sizing. The panel is taller than a 1080p screen once
# every section is expanded, so it scrolls rather than clipping.
SCREEN_MARGIN = 80          # leave room for the taskbar/titlebar
SCROLLBAR_ALLOWANCE = 20    # width the vertical scrollbar takes from content
MIN_WINDOW_HEIGHT = 400     # still usable on a very short display

XML_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "open_manipulator_x.xml")

JOINT_SPECS = [
    # (label, mujoco ctrl index, dxl id, slider min, slider max, kind)
    ("Joint1 - Base",     0, 11, -3.14159, 3.14159, "rad"),
    ("Joint2 - Shoulder", 1, 12, -1.5,     1.5,      "rad"),
    ("Joint3 - Elbow",    2, 13, -1.5,     1.4,      "rad"),
    ("Joint4 - Wrist",    3, 14, -1.7,     1.97,     "rad"),
    ("Gripper",           4, 15, -0.010,   0.010,    "gripper"),  # official ROBOTIS limits
]

# Keyboard jog controls: key -> (ctrl_idx, direction)
KEY_BINDINGS = {
    "a": (0, +1), "d": (0, -1),   # base
    "w": (1, +1), "s": (1, -1),   # shoulder
    "i": (2, +1), "k": (2, -1),   # elbow
    "j": (3, -1), "l": (3, +1),   # wrist
    "u": (4, -1), "o": (4, +1),   # gripper close/open
}
JOINT_SPEED = {0: 1.5, 1: 1.5, 2: 1.5, 3: 1.5, 4: 0.02}  # rad/s (gripper: m/s), used while the Tk window has focus
JOG_STEP = {0: 0.04, 1: 0.04, 2: 0.04, 3: 0.04, 4: 0.0015}  # per keypress, used when the MuJoCo viewer window has focus

# Cartesian jog: key -> (xyz axis index, direction). Disjoint from
# KEY_BINDINGS above so both key sets can be bound at once without
# ambiguity - which set is live depends on cartesian_jog_enabled.
CARTESIAN_KEY_BINDINGS = {
    "r": (0, +1), "f": (0, -1),   # X
    "t": (1, +1), "g": (1, -1),   # Y
    "y": (2, +1), "h": (2, -1),   # Z
}
CARTESIAN_JOG_RATE = 0.08    # m/s, used while the Tk window has focus
CARTESIAN_JOG_STEP = 0.003   # meters per keypress, used when the MuJoCo viewer window has focus
CARTESIAN_JOG_DAMPING = 0.05 # same damping as solve_ik, for the same reason

KEYBOARD_HELP = (
    "Joint jog:  A/D Base   W/S Shoulder   I/K Elbow   J/L Wrist   U/O Gripper\n"
    "Cartesian jog (tick the box below):  R/F  X    T/G  Y    Y/H  Z\n"
    "Works whether the control panel or the 3D viewer window has focus."
)

# Gamepad teleop. Axis/button indices below are the standard Xbox-style
# SDL2 layout on Linux via pygame - the most common mapping, but pad
# firmware varies. The GUI shows a live raw axis/button readout so you can
# verify against your specific pad and change these constants if it's off,
# the same way the repo's own controller_test.py was meant to be used.
GAMEPAD_AXIS_LEFT_X = 0
GAMEPAD_AXIS_LEFT_Y = 1
GAMEPAD_AXIS_RIGHT_X = 3
GAMEPAD_AXIS_RIGHT_Y = 4
GAMEPAD_BTN_GRIPPER_CLOSE = 4   # LB
GAMEPAD_BTN_GRIPPER_OPEN = 5    # RB
GAMEPAD_BTN_HOME = 3            # Y - go to home position
GAMEPAD_DEADZONE = 0.15
GAMEPAD_JOINT_RATE = 1.5      # rad/s at full stick deflection
GAMEPAD_CARTESIAN_RATE = 0.08 # m/s at full stick deflection
GAMEPAD_GRIPPER_RATE = 0.02   # m/s while a gripper button is held

GAMEPAD_HELP = (
    "Left stick: joint mode -> Base/Shoulder   cartesian mode -> X/Y\n"
    "Right stick: joint mode -> Wrist/Elbow    cartesian mode -> Z (up/down)\n"
    "LB / RB: gripper close/open   -   Proportional to how far you push (analog).\n"
    "Y: return to home position (same 2.5s eased move as the Home button).\n"
    "Uses the same Cartesian-jog-mode checkbox as the keyboard, above."
)


def apply_deadzone(value: float, deadzone: float) -> float:
    """Zeros out stick drift near center and rescales the rest to still
    reach +-1 at full deflection, rather than leaving a dead gap."""
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def gamepad_axis(axes, idx: int) -> float:
    return apply_deadzone(axes[idx], GAMEPAD_DEADZONE) if idx < len(axes) else 0.0


def rad_to_tick(rad: float) -> int:
    tick = round(CENTER_TICK + rad * (TICKS_PER_REV / (2 * math.pi)))
    return max(0, min(4095, tick))


def tick_to_rad(tick: int) -> float:
    return (tick - CENTER_TICK) * (2 * math.pi / TICKS_PER_REV)


def gripper_to_tick(pos_m: float, lo_m: float, hi_m: float, tick_at_lo: int, tick_at_hi: int) -> int:
    """Linear map from gripper opening (m) onto the measured tick endpoints."""
    span = hi_m - lo_m
    frac = 0.0 if span == 0 else (pos_m - lo_m) / span
    frac = max(0.0, min(1.0, frac))
    return max(0, min(4095, round(tick_at_lo + frac * (tick_at_hi - tick_at_lo))))


def tick_to_gripper(tick: int, lo_m: float, hi_m: float, tick_at_lo: int, tick_at_hi: int) -> float:
    span_ticks = tick_at_hi - tick_at_lo
    frac = 0.0 if span_ticks == 0 else (tick - tick_at_lo) / span_ticks
    frac = max(0.0, min(1.0, frac))
    return lo_m + frac * (hi_m - lo_m)


class HardwareLink:
    """Wraps the U2D2 connection. All hardware calls are best-effort and
    never raise into the GUI thread - failures just get reported as text."""

    def __init__(self, status_cb):
        self.port = None
        self.packet = None
        self.connected = False
        self.torque_on = False
        self.status_cb = status_cb
        self.sync_reader = None
        # Last goal tick actually sent per motor, so identical goals aren't
        # re-sent every tick - see sync_write_ticks for why that matters.
        self.last_sent_ticks = {}
        # A pyserial port is not safe for concurrent access, but this class
        # is called from several threads at once: the sim loop reads/writes
        # it continuously (feedback @30Hz, goals @30Hz, error poll @1Hz),
        # while Connect/Disconnect, Diagnose, Calibrate, and mirror-off all
        # call in directly from button handlers. Every method below that
        # touches self.port/self.packet holds this lock for its duration,
        # so two callers can never interleave writes/reads on the wire -
        # which was corrupting reads and dropping recorded frames.
        self.io_lock = threading.Lock()

    def _write_position_gains(self, dxl_id):
        """Arm joints only - the gripper runs current-based position control
        with ROBOTIS's own tuning, so its gains are left alone. Caller must
        already hold io_lock. Gains live in RAM, so this has to be re-applied
        after any reboot (see recover())."""
        self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_POSITION_D_GAIN, POSITION_D_GAIN)
        self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_POSITION_I_GAIN, POSITION_I_GAIN)
        self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_POSITION_P_GAIN, POSITION_P_GAIN)

    def apply_tuning(self, dxl_ids, p_gain, i_gain, d_gain, prof_vel, prof_acc):
        """Live-writes position PID gains AND profile velocity/acceleration.
        All of these are RAM registers, so they take effect immediately with
        torque still on - which is what makes interactive tuning possible."""
        with self.io_lock:
            if not self.connected:
                return False
            for dxl_id in dxl_ids:
                self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_POSITION_D_GAIN, int(d_gain))
                self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_POSITION_I_GAIN, int(i_gain))
                self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_POSITION_P_GAIN, int(p_gain))
                self.packet.write4ByteTxRx(self.port, dxl_id, ADDR_PROFILE_ACCELERATION, int(prof_acc))
                self.packet.write4ByteTxRx(self.port, dxl_id, ADDR_PROFILE_VELOCITY, int(prof_vel))
            return True

    def read_position_gains(self, dxl_id):
        """Returns (P, I, D) as the motor actually has them, or None."""
        with self.io_lock:
            if not self.connected:
                return None
            d, r1, _ = self.packet.read2ByteTxRx(self.port, dxl_id, ADDR_POSITION_D_GAIN)
            i, r2, _ = self.packet.read2ByteTxRx(self.port, dxl_id, ADDR_POSITION_I_GAIN)
            p, r3, _ = self.packet.read2ByteTxRx(self.port, dxl_id, ADDR_POSITION_P_GAIN)
            if r1 != 0 or r2 != 0 or r3 != 0:
                return None
            return p, i, d

    def connect(self):
        with self.io_lock:
            return self._connect_locked()

    def _connect_locked(self):
        self.port = PortHandler(DEVICENAME)
        self.packet = PacketHandler(PROTOCOL_VERSION)

        if not self.port.openPort():
            self.status_cb(f"Failed to open {DEVICENAME}")
            return False
        if not self.port.setBaudRate(BAUDRATE):
            self.status_cb("Failed to set baudrate")
            return False

        missing = []
        for dxl_id in DXL_IDS:
            _, comm_result, _ = self.packet.ping(self.port, dxl_id)
            if comm_result != 0:
                missing.append(dxl_id)
        if missing:
            self.status_cb(f"Ping failed for IDs {missing} - check power/wiring")
            self.port.closePort()
            self.connected = False
            return False

        for dxl_id in DXL_IDS:
            self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
            if dxl_id == GRIPPER_ID:
                continue  # gripper gets its own profile/gain setup below
            self.packet.write4ByteTxRx(self.port, dxl_id, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY)
            self.packet.write4ByteTxRx(self.port, dxl_id, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION)
            self._write_position_gains(dxl_id)

        # Gripper: official ROBOTIS current-based position control setup
        # (see GRIPPER_COEFFICIENT comment above for the source). Checked
        # for errors, unlike the writes above, because a silently-failed
        # mode switch here means the gripper won't move at all.
        cr, err = self.packet.write1ByteTxRx(self.port, GRIPPER_ID, ADDR_OPERATING_MODE, OPERATING_MODE_CURRENT_BASED_POSITION)
        if cr != 0 or err != 0:
            self.status_cb(f"Gripper operating-mode write failed (comm={cr}, err={err}) - gripper will not move")
        self.packet.write4ByteTxRx(self.port, GRIPPER_ID, ADDR_PROFILE_ACCELERATION, GRIPPER_PROFILE_ACCELERATION)
        self.packet.write4ByteTxRx(self.port, GRIPPER_ID, ADDR_PROFILE_VELOCITY, GRIPPER_PROFILE_VELOCITY)
        cr, err = self.packet.write2ByteTxRx(self.port, GRIPPER_ID, ADDR_GOAL_CURRENT, GRIPPER_GOAL_CURRENT)
        if cr != 0 or err != 0:
            self.status_cb(f"Gripper goal-current write failed (comm={cr}, err={err}) - gripper will not move")

        # One sync-read handler reused for the live feedback loop: reads all
        # 5 present positions in a single bus transaction instead of 5
        # separate round trips, which is what makes ~50 Hz mirroring viable.
        self.sync_reader = GroupSyncRead(self.port, self.packet, ADDR_PRESENT_POSITION, LEN_GOAL_POSITION)
        for dxl_id in DXL_IDS:
            self.sync_reader.addParam(dxl_id)

        self.connected = True
        self.status_cb(f"Connected on {DEVICENAME} @ {BAUDRATE} bps, all {len(DXL_IDS)} motors responded")
        return True

    def read_all_ticks_fast(self):
        """Present position of every motor in one bus transaction.
        Returns {id: tick}, or None if the read failed. Missing/!available
        motors are skipped rather than failing the whole batch, so one
        flaky servo can't blank the entire feedback display."""
        with self.io_lock:
            if not self.connected or self.sync_reader is None:
                return None
            if self.sync_reader.txRxPacket() != 0:
                return None
            result = {}
            for dxl_id in DXL_IDS:
                if self.sync_reader.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_GOAL_POSITION):
                    result[dxl_id] = self.sync_reader.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_GOAL_POSITION) & 0xFFFFFFFF
            return result or None

    def read_present_ticks(self):
        """Returns {id: tick} for all motors, or None on failure."""
        with self.io_lock:
            if not self.connected:
                return None
            result = {}
            for dxl_id in DXL_IDS:
                tick, comm_result, _ = self.packet.read4ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
                if comm_result != 0:
                    self.status_cb(f"Read failed for ID {dxl_id}")
                    return None
                result[dxl_id] = tick & 0xFFFFFFFF
            return result

    def diagnose_gripper(self):
        """Reads back the gripper's actual live state so a 'not moving'
        report can be root-caused instead of guessed at again: which
        operating mode it's really in, whether torque is really on,
        what current limit it's allowed vs actually drawing, and any
        hardware error - each is a distinct reason it could refuse to move."""
        with self.io_lock:
            if not self.connected:
                return "Not connected"
            fields = [
                ("Operating Mode", ADDR_OPERATING_MODE, 1),
                ("Torque Enable", ADDR_TORQUE_ENABLE, 1),
                ("Min Pos Limit", ADDR_MIN_POSITION_LIMIT, 4),
                ("Max Pos Limit", ADDR_MAX_POSITION_LIMIT, 4),
                ("Current Limit (EEPROM)", ADDR_CURRENT_LIMIT, 2),
                ("Goal Current", ADDR_GOAL_CURRENT, 2),
                ("Present Current", ADDR_PRESENT_CURRENT, 2),
                ("Hardware Error", ADDR_HARDWARE_ERROR_STATUS, 1),
            ]
            parts = []
            for name, addr, size in fields:
                reader = {1: self.packet.read1ByteTxRx, 2: self.packet.read2ByteTxRx, 4: self.packet.read4ByteTxRx}[size]
                val, comm_result, err = reader(self.port, GRIPPER_ID, addr)
                if comm_result != 0:
                    parts.append(f"{name}=<read failed>")
                else:
                    if size == 2 and val > 0x7FFF:
                        val -= 0x10000  # goal/present current are signed
                    parts.append(f"{name}={val}")
            pos, cr, _ = self.packet.read4ByteTxRx(self.port, GRIPPER_ID, ADDR_PRESENT_POSITION)
            if cr == 0:
                parts.append(f"Present Position(tick)={pos & 0xFFFFFFFF}")
            return "Gripper (ID 15): " + ", ".join(parts)

    def set_torque(self, on: bool):
        with self.io_lock:
            if not self.connected:
                return
            for dxl_id in DXL_IDS:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1 if on else 0)
            self.torque_on = on
            # While de-energised the arm can be moved by hand, so a cached
            # goal no longer reflects reality - drop it so the first write
            # after re-enabling torque always goes through.
            self.last_sent_ticks.clear()
        self.status_cb("Torque ENABLED - real motors will follow sliders" if on else "Torque disabled")

    def find_gripper_stops(self, progress_cb=None):
        """Measures the gripper's REAL mechanical travel by driving gently
        into each end stop and recording where it stalls.

        The spec figure (+-435 ticks from the ROBOTIS coefficient) is the
        nominal design travel; the actual usable range on an assembled arm
        depends on where the servo horn was indexed and how the linkage was
        built, which is why a calculated range kept coming up short. This
        measures it instead.

        Safe because the gripper is in Current-based Position Control with a
        reduced Goal Current for the sweep: meeting a hard stop just stalls
        against the current limit rather than driving at full PWM (which is
        what trips Overload). Returns (tick_low, tick_high) or None.

        Holds io_lock for the whole sweep (up to ~2x GRIPPER_CALIB_TIMEOUT) -
        deliberately, since interleaving a feedback read mid-sweep would
        just report the same in-progress position and adds no information."""
        with self.io_lock:
            if not self.connected:
                return None

            saved_current = GRIPPER_GOAL_CURRENT
            self.packet.write2ByteTxRx(self.port, GRIPPER_ID, ADDR_GOAL_CURRENT, GRIPPER_CALIB_CURRENT)
            self.packet.write1ByteTxRx(self.port, GRIPPER_ID, ADDR_TORQUE_ENABLE, 1)

            found = []
            for direction in (+1, -1):
                start, cr, _ = self.packet.read4ByteTxRx(self.port, GRIPPER_ID, ADDR_PRESENT_POSITION)
                if cr != 0:
                    self.packet.write2ByteTxRx(self.port, GRIPPER_ID, ADDR_GOAL_CURRENT, saved_current)
                    return None
                start &= 0xFFFFFFFF
                goal = max(0, min(4095, start + direction * GRIPPER_CALIB_SWEEP))
                self.packet.write4ByteTxRx(self.port, GRIPPER_ID, ADDR_GOAL_POSITION, goal)

                last = start
                still = 0
                stalled_at = start
                deadline = time.time() + GRIPPER_CALIB_TIMEOUT
                while time.time() < deadline:
                    time.sleep(0.05)
                    pos, cr, _ = self.packet.read4ByteTxRx(self.port, GRIPPER_ID, ADDR_PRESENT_POSITION)
                    if cr != 0:
                        continue
                    pos &= 0xFFFFFFFF
                    stalled_at = pos
                    if abs(pos - last) <= GRIPPER_CALIB_STILL_TICKS:
                        still += 1
                        if still >= GRIPPER_CALIB_STILL_SAMPLES:
                            break   # stopped moving = we're against the stop
                    else:
                        still = 0
                    last = pos
                found.append(stalled_at)
                if progress_cb:
                    progress_cb(f"Gripper stop {'A' if direction > 0 else 'B'} found at tick {stalled_at}")

            self.packet.write2ByteTxRx(self.port, GRIPPER_ID, ADDR_GOAL_CURRENT, saved_current)
            lo, hi = min(found), max(found)
            if hi - lo < GRIPPER_CALIB_MIN_SPAN:
                return None   # implausibly small travel - something blocked it
            return lo, hi

    def verify_torque(self, expected: bool) -> bool:
        """Reads Torque Enable back rather than trusting the write - a
        silently-refused torque enable looks exactly like 'motor is dead'."""
        with self.io_lock:
            if not self.connected:
                return False
            want = 1 if expected else 0
            for dxl_id in DXL_IDS:
                val, comm_result, _ = self.packet.read1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE)
                if comm_result != 0 or val != want:
                    return False
            return True

    def sync_write_ticks(self, id_to_tick: dict):
        """Writes goal positions, skipping any motor whose goal hasn't
        actually changed since the last write.

        This matters more than it looks: the caller runs at a fixed rate
        regardless of whether anything moved, so holding still used to
        re-send the *identical* Goal Position ~30x/second. Every Goal
        Position write restarts the servo's profile trajectory generator,
        so a motor at rest was being told to re-plan a move to where it
        already was, over and over - which shows up as buzzing/vibration on
        whichever joints carry enough load to react (11 and 12 here).
        Skipping unchanged goals lets a stationary joint actually settle."""
        with self.io_lock:
            if not self.connected or not self.torque_on:
                return
            changed = {i: t for i, t in id_to_tick.items() if self.last_sent_ticks.get(i) != t}
            if not changed:
                return
            writer = GroupSyncWrite(self.port, self.packet, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
            for dxl_id, tick in changed.items():
                data = [DXL_LOBYTE(DXL_LOWORD(tick)), DXL_HIBYTE(DXL_LOWORD(tick)),
                        DXL_LOBYTE(DXL_HIWORD(tick)), DXL_HIBYTE(DXL_HIWORD(tick))]
                writer.addParam(dxl_id, data)
            if writer.txPacket() == 0:
                self.last_sent_ticks.update(changed)
            writer.clearParam()

    def read_position_limits(self, dxl_id):
        """Returns (min_tick, max_tick) from the motor's own EEPROM travel
        limits, or None on failure. This is the arm's real mechanical range
        as configured on the actual hardware, independent of whatever the
        URDF/XML happens to claim."""
        with self.io_lock:
            if not self.connected:
                return None
            lo, r1, _ = self.packet.read4ByteTxRx(self.port, dxl_id, ADDR_MIN_POSITION_LIMIT)
            hi, r2, _ = self.packet.read4ByteTxRx(self.port, dxl_id, ADDR_MAX_POSITION_LIMIT)
            if r1 != 0 or r2 != 0:
                self.status_cb(f"Failed to read position limits for ID {dxl_id}")
                return None
            return lo & 0xFFFFFFFF, hi & 0xFFFFFFFF

    def check_hardware_errors(self):
        """Returns {id: error_byte} for any motor currently reporting a
        hardware error (bit5 = Overload, bit3 = Overheating, etc)."""
        with self.io_lock:
            if not self.connected:
                return {}
            errors = {}
            for dxl_id in DXL_IDS:
                err, comm_result, _ = self.packet.read1ByteTxRx(self.port, dxl_id, ADDR_HARDWARE_ERROR_STATUS)
                if comm_result == 0 and err != 0:
                    errors[dxl_id] = err
            return errors

    def recover(self, dxl_id):
        """Reboots a motor that tripped a hardware error (e.g. Overload)
        and restores its profile/torque settings - this is exactly what
        DYNAMIXEL Wizard's "Reboot" button does, automated."""
        with self.io_lock:
            self.packet.reboot(self.port, dxl_id)
            time.sleep(0.3)
            if dxl_id == GRIPPER_ID:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_OPERATING_MODE, OPERATING_MODE_CURRENT_BASED_POSITION)
                self.packet.write4ByteTxRx(self.port, dxl_id, ADDR_PROFILE_ACCELERATION, GRIPPER_PROFILE_ACCELERATION)
                self.packet.write4ByteTxRx(self.port, dxl_id, ADDR_PROFILE_VELOCITY, GRIPPER_PROFILE_VELOCITY)
                self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_GOAL_CURRENT, GRIPPER_GOAL_CURRENT)
            else:
                self.packet.write4ByteTxRx(self.port, dxl_id, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY)
                self.packet.write4ByteTxRx(self.port, dxl_id, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION)
                # Position gains are RAM too - a reboot resets D back to 0,
                # which would silently bring the vibration straight back.
                self._write_position_gains(dxl_id)
            # The motor lost its goal position across the reboot, so the
            # deadband cache below is stale - force the next write through.
            self.last_sent_ticks.pop(dxl_id, None)
            if self.torque_on:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)

    def disconnect(self):
        with self.io_lock:
            if self.connected:
                if self.port is not None:
                    for dxl_id in DXL_IDS:
                        self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
                    self.port.closePort()
                self.torque_on = False
            self.connected = False
            self.last_sent_ticks.clear()
        self.status_cb("Torque disabled")


def decode_hw_error(err_byte: int) -> str:
    bits = {
        0: "Input Voltage", 2: "Overheating", 3: "Motor Encoder",
        4: "Electrical Shock", 5: "Overload",
    }
    names = [name for bit, name in bits.items() if err_byte & (1 << bit)]
    return "+".join(names) if names else f"0x{err_byte:02x}"


class _NullViewer:
    """Stand-in for the MuJoCo passive viewer when running without a 3D window.

    A Raspberry Pi has no GPU driver MuJoCo can use well - it falls back to
    software rendering, which burns most of a core to draw a window nobody is
    looking at. The sim loop is written against the viewer's interface, so
    this satisfies it without opening anything."""

    class _Perturb:
        select = -1     # never matches workspace_body_id, so picking is inert
        active = 0

    def __init__(self, stop_event):
        self._stop = stop_event
        self.perturb = self._Perturb()

    def is_running(self):
        return not self._stop.is_set()

    def sync(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


class DigitalTwinApp:
    def __init__(self, root):
        self.root = root
        root.title("OpenManipulator-X Digital Twin")

        self.model = mujoco.MjModel.from_xml_path(XML_PATH)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        self.target = np.copy(self.data.qpos[:6])
        self.joint_limits = {ctrl_idx: (lo, hi) for _, ctrl_idx, _, lo, hi, _ in JOINT_SPECS}
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.keys_held = set()
        self.cartesian_jog_enabled = False
        self.gamepad = None
        self.gamepad_enabled = False
        self.gamepad_axes_snapshot = []
        self.gamepad_buttons_snapshot = []
        self.gamepad_home_was_pressed = False
        # Gripper tick endpoints for slider min/max: directly measured in
        # DYNAMIXEL Wizard (closed=125.2deg, open=270.4deg), not calculated.
        # "Calibrate Gripper" can still re-measure and override these.
        self.gripper_tick_at_lo, self.gripper_tick_at_hi = GRIPPER_TICK_CLOSED, GRIPPER_TICK_OPEN
        self.gripper_calibrated = True
        self.calibrating = False
        # Live feedback from the physical arm, and which way data flows:
        # mirror OFF = twin commands robot; mirror ON = robot drives twin.
        self.present_ticks = {}
        self.mirror_mode = False
        # Teach-by-demonstration: frames are (elapsed_seconds, 6-vector of
        # joint targets), captured from the real arm's own feedback.
        self.recording = False
        self.record_started_at = 0.0
        self.frames = []
        self.playing = False
        self.stop_playback = threading.Event()
        self.homing = False

        # Pick & Place: 3 taught joint-space waypoints (each a 6-vector
        # snapshot of self.target), None until captured. Kept as plain data
        # rather than something loaded from a file - re-teaching after moving
        # the bench is one press per waypoint, and nothing here claims to know
        # where a real object is without a camera telling it so.
        self.action_poses = {"hover": None, "pickup": None, "place": None}

        self.ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
        if self.ee_site_id < 0:
            raise RuntimeError("open_manipulator_x.xml has no 'end_effector' site - rerun urdf-xml.py")

        self.workspace_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "workspace_marker")
        if self.workspace_body_id < 0:
            raise RuntimeError("open_manipulator_x.xml has no 'workspace_marker' body - rerun urdf-xml.py")

        self.hw = HardwareLink(self.set_status)

        pygame.init()
        pygame.joystick.init()

        self._build_ui()
        self._bind_keyboard()

        self.sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
        self.sim_thread.start()

        # Serial I/O runs on its own thread so blocking bus transactions can
        # never stall the motion loop - see _hw_loop for the measurements.
        self.hw_thread = threading.Thread(target=self._hw_loop, daemon=True)
        self.hw_thread.start()

        self._poll_keyboard_refresh()
        self._poll_ee_readout()
        self._poll_feedback_readout()
        self._poll_gamepad_readout()
        self._poll_web_readout()

        root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Mobile control panel. Started last so it can never delay the
        # desktop UI coming up, and failure to bind is non-fatal - the
        # desktop app stays fully usable without it.
        self.web = None
        if OPTS.web:
            self.web = web_app.start(
                self, sys.modules[__name__],
                port=OPTS.port if OPTS.port is not None else web_app.WEB_PORT,
                bind=OPTS.bind if OPTS.bind is not None else web_app.WEB_BIND)

    def _bind_keyboard(self):
        self.root.bind_all("<KeyPress>", self._on_key_press)
        self.root.bind_all("<KeyRelease>", self._on_key_release)
        self.root.focus_force()

    def _on_key_press(self, event):
        key = event.keysym.lower()
        if key in KEY_BINDINGS or key in CARTESIAN_KEY_BINDINGS:
            self.keys_held.add(key)

    def _on_key_release(self, event):
        key = event.keysym.lower()
        self.keys_held.discard(key)

    def _poll_keyboard_refresh(self):
        if self.keys_held:
            self._refresh_sliders_from_target()
        if not self.stop_event.is_set():
            self.root.after(50, self._poll_keyboard_refresh)

    def _build_scroll_container(self):
        """Put the whole panel on a scrollable canvas and size the window to
        the screen.

        The panel's natural height is ~1000 px and grows with every section;
        on a shorter display the bottom sections (Gamepad Teleop, Position
        Gain Tuning) were simply cut off with no way to reach them, because
        a bare Tk window clips its children rather than scrolling them."""
        outer = ttk.Frame(self.root)
        outer.pack(fill="both", expand=True)

        self.canvas = tk.Canvas(outer, highlightthickness=0, borderwidth=0)
        vbar = ttk.Scrollbar(outer, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=vbar.set)
        vbar.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)

        self.container = ttk.Frame(self.canvas)
        self._container_win = self.canvas.create_window(
            (0, 0), window=self.container, anchor="nw")
        self.container.columnconfigure(0, weight=1)

        def _on_content_resize(_event):
            self.canvas.configure(scrollregion=self.canvas.bbox("all"))

        def _on_canvas_resize(event):
            # Keep the inner frame as wide as the viewport so the sections'
            # sticky="ew" still fills the width instead of hugging content.
            self.canvas.itemconfigure(self._container_win, width=event.width)

        self.container.bind("<Configure>", _on_content_resize)
        self.canvas.bind("<Configure>", _on_canvas_resize)
        self._bind_mousewheel()
        self.root.after_idle(self._fit_window_to_screen)

    def _bind_mousewheel(self):
        """Wheel scrolling, X11 (Button-4/5) and Windows/macOS (MouseWheel)."""
        def _wheel(event):
            if event.num == 4:
                delta = -1
            elif event.num == 5:
                delta = 1
            else:
                delta = -1 if event.delta > 0 else 1
            self.canvas.yview_scroll(delta, "units")

        for seq in ("<Button-4>", "<Button-5>", "<MouseWheel>"):
            self.root.bind_all(seq, _wheel)

    def _fit_window_to_screen(self):
        """Open at the panel's natural size, but never taller than the screen.

        Called after_idle so the geometry manager has computed the real
        requested size of the fully-populated panel first."""
        self.root.update_idletasks()
        want_w = self.container.winfo_reqwidth() + SCROLLBAR_ALLOWANCE
        want_h = self.container.winfo_reqheight()
        max_w = self.root.winfo_screenwidth() - SCREEN_MARGIN
        max_h = self.root.winfo_screenheight() - SCREEN_MARGIN
        self.root.geometry(f"{min(want_w, max_w)}x{min(want_h, max_h)}")
        # Wide enough to not clip the widest row, short enough to still fit a
        # small screen - the scrollbar covers whatever is left over.
        self.root.minsize(min(want_w, max_w), MIN_WINDOW_HEIGHT)

    def _build_ui(self):
        pad = {"padx": 10, "pady": 6}
        self._build_scroll_container()

        sliders = ttk.LabelFrame(self.container, text="Joint Control")
        sliders.grid(row=0, column=0, sticky="ew", **pad)

        self.scale_vars = []
        for row, (label, ctrl_idx, dxl_id, lo, hi, kind) in enumerate(JOINT_SPECS):
            ttk.Label(sliders, text=f"{label} (ID {dxl_id})").grid(row=row, column=0, sticky="w", padx=6, pady=4)
            var = tk.DoubleVar(value=self.target[ctrl_idx])
            scale = ttk.Scale(sliders, from_=lo, to=hi, orient="horizontal", variable=var, length=300,
                               command=lambda v, i=ctrl_idx: self._on_slider(i, v))
            scale.grid(row=row, column=1, sticky="ew", padx=6)
            val_label = ttk.Label(sliders, width=8, text=f"{self.target[ctrl_idx]:.3f}")
            val_label.grid(row=row, column=2, padx=6)
            self.scale_vars.append((var, val_label, ctrl_idx, scale))

        self.home_btn = ttk.Button(sliders, text="Home Position", command=self.on_go_home)
        self.home_btn.grid(row=len(JOINT_SPECS), column=0, columnspan=2, sticky="w", padx=6, pady=(6, 6))

        self.home_status_var = tk.StringVar(value="")
        ttk.Label(sliders, textvariable=self.home_status_var).grid(
            row=len(JOINT_SPECS), column=2, sticky="w", padx=6)

        fb = ttk.LabelFrame(self.container, text="Live Motor Feedback (present position read from the real arm)")
        fb.grid(row=4, column=0, sticky="ew", **pad)

        for col, head in enumerate(("Joint", "Tick", "Value", "Degrees")):
            ttk.Label(fb, text=head, font=("TkDefaultFont", 9, "bold")).grid(
                row=0, column=col, sticky="w", padx=8, pady=(4, 2))

        self.feedback_labels = {}
        for row, (label, ctrl_idx, dxl_id, lo, hi, kind) in enumerate(JOINT_SPECS, start=1):
            ttk.Label(fb, text=f"{label} (ID {dxl_id})").grid(row=row, column=0, sticky="w", padx=8, pady=1)
            cells = []
            for col in range(1, 4):
                cell = ttk.Label(fb, text="-", width=10)
                cell.grid(row=row, column=col, sticky="w", padx=8)
                cells.append(cell)
            self.feedback_labels[dxl_id] = cells

        self.mirror_var = tk.BooleanVar(value=False)
        self.mirror_check = ttk.Checkbutton(
            fb, text="Mirror mode - the REAL arm drives the twin (stops sending commands)",
            variable=self.mirror_var, command=self.on_toggle_mirror)
        self.mirror_check.grid(row=len(JOINT_SPECS) + 1, column=0, columnspan=4, sticky="w", padx=8, pady=(6, 2))

        self.mirror_note_var = tk.StringVar(value="")
        ttk.Label(fb, textvariable=self.mirror_note_var, wraplength=560).grid(
            row=len(JOINT_SPECS) + 2, column=0, columnspan=4, sticky="w", padx=8, pady=(0, 4))

        rec = ttk.LabelFrame(self.container, text="Record && Playback (hand-guide the arm, then replay it)")
        rec.grid(row=5, column=0, sticky="ew", **pad)

        self.record_btn = ttk.Button(rec, text="● Record", command=self.on_toggle_record, state="disabled")
        self.record_btn.grid(row=0, column=0, padx=6, pady=6)

        self.play_btn = ttk.Button(rec, text="▶ Play", command=self.on_toggle_play, state="disabled")
        self.play_btn.grid(row=0, column=1, padx=6, pady=6)

        self.loop_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(rec, text="Loop", variable=self.loop_var).grid(row=0, column=2, padx=6)

        ttk.Button(rec, text="Save...", command=self.on_save_recording).grid(row=0, column=3, padx=6)
        ttk.Button(rec, text="Load...", command=self.on_load_recording).grid(row=0, column=4, padx=6)
        ttk.Button(rec, text="Clear", command=self.on_clear_recording).grid(row=0, column=5, padx=6)

        self.record_status_var = tk.StringVar(value="No recording. Connect, then TORQUE OFF to hand-guide.")
        ttk.Label(rec, textvariable=self.record_status_var, wraplength=560).grid(
            row=1, column=0, columnspan=6, sticky="w", padx=8, pady=(0, 6))

        # Quick Actions: single-press gripper presets, and a taught Pick & Place
        # sequence - ROBOTIS's own flagship OpenManipulator-X demo, adapted for
        # a bench with no camera/AR-marker perception. Rather than guess at
        # real-world coordinates (which could drive the gripper into the table
        # or grasp at nothing), the three waypoints are TAUGHT: jog the arm by
        # hand or by slider, press Capture at each of Hover/Pickup/Place, then
        # Run plays the same eased ramp Home Position uses between them.
        act = ttk.LabelFrame(self.container, text="Quick Actions (gripper presets, taught Pick && Place)")
        act.grid(row=6, column=0, sticky="ew", **pad)

        ttk.Button(act, text="Open Gripper", command=lambda: self.on_gripper_preset(False)).grid(
            row=0, column=0, padx=6, pady=6)
        ttk.Button(act, text="Close Gripper", command=lambda: self.on_gripper_preset(True)).grid(
            row=0, column=1, padx=6, pady=6)

        ttk.Separator(act, orient="vertical").grid(row=0, column=2, rowspan=2, sticky="ns", padx=4)

        self.capture_btns = {}
        for col, name in ((3, "hover"), (4, "pickup"), (5, "place")):
            btn = ttk.Button(act, text=f"Capture {name.capitalize()}",
                              command=lambda n=name: self.on_capture_pose(n))
            btn.grid(row=0, column=col, padx=6, pady=6)
            self.capture_btns[name] = btn

        self.pickplace_run_btn = ttk.Button(act, text="Run Pick && Place",
                                             command=self.on_run_pick_place, state="disabled")
        self.pickplace_run_btn.grid(row=0, column=6, padx=6, pady=6)

        self.pickplace_status_var = tk.StringVar(
            value="Jog to a hover height above the object, click Capture Hover; same for Pickup (lowered onto "
                  "it) and Place (drop-off). Run plays: hover -> pickup -> close -> hover -> place -> open -> hover.")
        ttk.Label(act, textvariable=self.pickplace_status_var, wraplength=560).grid(
            row=1, column=0, columnspan=7, sticky="w", padx=8, pady=(0, 6))

        tune = ttk.LabelFrame(self.container, text="Position Gain Tuning (live - fixes joint vibration at rest)")
        tune.grid(row=8, column=0, sticky="ew", **pad)

        ttk.Label(tune, text="Joint:").grid(row=0, column=0, sticky="e", padx=(8, 2), pady=6)
        self.tune_target_var = tk.StringVar(value="All arm (11-14)")
        self.tune_target_combo = ttk.Combobox(
            tune, textvariable=self.tune_target_var, width=16, state="readonly",
            values=["All arm (11-14)"] + [f"{spec[0]} (ID {spec[2]})" for spec in JOINT_SPECS if spec[5] == "rad"])
        self.tune_target_combo.grid(row=0, column=1, sticky="w", padx=(0, 10))

        ttk.Label(tune, text="P:").grid(row=0, column=2, sticky="e", padx=(6, 2))
        self.tune_p_var = tk.StringVar(value=str(POSITION_P_GAIN))
        ttk.Entry(tune, textvariable=self.tune_p_var, width=7).grid(row=0, column=3, sticky="w")

        ttk.Label(tune, text="D:").grid(row=0, column=4, sticky="e", padx=(6, 2))
        self.tune_d_var = tk.StringVar(value=str(POSITION_D_GAIN))
        ttk.Entry(tune, textvariable=self.tune_d_var, width=7).grid(row=0, column=5, sticky="w")

        ttk.Label(tune, text="Vel:").grid(row=0, column=6, sticky="e", padx=(6, 2))
        self.tune_vel_var = tk.StringVar(value=str(PROFILE_VELOCITY))
        ttk.Entry(tune, textvariable=self.tune_vel_var, width=6).grid(row=0, column=7, sticky="w")

        ttk.Label(tune, text="Acc:").grid(row=0, column=8, sticky="e", padx=(6, 2))
        self.tune_acc_var = tk.StringVar(value=str(PROFILE_ACCELERATION))
        ttk.Entry(tune, textvariable=self.tune_acc_var, width=6).grid(row=0, column=9, sticky="w")

        self.tune_apply_btn = ttk.Button(tune, text="Apply", command=self.on_apply_gains, state="disabled")
        self.tune_apply_btn.grid(row=0, column=10, padx=8)

        self.tune_read_btn = ttk.Button(tune, text="Read current", command=self.on_read_gains, state="disabled")
        self.tune_read_btn.grid(row=0, column=11, padx=4)

        ttk.Label(tune, wraplength=600, foreground="#444", text=(
            "All four apply instantly with torque on - no restart. "
            "Vibrating at rest: lower P in ~200 steps (800->600->400), stop when the buzz goes. "
            "Stuttering while moving: Vel too HIGH makes it finish each step early and wait "
            "(70 tracks a 1.5 rad/s jog); too LOW makes it lag behind.")).grid(
            row=1, column=0, columnspan=12, sticky="w", padx=8, pady=(0, 4))

        self.tune_status_var = tk.StringVar(value="Connect to enable live tuning.")
        ttk.Label(tune, textvariable=self.tune_status_var, wraplength=600).grid(
            row=2, column=0, columnspan=12, sticky="w", padx=8, pady=(0, 6))

        gp = ttk.LabelFrame(self.container, text="Gamepad Teleop")
        gp.grid(row=7, column=0, sticky="ew", **pad)

        self.gamepad_connect_btn = ttk.Button(gp, text="Connect Gamepad", command=self.on_connect_gamepad)
        self.gamepad_connect_btn.grid(row=0, column=0, padx=6, pady=6)

        self.gamepad_var = tk.BooleanVar(value=False)
        self.gamepad_check = ttk.Checkbutton(gp, text="Enable Gamepad", variable=self.gamepad_var,
                                              command=self.on_toggle_gamepad, state="disabled")
        self.gamepad_check.grid(row=0, column=1, padx=6, pady=6)

        self.gamepad_status_var = tk.StringVar(value="No gamepad connected.")
        ttk.Label(gp, textvariable=self.gamepad_status_var, wraplength=560).grid(
            row=1, column=0, columnspan=4, sticky="w", padx=8, pady=(0, 2))

        ttk.Label(gp, text=GAMEPAD_HELP, wraplength=560).grid(
            row=2, column=0, columnspan=4, sticky="w", padx=8, pady=(2, 4))

        self.gamepad_raw_var = tk.StringVar(value="")
        ttk.Label(gp, textvariable=self.gamepad_raw_var, wraplength=560, foreground="#555").grid(
            row=3, column=0, columnspan=4, sticky="w", padx=8, pady=(0, 6))

        remote = ttk.LabelFrame(self.container, text="Remote Access (mobile web panel)")
        remote.grid(row=9, column=0, sticky="ew", **pad)

        self.web_url_var = tk.StringVar(value="Web panel starting...")
        ttk.Label(remote, textvariable=self.web_url_var, wraplength=600,
                  font=("TkDefaultFont", 10, "bold")).grid(
            row=0, column=0, sticky="w", padx=8, pady=(6, 2))

        self.web_info_var = tk.StringVar(value="")
        ttk.Label(remote, textvariable=self.web_info_var, wraplength=600,
                  foreground="#444").grid(row=1, column=0, sticky="w", padx=8, pady=(0, 6))

        kb = ttk.LabelFrame(self.container, text="Keyboard Controls (click the window first to give it focus)")
        kb.grid(row=2, column=0, sticky="ew", **pad)
        ttk.Label(kb, text=KEYBOARD_HELP).grid(row=0, column=0, sticky="w", padx=6, pady=4)

        hw = ttk.LabelFrame(self.container, text="Hardware (U2D2 /dev/ttyUSB0 @ 1,000,000 bps, protocol 2.0)")
        hw.grid(row=1, column=0, sticky="ew", **pad)

        self.connect_btn = ttk.Button(hw, text="Connect", command=self.on_connect)
        self.connect_btn.grid(row=0, column=0, padx=6, pady=6)

        self.disconnect_btn = ttk.Button(hw, text="Disconnect", command=self.on_disconnect, state="disabled")
        self.disconnect_btn.grid(row=0, column=1, padx=6, pady=6)

        self.torque_btn = ttk.Button(hw, text="Enable Torque", command=self.on_enable_torque, state="disabled")
        self.torque_btn.grid(row=0, column=2, padx=6, pady=6)

        self.estop_btn = tk.Button(hw, text="TORQUE OFF (E-STOP)", bg="#c0392b", fg="white",
                                    command=self.on_estop)
        self.estop_btn.grid(row=0, column=3, padx=6, pady=6)

        self.diag_btn = ttk.Button(hw, text="Diagnose Gripper", command=self.on_diagnose_gripper, state="disabled")
        self.diag_btn.grid(row=0, column=4, padx=6, pady=6)

        self.calib_btn = ttk.Button(hw, text="Calibrate Gripper", command=self.on_calibrate_gripper, state="disabled")
        self.calib_btn.grid(row=0, column=5, padx=6, pady=6)

        self.status_var = tk.StringVar(value="Not connected - simulation only")
        ttk.Label(hw, textvariable=self.status_var, wraplength=560).grid(row=1, column=0, columnspan=6, sticky="w", padx=6)

        ik = ttk.LabelFrame(self.container, text="Inverse Kinematics (end-effector target, meters)")
        ik.grid(row=3, column=0, sticky="ew", **pad)

        ttk.Label(ik, text="Tip: the translucent blue sphere in the 3D viewer is the arm's reach - "
                            "select and drag on it (see the viewer's own on-screen controls for the "
                            "exact gesture) to fill in X/Y/Z below.",
                  wraplength=560).grid(row=0, column=0, columnspan=7, sticky="w", padx=6, pady=(4, 2))

        self.ee_readout_var = tk.StringVar(value="Current EE pos: -")
        ttk.Label(ik, textvariable=self.ee_readout_var).grid(row=1, column=0, columnspan=4, sticky="w", padx=6, pady=(0, 8))

        self.ik_entries = {}
        for col, axis in enumerate(("X", "Y", "Z")):
            ttk.Label(ik, text=f"{axis}:").grid(row=2, column=2 * col, sticky="e", padx=(6, 2))
            var = tk.StringVar()
            entry = ttk.Entry(ik, textvariable=var, width=8)
            entry.grid(row=2, column=2 * col + 1, sticky="w", padx=(0, 6))
            self.ik_entries[axis] = var

        ttk.Button(ik, text="Solve & Move", command=self.on_solve_ik).grid(row=2, column=6, padx=10)

        self.ik_status_var = tk.StringVar(value="")
        ttk.Label(ik, textvariable=self.ik_status_var, wraplength=500).grid(row=3, column=0, columnspan=7, sticky="w", padx=6, pady=(4, 4))

        self.cartesian_jog_var = tk.BooleanVar(value=False)
        self.cartesian_jog_check = ttk.Checkbutton(
            ik, text="Cartesian jog mode - R/F=X  T/G=Y  Y/H=Z (drives the end-effector directly via IK)",
            variable=self.cartesian_jog_var, command=self.on_toggle_cartesian_jog)
        self.cartesian_jog_check.grid(row=4, column=0, columnspan=7, sticky="w", padx=6, pady=(2, 6))

    def _on_slider(self, ctrl_idx, value):
        value = float(value)
        with self.lock:
            self.target[ctrl_idx] = value
        for var, label, idx, scale in self.scale_vars:
            if idx == ctrl_idx:
                label.config(text=f"{value:.3f}")

    def set_status(self, text):
        self.root.after(0, lambda: self.status_var.set(text))

    def on_connect(self):
        self.connect_btn.config(state="disabled")
        threading.Thread(target=self._connect_worker, daemon=True).start()

    def _connect_worker(self):
        ok = self.hw.connect()
        if not ok:
            self.root.after(0, lambda: self.connect_btn.config(state="normal"))
            return

        ticks = self.hw.read_present_ticks()
        if ticks:
            with self.lock:
                self._sync_targets_from_ticks_locked(ticks)
            self.root.after(0, self._refresh_sliders_from_target)
            self.set_status(f"Connected. Gripper at tick {ticks[GRIPPER_ID]}, "
                            f"range {self.gripper_tick_at_lo}..{self.gripper_tick_at_hi}.")

        self._apply_hardware_joint_limits()
        self.root.after(0, lambda: self.disconnect_btn.config(state="normal"))
        self.root.after(0, lambda: self.torque_btn.config(state="normal"))
        self.root.after(0, lambda: self.diag_btn.config(state="normal"))
        self.root.after(0, lambda: self.calib_btn.config(state="normal"))
        self.root.after(0, lambda: self.record_btn.config(state="normal"))
        self.root.after(0, lambda: self.tune_apply_btn.config(state="normal"))
        self.root.after(0, lambda: self.tune_read_btn.config(state="normal"))
        self.root.after(0, lambda: self.tune_status_var.set(
            "Live tuning ready - vibrating joint? Lower P and Apply, no restart needed."))
        self.root.after(0, self._refresh_playback_buttons)

    def _sync_targets_from_ticks_locked(self, ticks):
        """Set every target from measured hardware ticks. Caller holds lock."""
        for label, ctrl_idx, dxl_id, lo, hi, kind in JOINT_SPECS:
            tick = ticks[dxl_id]
            if kind == "gripper":
                val = tick_to_gripper(tick, lo, hi, self.gripper_tick_at_lo, self.gripper_tick_at_hi)
            else:
                val = tick_to_rad(tick)
            self.target[ctrl_idx] = max(lo, min(hi, val))
        self.target[5] = self.target[4]

    def _apply_hardware_joint_limits(self):
        """Reads each arm motor's own EEPROM Min/Max Position Limit and
        intersects it with the URDF-derived joint_limits, so the digital
        twin (sliders, keyboard jog, and hence hardware sync) can never
        command past whatever the real arm's structure/assembly actually
        allows - independent of the model's own claimed range."""
        for label, ctrl_idx, dxl_id, lo, hi, kind in JOINT_SPECS:
            if kind != "rad":
                continue  # gripper handled separately via GRIPPER_COEFFICIENT
            limits = self.hw.read_position_limits(dxl_id)
            if limits is None:
                continue
            min_tick, max_tick = limits
            hw_lo, hw_hi = sorted((tick_to_rad(min_tick), tick_to_rad(max_tick)))
            new_lo, new_hi = max(lo, hw_lo), min(hi, hw_hi)
            if new_hi <= new_lo:
                self.set_status(f"ID {dxl_id} reported invalid hardware limits, keeping software range")
                continue
            with self.lock:
                self.joint_limits[ctrl_idx] = (new_lo, new_hi)
                self.target[ctrl_idx] = max(new_lo, min(new_hi, self.target[ctrl_idx]))
            self.root.after(0, lambda i=ctrl_idx, a=new_lo, b=new_hi: self._update_slider_range(i, a, b))

    def _update_slider_range(self, ctrl_idx, lo, hi):
        for var, label, idx, scale in self.scale_vars:
            if idx == ctrl_idx:
                scale.config(from_=lo, to=hi)

    def _refresh_sliders_from_target(self):
        with self.lock:
            snapshot = list(self.target)
        for var, label, idx, scale in self.scale_vars:
            var.set(snapshot[idx])
            label.config(text=f"{snapshot[idx]:.3f}")

    def on_enable_torque(self):
        threading.Thread(target=self._enable_torque_worker, daemon=True).start()

    def _enable_torque_worker(self):
        # Re-sync every target to the arm's ACTUAL present position before
        # energising. Without this, whatever the sliders happen to read
        # becomes an instant goal the moment torque engages, and any gap
        # gets taken at full speed - the lunge-into-the-stop that trips
        # Overload. After this, torque-on is always a no-motion event.
        ticks = self.hw.read_present_ticks()
        if ticks:
            with self.lock:
                self._sync_targets_from_ticks_locked(ticks)
            self.root.after(0, self._refresh_sliders_from_target)
        else:
            self.set_status("Could not read present positions - torque NOT enabled")
            return

        self.hw.set_torque(True)
        if not self.hw.verify_torque(True):
            self.set_status("Torque enable did NOT take effect - check power (12V) and wiring")

    def on_estop(self):
        self.hw.set_torque(False)

    def on_go_home(self):
        if self.recording or self.playing or self.homing:
            return
        threading.Thread(target=self._go_home_worker, daemon=True).start()

    def _ease_target_to(self, dest_pose, ease_gripper=True):
        """Smoothly ramps self.target from wherever it is now to dest_pose
        over HOME_APPROACH_TIME - the exact loop Home Position used to run
        inline. Pulled out so Pick & Place can reuse the identical, already-
        tuned motion for each leg of its sequence rather than a second copy
        that could quietly drift out of sync with Home's feel over time.

        ease_gripper=False leaves target[4] (and its mirror, target[5]) alone
        for the whole ramp. Pick & Place needs this: hover/pickup/place are
        taught poses that each freeze whatever gripper opening happened to be
        set at teach time, and blending that in while lifting or retreating
        would silently open a gripper that Pick & Place had just closed
        around an object. Home wants the opposite - HOME_POSITION specifies a
        real gripper target and returning it there is the point - so it keeps
        the default."""
        with self.lock:
            start_pose = np.copy(self.target)
        steps = max(1, int(HOME_APPROACH_TIME * HOME_APPROACH_RATE))
        for i in range(steps + 1):
            a = i / steps
            a = a * a * (3 - 2 * a)   # smoothstep: no jerk at either end
            with self.lock:
                self.target[:4] = start_pose[:4] + a * (dest_pose[:4] - start_pose[:4])
                if ease_gripper:
                    self.target[4] = start_pose[4] + a * (dest_pose[4] - start_pose[4])
                self.target[5] = self.target[4]
            self.root.after(0, self._refresh_sliders_from_target)
            time.sleep(1.0 / HOME_APPROACH_RATE)

    def _go_home_worker(self):
        # "Override everything": this is the same "one thing owns the
        # target" rule as Record/Play - mirror, cartesian jog, gamepad, the
        # sliders, and manual torque toggling are all locked out for the
        # duration so nothing fights the move, and restored when it's done.
        self.homing = True
        self.root.after(0, lambda: self.home_btn.config(state="disabled"))
        self.root.after(0, lambda: self.record_btn.config(state="disabled"))
        self.root.after(0, lambda: self.play_btn.config(state="disabled"))
        self.root.after(0, lambda: self._set_teach_mode_controls(True))
        if self.mirror_mode:
            self.mirror_var.set(False)
            self.mirror_mode = False
            self.root.after(0, lambda: self.mirror_note_var.set("Mirror OFF: cancelled by Home Position."))
        try:
            home = np.clip(HOME_POSITION, self.model.jnt_range[:6, 0], self.model.jnt_range[:6, 1])
            for ctrl_idx in range(4):
                lo, hi = self.joint_limits[ctrl_idx]
                home[ctrl_idx] = max(lo, min(hi, home[ctrl_idx]))

            self.home_status_var.set("Moving to home...")
            self._ease_target_to(home)
            self.home_status_var.set("At home position.")
        finally:
            self.homing = False
            self.root.after(0, lambda: self.home_btn.config(state="normal"))
            self.root.after(0, lambda: self.record_btn.config(state="normal"))
            self.root.after(0, lambda: self._set_teach_mode_controls(False))
            self.root.after(0, self._refresh_playback_buttons)

    def on_gripper_preset(self, closed: bool):
        """One-press gripper open/close, driven by the same calibrated
        joint_limits[4] the slider already respects (so Calibrate Gripper
        re-measuring the travel keeps this correct with no extra work)."""
        if self.recording or self.playing or self.homing:
            return
        lo, hi = self.joint_limits[4]
        with self.lock:
            self.target[4] = lo if closed else hi
            self.target[5] = self.target[4]
        self.root.after(0, self._refresh_sliders_from_target)

    def on_capture_pose(self, name: str):
        """Snapshots the CURRENT target as one of Pick & Place's three
        waypoints. Refused mid-motion (recording/playing/homing)
        because otherwise it would capture whatever transient in-flight pose
        the arm happens to be passing through, not a deliberate rest pose."""
        if self.recording or self.playing or self.homing:
            return
        with self.lock:
            pose = np.copy(self.target)
        self.action_poses[name] = pose
        self.pickplace_status_var.set(f"{name.capitalize()} captured.")
        self._refresh_action_buttons()

    def _refresh_action_buttons(self):
        ready = all(pose is not None for pose in self.action_poses.values())
        busy = self.recording or self.playing or self.homing
        self.pickplace_run_btn.config(state="normal" if (ready and not busy) else "disabled")

    def on_run_pick_place(self):
        if self.recording or self.playing or self.homing:
            return
        if any(pose is None for pose in self.action_poses.values()):
            self.pickplace_status_var.set("Capture Hover, Pickup, and Place first.")
            return
        threading.Thread(target=self._pick_place_worker, daemon=True).start()

    def _pick_place_worker(self):
        # Reuses self.homing as the busy flag, same "one thing owns the
        # target" lockout _go_home_worker uses - every gate that already
        # checks self.homing (jog, the gamepad Y binding, Home's own guard)
        # then also backs off for a running Pick & Place, at the cost of the
        # web UI's "HOMING" pill showing during one too. A second flag that
        # would always agree with this one isn't worth carrying.
        self.homing = True
        self.root.after(0, lambda: self.home_btn.config(state="disabled"))
        self.root.after(0, lambda: self.record_btn.config(state="disabled"))
        self.root.after(0, lambda: self.play_btn.config(state="disabled"))
        self.root.after(0, lambda: self._set_teach_mode_controls(True))
        self.root.after(0, self._refresh_action_buttons)
        if self.mirror_mode:
            self.mirror_var.set(False)
            self.mirror_mode = False
            self.root.after(0, lambda: self.mirror_note_var.set("Mirror OFF: cancelled by Pick & Place."))
        try:
            lo4, hi4 = self.joint_limits[4]
            hover = self.action_poses["hover"]
            pickup = self.action_poses["pickup"]
            place = self.action_poses["place"]

            def status(text):
                self.pickplace_status_var.set(text)

            # Every ease below is ease_gripper=False: the gripper is driven
            # ONLY by the two explicit writes (close after Pickup, open after
            # Place), never blended in from a taught pose's frozen opening -
            # see _ease_target_to's docstring for why that matters.
            status("Moving to hover...")
            self._ease_target_to(hover, ease_gripper=False)
            status("Descending to pickup...")
            self._ease_target_to(pickup, ease_gripper=False)
            status("Closing gripper...")
            with self.lock:
                self.target[4] = lo4
                self.target[5] = self.target[4]
            self.root.after(0, self._refresh_sliders_from_target)
            time.sleep(GRIPPER_ACTION_SETTLE)
            status("Lifting...")
            self._ease_target_to(hover, ease_gripper=False)
            status("Moving to place...")
            self._ease_target_to(place, ease_gripper=False)
            status("Opening gripper...")
            with self.lock:
                self.target[4] = hi4
                self.target[5] = self.target[4]
            self.root.after(0, self._refresh_sliders_from_target)
            time.sleep(GRIPPER_ACTION_SETTLE)
            status("Retreating...")
            self._ease_target_to(hover, ease_gripper=False)
            status("Pick & Place complete.")
        finally:
            self.homing = False
            self.root.after(0, lambda: self.home_btn.config(state="normal"))
            self.root.after(0, lambda: self.record_btn.config(state="normal"))
            self.root.after(0, lambda: self._set_teach_mode_controls(False))
            self.root.after(0, self._refresh_playback_buttons)
            self.root.after(0, self._refresh_action_buttons)

    def on_disconnect(self):
        """Releases /dev/ttyUSB0 so DYNAMIXEL Wizard (or anything else) can
        open it - only one program can hold the port at a time. Stops
        anything in-flight first, since none of it means anything with no
        port to talk over."""
        if self.recording:
            self.on_toggle_record()   # stop and keep whatever was captured
        if self.playing:
            self.stop_playback.set()
        if self.mirror_mode:
            self.mirror_var.set(False)
            self.on_toggle_mirror()

        self.hw.disconnect()
        with self.lock:
            self.present_ticks = {}

        self.disconnect_btn.config(state="disabled")
        self.torque_btn.config(state="disabled")
        self.diag_btn.config(state="disabled")
        self.calib_btn.config(state="disabled")
        self.record_btn.config(state="disabled")
        self.play_btn.config(state="disabled")
        self.tune_apply_btn.config(state="disabled")
        self.tune_read_btn.config(state="disabled")
        self.connect_btn.config(state="normal")
        self.set_status(f"Disconnected - {DEVICENAME} is free (e.g. for DYNAMIXEL Wizard). "
                        f"Click Connect to resume.")

    def _poll_web_readout(self):
        """Keep the desktop panel's Remote Access section current.

        Reads the same snapshot the phone polls, so if this section looks
        wrong the phone is seeing exactly the same wrong thing."""
        host = {}
        if getattr(self, "web", None) is not None:
            with self.web_state_lock:
                host = (self.web_state or {}).get("host", {})
        if not host:
            self.web_url_var.set("Web panel not running (--no-web, or the port was busy)")
            self.web_info_var.set("")
        else:
            auth = "token required" if host["auth"] else "NO AUTH - trusted network only"
            self.web_url_var.set(f"http://{host['ip']}:{host['port']}/    ({auth})")
            temp = f"   CPU {host['cpu_c']}C" if host.get("cpu_c") is not None else ""
            mins, secs = divmod(host["uptime_s"], 60)
            hours, mins = divmod(mins, 60)
            self.web_info_var.set(
                f"host {host['name']}   up {hours}h{mins:02d}m{secs:02d}s{temp}   "
                f"feedback {host['feedback_hz']:.0f} Hz / record {host['record_hz']:.0f} Hz   "
                f"viewer {'on' if host['viewer'] else 'off'}")
        if not self.stop_event.is_set():
            self.root.after(1000, self._poll_web_readout)

    def _poll_feedback_readout(self):
        """Refresh the feedback table from whatever the sim loop last read."""
        with self.lock:
            ticks = dict(self.present_ticks)
        for label, ctrl_idx, dxl_id, lo, hi, kind in JOINT_SPECS:
            cells = self.feedback_labels[dxl_id]
            tick = ticks.get(dxl_id)
            if tick is None:
                for c in cells:
                    c.config(text="-")
                continue
            if kind == "gripper":
                val = tick_to_gripper(tick, lo, hi, self.gripper_tick_at_lo, self.gripper_tick_at_hi)
                cells[1].config(text=f"{val:+.4f} m")
                cells[2].config(text="-")
            else:
                rad = tick_to_rad(tick)
                cells[1].config(text=f"{rad:+.3f} rad")
                cells[2].config(text=f"{math.degrees(rad):+.1f}")
            cells[0].config(text=str(tick))
        if not self.stop_event.is_set():
            self.root.after(100, self._poll_feedback_readout)

    def on_toggle_cartesian_jog(self):
        self.cartesian_jog_enabled = bool(self.cartesian_jog_var.get())
        self.ik_status_var.set(
            "Cartesian jog ON - R/F/T/G/Y/H move the end-effector in X/Y/Z."
            if self.cartesian_jog_enabled else "Cartesian jog off - keyboard jogs joints again.")

    def on_connect_gamepad(self):
        pygame.joystick.quit()
        pygame.joystick.init()
        count = pygame.joystick.get_count()
        if count == 0:
            self.gamepad_status_var.set("No gamepad detected - plug it in and click Connect Gamepad again.")
            self.gamepad_check.config(state="disabled")
            self.gamepad = None
            return
        self.gamepad = pygame.joystick.Joystick(0)
        self.gamepad.init()
        self.gamepad_status_var.set(
            f"Connected: {self.gamepad.get_name()}  "
            f"({self.gamepad.get_numaxes()} axes, {self.gamepad.get_numbuttons()} buttons). "
            f"If the mapping below looks wrong for your pad, watch the raw axis readout while "
            f"moving each stick and adjust the GAMEPAD_AXIS_* constants at the top of the file.")
        self.gamepad_check.config(state="normal")

    def on_toggle_gamepad(self):
        self.gamepad_enabled = bool(self.gamepad_var.get())

    def _tune_selected_ids(self):
        """Which motor IDs the tuning panel currently targets."""
        choice = self.tune_target_var.get()
        arm_ids = [spec[2] for spec in JOINT_SPECS if spec[5] == "rad"]
        if choice.startswith("All"):
            return arm_ids
        for spec in JOINT_SPECS:
            if spec[5] == "rad" and f"(ID {spec[2]})" in choice:
                return [spec[2]]
        return arm_ids

    def on_apply_gains(self):
        try:
            p = int(float(self.tune_p_var.get()))
            d = int(float(self.tune_d_var.get()))
            vel = int(float(self.tune_vel_var.get()))
            acc = int(float(self.tune_acc_var.get()))
        except ValueError:
            self.tune_status_var.set("P, D, Vel and Acc must all be numbers.")
            return
        if not (0 <= p <= 16383 and 0 <= d <= 16383):
            self.tune_status_var.set("P and D must be within 0-16383 (DYNAMIXEL gain range).")
            return
        if vel <= 0 or acc <= 0:
            # 0 means "no profile / maximum speed" on a DYNAMIXEL, which for
            # a streamed goal means an uncontrolled full-speed lunge.
            self.tune_status_var.set("Vel and Acc must be > 0 (0 disables the profile = full-speed moves).")
            return
        ids = self._tune_selected_ids()

        def worker():
            ok = self.hw.apply_tuning(ids, p, POSITION_I_GAIN, d, vel, acc)
            rad_s = vel * 0.229 * (2 * math.pi / 60)
            self.root.after(0, lambda: self.tune_status_var.set(
                f"Applied P={p} D={d} Vel={vel} ({rad_s:.2f} rad/s cap) Acc={acc} to ID {ids}."
                if ok else "Not connected - cannot apply."))
        threading.Thread(target=worker, daemon=True).start()

    def on_read_gains(self):
        ids = self._tune_selected_ids()

        def worker():
            parts = []
            for dxl_id in ids:
                gains = self.hw.read_position_gains(dxl_id)
                parts.append(f"ID{dxl_id}: P={gains[0]} I={gains[1]} D={gains[2]}"
                             if gains else f"ID{dxl_id}: <read failed>")
            self.root.after(0, lambda: self.tune_status_var.set("   ".join(parts)))
        threading.Thread(target=worker, daemon=True).start()

    def _poll_gamepad_readout(self):
        if self.gamepad is not None:
            with self.lock:
                axes = list(self.gamepad_axes_snapshot)
                buttons = list(self.gamepad_buttons_snapshot)
            if axes:
                axes_str = "  ".join(f"a{i}={v:+.2f}" for i, v in enumerate(axes))
                btn_str = "  ".join(f"b{i}={v}" for i, v in enumerate(buttons) if v)
                self.gamepad_raw_var.set(f"Raw: {axes_str}" + (f"   pressed: {btn_str}" if btn_str else ""))
        if not self.stop_event.is_set():
            self.root.after(100, self._poll_gamepad_readout)

    def on_toggle_mirror(self):
        self.mirror_mode = bool(self.mirror_var.get())
        if self.mirror_mode:
            # Deliberately does NOT drop torque: on a raised arm that would
            # let it collapse under gravity. The user decides via E-STOP.
            self.mirror_note_var.set(
                "Mirror ON: no commands are sent; the twin follows measured positions. "
                "To move the arm by hand, press TORQUE OFF first - but support the arm, "
                "it will drop under its own weight when de-energised.")
        else:
            # Re-sync targets to reality before resuming command flow, so
            # handing control back to the twin isn't a jump.
            ticks = self.hw.read_all_ticks_fast() if self.hw.connected else None
            if ticks and all(i in ticks for i in DXL_IDS):
                with self.lock:
                    self._sync_targets_from_ticks_locked(ticks)
                self._refresh_sliders_from_target()
            self.mirror_note_var.set("Mirror OFF: the twin commands the robot again.")

    # ---------------- Record / playback (teach by demonstration) ----------

    def _refresh_playback_buttons(self):
        can_play = bool(self.frames) and self.hw.connected and not self.recording and not self.playing
        self.play_btn.config(state="normal" if can_play else "disabled")

    def _set_record_status(self, text):
        self.root.after(0, lambda: self.record_status_var.set(text))

    def _set_teach_mode_controls(self, active: bool):
        """Record and Play both need exclusive control of the hardware
        write path - anything else that can command the arm (mirror,
        Calibrate, manual Enable Torque, the joint sliders) gets locked out
        while either is running, and restored when it stops. E-STOP is
        deliberately never touched here - it must always be reachable."""
        state = "disabled" if active else "normal"
        self.mirror_check.config(state=state)
        self.cartesian_jog_check.config(state=state)
        self.gamepad_check.config(state="disabled" if active else ("normal" if self.gamepad else "disabled"))
        self.calib_btn.config(state="disabled" if active else ("normal" if self.hw.connected else "disabled"))
        self.torque_btn.config(state="disabled" if active else ("normal" if self.hw.connected else "disabled"))
        for var, label, idx, scale in self.scale_vars:
            scale.config(state=state)
        # Capturing a Pick & Place waypoint mid-motion would snapshot a
        # transient in-flight pose rather than a deliberate rest pose, so it
        # gets the same lockout as everything else here.
        for btn in self.capture_btns.values():
            btn.config(state=state)
        if active:
            self.pickplace_run_btn.config(state="disabled")
        else:
            self._refresh_action_buttons()
        if active and self.cartesian_jog_enabled:
            # Recording/playback owns the target; a stray held jog key from
            # before the mode switch shouldn't keep nudging it mid-capture.
            self.cartesian_jog_var.set(False)
            self.cartesian_jog_enabled = False
        if active and self.gamepad_enabled:
            self.gamepad_var.set(False)
            self.gamepad_enabled = False

    def on_toggle_record(self):
        if self.playing or self.homing:
            return
        if not self.recording:
            if not self.hw.connected:
                self._set_record_status("Connect first - recording reads positions from the real arm.")
                return

            # Recording needs to be the ONLY thing driving state: mirror
            # would otherwise also be writing self.target from the same
            # feedback, which is harmless but redundant and confusing to
            # toggle mid-recording, so it's forced off and locked out.
            if self.mirror_mode:
                self.mirror_var.set(False)
                self.on_toggle_mirror()

            # Hand-guiding needs torque off, or you're fighting the servo.
            # This is the one control-table write recording makes for you
            # automatically, per your request - but de-energising a raised
            # arm lets it drop, so it's a confirmation, not a silent action.
            if self.hw.torque_on:
                if not messagebox.askyesno(
                        "Disable torque to record?",
                        "Recording needs torque OFF so you can hand-guide the arm.\n\n"
                        "Support the arm before continuing - it will drop under its "
                        "own weight once de-energised.\n\nDisable torque and start recording?"):
                    return
                self.hw.set_torque(False)

            self._set_teach_mode_controls(True)
            with self.lock:
                self.frames = []
                self.record_started_at = time.time()
                self.recording = True
            self.record_btn.config(text="■ Stop Recording")
            self.play_btn.config(state="disabled")
            self._set_record_status("RECORDING - hand-guide the arm now.")
        else:
            with self.lock:
                self.recording = False
                count = len(self.frames)
                dur = self.frames[-1][0] if self.frames else 0.0
            self.record_btn.config(text="● Record")
            self._set_teach_mode_controls(False)
            self._refresh_playback_buttons()
            self._set_record_status(f"Recorded {count} frames over {dur:.1f} s. Press Play to replay it.")

    def on_toggle_play(self):
        if self.recording or self.homing:
            return
        if self.playing:
            self.stop_playback.set()
            return
        if not self.hw.connected:
            self._set_record_status("Connect first - playback drives the real arm.")
            return
        if not self.frames:
            self._set_record_status("Nothing recorded yet.")
            return
        self.stop_playback.clear()
        threading.Thread(target=self._playback_worker, daemon=True).start()

    def _playback_worker(self):
        self.playing = True
        self.root.after(0, lambda: self.play_btn.config(text="■ Stop"))
        self.root.after(0, lambda: self.record_btn.config(state="disabled"))
        self.root.after(0, lambda: self._set_teach_mode_controls(True))
        # Mirror would fight playback (robot driving twin while twin drives
        # robot), so force it off for the duration.
        was_mirror = self.mirror_mode
        self.mirror_mode = False
        self.root.after(0, lambda: self.mirror_var.set(False))
        try:
            # Torque must be on to replay, and it has to come on at the arm's
            # CURRENT pose - the shared helper reads present positions first
            # so energising never causes a jump.
            if not self.hw.torque_on:
                self._set_record_status("Enabling torque before playback...")
                self._enable_torque_worker()
                if not self.hw.torque_on:
                    self._set_record_status("Could not enable torque - playback aborted.")
                    return

            with self.lock:
                start_pose = np.copy(self.target)
            first = np.array(self.frames[0][1])

            # Ease into frame 0 rather than snapping to it: the arm is
            # wherever your hand left it, which can be far from the start
            # of the recording. This is the same lunge risk as torque-on.
            self._set_record_status("Moving to start of recording...")
            steps = max(1, int(PLAYBACK_APPROACH_TIME * PLAYBACK_APPROACH_RATE))
            for i in range(steps + 1):
                if self.stop_playback.is_set():
                    return
                a = i / steps
                a = a * a * (3 - 2 * a)   # smoothstep: no jerk at either end
                with self.lock:
                    self.target[:] = start_pose + a * (first - start_pose)
                    self.target[5] = self.target[4]
                time.sleep(1.0 / PLAYBACK_APPROACH_RATE)

            passes = 0
            while not self.stop_playback.is_set():
                passes += 1
                t0 = time.time()
                for stamp, pose in self.frames:
                    if self.stop_playback.is_set():
                        return
                    # Sleep until this frame's own timestamp so playback runs
                    # at the speed it was demonstrated at.
                    lag = stamp - (time.time() - t0)
                    if lag > 0:
                        time.sleep(lag)
                    with self.lock:
                        self.target[:] = pose
                        self.target[5] = self.target[4]
                    self.root.after(0, self._refresh_sliders_from_target)
                if not self.loop_var.get():
                    break
                self._set_record_status(f"Looping - pass {passes} complete.")
            self._set_record_status(f"Playback finished ({passes} pass{'es' if passes != 1 else ''}).")
        finally:
            self.playing = False
            self.mirror_mode = was_mirror
            self.root.after(0, lambda: self.mirror_var.set(was_mirror))
            self.root.after(0, lambda: self.play_btn.config(text="▶ Play"))
            self.root.after(0, lambda: self.record_btn.config(state="normal"))
            self.root.after(0, lambda: self._set_teach_mode_controls(False))
            self.root.after(0, self._refresh_playback_buttons)
            self.root.after(0, self._refresh_sliders_from_target)

    def on_clear_recording(self):
        if self.playing or self.recording:
            return
        with self.lock:
            self.frames = []
        self._refresh_playback_buttons()
        self._set_record_status("Recording cleared.")

    def on_save_recording(self):
        if not self.frames:
            self._set_record_status("Nothing to save.")
            return
        os.makedirs(RECORDINGS_DIR, exist_ok=True)
        path = filedialog.asksaveasfilename(
            initialdir=RECORDINGS_DIR, defaultextension=".json",
            filetypes=[("Recording", "*.json")], title="Save recording")
        if not path:
            return
        self.save_recording_to(path)

    def save_recording_to(self, path):
        """Serialize the current frames to `path`. Shared by the Tk save
        dialog and the web API, so both write the identical format."""
        with self.lock:
            payload = {
                "joints": [spec[0] for spec in JOINT_SPECS],
                "gripper_ticks": [self.gripper_tick_at_lo, self.gripper_tick_at_hi],
                # .tolist() not list(): the latter yields np.float64, which
                # the json encoder refuses.
                "frames": [[float(t), np.asarray(p).tolist()] for t, p in self.frames],
            }
        with open(path, "w") as f:
            json.dump(payload, f)
        self._set_record_status(f"Saved {len(payload['frames'])} frames to {os.path.basename(path)}")
        return len(payload["frames"])

    def on_load_recording(self):
        if self.playing or self.recording:
            return
        os.makedirs(RECORDINGS_DIR, exist_ok=True)
        path = filedialog.askopenfilename(
            initialdir=RECORDINGS_DIR, filetypes=[("Recording", "*.json")], title="Load recording")
        if not path:
            return
        self.load_recording_from(path)

    def load_recording_from(self, path):
        """Load frames from `path`. Shared by the Tk load dialog and the web
        API. Returns True on success; status text explains any failure."""
        try:
            with open(path) as f:
                payload = json.load(f)
            frames = [(float(t), np.array(p, dtype=float)) for t, p in payload["frames"]]
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self._set_record_status(f"Could not load recording: {exc}")
            return False
        if not frames:
            self._set_record_status("That file has no frames.")
            return False
        with self.lock:
            self.frames = frames
        self._refresh_playback_buttons()
        self._set_record_status(
            f"Loaded {len(frames)} frames ({frames[-1][0]:.1f} s) from {os.path.basename(path)}")
        return True

    def on_diagnose_gripper(self):
        threading.Thread(target=lambda: self.set_status(self.hw.diagnose_gripper()), daemon=True).start()

    def on_calibrate_gripper(self):
        self.calib_btn.config(state="disabled")
        threading.Thread(target=self._calibrate_gripper_worker, daemon=True).start()

    def _calibrate_gripper_worker(self):
        self.set_status("Calibrating gripper - sweeping to both mechanical stops...")
        self.calibrating = True
        try:
            stops = self.hw.find_gripper_stops(progress_cb=self.set_status)
        finally:
            self.calibrating = False
            self.root.after(0, lambda: self.calib_btn.config(state="normal"))

        if stops is None:
            self.set_status("Gripper calibration failed (no clear travel found) - keeping previous range")
            return

        low, high = stops
        low += GRIPPER_CALIB_MARGIN     # back off the hard stops so we don't
        high -= GRIPPER_CALIB_MARGIN    # sit pressed against them
        # Keep the coefficient's sign convention: opening (+m) = fewer ticks.
        with self.lock:
            self.gripper_tick_at_lo, self.gripper_tick_at_hi = high, low
            self.gripper_calibrated = True

        ticks = self.hw.read_present_ticks()
        if ticks:
            with self.lock:
                self._sync_targets_from_ticks_locked(ticks)
            self.root.after(0, self._refresh_sliders_from_target)

        self.set_status(f"Gripper calibrated: measured stops {low - GRIPPER_CALIB_MARGIN}"
                        f"..{high + GRIPPER_CALIB_MARGIN}, using {low}..{high} "
                        f"({high - low} ticks travel). Slider now spans the full real range.")

    def _poll_ee_readout(self):
        with self.lock:
            ee_pos = self.data.site_xpos[self.ee_site_id].copy()
        self.ee_readout_var.set(f"Current EE pos: x={ee_pos[0]:.3f}  y={ee_pos[1]:.3f}  z={ee_pos[2]:.3f}")
        if not self.ik_entries["X"].get():
            for axis, val in zip(("X", "Y", "Z"), ee_pos):
                self.ik_entries[axis].set(f"{val:.3f}")
        if not self.stop_event.is_set():
            self.root.after(200, self._poll_ee_readout)

    def solve_ik(self, target_pos, max_iters=150, tol=1e-3, damping=0.05):
        """Damped least-squares IK over joint1-4 (arm only, gripper excluded)
        using the end_effector site's Jacobian. Runs on a scratch MjData so
        it never disturbs the live simulation/hardware state while solving,
        and every step is clamped to self.joint_limits - the same limits
        the sliders/keyboard obey, hardware-clamped once connected - so a
        solution can never ask the real arm to move somewhere it can't."""
        scratch = mujoco.MjData(self.model)
        with self.lock:
            scratch.qpos[:] = self.data.qpos[:]
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        err = np.zeros(3)

        for _ in range(max_iters):
            mujoco.mj_forward(self.model, scratch)
            cur = scratch.site_xpos[self.ee_site_id]
            err = target_pos - cur
            if np.linalg.norm(err) < tol:
                break
            mujoco.mj_jacSite(self.model, scratch, jacp, jacr, self.ee_site_id)
            J = jacp[:, :4]
            lam_sq = damping * damping * np.eye(3)
            dq = J.T @ np.linalg.solve(J @ J.T + lam_sq, err)
            scratch.qpos[:4] += dq
            for ctrl_idx in range(4):
                lo, hi = self.joint_limits[ctrl_idx]
                scratch.qpos[ctrl_idx] = max(lo, min(hi, scratch.qpos[ctrl_idx]))

        return np.copy(scratch.qpos[:4]), float(np.linalg.norm(err))

    def cartesian_jog_step(self, target4: np.ndarray, delta_xyz: np.ndarray) -> np.ndarray:
        """One damped-least-squares step converting a small Cartesian
        delta into a joint delta, evaluated at target4's own pose (not the
        live simulated qpos, which lags target under the position-control
        actuators) - same math as solve_ik's inner loop, but applied once
        per tick as velocity control rather than iterated to convergence,
        which is what jogging needs. Clamped to joint_limits, same as
        every other path that can move the arm."""
        scratch = mujoco.MjData(self.model)
        scratch.qpos[:4] = target4
        mujoco.mj_forward(self.model, scratch)
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, scratch, jacp, jacr, self.ee_site_id)
        J = jacp[:, :4]
        lam_sq = CARTESIAN_JOG_DAMPING * CARTESIAN_JOG_DAMPING * np.eye(3)
        dq = J.T @ np.linalg.solve(J @ J.T + lam_sq, delta_xyz)
        new_target = target4 + dq
        for ctrl_idx in range(4):
            lo, hi = self.joint_limits[ctrl_idx]
            new_target[ctrl_idx] = max(lo, min(hi, new_target[ctrl_idx]))
        return new_target

    def _on_workspace_pick(self, world_pt):
        for axis, val in zip(("X", "Y", "Z"), world_pt):
            self.ik_entries[axis].set(f"{val:.3f}")
        self.ik_status_var.set(
            f"Picked from workspace sphere: x={world_pt[0]:.3f} y={world_pt[1]:.3f} z={world_pt[2]:.3f} "
            f"- click Solve & Move to go there.")

    def on_solve_ik(self):
        try:
            target_pos = np.array([float(self.ik_entries[axis].get()) for axis in ("X", "Y", "Z")])
        except ValueError:
            self.ik_status_var.set("Enter numeric X, Y, Z values (meters)")
            return
        threading.Thread(target=self._solve_ik_worker, args=(target_pos,), daemon=True).start()

    def _solve_ik_worker(self, target_pos):
        solution, residual = self.solve_ik(target_pos)
        if residual > 0.01:
            self.root.after(0, lambda: self.ik_status_var.set(
                f"Target likely unreachable within joint limits (residual {residual*1000:.1f} mm) - "
                f"moving as close as possible."))
        else:
            self.root.after(0, lambda: self.ik_status_var.set(f"Solved (residual {residual*1000:.2f} mm)"))
        with self.lock:
            self.target[0:4] = solution
        self.root.after(0, self._refresh_sliders_from_target)

    def _on_glfw_key(self, keycode):
        # Fires when the MuJoCo viewer window itself has OS focus (it's a
        # separate native window from the Tk control panel, so Tk's
        # bind_all never sees these). GLFW only reports key-down, so this
        # jogs by a fixed step per press/repeat rather than tracking hold state.
        if self.mirror_mode or self.recording or self.playing:
            return
        try:
            key = chr(keycode).lower()
        except ValueError:
            return

        if self.cartesian_jog_enabled:
            if key not in CARTESIAN_KEY_BINDINGS:
                return
            axis, direction = CARTESIAN_KEY_BINDINGS[key]
            delta = np.zeros(3)
            delta[axis] = direction * CARTESIAN_JOG_STEP
            with self.lock:
                self.target[:4] = self.cartesian_jog_step(self.target[:4].copy(), delta)
            self.root.after(0, self._refresh_sliders_from_target)
            return

        if key not in KEY_BINDINGS:
            return
        ctrl_idx, direction = KEY_BINDINGS[key]
        lo, hi = self.joint_limits[ctrl_idx]
        with self.lock:
            self.target[ctrl_idx] += direction * JOG_STEP[ctrl_idx]
            self.target[ctrl_idx] = max(lo, min(hi, self.target[ctrl_idx]))
        self.root.after(0, self._refresh_sliders_from_target)

    def _sim_loop(self):
        """Motion + rendering only - deliberately contains NO serial I/O.

        Serial calls block for milliseconds, and while they lived in this
        loop they stalled it 30x/second. Measured: p50 2.07 ms but p99
        5-8 ms, i.e. a 2.5-4x periodic hitch at exactly the hardware rate -
        which is what "moving with breakers in between" feels like. All
        hardware traffic now runs on its own thread, see _hw_loop.
        """
        dt = self.model.opt.timestep
        self.model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
        pick_tick_accum = 0.0
        pick_period = 1.0 / 15.0  # workspace-sphere pick poll at 15 Hz
        last_t = time.perf_counter()

        viewer_ctx = (mujoco.viewer.launch_passive(
                          self.model, self.data, key_callback=self._on_glfw_key)
                      if OPTS.viewer else _NullViewer(self.stop_event))
        with viewer_ctx as viewer:
            while viewer.is_running() and not self.stop_event.is_set():
                # Integrate jog motion against the WALL CLOCK rather than the
                # sim timestep: the loop assumed 2.0 ms per iteration while
                # actually taking 2.3-2.5 ms, so every jog ran at only 81-88%
                # of its commanded speed and drifted with system load.
                now = time.perf_counter()
                elapsed = min(now - last_t, 0.05)  # cap, so a stall can't fling the arm
                last_t = now

                # --- Workspace-sphere click pick ----------------------------
                # MuJoCo's viewer handles body selection/dragging internally
                # (ctrl+double-click-drag is its convention - see the
                # viewer's own on-screen help for the exact gesture) and
                # exposes what's selected via viewer.perturb; it does NOT
                # apply that as a physical force in passive mode unless we
                # ask it to, so reading it here is purely a coordinate pick,
                # not a real interaction with the arm.
                pick_tick_accum += elapsed
                if pick_tick_accum >= pick_period:
                    pick_tick_accum = 0.0
                    if (viewer.perturb.select == self.workspace_body_id
                            and viewer.perturb.active
                            & (mujoco.mjtPertBit.mjPERT_TRANSLATE | mujoco.mjtPertBit.mjPERT_ROTATE)):
                        xmat = self.data.xmat[self.workspace_body_id].reshape(3, 3)
                        xpos = self.data.xpos[self.workspace_body_id]
                        world_pt = xpos + xmat @ viewer.perturb.localpos
                        self.root.after(0, lambda p=np.copy(world_pt): self._on_workspace_pick(p))

                # --- Gamepad poll --------------------------------------------
                # pygame calls happen outside self.lock (unrelated to what it
                # protects); only the resulting target mutation below needs it.
                gp_axes = gp_buttons = None
                if self.gamepad is not None:
                    pygame.event.pump()
                    gp_axes = [self.gamepad.get_axis(i) for i in range(self.gamepad.get_numaxes())]
                    gp_buttons = [self.gamepad.get_button(i) for i in range(self.gamepad.get_numbuttons())]
                    with self.lock:
                        self.gamepad_axes_snapshot = gp_axes
                        self.gamepad_buttons_snapshot = gp_buttons

                # --- Gamepad Y -> Home position ------------------------------
                # Edge-triggered, so holding Y fires once instead of restarting
                # the 2.5s ramp every frame. Dispatched through root.after like
                # every other cross-thread call here, and deliberately OUTSIDE
                # self.lock: _go_home_worker takes that lock itself, so
                # triggering while holding it would make the new thread wait on
                # a lock this loop still owns. on_go_home() applies the same
                # record/play/homing guard as the Home button.
                if gp_buttons is None:
                    self.gamepad_home_was_pressed = False
                else:
                    home_pressed = bool(len(gp_buttons) > GAMEPAD_BTN_HOME
                                        and gp_buttons[GAMEPAD_BTN_HOME])
                    if (home_pressed and not self.gamepad_home_was_pressed
                            and self.gamepad_enabled):
                        self.root.after(0, self.on_go_home)
                    self.gamepad_home_was_pressed = home_pressed

                with self.lock:
                    jog_allowed = not self.mirror_mode and not self.recording and not self.playing and not self.homing
                    if self.keys_held and jog_allowed:
                        if self.cartesian_jog_enabled:
                            delta = np.zeros(3)
                            for key in self.keys_held:
                                if key in CARTESIAN_KEY_BINDINGS:
                                    axis, direction = CARTESIAN_KEY_BINDINGS[key]
                                    delta[axis] += direction * CARTESIAN_JOG_RATE * elapsed
                            if np.any(delta):
                                self.target[:4] = self.cartesian_jog_step(self.target[:4].copy(), delta)
                        else:
                            for key in self.keys_held:
                                if key in KEY_BINDINGS:
                                    ctrl_idx, direction = KEY_BINDINGS[key]
                                    lo, hi = self.joint_limits[ctrl_idx]
                                    self.target[ctrl_idx] += direction * JOINT_SPEED[ctrl_idx] * elapsed
                                    self.target[ctrl_idx] = max(lo, min(hi, self.target[ctrl_idx]))

                    if self.gamepad_enabled and gp_axes is not None and jog_allowed:
                        lx = gamepad_axis(gp_axes,GAMEPAD_AXIS_LEFT_X)
                        ly = gamepad_axis(gp_axes,GAMEPAD_AXIS_LEFT_Y)
                        rx = gamepad_axis(gp_axes,GAMEPAD_AXIS_RIGHT_X)
                        ry = gamepad_axis(gp_axes,GAMEPAD_AXIS_RIGHT_Y)
                        if self.cartesian_jog_enabled:
                            # Left stick = horizontal plane (X/Y), right stick
                            # vertical (Z) - stick "up" (negative raw axis) is
                            # +X / +Z, matching typical flight-stick intuition.
                            delta = np.array([-ly, lx, -ry]) * CARTESIAN_JOG_RATE * elapsed
                            if np.any(delta):
                                self.target[:4] = self.cartesian_jog_step(self.target[:4].copy(), delta)
                        else:
                            for ctrl_idx, stick_val in ((0, -lx), (1, -ly), (2, -ry), (3, rx)):
                                if stick_val == 0.0:
                                    continue
                                lo, hi = self.joint_limits[ctrl_idx]
                                self.target[ctrl_idx] += stick_val * GAMEPAD_JOINT_RATE * elapsed
                                self.target[ctrl_idx] = max(lo, min(hi, self.target[ctrl_idx]))
                        gripper_dir = 0
                        if gp_buttons and len(gp_buttons) > GAMEPAD_BTN_GRIPPER_CLOSE and gp_buttons[GAMEPAD_BTN_GRIPPER_CLOSE]:
                            gripper_dir -= 1
                        if gp_buttons and len(gp_buttons) > GAMEPAD_BTN_GRIPPER_OPEN and gp_buttons[GAMEPAD_BTN_GRIPPER_OPEN]:
                            gripper_dir += 1
                        if gripper_dir:
                            lo, hi = self.joint_limits[4]
                            self.target[4] += gripper_dir * GAMEPAD_GRIPPER_RATE * elapsed
                            self.target[4] = max(lo, min(hi, self.target[4]))

                    # gripper_right_joint mirrors gripper_left_joint (the URDF's
                    # <mimic> tag is dropped by MuJoCo's URDF importer, so this
                    # has to be enforced manually every tick).
                    self.target[5] = self.target[4]
                    target_snapshot = np.copy(self.target)

                clipped = np.clip(target_snapshot, self.model.jnt_range[:6, 0], self.model.jnt_range[:6, 1])
                self.data.ctrl[:6] = clipped
                mujoco.mj_step(self.model, self.data)
                viewer.sync()

                time.sleep(dt)

    def _hw_loop(self):
        """All serial traffic, on its own thread at a steady rate.

        Split out of _sim_loop because serial calls block for milliseconds:
        running them inline made the motion loop hitch 30x/second (p99 rose
        from ~2 ms to 5-8 ms), which is felt as stuttering while jogging.
        Out here a slow bus transaction delays only the next hardware
        update, never the motion integration or the render."""
        hw_period = 1.0 / HW_FEEDBACK_RATE
        record_period = 1.0 / RECORD_RATE
        err_period = 1.0
        last_err = time.perf_counter()

        while not self.stop_event.is_set():
            cycle_start = time.perf_counter()

            # `calibrating` holds io_lock for its whole sweep; skipping here
            # keeps this thread from queueing up behind it.
            if self.hw.connected and not self.calibrating:
                ticks = self.hw.read_all_ticks_fast()
                if ticks:
                    complete = all(i in ticks for i in DXL_IDS)
                    with self.lock:
                        self.present_ticks = ticks
                        # Mirror mode and recording both make the physical
                        # arm the source of truth: it drives the twin's
                        # targets rather than the reverse.
                        if (self.mirror_mode or self.recording) and complete:
                            self._sync_targets_from_ticks_locked(ticks)
                            if self.recording:
                                stamp = time.time() - self.record_started_at
                                pose = np.copy(self.target)
                                # Only keep frames where something actually
                                # moved, so holding still doesn't bloat it.
                                if (not self.frames or
                                        np.max(np.abs(pose - self.frames[-1][1])) > RECORD_MIN_DELTA):
                                    self.frames.append((stamp, pose))

                # Don't write goals in mirror/record (the robot is driving
                # the twin there - writing back would fight the operator).
                if (self.hw.torque_on and not self.mirror_mode and not self.recording):
                    with self.lock:
                        snapshot = np.copy(self.target)
                    clipped = np.clip(snapshot, self.model.jnt_range[:6, 0], self.model.jnt_range[:6, 1])
                    id_to_tick = {}
                    for label, ctrl_idx, dxl_id, lo, hi, kind in JOINT_SPECS:
                        val = clipped[ctrl_idx]
                        if kind == "gripper":
                            id_to_tick[dxl_id] = gripper_to_tick(
                                val, lo, hi, self.gripper_tick_at_lo, self.gripper_tick_at_hi)
                        else:
                            id_to_tick[dxl_id] = rad_to_tick(val)
                    self.hw.sync_write_ticks(id_to_tick)

                if time.perf_counter() - last_err >= err_period:
                    last_err = time.perf_counter()
                    for dxl_id, err_byte in self.hw.check_hardware_errors().items():
                        desc = decode_hw_error(err_byte)
                        self.set_status(f"ID {dxl_id} hardware error: {desc} - auto-recovering...")
                        self.hw.recover(dxl_id)
                        self.set_status(f"ID {dxl_id} recovered from {desc}")

            # Pace by how long the cycle actually took, so bus latency
            # doesn't compound into an ever-slower update rate.
            period = record_period if self.recording else hw_period
            remaining = period - (time.perf_counter() - cycle_start)
            time.sleep(remaining if remaining > 0 else 0.001)

    def on_close(self):
        self.stop_event.set()
        if getattr(self, "web", None) is not None:
            self.web.shutdown()
        self.hw.disconnect()
        pygame.joystick.quit()
        pygame.quit()
        self.root.destroy()


def _parse_args(argv):
    import argparse
    ap = argparse.ArgumentParser(
        prog="digital_twin_gui.py",
        description="OpenManipulator-X digital twin: MuJoCo sim, desktop panel "
                    "and mobile web control, all driving one arm.")
    ap.add_argument("--no-viewer", action="store_true",
                    help="don't open the MuJoCo 3D window (saves a lot of CPU "
                         "on a Raspberry Pi, which has no usable GL driver for it)")
    ap.add_argument("--no-panel", action="store_true",
                    help="hide the Tk control panel window; the app still runs "
                         "and the web panel still works")
    ap.add_argument("--headless", action="store_true",
                    help="shorthand for --no-viewer --no-panel: web control only. "
                         "Still needs an X display for Tk's event loop - run it "
                         "under Xvfb (the Pi installer sets this up).")
    ap.add_argument("--no-web", action="store_true",
                    help="don't serve the mobile web panel")
    ap.add_argument("--port", type=int, default=None, help="web panel port (default 8080)")
    ap.add_argument("--bind", default=None,
                    help="web panel bind address (default 0.0.0.0; use 127.0.0.1 "
                         "to accept only local/tunnelled connections)")
    return ap.parse_args(argv)


if __name__ == "__main__":
    args = _parse_args(sys.argv[1:])
    OPTS.viewer = not (args.no_viewer or args.headless)
    OPTS.panel = not (args.no_panel or args.headless)
    OPTS.web = not args.no_web
    OPTS.port = args.port
    OPTS.bind = args.bind

    root = tk.Tk()
    if not OPTS.panel:
        # Withdrawn, not destroyed: every status update and cross-thread call
        # in this app is scheduled through root.after(), so the Tk event loop
        # has to keep running even when nobody is looking at the window.
        root.withdraw()
    app = DigitalTwinApp(root)
    root.mainloop()
