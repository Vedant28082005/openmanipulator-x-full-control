# OpenManipulator-X — Hardware Integration Context

Context document for adapting a simulation-only repo (vision / VLA agents) to
drive **real** OpenManipulator-X hardware. This describes a working, debugged
hardware control stack: what exists, how to hook into it, and — most
importantly — the non-obvious hardware behaviours that cost real debugging
time. Read "Hard-won gotchas" before writing any motor code.

Reference implementation: `Controller/digital_twin_gui.py` (~1800 lines,
single file, no ROS dependency).

---

## 1. What exists today

A MuJoCo digital twin kept in sync with the physical arm over U2D2/DYNAMIXEL:

| Capability | Status |
|---|---|
| Bidirectional sync (twin→robot commands, robot→twin mirror) | working |
| Joint-space jog (sliders / keyboard / gamepad) | working |
| Cartesian jog + damped least-squares IK | working |
| Teach-by-demonstration record & playback (JSON) | working |
| Auto-recovery from Overload/hardware errors | working |
| Hardware-measured joint limits | working |
| Live PID / motion-profile tuning (no restart) | working |
| Home position with smooth approach | working |

No ROS. Pure `dynamixel-sdk` + `mujoco` + `numpy` + `pygame` + `tkinter`.

---

## 2. Hardware facts

```
Port      /dev/ttyUSB0     (U2D2; Linux — user must be in `dialout` group)
Baudrate  1_000_000
Protocol  2.0
Motors    5x XM430-W350
```

| ID | Joint | Limits | Control mode |
|----|-------|--------|--------------|
| 11 | joint1 base yaw   | ±3.14159 rad | Position (3) |
| 12 | joint2 shoulder   | ±1.5 rad     | Position (3) |
| 13 | joint3 elbow      | −1.5 … 1.4 rad | Position (3) |
| 14 | joint4 wrist      | −1.7 … 1.97 rad | Position (3) |
| 15 | gripper           | −0.010 … 0.010 m | **Current-based Position (5)** |

**Only one process can hold `/dev/ttyUSB0`.** DYNAMIXEL Wizard and this app
cannot both be connected — there is an explicit Disconnect that frees the port.

### Unit conversions

```python
TICKS_PER_REV = 4096
CENTER_TICK   = 2048          # 0 rad for the ARM joints

rad_to_tick(r) = round(2048 + r * 4096/(2*pi))
tick_to_rad(t) = (t - 2048) * 2*pi/4096
```

**The gripper does NOT use that mapping.** See gotcha #2.

---

## 3. Control architecture — the part that matters most

Three threads. **This separation is load-bearing, not stylistic.**

```
Tk main thread   — UI only. Never touches serial directly; workers + root.after().
_sim_loop        — MuJoCo physics, rendering, input→target integration.
                   CONTAINS ZERO SERIAL I/O. This is enforced deliberately.
_hw_loop         — ALL serial traffic @30 Hz: feedback read, goal write,
                   error poll @1 Hz.
```

**Why:** serial calls block for milliseconds. When they lived in the motion
loop, they stalled it 30×/second — measured p50 2.07 ms but **p99 5–8 ms**, a
2.5–4× periodic hitch that is plainly felt as stuttering during teleop.
Moving serial to its own thread fixed it. If your VLA policy does inference
in-loop, **give it its own thread too** — anything that can block for
milliseconds must not sit in the motion path.

Two locks, do not conflate:
- `app.lock` — guards `target`, `present_ticks`, `frames`, mode flags.
- `HardwareLink.io_lock` — serialises **every** serial transaction. pyserial
  is not concurrency-safe; without this, concurrent UI actions (Diagnose,
  Calibrate) corrupt reads from the polling loop. Every method touching the
  port holds it.

### Integration point for a policy / VLA agent

`self.target` — a **6-vector** — is the single source of truth for commanded
pose. Everything (sliders, keyboard, gamepad, IK, playback, homing) writes
only here; `_hw_loop` converts it to ticks and streams it.

```
target[0:4]  joint1..joint4      radians
target[4]    gripper_left        meters
target[5]    gripper_right       meters — MUST mirror target[4] (see gotcha #6)
```

To drive the arm from a policy:

```python
with app.lock:
    app.target[0:4] = predicted_joint_angles     # radians
    app.target[4]   = predicted_gripper_opening  # meters
    app.target[5]   = app.target[4]              # mandatory mirror
```

Observations:
```python
with app.lock:
    ticks = dict(app.present_ticks)   # {dxl_id: raw_tick}, refreshed @30 Hz
# app.data.qpos / app.data.site_xpos[app.ee_site_id] -> simulated state / EE pose
```

IK is available directly: `app.solve_ik(np.array([x,y,z]))` returns
`(joint_angles[4], residual_metres)`. Residual > ~0.01 means unreachable —
**check it**, the solver returns a best-effort pose rather than failing.

---

## 4. Safety invariants — preserve these

These were each written in response to a real failure. Removing them
reintroduces the failure.

1. **Never enable torque against a stale target.** Always read present
   positions and set `target` from them *before* energising. Otherwise the
   arm lunges from wherever it physically is to whatever the software last
   thought — potentially into a hard stop.
2. **Never jump to a distant pose.** Homing and playback both ease in over
   ~2.5 s with a smoothstep ramp. A VLA emitting large per-step deltas should
   do the same, or rate-limit.
3. **One owner at a time.** Record / playback / homing take exclusive control
   and lock out mirror, jog, gamepad, sliders, manual torque. If two sources
   write `target` concurrently they fight. A policy needs the same mutex.
4. **E-STOP always reachable** — never disabled by any mode.
5. **Everything clamps to `joint_limits`**, which are intersected with the
   motors' own EEPROM limits at connect (see gotcha #4).
6. **Torque off drops the arm.** Any automatic torque-disable must be
   confirmed by a human, not silent.

---

## 5. Hard-won gotchas

These are non-obvious, cost significant debugging, and will bite any hardware
port.

**1. MuJoCo's URDF importer silently drops things.** `<actuator>`, the skybox
asset, and any fixed massless link are all discarded on URDF→XML conversion.
`urdf-xml.py` re-injects actuators, the `end_effector` IK site, the workspace
sphere, and `<option integrator="implicitfast">` as a post-processing step.
**Regenerate with `python urdf-xml.py`; never hand-edit the XML.**

**2. The gripper's tick mapping is NOT the arm formula.** ROBOTIS's
`open_manipulator_libs` drives it through a linear coefficient
(`joint_rad = metres / -0.015`) assuming tick 2048 = gripper zero — which only
holds if the servo horn was indexed there. **It was not on this unit.** The
working values were obtained by physically jogging to each stop in DYNAMIXEL
Wizard and reading the position:

```
GRIPPER_DEG_CLOSED = 125.2   -> tick 1424
GRIPPER_DEG_OPEN   = 270.4   -> tick 3077     (~145 deg of real travel)
```

**Re-measure these for any other physical unit.** There is also an automatic
`find_gripper_stops()` sweep that measures them by gently stalling into each
stop. A wrong mapping here commands the gripper past its mechanical stop,
which is what caused repeated Overload trips.

**3. Overload (error bit 5) does not self-clear.** A position-mode DYNAMIXEL
driven at an unreachable goal trips Overload and **stays tripped until
rebooted**, even after the load is removed. `check_hardware_errors()` polls
register 70 at 1 Hz and `recover()` auto-reboots + restores settings.
Mitigation: the gripper runs **Current-based Position Control (mode 5)** with
`Goal Current = 200` (ROBOTIS's own value), so hitting an obstruction stalls
softly instead of driving at full PWM.

**4. Trust the motors' limits over the model's.** Each arm motor's EEPROM
Min/Max Position Limit (regs 48/52) is read at connect and intersected with
the URDF range. The model file is not authoritative about the physical build.

**5. RAM registers reset on reboot.** Position P/I/D gains and profile
velocity/acceleration are RAM. After any Overload reboot they revert to
factory — `recover()` must re-apply them or the tuning silently vanishes.

**6. `<mimic>` is not honoured.** The URDF mimics gripper_right→gripper_left,
but MuJoCo drops it. `target[5] = target[4]` must be enforced **every tick**
or only one finger moves.

**7. Motion-profile velocity must match your command rate.** With streamed
goal positions, motion is continuous only while the servo is *still moving*
when the next goal arrives. Measured against a 1.5 rad/s jog:

```
PROFILE_VELOCITY  40 = 0.96 rad/s (0.64x) -> lags, re-accelerates constantly
PROFILE_VELOCITY  70 = 1.68 rad/s (1.12x) -> continuous          <- current
PROFILE_VELOCITY 150 = 3.60 rad/s (2.40x) -> finishes early, sits idle,
                                             stop/start micro-stutter
```
**If your policy commands at a different rate, recompute this.**
`raw = (rad/s) * 60/(2*pi) / 0.229`

**8. Integrate motion against the wall clock, not a fixed timestep.** The loop
assumed 2.0 ms/iteration while actually taking 2.3–2.5 ms → every jog ran at
**81–88% of commanded speed**, drifting with system load.

**9. Don't re-send unchanged goals.** Each Goal Position write restarts the
servo's trajectory generator. A stationary arm re-writing the same goal 30×/s
is telling it to re-plan a move to where it already is. Goals are only sent
on change; the cache is invalidated on torque-cycle, reboot and disconnect.

**10. Undamped position loop.** XM430 defaults are P=800, I=0, **D=0** — no
damping — and ROBOTIS's own firmware never overrides them. Loaded joints
(11 base, 12 shoulder) hunt around their goal; the unloaded wrist does not.
Current defaults P=800 / D=1000, **live-tunable** via the GUI panel (RAM
registers, applies instantly with torque on).

---

## 6. Simulation-side notes

- **Actuators:** `<position kp="50" kv="2">` on all 6 joints. `kv` matters —
  with the default `kv=0` the position servo is an undamped spring and the
  sim visibly vibrates even at rest (measured: joint velocity never settled).
- **Integrator:** `implicitfast`. Adding `kv` under the default Euler
  integrator made the sim numerically explode; `implicitfast` integrates the
  damping term implicitly and is stable. Verified: settle velocity ~1e-10,
  step overshoot 0.37 rad → 0.01 rad.
- **Contacts are disabled** (`mjDSBL_CONTACT`). There is **no collision
  checking** — the sim will happily pass through itself and any object.
  **A vision/VLA pipeline that reasons about grasping or obstacles must add
  this back**, at minimum as warnings.
- **IK:** damped least-squares on the `end_effector` site Jacobian, solved on
  scratch `MjData` so it never perturbs live state, clamped to `joint_limits`
  each iteration. EE at home ≈ `(0.286, 0, 0.1875)` m; max reach ≈ 0.44 m
  from the base yaw axis at `(0.012, 0, 0)`.

---

## 7. Known open issues

- **Residual at-rest vibration on IDs 11/12.** Small amplitude (±1–2 ticks
  ≈ 0.09°) at ≥10 Hz. Root-caused to position-loop damping; awaiting live
  P/D tuning on hardware. IDs 13/14 are unaffected.
- **Gamepad axis indices unverified.** `GAMEPAD_AXIS_*` / `GAMEPAD_BTN_*` are
  the standard Xbox/SDL2 layout but were never tested against a physical pad.
  The GUI shows a live raw axis/button readout for remapping.
- **Workspace-sphere click gesture unverified** — relies on MuJoCo's built-in
  object selection; exact gesture is viewer-version-dependent.
- **`HOME_POSITION` is unit-specific**: `[1.589, 0.330, −0.107, 1.663, 0.0066]`
  — re-measure per arm.
- **Uncommitted work.** The pushed commit predates the threading split, PID
  gains, live tuning panel, home button, base-direction fix and profile
  retune. Commit before relying on the remote.

---

## 8. Recommended path for a VLA / vision integration

1. **Reuse `HardwareLink` as-is.** It encapsulates the entire DYNAMIXEL
   protocol surface, thread-safely, with recovery. Don't reimplement it.
2. **Run policy inference on its own thread**, writing `app.target` under
   `app.lock`. Never inference inside the motion loop (gotcha, §3).
3. **Add a policy mode to `_set_teach_mode_controls()`** so it participates in
   the existing one-owner-at-a-time mutex rather than fighting teleop.
4. **Rate-limit or ramp policy output** — see safety invariant #2.
5. **Re-enable contacts** before trusting grasp/obstacle reasoning (§6).
6. **Teleop and policy compose well:** record/playback already provides
   demonstration collection, and mirror mode gives a hand-guided data path —
   both usable for imitation-learning datasets, writing to the same `target`
   representation the policy will output.
