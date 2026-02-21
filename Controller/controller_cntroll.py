import mujoco
import mujoco.viewer
import time
import pygame
import numpy as np
import sys

# ==========================================
# 1. INITIALIZE XBOX CONTROLLER
# ==========================================
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("❌ No controller detected. Connect one and restart.")
    sys.exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"✅ Controller detected: {joystick.get_name()}")
print("🎮 Controls:")
print("   Left Stick: Base Yaw & Shoulder Pitch")
print("   Right Stick: Elbow & Wrist Pitch")
print("   A/B Buttons: Gripper Close/Open")

# ==========================================
# 2. LOAD MUJOCO XML
# ==========================================
try:
    # Make sure we are loading the compiled XML with the <actuator> tags!
    model = mujoco.MjModel.from_xml_path("open_manipulator_x.xml")
    data = mujoco.MjData(model)
except Exception as e:
    print(f"Error loading XML: {e}")
    sys.exit()

if model.nu == 0:
    print("⚠️ ERROR: No actuators found in the XML!")
    sys.exit()

# Disable self-collisions during testing so it doesn't glitch
model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT

# Initialize targets to the default resting pose
mujoco.mj_forward(model, data)
target_q = np.copy(data.qpos[:6]) 

# Helper to ignore slight stick drift
def deadzone(val, thresh=0.2):
    return val if abs(val) > thresh else 0.0

# ==========================================
# 3. MAIN SIMULATION LOOP
# ==========================================
with mujoco.viewer.launch_passive(model, data) as viewer:
    
    dt = model.opt.timestep
    speed = 1.5 # How fast the joints move (radians per second)
    
    while viewer.is_running():
        pygame.event.pump() 
        
        # --- THUMBSTICK MAPPINGS ---
        # Note: If your arm moves the wrong joint, swap the axis numbers (0, 1, 3, 4)
        # based on what your test script printed out earlier.
        
        # Left Stick X (Axis 0) -> Joint 1 (Base)
        target_q[0] -= deadzone(joystick.get_axis(0)) * speed * dt
        
        # Left Stick Y (Axis 1) -> Joint 2 (Shoulder)
        target_q[1] -= deadzone(joystick.get_axis(1)) * speed * dt
        
        # Right Stick Y (Axis 4) -> Joint 3 (Elbow - The center one!)
        target_q[2] -= deadzone(joystick.get_axis(4)) * speed * dt
        
        # Right Stick X (Axis 3) -> Joint 4 (Wrist)
        target_q[3] -= deadzone(joystick.get_axis(3)) * speed * dt
        
        
        # --- GRIPPER CONTROLS ---
        if joystick.get_button(0):   # Button A (Close)
            target_q[4], target_q[5] = -0.01, -0.01
        elif joystick.get_button(1): # Button B (Open)
            target_q[4], target_q[5] = 0.01, 0.01

        # Prevent the targets from going past the robot's physical limits
        target_q = np.clip(target_q, model.jnt_range[:6, 0], model.jnt_range[:6, 1])

        # Pass the desired angles to MuJoCo's built-in actuators
        data.ctrl[:6] = target_q
        
        # Step the physics engine and update the viewer
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(dt)