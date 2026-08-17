import mujoco

# 1. Load your working URDF
# (MuJoCo automatically parses and compiles it internally)
model = mujoco.MjModel.from_xml_path("open_manipulator_x.urdf")

# 2. Save the compiled model as a native MuJoCo XML
mujoco.mj_saveLastXML("open_manipulator_x.xml", model)

# 3. Inject position actuators for the 6 controllable joints.
# MuJoCo's URDF <mujoco> extension block doesn't support <actuator>,
# so the saved XML has none - add them here as a post-processing step.
#
# kv="2" (velocity/damping gain) matters more than it looks: kp="50" alone
# is an UNDAMPED spring (kv defaults to 0). With this arm's light link
# inertias that spring never truly settles - it was still oscillating at
# ~0.05 rad/s in simulated qvel even holding perfectly still, which is the
# visible vibration. Verified headless: kv=2 (with the implicitfast
# integrator below) drops that to ~0 and cuts step-response overshoot from
# 0.37 rad to 0.01 rad.
ACTUATOR_XML = """  <actuator>
    <position name="act_joint1" joint="joint1" kp="50" kv="2" ctrlrange="-3.14159 3.14159"/>
    <position name="act_joint2" joint="joint2" kp="50" kv="2" ctrlrange="-1.5 1.5"/>
    <position name="act_joint3" joint="joint3" kp="50" kv="2" ctrlrange="-1.5 1.4"/>
    <position name="act_joint4" joint="joint4" kp="50" kv="2" ctrlrange="-1.7 1.97"/>
    <position name="act_gripper_left" joint="gripper_left_joint" kp="50" kv="2" ctrlrange="-0.011 0.02"/>
    <position name="act_gripper_right" joint="gripper_right_joint" kp="50" kv="2" ctrlrange="-0.011 0.02"/>
  </actuator>
"""

# implicitfast integrates the actuator's damping term implicitly, which is
# what makes kv above numerically stable at this timestep - adding the same
# kv under the default Euler integrator made it blow up instead of damping
# (verified headless), because kp=50 is very stiff relative to how light
# these links are.
OPTION_XML = """  <option integrator="implicitfast"/>
"""

# mj_saveLastXML silently drops the URDF's <mujoco><asset><texture skybox>
# block, so re-add a plain white skybox here as a post-processing step too.
BACKGROUND_XML = """  <asset>
    <texture type="skybox" builtin="flat" rgb1="1 1 1" rgb2="1 1 1" width="512" height="512"/>
  </asset>
"""

# The URDF's fixed "end_effector_joint" (link5 -> end_effector_link, offset
# 0.126 0 0) gets welded away by the compiler since it's a massless fixed
# link, so there's no body left to target for IK. Add an explicit site at
# the same offset, inside link5's body, to use as the IK target point.
SITE_ANCHOR = '<geom type="mesh" rgba="0.2 0.2 0.2 1" mesh="link5"/>'
SITE_XML = SITE_ANCHOR + '\n            <site name="end_effector" pos="0.126 0 0" size="0.005"/>'

# Workspace sphere: a translucent, non-colliding reference sphere bounding
# the arm's max reach, click-pickable in the 3D viewer (ctrl+double-click
# to select it, then drag - see the viewer's own on-screen control legend
# for the exact gesture, it's version-dependent). Center/radius were found
# by sampling forward kinematics over joint2-4's full range at joint1=0
# (joint1 is a pure yaw about this same axis, so it doesn't change reach):
# max reach from the joint1 axis was 0.4396 m.
WORKSPACE_CENTER = "0.012 0 0"
WORKSPACE_RADIUS = "0.44"
WORKSPACE_ANCHOR = "<worldbody>"
WORKSPACE_XML = (WORKSPACE_ANCHOR +
    f'\n    <body name="workspace_marker" pos="{WORKSPACE_CENTER}">'
    f'\n      <geom name="workspace_sphere" type="sphere" size="{WORKSPACE_RADIUS}" '
    'rgba="0.2 0.6 1 0.12" contype="0" conaffinity="0" group="2"/>'
    '\n    </body>')

with open("open_manipulator_x.xml") as f:
    xml_text = f.read()

if "<actuator>" not in xml_text:
    xml_text = xml_text.replace("</mujoco>", ACTUATOR_XML + "</mujoco>")
if 'integrator="implicitfast"' not in xml_text:
    xml_text = xml_text.replace("</mujoco>", OPTION_XML + "</mujoco>")
if "skybox" not in xml_text:
    xml_text = xml_text.replace("</mujoco>", BACKGROUND_XML + "</mujoco>")
if 'site name="end_effector"' not in xml_text:
    xml_text = xml_text.replace(SITE_ANCHOR, SITE_XML)
if 'name="workspace_marker"' not in xml_text:
    xml_text = xml_text.replace(WORKSPACE_ANCHOR, WORKSPACE_XML, 1)

with open("open_manipulator_x.xml", "w") as f:
    f.write(xml_text)

print("Conversion complete! Check your folder for open_manipulator_x.xml")