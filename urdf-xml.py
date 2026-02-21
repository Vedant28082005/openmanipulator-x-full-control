import mujoco

# 1. Load your working URDF
# (MuJoCo automatically parses and compiles it internally)
model = mujoco.MjModel.from_xml_path("open_manipulator_x.urdf")

# 2. Save the compiled model as a native MuJoCo XML
mujoco.mj_saveLastXML("open_manipulator_x.xml", model)

print("Conversion complete! Check your folder for open_manipulator_x.xml")