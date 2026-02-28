# OpenManipulator-X MuJoCo Simulation: From Manual Control to Full Autonomy

A comprehensive MuJoCo-based physics simulation environments for the OpenManipulator-X robotic arm.

Currently, the project is in **Phase 1**, focusing on robust real-time manual control using a game controller.

## 🚀 Project Roadmap & Goals

The ultimate objective of this simulation is to serve as a testing ground for advanced robotic autonomy. The planned roadmap includes:

- **[CURRENT] Phase 1: Manual Teleoperation**
  - Interactive Pygame-based controller mapping.
  - Real-time manipulation of a 4-DOF arm (Base, Shoulder, Elbow, Wrist) plus gripper.
  - Easy conversion from standard URDF to MuJoCo's native XML.
- **[PLANNED] Phase 2: Computer Vision (CV) Integration**
  - Camera sensor integration within MuJoCo.
  - Object detection, pose estimation, and workspace understanding using OpenCV/Deep Learning.
- **[PLANNED] Phase 3: Autonomous Path Planning & Obstacle Avoidance**
  - Intelligent trajectory generation in complex, dynamic environments.
  - Collision detection and avoidance algorithms.
- **[PLANNED] Phase 4: Autonomous Pick-and-Place & Advanced AI**
  - AI-driven manipulation and grasping of dynamic objects.
  - Reinforcement Learning (RL) agents for complex, unscripted tasks.

---

## Prerequisites

1. Python 3.8+
2. An Xbox Controller (or compatible gamepad) connected to your PC.

### Installation

Install the required Python packages using `pip`:

```bash
pip install -r requirements.txt
```

_(Alternatively, you can install them manually using: `pip install mujoco pygame numpy`)_

---

## Project Structure

- `open_manipulator_x.urdf`: The original URDF definition file of the robot.
- `urdf-xml.py`: Script to convert the URDF to MuJoCo's native XML format.
- `Controller/controller_test.py`: Utility to test gamepad connectivity and mapping.
- `Controller/controller_cntroll.py`: Main simulation and control script.
- `STL/` / `meshes/`: Contain the 3D mesh geometry components for the robot.

---

## Usage Guide

### 1. Generate MuJoCo XML

Before running the simulation, you must convert the `.urdf` model into MuJoCo's native `.xml` format so that `controller_cntroll.py` can load it.

Run the conversion script from the project root:

```bash
python urdf-xml.py
```

_This will generate `open_manipulator_x.xml` in your working directory._

### 2. Test Your Controller (Optional)

If you want to ensure your controller is properly recognized by Pygame and verify its axis/button layouts, run the test script:

```bash
python Controller/controller_test.py
```

Move your thumbsticks and press buttons to see their output values mapped in the terminal. Press `Ctrl+C` to exit.

### 3. Run the Simulation

Ensure your controller is connected, and then launch the main simulation script.

**Note**: The main control script `Controller/controller_cntroll.py` expects `open_manipulator_x.xml` to be present in the directory you are executing from. So ensure you run it from the root folder:

```bash
python Controller/controller_cntroll.py
```

---

## 🎮 Gamepad Controls

| Input                               | Action         |
| :---------------------------------- | :------------- |
| **Left Stick X-Axis (Horizontal)**  | Base Yaw       |
| **Left Stick Y-Axis (Vertical)**    | Shoulder Pitch |
| **Right Stick Y-Axis (Vertical)**   | Elbow Pitch    |
| **Right Stick X-Axis (Horizontal)** | Wrist Pitch    |
| **Button A**                        | Close Gripper  |
| **Button B**                        | Open Gripper   |

> **Note:** Different controllers may output axes differently. If the arm joints move on the wrong axis, use `controller_test.py` to check the actual axis indices being triggered by your actions, and update the bindings accordingly in `Controller/controller_cntroll.py` (lines `67-78`).
