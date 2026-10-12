# Piper Robotic Arm MuJoCo Model

[MuJoCo](https://mujoco.org/) simulation model for the Piper 6-DOF robotic arm, extracted and refactored from the `piper_ros` ROS package.

## Model Source

Kinematic and inertial parameters are derived from the URDF/XACRO files in `piper_ros/src/piper_description/`. Mesh files are in their original STL format.

## File Structure

| File | Description |
|---|---|
| `piper.xml` | Complete model with gripper (6 revolute joints + 2 prismatic gripper joints), torque control mode |
| `piper_nohand.xml` | 6-DOF model without gripper, torque control mode |
| `scene.xml` | Simulation scene (including `piper.xml` + ground + lighting + skybox) |
| `assets/` | STL mesh files directory |

## Control Mode

All models use **pure torque control** (`motor` actuator), where the `ctrl` signal directly maps to joint torque τ:

- **Joint 1-6** (revolute joints): Torque range ±100 Nm
- **Gripper** (in `piper.xml`): Two fingers synchronized via tendon, force range ±10 N

## Quick Start

```python
import mujoco

# Load model
model = mujoco.MjModel.from_xml_path("piper/scene.xml")
data = mujoco.MjData(model)

# Reset to home keyframe
mujoco.mj_resetDataKeyframe(model, data, 0)

# Simulation step
mujoco.mj_step(model, data)
```

```bash
# Use MuJoCo built-in viewer
python3 -m mujoco.viewer --mjcf piper/scene.xml
```

## License

Mesh files originate from the Piper robotic arm ROS package by [AgileX Robotics](https://github.com/agilexrobotics).
