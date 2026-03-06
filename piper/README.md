# Piper 机械臂 MuJoCo 模型

Piper 6-DOF 机械臂的 [MuJoCo](https://mujoco.org/) 仿真模型，从 `piper_ros` ROS 包中提取并重构。

## 模型来源

运动学参数和惯性参数来自 `piper_ros/src/piper_description/` 中的 URDF/XACRO 文件，mesh 文件为原始 STL 格式。

## 文件结构

| 文件 | 说明 |
|---|---|
| `piper.xml` | 含夹爪的完整模型（6 旋转关节 + 2 平移夹爪关节），力矩控制模式 |
| `piper_nohand.xml` | 不含夹爪的 6-DOF 模型，力矩控制模式 |
| `scene.xml` | 仿真场景（包含 `piper.xml` + 地面 + 灯光 + 天空盒） |
| `assets/` | STL 网格文件目录 |

## 控制模式

所有模型使用**纯力矩控制**（`motor` actuator），`ctrl` 信号直接映射为关节力矩 τ：

- **Joint 1-6**（旋转关节）：力矩范围 ±100 Nm
- **夹爪**（`piper.xml` 中）：通过 tendon 同步两个手指，力范围 ±10 N

## 快速使用

```python
import mujoco

# 加载模型
model = mujoco.MjModel.from_xml_path("piper/scene.xml")
data = mujoco.MjData(model)

# 设置到 home 位姿
mujoco.mj_resetDataKeyframe(model, data, 0)

# 仿真步进
mujoco.mj_step(model, data)
```

```bash
# 使用 MuJoCo 自带查看器
python3 -m mujoco.viewer --mjcf piper/scene.xml
```

## 许可证

mesh 文件来源于 [agilex robotics](https://github.com/agilexrobotics) 的 Piper 机械臂 ROS 包。
