# Robot Dynamics Parameter Identification System

This branch is a **pure MuJoCo / pure CMake / ROS-free** experimental version for robot parameter identification, designed to build and run directly on Windows or Linux.

> **Attribution**: This project references the [BIRDy (Benchmark for Identification of Robot Dynamics)](https://github.com/TUM-ICS/BIRDY) open-source project.

---

## Quick Start

### 1. Install Dependencies

The following dependencies need to be installed and configured in advance:

- MuJoCo
- GLFW
- Eigen3
- CMake 3.21+
- C++17 compiler

If MuJoCo is not installed in the default system location, please set the `MUJOCO_DIR` environment variable.

### 2. Configure the Project

```bash
cmake -S . -B build
```

### 3. Build

```bash
cmake --build build --parallel
```

### 4. Run Unified Experiment Entry Point

```bash
./build/run_experiment
```

By default, this will:

- Read [experiment.yaml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/config/experiment.yaml)
- Read [force_controller_node.yaml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/config/force_controller_node.yaml)
- Read [panda_sim_node.yaml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/config/panda_sim_node.yaml)
- Use [scene.xml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/franka_emika_panda/scene.xml)
- Output sampled data to `data/benchmark_data.csv`

By modifying the `robot` and `backend` fields in [experiment.yaml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/config/experiment.yaml), you can switch between the simulation backend and the Piper physical hardware backend. When switching to `piper` with `backend: sim`, the system automatically uses [piper_force_controller_node.yaml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/config/piper_force_controller_node.yaml) and the corresponding Piper MuJoCo model.

Recommended backend switching options:

- `backend: sim`: Use MuJoCo simulation backend
- `backend: piper_real`: Use Piper physical hardware backend

Optional arguments:

```bash
./build/run_experiment --headless --output data/my_run.csv
```

You can also explicitly specify the experiment configuration:

```bash
./build/run_experiment --experiment-config config/experiment.yaml
```

### 5. Run Parameter Identification

```bash
./build/identify
```

By default, this reads [identification.yaml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/config/identification.yaml). The `robot` field can be toggled between `panda` and `piper`. Based on this, `identify` selects the corresponding degrees of freedom, dynamics baseline, and regressor matrix construction method. `panda` uses a 7-axis kinematic chain, while `piper` uses a 6-axis chain.

Optional arguments:

```bash
./build/identify --config config/identification.yaml
./build/identify --robot piper
./build/identify --data-file data/benchmark_data.csv --algorithm 1
./build/identify --data-file data/benchmark_data.csv --algorithm 8
./build/mujoco_identify data/benchmark_data.csv
```

### 6. Run Piper Physical Hardware Experiment

Physical hardware experiments share the same entry executable with simulation experiments, differing only in the backend:

```bash
./build/run_experiment --experiment-config config/experiment.yaml --headless
```

Before running, please confirm:

- CAN environment is set up according to [piper_sdk/README(ZH).MD](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/piper_sdk/README(ZH).MD)
- `python-can` is installed
- `backend: piper_real` is configured in [experiment.yaml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/config/experiment.yaml)
- Check `control_mode`, soft limits, and home position in [config/piper_real_experiment.yaml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/config/piper_real_experiment.yaml)

The default configuration uses `excitation_trajectory`, executing a Fourier excitation trajectory via the MIT interface and recording `time / q / qd / qdd / tau` with the same structure as in simulation.

Before actually executing the excitation trajectory, the bridge runs a "return to safe initial position" routine:

- If `move_to_home_before_start: true`
- And current joint positions are outside the `home_position_tolerance` window
- The bridge first moves the arm to `home_position` at `home_speed_rate`
- Only after entering the tolerance window does the unified experiment main loop begin executing the excitation trajectory

Here, "homing" is practically equivalent to "returning to the configured safe initial pose `home_position`", rather than rewriting the motor zero-point calibration.

Available modes:

- `excitation_trajectory`: Main mode for hardware parameter identification, executing a Fourier excitation trajectory
- `mit_identification`: MIT interface experimental mode identical to excitation trajectory, convenient for future control strategy extensions
- `position_validation`: Small-amplitude joint position trajectory validation
- `state_only`: Connection and sampling only, without issuing motion commands

Excitation trajectory parameters are located in [config/piper_real_experiment.yaml](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/config/piper_real_experiment.yaml):

- `trajectory_period`
- `trajectory_harmonics`
- `trajectory_coefficient_scale`
- `trajectory_seed`

These parameters determine the shape of the Fourier trajectory used in hardware experiments. The current implementation uses a fixed random seed to generate reproducible trajectory coefficients, automatically satisfying the initial position / zero velocity constraints at `t=0`.

The hardware bridge additionally checks:

- `joint_soft_limits`
- `max_command_velocity`
- `max_command_torque`
- `home_position_tolerance`

If any joint command exceeds these thresholds, the bridge rejects the step command and triggers an emergency stop, preventing the upper-level unified main loop from sending abnormal control values directly to the physical arm.

---

## Project Structure

```text
├── franka_emika_panda/   # MuJoCo Panda model and assets
├── src/app/              # Single-process experiment entry points
├── src/sim_com_node/     # Pure C++ MuJoCo simulator
├── src/force_node/       # Pure C++ controller, trajectory generator, collision check
├── src/identification/   # Offline parameter identification and diagnostic tools
├── config/               # Runtime configuration files
└── doc/                  # Mathematical derivations and documentation
```

---

## System Architecture

The current branch adopts a unified experiment scheduling architecture independent of ROS middleware. The experimental pipeline is divided into five tiers: "Configuration -> Control -> Execution Backend -> Data -> Identification".

```text
config/*.yaml
    |
    v
run_experiment
    |
    +--> ForceController --------------------+
    |                                        |
    |                                        | Unified control commands
    |                                        v
    +--> SimulationBackend / PiperHardwareBackend
             |
             | Joint states q / qd
             +---------------------------> ForceController
             |
             | Unified CSV recording
             v
        benchmark_data.csv
             |
             v
     identify / mujoco_identify
             |
             v
   results/identification.yaml
```

- `run_experiment` is the unified experiment scheduling entry point, responsible for loading configurations, creating controllers, selecting execution backends, and driving the single unified time loop.
- `ForceController` handles excitation trajectory generation, PD tracking, torque limiting, and output of unified control commands.
- `SimulationBackend` and `PiperHardwareBackend` handle simulation stepping and physical arm command execution, respectively, without altering the upper experiment workflow.
- `identify` and `mujoco_identify` perform offline identification, loading experimental CSV data, constructing observation matrices, and outputting parameter results.

In terms of runtime sequence, a single-step loop proceeds as follows:

1. Backend returns current joint states.
2. `ForceController` computes unified control commands based on current timestamp and joint states.
3. Backend executes one step of simulation or sends one step of hardware commands.
4. Shared recorder logs `q / qd / qdd / tau` to CSV for downstream identification.

---

## Main Executables

- `run_experiment`: Runs single-process MuJoCo closed-loop experiments and generates CSV data
- `identify`: Loads CSV data and executes a single algorithm or full benchmark
  Supports `NLS_FRICTION` non-linear friction joint identification (`--algorithm 8`)
- `mujoco_identify`: Quick execution of single MuJoCo regressor identification
- `dynamics_diagnostic`: Compares dynamic models against recorded data
- `model_comparison`: Compares different dynamic models
- `regressor_test`: Verifies consistency between regressor matrix and MuJoCo dynamics
