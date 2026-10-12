# Piper Physical Arm Parameter Identification Integration Plan

> Objective: Ensure that the MuJoCo simulation data collection workflow and the Piper physical hardware data collection workflow remain fully aligned across six dimensions: "Experiment Entry Point, Excitation Trajectory, Control Loop, Sampling Timing, CSV Fields, Identification Entry Point", allowing differences only at the underlying execution backend layer.

## 1. Background and Current Status

The current primary experimental pipeline of the repository is:

`run_experiment -> ForceController -> PandaSimulator -> benchmark_data.csv -> identify`

Where:

- [src/app/run_experiment.cpp](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/src/app/run_experiment.cpp) reads configuration and drives the main loop.
- [src/force_node/src/force_controller.cpp](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/src/force_node/src/force_controller.cpp) handles excitation trajectories, PD tracking, and torque limiting.
- [src/sim_com_node/src/panda_simulator.cpp](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/src/sim_com_node/src/panda_simulator.cpp) handles MuJoCo stepping and CSV logging.
- [src/identification](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/src/identification) performs offline parameter identification.

Currently, `piper` only supports MuJoCo model switching, without physical hardware communication or control.

New requirements and constraints:

1. During execution, the data collection procedure in simulation and the procedure on physical hardware must be completely aligned.
2. A dual-track scheme ("one workflow for simulation and a separate workflow for physical hardware, compatible only at the CSV level") is not acceptable.
3. The implementation must converge to: "Unified experiment scheduling layer + Unified control layer + Unified recording layer + Pluggable execution backends".

## 2. Overall Objectives

The goal of this refactoring is not to "add standalone hardware scripts", but to "abstract the existing experiment pipeline into a unified experimental framework that supports both simulation and physical hardware backends". The final system should achieve:

1. Establishing connection to the Piper physical robot arm via `piper_sdk`.
2. Executing safe hardware initialization routines, including connection checks, arm enabling, mode switching, and exception shutdown.
3. Ensuring simulation and hardware share the exact same excitation trajectory definitions and main experiment loop.
4. Ensuring simulation and hardware record identical CSV fields, identical column ordering, and identical sampling cadence.
5. Ensuring `identify` consumes simulation data and hardware data uniformly without branching.

## 3. Key Design Decisions

### 3.1 Unified Experiment Entry Point & Unified Experiment State Machine

The implementation must abstract the existing [run_experiment.cpp](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/src/app/run_experiment.cpp) into a unified experiment state machine, rather than maintaining separate entry points for simulation and hardware.

The unified state machine contains at least the following stages:

1. Load experiment configuration
2. Construct controller and excitation trajectory
3. Construct experiment backend
4. Enter unified time loop
5. Read state
6. Compute control output
7. Step backend by one increment
8. Record one sample
9. Conclude experiment and tear down

Only step 7 ("step backend by one increment") differs between simulation and hardware; all other steps must reuse the identical logic.

### 3.2 Unified Backend Interface, No Dual-Track Logic Branching

A unified experiment backend abstraction must be introduced, for example:

- `ExperimentBackend`
- `SimulationBackend`
- `PiperHardwareBackend`

The unified interface includes at least:

- `initialize()`
- `read_state()`
- `apply_command()`
- `step()`
- `shutdown()`

Constraints are as follows:

- Simulation backend: `step()` internally advances MuJoCo
- Hardware backend: `step()` internally sends hardware commands, waits for period, and reads back state
- The upper experiment scheduling layer must not write `if sim else real` style dual main loops

### 3.3 Excitation Trajectory & Control Output Must Share the Same Implementation

The identifier currently reads by default:

- `time`
- `q0..qN`
- `qd0..qdN`
- `qdd0..qN`
- `tau0..tauN`

The project must satisfy:

1. Simulation and hardware share the identical excitation trajectory generator
2. Simulation and hardware share the identical controller interface
3. At any timestamp `t`, target `q / qd / qdd / tau_cmd` computed for simulation and hardware have identical semantics

In other words, hardware does not use an "independently defined convenient trajectory", but executes the trajectory derived from the same source as simulation.

### 3.4 Data Recording Protocol Must Be Fully Consistent

The hardware side must not merely be "roughly similar in fields", but must achieve:

- Identical column names
- Identical column order
- Identical physical units
- Identical sampling condition triggers
- Identical outlier rejection rules

Recommended unified records:

- `time`
- `q0..qN`
- `qd0..qdN`
- `qdd0..qddN`
- `tau0..tauN`

Where:

- Simulation: `qdd` comes from MuJoCo, `tau` comes from control command
- Hardware: `qdd` is estimated via a unified algorithm, `tau` must be defined with the same physical meaning as in simulation; if this cannot be met, simulation recording definitions must be adjusted simultaneously until semantics match across both sides

### 3.5 Safety Mechanisms May Differ, But Must Not Break Upper Flow Alignment

Hardware safety logic is permitted to be stricter than simulation, but must reside exclusively within the backend or safety gating layers, without altering the structure of the experiment main loop.

Permitted differences:

- Initialization checks
- Arm enabling / emergency stop
- Soft joint limits protection
- Disconnection handling

Non-permitted differences:

- Trajectory generation approach
- Control loop sequence
- Data recording timing
- CSV output format

## 4. Phased Implementation Plan

### Phase A: Refactor Unified Experiment Framework

Objective: Abstract "how simulation experiments run" into a unified framework before integrating physical hardware.

Tasks:

1. Extract the current experiment main loop from [run_experiment.cpp](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/src/app/run_experiment.cpp) into a unified scheduling layer
2. Abstract unified backend interface
3. Refactor MuJoCo simulator into `SimulationBackend`
4. Decouple CSV writing logic from simulator into an independent recorder
5. Solidify "Read State -> Compute Control -> Step Backend -> Record Sample" as the single main loop

### Phase B: Unify Excitation Trajectory & Controller

Objective: Ensure simulation and physical hardware use completely shared experimental inputs.

Tasks:

1. Retain and reuse excitation trajectory generation logic from `ForceController`
2. Clarify input/output contracts for `computeTorque()` as a common interface for simulation and hardware
3. Define how the hardware side executes target `q / qd / qdd / tau_cmd` derived from the same source
4. If `tau_cmd` cannot directly map to low-level interfaces, specify a unified transformation layer in the plan rather than creating a separate hardware trajectory

### Phase C: Integrate Piper Physical Hardware Backend

Objective: Integrate physical hardware within the unified experiment framework without altering the upper experiment flow.

Tasks:

1. Add `PiperHardwareBackend`
2. Encapsulate `piper_sdk` connection, enable, state reading, command dispatch, emergency stop, and disconnection handling
3. Ensure hardware backend implements identical interface methods with identical semantics as the simulation backend
4. Restrict hardware safety logic to backend or safety gating layers, without impacting the unified main loop

### Phase D: Unify Sampling & Data Recorder

Objective: Ensure simulation and hardware produce identical data structures in the same recorder.

Tasks:

1. Implement an independent shared CSV recorder
2. Unify physical units:
   - Angles in `rad`
   - Velocities in `rad/s`
   - Accelerations in `rad/s^2`
   - Torques in `N*m`
3. Unify sample write timing, ensuring both simulation and hardware record at the "same stage after backend step completion"
4. Add a unified estimator for hardware `qdd`, retaining the same data entry point on the simulation side
5. Add unified outlier rejection rules:
   - Saturated samples
   - Anomalous step changes
   - Communication loss samples
6. Verify generated CSV can be parsed without branching by [src/identification/src/main.cpp](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/src/identification/src/main.cpp)

### Phase E: Unified Entry Point & Operation

Objective: Enable users to run simulation and hardware experiments using the same executable entry point and configuration structure.

Tasks:

1. Unify experiment configuration structure, e.g., switching backends via `backend: sim | piper_real`
2. Unify entry arguments, avoiding "one executable for simulation and another for hardware"
3. Ensure the same configuration preserves trajectory and sampling semantics across backends, except for safety-specific fields

## 5. Scope of Planned File Changes

Expected areas of modification:

- Refactor [src/app/run_experiment.cpp](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/src/app/run_experiment.cpp) and related scheduling logic
- Add unified experiment backend abstraction
- Separate logging responsibilities from [src/sim_com_node/src/panda_simulator.cpp](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/src/sim_com_node/src/panda_simulator.cpp)
- Add Piper SDK wrapper module
- Adjust configuration structures to support backend toggling via unified entry point
- Update README with unified usage instructions

Avoid direct modifications where possible:

- Existing core identification algorithms

Only make minimal adjustments to the identification entry point if unified data semantics cannot hold otherwise.

## 6. Risk Points

### High Risk Items

- Directly mapping ideal simulation torques to physical hardware may cause impact or trigger protection
- Hardware cannot directly obtain low-noise `qdd`, affecting feasibility of "complete alignment"
- Feedback effort and commanded torque may not be strictly equivalent, potentially breaking semantic consistency
- Retaining dual entry points or dual main loops makes future behavioral drift almost unavoidable

### Medium Risk Items

- Control loop jitter causing irregular sampling
- Occasional CAN frame drops
- Discrepancies between physical arm firmware/hardware versions and SDK documentation

## 7. Stop-Loss & Rollback Strategies

1. If the unified framework refactoring affects existing simulation results, roll back until "simulation backend + new abstraction layer" passes
2. If hardware behaves abnormally at any stage, immediately halt, but do not alter the hardware flow into a separate simplified main loop
3. If MIT mapping is unmanageable, prioritize adjusting the unified control transformation layer rather than creating a dedicated hardware trajectory
4. If `tau` or `qdd` semantics cannot align, correct recording definitions first before proceeding with identification experiments

## 8. Acceptance Criteria

Following physical hardware integration, the following criteria must be met:

1. Users can switch between `sim` and `piper_real` through the same experiment entry point
2. Simulation and hardware use the same excitation trajectory configuration and control loop
3. Simulation and hardware output completely identical CSV structures and units
4. `identify` completes identification on both data sets without branching logic
5. The system shuts down safely upon disconnection, limits violation, or abnormal status during hardware execution

## 9. Recommended Execution Order

Recommended implementation sequence:

1. Refactor unified experiment main loop first
2. Abstract unified backend interface
3. Migrate simulation pipeline into the new framework and verify identical results
4. Integrate Piper physical hardware backend
5. Align sampling, recording, filtering, and CSV output
6. Perform on-site physical hardware safety validation

## 10. Conclusion

When transitioning from simulation to the Piper physical robot, subsequent development must adhere to a strict constraint:

- Simulation and hardware are not "two experimental programs with similar functionality"
- They are "one single experiment program + one set of control and sampling logic + two different execution backends"

Only by embedding this constraint into the architecture will physical arm parameter identification remain free from trajectory, control, sampling, and data semantic drift.
