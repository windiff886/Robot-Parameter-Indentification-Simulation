# Piper 真机参数辨识接入计划

> 目标：让 MuJoCo 仿真采集流程与 Piper 真机采集流程在“实验入口、激励轨迹、控制循环、采样时序、CSV 字段、辨识入口”六个层面保持完全对齐，只允许底层执行后端不同。

## 1. 背景与当前现状

当前仓库的实验主链路是：

`run_experiment -> ForceController -> PandaSimulator -> benchmark_data.csv -> identify`

其中：

- [src/app/run_experiment.cpp](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/src/app/run_experiment.cpp) 负责读取配置并驱动主循环。
- [src/force_node/src/force_controller.cpp](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/src/force_node/src/force_controller.cpp) 负责激励轨迹、PD 跟踪和力矩限幅。
- [src/sim_com_node/src/panda_simulator.cpp](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/src/sim_com_node/src/panda_simulator.cpp) 负责 MuJoCo 步进和 CSV 记录。
- [src/identification](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/src/identification) 负责离线参数辨识。

当前 `piper` 仅支持 MuJoCo 模型切换，不支持真实硬件通信和控制。

新增需求约束：

1. 执行程序时，仿真收集数据的过程和真机收集数据的过程必须完全对齐。
2. 不接受“仿真一套流程、真机另一套流程，只在 CSV 上兼容”的方案。
3. 后续实现必须收敛为“统一实验调度层 + 统一控制层 + 统一记录层 + 不同执行后端”。

## 2. 总体目标

本次改造的落点不是“单独新增真机脚本”，而是“把现有实验链路抽象成统一实验框架，并同时支持仿真后端和真机后端”。最终应支持：

1. 通过 `piper_sdk` 建立与 Piper 真机的连接。
2. 执行真机安全初始化流程，包括连接检查、使能、模式切换、异常停机。
3. 仿真与真机使用同一套激励轨迹定义与同一套实验主循环。
4. 仿真与真机记录同一套 CSV 字段、同一列顺序、同一采样节奏。
5. `identify` 对仿真数据和真机数据不做分支处理，直接统一消费。

## 3. 关键设计决策

### 3.1 统一实验入口与统一实验状态机

后续实现必须把现有 [run_experiment.cpp](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/src/app/run_experiment.cpp) 抽象成统一实验状态机，而不是让仿真和真机分别维护不同入口。

统一状态机至少包含以下阶段：

1. 读取实验配置
2. 构造控制器与激励轨迹
3. 构造实验后端
4. 进入统一时间循环
5. 读取状态
6. 计算控制输出
7. 推进一步后端
8. 记录一条样本
9. 实验结束并收尾

其中只有“推进一步后端”的实现因仿真/真机不同而不同，其余步骤必须复用同一套逻辑。

### 3.2 统一后端接口，不允许双轨逻辑分叉

必须新增统一实验后端抽象，例如：

- `ExperimentBackend`
- `SimulationBackend`
- `PiperHardwareBackend`

统一接口至少包括：

- `initialize()`
- `read_state()`
- `apply_command()`
- `step()`
- `shutdown()`

约束如下：

- 仿真后端：`step()` 内部执行 MuJoCo 推进
- 真机后端：`step()` 内部完成真机命令下发、等待周期、回读状态
- 上层实验调度层不得写 `if sim else real` 风格的双套主循环

### 3.3 激励轨迹与控制输出必须共用同一实现

当前辨识器默认读取：

- `time`
- `q0..qN`
- `qd0..qdN`
- `qdd0..qN`
- `tau0..tauN`

本项目后续必须满足：

1. 仿真与真机共用同一个激励轨迹生成器
2. 仿真与真机共用同一个控制器接口
3. 仿真与真机在同一时刻 `t` 上计算出的目标 `q / qd / qdd / tau_cmd` 含义一致

也就是说，真机不是“另外定义一套方便执行的轨迹”，而是要执行与仿真同源的实验轨迹。

### 3.4 数据记录协议必须完全一致

真机侧不能只做到“字段差不多”，而必须做到：

- 同样的列名
- 同样的列顺序
- 同样的单位
- 同样的采样判定条件
- 同样的样本剔除原则

推荐统一记录：

- `time`
- `q0..qN`
- `qd0..qdN`
- `qdd0..qddN`
- `tau0..tauN`

其中：

- 仿真：`qdd` 来自 MuJoCo，`tau` 来自控制命令
- 真机：`qdd` 通过统一算法估计，`tau` 必须定义成与仿真侧同物理含义的量；如果做不到，就需要同时调整仿真记录定义，直到两侧语义统一

### 3.5 安全机制可以不同，但不能破坏上层流程对齐

真机安全逻辑允许比仿真更严格，但只能存在于后端或安全门控层，不能改变实验主循环的结构。

允许不同的部分：

- 初始化检查
- 使能/急停
- 软限位保护
- 断连保护

不允许不同的部分：

- 轨迹生成方式
- 控制循环顺序
- 数据记录时机
- CSV 输出格式

## 4. 分阶段实施计划

### 阶段 A：重构统一实验框架

目标：先把“仿真实验怎么跑”抽象成一套统一框架，再接入真机。

任务：

1. 把当前实验主循环从 [run_experiment.cpp](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/src/app/run_experiment.cpp) 中提炼为统一调度层
2. 抽象统一后端接口
3. 把 MuJoCo 仿真器改造成 `SimulationBackend`
4. 把 CSV 写出逻辑从仿真器内剥离成独立记录器
5. 把“读取状态 -> 计算控制 -> 后端步进 -> 记录样本”固定成唯一主循环

### 阶段 B：统一激励轨迹与控制器

目标：保证仿真与真机使用完全同源的实验输入。

任务：

1. 保留并复用当前 `ForceController` 的激励轨迹生成逻辑
2. 明确 `computeTorque()` 的输入输出契约，作为仿真/真机共同控制接口
3. 定义真机侧如何执行与仿真同源的目标 `q / qd / qdd / tau_cmd`
4. 若 `tau_cmd` 不能直接映射到底层接口，需在计划中明确统一变换层，而不是另起一套真机轨迹

### 阶段 C：接入 Piper 真机后端

目标：在统一实验框架中接入真实硬件，但不改变上层实验流程。

任务：

1. 新增 `PiperHardwareBackend`
2. 封装 `piper_sdk` 的连接、使能、状态读取、命令发送、急停和断连处理
3. 让真机后端实现与仿真后端同名同义的接口
4. 将真机安全逻辑限制在后端或安全门控层，不影响统一主循环

### 阶段 D：统一采样与记录器

目标：让仿真和真机在同一个记录器中生成同结构数据。

任务：

1. 实现独立的共享 CSV 记录器
2. 单位统一：
   - 角度统一为 `rad`
   - 速度统一为 `rad/s`
   - 加速度统一为 `rad/s^2`
   - 力矩统一为 `N*m`
3. 统一样本写出时机，确保仿真与真机都在“后端步进完成后的同一阶段”记录
4. 对真机 `qdd` 增加统一的估计器，并在仿真侧保留同样的数据入口
5. 对失真样本增加统一剔除规则：
   - 饱和样本
   - 异常跳变样本
   - 通信中断样本
6. 验证生成的 CSV 能被 [src/identification/src/main.cpp](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/src/identification/src/main.cpp) 无分支读取

### 阶段 E：统一入口与运行方式

目标：让用户执行仿真和真机实验时，使用同一程序入口和同一配置结构。

任务：

1. 统一实验配置结构，例如通过 `backend: sim | piper_real` 切换后端
2. 统一入口参数，避免“仿真运行一个程序、真机运行另一个程序”
3. 保证同一份实验配置在切换后端时，除了安全相关字段外，轨迹与采样参数含义不变

## 5. 计划中的文件改动范围

预计会涉及以下区域：

- 重构 [src/app/run_experiment.cpp](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/src/app/run_experiment.cpp) 或其周边调度逻辑
- 新增统一实验后端抽象
- 拆分 [src/sim_com_node/src/panda_simulator.cpp](/home/windiff/Code/Robot-Parameter-Indentification-Simulation/src/sim_com_node/src/panda_simulator.cpp) 中的记录职责
- 新增 Piper SDK 封装模块
- 调整配置结构，支持统一入口切换后端
- 更新 README，补充统一运行方式说明

尽量避免直接修改：

- 现有辨识核心算法

只有在统一数据语义无法成立时，才对辨识入口做最小调整。

## 6. 风险点

### 高风险项

- 直接将仿真中的理想力矩控制映射到真机，可能导致冲击和保护触发
- 真机无法直接获得低噪声 `qdd`，会影响“完全对齐”的可实现性
- 反馈 effort 与命令力矩可能不完全等价，会破坏仿真/真机数据语义一致性
- 若继续保留双入口或双主循环，后续行为漂移几乎不可避免

### 中风险项

- 控制周期抖动导致采样不均匀
- CAN 通信偶发丢帧
- 真实机械臂软硬件版本与 SDK 文档存在偏差

## 7. 止损与回滚策略

1. 若统一框架重构影响现有仿真结果，先回退到“仿真后端 + 新抽象层”通过为止
2. 任何阶段只要真机行为异常，立即停机，但不允许因此把真机流程改成另一套简化主循环
3. 若 MIT 映射不可控，优先调整统一控制变换层，而不是另外定义真机专用轨迹
4. 若 `tau` 或 `qdd` 语义无法对齐，必须先修正记录定义，再继续辨识实验

## 8. 验收标准

完成真机接入后，至少应满足：

1. 用户通过同一实验入口即可切换 `sim` 和 `piper_real`
2. 仿真与真机使用同一套激励轨迹配置和同一套控制循环
3. 仿真与真机输出完全一致的 CSV 结构与单位
4. `identify` 对两类数据不做分支处理即可完成辨识
5. 真机执行中出现断连、超限或异常状态时，程序能安全停机

## 9. 推荐执行顺序

建议按以下顺序推进：

1. 先重构统一实验主循环
2. 再抽象统一后端接口
3. 把仿真链路迁入新框架并验证结果不变
4. 接入 Piper 真机后端
5. 对齐采样、记录、过滤和 CSV 输出
6. 最后做真机现场安全验证

## 10. 本计划的结论

本项目从仿真迁移到 Piper 真机时，后续实现必须遵守一个硬约束：

- 仿真和真机不是“两个功能相似的实验程序”
- 而是“同一个实验程序 + 同一套控制与采样逻辑 + 两个不同的执行后端”

只有先把这个约束写进架构，后续真机参数辨识才不会在轨迹、控制、采样和数据语义上逐步漂移。
