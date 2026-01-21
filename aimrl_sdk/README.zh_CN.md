[English](README.md) | 中文

# AimRL SDK（Python）

`aimrl_sdk` 是一套面向 **AIMRT + ROS2 通信后端** 的 Python SDK（pybind11 绑定）。
它提供了：
- 观测帧（arm/leg/imu）拉流与“对齐帧”生成（timestamp 对齐、valid 标记）
- 关节命令下发（arm/leg 的 position/velocity/effort/Kp/Kd）
- A2 踝关节闭链（toe A/B ↔ ankle pitch/roll）在 **观测/命令** 两侧的转换

本文档包含：系统架构、配置/参数解释、example 架构、以及基于 `uv` 的安装与运行方式。

## 系统架构

整体数据链路如下（以默认后端为例）：

```
ROS2 topics (/body_drive/*)
   ├─ /body_drive/arm_joint_state   (JointState)
   ├─ /body_drive/leg_joint_state   (JointState)
   ├─ /body_drive/imu/data          (Imu)
   ├─ /body_drive/arm_joint_command (JointCommand)   ← publish
   └─ /body_drive/leg_joint_command (JointCommand)   ← publish
            │
            ▼
AimRT Core + ros2_plugin (aimrl_sdk/src/aimrl_sdk/config/aimrt_ros2_backend.yaml)
            │
            ▼
C++ Core（ring buffer + sync_loop 生成 aligned frame(valid/invalid)）
            │
            ▼
pybind11 bindings（`aimrl_sdk._bindings`）
            │
            ▼
Python API（`aimrl_sdk.open()/close()` + `StateInterface/CommandInterface`）
```

关键点：
- `StateInterface.latest_frame()` 返回 `(stamp_ns, valid, obs)`；`valid` 表示该帧是否满足对齐条件（时间戳偏差、数据齐全等）。
- “对齐帧”的生成频率由 `sync_hz` 决定（`aimrl_sdk.open(sync_hz=...)`）。
- 默认会启用 A2 踝关节闭链转换：观测中把 toe motorA/B 映射成 `(toe_pitch, toe_roll)`，命令侧把 ankle 指令再转换回 motorA/B（可通过 `use_closed_ankle=False` 关闭）。

## 安装与运行（uv）

下面以在本仓库内开发为例（推荐 `uv` 管理依赖 + 构建扩展）。

### 1) 创建环境并安装依赖/构建扩展

在仓库根目录执行：

```bash
cd aimrl_sdk
uv sync
```

说明：
- `uv sync` 会根据 `aimrl_sdk/uv.lock` 安装 Python 依赖，并构建本地扩展（scikit-build-core + pybind11）。
- 若你希望严格使用 lock：`uv sync --frozen`。

### 2) 运行 example

在 `aimrl_sdk/` 目录内运行：

```bash
uv run python examples/rl_deploy_basic.py --cfg examples/configs/agibot_a2_dof12.yaml
```

从仓库根目录运行也可以：

```bash
uv run --project aimrl_sdk python aimrl_sdk/examples/rl_deploy_basic.py --cfg aimrl_sdk/examples/configs/agibot_a2_dof12.yaml
```

## Python API（核心概念）

### `aimrl_sdk.open(...)`

`aimrl_sdk.open()` 返回 `(state, cmd)`：
- `state`: `StateInterface`，用于读观测
- `cmd`: `CommandInterface`，用于下发关节命令

常用参数：
- `config_path`: AimRT 后端 YAML；默认会用 `AIMRL_SDK_CONFIG`，否则回落到包内默认 `aimrl_sdk/src/aimrl_sdk/config/aimrt_ros2_backend.yaml`
- `sync_hz`: aligned frame 生成频率（Hz）
- `max_skew_ms`: 允许的最大时间戳偏差（ms），超出则该帧 `valid=False`
- `require_all`: 若为 True，则 arm/leg/imu 任一缺失会标记 invalid
- `drop_invalid`: 若为 True，则 invalid 帧不会写入对齐帧 ring
- `use_closed_ankle`: 是否启用踝关节闭链转换（默认 True）

### `StateInterface`

- `latest_frame() -> (stamp_ns, valid, obs)`
- `wait_frame(timeout_s=...) -> (stamp_ns, valid, obs)`：阻塞直到新帧（或超时）

`obs` 是 `float32` 的 1D 向量，布局在 C++ 中定义，Python 侧用 `aimrl_sdk.OBS` 提供切片：
- `obs[aimrl_sdk.OBS.leg_pos]`
- `obs[aimrl_sdk.OBS.leg_vel]`
- `obs[aimrl_sdk.OBS.imu_quat_xyzw]`
- `obs[aimrl_sdk.OBS.imu_gyro_xyz]`
等

### `CommandInterface`

- `set_leg(position=..., velocity=..., effort=..., stiffness=..., damping=...)`
- `set_arm(position=..., velocity=..., effort=..., stiffness=..., damping=...)`
- `commit(stamp_ns=..., sequence=...)`

通常建议：
- `stamp_ns` 传入对应观测帧的时间戳（用于下游同步）
- 以 `control_hz` 频率循环：读 `latest_frame()` → 计算 → `set_*()` → `commit()`

## 配置文件（example schema）与参数含义

example 使用 `aimrl_sdk/examples/configs/agibot_a2_dof12.yaml` 的 schema（由 `aimrl_sdk/examples/rl_deploy_config.py` 解析）。

### `control`
- `control.hz`: 控制循环频率（Hz），example 用它来计算 `dt=1/hz`
- `control.sync_hz`: SDK 对齐帧频率（Hz），若不填默认等于 `control.hz`

### `model`
- `model.path`: ONNX 模型路径（相对路径会相对于 YAML 所在目录解析）

### `robot`
- `robot.leg_default_joint_angles`: 12 DoF 腿默认角（用于 obs 归一化与 action 反归一化）
- `robot.leg_stiffness`: 12 DoF 腿刚度（Kp）
- `robot.leg_damping`: 12 DoF 腿阻尼（Kd）

### `policy`
- `policy.action_scale`: 动作缩放（`action * scale + default_joint_angles`）
- `policy.clip_actions`: 动作裁剪范围

### `phase`
- `phase.cycle_time`: 步态相位周期（秒）
- `phase.sw_mode`: 是否启用“静止时相位锁定”（无指令时相位归零）
- `phase.cmd_threshold`: 指令范数阈值，小于该值认为静止

### `observation`
- `observation.size`: 单步观测向量长度
- `observation.num_hist`: 历史堆叠长度（最终输入维度为 `size * num_hist`）
- `observation.clip`: 观测裁剪范围
- `observation.components`: 观测拼接顺序与缩放，类型包括：
  - `command`（5D）：`[sin(2πphase), cos(2πphase), cmd_x, cmd_y, cmd_yaw]`
  - `leg_pos`（12D）
  - `leg_vel`（12D）
  - `last_actions`（12D）
  - `imu_gyro`（3D）
  - `imu_euler`（3D）
  - `imu_quat`（4D）

## Examples：结构与内容

`aimrl_sdk/examples` 目录主要文件：
- `aimrl_sdk/examples/rl_deploy_basic.py`：端到端示例（读帧→ONNX 推理→下发命令）
- `aimrl_sdk/examples/rl_deploy_config.py`：example YAML schema + 校验/路径解析
- `aimrl_sdk/examples/teleop_control.py`：键盘/手柄输入解析 + 简化运控状态机（可选）
- `aimrl_sdk/examples/configs/*.yaml`：示例配置

### `rl_deploy_basic.py`（大致流程）

1. 解析 CLI 参数（cfg/model/hz/teleop 等）
2. `load_app_cfg()` 读取 YAML，得到 `AppCfg`
3. `aimrl_sdk.open(sync_hz=...)` 打开 SDK
4. 等待第一帧对齐（`wait_frame()`）
5. 控制循环：
   - 读取 `latest_frame()`
   - 读取 teleop 输入（cmd_x/cmd_y/cmd_yaw + 按键边沿）
   - 计算关节目标（FSM 模式：`DEFAULT/LIE/STAND/WALK`；或 `--no-fsm` 直接走 WALK）
   - `cmd.set_leg(...)` / `cmd.set_arm(...)`
   - `cmd.commit(stamp_ns=...)`

### `teleop_control.py`（输入与状态机）

- 输入：
  - Linux joystick：默认读 `/dev/input/js0`（可通过 `--joystick` 指定）
  - 键盘：cbreak 模式（非 canonical + 无 echo，但保持正常日志输出）
- 状态机（简化版，语义对齐 `deploy/rl_controllers`）：
  - `DEFAULT`：不主动控制（示例里为零刚度或保持当前）
  - `LIE` / `STAND`：过渡姿态（示例里用 default pose + 不同增益做简化）
  - `WALK`：运行 ONNX policy

## CLI 参数（rl_deploy_basic.py）

- `--cfg`: example YAML 路径
- `--model`: 覆盖 YAML 里的 `model.path`
- `--control-hz`: 覆盖 YAML 里的 `control.hz`
- `--sync-hz`: 覆盖 YAML 里的 `control.sync_hz`（传给 `aimrl_sdk.open(sync_hz=...)`）
- `--cmd-x/--cmd-y/--cmd-yaw`: 初始指令（teleop 未启用或键盘增量控制时有用）
- `--joystick`: joystick 设备路径（默认 `/dev/input/js0`）
- `--no-fsm`: 禁用状态机，程序启动默认直接进入 WALK（policy），仍由 deadman 门控

## 常见问题（FAQ）

### 1) `Joystick not available: /dev/input/js0`

说明系统上没有该设备或权限不足：
- 用 `--joystick /dev/input/jsX` 指定正确设备
- 或把当前用户加入 `input` 组/调整 udev 权限（不同系统策略不同）

### 2) `Latest frame is not aligned`

说明当前对齐帧 `valid=False`，常见原因：
- 上游 topic 的时间戳抖动/延迟导致超过 `max_skew_ms`
- arm/leg/imu 有一项缺失（`require_all=True`）
- `sync_hz` 与实际发布频率差异过大

你可以通过 `aimrl_sdk.open(max_skew_ms=..., require_all=..., drop_invalid=...)` 调整策略。
