[English](README.md) | 中文

# AimRL SDK（Python）

`aimrl_sdk` 是一套面向 **AIMRT + ROS2 通信后端** 的 Python SDK（pybind11 绑定）。
它提供了：
- 观测帧（arm/leg/imu）拉流与“对齐帧”生成（timestamp 对齐、aligned/complete 标记）
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
C++ Core（ring buffer + sync_loop 生成 aligned frame，并提供 aligned/complete 标记）
            │
            ▼
pybind11 bindings（`aimrl_sdk._bindings`）
            │
            ▼
Python API（`aimrl_sdk.open()/close()` + `StateInterface/CommandInterface`）
```

关键点：
- `StateInterface.latest_frame()` 返回 `(stamp_ns, aligned, complete, obs)`：
  - `complete`：tick 时刻 arm/leg/imu 是否都齐
  - `aligned`：在 `complete=True` 的前提下，是否满足 `max_skew_ms` 对齐约束
- “对齐帧”的生成频率由 `sync_hz` 决定（`aimrl_sdk.open(sync_hz=...)`）。
- 默认会启用 A2 踝关节闭链转换：观测中把 toe motorA/B 映射成 `(toe_pitch, toe_roll)`，命令侧把 ankle 指令再转换回 motorA/B（可通过 `use_closed_ankle=False` 关闭）。

## 安装与运行（uv）

下面以在本仓库内开发为例（推荐 `uv` 管理依赖 + 构建扩展）。

仅支持 Linux。

类型提示：
- 本包会随 wheel 一起发布 pybind11 扩展的 stub：`aimrl_sdk/_bindings.pyi`。
- 重新生成：`uv sync --extra stubs && uv run python tools/generate_pyi.py --write-to-src`。

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
- `max_skew_ms`: 允许的最大时间戳偏差（ms），超出则该帧 `aligned=False`
- `use_closed_ankle`: 是否启用踝关节闭链转换（默认 True）
- `enable_statistics`: 启用低开销运行时统计（默认 False）
- `statistics_sample_every`: 统计采样 1/N 事件（默认 1）
- `statistics_ema_shift`: EMA 平滑参数（alpha = 1/2^shift，默认 4）

### `StateInterface`

- `latest_frame() -> (stamp_ns, aligned, complete, obs)`
- `wait_frame(timeout_s=...) -> (stamp_ns, aligned, complete, obs)`：阻塞直到新帧（或超时）
- `statistics() -> dict`：统计快照（延迟/抖动、publish 耗时、sync 有效性/缺失原因等）
- `configure_statistics(enabled, sample_every=1, ema_shift=4)`：运行时开关与采样/平滑控制
- `reset_statistics()`：重置统计计数器

#### 当帧 incomplete / unaligned 时，最终拿到的数据是什么样？

默认行为（面向强化学习部署的推荐默认）：
- SDK 以 `sync_hz` 频率生成对齐帧并写入内部 ring。
- `complete=False` 表示 tick 时刻 arm/leg/imu 任一路缺失。
- `aligned=False` 表示在 `complete=True` 的前提下，时间戳偏差超过 `max_skew_ms`。
- 当 `complete=False` 时，`obs` 会**保持为上一帧 complete 的观测**（如果启动阶段尚未得到任何 complete 帧，则 `obs` 全零）。

实用建议：用 `(complete and aligned)` 作为主要的“这帧能不能用”门控；启用 statistics 后，可用 `statistics()['sync']` 查看缺失/未对齐的计数与原因拆解。

#### 对齐逻辑（`aligned/complete/skew_ns` 是如何计算的）

对齐帧生成线程以 `sync_hz` 频率运行。每个 tick 的逻辑大致如下：

- **tick 时间**：按固定周期网格选择目标 `tick`（`period_ns = 1e9 / sync_hz`），然后 `sleep_until(tick)`。
- **样本选择**：对每一路（arm/leg/imu），从 ring buffer 向后扫描，选择满足 `sample.stamp <= tick` 的最新样本（最多回溯 `max_backtrack` 个样本）。
- **complete（数据齐全）**：
  - `complete = arm_ok && leg_ok && imu_ok`
- **skew（时间戳偏差）**：
  - `skew_ns = max(arm.stamp, leg.stamp, imu.stamp) - min(arm.stamp, leg.stamp, imu.stamp)`（仅在 `complete=True` 时有意义）
- **aligned（对齐成功）**：
  - `aligned = complete && (skew_ns <= max_skew_ms * 1e6)`

关于帧内容与时间戳：
- `frame.stamp_ns` 使用的是 **tick 时间**（不是被选中的样本时间）。
- 当 `complete=True` 时，`obs` 由选中的 arm/leg/imu 样本填充（并可选应用踝关节闭链转换）。
- 当 `complete=False` 时，`obs` 会保持为上一帧 complete 的观测（见上一节）。

关于 timestamp 的重要说明：
- 对齐判断基于消息的 `header.stamp`（若 stamp 缺失/非法则用本地时间做兜底），这里衡量的是“跨流一致性”（不是“相对 tick 的新鲜度”），因此上游时间戳是否正确、以及时钟是否同步，会显著影响 `aligned`。

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

## 运行时统计（延迟/抖动/性能）

`aimrl_sdk` 支持采集**低开销运行时统计**（默认关闭），重点覆盖：
- **订阅链路**：arm/leg/imu 的延迟与抖动
- **发布链路**：arm/leg JointCommand 的 publish 耗时，以及 `commit()` 总耗时
- **对齐线程（sync_loop）**：每 tick 的性能、overrun、对齐帧 `aligned/complete` 的原因拆解
- **同步等待（wait_frame）**：ok/timeout/stopped 计数与等待耗时

设计目标：
- **默认不开**；关闭时尽量减少对主链路影响（热路径不做大部分计时/聚合）。
- 通过采样（`sample_every`）与 EMA 平滑（`ema_shift`）**可控开销**。
- 全部统计用 **atomic** 做计数与聚合；`statistics()` 读取不加锁。

### 大致原理

- 每次订阅回调触发时，SDK 记录本地接收时间戳，并与消息 `header.stamp` 对比得到 **delay**（延迟估计）。
- 同时记录相邻两次接收时间戳差值得到 **interval**，并用 `|interval - EMA(interval)|` 作为 **jitter**（抖动代理）。
- 每次 `commit()` 记录：
  - arm/leg 的 publish 调用耗时（只有该侧确实有命令时才计时）
  - `commit()` 总耗时（包含闭链踝关节转换与两次 publish）
- sync 线程每 tick 记录：
  - wake lateness：相对于理想 tick 时间（`tick`）醒来的“迟到量”
  - compute 时间与 overrun（简单判断：`compute_ns > tick_period_ns`）
  - 对齐帧状态原因：数据缺失（`complete=False`） vs `max_skew` 超限（`aligned=False`），以及具体缺失哪一路

注意：
- 输出中所有时间相关字段单位都是 **ns**（纳秒）。
- `delay = recv_time_ns - header_stamp_ns`。如果上游 stamp 用了不同的时钟源/时间不同步，可能会出现 `rx_negative_delay`。

### 启用与配置

可在 open 时启用，也可以运行时切换（`state` 或 `cmd` 都可以操作）：

- open 参数：
  - `aimrl_sdk.open(..., enable_statistics=True, statistics_sample_every=10, statistics_ema_shift=4)`
- 运行时：
  - `state.configure_statistics(True, sample_every=10, ema_shift=4)`
  - `state.reset_statistics()`
  - `snap = state.statistics()`

参数含义：
- `enable_statistics` / `enabled`：总开关。
- `statistics_sample_every` / `sample_every`：统计采样比率 1/N（1 表示每次都统计）。
  - `rx_total` 始终是总消息数，但如 `delay_ns.count` 这类 metric 的 `count` 是“被采样到并聚合”的点数。
- `statistics_ema_shift` / `ema_shift`：EMA 平滑参数（`alpha = 1 / 2^ema_shift`）。
  - `ema_shift=0` 近似“无平滑”（EMA 更接近 last）。

### 输出数据结构（`statistics() -> dict`）

顶层字段：
- `enabled`：当前是否启用统计
- `sample_every`, `ema_shift`：当前配置
- `now_steady_ns`, `start_steady_ns`, `uptime_ns`：内部 steady clock 时间戳（推荐用 `uptime_ns` 做稳定时间间隔）

通用 metric 字段（如 `delay_ns` / `interval_ns` / `duration_ns` 等）：
- `count`：采样聚合点数
- `last_ns`：最后一次采样值
- `ema_ns`：EMA 平滑值
- `min_ns`, `max_ns`：采样点上的最小/最大值

#### `arm_state` / `leg_state` / `imu`（订阅侧）

每路字段：
- `rx_total`：接收消息总数
- `rx_stamp_missing`：消息头 stamp 缺失/非法（`stamp_ns <= 0`）
- `rx_negative_delay`：`recv_time_ns - stamp_ns < 0`（时钟不一致或 stamp 异常）
- `delay_ns`：延迟估计（接收时间 - 消息 stamp）
- `interval_ns`：相邻两条消息的接收间隔
- `interval_jitter_ns`：间隔抖动代理（`|interval - EMA(interval)|`）

解释建议：
- `delay_ns` 只有在上游 stamp 与本地接收时间可比较（同一时钟源/已同步）时，才可视作端到端时延。
- `interval_jitter_ns` 是简单抖动代理，不是分位数统计。

#### `publish_arm` / `publish_leg` / `commit_total`（发布侧）

- `attempts`：观察到的 commit 次数
- `skipped_no_cmd`：该侧没有命令（`has_any==false`）而跳过计时/发布的次数
- `duration_ns`：耗时（采样统计）

#### `sync`（对齐帧生成线程）

- `tick_total`：tick 总数
- `tick_overrun`：compute 超过 tick 周期的次数（粗略 overrun）
- `wake_lateness_ns`：醒来迟到量（采样）
- `compute_ns`：每 tick 计算耗时（采样）
- `age_arm_ns` / `age_leg_ns` / `age_imu_ns`：每路样本时间戳相对 tick 的“滞后量”（`tick - stamp`，采样）
- `missing_arm` / `missing_leg` / `missing_imu`：每 tick 缺失各路样本的次数
- `frame_written`：写入 frame ring 的帧数
- `frame_complete`：arm+leg+imu 都齐的帧数
- `frame_incomplete`：至少缺失一路的帧数
- `frame_aligned`：complete 且满足 skew 约束（可用对齐）的帧数
- `frame_unaligned_skew`：complete 但 skew 超限的帧数
- `frame_incomplete_missing_arm/leg/imu`：当出现 incomplete 时，具体缺失哪一路的计数

#### `wait_frame`（同步订阅等待）

- `calls`：调用次数
- `ok` / `timeout` / `stopped`：结果计数
- `wait_ns`：等待耗时（仅对 ok 的采样统计）

### 示例：打印关键信息

```python
st, cmd = aimrl_sdk.open(sync_hz=100.0, enable_statistics=True, statistics_sample_every=10)
snap = st.statistics()
print("arm delay ema(ms) =", snap["arm_state"]["delay_ns"]["ema_ns"] / 1e6)
print("commit ema(ms)    =", snap["commit_total"]["duration_ns"]["ema_ns"] / 1e6)
print("sync unaligned(skew) =", snap["sync"]["frame_unaligned_skew"])
```

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
    - `w/s` 或 `↑/↓`：前进/后退
    - `a/d` 或 `←/→`：左/右转（yaw）
    - `q/e`：横移（lateral）
    - `space`：切换 deadman（门控运动），`x/ESC`：急停
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

通常表示 `(aligned=False)` 和/或 `(complete=False)`，常见原因：
- 上游 topic 的时间戳抖动/延迟导致超过 `max_skew_ms`
- arm/leg/imu 有一项缺失（导致 `complete=False`）
- `sync_hz` 与实际发布频率差异过大

你可以通过 `aimrl_sdk.open(max_skew_ms=..., sync_hz=...)` 调整策略，并检查上游 timestamp/时钟同步。
