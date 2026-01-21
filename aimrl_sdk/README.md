English | [中文](README.zh_CN.md)

# AimRL SDK (Python)

`aimrl_sdk` is a Python SDK (pybind11 bindings) designed for an **AIMRT + ROS2 backend**. It provides:
- streaming observation frames (arm/leg/imu) and generating “aligned frames” (timestamp alignment + `valid` flag)
- sending joint commands (arm/leg `position/velocity/effort/Kp/Kd`)
- A2 closed-chain ankle conversion (toe A/B ↔ ankle pitch/roll) on both the **observation** and **command** sides

This document covers: system architecture, configs/parameters, example structure, and `uv`-based install & usage.

## System Architecture

Data path (default backend):

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
C++ Core (ring buffers + sync_loop to produce aligned frames valid/invalid)
            │
            ▼
pybind11 bindings (`aimrl_sdk._bindings`)
            │
            ▼
Python API (`aimrl_sdk.open()/close()` + `StateInterface/CommandInterface`)
```

Key points:
- `StateInterface.latest_frame()` returns `(stamp_ns, valid, obs)`; `valid` indicates whether the frame passes the alignment constraints (timestamp skew, completeness, etc.).
- Aligned-frame generation frequency is controlled by `sync_hz` (`aimrl_sdk.open(sync_hz=...)`).
- By default, the A2 ankle closed-chain conversion is enabled: toe motorA/B are mapped to `(toe_pitch, toe_roll)` in observations; ankle commands are mapped back to motorA/B (disable with `use_closed_ankle=False`).

## Install & Run (uv)

This section assumes you are developing inside this repository (recommended: manage deps + build extension via `uv`).

### 1) Create environment, install deps, build extension

Run at repo root:

```bash
cd aimrl_sdk
uv sync
```

Notes:
- `uv sync` installs Python dependencies based on `aimrl_sdk/uv.lock`, and builds the local extension (scikit-build-core + pybind11).
- To strictly use the lockfile: `uv sync --frozen`.

### 2) Run an example

Run inside `aimrl_sdk/`:

```bash
uv run python examples/rl_deploy_basic.py --cfg examples/configs/agibot_a2_dof12.yaml
```

Or run from repo root:

```bash
uv run --project aimrl_sdk python aimrl_sdk/examples/rl_deploy_basic.py --cfg aimrl_sdk/examples/configs/agibot_a2_dof12.yaml
```

## Python API (Core Concepts)

### `aimrl_sdk.open(...)`

`aimrl_sdk.open()` returns `(state, cmd)`:
- `state`: `StateInterface` for reading observations
- `cmd`: `CommandInterface` for sending commands

Common parameters:
- `config_path`: path to the AimRT backend YAML; by default it uses `AIMRL_SDK_CONFIG`, otherwise falls back to `aimrl_sdk/src/aimrl_sdk/config/aimrt_ros2_backend.yaml`
- `sync_hz`: aligned-frame frequency (Hz)
- `max_skew_ms`: maximum allowed timestamp skew (ms); frames beyond this will be `valid=False`
- `require_all`: if True, frame becomes invalid if any of arm/leg/imu is missing
- `drop_invalid`: if True, invalid frames are not written to the aligned-frame ring
- `use_closed_ankle`: enable the ankle closed-chain conversion (default True)

### `StateInterface`

- `latest_frame() -> (stamp_ns, valid, obs)`
- `wait_frame(timeout_s=...) -> (stamp_ns, valid, obs)`: block until a new frame arrives (or timeout)

`obs` is a 1D `float32` vector with layout defined in C++. Python exposes slices via `aimrl_sdk.OBS`, e.g.:
- `obs[aimrl_sdk.OBS.leg_pos]`
- `obs[aimrl_sdk.OBS.leg_vel]`
- `obs[aimrl_sdk.OBS.imu_quat_xyzw]`
- `obs[aimrl_sdk.OBS.imu_gyro_xyz]`

### `CommandInterface`

- `set_leg(position=..., velocity=..., effort=..., stiffness=..., damping=...)`
- `set_arm(position=..., velocity=..., effort=..., stiffness=..., damping=...)`
- `commit(stamp_ns=..., sequence=...)`

Typical usage:
- pass `stamp_ns` from the corresponding observation frame (for downstream synchronization)
- run at `control_hz`: read `latest_frame()` → compute → `set_*()` → `commit()`

## Example Config Schema & Parameter Meanings

Examples use the schema in `aimrl_sdk/examples/configs/agibot_a2_dof12.yaml` (parsed by `aimrl_sdk/examples/rl_deploy_config.py`).

### `control`
- `control.hz`: control loop frequency (Hz); example uses it to compute `dt = 1/hz`
- `control.sync_hz`: aligned-frame sync frequency (Hz); if omitted, defaults to `control.hz`

### `model`
- `model.path`: ONNX model path (relative paths are resolved relative to the YAML directory)

### `robot`
- `robot.leg_default_joint_angles`: default 12-DoF leg joint angles (used for observation normalization and action de-normalization)
- `robot.leg_stiffness`: 12-DoF leg stiffness (Kp)
- `robot.leg_damping`: 12-DoF leg damping (Kd)

### `policy`
- `policy.action_scale`: action scaling (`action * scale + default_joint_angles`)
- `policy.clip_actions`: action clipping range

### `phase`
- `phase.cycle_time`: gait phase cycle time (seconds)
- `phase.sw_mode`: whether to lock phase at zero when the command is small (“standing still”)
- `phase.cmd_threshold`: command norm threshold below which the robot is considered stationary

### `observation`
- `observation.size`: per-step observation vector length
- `observation.num_hist`: history stacking length (model input dim is `size * num_hist`)
- `observation.clip`: observation clipping range
- `observation.components`: concatenation order + scaling; supported types:
  - `command` (5D): `[sin(2πphase), cos(2πphase), cmd_x, cmd_y, cmd_yaw]`
  - `leg_pos` (12D)
  - `leg_vel` (12D)
  - `last_actions` (12D)
  - `imu_gyro` (3D)
  - `imu_euler` (3D)
  - `imu_quat` (4D)

## Examples: Structure & Contents

Main files under `aimrl_sdk/examples`:
- `aimrl_sdk/examples/rl_deploy_basic.py`: end-to-end demo (read frames → ONNX inference → send commands)
- `aimrl_sdk/examples/rl_deploy_config.py`: YAML schema + validation + path resolution
- `aimrl_sdk/examples/teleop_control.py`: keyboard/gamepad input + a simple motion FSM (optional)
- `aimrl_sdk/examples/configs/*.yaml`: example configs

### `rl_deploy_basic.py` (high-level flow)

1. parse CLI args (cfg/model/hz/teleop, etc.)
2. load YAML via `load_app_cfg()` into `AppCfg`
3. open SDK via `aimrl_sdk.open(sync_hz=...)`
4. wait for the first aligned frame (`wait_frame()`)
5. control loop:
   - read `latest_frame()`
   - read teleop input (cmd_x/cmd_y/cmd_yaw + button edges)
   - compute target joints (FSM: `DEFAULT/LIE/STAND/WALK`; or `--no-fsm` to directly run WALK)
   - `cmd.set_leg(...)` / `cmd.set_arm(...)`
   - `cmd.commit(stamp_ns=...)`

### `teleop_control.py` (input + FSM)

- Input sources:
  - Linux joystick: reads `/dev/input/js0` by default (override via `--joystick`)
  - Keyboard: cbreak-like mode (non-canonical + no echo, but keep normal log output)
- Motion FSM (simplified, semantics aligned with `deploy/rl_controllers`):
  - `DEFAULT`: not actively controlling (example uses zero stiffness or hold current)
  - `LIE` / `STAND`: transitional poses (simplified using default pose + different gains)
  - `WALK`: run the ONNX policy

## CLI Args (`rl_deploy_basic.py`)

- `--cfg`: example YAML path
- `--model`: override YAML `model.path`
- `--control-hz`: override YAML `control.hz`
- `--sync-hz`: override YAML `control.sync_hz` (passed to `aimrl_sdk.open(sync_hz=...)`)
- `--cmd-x/--cmd-y/--cmd-yaw`: initial command (useful for incremental keyboard control)
- `--joystick`: joystick device path (default `/dev/input/js0`)
- `--no-fsm`: disable the FSM and start directly in WALK (policy) mode, still gated by deadman

## FAQ

### 1) `Joystick not available: /dev/input/js0`

The device does not exist or permissions are missing:
- use `--joystick /dev/input/jsX` to specify the correct device
- or add your user to the `input` group / adjust udev rules (depends on your distro)

### 2) `Latest frame is not aligned`

This means the aligned frame is `valid=False`. Common causes:
- upstream timestamp jitter/latency exceeds `max_skew_ms`
- one of arm/leg/imu is missing while `require_all=True`
- `sync_hz` differs too much from the actual publish rate

Tune via `aimrl_sdk.open(max_skew_ms=..., require_all=..., drop_invalid=...)`.
