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
- `enable_statistics`: enable low-overhead runtime statistics (default False)
- `statistics_sample_every`: sample 1/N events for stats aggregation (default 1)
- `statistics_ema_shift`: EMA smoothing shift (alpha = 1/2^shift, default 4)

### `StateInterface`

- `latest_frame() -> (stamp_ns, valid, obs)`
- `wait_frame(timeout_s=...) -> (stamp_ns, valid, obs)`: block until a new frame arrives (or timeout)
- `statistics() -> dict`: snapshot counters/latency/jitter/publish cost/sync validity breakdown
- `configure_statistics(enabled, sample_every=1, ema_shift=4)`: runtime toggle + sampling/smoothing
- `reset_statistics()`: reset all statistics counters

#### What happens when a frame is `valid=False` (or data is missing)

The SDK produces an aligned frame on each sync tick. Whether you see invalid frames, and what `obs` contains, depends on `require_all` (default True) and `drop_invalid` (default False):

- `drop_invalid=True`: invalid frames are **not written** to the internal ring.
  - `wait_frame()` will only return when a **written** frame arrives (so you usually won’t observe `valid=False`).
  - `latest_frame()` keeps returning the **last written** frame. If the stream is currently invalid, this can look like “previous frame” (it is, and the `stamp_ns` will also be older).
- `drop_invalid=False`: invalid frames are written with `valid=False`, and `wait_frame()` can return them like any other frame.

What `obs` looks like:
- **Skew invalid** (all arm+leg+imu are present, but skew exceeds `max_skew_ms`): `valid=False` and `obs` is still **fully populated** (but marked invalid due to timing misalignment).
- **Missing data** (any stream missing at that tick):
  - with `require_all=True`: `valid=False` and `obs` is **zero-filled** (not “previous frame”, not a partial frame).
  - with `require_all=False`: the frame may still be marked `valid=True` (skew-only check), but `obs` is still **zero-filled** because the SDK only fills `obs` when arm+leg+imu are all available. If you need complete data, keep `require_all=True` (default).

Practical tip: treat `valid` as the primary “can I use this frame?” gate. If you enable stats, `statistics()["sync"]` can tell you *why* frames were invalid/missing.

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

## Runtime Statistics (Latency/Jitter/Performance)

`aimrl_sdk` can collect **low-overhead runtime statistics** for the messaging pipeline, focused on:
- **Subscribe path**: message delay and jitter (arm/leg/imu)
- **Publish path**: publish cost for arm/leg JointCommand and total `commit()` time
- **Sync loop**: per-tick timing + aligned-frame validity breakdown (skew vs missing data)
- **Sync wait**: `wait_frame()` outcome (ok/timeout/stopped) and wait time

Design goals:
- **Off by default**; when disabled, hot paths avoid most timing work.
- **Configurable overhead** via sampling (`sample_every`) and EMA smoothing (`ema_shift`).
- **Thread-safe counters**: all metrics use atomics; reading `statistics()` is lock-free.

### How it works (high level)

- On every subscribed message callback, the SDK captures a local receive timestamp and compares it to the message `header.stamp` to estimate **delay**.
- It also tracks consecutive receive timestamps to estimate **interval** and **jitter**.
- On `commit()`, it measures publish duration for arm/leg (when those commands are present) and total `commit()` time.
- In the sync thread, it measures:
  - wake lateness (how late the thread wakes up relative to the ideal tick time)
  - compute time per tick, and a simple overrun indicator (`compute_ns > tick_period_ns`)
  - aligned-frame validity causes: missing data (require_all) vs skew threshold

Notes/assumptions:
- All time metrics are in **nanoseconds** (`*_ns`) in the output.
- Delay is computed as `recv_time_ns - msg_header_stamp_ns`. If upstream stamping uses a different clock source, you may see `rx_negative_delay`.

### Enabling / configuring

You can enable statistics either when opening the SDK, or at runtime:

- At open:
  - `aimrl_sdk.open(..., enable_statistics=True, statistics_sample_every=10, statistics_ema_shift=4)`
- At runtime (on either `state` or `cmd`):
  - `state.configure_statistics(True, sample_every=10, ema_shift=4)`
  - `state.reset_statistics()`
  - `snap = state.statistics()`

Parameters:
- `enable_statistics` / `enabled`: master switch.
- `statistics_sample_every` / `sample_every`: aggregate 1/N events (1 means “every event”).
  - `rx_total` is still the total event count, but metric fields like `delay_ns.count` reflect the number of **sampled** points.
- `statistics_ema_shift` / `ema_shift`: EMA smoothing shift (`alpha = 1 / 2^ema_shift`).
  - `ema_shift=0` approximates “no smoothing” (EMA follows last sample).

### Output schema (`statistics() -> dict`)

Top-level fields:
- `enabled`: whether statistics collection is enabled
- `sample_every`, `ema_shift`: current config
- `now_steady_ns`, `start_steady_ns`, `uptime_ns`: internal steady-clock timestamps (use `uptime_ns` for a stable interval)

Common metric dict format (used by `delay_ns`, `interval_ns`, `duration_ns`, etc.):
- `count`: number of sampled points aggregated
- `last_ns`: last sampled value
- `ema_ns`: exponentially smoothed value
- `min_ns`, `max_ns`: min/max over sampled points

#### `arm_state` / `leg_state` / `imu` (subscribe-side)

Per-stream fields:
- `rx_total`: total number of received messages
- `rx_stamp_missing`: header stamp missing/invalid (`stamp_ns <= 0`)
- `rx_negative_delay`: `recv_time_ns - stamp_ns < 0` (clock mismatch or bad stamp)
- `delay_ns`: delay between receive time and message timestamp
- `interval_ns`: time between consecutive received messages (receive timestamp delta)
- `interval_jitter_ns`: absolute deviation of `interval_ns` from its EMA (`|interval - ema(interval)|`)

Interpretation:
- `delay_ns` reflects end-to-end latency **only if** the publisher stamp represents “publish time” on a clock comparable to local receive time.
- `interval_jitter_ns` is a simple jitter proxy; it is not a full distribution/percentile.

#### `publish_arm` / `publish_leg` / `commit_total` (publish-side)

- `attempts`: number of times `commit()` observed this publish category
- `skipped_no_cmd`: how many times no command was present (e.g., `has_any == false`)
- `duration_ns`: measured duration (sampled) of the publish call / total commit

#### `sync` (aligned-frame generator thread)

- `tick_total`: total sync ticks executed
- `tick_overrun`: ticks whose compute time exceeded tick period (approximate “overrun”)
- `wake_lateness_ns`: lateness relative to ideal tick time (sampled)
- `compute_ns`: compute duration per tick (sampled)
- `missing_arm` / `missing_leg` / `missing_imu`: how often each stream was missing a usable sample at tick time
- `frame_written`: frames written to the ring
- `frame_dropped_invalid`: invalid frames dropped due to `drop_invalid=True`
- `frame_valid`: frames with `valid=True`
- `frame_invalid_require_all_missing`: frames invalid because `require_all=True` and at least one stream was missing
- `frame_invalid_missing_arm` / `frame_invalid_missing_leg` / `frame_invalid_missing_imu`: which stream(s) caused require_all invalidation
- `frame_invalid_skew`: frames invalid because timestamp skew exceeded `max_skew_ns`

#### `wait_frame` (sync subscription / blocking wait)

- `calls`: number of `wait_frame()` calls
- `ok` / `timeout` / `stopped`: outcome counters
- `wait_ns`: time spent waiting (sampled for `ok` calls)

### Example: logging a compact snapshot

```python
st, cmd = aimrl_sdk.open(sync_hz=100.0, enable_statistics=True, statistics_sample_every=10)
snap = st.statistics()
print("arm delay ema(ms) =", snap["arm_state"]["delay_ns"]["ema_ns"] / 1e6)
print("commit ema(ms)    =", snap["commit_total"]["duration_ns"]["ema_ns"] / 1e6)
print("sync invalid skew =", snap["sync"]["frame_invalid_skew"])
```

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
