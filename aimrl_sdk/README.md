English | [中文](README.zh_CN.md)

# AimRL SDK (Python)

`aimrl_sdk` is a Python SDK (pybind11 bindings) designed for an **AIMRT + ROS2 backend**. It provides:
- streaming observation frames (arm/leg/imu) and generating “aligned frames” (timestamp alignment + `aligned/complete` flags)
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
C++ Core (ring buffers + sync_loop to produce aligned frames with `aligned/complete` flags)
            │
            ▼
pybind11 bindings (`aimrl_sdk._bindings`)
            │
            ▼
Python API (`aimrl_sdk.open()/close()` + `StateInterface/CommandInterface`)
```

Key points:
- `StateInterface.latest_frame()` returns `(stamp_ns, aligned, complete, obs)`.
  - `complete`: whether arm+leg+imu are all available for that tick
  - `aligned`: whether a complete frame passes the timestamp-skew constraint (`max_skew_ms`)
- Aligned-frame generation frequency is controlled by `sync_hz` (`aimrl_sdk.open(sync_hz=...)`).
- By default, the A2 ankle closed-chain conversion is enabled: toe motorA/B are mapped to `(toe_pitch, toe_roll)` in observations; ankle commands are mapped back to motorA/B (disable with `use_closed_ankle=False`).

## Install & Run (uv)

This section assumes you are developing inside this repository (recommended: manage deps + build extension via `uv`).

Linux only.

Type hints:
- This package ships a generated stub file for the pybind11 extension: `aimrl_sdk/_bindings.pyi`.
- To regenerate: `uv sync --extra stubs && uv run python tools/generate_pyi.py --write-to-src`.

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
- `max_skew_ms`: maximum allowed timestamp skew (ms); frames beyond this will be `aligned=False`
- `use_closed_ankle`: enable the ankle closed-chain conversion (default True)
- `enable_statistics`: enable low-overhead runtime statistics (default False)
- `statistics_sample_every`: sample 1/N events for stats aggregation (default 1)
- `statistics_ema_shift`: EMA smoothing shift (alpha = 1/2^shift, default 4)

### `StateInterface`

- `latest_frame() -> (stamp_ns, aligned, complete, obs)`
- `wait_frame(timeout_s=...) -> (stamp_ns, aligned, complete, obs)`: block until a new frame arrives (or timeout)
- `statistics() -> dict`: snapshot counters/latency/jitter/publish cost/sync validity breakdown
- `configure_statistics(enabled, sample_every=1, ema_shift=4)`: runtime toggle + sampling/smoothing
- `reset_statistics()`: reset all statistics counters

#### What happens when a frame is not complete/aligned

Default behavior (recommended for RL deployment):
- Frames are generated at `sync_hz` and written to the internal ring.
- `complete=False` if any of arm/leg/imu is missing at that tick.
- `aligned=False` if the frame is complete but exceeds the `max_skew_ms` constraint.
- If `complete=False`, `obs` is **held from the last complete frame** (at startup, before any complete frame exists, it is all zeros).

Practical tip: treat `(complete and aligned)` as the primary “can I use this frame?” gate. If you enable stats, `statistics()["sync"]` provides counts/breakdowns.

#### Alignment logic (how `aligned/complete/skew_ns` are computed)

The aligned-frame generator runs in a dedicated thread at `sync_hz`. For each tick:

- **Tick time**: choose a target `tick` on a fixed period grid (`period_ns = 1e9 / sync_hz`), then `sleep_until(tick)`.
- **Sample selection**: for each stream (arm/leg/imu), pick the most recent sample with `sample.stamp <= tick` by scanning backward in the ring buffer, up to `max_backtrack`.
- **Completeness**:
  - `complete = arm_ok && leg_ok && imu_ok`
- **Skew**:
  - `skew_ns = max(arm.stamp, leg.stamp, imu.stamp) - min(arm.stamp, leg.stamp, imu.stamp)` (only meaningful when `complete=True`)
- **Aligned**:
  - `aligned = complete && (skew_ns <= max_skew_ms * 1e6)`

Frame contents and timestamping:
- `frame.stamp_ns` is the **tick** time (not the chosen sample time).
- When `complete=True`, `obs` is filled from the chosen arm/leg/imu samples (and optionally with the closed-ankle conversion applied).
- When `complete=False`, `obs` is held from the last complete frame (see above).

Important note about timestamps:
- The alignment check is based on the message `header.stamp` (or a local fallback if the stamp is missing/invalid), and it measures cross-stream consistency (not “freshness vs tick”).

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
  - aligned-frame status causes: incomplete data (`complete=False`) vs skew threshold (`aligned=False`)

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
- `age_arm_ns` / `age_leg_ns` / `age_imu_ns`: per-stream stamp “age” relative to tick (`tick - stamp`, sampled)
- `missing_arm` / `missing_leg` / `missing_imu`: how often each stream was missing a usable sample at tick time
- `frame_written`: frames written to the ring
- `frame_complete`: frames where arm+leg+imu were all available
- `frame_incomplete`: frames where at least one stream was missing
- `frame_aligned`: frames that are complete and pass the skew constraint
- `frame_unaligned_skew`: frames that are complete but fail the skew constraint
- `frame_incomplete_missing_arm/leg/imu`: which stream(s) were missing when `frame_incomplete` occurred

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
print("sync unaligned(skew) =", snap["sync"]["frame_unaligned_skew"])
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
    - `w/s` or `↑/↓`: forward/back
    - `a/d` or `←/→`: yaw left/right
    - `q/e`: lateral left/right
    - `space`: toggle deadman (gates motion), `x/ESC`: emergency stop
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

This usually means `(aligned=False)` and/or `(complete=False)`. Common causes:
- upstream timestamp jitter/latency exceeds `max_skew_ms`
- one of arm/leg/imu is missing (so `complete=False`)
- `sync_hz` differs too much from the actual publish rate

Tune via `aimrl_sdk.open(max_skew_ms=..., sync_hz=...)` and by improving upstream timestamping/clock sync.
