#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
from loguru import logger

import aimrl_sdk
from rl_deploy_config import AppCfg, component_dim, load_app_cfg
from teleop_control import MotionFSM, TeleopInput


def quat_xyzw_to_euler_xyz(q_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = [float(v) for v in q_xyzw]

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw], dtype=np.float32)


def hold_joints_towards_target(cur_pos: np.ndarray, target_pos: np.ndarray, max_delta_pos: float) -> np.ndarray:
    cur = cur_pos.astype(np.float64, copy=False).reshape(-1)
    target = target_pos.astype(np.float64, copy=False).reshape(-1)
    if cur.shape != target.shape:
        raise ValueError(f"shape mismatch: cur={cur.shape} target={target.shape}")
    if max_delta_pos <= 0.0:
        return cur.copy()
    delta = target - cur
    step = np.clip(delta, -max_delta_pos, max_delta_pos)
    return cur + step


class OnnxPolicyRunner:
    def __init__(self, cfg: AppCfg):
        try:
            import onnxruntime as ort
        except Exception as e:
            raise RuntimeError("onnxruntime is required to run model inference") from e

        self.cfg = cfg
        self.session = ort.InferenceSession(str(cfg.model_path), providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

        in_shape = self.session.get_inputs()[0].shape
        if len(in_shape) == 2 and isinstance(in_shape[1], int):
            expect = int(cfg.observation_size) * int(cfg.num_hist)
            if in_shape[1] != expect:
                raise ValueError(f"model input dim mismatch: model={in_shape[1]} cfg={expect}")

        self.last_actions = np.zeros((12,), dtype=np.float32)
        self.hist = np.zeros((cfg.num_hist, cfg.observation_size), dtype=np.float32)
        self.is_first = True
        self.phase_start_time = time.time()
        self.phase = 0.0
        self._last_actions_slices: list[slice] = []

        offset = 0
        for comp in cfg.obs_components:
            dim = component_dim(comp.type)
            if comp.type == "last_actions":
                self._last_actions_slices.append(slice(offset, offset + dim))
            offset += dim
        if offset != cfg.observation_size:
            raise ValueError(f"observation.size mismatch: components sum={offset} cfg={cfg.observation_size}")

    def _update_phase(self, cmd_x: float, cmd_y: float, cmd_yaw: float) -> None:
        if not self.cfg.sw_mode:
            t = time.time() - self.phase_start_time
            self.phase = (t / self.cfg.cycle_time) % 1.0
            return

        cmd_norm = math.sqrt(cmd_x * cmd_x + cmd_y * cmd_y + cmd_yaw * cmd_yaw)
        if cmd_norm <= self.cfg.cmd_threshold:
            self.phase = 0.0
            self.phase_start_time = time.time()
            return

        t = time.time() - self.phase_start_time
        self.phase = (t / self.cfg.cycle_time) % 1.0

    def _build_step_observation(self, obs: np.ndarray, cmd_x: float, cmd_y: float, cmd_yaw: float) -> np.ndarray:
        out_parts: list[np.ndarray] = []
        for comp in self.cfg.obs_components:
            typ = comp.type
            scale = comp.scale

            if typ == "command":
                out_parts.append(
                    np.array(
                        [
                            math.sin(2.0 * math.pi * self.phase),
                            math.cos(2.0 * math.pi * self.phase),
                            cmd_x,
                            cmd_y,
                            cmd_yaw,
                        ],
                        dtype=np.float32,
                    )
                )
            elif typ == "leg_pos":
                leg_pos = obs[aimrl_sdk.OBS.leg_pos].astype(np.float32, copy=False)
                out_parts.append((leg_pos - self.cfg.default_joint_angles) * scale)
            elif typ == "leg_vel":
                leg_vel = obs[aimrl_sdk.OBS.leg_vel].astype(np.float32, copy=False)
                out_parts.append(leg_vel * scale)
            elif typ == "last_actions":
                out_parts.append(self.last_actions)
            elif typ == "imu_gyro":
                imu_gyro = obs[aimrl_sdk.OBS.imu_gyro_xyz].astype(np.float32, copy=False)
                out_parts.append(imu_gyro * scale)
            elif typ == "imu_euler":
                imu_quat = obs[aimrl_sdk.OBS.imu_quat_xyzw].astype(np.float32, copy=False)
                out_parts.append(quat_xyzw_to_euler_xyz(imu_quat) * scale)
            elif typ == "imu_quat":
                imu_quat = obs[aimrl_sdk.OBS.imu_quat_xyzw].astype(np.float32, copy=False)
                out_parts.append(imu_quat * scale)
            else:
                raise ValueError(f"unsupported observation component type: {typ}")

        step_obs = np.concatenate(out_parts, dtype=np.float32)
        if step_obs.shape != (self.cfg.observation_size,):
            raise RuntimeError(
                f"unexpected step observation shape {step_obs.shape}, expected {(self.cfg.observation_size,)}"
            )
        return step_obs

    def step(self, obs: np.ndarray, cmd_x: float, cmd_y: float, cmd_yaw: float) -> np.ndarray:
        self._update_phase(cmd_x, cmd_y, cmd_yaw)
        step_obs = self._build_step_observation(obs, cmd_x, cmd_y, cmd_yaw)

        if self.is_first:
            step0 = step_obs.copy()
            for s in self._last_actions_slices:
                step0[s] = 0.0
            self.hist[:] = step0
            self.is_first = False
        else:
            self.hist[:-1] = self.hist[1:]
            self.hist[-1] = step_obs

        inp = self.hist.reshape(1, -1)
        np.clip(inp, -self.cfg.clip_obs, self.cfg.clip_obs, out=inp)

        out = self.session.run([self.output_name], {self.input_name: inp})[0]
        actions = out.reshape(-1).astype(np.float32, copy=False)
        if actions.shape != (12,):
            raise RuntimeError(f"unexpected actions shape {actions.shape}")

        np.clip(actions, -self.cfg.clip_actions, self.cfg.clip_actions, out=actions)
        self.last_actions = actions.copy()

        return actions * self.cfg.action_scale + self.cfg.default_joint_angles


def _setup_logger() -> None:
    logger.remove()
    logger.add(sys.stderr, format="[{time:YYYY-MM-DD HH:mm:ss.SSS}] [{level}] {message}", level="INFO")


def parse_args() -> argparse.Namespace:
    examples_dir = Path(__file__).resolve().parent
    default_cfg = examples_dir / "configs" / "agibot_a2_dof12.yaml"

    p = argparse.ArgumentParser()
    p.add_argument(
        "--aimrt-backend",
        type=str,
        default="iceoryx",
        choices=["iceoryx", "ros2"],
        help="Select built-in AimRT backend (default: iceoryx)",
    )
    p.add_argument(
        "--aimrt-config-path",
        type=Path,
        default=None,
        help="Custom AimRT backend YAML path (overrides --aimrt-backend)",
    )
    p.add_argument("--control-hz", type=float, default=None)
    p.add_argument("--sync-hz", type=float, default=None)
    p.add_argument("--model", type=Path, default=None)
    p.add_argument("--cfg", type=Path, default=default_cfg)
    p.add_argument("--cmd-x", type=float, default=0.0, help="initial command x (forward)")
    p.add_argument("--cmd-y", type=float, default=0.0, help="initial command y (left/right)")
    p.add_argument("--cmd-yaw", type=float, default=0.0, help="initial command yaw (turn)")
    p.add_argument("--joystick", type=Path, default=Path("/dev/input/js0"), help="Linux joystick device path")
    p.add_argument(
        "--no-fsm",
        action="store_true",
        help="Disable motion FSM; start directly in WALK (policy) mode and only gate motion by deadman",
    )
    p.add_argument("--enable-statistics", action="store_true", help="Enable aimrl_sdk runtime statistics (default off)")
    p.add_argument(
        "--statistics-sample-every",
        type=int,
        default=1,
        help="Statistics sampling: aggregate 1/N events (default: 1)",
    )
    p.add_argument(
        "--statistics-ema-shift",
        type=int,
        default=4,
        help="Statistics EMA shift (alpha=1/2^shift, default: 4)",
    )
    p.add_argument(
        "--statistics-log-every-s",
        type=float,
        default=0.0,
        help="Log statistics snapshot every N seconds (0 disables, default: 0)",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    _setup_logger()

    if not args.cfg.exists():
        raise FileNotFoundError(f"cfg not found: {args.cfg}")

    app_cfg = load_app_cfg(args.cfg, model_override=args.model)
    if not Path(app_cfg.model_path).exists():
        raise FileNotFoundError(f"model not found: {app_cfg.model_path}")

    cmd_x = float(args.cmd_x)
    cmd_y = float(args.cmd_y)
    cmd_yaw = float(args.cmd_yaw)

    control_hz = float(args.control_hz) if args.control_hz is not None else app_cfg.control_hz
    sync_hz = float(args.sync_hz) if args.sync_hz is not None else app_cfg.sync_hz
    policy = OnnxPolicyRunner(app_cfg)

    open_kwargs = dict(
        sync_hz=sync_hz,
        enable_statistics=bool(args.enable_statistics),
        statistics_sample_every=int(args.statistics_sample_every),
        statistics_ema_shift=int(args.statistics_ema_shift),
    )
    if args.aimrt_config_path is not None:
        state, cmd = aimrl_sdk.open(config_path=args.aimrt_config_path, **open_kwargs)
        logger.info(f"AimRT config: {args.aimrt_config_path}")
    else:
        state, cmd = aimrl_sdk.open(aimrt_backend=str(args.aimrt_backend), **open_kwargs)
        logger.info(f"AimRT backend: {args.aimrt_backend}")

    logger.info(f"Opened AimRL SDK successfully (sync_hz={sync_hz})")
    logger.info(f"ONNX policy: {app_cfg.model_path}")
    if args.enable_statistics:
        logger.info(
            "Statistics enabled "
            f"(sample_every={int(args.statistics_sample_every)}, ema_shift={int(args.statistics_ema_shift)})"
        )

    stats_started = not args.enable_statistics
    while True:
        logger.info("Waiting for first aligned frame")
        stamp_ns, aligned, complete, _ = state.wait_frame(timeout_s=1.0)
        if args.enable_statistics and (not stats_started) and complete and stamp_ns > 0:
            state.reset_statistics()
            stats_started = True
            logger.info("Statistics reset; starting collection after first complete frame")
        if aligned and complete and stamp_ns > 0:
            logger.info(f"Received first aligned frame at timestamp: {stamp_ns / 1e9:.3f} s")
            break

    dt = 1.0 / control_hz
    log_every = max(1, int(control_hz))
    arm_target = np.array(app_cfg.arm_default_joint_angles, dtype=np.float64)
    arm_stiffness_nom = np.array(app_cfg.arm_stiffness, dtype=np.float64)
    arm_damping_nom = np.array(app_cfg.arm_damping, dtype=np.float64)
    arm_zero_stiffness = np.zeros(aimrl_sdk.OBS.arm_dof, dtype=np.float64)
    arm_emergency_damping = float(app_cfg.arm_emergency_damping)
    arm_emergency_damping_arr = np.full(aimrl_sdk.OBS.arm_dof, arm_emergency_damping, dtype=np.float64)
    arm_max_delta = float(app_cfg.arm_delta_pos_threshold)
    teleop = TeleopInput(args.joystick, init_cmd=(cmd_x, cmd_y, cmd_yaw))
    fsm = MotionFSM(app_cfg) if not args.no_fsm else None
    emergency_stop = False
    stats_next_log_t = time.monotonic() + max(0.0, float(args.statistics_log_every_s))

    try:
        loop_idx = 0
        last_aligned = True
        last_align_warn_t = 0.0
        while True:
            loop_idx += 1
            stamp_ns, aligned, complete, obs = state.latest_frame()

            if not complete:
                now = time.monotonic()
                if last_aligned or (now - last_align_warn_t) >= 1.0:
                    logger.warning("Latest frame is incomplete (missing arm/leg/imu); using held last observation")
                    last_align_warn_t = now
                last_aligned = False
            elif not aligned:
                now = time.monotonic()
                if last_aligned or (now - last_align_warn_t) >= 1.0:
                    logger.warning("Latest frame is not aligned (skew too large)")
                    last_align_warn_t = now
                last_aligned = False
            elif not last_aligned:
                logger.info("Frames are aligned again")
                last_aligned = True

            snap = teleop.poll()
            if snap.quit_edge:
                raise KeyboardInterrupt
            if fsm is not None:
                fsm.on_input(snap, obs)
            else:
                if snap.emergency_stop_edge:
                    emergency_stop = True
                    logger.warning("Emergency stop set (no-fsm mode); press 'p' to clear")
                if snap.start_control_edge and emergency_stop:
                    emergency_stop = False
                    logger.warning("Emergency stop cleared (no-fsm mode)")

            start_time = time.perf_counter()
            if fsm is not None:
                leg_pos_des, leg_stiffness, leg_damping = fsm.step(obs, policy, snap, dt)
                arm_emergency = bool(fsm.emergency_stop)
            else:
                if emergency_stop:
                    leg_pos_des = obs[aimrl_sdk.OBS.leg_pos].astype(np.float64, copy=False)
                    leg_stiffness = np.zeros(12, dtype=np.float64)
                    leg_damping = np.array(app_cfg.leg_damping, dtype=np.float64)
                    arm_emergency = True
                else:
                    leg_pos_des = policy.step(obs, snap.cmd_x, snap.cmd_y, snap.cmd_yaw).astype(np.float64, copy=False)
                    leg_stiffness = np.array(app_cfg.leg_stiffness, dtype=np.float64)
                    leg_damping = np.array(app_cfg.leg_damping, dtype=np.float64)
                    arm_emergency = False

            arm_pos_cur = obs[aimrl_sdk.OBS.arm_pos].astype(np.float64, copy=False)
            if arm_emergency:
                arm_pos_des = arm_pos_cur
                arm_stiffness = arm_zero_stiffness
                arm_damping = arm_emergency_damping_arr
            else:
                arm_pos_des = hold_joints_towards_target(
                    arm_pos_cur,
                    arm_target,
                    arm_max_delta,
                )
                arm_stiffness = arm_stiffness_nom
                arm_damping = arm_damping_nom
            end_time = time.perf_counter()
            if loop_idx % log_every == 0:
                if fsm is not None:
                    logger.info(
                        f"mode={fsm.mode.value} start={int(fsm.start_control)} emg={int(fsm.emergency_stop)} "
                        f"deadman={int(snap.deadman)} cmd=({snap.cmd_x:.2f},{snap.cmd_y:.2f},{snap.cmd_yaw:.2f}) "
                        f"step_time={(end_time - start_time) * 1000.0:.3f} ms"
                    )
                else:
                    logger.info(
                        f"mode=walk(no-fsm) emg={int(emergency_stop)} deadman={int(snap.deadman)} "
                        f"cmd=({snap.cmd_x:.2f},{snap.cmd_y:.2f},{snap.cmd_yaw:.2f}) "
                        f"step_time={(end_time - start_time) * 1000.0:.3f} ms"
                    )

            # start_time = time.perf_counter()
            cmd.set_leg(position=leg_pos_des, stiffness=leg_stiffness, damping=leg_damping)
            cmd.set_arm(position=arm_pos_des, stiffness=arm_stiffness, damping=arm_damping)
            cmd.commit(stamp_ns=stamp_ns)
            # end_time = time.perf_counter()
            # if loop_idx % log_every == 0:
            #     logger.info(f"commit time: {(end_time - start_time) * 1000.0:.3f} ms")

            if args.enable_statistics and args.statistics_log_every_s > 0.0:
                now = time.monotonic()
                if now >= stats_next_log_t:
                    stats_next_log_t = now + float(args.statistics_log_every_s)
                    s = state.statistics()

                    def _metric_ms(metric: dict) -> str:
                        count = int(metric.get("count", 0))
                        if count <= 0:
                            return "n/a"
                        last_ms = float(metric.get("last_ns", 0)) / 1e6
                        ema_ms = float(metric.get("ema_ns", 0)) / 1e6
                        min_ms = float(metric.get("min_ns", 0)) / 1e6
                        max_ms = float(metric.get("max_ns", 0)) / 1e6
                        return f"ema {ema_ms:7.3f} (min {min_ms:7.3f} max {max_ms:7.3f} last {last_ms:7.3f}, n={count})"

                    def _hz_from_interval(interval_metric: dict) -> str:
                        ema_ns = float(interval_metric.get("ema_ns", 0))
                        if ema_ns <= 0:
                            return "n/a"
                        hz = 1e9 / ema_ns
                        return f"{hz:6.1f}"

                    arm_state = s.get("arm_state", {})
                    leg_state = s.get("leg_state", {})
                    imu_state = s.get("imu", {})

                    arm_delay = _metric_ms(arm_state.get("delay_ns", {}))
                    leg_delay = _metric_ms(leg_state.get("delay_ns", {}))
                    imu_delay = _metric_ms(imu_state.get("delay_ns", {}))

                    arm_jitter = _metric_ms(arm_state.get("interval_jitter_ns", {}))
                    leg_jitter = _metric_ms(leg_state.get("interval_jitter_ns", {}))
                    imu_jitter = _metric_ms(imu_state.get("interval_jitter_ns", {}))

                    arm_hz = _hz_from_interval(arm_state.get("interval_ns", {}))
                    leg_hz = _hz_from_interval(leg_state.get("interval_ns", {}))
                    imu_hz = _hz_from_interval(imu_state.get("interval_ns", {}))

                    pub_arm = _metric_ms(s.get("publish_arm", {}).get("duration_ns", {}))
                    pub_leg = _metric_ms(s.get("publish_leg", {}).get("duration_ns", {}))
                    commit_total = _metric_ms(s.get("commit_total", {}).get("duration_ns", {}))

                    sync = s.get("sync", {})
                    wait_frame = s.get("wait_frame", {})
                    uptime_s = float(s.get("uptime_ns", 0)) / 1e9
                    age_arm = _metric_ms(sync.get("age_arm_ns", {}))
                    age_leg = _metric_ms(sync.get("age_leg_ns", {}))
                    age_imu = _metric_ms(sync.get("age_imu_ns", {}))

                    logger.info(
                        f"statistics (uptime={uptime_s:.1f}s, sample_every={int(s.get('sample_every', 1))}, ema_shift={int(s.get('ema_shift', 4))}):\n"
                        f"  RX rate  (Hz):  arm={arm_hz}  leg={leg_hz}  imu={imu_hz}\n"
                        f"  RX delay (ms):  arm={arm_delay} \n"
                        f"                  leg={leg_delay} \n"
                        f"                  imu={imu_delay} \n"
                        f"  RX jitter(ms):  arm={arm_jitter} \n"
                        f"                  leg={leg_jitter} \n"
                        f"                  imu={imu_jitter} \n"
                        f"  TX cost  (ms):  commit={commit_total} \n"
                        f"                  pub_arm={pub_arm} \n"
                        f"                  pub_leg={pub_leg} \n"
                        f"  SYNC:\n"
                        f"    ticks         : {int(sync.get('tick_total', 0)):,}   (overrun: {int(sync.get('tick_overrun', 0)):,})\n"
                        f"    tick age (ms) : arm={age_arm} \n"
                        f"                    leg={age_leg} \n"
                        f"                    imu={age_imu} \n"
                        f"    frames:\n"
                        f"      written     : {int(sync.get('frame_written', 0)):,}\n"
                        f"      complete    : {int(sync.get('frame_complete', 0)):,}\n"
                        f"      incomplete  : {int(sync.get('frame_incomplete', 0)):,} (missing: arm {int(sync.get('frame_incomplete_missing_arm', 0)):,} | leg {int(sync.get('frame_incomplete_missing_leg', 0)):,} | imu {int(sync.get('frame_incomplete_missing_imu', 0)):,})\n"
                        f"      aligned     : {int(sync.get('frame_aligned', 0)):,}\n"
                        f"      unaligned   : {int(sync.get('frame_unaligned_skew', 0)):,} (skew)\n"
                        f"    wait_frame    : ok {int(wait_frame.get('ok', 0)):,} | timeout {int(wait_frame.get('timeout', 0)):,} | stopped {int(wait_frame.get('stopped', 0)):,}",
                    )
            time.sleep(dt)

    except KeyboardInterrupt:
        logger.info("Keyboard interrupt")
    finally:
        teleop.close()
        aimrl_sdk.close(state)


if __name__ == "__main__":
    main()
