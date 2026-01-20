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
    p.add_argument("--control-hz", type=float, default=None)
    p.add_argument("--sync-hz", type=float, default=None)
    p.add_argument("--model", type=Path, default=None)
    p.add_argument("--cfg", type=Path, default=default_cfg)
    p.add_argument("--cmd-x", type=float, default=0.0)
    p.add_argument("--cmd-y", type=float, default=0.0)
    p.add_argument("--cmd-yaw", type=float, default=0.0)
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

    state, cmd = aimrl_sdk.open(sync_hz=sync_hz)
    logger.info(f"Opened AimRL SDK successfully (sync_hz={sync_hz})")
    logger.info(f"ONNX policy: {app_cfg.model_path}")

    while True:
        logger.info("Waiting for first aligned frame")
        stamp_ns, aligned, _ = state.wait_frame(timeout_s=1.0)
        if aligned and stamp_ns > 0:
            logger.info(f"Received first aligned frame at timestamp: {stamp_ns / 1e9:.3f} s")
            break

    dt = 1.0 / control_hz
    log_every = max(1, int(control_hz))
    arm_zero = np.zeros(14, dtype=np.float64)

    try:
        loop_idx = 0
        while True:
            loop_idx += 1
            stamp_ns, aligned, obs = state.latest_frame()

            if not aligned:
                logger.warning("Latest frame is not aligned")

            start_time = time.time()
            leg_pos_des = policy.step(obs, cmd_x, cmd_y, cmd_yaw).astype(np.float64)
            end_time = time.time()
            if loop_idx % log_every == 0:
                logger.info(f"policy time: {(end_time - start_time) * 1000.0:.3f} ms")

            start_time = time.time()
            cmd.set_leg(position=leg_pos_des, stiffness=app_cfg.leg_stiffness, damping=app_cfg.leg_damping)
            cmd.set_arm(position=arm_zero, stiffness=0.0, damping=0.0)
            cmd.commit(stamp_ns=stamp_ns)
            end_time = time.time()
            if loop_idx % log_every == 0:
                logger.info(f"commit time: {(end_time - start_time) * 1000.0:.3f} ms")

            time.sleep(dt)

    except KeyboardInterrupt:
        pass
    finally:
        aimrl_sdk.close(state)


if __name__ == "__main__":
    main()
