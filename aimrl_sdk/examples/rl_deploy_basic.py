#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from loguru import logger

import aimrl_sdk


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


@dataclass
class AppCfg:
    cfg_path: Path
    model_path: Path
    control_hz: float
    sync_hz: float
    action_scale: float
    clip_actions: float
    clip_obs: float
    observation_size: int
    num_hist: int
    cycle_time: float
    cmd_threshold: float
    sw_mode: bool
    default_joint_angles: np.ndarray  # (12,)
    leg_stiffness: np.ndarray  # (12,)
    leg_damping: np.ndarray  # (12,)
    obs_components: list[object]


def _resolve_path(base: Path, p: str) -> Path:
    path = Path(p)
    return path if path.is_absolute() else (base / path)


def _as_floats(x: object, n: int, what: str) -> np.ndarray:
    if not isinstance(x, list) or len(x) != n:
        raise ValueError(f"{what} must be a list[{n}]")
    return np.array([float(v) for v in x], dtype=np.float32)


def load_app_cfg(cfg_path: Path, model_override: Path | None = None) -> AppCfg:
    cfg = yaml.safe_load(cfg_path.read_text())
    base = cfg_path.parent

    if isinstance(cfg, dict) and "model" in cfg:
        control = cfg["control"]
        model = cfg["model"]
        robot = cfg["robot"]
        policy = cfg["policy"]
        phase = cfg["phase"]
        observation = cfg["observation"]

        model_path = model_override if model_override is not None else _resolve_path(base, str(model["path"]))
        control_hz = float(control["hz"])
        sync_hz = float(control.get("sync_hz", control_hz))

        default_joint_angles = _as_floats(robot["leg_default_joint_angles"], 12, "robot.leg_default_joint_angles")
        leg_stiffness = _as_floats(robot["leg_stiffness"], 12, "robot.leg_stiffness")
        leg_damping = _as_floats(robot["leg_damping"], 12, "robot.leg_damping")

        observation_size = int(observation["size"])
        num_hist = int(observation["num_hist"])
        clip_obs = float(observation["clip"])
        obs_components = list(observation["components"])

        return AppCfg(
            cfg_path=cfg_path,
            model_path=model_path,
            control_hz=control_hz,
            sync_hz=sync_hz,
            action_scale=float(policy["action_scale"]),
            clip_actions=float(policy["clip_actions"]),
            clip_obs=clip_obs,
            observation_size=observation_size,
            num_hist=num_hist,
            cycle_time=float(phase["cycle_time"]),
            cmd_threshold=float(phase["cmd_threshold"]),
            sw_mode=bool(phase["sw_mode"]),
            default_joint_angles=default_joint_angles,
            leg_stiffness=leg_stiffness,
            leg_damping=leg_damping,
            obs_components=obs_components,
        )

    # Backward compatible: deploy-style config.
    if "LeggedRobotCfg" in cfg:
        p = cfg["LeggedRobotCfg"]
    else:
        p = cfg.get("rl_controllers", {}).get("ros__parameters", {}).get("LeggedRobotCfg")
        if p is None:
            raise KeyError("Missing config root (supported: new schema with `model`, or deploy-style `LeggedRobotCfg`)")

    joints = [
        "idx01_left_hip_roll",
        "idx02_left_hip_yaw",
        "idx03_left_hip_pitch",
        "idx04_left_tarsus",
        "idx05_left_toe_pitch",
        "idx06_left_toe_roll",
        "idx07_right_hip_roll",
        "idx08_right_hip_yaw",
        "idx09_right_hip_pitch",
        "idx10_right_tarsus",
        "idx11_right_toe_pitch",
        "idx12_right_toe_roll",
    ]

    size = p["size"]
    control_cfg = p["control"]
    mode = p["mode"]
    obs_scales = p["normalization"]["obs_scales"]
    clip = p["normalization"]["clip_scales"]

    default_joint_angle = p["init_state"]["default_joint_angle"]
    default_joint_angles = np.array([float(default_joint_angle[j]) for j in joints], dtype=np.float32)
    stiffness = control_cfg["stiffness"]
    damping = control_cfg["damping"]
    leg_stiffness = np.array([float(stiffness[j]) for j in joints], dtype=np.float32)
    leg_damping = np.array([float(damping[j]) for j in joints], dtype=np.float32)

    observation_size = int(size["observations_size"])
    num_hist = int(size["num_hist"])
    obs_components = [
        {"type": "command"},
        {"type": "leg_pos", "scale": float(obs_scales["dof_pos"])},
        {"type": "leg_vel", "scale": float(obs_scales["dof_vel"])},
        {"type": "last_actions"},
        {"type": "imu_gyro", "scale": float(obs_scales["ang_vel"])},
        {"type": "imu_euler", "scale": float(obs_scales["quat"])},
    ]

    bundled_model = Path(__file__).resolve().parent / "policy" / "model.onnx"
    model_path = model_override if model_override is not None else bundled_model
    return AppCfg(
        cfg_path=cfg_path,
        model_path=model_path,
        control_hz=100.0,
        sync_hz=100.0,
        action_scale=float(control_cfg["action_scale"]),
        clip_actions=float(clip["clip_actions"]),
        clip_obs=float(clip["clip_observations"]),
        observation_size=observation_size,
        num_hist=num_hist,
        cycle_time=float(control_cfg["cycle_time"]),
        cmd_threshold=float(mode["cmd_threshold"]),
        sw_mode=bool(mode["sw_mode"]),
        default_joint_angles=default_joint_angles,
        leg_stiffness=leg_stiffness,
        leg_damping=leg_damping,
        obs_components=obs_components,
    )


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
        for spec in cfg.obs_components:
            if isinstance(spec, str):
                typ = spec
            elif isinstance(spec, dict):
                typ = str(spec["type"])
            else:
                raise TypeError(f"invalid observation component spec: {spec!r}")
            dim = self._component_dim(typ)
            if typ == "last_actions":
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

    @staticmethod
    def _component_dim(typ: str) -> int:
        if typ == "command":
            return 5
        if typ in ("leg_pos", "leg_vel", "last_actions"):
            return 12
        if typ in ("imu_gyro", "imu_euler"):
            return 3
        if typ == "imu_quat":
            return 4
        raise ValueError(f"unsupported observation component type: {typ}")

    def _build_step_observation(self, obs: np.ndarray, cmd_x: float, cmd_y: float, cmd_yaw: float) -> np.ndarray:
        out_parts: list[np.ndarray] = []
        for spec in self.cfg.obs_components:
            if isinstance(spec, str):
                typ = spec
                scale = 1.0
            elif isinstance(spec, dict):
                typ = str(spec["type"])
                scale = float(spec.get("scale", 1.0))
            else:
                raise TypeError(f"invalid observation component spec: {spec!r}")

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
            raise RuntimeError(f"unexpected step observation shape {step_obs.shape}, expected {(self.cfg.observation_size,)}")
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

    logger.remove()
    logger.add(sys.stderr, format="[{time:YYYY-MM-DD HH:mm:ss.SSS}] [{level}] {message}", level="INFO")

    if not args.cfg.exists():
        raise FileNotFoundError(f"cfg not found: {args.cfg}")

    cfg = load_app_cfg(args.cfg, model_override=args.model)
    if not cfg.model_path.exists():
        raise FileNotFoundError(f"model not found: {cfg.model_path}")

    control_hz = float(args.control_hz) if args.control_hz is not None else cfg.control_hz
    sync_hz = float(args.sync_hz) if args.sync_hz is not None else cfg.sync_hz
    policy = OnnxPolicyRunner(cfg)

    state, cmd = aimrl_sdk.open(sync_hz=sync_hz)
    logger.info(f"Opened AimRL SDK successfully (sync_hz={sync_hz})")
    logger.info(f"ONNX policy: {cfg.model_path}")

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
            stamp_ns, _, obs = state.latest_frame()

            start_time = time.time()
            leg_pos_des = policy.step(obs, args.cmd_x, args.cmd_y, args.cmd_yaw).astype(np.float64)
            end_time = time.time()
            if loop_idx % log_every == 0:
                logger.info(f"policy time: {(end_time - start_time) * 1000.0:.3f} ms")

            start_time = time.time()
            cmd.set_leg(position=leg_pos_des, stiffness=cfg.leg_stiffness, damping=cfg.leg_damping)
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
