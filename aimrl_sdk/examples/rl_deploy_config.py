from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

try:
    from omegaconf import MISSING, OmegaConf
except ModuleNotFoundError as e:
    raise RuntimeError("This example requires `omegaconf` (pip package: `omegaconf`).") from e


@dataclass(frozen=True)
class ObsComponent:
    type: str
    scale: float = 1.0


def component_dim(typ: str) -> int:
    if typ == "command":
        return 5
    if typ in ("leg_pos", "leg_vel", "last_actions"):
        return 12
    if typ in ("imu_gyro", "imu_euler"):
        return 3
    if typ == "imu_quat":
        return 4
    raise ValueError(f"unsupported observation component type: {typ}")


@dataclass
class AppCfg:
    model_path: str = MISSING
    control_hz: float = MISSING
    sync_hz: float = MISSING
    action_scale: float = MISSING
    clip_actions: float = MISSING
    clip_obs: float = MISSING
    observation_size: int = MISSING
    num_hist: int = MISSING
    cycle_time: float = MISSING
    cmd_threshold: float = MISSING
    sw_mode: bool = MISSING
    default_joint_angles: list[float] = MISSING  # -> np.ndarray(12,) in validate()
    leg_stiffness: list[float] = MISSING  # -> np.ndarray(12,) in validate()
    leg_damping: list[float] = MISSING  # -> np.ndarray(12,) in validate()
    arm_default_joint_angles: list[float] = field(default_factory=lambda: [0.0] * 14)  # -> np.ndarray(14,)
    arm_stiffness: list[float] = field(
        default_factory=lambda: [100.0, 200.0, 200.0, 100.0, 200.0, 200.0, 200.0] * 2
    )  # -> np.ndarray(14,)
    arm_damping: list[float] = field(default_factory=lambda: [4.0, 0.2, 0.2, 4.0, 0.2, 0.2, 0.2] * 2)  # -> np.ndarray(14,)
    arm_delta_pos_threshold: float = 0.05
    arm_emergency_damping: float = 3.0
    obs_components: list[ObsComponent] = MISSING

    def validate(self, cfg_dir: Path, model_override: Path | None = None) -> AppCfg:
        model_path_s = str(model_override) if model_override is not None else str(self.model_path)
        model_path = resolve_path(cfg_dir, model_path_s)

        control_hz = float(self.control_hz)
        sync_hz = float(self.sync_hz)
        if control_hz <= 0.0:
            raise ValueError("control.hz must be > 0")
        if sync_hz <= 0.0:
            raise ValueError("control.sync_hz must be > 0")

        action_scale = float(self.action_scale)
        clip_actions = float(self.clip_actions)
        clip_obs = float(self.clip_obs)
        if clip_actions <= 0.0:
            raise ValueError("policy.clip_actions must be > 0")
        if clip_obs <= 0.0:
            raise ValueError("observation.clip must be > 0")

        observation_size = int(self.observation_size)
        num_hist = int(self.num_hist)
        if observation_size <= 0:
            raise ValueError("observation.size must be > 0")
        if num_hist <= 0:
            raise ValueError("observation.num_hist must be > 0")

        cycle_time = float(self.cycle_time)
        cmd_threshold = float(self.cmd_threshold)
        sw_mode = bool(self.sw_mode)
        if cycle_time <= 0.0:
            raise ValueError("phase.cycle_time must be > 0")
        if cmd_threshold < 0.0:
            raise ValueError("phase.cmd_threshold must be >= 0")

        default_joint_angles = as_floats(self.default_joint_angles, 12, "robot.leg_default_joint_angles")
        leg_stiffness = as_floats(self.leg_stiffness, 12, "robot.leg_stiffness")
        leg_damping = as_floats(self.leg_damping, 12, "robot.leg_damping")

        arm_default_joint_angles = as_floats(self.arm_default_joint_angles, 14, "robot.arm_default_joint_angles")
        arm_stiffness = as_floats(self.arm_stiffness, 14, "robot.arm_stiffness")
        arm_damping = as_floats(self.arm_damping, 14, "robot.arm_damping")

        arm_delta_pos_threshold = float(self.arm_delta_pos_threshold)
        arm_emergency_damping = float(self.arm_emergency_damping)
        if arm_delta_pos_threshold < 0.0:
            raise ValueError("robot.arm_delta_pos_threshold must be >= 0")
        if arm_emergency_damping < 0.0:
            raise ValueError("robot.arm_emergency_damping must be >= 0")

        obs_components_raw = self.obs_components
        if not isinstance(obs_components_raw, list) or not obs_components_raw:
            raise ValueError("observation.components must be a non-empty list")
        obs_components: list[ObsComponent] = []
        for i, item in enumerate(obs_components_raw):
            if isinstance(item, ObsComponent):
                obs_components.append(item)
            elif isinstance(item, dict):
                if "type" not in item:
                    raise KeyError(f"observation.components[{i}] missing required key `type`")
                obs_components.append(ObsComponent(type=str(item["type"]), scale=float(item.get("scale", 1.0))))
            else:
                raise TypeError(f"observation.components[{i}] must be a dict, got {type(item).__name__}")

        offset = 0
        for comp in obs_components:
            offset += component_dim(comp.type)
        if offset != observation_size:
            raise ValueError(f"observation.size mismatch: components sum={offset} cfg={observation_size}")

        return AppCfg(
            model_path=str(model_path),
            control_hz=control_hz,
            sync_hz=sync_hz,
            action_scale=action_scale,
            clip_actions=clip_actions,
            clip_obs=clip_obs,
            observation_size=observation_size,
            num_hist=num_hist,
            cycle_time=cycle_time,
            cmd_threshold=cmd_threshold,
            sw_mode=sw_mode,
            default_joint_angles=default_joint_angles,  # type: ignore[arg-type]
            leg_stiffness=leg_stiffness,  # type: ignore[arg-type]
            leg_damping=leg_damping,  # type: ignore[arg-type]
            arm_default_joint_angles=arm_default_joint_angles,  # type: ignore[arg-type]
            arm_stiffness=arm_stiffness,  # type: ignore[arg-type]
            arm_damping=arm_damping,  # type: ignore[arg-type]
            arm_delta_pos_threshold=arm_delta_pos_threshold,
            arm_emergency_damping=arm_emergency_damping,
            obs_components=obs_components,
        )


@dataclass
class ControlCfg:
    hz: float = MISSING
    sync_hz: float | None = None


@dataclass
class ModelCfg:
    path: str = MISSING


@dataclass
class RobotCfg:
    leg_default_joint_angles: list[float] = MISSING
    leg_stiffness: list[float] = MISSING
    leg_damping: list[float] = MISSING
    arm_default_joint_angles: list[float] = field(default_factory=lambda: [0.0] * 14)
    arm_stiffness: list[float] = field(default_factory=lambda: [100.0, 200.0, 200.0, 100.0, 200.0, 200.0, 200.0] * 2)
    arm_damping: list[float] = field(default_factory=lambda: [4.0, 0.2, 0.2, 4.0, 0.2, 0.2, 0.2] * 2)
    arm_delta_pos_threshold: float = 0.05
    arm_emergency_damping: float = 3.0


@dataclass
class PolicyCfg:
    action_scale: float = MISSING
    clip_actions: float = MISSING


@dataclass
class PhaseCfg:
    cycle_time: float = MISSING
    sw_mode: bool = MISSING
    cmd_threshold: float = MISSING


@dataclass
class ObservationCfg:
    size: int = MISSING
    num_hist: int = MISSING
    clip: float = MISSING
    components: list[ObsComponent] = MISSING


@dataclass
class AppCfgFile:
    control: ControlCfg = field(default_factory=ControlCfg)
    model: ModelCfg = field(default_factory=ModelCfg)
    robot: RobotCfg = field(default_factory=RobotCfg)
    policy: PolicyCfg = field(default_factory=PolicyCfg)
    phase: PhaseCfg = field(default_factory=PhaseCfg)
    observation: ObservationCfg = field(default_factory=ObservationCfg)


def resolve_path(base: Path, p: str) -> Path:
    path = Path(p)
    return path if path.is_absolute() else (base / path)


def as_floats(x: object, n: int, what: str) -> np.ndarray:
    arr: np.ndarray
    if isinstance(x, np.ndarray):
        arr = x.astype(np.float32, copy=False).reshape(-1)
    elif isinstance(x, list):
        arr = np.array([float(v) for v in x], dtype=np.float32)
    else:
        raise TypeError(f"{what} must be a list[{n}]")
    if arr.shape != (n,):
        raise ValueError(f"{what} must be a list[{n}]")
    return arr


def load_app_cfg(cfg_path: Path, model_override: Path | None = None) -> AppCfg:
    raw = OmegaConf.load(str(cfg_path))
    raw_container = OmegaConf.to_container(raw, resolve=False)
    if isinstance(raw_container, dict) and ("LeggedRobotCfg" in raw_container or "rl_controllers" in raw_container):
        raise KeyError(
            "deploy-style config is not supported by this example anymore; "
            "please use the example schema in `aimrl_sdk/examples/configs/agibot_a2_dof12.yaml`"
        )

    cfg_file = OmegaConf.merge(OmegaConf.structured(AppCfgFile), raw)
    cfg_obj: AppCfgFile = OmegaConf.to_object(cfg_file)

    control_hz = float(cfg_obj.control.hz)
    sync_hz = float(cfg_obj.control.sync_hz) if cfg_obj.control.sync_hz is not None else control_hz

    app_dict = {
        "model_path": str(cfg_obj.model.path),
        "control_hz": control_hz,
        "sync_hz": sync_hz,
        "action_scale": float(cfg_obj.policy.action_scale),
        "clip_actions": float(cfg_obj.policy.clip_actions),
        "clip_obs": float(cfg_obj.observation.clip),
        "observation_size": int(cfg_obj.observation.size),
        "num_hist": int(cfg_obj.observation.num_hist),
        "cycle_time": float(cfg_obj.phase.cycle_time),
        "cmd_threshold": float(cfg_obj.phase.cmd_threshold),
        "sw_mode": bool(cfg_obj.phase.sw_mode),
        "default_joint_angles": list(cfg_obj.robot.leg_default_joint_angles),
        "leg_stiffness": list(cfg_obj.robot.leg_stiffness),
        "leg_damping": list(cfg_obj.robot.leg_damping),
        "arm_default_joint_angles": list(cfg_obj.robot.arm_default_joint_angles),
        "arm_stiffness": list(cfg_obj.robot.arm_stiffness),
        "arm_damping": list(cfg_obj.robot.arm_damping),
        "arm_delta_pos_threshold": float(cfg_obj.robot.arm_delta_pos_threshold),
        "arm_emergency_damping": float(cfg_obj.robot.arm_emergency_damping),
        "obs_components": list(cfg_obj.observation.components),
    }

    cfg_app = OmegaConf.merge(OmegaConf.structured(AppCfg), OmegaConf.create(app_dict))
    cfg: AppCfg = OmegaConf.to_object(cfg_app)
    return cfg.validate(cfg_dir=cfg_path.parent, model_override=model_override)
