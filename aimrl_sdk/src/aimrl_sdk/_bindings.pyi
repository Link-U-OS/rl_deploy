from __future__ import annotations
import typing
__all__: list[str] = ['ARM_DOF', 'ARM_EFF0', 'ARM_POS0', 'ARM_VEL0', 'CommandInterface', 'FRAME_DIM', 'IMU_ACC0', 'IMU_GYRO0', 'IMU_QUAT0', 'LEG_DOF', 'LEG_EFF0', 'LEG_POS0', 'LEG_VEL0', 'StateInterface', 'close', 'open']
class CommandInterface:
    def commit(self, stamp_ns: typing.Any = None, sequence: typing.Any = None) -> None:
        ...
    def configure_statistics(self, enabled: bool, sample_every: typing.SupportsInt = 1, ema_shift: typing.SupportsInt = 4) -> None:
        ...
    def reset_statistics(self) -> None:
        ...
    def set_arm(self, position: typing.Any = None, velocity: typing.Any = None, effort: typing.Any = None, stiffness: typing.Any = None, damping: typing.Any = None) -> None:
        ...
    def set_leg(self, position: typing.Any = None, velocity: typing.Any = None, effort: typing.Any = None, stiffness: typing.Any = None, damping: typing.Any = None) -> None:
        ...
    def statistics(self) -> dict:
        ...
class StateInterface:
    def configure_statistics(self, enabled: bool, sample_every: typing.SupportsInt = 1, ema_shift: typing.SupportsInt = 4) -> None:
        ...
    def latest_frame(self) -> tuple:
        ...
    def read_frames(self, arg0: typing.SupportsInt) -> tuple:
        ...
    def reset_statistics(self) -> None:
        ...
    def statistics(self) -> dict:
        ...
    def wait_frame(self, timeout_s: typing.Any = None) -> tuple:
        ...
def close(handle: typing.Any = None) -> None:
    ...
def open(config_path: typing.Any = None, sync_hz: typing.SupportsFloat = 100.0, max_skew_ms: typing.SupportsFloat = 3.0, max_backtrack: typing.SupportsInt = 200, raw_ring: typing.SupportsInt = 2048, frame_ring: typing.SupportsInt = 512, arm_names: typing.Any = None, leg_names: typing.Any = None, use_closed_ankle: bool = True, ankle_torque_control: bool = True, ankle_motor1_direction: typing.SupportsInt = 1, ankle_motor2_direction: typing.SupportsInt = 1, ankle_pitch_direction: typing.SupportsInt = 1, ankle_roll_direction: typing.SupportsInt = 1, ankle_d: typing.SupportsFloat = 0.0315, ankle_l: typing.SupportsFloat = 0.063, ankle_h1: typing.SupportsFloat = 0.239, ankle_h2: typing.SupportsFloat = 0.145, ankle_actuator_pos_limit: typing.SupportsFloat = 1.0, ankle_pitch_limit: typing.SupportsFloat = 1.0, ankle_roll_limit: typing.SupportsFloat = 0.5, enable_statistics: bool = False, statistics_sample_every: typing.SupportsInt = 1, statistics_ema_shift: typing.SupportsInt = 4) -> tuple:
    """
    Open the AimRL SDK and return `(state, cmd)`.
    
      Args:
      config_path: Optional path to the AimRT backend YAML. If None/empty, uses
        `AIMRL_SDK_CONFIG` (if set) or the built-in default.
      sync_hz: Frame synchronization frequency (Hz) for generating aligned frames.
      max_skew_ms: Max allowed timestamp skew (ms) for a frame to be marked `aligned`.
      max_backtrack: Max samples to scan backward per tick to find `<= tick` samples.
      raw_ring: Raw sample ring capacity (arm/leg/imu).
      frame_ring: Aligned frame ring capacity.
      arm_names: Optional list[str] of length 14 for command joint names.
      leg_names: Optional list[str] of length 12 for command joint names.
      use_closed_ankle: If True, convert the ankle closed-chain motors (toe A/B)
        to ankle (pitch,roll) for frames, and convert commands back to motors.
      ankle_torque_control: If True and `use_closed_ankle`, ankle motors are
        commanded in effort (torque) based on (pitch,roll) PD in `commit()`.
      ankle_*: Closed-chain ankle geometry/sign parameters (advanced).
      enable_statistics: Enable lightweight runtime statistics counters/latency/jitter.
      statistics_sample_every: Sample 1/N events when aggregating statistics.
      statistics_ema_shift: EMA smoothing shift (alpha = 1/2^shift).
    """
ARM_DOF: int = 14
ARM_EFF0: int = 28
ARM_POS0: int = 0
ARM_VEL0: int = 14
FRAME_DIM: int = 88
IMU_ACC0: int = 85
IMU_GYRO0: int = 82
IMU_QUAT0: int = 78
LEG_DOF: int = 12
LEG_EFF0: int = 66
LEG_POS0: int = 42
LEG_VEL0: int = 54
