from __future__ import annotations

from dataclasses import dataclass
from typing import Any


def _get_int_attr(mod: Any, name: str, default: int) -> int:
    v = getattr(mod, name, default)
    return int(v)


@dataclass(frozen=True)
class ObsSlices:
    dim: int
    arm_dof: int
    leg_dof: int

    arm_pos0: int
    arm_vel0: int
    arm_eff0: int
    leg_pos0: int
    leg_vel0: int
    leg_eff0: int
    imu_quat0: int
    imu_gyro0: int
    imu_acc0: int

    @classmethod
    def from_bindings(cls, bindings: Any) -> "ObsSlices":
        dim = _get_int_attr(bindings, "FRAME_DIM", 88)
        arm_dof = _get_int_attr(bindings, "ARM_DOF", 14)
        leg_dof = _get_int_attr(bindings, "LEG_DOF", 12)
        return cls(
            dim=dim,
            arm_dof=arm_dof,
            leg_dof=leg_dof,
            arm_pos0=_get_int_attr(bindings, "ARM_POS0", 0),
            arm_vel0=_get_int_attr(bindings, "ARM_VEL0", 14),
            arm_eff0=_get_int_attr(bindings, "ARM_EFF0", 28),
            leg_pos0=_get_int_attr(bindings, "LEG_POS0", 42),
            leg_vel0=_get_int_attr(bindings, "LEG_VEL0", 54),
            leg_eff0=_get_int_attr(bindings, "LEG_EFF0", 66),
            imu_quat0=_get_int_attr(bindings, "IMU_QUAT0", 78),
            imu_gyro0=_get_int_attr(bindings, "IMU_GYRO0", 82),
            imu_acc0=_get_int_attr(bindings, "IMU_ACC0", 85),
        )

    def _validate(self) -> None:
        if self.dim <= 0:
            raise ValueError("obs dim must be > 0")
        if self.arm_dof <= 0 or self.leg_dof <= 0:
            raise ValueError("arm/leg dof must be > 0")

        segments: list[tuple[str, int, int]] = [
            ("arm_pos", self.arm_pos0, self.arm_pos0 + self.arm_dof),
            ("arm_vel", self.arm_vel0, self.arm_vel0 + self.arm_dof),
            ("arm_eff", self.arm_eff0, self.arm_eff0 + self.arm_dof),
            ("leg_pos", self.leg_pos0, self.leg_pos0 + self.leg_dof),
            ("leg_vel", self.leg_vel0, self.leg_vel0 + self.leg_dof),
            ("leg_eff", self.leg_eff0, self.leg_eff0 + self.leg_dof),
            ("imu_quat", self.imu_quat0, self.imu_quat0 + 4),
            ("imu_gyro", self.imu_gyro0, self.imu_gyro0 + 3),
            ("imu_acc", self.imu_acc0, self.imu_acc0 + 3),
        ]

        for name, start, end in segments:
            if start < 0 or end < 0 or end < start:
                raise ValueError(f"invalid {name} segment: [{start}, {end})")
            if end > self.dim:
                raise ValueError(f"{name} segment exceeds dim: end={end} dim={self.dim}")

        for i in range(len(segments)):
            ni, si, ei = segments[i]
            for j in range(i + 1, len(segments)):
                nj, sj, ej = segments[j]
                if max(si, sj) < min(ei, ej):
                    raise ValueError(f"overlapping segments: {ni} [{si},{ei}) and {nj} [{sj},{ej})")

        if self.arm_dof % 2 != 0:
            raise ValueError("arm_dof must be even for left/right splits")
        if self.leg_dof % 2 != 0:
            raise ValueError("leg_dof must be even for left/right splits")

    @property
    def arm_pos(self) -> slice:
        return slice(self.arm_pos0, self.arm_pos0 + self.arm_dof)

    @property
    def arm_vel(self) -> slice:
        return slice(self.arm_vel0, self.arm_vel0 + self.arm_dof)

    @property
    def arm_eff(self) -> slice:
        return slice(self.arm_eff0, self.arm_eff0 + self.arm_dof)

    @property
    def leg_pos(self) -> slice:
        return slice(self.leg_pos0, self.leg_pos0 + self.leg_dof)

    @property
    def leg_vel(self) -> slice:
        return slice(self.leg_vel0, self.leg_vel0 + self.leg_dof)

    @property
    def leg_eff(self) -> slice:
        return slice(self.leg_eff0, self.leg_eff0 + self.leg_dof)

    @property
    def imu_quat_xyzw(self) -> slice:
        return slice(self.imu_quat0, self.imu_quat0 + 4)

    @property
    def imu_gyro_xyz(self) -> slice:
        return slice(self.imu_gyro0, self.imu_gyro0 + 3)

    @property
    def imu_acc_xyz(self) -> slice:
        return slice(self.imu_acc0, self.imu_acc0 + 3)

    @property
    def arm_left_pos(self) -> slice:
        half = self.arm_dof // 2
        return slice(self.arm_pos0, self.arm_pos0 + half)

    @property
    def arm_left_vel(self) -> slice:
        half = self.arm_dof // 2
        return slice(self.arm_vel0, self.arm_vel0 + half)

    @property
    def arm_left_eff(self) -> slice:
        half = self.arm_dof // 2
        return slice(self.arm_eff0, self.arm_eff0 + half)

    @property
    def arm_right_pos(self) -> slice:
        half = self.arm_dof // 2
        return slice(self.arm_pos0 + half, self.arm_pos0 + self.arm_dof)

    @property
    def arm_right_vel(self) -> slice:
        half = self.arm_dof // 2
        return slice(self.arm_vel0 + half, self.arm_vel0 + self.arm_dof)

    @property
    def arm_right_eff(self) -> slice:
        half = self.arm_dof // 2
        return slice(self.arm_eff0 + half, self.arm_eff0 + self.arm_dof)

    @property
    def leg_left_pos(self) -> slice:
        half = self.leg_dof // 2
        return slice(self.leg_pos0, self.leg_pos0 + half)

    @property
    def leg_left_vel(self) -> slice:
        half = self.leg_dof // 2
        return slice(self.leg_vel0, self.leg_vel0 + half)

    @property
    def leg_left_eff(self) -> slice:
        half = self.leg_dof // 2
        return slice(self.leg_eff0, self.leg_eff0 + half)

    @property
    def leg_right_pos(self) -> slice:
        half = self.leg_dof // 2
        return slice(self.leg_pos0 + half, self.leg_pos0 + self.leg_dof)

    @property
    def leg_right_vel(self) -> slice:
        half = self.leg_dof // 2
        return slice(self.leg_vel0 + half, self.leg_vel0 + self.leg_dof)

    @property
    def leg_right_eff(self) -> slice:
        half = self.leg_dof // 2
        return slice(self.leg_eff0 + half, self.leg_eff0 + self.leg_dof)

    def split(self, obs: Any) -> dict[str, Any]:
        return {
            "arm_pos": obs[self.arm_pos],
            "arm_vel": obs[self.arm_vel],
            "arm_eff": obs[self.arm_eff],
            "leg_pos": obs[self.leg_pos],
            "leg_vel": obs[self.leg_vel],
            "leg_eff": obs[self.leg_eff],
            "imu_quat_xyzw": obs[self.imu_quat_xyzw],
            "imu_gyro_xyz": obs[self.imu_gyro_xyz],
            "imu_acc_xyz": obs[self.imu_acc_xyz],
        }


def _make_default_obs_slices() -> ObsSlices:
    from . import _bindings

    obs = ObsSlices.from_bindings(_bindings)
    obs._validate()
    return obs


OBS = _make_default_obs_slices()
