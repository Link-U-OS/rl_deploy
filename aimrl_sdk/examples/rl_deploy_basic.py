#!/usr/bin/env python3
import os
import time

import numpy as np

import aimrl_sdk


def policy(obs: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Dummy policy: replace with your RL model inference."""
    leg_pos = np.zeros(12, dtype=np.float64)
    arm_pos = np.zeros(14, dtype=np.float64)
    return leg_pos, arm_pos


def main() -> None:
    state, cmd = aimrl_sdk.open()

    try:
        while True:
            try:
                stamp_ns, valid, obs = state.latest_frame()
                print(f"stamp_ns: {stamp_ns}, valid: {valid}")
            except RuntimeError:
                continue

            leg_pos, arm_pos = policy(obs)

            # Set command fields. Scalars apply to all joints.
            cmd.set_leg(position=leg_pos, stiffness=100.0, damping=1.0)
            cmd.set_arm(position=arm_pos, stiffness=100.0, damping=1.0)
            cmd.commit(stamp_ns=stamp_ns)

            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        aimrl_sdk.close(state)


if __name__ == "__main__":
    main()
