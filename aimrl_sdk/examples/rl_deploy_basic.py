#!/usr/bin/env python3
import sys
import time

import numpy as np
from loguru import logger

import aimrl_sdk

CONTROL_HZ = 100


def policy(obs: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Dummy policy: replace with your RL model inference."""
    leg_pos = np.zeros(12, dtype=np.float64)
    arm_pos = np.zeros(14, dtype=np.float64)
    return leg_pos, arm_pos


def main() -> None:
    logger.remove()
    logger.add(sys.stderr, format="[{time:YYYY-MM-DD HH:mm:ss.SSS}] [{level}] {message}", level="INFO")

    state, cmd = aimrl_sdk.open(sync_hz=CONTROL_HZ, max_skew_ms=2.0)
    logger.info("Opened AimRL SDK successfully")

    while True:
        logger.info("Waiting for first aligned frame")
        stamp_ns, aligned, _ = state.wait_frame(timeout_s=1.0)
        if aligned and stamp_ns > 0:
            logger.info("Received first aligned frame at timestamp: {:.3f} s", stamp_ns / 1e9)
            break

    try:
        loop_idx = 0
        while True:
            loop_idx += 1

            try:
                stamp_ns, aligned, obs = state.latest_frame()
            except RuntimeError as e:
                logger.error("Error getting latest frame: {}", e)
                continue

            if not aligned:
                logger.warning("Latest frame is not aligned")

            if loop_idx % CONTROL_HZ == 0:
                logger.info(
                    "Loop {}: timestamp: {:.3f} s, aligned: {}",
                    loop_idx,
                    stamp_ns / 1e9,
                    aligned,
                )

            leg_pos, arm_pos = policy(obs)

            # Set command fields. Scalars apply to all joints.
            start_time = time.time()
            cmd.set_leg(position=leg_pos, stiffness=100.0, damping=1.0)
            cmd.set_arm(position=arm_pos, stiffness=100.0, damping=1.0)
            cmd.commit(stamp_ns=stamp_ns)
            end_time = time.time()
            # if loop_idx % CONTROL_HZ == 0:
            #     logger.info("commit time: {:.3f} ms", (end_time - start_time) * 1000.0)

            time.sleep(1.0 / CONTROL_HZ)

    except KeyboardInterrupt:
        pass
    finally:
        aimrl_sdk.close(state)


if __name__ == "__main__":
    main()
