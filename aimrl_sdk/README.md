## Quick start

Minimal RL deployment example:

```bash
export PYTHONPATH=$PWD/aimrl_sdk/src
python aimrl_sdk/examples/rl_deploy_basic.py
```

The example shows how to:
- open the SDK,
- read observation frames,
- run an ONNX policy (same observation stacking as `deploy/rl_controllers`), and
- send joint commands.

If your config file is elsewhere, either:
- pass `config_path=...` to `aimrl_sdk.open()`, or
- set `AIMRL_SDK_CONFIG` to the YAML path.

You can also set the aligned-frame sync frequency via `aimrl_sdk.open(sync_hz=...)`.

By default, `aimrl_sdk.open()` enables the A2 ankle closed-chain conversion:
the `/body_drive/leg_joint_state` toe motors (A/B) are converted to ankle
`(toe_pitch, toe_roll)` in observation frames, and ankle commands are converted
back to motor effort in `commit()`. Use `aimrl_sdk.open(use_closed_ankle=False)`
to work directly in motor space.

Observation slices are available via `aimrl_sdk.OBS`, e.g. `obs[aimrl_sdk.OBS.leg_pos]` (auto-derived from the C++ layout).

To run with the included policy + example config (model path + control hz are in the YAML):

```bash
export PYTHONPATH=$PWD/aimrl_sdk/src
python aimrl_sdk/examples/rl_deploy_basic.py --cfg aimrl_sdk/examples/configs/agibot_a2_dof12.yaml
```
