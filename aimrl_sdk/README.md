## Quick start

Minimal RL deployment example:

```bash
export PYTHONPATH=$PWD/aimrl_sdk/src
python aimrl_sdk/examples/rl_deploy_basic.py
```

The example shows how to:
- open the SDK,
- read observation frames,
- run a dummy policy, and
- send joint commands.

If your config file is elsewhere, either:
- pass `config_path=...` to `aimrl_sdk.open()`, or
- set `AIMRL_SDK_CONFIG` to the YAML path.

You can also set the aligned-frame sync frequency via `aimrl_sdk.open(sync_hz=...)`.
