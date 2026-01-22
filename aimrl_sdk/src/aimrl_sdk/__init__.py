import os
import sys
from pathlib import Path
from typing import Any, Tuple

if sys.platform != "linux":
    raise RuntimeError(f"aimrl_sdk only supports Linux (sys.platform={sys.platform!r}).")

from . import _bindings as _bindings
from .obs import OBS as OBS
from .obs import ObsSlices as ObsSlices


def _ensure_runtime_env() -> None:
    plugin_dir = getattr(_bindings, "__file__", "")
    if plugin_dir:
        plugin_dir = os.path.dirname(plugin_dir)
    if not plugin_dir:
        return

    os.environ.setdefault("AIMRT_PLUGIN_DIR", plugin_dir)

    ld_path = os.environ.get("LD_LIBRARY_PATH", "")
    parts = [p for p in ld_path.split(":") if p] if ld_path else []
    if plugin_dir not in parts:
        os.environ["LD_LIBRARY_PATH"] = ":".join([plugin_dir] + parts)

    _preload_typesupport_libraries(plugin_dir)


def _preload_typesupport_libraries(plugin_dir: str) -> None:
    try:
        import ctypes
        import glob
    except Exception:
        return

    patterns = [
        "libjoint_msgs__rosidl_typesupport_*.so",
        "libros2_plugin_proto__rosidl_typesupport_*.so",
    ]
    for pattern in patterns:
        for path in glob.glob(os.path.join(plugin_dir, pattern)):
            try:
                ctypes.CDLL(path, mode=ctypes.RTLD_GLOBAL)
            except OSError:
                pass


_ensure_runtime_env()

_open_native = _bindings.open
close = _bindings.close
StateInterface = _bindings.StateInterface
CommandInterface = _bindings.CommandInterface


_AIMRT_BUILTIN_CONFIGS = {
    "iceoryx": "aimrt_iceoryx_backend.yaml",
    "ros2": "aimrt_ros2_backend.yaml",
}


def aimrt_config_path(backend: str) -> str:
    key = str(backend).strip().lower()
    if key not in _AIMRT_BUILTIN_CONFIGS:
        supported = ", ".join(sorted(_AIMRT_BUILTIN_CONFIGS.keys()))
        raise ValueError(f"unsupported aimrt backend: {backend!r} (supported: {supported})")

    filename = _AIMRT_BUILTIN_CONFIGS[key]
    plugin_dir = os.path.dirname(getattr(_bindings, "__file__", "") or "")
    if plugin_dir:
        candidate = os.path.join(plugin_dir, "config", filename)
        if os.path.exists(candidate):
            return candidate

    return os.path.join(os.path.dirname(__file__), "config", filename)


def open(
    *args: Any,
    aimrt_backend: str = "iceoryx",
    config_path: str | os.PathLike | None = None,
    **kwargs: Any,
) -> Tuple[StateInterface, CommandInterface]:
    if args and (config_path is not None):
        raise TypeError("config_path specified both positionally and by keyword")

    if args:
        return _open_native(*args, **kwargs)

    if config_path is not None:
        cfg = str(Path(config_path))
        return _open_native(cfg, **kwargs)

    return _open_native(aimrt_config_path(aimrt_backend), **kwargs)


__all__ = [
    "open",
    "close",
    "StateInterface",
    "CommandInterface",
    "OBS",
    "ObsSlices",
    "aimrt_config_path",
]
