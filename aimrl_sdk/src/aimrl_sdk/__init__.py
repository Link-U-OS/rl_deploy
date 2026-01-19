from . import _bindings as _bindings

import os


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

open = _bindings.open
close = _bindings.close
StateInterface = _bindings.StateInterface
CommandInterface = _bindings.CommandInterface

__all__ = ["open", "close", "StateInterface", "CommandInterface"]
