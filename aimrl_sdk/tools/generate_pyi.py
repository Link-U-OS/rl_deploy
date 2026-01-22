#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _src_pkg_dir(repo_root: Path) -> Path:
    return repo_root / "src" / "aimrl_sdk"


def _run_stubgen(module: str, out_dir: Path) -> None:
    cmd = [sys.executable, "-m", "pybind11_stubgen", module, "-o", str(out_dir)]
    subprocess.run(cmd, check=True)


def main() -> int:
    p = argparse.ArgumentParser(description="Generate .pyi stubs for aimrl_sdk pybind11 bindings.")
    p.add_argument(
        "--module",
        type=str,
        default="aimrl_sdk._bindings",
        help="Target module to stub (default: aimrl_sdk._bindings)",
    )
    p.add_argument(
        "--write-to-src",
        action="store_true",
        help="Write the generated .pyi into aimrl_sdk/src/aimrl_sdk/ (recommended for committing)",
    )
    args = p.parse_args()

    repo_root = _repo_root()
    src_pkg_dir = _src_pkg_dir(repo_root)
    if not src_pkg_dir.is_dir():
        raise RuntimeError(f"unexpected layout: missing {src_pkg_dir}")

    with tempfile.TemporaryDirectory(prefix="aimrl_sdk_stubgen_") as td:
        out_dir = Path(td)
        _run_stubgen(args.module, out_dir)

        rel_parts = args.module.split(".")
        if len(rel_parts) < 2:
            raise RuntimeError(f"expected a package.module target, got {args.module!r}")
        pkg_name, mod_name = rel_parts[-2], rel_parts[-1]
        generated = out_dir / pkg_name / f"{mod_name}.pyi"
        if not generated.is_file():
            raise RuntimeError(f"stubgen did not produce expected file: {generated}")

        if args.write_to_src:
            dest = src_pkg_dir / f"{mod_name}.pyi"
            os.makedirs(dest.parent, exist_ok=True)
            shutil.copyfile(generated, dest)
            print(f"Wrote {dest}")
        else:
            print(generated.read_text(encoding="utf-8"), end="")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
