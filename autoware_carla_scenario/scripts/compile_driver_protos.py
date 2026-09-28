#!/usr/bin/env python
"""Compile ``proto/`` into ``driver_policy/_proto/``.

The ``.proto`` files under ``proto/`` are verbatim copies of the egodriver wire
contract (see ``proto/README.md`` for where each one comes from). They are
vendored rather than taken from the ``alpasim-grpc`` package because that
package requires Python >= 3.11 and this workspace is pinned to 3.10.

The generated modules are committed, so installing this package needs neither
``grpcio-tools`` nor network access. Re-run this script after updating a
vendored ``.proto``::

    uvx --python 3.10 --from grpcio-tools==1.62.3 \\
        python autoware_carla_scenario/scripts/compile_driver_protos.py

``grpcio-tools`` 1.62.3 is the version ``carla_driver_interface`` generates
with. It bundles protoc for protobuf 4.25, which is what ``alpasim-grpc``
pins (``protobuf<5``).

protoc emits absolute imports (``from alpasim_grpc.v0 import common_pb2``).
Those are rewritten to point into this package, so the generated code never
claims the top-level ``alpasim_grpc`` name.
"""

from __future__ import annotations

import re
import shutil
import subprocess
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROTO_ROOT = PACKAGE_ROOT / "proto"
OUT_ROOT = PACKAGE_ROOT / "src" / "autoware_carla_scenario" / "driver_policy" / "_proto"
IMPORT_PREFIX = "autoware_carla_scenario.driver_policy._proto"
PROTO_FILES = (
    "alpasim_grpc/v0/common.proto",
    "alpasim_grpc/v0/sensorsim.proto",
    "alpasim_grpc/v0/egodriver.proto",
    "carla_driver/v0/carla_driver.proto",
)
GENERATED_PACKAGES = ("alpasim_grpc", "carla_driver")

_ABSOLUTE_IMPORT = re.compile(
    r"^from ((?:alpasim_grpc|carla_driver)\.v0) import ", re.M
)
_MODULE_NAME = re.compile(r"'((?:alpasim_grpc|carla_driver)\.v0\.\w+_pb2)'")


def _rewrite_imports(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    text = _ABSOLUTE_IMPORT.sub(rf"from {IMPORT_PREFIX}.\1 import ", text)
    text = _MODULE_NAME.sub(rf"'{IMPORT_PREFIX}.\1'", text)
    path.write_text(text, encoding="utf-8")


def compile_protos(out_root: Path | None = None) -> Path:
    """Generate the protobuf modules. Returns the output root."""
    out_root = Path(out_root) if out_root is not None else OUT_ROOT
    for name in GENERATED_PACKAGES:
        if (out_root / name).exists():
            shutil.rmtree(out_root / name)
    out_root.mkdir(parents=True, exist_ok=True)

    cmd = [
        sys.executable,
        "-m",
        "grpc_tools.protoc",
        f"-I{PROTO_ROOT}",
        f"--python_out={out_root}",
        f"--grpc_python_out={out_root}",
        f"--pyi_out={out_root}",
        *(str(PROTO_ROOT / f) for f in PROTO_FILES),
    ]
    subprocess.run(cmd, check=True, cwd=PACKAGE_ROOT)

    for generated in out_root.rglob("*.py*"):
        if generated.suffix in (".py", ".pyi"):
            _rewrite_imports(generated)

    # protoc does not emit package markers for the intermediate directories.
    marker = "# Generated package marker; see scripts/compile_driver_protos.py\n"
    (out_root / "__init__.py").write_text(marker)
    for name in GENERATED_PACKAGES:
        for pkg_dir in (out_root / name, out_root / name / "v0"):
            (pkg_dir / "__init__.py").write_text(marker)
    return out_root


def main() -> None:
    out = compile_protos()
    print(f"Generated protobuf modules under {out}")


if __name__ == "__main__":
    main()
