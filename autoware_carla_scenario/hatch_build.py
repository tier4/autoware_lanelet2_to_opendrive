"""Hatchling build hook: generate the vendored protobuf stubs before building.

The ``*/_proto`` packages are generated, not committed (see
``scripts/compile_protos.py`` and ``proto/README.md``). Running the generator
here means a fresh ``uv sync`` -- which builds the editable install -- produces
the stubs into the source tree, and every wheel/sdist ships them, without anyone
having to run the generator by hand or keep the output under version control.

``grpcio-tools`` is a build requirement (see ``[build-system].requires``) so it is
available in the isolated build environment this hook runs in; the installed
runtime still never needs it.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from hatchling.builders.hooks.plugin.interface import BuildHookInterface


class CustomBuildHook(BuildHookInterface):
    """Regenerate the protobuf stubs at the start of every build."""

    PLUGIN_NAME = "custom"

    def initialize(self, version: str, build_data: dict) -> None:
        del version, build_data
        script = Path(self.root) / "scripts" / "compile_protos.py"
        subprocess.run([sys.executable, str(script)], check=True)
