#!/usr/bin/env python3
"""Refuse to run tests in a container whose ``.venv`` no longer matches ``uv.lock``.

The container's ``/workspace/.venv`` lives in a named volume (``venv-cache``
or ``venv-nocarla-cache``), not in the image. The volume is filled from the
image once, when it is first created, and survives every later image
rebuild. So when ``uv.lock`` moves -- a merge from master, a dependency
bump -- the tests keep running against the old dependencies, and nothing
says so.

This asks ``uv`` itself, with the same arguments the image was synced with
(``L2O_SYNC_ARGS``, set in the Dockerfile), whether every locked dependency
is installed at its locked version::

    python3 scripts/check_container_env.py                 # report only
    python3 scripts/check_container_env.py -- pytest -n 4  # check, then exec

Exit 1 means stale; uv's own list of what differs is printed with both ways
to fix it. Any other uv failure is reported but does not block, because it
is not evidence that the venv is stale. Set ``L2O_ALLOW_STALE_VENV=1`` to
skip the check, which the in-place fix itself needs.

Standard library only, and meant to run under the system ``/usr/bin/python3``:
a venv from the wrong architecture cannot start its own interpreter.
"""

from __future__ import annotations

import os
import shlex
import subprocess
import sys
from typing import NamedTuple, Sequence

# uv 0.9.7's wording for "the environment differs from the lock".
_OUTDATED = "The environment is outdated"


class Verdict(NamedTuple):
    ok: bool
    message: str


def build_check_command(sync_args: str) -> list[str]:
    """The ``uv sync`` the image ran, as a check that changes nothing.

    ``--no-install-workspace``: with the source bind-mounted over the copy
    the image was synced from, uv reports the workspace's own package as out
    of date even in a venv freshly filled from the image, so without it every
    venv would be called stale.
    ``--inexact``: a package the lock no longer names is left over, not
    missing; only a missing or wrong-version dependency changes what the
    tests run against.
    """
    return [
        "uv",
        "sync",
        "--frozen",
        "--check",
        "--no-install-workspace",
        "--inexact",
        *shlex.split(sync_args),
    ]


def explain(returncode: int, output: str, sync_args: str, volume: str) -> Verdict:
    """Turn the result of :func:`build_check_command` into a verdict."""
    if returncode == 0:
        return Verdict(True, "")
    if _OUTDATED not in output:
        return Verdict(
            True,
            "check_container_env: could not check the venv against uv.lock "
            f"(uv exit {returncode}); running anyway:\n{output}",
        )
    return Verdict(
        False,
        f"check_container_env: the container's .venv (volume `{volume}`) no "
        "longer matches uv.lock, so the tests would run against other "
        f"dependencies than the lock names:\n{output}\n"
        "Bring it up to date, either in place:\n"
        "  docker compose run --rm -e L2O_ALLOW_STALE_VENV=1 <service> "
        f"uv sync --frozen {sync_args}\n"
        "or from a rebuilt image:\n"
        f'  docker compose build && docker volume rm "$(basename "$PWD")_{volume}"\n'
        "See docs/docker.md, Troubleshooting.",
    )


def main(argv: Sequence[str]) -> int:
    command = list(argv[1:] if argv[:1] == ["--"] else argv)
    sync_args = os.environ.get("L2O_SYNC_ARGS")
    if sync_args is None:
        # Checking without the image's extras would call a synchronised venv
        # stale, so an image built before this check is not judged at all.
        print(
            "check_container_env: this image does not record how its venv was "
            "synced (no L2O_SYNC_ARGS); rebuild the image to enable the check",
            file=sys.stderr,
        )
    elif not os.environ.get("L2O_ALLOW_STALE_VENV"):
        result = subprocess.run(
            build_check_command(sync_args),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        verdict = explain(
            result.returncode,
            result.stdout,
            sync_args,
            os.environ.get("L2O_VENV_VOLUME", "venv-cache"),
        )
        if verdict.message:
            print(verdict.message, file=sys.stderr)
        if not verdict.ok:
            return 1
    if command:
        os.execvp(command[0], command)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
