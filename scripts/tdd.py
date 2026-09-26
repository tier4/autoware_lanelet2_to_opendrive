#!/usr/bin/env python3
"""Red/green test runner for test-driven development, executed in the dev container.

Usage::

    python scripts/tdd.py red   <pytest args, e.g. path/test_x.py::test_y>
    python scripts/tdd.py green <pytest args>
    python scripts/tdd.py run   <pytest args>     # plain run, no verdict
    python scripts/tdd.py stop                    # stop the resident container

``red`` succeeds only when every selected test *ran and failed*. Each of these
is reported as "not red" instead, because none of them shows that the test can
detect the missing behaviour:

- the test passes already, so it does not constrain the change;
- it errors in setup or teardown, or is never collected (an import error);
- it is skipped, or nothing was selected at all.

``green`` succeeds only when every selected test *passed*: a skip or a leftover
``xfail`` marker is not green.

``red`` passes ``--runxfail``, so a red test committed under
``@pytest.mark.xfail(strict=True)`` is still checked as failing.

Both clear the project's ``addopts`` (``-n auto --testmon``): testmon silently
deselects tests it considers unaffected, and a single test does not need a
pool of xdist workers. Pass ``-n N`` explicitly to parallelise.

The tests run inside a resident ``tdd`` compose service, so each invocation is
a ``docker compose exec`` rather than a fresh container. Uses only the
standard library, so it runs with any host Python 3.10+.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import NamedTuple, Optional, Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
SERVICE = "tdd"
PROFILE = "tdd"

# pytest exit codes (``pytest.ExitCode``), restated so this file needs no pytest.
EXIT_OK = 0
EXIT_TESTS_FAILED = 1
EXIT_INTERRUPTED = 2
EXIT_INTERNAL_ERROR = 3
EXIT_USAGE_ERROR = 4
EXIT_NO_TESTS_COLLECTED = 5

_EXIT_EXPLANATION = {
    EXIT_INTERRUPTED: "interrupted -- usually a collection error such as an import "
    "failure, so the test never ran",
    EXIT_INTERNAL_ERROR: "internal pytest error",
    EXIT_USAGE_ERROR: "usage error -- check the pytest arguments or the node id",
    EXIT_NO_TESTS_COLLECTED: "no tests were collected -- check the node id",
}


class Verdict(NamedTuple):
    ok: bool
    message: str


def parse_junit(xml_text: Optional[str]) -> list[tuple[str, str]]:
    """Return ``(test id, outcome)`` for every test case in a JUnit XML report.

    The outcome is one of ``passed``, ``failed``, ``error``, ``skipped`` or
    ``xfailed``. The test id is ``classname::name``, which is close to, but not
    exactly, the pytest node id.
    """
    if not xml_text:
        return []
    outcomes = []
    for case in ET.fromstring(xml_text).iter("testcase"):
        test_id = f"{case.get('classname', '')}::{case.get('name', '')}"
        if case.find("error") is not None:
            outcome = "error"
        elif case.find("failure") is not None:
            outcome = "failed"
        elif (skipped := case.find("skipped")) is not None:
            outcome = "xfailed" if skipped.get("type") == "pytest.xfail" else "skipped"
        else:
            outcome = "passed"
        outcomes.append((test_id, outcome))
    return outcomes


def _failure_messages(xml_text: str) -> list[str]:
    """``test id: message`` for every failed test case, in report order."""
    lines = []
    for case in ET.fromstring(xml_text).iter("testcase"):
        failure = case.find("failure")
        if failure is not None:
            test_id = f"{case.get('classname', '')}::{case.get('name', '')}"
            message = (failure.get("message") or "").splitlines()
            lines.append(f"{test_id}: {message[0] if message else '(no message)'}")
    return lines


def _list(outcomes: list[tuple[str, str]], wanted: str) -> list[str]:
    return [test_id for test_id, outcome in outcomes if outcome == wanted]


def _bullets(test_ids: list[str]) -> str:
    return "".join(f"\n  - {test_id}" for test_id in test_ids)


def judge(mode: str, exit_code: int, junit_xml: Optional[str]) -> Verdict:
    """Decide whether a pytest run is a genuine red (or green)."""
    if exit_code in _EXIT_EXPLANATION:
        return Verdict(
            False,
            f"NOT {mode.upper()}: {_EXIT_EXPLANATION[exit_code]} (exit {exit_code})",
        )

    outcomes = parse_junit(junit_xml)
    if not outcomes:
        return Verdict(False, f"NOT {mode.upper()}: no tests ran")

    errored = _list(outcomes, "error")
    if errored:
        return Verdict(
            False,
            f"NOT {mode.upper()}: error outside the test body (setup/teardown), "
            f"so the assertion was never reached:{_bullets(errored)}",
        )

    skipped = _list(outcomes, "skipped")
    if skipped:
        return Verdict(False, f"NOT {mode.upper()}: skipped:{_bullets(skipped)}")

    if mode == "red":
        passed = _list(outcomes, "passed")
        if passed:
            return Verdict(
                False,
                "NOT RED: passes already, so it does not constrain the change:"
                f"{_bullets(passed)}",
            )
        # A failure is not necessarily the right failure: an ImportError in the
        # test body fails too. Show every reason rather than only a count.
        assert junit_xml is not None
        return Verdict(
            True,
            f"RED: {len(outcomes)} test(s) ran and failed -- check each reason is "
            f"the missing behaviour:{_bullets(_failure_messages(junit_xml))}",
        )

    xfailed = _list(outcomes, "xfailed")
    if xfailed:
        return Verdict(
            False,
            f"NOT GREEN: still marked xfail -- remove the marker:{_bullets(xfailed)}",
        )
    if _list(outcomes, "failed"):
        assert junit_xml is not None
        return Verdict(
            False, f"NOT GREEN: failed:{_bullets(_failure_messages(junit_xml))}"
        )
    return Verdict(True, f"GREEN: {len(outcomes)} test(s) passed")


def build_pytest_command(mode: str, args: Sequence[str], junit_path: str) -> list[str]:
    """The pytest invocation run inside the container."""
    cmd = ["pytest", "-o", "addopts=", "-p", "no:testmon", f"--junitxml={junit_path}"]
    if mode == "red":
        cmd.append("--runxfail")
    return cmd + list(args)


# --- container plumbing -------------------------------------------------------


def _compose(*args: str) -> list[str]:
    docker = shutil.which("docker")
    if docker is None:
        sys.exit("tdd.py: docker not found on PATH")
    return [
        docker,
        "compose",
        "--project-directory",
        str(REPO_ROOT),
        "-f",
        str(REPO_ROOT / "docker-compose.yml"),
        "--profile",
        PROFILE,
        *args,
    ]


def _ensure_service() -> None:
    # Idempotent: a no-op when the service is already running.
    subprocess.run(_compose("up", "-d", "--wait", SERVICE), check=True)


def _exec(cmd: Sequence[str], **kwargs) -> subprocess.CompletedProcess:
    return subprocess.run(_compose("exec", "-T", SERVICE, *cmd), **kwargs)


def main(argv: Sequence[str]) -> int:
    if not argv or argv[0] not in {"red", "green", "run", "stop"}:
        print(__doc__, file=sys.stderr)
        return 2
    mode, args = argv[0], list(argv[1:])

    if mode == "stop":
        return subprocess.run(_compose("stop", SERVICE)).returncode

    _ensure_service()
    junit_path = f"/tmp/tdd-junit-{os.getpid()}.xml"
    color = ["--color=yes"] if sys.stdout.isatty() else []
    exit_code = _exec(build_pytest_command(mode, color + args, junit_path)).returncode
    if mode == "run":
        return exit_code

    report = _exec(["cat", junit_path], capture_output=True, text=True)
    _exec(["rm", "-f", junit_path])
    verdict = judge(mode, exit_code, report.stdout if report.returncode == 0 else None)
    print(f"\n{verdict.message}")
    return 0 if verdict.ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
