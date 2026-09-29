"""Tests for the red/green verdicts of ``scripts/tdd.py``.

The verdict is the whole point of the tool: a "red" that is really an import
error, a skip, or zero collected tests must never be reported as red.
"""

import importlib.util
from pathlib import Path

import pytest

# ``scripts/`` is deliberately not a package: tdd.py has to run as
# ``python scripts/tdd.py`` on hosts that cannot build the workspace. Load it
# by path, so the test does not depend on pytest's ``prepend`` import mode
# putting this directory on ``sys.path``.
_spec = importlib.util.spec_from_file_location(
    "tdd", Path(__file__).with_name("tdd.py")
)
assert _spec is not None and _spec.loader is not None
tdd = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(tdd)

build_pytest_command = tdd.build_pytest_command
judge = tdd.judge
parse_junit = tdd.parse_junit

# --- JUnit fixtures --------------------------------------------------------


def _junit(*cases: str) -> str:
    body = "".join(cases)
    return (
        '<?xml version="1.0" encoding="utf-8"?>'
        f'<testsuites><testsuite name="pytest">{body}</testsuite></testsuites>'
    )


PASSED = '<testcase classname="pkg.test_a" name="test_passes" time="0.01"/>'
FAILED = (
    '<testcase classname="pkg.test_a" name="test_fails" time="0.01">'
    '<failure message="assert 1 == 2">AssertionError</failure></testcase>'
)
ERRORED = (
    '<testcase classname="pkg.test_a" name="test_errors" time="0.01">'
    '<error message="failed on setup with &quot;fixture boom&quot;">E</error>'
    "</testcase>"
)
SKIPPED = (
    '<testcase classname="pkg.test_a" name="test_skips" time="0.0">'
    '<skipped type="pytest.skip" message="no data">skip</skipped></testcase>'
)
XFAILED = (
    '<testcase classname="pkg.test_a" name="test_xfails" time="0.0">'
    '<skipped type="pytest.xfail" message="#123">xfail</skipped></testcase>'
)


# --- parse_junit ------------------------------------------------------------


def test_parse_junit_classifies_every_outcome() -> None:
    outcomes = parse_junit(_junit(PASSED, FAILED, ERRORED, SKIPPED, XFAILED))
    assert outcomes == [
        ("pkg.test_a::test_passes", "passed"),
        ("pkg.test_a::test_fails", "failed"),
        ("pkg.test_a::test_errors", "error"),
        ("pkg.test_a::test_skips", "skipped"),
        ("pkg.test_a::test_xfails", "xfailed"),
    ]


def test_parse_junit_of_nothing_is_empty() -> None:
    assert parse_junit(None) == []
    assert parse_junit(_junit()) == []


# --- red --------------------------------------------------------------------


def test_red_accepts_only_assertion_failures() -> None:
    verdict = judge("red", 1, _junit(FAILED))
    assert verdict.ok, verdict.message


def test_red_shows_why_each_test_failed() -> None:
    """A failure inside the test body can still be the wrong reason (an
    ImportError, say), so the verdict lists every failure message for a
    human to read rather than hiding them behind a count."""
    import_error = (
        '<testcase classname="pkg.test_a" name="test_imports" time="0.01">'
        "<failure message=\"ModuleNotFoundError: No module named 'x'\">E"
        "</failure></testcase>"
    )
    verdict = judge("red", 1, _junit(FAILED, import_error))
    assert verdict.ok
    assert "pkg.test_a::test_fails: assert 1 == 2" in verdict.message
    assert "ModuleNotFoundError: No module named 'x'" in verdict.message


@pytest.mark.parametrize(
    "exit_code, cases, reason",
    [
        (0, (PASSED,), "passes"),
        (1, (FAILED, PASSED), "passes"),
        (1, (ERRORED,), "error"),
        (1, (FAILED, ERRORED), "error"),
        (0, (SKIPPED,), "skipped"),
        (5, (), "no tests"),
        (2, (), "collection"),
        (4, (), "usage"),
        (3, (), "internal"),
    ],
)
def test_red_rejects_everything_that_is_not_a_failed_assertion(
    exit_code: int, cases: tuple, reason: str
) -> None:
    verdict = judge("red", exit_code, _junit(*cases))
    assert not verdict.ok
    assert reason in verdict.message.lower()


# --- green ------------------------------------------------------------------


def test_green_accepts_all_passed() -> None:
    verdict = judge("green", 0, _junit(PASSED))
    assert verdict.ok, verdict.message


@pytest.mark.parametrize(
    "exit_code, cases, reason",
    [
        (1, (FAILED,), "fail"),
        (1, (ERRORED,), "error"),
        (0, (PASSED, SKIPPED), "skipped"),
        (0, (XFAILED,), "xfail"),
        (5, (), "no tests"),
        (2, (), "collection"),
    ],
)
def test_green_rejects_anything_short_of_every_test_passing(
    exit_code: int, cases: tuple, reason: str
) -> None:
    verdict = judge("green", exit_code, _junit(*cases))
    assert not verdict.ok
    assert reason in verdict.message.lower()


def test_green_names_a_strict_xpass() -> None:
    """A strict xfail that now passes fails the run; say so, not just 'failed'."""
    xpass = (
        '<testcase classname="pkg.test_a" name="test_fixed" time="0.01">'
        '<failure message="[XPASS(strict)] #123">E</failure></testcase>'
    )
    verdict = judge("green", 1, _junit(xpass))
    assert not verdict.ok
    assert "pkg.test_a::test_fixed: [XPASS(strict)] #123" in verdict.message


# --- the pytest command ------------------------------------------------------


def test_command_neutralises_project_addopts() -> None:
    cmd = build_pytest_command("green", ["t.py::test_x"], "/tmp/j.xml")
    assert cmd[:3] == ["pytest", "-o", "addopts="]
    assert "no:testmon" in cmd
    assert "--junitxml=/tmp/j.xml" in cmd
    assert cmd[-1] == "t.py::test_x"


def test_red_ignores_xfail_markers_but_green_honours_them() -> None:
    assert "--runxfail" in build_pytest_command("red", ["t.py"], "/tmp/j.xml")
    assert "--runxfail" not in build_pytest_command("green", ["t.py"], "/tmp/j.xml")
