"""The post-conversion analysis result must not be discarded.

`convert` ends by calling `run_analysis`, which prints its own
`FINAL RESULT: PASS` / `FINAL RESULT: FAIL  (n error(s) found)` banner and
returns the error count. The convert path threw the return value away, so a
conversion whose own analysis had just printed FAIL still exited 0 and no log
line recorded the count. The `analyze` CLI, given the same result, exits 1.

The count is 0 on both shipped maps today, so this is a gate that has not yet
had to fire -- which is exactly why nothing noticed it was not wired.
"""

import logging

import pytest


def _act(error_count: int, strict: bool) -> None:
    from autoware_lanelet2_to_opendrive.main import _fail_on_analysis_errors

    _fail_on_analysis_errors(error_count, strict=strict)


def test_a_clean_analysis_says_nothing() -> None:
    """Zero errors must not produce a warning or an exception."""
    _act(0, strict=False)
    _act(0, strict=True)


@pytest.mark.xfail(strict=True, reason="convert discards the analysis result")
def test_analysis_errors_are_reported_when_not_strict(
    caplog: pytest.LogCaptureFixture,
) -> None:
    """Non-strict keeps the output, but the count has to reach the log."""
    with caplog.at_level(logging.WARNING, logger="autoware_lanelet2_to_opendrive.main"):
        _act(3, strict=False)

    messages = [record.getMessage() for record in caplog.records]
    assert messages, "the analysis reported 3 errors and nothing was logged"
    assert any("3" in message for message in messages), (
        "the warning must name the error count, so a log reader can tell"
        f" a clean run from a failing one: {messages}"
    )


@pytest.mark.xfail(strict=True, reason="convert discards the analysis result")
def test_analysis_errors_abort_the_run_when_strict() -> None:
    """Strict must fail the process, as `analyze` already does."""
    with pytest.raises(RuntimeError, match="3"):
        _act(3, strict=True)
