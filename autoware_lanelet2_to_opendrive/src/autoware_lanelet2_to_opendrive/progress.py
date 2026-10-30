"""Progress reporting for long conversion phases.

Two modules walk thousands of roads and lanelets and want to say how far they
have got. They did that by importing ``tqdm`` and calling it directly, which
had three consequences:

* the bar was written to stderr unconditionally, so a non-interactive run --
  CI, a redirected log, a pytest capture -- collected carriage-return noise;
* diagnostics had to go through :func:`tqdm.write` to avoid being garbled by
  the bar, which put them outside ``logging``: unroutable, unfilterable, and
  invisible to a test;
* the two modules could not be exercised without a progress bar existing.

This module keeps the bar in one place. Progress goes through a reporter;
diagnostics go through ``logging`` as everywhere else in the package, and
:func:`progress_aware_logging` keeps the two from overwriting each other.

The default reporter follows ``tqdm``'s own convention for ``disable=None``:
show the bar when stderr is a terminal, stay silent otherwise.
"""

from __future__ import annotations

import contextlib
import logging
from typing import Iterable, Iterator, Optional, Protocol, TypeVar

from tqdm import tqdm
from tqdm.contrib.logging import logging_redirect_tqdm

T = TypeVar("T")

logger = logging.getLogger(__name__)


class ProgressHandle(Protocol):
    """A bar whose total is not known in advance, advanced by hand."""

    def advance(self, n: int = 1) -> None:
        """Move the bar on by ``n`` steps."""

    def close(self) -> None:
        """Finish the bar."""


class ProgressReporter(Protocol):
    """How a conversion phase reports its progress."""

    def track(
        self,
        iterable: Iterable[T],
        description: str,
        unit: str = "it",
        total: Optional[int] = None,
    ) -> Iterable[T]:
        """Yield from ``iterable``, reporting progress as it goes."""

    def manual(self, description: str, unit: str = "it") -> ProgressHandle:
        """Open a bar for a loop whose length is not known in advance."""


class _TqdmHandle:
    def __init__(self, bar: tqdm) -> None:
        self._bar = bar

    def advance(self, n: int = 1) -> None:
        self._bar.update(n)

    def close(self) -> None:
        self._bar.close()


class TqdmProgressReporter:
    """Reports with a ``tqdm`` bar, silent when stderr is not a terminal.

    ``disable=None`` is ``tqdm``'s own "only when interactive" setting. It is
    what makes a redirected conversion log readable.
    """

    def track(
        self,
        iterable: Iterable[T],
        description: str,
        unit: str = "it",
        total: Optional[int] = None,
    ) -> Iterable[T]:
        return tqdm(
            iterable,
            desc=description,
            unit=unit,
            total=total,
            disable=None,
        )

    def manual(self, description: str, unit: str = "it") -> ProgressHandle:
        return _TqdmHandle(tqdm(desc=description, unit=unit, disable=None))


class _NullHandle:
    def advance(self, n: int = 1) -> None:
        return None

    def close(self) -> None:
        return None


class NullProgressReporter:
    """Reports nothing. For tests, and for callers that want no output."""

    def track(
        self,
        iterable: Iterable[T],
        description: str,
        unit: str = "it",
        total: Optional[int] = None,
    ) -> Iterable[T]:
        return iterable

    def manual(self, description: str, unit: str = "it") -> ProgressHandle:
        return _NullHandle()


_reporter: ProgressReporter = TqdmProgressReporter()


def get_progress_reporter() -> ProgressReporter:
    """Return the reporter the conversion phases should use."""
    return _reporter


def set_progress_reporter(reporter: ProgressReporter) -> ProgressReporter:
    """Install ``reporter`` and return the one it replaced."""
    global _reporter
    previous = _reporter
    _reporter = reporter
    return previous


def track(
    iterable: Iterable[T],
    description: str,
    unit: str = "it",
    total: Optional[int] = None,
) -> Iterable[T]:
    """Report progress over ``iterable`` with the installed reporter."""
    return _reporter.track(iterable, description, unit=unit, total=total)


def manual(description: str, unit: str = "it") -> ProgressHandle:
    """Open a hand-advanced bar with the installed reporter."""
    return _reporter.manual(description, unit=unit)


@contextlib.contextmanager
def progress_aware_logging() -> Iterator[None]:
    """Keep log records and an active progress bar off each other's lines.

    ``logging`` writes to stderr and so does the bar, so a record emitted
    while a bar is on screen lands in the middle of it. Inside this context
    ``tqdm`` takes the record, clears its own line, prints it and redraws.
    """
    with logging_redirect_tqdm():
        yield
