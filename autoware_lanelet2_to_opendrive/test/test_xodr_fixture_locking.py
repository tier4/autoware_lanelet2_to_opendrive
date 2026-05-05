"""Regression tests for the cross-worker locking used by the
``_ensure_nishishinjuku_xodr`` session fixture in
``autoware_carla_scenario/test/conftest.py``.

The fixture was racy under ``pytest-xdist`` (issue #462): two workers
could both pass the ``_XODR_PATH.exists()`` check and either run the
convert subprocess concurrently or read a partially-written file. The
fix uses :class:`filelock.FileLock` plus a stage-then-:func:`os.replace`
write. These tests exercise that pattern directly so the regression
cannot silently come back.
"""

from __future__ import annotations

import multiprocessing as mp
import os
from multiprocessing.synchronize import Barrier as MPBarrier
from pathlib import Path
from typing import Optional

from filelock import FileLock


_PAYLOAD = "x" * 100_000

# Generous upper bound on per-worker wall time. The bodies are tiny and
# usually complete in well under a second; the budget exists only so a
# hung worker fails fast instead of hanging the whole pytest invocation.
_JOIN_TIMEOUT_S = 30.0


def _ensure_file(target: Path, lock_path: Path, marker: Path) -> None:
    """Mirror of the conftest pattern: lock, double-check, atomic rename.

    Records the worker pid in ``marker`` only when this worker actually
    performs the write, so the test can assert generation happened
    exactly once across concurrent processes.
    """
    if target.exists():
        return
    with FileLock(str(lock_path)):
        if target.exists():
            return
        with marker.open("a") as f:
            f.write(f"{os.getpid()}\n")
        tmp = target.with_name(f"{target.name}.tmp.{os.getpid()}")
        try:
            tmp.write_text(_PAYLOAD)
            os.replace(tmp, target)
        finally:
            if tmp.exists():
                try:
                    tmp.unlink()
                except OSError:
                    pass


def _ensure_file_no_lock(
    target: Path, marker: Path, barrier: Optional[MPBarrier]
) -> None:
    """Buggy variant kept only for the negative test below.

    The optional barrier lets the test force every worker past the
    ``target.exists()`` check before any of them writes, making the
    race deterministic instead of timing-dependent.
    """
    if target.exists():
        return
    if barrier is not None:
        barrier.wait()
    with marker.open("a") as f:
        f.write(f"{os.getpid()}\n")
    target.write_text(_PAYLOAD)


def _join_or_kill(p: "mp.Process", timeout: float = _JOIN_TIMEOUT_S) -> None:
    """Join *p* within *timeout*; otherwise terminate/kill so we never leak.

    A bare ``p.join(timeout=...)`` followed by an ``exitcode`` assertion
    leaves the child running on timeout — orphaning it would let it
    contend with the next test. Escalate SIGTERM → SIGKILL and raise
    ``AssertionError`` so the failure is loud and the child is gone.
    """
    p.join(timeout=timeout)
    if not p.is_alive():
        return
    p.terminate()
    p.join(timeout=5)
    if p.is_alive():
        p.kill()
        p.join()
    raise AssertionError(
        f"worker pid={p.pid} did not exit within {timeout:.0f}s; killed"
    )


def test_lock_serialises_concurrent_workers(tmp_path: Path) -> None:
    """Eight concurrent workers — only one performs the write."""
    target = tmp_path / "out.txt"
    lock_path = tmp_path / "out.txt.lock"
    marker = tmp_path / "marker.log"

    ctx = mp.get_context("spawn")
    procs = [
        ctx.Process(target=_ensure_file, args=(target, lock_path, marker))
        for _ in range(8)
    ]
    for p in procs:
        p.start()
    try:
        for p in procs:
            _join_or_kill(p)
            assert p.exitcode == 0, f"worker pid={p.pid} exited with {p.exitcode}"
    finally:
        for p in procs:
            if p.is_alive():
                p.kill()
                p.join()

    # Exactly one worker should have produced the file.
    assert marker.read_text().count("\n") == 1
    # And the file should be fully written (no partial-content surface).
    assert target.read_text() == _PAYLOAD


def test_unlocked_variant_races_deterministically(tmp_path: Path) -> None:
    """Without the lock, every worker writes after the existence check.

    A :class:`multiprocessing.Barrier` synchronises the workers
    immediately after they pass the ``target.exists()`` check, so the
    race is forced to fire on every run. ``writers == n_workers``
    exactly demonstrates the failure mode the lock prevents (compare
    with :func:`test_lock_serialises_concurrent_workers`).
    """
    target = tmp_path / "out.txt"
    marker = tmp_path / "marker.log"

    n_workers = 8
    ctx = mp.get_context("spawn")
    barrier = ctx.Barrier(n_workers)
    procs = [
        ctx.Process(target=_ensure_file_no_lock, args=(target, marker, barrier))
        for _ in range(n_workers)
    ]
    for p in procs:
        p.start()
    try:
        for p in procs:
            _join_or_kill(p)
            assert p.exitcode == 0, f"worker pid={p.pid} exited with {p.exitcode}"
    finally:
        for p in procs:
            if p.is_alive():
                p.kill()
                p.join()

    writers = marker.read_text().count("\n")
    assert writers == n_workers
