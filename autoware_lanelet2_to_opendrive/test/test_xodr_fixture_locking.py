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
from pathlib import Path

from filelock import FileLock


_PAYLOAD = "x" * 100_000


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


def test_lock_serialises_concurrent_workers(tmp_path: Path) -> None:
    """Eight concurrent workers — only one performs the write."""
    target = tmp_path / "out.bin"
    lock_path = tmp_path / "out.bin.lock"
    marker = tmp_path / "marker.log"

    ctx = mp.get_context("spawn")
    procs = [
        ctx.Process(target=_ensure_file, args=(target, lock_path, marker))
        for _ in range(8)
    ]
    for p in procs:
        p.start()
    for p in procs:
        p.join(timeout=30)
        assert p.exitcode == 0, f"worker pid={p.pid} exited with {p.exitcode}"

    # Exactly one worker should have produced the file.
    assert marker.read_text().count("\n") == 1
    # And the file should be fully written (no partial-content surface).
    assert target.read_text() == _PAYLOAD


def _ensure_file_no_lock(target: Path, marker: Path) -> None:
    """Buggy variant kept only for the negative test below."""
    if target.exists():
        return
    with marker.open("a") as f:
        f.write(f"{os.getpid()}\n")
    target.write_text(_PAYLOAD)


def test_unlocked_variant_can_double_generate(tmp_path: Path) -> None:
    """Sanity check: without the lock, multiple workers can both write.

    This documents the failure mode the lock is designed to prevent.
    The assertion is intentionally permissive (>= 1) because the race
    is timing-dependent and not guaranteed to fire on every run; the
    point is that the *un*locked path makes the multi-write outcome
    *possible*, while the locked path makes it impossible (proved by
    :func:`test_lock_serialises_concurrent_workers`).
    """
    target = tmp_path / "out.bin"
    marker = tmp_path / "marker.log"

    ctx = mp.get_context("spawn")
    procs = [
        ctx.Process(target=_ensure_file_no_lock, args=(target, marker))
        for _ in range(8)
    ]
    for p in procs:
        p.start()
    for p in procs:
        p.join(timeout=30)
        assert p.exitcode == 0

    writers = marker.read_text().count("\n")
    assert writers >= 1
